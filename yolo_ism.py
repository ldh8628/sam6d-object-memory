#!/usr/bin/env python3
"""yolo_ism.py — YOLO-World proposals × SAM-6D ISM DINOv2 matching (MVP, step A).

Reproduces SAM-6D's *ISM* (Instance Segmentation Model) object-recognition
matching on top of YOLO-World bbox proposals. Per RGB frame:

  1. YOLO-World generates up to --top-k "milk carton" bbox proposals.
  2. Each proposal crop -> DINOv2 *cls* token -> cosine vs pre-extracted Milk
     template cls features (top-k mean) = SEMANTIC score. The highest-semantic
     proposal passing --similarity-threshold is selected as the Milk proposal.
  3. (STEP A) For the SELECTED Milk proposal, DINOv2 *patch* tokens are
     extracted and compared to the Milk template's masked patch features
     (appearance descriptors) = APPEARANCE (appe) score.

This mirrors SAM-6D ISM's two DINOv2 paths:
  * semantic  : token `x_norm_clstoken`     vs  descriptors.pth
  * appearance: token `x_norm_patchtokens`  vs  descriptors_appe.pth

SCOPE / what is deferred
  * Mask segmentation is NOT yet implemented — proposals are YOLO *bboxes*, not
    SAM masks. Consequently the appearance score is a 1st-step approximation:
    template patches are masked (foreground only, via mask_*.png) but the query
    proposal has no mask, so ALL crop patches are used. Appearance is anchored on
    template foreground patches (each fg patch -> best-matching query patch).
  * No PEM / pose, no ISM geometric (depth) score.
  * SAM-6D source is NOT modified — only its vendored DINOv2 ViT is imported.
"""

import argparse
import csv
import os
import sys
import glob
import statistics

import numpy as np
import torch
import torch.nn.functional as F
import cv2
from PIL import Image

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
ISM_DIR = os.path.join(
    REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"
)
DEFAULT_DINOV2_CKPT = os.path.join(
    ISM_DIR, "checkpoints", "dinov2", "dinov2_vits14_pretrain.pth"
)
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
PROPOSAL_SIZE = 224          # SAM-6D dinov2.yaml image_size
PATCH = 14                   # ViT-S/14 patch size
VALID_PATCH_THRESH = 0.5     # SAM-6D validpatch_thresh


# ---------------------------------------------------------------------------
# DINOv2 backbone (SAM-6D's exact ViT-S/14), loaded standalone (read-only import)
# ---------------------------------------------------------------------------
def build_dinov2(checkpoint_path, device):
    if ISM_DIR not in sys.path:
        sys.path.insert(0, ISM_DIR)
    import model.vision_transformer as vits  # noqa: E402 (SAM-6D vendored DINOv2)

    model = vits.vit_small(
        patch_size=14, img_size=518, init_values=1.0, block_chunks=0,
        ffn_layer="mlp", num_register_tokens=0,
    )
    model.load_state_dict(torch.load(checkpoint_path, map_location="cpu"), strict=True)
    model.eval().to(device)
    return model


# ---------------------------------------------------------------------------
# Preprocessing — faithful inline replica of SAM-6D CropResizePad(224)
# Works on either a 3-channel rgb tensor or a 1-channel mask tensor.
# ---------------------------------------------------------------------------
def crop_resize_pad(image_chw, box_xyxy, target=PROPOSAL_SIZE, mode="bilinear"):
    x1, y1, x2, y2 = [int(v) for v in box_xyxy]
    crop = image_chw[:, y1:y2, x1:x2]
    h, w = crop.shape[1], crop.shape[2]
    if h < 1 or w < 1:
        return None
    scale = target / max(h, w)
    crop = F.interpolate(crop.unsqueeze(0), scale_factor=scale, mode=mode,
                         align_corners=False if mode == "bilinear" else None,
                         recompute_scale_factor=True)[0]
    oh, ow = crop.shape[1], crop.shape[2]
    pt = max((target - oh) // 2, 0); pb = max(target - oh - pt, 0)
    pl = max((target - ow) // 2, 0); pr = max(target - ow - pl, 0)
    crop = F.pad(crop, (pl, pr, pt, pb))
    if crop.shape[1] != target or crop.shape[2] != target:
        crop = F.interpolate(crop.unsqueeze(0), size=(target, target),
                             mode=mode, align_corners=False if mode == "bilinear" else None)[0]
    return crop


def normalize_rgb(rgb_uint8):
    t = torch.from_numpy(rgb_uint8).float().permute(2, 0, 1) / 255.0
    mean = torch.tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = torch.tensor(IMAGENET_STD).view(3, 1, 1)
    return (t - mean) / std


@torch.no_grad()
def dinov2_forward(model, crop_chw, device, want_patch=False,
                   patch_stays_on_device=False):
    """Return (cls[384], patch[256,384] or None) for one preprocessed crop.

    patch_stays_on_device=True keeps the normalized patch tokens on the GPU so
    a caller racing several proposals can transfer only the winner once
    (values identical either way; only the copy moment changes).
    """
    out = model(crop_chw.unsqueeze(0).to(device), is_training=True)
    cls = F.normalize(out["x_norm_clstoken"], dim=-1)[0].cpu()
    patch = None
    if want_patch:
        patch = F.normalize(out["x_norm_patchtokens"], dim=-1)[0]  # [256,384]
        if not patch_stays_on_device:
            patch = patch.cpu()
    return cls, patch


@torch.no_grad()
def dinov2_forward_batch(model, crops, device):
    """ONE forward for a whole frame's crops (opt2).

    crops: list of [3,224,224] tensors. Returns (cls[B,384] on CPU,
    patch[B,256,384] kept ON DEVICE — transfer only the winners). Same math
    as dinov2_forward called B times; batching removes the per-call
    kernel-launch overhead (probe-verified bit-identical scores).
    """
    x = torch.stack(crops, 0).to(device)
    out = model(x, is_training=True)
    cls = F.normalize(out["x_norm_clstoken"], dim=-1).cpu()
    patch = F.normalize(out["x_norm_patchtokens"], dim=-1)
    return cls, patch


# ---------------------------------------------------------------------------
# Template features (cls + masked patch), built once and cached
# ---------------------------------------------------------------------------
def mask_bbox(mask_uint8):
    ys, xs = np.where(mask_uint8 > 0)
    if len(xs) == 0:
        return None
    return [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]


def _template_paths(template_dir):
    rgb_paths = sorted(
        glob.glob(os.path.join(template_dir, "rgb_*.png")),
        key=lambda p: int(os.path.basename(p).split("_")[1].split(".")[0]),
    )
    if not rgb_paths:
        raise FileNotFoundError(f"No rgb_*.png templates under {template_dir}")
    return rgb_paths


def build_template_cls(template_dir, model, device, cache_path, rebuild=False):
    if os.path.isfile(cache_path) and not rebuild:
        blob = torch.load(cache_path, map_location="cpu")
        return blob["features"], blob
    feats, used = [], []
    for rp in _template_paths(template_dir):
        idx = os.path.basename(rp).split("_")[1].split(".")[0]
        mp = os.path.join(template_dir, f"mask_{idx}.png")
        rgb = np.array(Image.open(rp).convert("RGB"))
        box = mask_bbox(np.array(Image.open(mp).convert("L"))) if os.path.isfile(mp) else None
        if box is None:
            box = [0, 0, rgb.shape[1], rgb.shape[0]]
        crop = crop_resize_pad(normalize_rgb(rgb), box)
        if crop is None:
            continue
        cls, _ = dinov2_forward(model, crop, device, want_patch=False)
        feats.append(cls); used.append(os.path.basename(rp))
    feats = torch.stack(feats)
    blob = {"features": feats, "template_paths": used, "model": "dinov2_vits14",
            "token": "x_norm_clstoken", "dim": feats.shape[1], "normalized": True,
            "template_dir": template_dir}
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    torch.save(blob, cache_path)
    return feats, blob


def build_template_appe(template_dir, model, device, cache_path, rebuild=False):
    """Per-template *masked* patch features (foreground patches only).

    Mirrors SAM-6D compute_masked_patch_feature: masked rgb crop -> patch tokens,
    keep patches whose pooled mask coverage > VALID_PATCH_THRESH, L2-normalize.
    Returns a list of [Np_i, 384] tensors (variable length per template).
    """
    if os.path.isfile(cache_path) and not rebuild:
        blob = torch.load(cache_path, map_location="cpu")
        return blob["patch_features"], blob
    pool = torch.nn.AvgPool2d(PATCH, PATCH)
    patch_feats, used = [], []
    for rp in _template_paths(template_dir):
        idx = os.path.basename(rp).split("_")[1].split(".")[0]
        mp = os.path.join(template_dir, f"mask_{idx}.png")
        rgb = np.array(Image.open(rp).convert("RGB"))
        if not os.path.isfile(mp):
            continue
        mask = np.array(Image.open(mp).convert("L"))
        box = mask_bbox(mask)
        if box is None:
            continue
        # masked rgb crop (SAM-6D masks rgb before crop) + cropped mask
        norm = normalize_rgb(rgb)
        m = torch.from_numpy((mask > 0).astype(np.float32)).unsqueeze(0)  # [1,H,W]
        masked = norm * m
        crop = crop_resize_pad(masked, box)
        mcrop = crop_resize_pad(m, box, mode="nearest")
        if crop is None or mcrop is None:
            continue
        with torch.no_grad():
            out = model(crop.unsqueeze(0).to(device), is_training=True)
        pt = F.normalize(out["x_norm_patchtokens"][0].cpu(), dim=-1)  # [256,384]
        cover = pool(mcrop.unsqueeze(0)).flatten(-2)[0, 0]            # [256]
        valid = cover > VALID_PATCH_THRESH
        if valid.sum() < 1:
            valid = cover > 0
        patch_feats.append(pt[valid])  # [Np_i, 384]
        used.append(os.path.basename(rp))
    blob = {"patch_features": patch_feats, "template_paths": used,
            "model": "dinov2_vits14", "token": "x_norm_patchtokens",
            "valid_patch_thresh": VALID_PATCH_THRESH, "template_dir": template_dir}
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    torch.save(blob, cache_path)
    return patch_feats, blob


# ---------------------------------------------------------------------------
# Scores
# ---------------------------------------------------------------------------
def semantic_score(crop_cls, template_cls, match_topk):
    """cosine(crop cls, each template cls) -> top-k mean  (ISM semantic)."""
    sims = template_cls @ crop_cls           # [N_tmpl]
    k = min(match_topk, sims.shape[0])
    return float(torch.topk(sims, k).values.mean())


def appearance_score(query_patches, template_patch_list, match_topk):
    """Appearance (appe) score for the selected proposal.

    For each template: every template foreground patch finds its best-matching
    query patch (max cosine); average over template patches. Aggregate across
    templates with a top-k mean (ISM-style). Query has no mask (deferred), so all
    crop patches participate; anchoring on template fg patches keeps it robust to
    query background.
    """
    per_t = []
    for tp in template_patch_list:
        if tp.shape[0] == 0:
            continue
        sim = tp @ query_patches.T          # [Np_t, 256]
        per_t.append(float(sim.max(dim=1).values.mean()))
    if not per_t:
        return 0.0
    vals = torch.tensor(per_t)
    k = min(match_topk, vals.shape[0])
    return float(torch.topk(vals, k).values.mean())


# ---------------------------------------------------------------------------
# Mask (MobileSAM) + masked-appe 2nd gate  (opt-in via --use-mask)
# ---------------------------------------------------------------------------
def build_segmentor(weights, device):
    """ultralytics SAM/MobileSAM for bbox-prompted mask of the selected proposal."""
    from ultralytics import SAM
    return SAM(weights)


def segment_box(seg, bgr, box, device):
    """Full-image boolean mask for the bbox prompt, or None."""
    res = seg(bgr, bboxes=[box], verbose=False,
              device=0 if device.startswith("cuda") else "cpu")
    if res and res[0].masks is not None and len(res[0].masks.data) > 0:
        m = res[0].masks.data[0].cpu().numpy().astype(bool)
        if m.shape != bgr.shape[:2]:
            m = cv2.resize(m.astype(np.uint8), (bgr.shape[1], bgr.shape[0]),
                           interpolation=cv2.INTER_NEAREST).astype(bool)
        return m
    return None


def segment_boxes(seg, bgr, boxes, device):
    """ONE MobileSAM call for ALL bbox prompts of a frame (opt2).

    The expensive image encoding runs once; each bbox only pays the decoder.
    Returns full-image boolean masks aligned to `boxes` (None where missing).
    Mask<->box order correspondence was verified empirically against per-box
    segment_box() calls (IoU=1.0000 on real frames).
    """
    res = seg(bgr, bboxes=list(boxes), verbose=False,
              device=0 if device.startswith("cuda") else "cpu")
    masks = []
    if res and res[0].masks is not None:
        data = res[0].masks.data
        for i in range(min(len(boxes), len(data))):
            m = data[i].cpu().numpy().astype(bool)
            if m.shape != bgr.shape[:2]:
                m = cv2.resize(m.astype(np.uint8), (bgr.shape[1], bgr.shape[0]),
                               interpolation=cv2.INTER_NEAREST).astype(bool)
            masks.append(m)
    while len(masks) < len(boxes):
        masks.append(None)
    return masks


def masked_query_patches(qpatch, mask_full, box, pool):
    """Keep query patches whose pooled mask coverage > thresh (foreground)."""
    mt = torch.from_numpy(mask_full[None].astype(np.float32))
    mcrop = crop_resize_pad(mt, box, mode="nearest")
    if mcrop is None:
        return qpatch, qpatch.shape[0]
    cover = pool(mcrop.unsqueeze(0))[0].flatten()
    valid = cover > VALID_PATCH_THRESH
    return qpatch[valid], int(valid.sum())


def masked_appe_score(query_fg, template_patches):
    """SAM-6D faithful query-anchored masked-patch similarity vs ONE template.

    Each query foreground patch finds its best-matching template patch (max
    cosine); mean over query foreground patches (= loss.py compute_straight).
    """
    if query_fg.shape[0] == 0 or template_patches.shape[0] == 0:
        return 0.0
    sim = query_fg @ template_patches.T
    return float(sim.max(dim=1).values.mean().clamp(0, 1))


# ---------------------------------------------------------------------------
# Geometric score (depth) — SAM-6D ISM faithful: project CAD at best pose +
# depth-estimated translation, IoU vs proposal; + visible_ratio.  (--use-geometric)
# ---------------------------------------------------------------------------
def load_cad_pointcloud(ply_path, n_points=2048):
    import trimesh
    mesh = trimesh.load(ply_path, force="mesh")
    pts = np.asarray(mesh.sample(n_points), dtype=np.float32)
    if np.abs(pts).max() > 10:        # mm -> m (SAM-6D convention)
        pts = pts / 1000.0
    return torch.from_numpy(pts)       # [N,3] meters


def estimate_translation(depth_m, mask, fx, fy, cx, cy):
    """Mean 3D point of the masked depth (proposal) = object translation (m)."""
    sel = mask & (depth_m > 0)
    ys, xs = np.where(sel)
    if xs.size < 10:
        return None
    z = depth_m[ys, xs]
    X = (xs - cx) / fx * z
    Y = (ys - cy) / fy * z
    return np.array([X.mean(), Y.mean(), z.mean()], dtype=np.float32)


def box_iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return float(inter / ua) if ua > 0 else 0.0


def geometric_iou(cad_pc, R, t, fx, fy, cx, cy, proposal_box):
    """Project CAD (posed by R, t) to image; IoU(projected bbox, proposal bbox)."""
    posed = (torch.as_tensor(R, dtype=torch.float32) @ cad_pc.T).T \
        + torch.as_tensor(t, dtype=torch.float32)        # [N,3]
    z = posed[:, 2].clamp(min=1e-6)
    u = posed[:, 0] * fx / z + cx
    v = posed[:, 1] * fy / z + cy
    proj = [float(u.min()), float(v.min()), float(u.max()), float(v.max())]
    return box_iou(proj, proposal_box), proj


def visible_ratio_score(query_fg, template_patches, thred=0.5):
    """Fraction of template (object) patches matched (cos>thred) in the query."""
    if query_fg.shape[0] == 0 or template_patches.shape[0] == 0:
        return 0.0
    sim = query_fg @ template_patches.T          # [Nq, Nr]
    per_ref = sim.max(dim=0).values              # [Nr]
    return float((per_ref > thred).float().mean())


# ---------------------------------------------------------------------------
# Frame source — reuse extracted frames, else read ROS2 bag via rosbags
# ---------------------------------------------------------------------------
def resolve_frames(bag_path, frames_dir, stride, max_frames, topic):
    bag_name = os.path.basename(os.path.normpath(bag_path))
    candidates = []
    if frames_dir:
        candidates.append(frames_dir)
    candidates.append(os.path.join(REPO_ROOT, "outputs", "yolo_test", bag_name, "frames"))
    frame_files, src = None, None
    for c in candidates:
        if c and os.path.isdir(c):
            fs = sorted(glob.glob(os.path.join(c, "*.png")) + glob.glob(os.path.join(c, "*.jpg")))
            if fs:
                frame_files, src = fs, c
                break
    if frame_files is not None:
        print(f"[frames] reusing {len(frame_files)} extracted frames from {src}")
        emitted = 0
        for i, fp in enumerate(frame_files):
            if i % stride != 0:
                continue
            if max_frames and emitted >= max_frames:
                break
            img = cv2.imread(fp)
            if img is None:
                continue
            emitted += 1
            yield i, os.path.splitext(os.path.basename(fp))[0], img
        return

    print(f"[frames] no extracted frames found; reading bag {bag_path} via rosbags")
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    uri = bag_path
    if os.path.isdir(os.path.join(bag_path, "bag")):
        uri = os.path.join(bag_path, "bag")
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(uri)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise RuntimeError(f"topic {topic} not found in {uri}")
        i, emitted = -1, 0
        for conn, _t, raw in reader.messages(connections=conns):
            i += 1
            if i % stride != 0:
                continue
            if max_frames and emitted >= max_frames:
                break
            msg = reader.deserialize(raw, conn.msgtype)
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if msg.encoding.lower() == "rgb8" else buf.copy()
            emitted += 1
            yield i, f"{i:06d}", img


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------
def draw_result(bgr, box, rank, yolo_score, sem, appe):
    out = bgr.copy()
    x1, y1, x2, y2 = [int(v) for v in box]
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 200, 0), 2)
    label = f"#{rank} milk yolo={yolo_score:.3f} sem={sem:.3f} appe={appe:.3f}"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(out, (x1, max(0, y1 - th - 6)), (x1 + tw + 4, y1), (0, 200, 0), -1)
    cv2.putText(out, label, (x1 + 2, max(10, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------
# Args / main
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="YOLO-World proposals matched by SAM-6D ISM DINOv2 cls(semantic)+patch(appe) (MVP step A).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--bag", default="data/ros2_bag/two_table_diagonal1")
    p.add_argument("--template-dir",
                   default="sam6d_master/SAM-6D/Data/custom/Milk_scaled_195mm/templates/")
    p.add_argument("--output-dir", default="outputs/temp/yolo_ism/two_table_diagonal1/")
    p.add_argument("--top-k", type=int, default=3, help="max YOLO proposals kept per frame")
    p.add_argument("--score-threshold", type=float, default=0.05,
                   help="YOLO-World confidence threshold (existing pipeline PRIMARY=0.05)")
    p.add_argument("--similarity-threshold", type=float, default=0.5,
                   help="semantic(cls) 1st gate for Milk acceptance")
    p.add_argument("--use-mask", action="store_true",
                   help="segment the SELECTED proposal (MobileSAM) and apply a "
                        "masked-appe 2nd gate (precision recovery)")
    p.add_argument("--seg-weights", default=os.path.join(REPO_ROOT, "mobile_sam.pt"),
                   help="ultralytics SAM/MobileSAM weights for proposal masking")
    p.add_argument("--appe-gate", type=float, default=0.59,
                   help="masked-appe 2nd-gate threshold (only with --use-mask)")
    p.add_argument("--use-geometric", action="store_true",
                   help="compute SAM-6D geometric score (depth) for sem-passing "
                        "frames -> geometric.csv (implies --use-mask)")
    p.add_argument("--intrinsics", default=os.path.join(REPO_ROOT, "config", "camera_intrinsics.json"))
    p.add_argument("--cad-ply", default=os.path.join(REPO_ROOT, "data", "cad", "milk", "Milk.ply"))
    p.add_argument("--poses-npy", default=os.path.join(
        ISM_DIR, "utils", "poses", "predefined_poses", "cam_poses_level0.npy"))
    p.add_argument("--depth-dir", default="",
                   help="dir with <name>.png aligned-depth (uint16); "
                        "default outputs/depth_synced/<bag>/")
    p.add_argument("--visible-thred", type=float, default=0.5)
    p.add_argument("--geom-pc-points", type=int, default=2048)
    p.add_argument("--match-topk", type=int, default=5,
                   help="top-k template average for semantic & appe (ISM-style)")
    p.add_argument("--frames-dir", default="")
    p.add_argument("--topic", default="/camera/camera/color/image_raw")
    p.add_argument("--stride", type=int, default=10, help="process every Nth frame")
    p.add_argument("--max-frames", type=int, default=0, help="0 = no cap")
    p.add_argument("--weights", default="yolov8m-worldv2.pt")
    p.add_argument("--prompt", default="milk carton")
    p.add_argument("--dinov2-checkpoint", default=DEFAULT_DINOV2_CKPT)
    p.add_argument("--cls-cache",
                   default="outputs/temp/yolo_ism/template_features/milk_dinov2_cls_features.pt")
    p.add_argument("--appe-cache",
                   default="outputs/temp/yolo_ism/template_features/milk_dinov2_appe_features.pt")
    p.add_argument("--rebuild-features", action="store_true")
    p.add_argument("--device", default="cuda:0")
    return p.parse_args()


def main():
    args = parse_args()
    device = args.device if torch.cuda.is_available() else "cpu"
    os.makedirs(args.output_dir, exist_ok=True)

    print(f"[dinov2] loading {args.dinov2_checkpoint}")
    model = build_dinov2(args.dinov2_checkpoint, device)

    tcls, cls_blob = build_template_cls(args.template_dir, model, device,
                                        args.cls_cache, args.rebuild_features)
    tappe, appe_blob = build_template_appe(args.template_dir, model, device,
                                           args.appe_cache, args.rebuild_features)
    n_patch = [t.shape[0] for t in tappe]
    print(f"[templates] cls={tuple(tcls.shape)} | appe: {len(tappe)} templates, "
          f"fg-patches/template min/med/max = {min(n_patch)}/"
          f"{int(statistics.median(n_patch))}/{max(n_patch)} (each patch dim 384)")

    from ultralytics import YOLOWorld
    try:
        yolo = YOLOWorld(args.weights)
    except Exception as e:
        print(f"[yolo] {args.weights} failed ({e}); falling back to yolov8s-worldv2.pt")
        yolo = YOLOWorld("yolov8s-worldv2.pt")
    yolo.set_classes([args.prompt])

    segmentor = appe_pool = None
    if args.use_mask or args.use_geometric:
        print(f"[seg] loading {args.seg_weights} (masked-appe gate={args.appe_gate})")
        segmentor = build_segmentor(args.seg_weights, device)
        appe_pool = torch.nn.AvgPool2d(PATCH, PATCH)

    geo = None
    if args.use_geometric:
        import json
        K = json.load(open(args.intrinsics))
        cad_pc = load_cad_pointcloud(args.cad_ply, args.geom_pc_points)
        poses = np.load(args.poses_npy)            # [42,4,4]
        depth_dir = args.depth_dir or os.path.join(
            REPO_ROOT, "outputs", "depth_synced",
            os.path.basename(os.path.normpath(args.bag)))
        geo = {"fx": K["fx"], "fy": K["fy"], "cx": K["cx"], "cy": K["cy"],
               "depth_scale": K.get("depth_scale", 0.001), "cad": cad_pc,
               "poses": poses, "depth_dir": depth_dir, "vis_thred": args.visible_thred}
        print(f"[geo] CAD pts={tuple(cad_pc.shape)} poses={poses.shape} "
              f"depth_dir={depth_dir} (exists={os.path.isdir(depth_dir)})")
    geo_rows = []

    csv_path = os.path.join(args.output_dir, "yolo_ism_results.csv")
    fields = ["frame_idx", "timestamp", "num_yolo_proposals", "best_yolo_score",
              "best_semantic_score", "best_appe_score", "selected_bbox_xyxy",
              "decision", "output_image_path", "masked_appe_score", "mask_area_px"]
    rows = []
    n_frames = n_no_prop = n_below_sim = n_below_appe = n_milk = 0
    sem_accepted, appe_accepted = [], []

    for frame_idx, name, bgr in resolve_frames(
        args.bag, args.frames_dir, max(1, args.stride), args.max_frames, args.topic):
        n_frames += 1
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        res = yolo.predict(bgr, conf=args.score_threshold, imgsz=640, verbose=False,
                           device=0 if device.startswith("cuda") else "cpu")
        boxes, scores = [], []
        if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
            b = res[0].boxes
            for j in torch.argsort(b.conf, descending=True)[: args.top_k]:
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                boxes.append([x1, y1, x2, y2]); scores.append(float(b.conf[j]))

        num_prop = len(boxes)
        best_yolo = max(scores) if scores else 0.0
        out_path = os.path.join(args.output_dir, f"frame_{frame_idx:06d}.jpg")

        if num_prop == 0:
            n_no_prop += 1
            cv2.imwrite(out_path, bgr)
            rows.append([frame_idx, name, 0, 0.0, 0.0, 0.0, "",
                         "no-object(no-proposal)", out_path, 0.0, 0])
            continue

        # --- SEMANTIC: select best proposal among up to 3 ---
        norm_full = normalize_rgb(rgb)
        best = None  # (sem, box, yolo, rank, crop, cls)
        for rank, (box, sc) in enumerate(zip(boxes, scores), start=1):
            crop = crop_resize_pad(norm_full, box)
            if crop is None:
                continue
            cls, _ = dinov2_forward(model, crop, device, want_patch=False)
            sem = semantic_score(cls, tcls, args.match_topk)
            if best is None or sem > best[0]:
                best = (sem, box, sc, rank, crop, cls)

        best_sem = best[0] if best else 0.0
        if not (best and best_sem >= args.similarity_threshold):
            n_below_sim += 1
            cv2.imwrite(out_path, bgr)
            rows.append([frame_idx, name, num_prop, round(best_yolo, 4),
                         round(best_sem, 4), 0.0, "", "no-object(below-sim)",
                         out_path, 0.0, 0])
            continue

        # semantic 1st gate passed -> SELECTED proposal
        _, qpatch = dinov2_forward(model, best[4], device, want_patch=True)
        if args.use_mask:
            # --- MASK + masked-appe 2nd gate (SAM-6D faithful, single best tmpl) ---
            x1, y1, x2, y2 = best[1]
            mask = segment_box(segmentor, bgr, best[1], device)
            mask_area = int(mask[y1:y2, x1:x2].sum()) if mask is not None else 0
            q_fg = masked_query_patches(qpatch, mask, best[1], appe_pool)[0] \
                if mask is not None else qpatch
            best_t = int(torch.argmax(tcls @ best[5]))
            appe = masked_appe_score(q_fg, tappe[best_t])
            if geo is not None:
                geo_iou = vis = 0.0
                dpath = os.path.join(geo["depth_dir"], f"{name}.png")
                dimg = cv2.imread(dpath, cv2.IMREAD_UNCHANGED) if os.path.isfile(dpath) else None
                if dimg is not None and mask is not None:
                    depth_m = dimg.astype(np.float32) * geo["depth_scale"]
                    t = estimate_translation(depth_m, mask, geo["fx"], geo["fy"],
                                             geo["cx"], geo["cy"])
                    if t is not None:
                        R = geo["poses"][best_t][:3, :3]
                        geo_iou, _ = geometric_iou(geo["cad"], R, t, geo["fx"], geo["fy"],
                                                   geo["cx"], geo["cy"], best[1])
                        vis = visible_ratio_score(q_fg, tappe[best_t], geo["vis_thred"])
                final_score = (best_sem + appe + geo_iou * vis) / (1 + 1 + vis)
                geo_rows.append([frame_idx, name, round(best_sem, 4), round(appe, 4),
                                 round(geo_iou, 4), round(vis, 4), round(final_score, 4),
                                 ";".join(str(v) for v in best[1])])
            if appe >= args.appe_gate:
                n_milk += 1; sem_accepted.append(best_sem); appe_accepted.append(appe)
                out = draw_result(bgr, best[1], best[3], best[2], best_sem, appe)
                if mask is not None:
                    cnts, _ = cv2.findContours(mask.astype(np.uint8),
                                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(out, cnts, -1, (0, 200, 0), 2)
                cv2.imwrite(out_path, out)
                rows.append([frame_idx, name, num_prop, round(best_yolo, 4),
                             round(best_sem, 4), 0.0,
                             ";".join(str(v) for v in best[1]), "milk", out_path,
                             round(appe, 4), mask_area])
            else:
                n_below_appe += 1
                cv2.imwrite(out_path, bgr)
                rows.append([frame_idx, name, num_prop, round(best_yolo, 4),
                             round(best_sem, 4), 0.0, "", "no-object(below-appe)",
                             out_path, round(appe, 4), mask_area])
        else:
            # --- legacy step-A: appe recorded only, semantic is the gate ---
            appe = appearance_score(qpatch, tappe, args.match_topk)
            n_milk += 1; sem_accepted.append(best_sem); appe_accepted.append(appe)
            out = draw_result(bgr, best[1], best[3], best[2], best_sem, appe)
            cv2.imwrite(out_path, out)
            rows.append([frame_idx, name, num_prop, round(best_yolo, 4),
                         round(best_sem, 4), round(appe, 4),
                         ";".join(str(v) for v in best[1]), "milk", out_path, 0.0, 0])

    with open(csv_path, "w", newline="") as f:
        wcsv = csv.writer(f); wcsv.writerow(fields); wcsv.writerows(rows)

    if args.use_geometric:
        geo_path = os.path.join(args.output_dir, "geometric.csv")
        with open(geo_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame_idx", "timestamp", "best_semantic_score",
                        "masked_appe_score", "geometric_iou", "visible_ratio",
                        "final_score", "selected_bbox_xyxy"])
            w.writerows(geo_rows)
        print(f"[geo] {len(geo_rows)} sem-pass frames scored -> {geo_path}")

    print("\n===== SUMMARY =====")
    print(f"frames processed        : {n_frames}")
    print(f"0 YOLO proposals        : {n_no_prop}")
    print(f"proposals but below sem : {n_below_sim}")
    if args.use_mask:
        print(f"sem-pass but below appe : {n_below_appe}")
    print(f"final Milk decisions    : {n_milk}")
    if sem_accepted:
        print(f"accepted semantic mean/med : {statistics.mean(sem_accepted):.4f} / "
              f"{statistics.median(sem_accepted):.4f}")
    if appe_accepted:
        print(f"accepted appe     mean/med : {statistics.mean(appe_accepted):.4f} / "
              f"{statistics.median(appe_accepted):.4f}")
    print(f"CSV  : {csv_path}")
    print(f"imgs : {args.output_dir}")


if __name__ == "__main__":
    main()
