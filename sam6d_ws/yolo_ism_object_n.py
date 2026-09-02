#!/usr/bin/env python3
"""yolo_ism_object_n.py — config-driven MULTI-object YOLO-World × SAM-6D ISM.

Generalizes the single-object ``yolo_ism.py`` to N objects WITHOUT modifying it:
every DINOv2 / template / scoring / frame-source helper is imported and reused
from ``yolo_ism`` (read-only). Per RGB frame:

  1. YOLO-World runs ONCE over the set of UNIQUE object prompts (set_classes).
  2. Detected boxes are routed by their predicted prompt to every enabled
     object sharing that prompt (high/low variants share a prompt; they differ
     only in their ISM *template* features, not in YOLO text).
  3. For each object: its proposals -> DINOv2 cls (SEMANTIC, 1st gate) selects
     the best proposal; if --use_mask, MobileSAM masks it and a masked DINOv2
     patch score (APPEarance, 2nd gate) decides acceptance.

Object prompts / template dirs / feature-cache paths / per-object threshold
overrides all come from a YAML config (configs/yolo_ism_objects.yaml) — nothing
object-specific is hard-coded here.

SCOPE: ISM recognition only. PEM / 6D pose / geometric(depth) are OUT OF SCOPE.
"""

import argparse
import csv
import os
import statistics

import cv2
import torch

import yolo_ism as yi  # read-only reuse of the single-object pipeline
import ism_hsv         # Phase 1B: HSV shadow-gate authority (shadow only; no active gate)
import ism_training_free_gate as _tfg   # Phase 2.2: training-free rescue (DEFAULT OFF = no-op)


REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "configs", "yolo_ism_objects.yaml")
DEFAULT_FEATURE_DIR = os.path.join(
    REPO_ROOT, "outputs", "yolo_ism_object_n", "template_features")

# Keys an object entry may override from `defaults`.
_OVERRIDABLE = (
    "weights", "device", "top_k", "score_threshold", "similarity_threshold",
    "use_mask", "appe_gate", "match_topk", "seg_weights", "stride",
    "dinov2_checkpoint", "appe_blocks", "nms_rank_blocks",
    # Phase 1B HSV gate (per-object override allowed; deploy keeps them uniform)
    "hsv_gate_enabled", "hsv_gate_shadow_mode", "hsv_gate_threshold",
    "hsv_hue_shift_deg", "hsv_sat_gain", "hsv_hist_bins", "hsv_lowsat_exception",
    "hsv_cache", "rg_gate_threshold",
    # Phase 1C class color correction (OpenCV hue units, baked into the class HSV cache)
    "hsv_hue_correction_ocv",
    # Phase 3 appearance-v2 + cross-object NMS (per-object override allowed; deploy uniform)
    "appe_v2_enabled", "appe_v2_blocks", "appe_v2_gate",
    "cross_object_nms_enabled", "cross_object_nms_iou",
    # Phase 4 multi-label proposals + relative (per-box) owner election + colour tie-break
    "yolo_multi_label", "relative_assignment_enabled", "color_tiebreak_template_sim",
)

# TWO separate appe scores, on purpose:
#   appe_blocks     -> res["masked_appe"]: drives the ABSOLUTE gate and the output
#                      json score. [11] = final block = historical, bit-identical.
#   nms_rank_blocks -> res["rank_appe"]:  drives ONLY the cross-object NMS winner,
#                      a RELATIVE decision between two labels claiming one box.
# They are split because a gate and a ranking are different problems. Changing the
# gate's score changes its operating point: shipping appe_blocks=[2,9] on 2026-07-15
# raised every score, left the absolute gate at 0.55, and let detections jump
# 5,476 -> 6,357 (+16%) -- mostly cartons. A ranking has no threshold to slip, so
# a better ranking score is free: detection count cannot change.
DEFAULT_APPE_BLOCKS = [11]


DEFAULT_APPE_V2_BLOCKS = [2, 9]
DEFAULT_APPE_V2_GATE = 0.605


def _appe_v2_on(o):
    return bool(o.get("appe_v2_enabled", False))


def _blocks_of(o):
    """Blocks the ABSOLUTE masked-appe gate averages over.

    appe_v2 (Phase 3, default OFF) swaps blocks AND gate together. They must move
    as a pair: [2,9] shifts the whole score distribution upward, so reusing the
    old 0.55 gate is the exact regression that got [2,9] reverted on 2026-07-15.
    """
    if _appe_v2_on(o):
        b = o.get("appe_v2_blocks") or DEFAULT_APPE_V2_BLOCKS
    else:
        b = o.get("appe_blocks") or DEFAULT_APPE_BLOCKS
    return [int(v) for v in (b if isinstance(b, (list, tuple)) else [b])]


def _appe_gate_of(o):
    """Absolute masked-appe threshold, matched to whichever blocks _blocks_of picked."""
    if _appe_v2_on(o):
        return float(o.get("appe_v2_gate", DEFAULT_APPE_V2_GATE))
    return float(o["appe_gate"])


def _rank_blocks_of(o):
    """Blocks for the NMS-ranking score; defaults to the gate's blocks (= no change)."""
    b = o.get("nms_rank_blocks")
    if not b:
        return _blocks_of(o)
    return [int(v) for v in (b if isinstance(b, (list, tuple)) else [b])]


@torch.no_grad()
def dinov2_blocks_forward(model, crops, device, blocks):
    """(cls, {block: patch}) for a batch of crops, in ONE pass over the blocks.

    get_intermediate_layers() runs prepare_tokens + every block exactly once and
    applies self.norm only to the blocks asked for, so requesting [2, 9, 11]
    costs the same as the plain forward (measured +0.1%, not +104% as a second
    forward would). For the final block its output is bit-identical to
    model(x, is_training=True)'s x_norm_* (verified max|diff| = 0.0), which is
    what guarantees appe_blocks=[11] reproduces the old scores exactly.

    Patch tokens stay ON DEVICE (callers transfer only the winner).
    """
    need = sorted(set(blocks) | {model_last_block(model)})
    x = torch.stack(crops, 0).to(device) if isinstance(crops, list) else crops.to(device)
    outs = model.get_intermediate_layers(x, n=need, return_class_token=True, norm=True)
    per = {b: torch.nn.functional.normalize(o, dim=-1) for b, (o, _) in zip(need, outs)}
    cls = torch.nn.functional.normalize(outs[need.index(model_last_block(model))][1], dim=-1).cpu()
    return cls, per


def model_last_block(model):
    return len(model.blocks) - 1

CSV_FIELDS = ["frame_idx", "timestamp", "num_proposals", "best_yolo_score",
              "best_semantic_score", "best_appe_score", "selected_bbox_xyxy",
              "decision", "output_image_path", "masked_appe_score", "mask_area_px",
              # Phase 1A diagnostics (append-only; existing columns unchanged)
              "configured_score_threshold",
              "candidate_count_before_score_threshold",
              "candidate_count_after_score_threshold",
              # Phase 1B HSV shadow diagnostics (append-only; empty when HSV off)
              "hsv_score", "hsv_threshold", "hsv_pass", "would_hsv_reject",
              "hsv_authority_version", "hsv_template_cache_version"]


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
def _abspath(p):
    return p if os.path.isabs(p) else os.path.join(REPO_ROOT, p)


def _hsv_on(o):
    """True when the HSV gate should be COMPUTED (shadow logging or active). Phase 1B = shadow."""
    return bool(o.get("hsv_gate_enabled", False)) or bool(o.get("hsv_gate_shadow_mode", False))


def _apply_hsv_gate(o, res, bgr):
    """HSV + optional normalized-rg gate on an ACCEPTED candidate. Computes+logs HSV always (when HSV
    is on), and — ONLY when `hsv_gate_enabled` is true (Phase 1C active) — flips an HSV
    failure to reject. Shadow mode (`hsv_gate_shadow_mode` true, `hsv_gate_enabled` false)
    records the fields but NEVER changes the decision (Phase 1B). Safe no-op when HSV is off,
    the candidate was not accepted, or the box is missing. Fail-open: missing cache -> proto
    None -> score 1.0 -> passes (never wrongly rejects on a cache error).
    """
    if not _hsv_on(o) or not res.get("accepted") or res.get("box") is None:
        return
    thr = float(o.get("hsv_gate_threshold", 0.0))
    score = ism_hsv.shadow_score(bgr, res["box"], res.get("mask"), o.get("_hsv_proto"))
    hp = bool(score >= thr)                             # higher score = more similar color
    res["hsv_score"] = round(float(score), 5)
    res["hsv_threshold"] = thr
    res["hsv_pass"] = int(hp)
    res["would_hsv_reject"] = int(not hp)              # accepted now; active-HSV would drop it
    res["hsv_authority_version"] = ism_hsv.HSV_AUTHORITY_VERSION
    res["hsv_template_cache_version"] = o.get("_hsv_meta", {}).get("cache_version", "")
    res["hsv_gate_enabled"] = int(bool(o.get("hsv_gate_enabled", False)))
    if bool(o.get("hsv_gate_enabled", False)) and not hp:
        res["accepted"] = False
        res["decision"] = "no-object(below-hsv)"
        return
    rg_thr = float(o.get("rg_gate_threshold", 0.0))
    rg_pass = True
    if rg_thr > 0:
        rg_score = ism_hsv.shadow_rg_score(
            bgr, res["box"], res.get("mask"), o.get("_rg_proto"))
        rg_pass = bool(rg_score >= rg_thr)
        res.update({"rg_score": round(float(rg_score), 5), "rg_threshold": rg_thr,
                    "rg_pass": int(rg_pass)})
    if bool(o.get("hsv_gate_enabled", False)) and not rg_pass:
        res["accepted"] = False
        res["decision"] = "no-object(below-rg)"


def _apply_tf_gate(o, res):
    """Phase 2.2 training-free rescue hook. DEFAULT OFF => complete no-op (Phase 1C exact).
    When enabled, applies a fixed deterministic rule (no learning) to a rejected candidate
    using only its already-computed gate scores; fail-safe (missing scores -> no rescue)."""
    if not bool(o.get("training_free_gate_enabled", False)):
        return
    changed, reason = _tfg.decide(o.get("training_free_gate_method", "phase1c"), res, o)
    res["tf_gate_reason"] = reason
    if changed:
        res["accepted"] = True
        res["decision"] = "detected(tf-rescued)"


def load_config(config_path):
    import yaml
    with open(config_path) as f:
        cfg = yaml.safe_load(f) or {}
    defaults = dict(cfg.get("defaults", {}))
    raw_objs = cfg.get("objects", [])
    if not isinstance(raw_objs, list):
        raise SystemExit(f"[config] `objects` must be a list in {config_path}")
    objs, seen = [], set()
    for i, raw in enumerate(raw_objs):
        if not isinstance(raw, dict):
            raise SystemExit(f"[config] objects[{i}] must be a mapping")
        if not raw.get("enabled", True):
            continue
        o = dict(defaults)            # start from defaults
        o.update(raw)                 # object keys override defaults
        for req in ("name", "yolo_prompt", "template_dir"):
            if not o.get(req):
                raise SystemExit(
                    f"[config] objects[{i}] missing required key '{req}'")
        name = o["name"]
        if name in seen:
            raise SystemExit(f"[config] duplicate object name '{name}' "
                             "(names must be unique — they key CSVs/dirs)")
        seen.add(name)
        o["template_dir"] = _abspath(o["template_dir"])
        o["cls_cache"] = _abspath(o.get(
            "cls_cache", os.path.join(DEFAULT_FEATURE_DIR, f"{name}_cls.pt")))
        o["appe_cache"] = _abspath(o.get(
            "appe_cache", os.path.join(DEFAULT_FEATURE_DIR, f"{name}_appe.pt")))
        o["hsv_cache"] = _abspath(o.get(
            "hsv_cache", os.path.join(DEFAULT_FEATURE_DIR, f"{name}_hsv.npz")))
        objs.append(o)
    if not objs:
        raise SystemExit(f"[config] no enabled objects in {config_path}")
    return defaults, objs


# ---------------------------------------------------------------------------
# Per-object template features (cls + masked-appe), built once via yolo_ism
# ---------------------------------------------------------------------------
def build_template_appe_blocks(template_dir, model, device, cache_path, blocks, rebuild=False):
    """Masked template patch features at EACH requested block -> {block: [ [Np,384], ... ]}.

    Same masking/coverage rules as yolo_ism.build_template_appe (masked rgb before
    crop, AvgPool coverage > VALID_PATCH_THRESH); the only change is that features
    are taken from several blocks in one pass instead of only the final one.
    """
    import glob
    from PIL import Image
    key = "-".join(str(b) for b in blocks)
    if os.path.isfile(cache_path) and not rebuild:
        blob = torch.load(cache_path, map_location="cpu")
        if blob.get("blocks_key") == key:
            return blob["per_block"]
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    per = {b: [] for b in blocks}
    for rp in yi._template_paths(template_dir):
        idx = os.path.basename(rp).split("_")[1].split(".")[0]
        mp = os.path.join(template_dir, f"mask_{idx}.png")
        if not os.path.isfile(mp):
            continue
        import numpy as np
        rgb = np.array(Image.open(rp).convert("RGB"))
        mask = np.array(Image.open(mp).convert("L"))
        box = yi.mask_bbox(mask)
        if box is None:
            continue
        m = torch.from_numpy((mask > 0).astype("float32")).unsqueeze(0)
        crop = yi.crop_resize_pad(yi.normalize_rgb(rgb) * m, box)
        mcrop = yi.crop_resize_pad(m, box, mode="nearest")
        if crop is None or mcrop is None:
            continue
        _, pb = dinov2_blocks_forward(model, [crop], device, blocks)
        cover = pool(mcrop.unsqueeze(0)).flatten(-2)[0, 0]
        valid = cover > yi.VALID_PATCH_THRESH
        if valid.sum() < 1:
            valid = cover > 0
        for b in blocks:
            per[b].append(pb[b][0].cpu()[valid])
    blob = {"per_block": per, "blocks_key": key, "template_dir": template_dir,
            "model": "dinov2_vits14", "valid_patch_thresh": yi.VALID_PATCH_THRESH}
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    torch.save(blob, cache_path)
    return per


def prepare_objects(objs, model, device, rebuild):
    """Attach tcls / tappe template features; drop objects with bad templates."""
    ready = []
    for o in objs:
        tdir = o["template_dir"]
        rgbs = []
        if os.path.isdir(tdir):
            import glob
            rgbs = glob.glob(os.path.join(tdir, "rgb_*.png"))
        if not rgbs:
            print(f"[skip] {o['name']}: no rgb_*.png under {tdir}")
            continue
        blocks = _blocks_of(o)
        rank_blocks = _rank_blocks_of(o)
        need = sorted(set(blocks) | set(rank_blocks))
        try:
            tcls, _ = yi.build_template_cls(tdir, model, device, o["cls_cache"], rebuild)
            tappe, _ = yi.build_template_appe(tdir, model, device, o["appe_cache"], rebuild)
            o["tappe_blocks"] = build_template_appe_blocks(
                tdir, model, device,
                o["appe_cache"].replace("_appe.pt", f"_appe_b{'-'.join(map(str, need))}.pt"),
                need, rebuild)
        except Exception as e:
            print(f"[skip] {o['name']}: template build failed ({e})")
            continue
        o["tcls"], o["tappe"] = tcls, tappe
        o["_blocks"], o["_rank_blocks"], o["_need_blocks"] = blocks, rank_blocks, need
        # ---- Phase 1B: attach HSV reference proto if shadow/active requested ----
        # Loaded ONCE (never per-frame). fail-open: a missing/stale/corrupt cache leaves
        # proto=None so the shadow score is 1.0 and NEVER alters the baseline decision.
        o["_hsv_proto"], o["_rg_proto"], o["_hsv_meta"] = None, None, {"status": "off"}
        if _hsv_on(o):
            proto, meta = ism_hsv.load_cache(o["hsv_cache"], tdir)
            want_hc = int(o.get("hsv_hue_correction_ocv", 0))
            if proto is not None and int(meta.get("hue_correction_ocv", 0)) != want_hc:
                # cache built with a different class color-correction than config -> treat as stale
                meta["status"] = f"hue_correction-mismatch(cache={meta.get('hue_correction_ocv',0)},cfg={want_hc})"
                proto = None
            o["_hsv_proto"], o["_hsv_meta"] = proto, meta
            o["_rg_proto"] = meta.get("rg_proto")
            mode = "ACTIVE" if o.get("hsv_gate_enabled") else "shadow"
            if proto is None:
                print(f"[hsv] {o['name']:20s} cache {meta['status']} -> fail-open "
                      f"(score=1.0, no reject). run tools/build_hsv_template_cache.py")
            else:
                hc = int(meta.get("hue_correction_ocv", 0))
                print(f"[hsv] {o['name']:20s} {mode} proto={tuple(proto.shape)} "
                      f"thr={o.get('hsv_gate_threshold')} hue_corr={hc}ocv ver={meta['cache_version'][:12]}")
            if float(o.get("rg_gate_threshold", 0.0)) > 0 and o.get("_rg_proto") is None:
                print(f"[rg]  {o['name']:20s} cache missing -> fail-open; rebuild HSV cache")
        ready.append(o)
        rk = "" if rank_blocks == blocks else f" rank={rank_blocks}"
        print(f"[templates] {o['name']:20s} cls={tuple(tcls.shape)} "
              f"appe={len(tappe)} tmpl  gate={blocks}{rk}  prompt='{o['yolo_prompt']}'")
    if not ready:
        raise SystemExit("[templates] no usable objects after template loading")
    return ready


def masked_appe_blocks(q_per_block, mask, box, pool, o, t_idx, blocks=None):
    """masked-appe averaged over `blocks` (mean of cosine scores).

    A plain mean, not a rank fusion: every block's score is a cosine similarity on
    the same [0,1] scale, so averaging needs no population statistics and is safe
    to compute one detection at a time.

    Measured 2026-07-15 on 501 hand-labelled detections (6 objects x 6 datasets),
    AUC separating the real object from its false positives:
        object              block 2   block 9   block 11 (gate default)
        choco vs carton       0.55      1.00      0.93
        Dinosaur vs rabbit    1.00      0.66      0.47   <- worse than chance
        Bear vs rabbit/dino   0.92      0.49      0.56
        milk vs carton        0.95      0.80      0.88
        Febreze vs saffron    0.97      0.94      0.91
        saffron vs carton     0.83      0.89      0.85
    Early blocks carry colour, late blocks carry texture; objects whose FP shares
    their colour (choco/saffron: brown box vs brown box) need the late block, the
    rest are separated by the early one. Mean of [2, 9]: average AUC 0.766 -> 0.890,
    worst object 0.466 -> 0.713.
    """
    bl = blocks if blocks is not None else o["_blocks"]
    tot = 0.0
    for b in bl:
        q_fg = yi.masked_query_patches(q_per_block[b], mask, box, pool)[0] \
            if mask is not None else q_per_block[b]
        tot += yi.masked_appe_score(q_fg, o["tappe_blocks"][b][t_idx])
    return tot / len(bl)


def build_prompt_groups(objs):
    """Unique prompts (for set_classes) + prompt-index -> [objects] routing."""
    unique, groups = [], {}
    for o in objs:
        p = o["yolo_prompt"]
        if p not in unique:
            unique.append(p)
        groups.setdefault(unique.index(p), []).append(o)
    return unique, groups


# ---------------------------------------------------------------------------
# Per-object ISM gate on a set of proposals (mirrors yolo_ism --use_mask path)
# ---------------------------------------------------------------------------
def recognize(o, cand, bgr, rgb, model, device, segmentor, pool, norm_full=None):
    """Return dict describing the object's decision for this frame.

    `cand` = the prompt's candidate [(box, conf), ...] sorted by conf desc.
    Per-object `score_threshold` / `top_k` overrides are applied HERE so that a
    single shared YOLO pass still honors each object's own gate.

    Perf (probe-verified, decisions bit-identical): the proposal loop requests
    patch tokens together with cls (the ViT computes them anyway) and keeps
    them on-device, so the old duplicate best-crop forward is gone and only
    the winner pays one device->host copy. `norm_full` may be precomputed
    once per frame by the caller; None keeps the original per-call behavior.
    """
    thr = float(o.get("score_threshold", 0.0))
    passed = [(b, s) for (b, s) in cand if s >= thr]
    sel = passed[:int(o.get("top_k", 3))]
    boxes = [b for b, _ in sel]
    scores = [s for _, s in sel]
    num_prop = len(boxes)
    best_yolo = max(scores) if scores else 0.0
    res = {"num_proposals": num_prop, "best_yolo": best_yolo, "best_sem": 0.0,
           "appe": 0.0, "masked_appe": 0.0, "rank_appe": 0.0, "mask_area": 0,
           "box": None, "mask": None, "rank": 0,
           "configured_score_threshold": thr,
           "candidate_count_before_score_threshold": len(cand),
           "candidate_count_after_score_threshold": len(passed),
           "decision": "no-object(no-proposal)", "accepted": False}
    if num_prop == 0:
        return res

    if norm_full is None:
        norm_full = yi.normalize_rgb(rgb)
    LAST = model_last_block(model)
    best = None  # (sem, box, yolo, rank, crop, cls, per_block_patches)
    for rank, (box, sc) in enumerate(zip(boxes, scores), start=1):
        crop = yi.crop_resize_pad(norm_full, box)
        if crop is None:
            continue
        cls_b, pb = dinov2_blocks_forward(model, [crop], device, o["_need_blocks"])
        cls = cls_b[0]
        sem = yi.semantic_score(cls, o["tcls"], o["match_topk"])
        if best is None or sem > best[0]:
            best = (sem, box, sc, rank, crop, cls, {b: pb[b][0] for b in pb})

    res["best_sem"] = best[0] if best else 0.0
    if not (best and res["best_sem"] >= o["similarity_threshold"]):
        res["decision"] = "no-object(below-sim)"
        return res

    res["box"], res["rank"] = best[1], best[3]
    qblocks = {b: v.cpu() for b, v in best[6].items()}   # winner-only device->host copy

    if not o.get("use_mask", True):
        # legacy step-A: semantic is the gate, appe recorded only
        res["appe"] = yi.appearance_score(qblocks[LAST], o["tappe"], o["match_topk"])
        res["decision"], res["accepted"] = "detected", True
        _apply_hsv_gate(o, res, bgr)               # HSV gate (active reject if enabled; else shadow)
        _apply_tf_gate(o, res)                       # Phase 2.2 training-free rescue (OFF=no-op)
        return res

    # MASK + masked-appe 2nd gate (SAM-6D faithful, single best template)
    x1, y1, x2, y2 = best[1]
    mask = yi.segment_box(segmentor, bgr, best[1], device)
    res["mask"] = mask
    res["mask_area"] = int(mask[y1:y2, x1:x2].sum()) if mask is not None else 0
    best_t = int(torch.argmax(o["tcls"] @ best[5]))
    res["masked_appe"] = masked_appe_blocks(qblocks, mask, best[1], pool, o, best_t)
    res["rank_appe"] = res["masked_appe"] if o["_rank_blocks"] == o["_blocks"] else \
        masked_appe_blocks(qblocks, mask, best[1], pool, o, best_t, o["_rank_blocks"])
    if res["masked_appe"] >= _appe_gate_of(o):
        res["decision"], res["accepted"] = "detected", True
    else:
        res["decision"] = "no-object(below-appe)"
    _apply_hsv_gate(o, res, bgr)                   # HSV gate (active reject if enabled; else shadow)
    _apply_tf_gate(o, res)                       # Phase 2.2 training-free rescue (OFF=no-op)
    return res


# ---------------------------------------------------------------------------
# Phase 4 — one box may carry SEVERAL prompts, and the owner is chosen by
# comparing the objects against each other instead of by independent thresholds.
# ---------------------------------------------------------------------------
def _multi_label_on(o):
    """YOLO NMS keeps only the top-1 prompt per box unless this is set.

    Measured on the 959-cell human GT (2026-08-12): the deployed top-1 rule threw
    away a correctly-placed box whenever a competing prompt outscored the right one
    -- the Sikhye can carried 'white jug 0.225', the green dinosaur carried
    'brown bear doll 0.309'. Letting a box keep every prompt above threshold
    recovered TP 586 -> 686 and LOWERED FP 37 -> 33 (the extra candidates give
    cross-object NMS / relative assignment something to compare).
    """
    return bool(o.get("yolo_multi_label", False))


class multi_label_nms:
    """Context manager: make ultralytics keep every class above conf per box.

    predict() does not expose `multi_label`, so the NMS entry point is wrapped for
    the duration of the call. No-op when `on` is false, and always restored.
    """

    def __init__(self, on=True):
        self.on = bool(on)
        self._mod = self._orig = None

    def __enter__(self):
        if not self.on:
            return self
        from ultralytics.utils import nms as _nms
        self._mod, self._orig = _nms, _nms.non_max_suppression
        orig = self._orig
        _nms.non_max_suppression = lambda *a, **k: orig(*a, **{**k, "multi_label": True})
        return self

    def __exit__(self, *exc):
        if self._mod is not None:
            self._mod.non_max_suppression = self._orig
        return False


def template_similarity(objs):
    """{name: {name: cos}} between the objects' own template (PLY render) features.

    Object-agnostic by construction: it is read off the templates, not written down
    per class. Used to find which objects the SHAPE feature cannot tell apart --
    for the deployed set that is exactly the three plush dolls (Bear/Rabbit 0.83,
    Bear/Dinosaur 0.80, Rabbit/Dinosaur 0.83; every other pair <= 0.63).
    """
    names = [o["name"] for o in objs]
    m = torch.stack([torch.nn.functional.normalize(o["tcls"].float().mean(0), dim=-1)
                     for o in objs])
    s = (m @ m.T).cpu().numpy()
    return {a: {b: float(s[i, j]) for j, b in enumerate(names)} for i, a in enumerate(names)}


def _color_tiebreak_tau(o):
    """Template similarity at/above which two objects are 'shape-indistinguishable'
    and the colour score decides between them. 0 disables the tie-break."""
    return float(o.get("color_tiebreak_template_sim", 0.0))


def assign_frame_relative(objs, prompt_boxes, bgr, rgb, norm_full,
                          model, device, segmentor, pool, tsim=None):
    """Per-BOX owner election. Returns {object_name: res} like recognize_frame().

    recognize_frame() asks, per object, "does this box clear MY thresholds?" — so a
    box the white rabbit should own can be taken by the brown bear simply because
    the bear's prompt won YOLO's top-1 and the rabbit never saw the box. Here every
    distinct box in the frame is scored against EVERY object and goes to the best
    match; the absolute gates (semantic / masked-appe / HSV) still run afterwards
    because FP control is their job, not the election's.

    Colour tie-break: when the runner-up is one the template features cannot
    separate from the winner (`template_similarity >= tau`), the colour score picks
    between them. DINOv2 sees three plush dolls as the same thing (0.80-0.83) while
    their colours -- white / brown / green -- are unmistakable.

    959-cell human GT, measured 2026-08-12:
        deployed                       TP 586  FP 37  F1 .7408
        + multi_label                  TP 686  FP 33  F1 .8176
        + relative assignment          TP 681  FP 32  F1 .8146
        + colour tie-break tau 0.70    TP 706  FP 32  F1 .8321
        + colour tie-break tau 0.60    TP 721  FP 35  F1 .8408   <- deployed value
    """
    by_name = {o["name"]: o for o in objs}
    tsim = tsim if tsim is not None else template_similarity(objs)
    tau = _color_tiebreak_tau(objs[0]) if objs else 0.0

    uniq = []
    for lst in prompt_boxes.values():
        for box, conf in lst:
            for u in uniq:
                if _iou_xyxy(u["box"], box) > 0.95:
                    u["conf"] = max(u["conf"], conf)
                    break
            else:
                uniq.append({"box": list(box), "conf": float(conf)})
    thr = min(float(o.get("score_threshold", 0.0)) for o in objs) if objs else 0.0
    uniq = [u for u in uniq if u["conf"] >= thr]

    res = {o["name"]: {"num_proposals": len(uniq), "best_yolo": 0.0, "best_sem": 0.0,
                       "appe": 0.0, "masked_appe": 0.0, "rank_appe": 0.0, "mask_area": 0,
                       "box": None, "mask": None, "rank": 0,
                       "decision": "no-object(no-proposal)", "accepted": False}
           for o in objs}
    crops, keep = [], []
    for u in uniq:
        c = yi.crop_resize_pad(norm_full, u["box"])
        if c is not None:
            crops.append(c)
            keep.append(u)
    if not crops:
        return res

    need = sorted({b for o in objs for b in o["_need_blocks"]})
    cls_all, patch_all = dinov2_blocks_forward(model, crops, device, need)
    masks = yi.segment_boxes(segmentor, bgr, [u["box"] for u in keep], device)

    best = {}
    for i, u in enumerate(keep):
        sems = {o["name"]: float(yi.semantic_score(cls_all[i], o["tcls"], o["match_topk"]))
                for o in objs}
        top = max(sems, key=sems.get)
        mask = masks[i]
        hsv = {}
        if mask is not None:
            hsv = {o["name"]: float(ism_hsv.shadow_score(bgr, u["box"], mask, o.get("_hsv_proto")))
                   for o in objs}
        if tau > 0.0 and hsv:
            tie = [k for k in sems if tsim[top].get(k, 0.0) >= tau]
            if len(tie) > 1:
                top = max(tie, key=lambda k: hsv[k])
        o = by_name[top]
        r = res[top]
        if sems[top] > r["best_sem"]:
            r["best_sem"] = sems[top]
            r["best_yolo"] = u["conf"]
        if sems[top] < o["similarity_threshold"]:
            if not r["accepted"]:
                r["decision"] = "no-object(below-sim)"
            continue
        if o.get("use_mask", True) and mask is None:
            continue
        qb = {b: patch_all[b][i].cpu() for b in o["_need_blocks"]}
        bt = int(torch.argmax(o["tcls"] @ cls_all[i]))
        m_appe = float(masked_appe_blocks(qb, mask, u["box"], pool, o, bt)) if mask is not None else 0.0
        if m_appe < _appe_gate_of(o):
            if not r["accepted"]:
                r["decision"] = "no-object(below-appe)"
            continue
        if bool(o.get("hsv_gate_enabled", False)) and hsv:
            if hsv[top] < float(o.get("hsv_gate_threshold", 0.0)):
                if not r["accepted"]:
                    r["decision"] = "no-object(below-hsv)"
                continue
        rg_thr = float(o.get("rg_gate_threshold", 0.0))
        rg_score = (ism_hsv.shadow_rg_score(bgr, u["box"], mask, o.get("_rg_proto"))
                    if rg_thr > 0 else 1.0)
        if bool(o.get("hsv_gate_enabled", False)) and rg_score < rg_thr:
            if not r["accepted"]:
                r["decision"] = "no-object(below-rg)"
            continue
        if top in best and m_appe <= best[top]:
            continue                                  # one slot per object, best appe wins
        best[top] = m_appe
        x1, y1, x2, y2 = u["box"]
        r.update({"box": list(u["box"]), "mask": mask, "masked_appe": m_appe,
                  "rank_appe": m_appe, "mask_area": int(mask[y1:y2, x1:x2].sum()) if mask is not None else 0,
                  "decision": "detected", "accepted": True})
        if hsv:
            r["hsv_score"] = round(hsv[top], 5)
            r["hsv_pass"] = 1
        if rg_thr > 0:
            r.update({"rg_score": round(float(rg_score), 5), "rg_threshold": rg_thr,
                      "rg_pass": 1})
    return res


def recognize_frame_auto(groups, prompt_boxes, bgr, rgb, norm_full,
                         model, device, segmentor, pool, tsim=None):
    """Whichever frame path the config asks for, then cross-object NMS.

    `relative_assignment_enabled` false => byte-identical to the previous
    recognize_frame + apply_cross_object_nms pair, so rollback is a config edit.
    """
    objs = [o for g in groups.values() for o in g]
    if objs and bool(objs[0].get("relative_assignment_enabled", False)):
        res = assign_frame_relative(objs, prompt_boxes, bgr, rgb, norm_full,
                                    model, device, segmentor, pool, tsim)
    else:
        res = recognize_frame(groups, prompt_boxes, bgr, rgb, norm_full,
                              model, device, segmentor, pool)
    return apply_cross_object_nms(objs, res)


def _iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / union if union > 0 else 0.0


def recognize_multi(o, cand, bgr, rgb, model, device, segmentor, pool,
                    norm_full=None, nms_iou=0.5):
    """Like recognize(), but returns EVERY proposal that passes both gates.

    recognize() keeps a single slot per object per frame (argmax semantic), so
    when a real object and a look-alike are proposed together only one survives
    — the loser becomes a false negative even though its gates would have
    passed. The 2026-07-15 audit traced FP/FN pairs to exactly this (Rabbit:
    0 FP but 258 FN — its slot was taken by Bear/Dinosaur, never the reverse).

    Every proposal is scored through the SAME thresholds recognize() uses; the
    only change is that survivors are all returned instead of just the best.
    Setting the result back to a 1-element list reproduces recognize().

    `best_yolo` here is the proposal's OWN confidence (recognize() reports the
    per-frame max regardless of which box won), so downstream NMS ranks the
    actual box rather than a frame-level statistic.
    """
    sel = [(b, s) for (b, s) in cand if s >= float(o.get("score_threshold", 0.0))]
    sel = sel[:int(o.get("top_k", 3))]
    if not sel:
        return []
    if norm_full is None:
        norm_full = yi.normalize_rgb(rgb)

    LAST = model_last_block(model)
    passing = []
    for rank, (box, sc) in enumerate(sel, start=1):
        crop = yi.crop_resize_pad(norm_full, box)
        if crop is None:
            continue
        cls_b, pb = dinov2_blocks_forward(model, [crop], device, o["_need_blocks"])
        cls = cls_b[0]
        sem = yi.semantic_score(cls, o["tcls"], o["match_topk"])
        if sem < o["similarity_threshold"]:
            continue
        passing.append((sem, box, sc, rank, cls, {b: pb[b][0] for b in pb}))

    out = []
    for sem, box, sc, rank, cls, patch in passing:
        res = {"num_proposals": len(sel), "best_yolo": sc, "best_sem": sem,
               "appe": 0.0, "masked_appe": 0.0, "rank_appe": 0.0, "mask_area": 0,
               "box": box, "mask": None, "rank": rank, "decision": "detected",
               "accepted": True}
        qblocks = {b: v.cpu() for b, v in patch.items()}
        if not o.get("use_mask", True):
            res["appe"] = yi.appearance_score(qblocks[LAST], o["tappe"], o["match_topk"])
            out.append(res)
            continue
        x1, y1, x2, y2 = box
        mask = yi.segment_box(segmentor, bgr, box, device)
        if mask is None:
            continue
        res["mask"] = mask
        res["mask_area"] = int(mask[y1:y2, x1:x2].sum())
        best_t = int(torch.argmax(o["tcls"] @ cls))
        res["masked_appe"] = masked_appe_blocks(qblocks, mask, box, pool, o, best_t)
        res["rank_appe"] = res["masked_appe"] if o["_rank_blocks"] == o["_blocks"] else \
            masked_appe_blocks(qblocks, mask, box, pool, o, best_t, o["_rank_blocks"])
        if res["masked_appe"] >= _appe_gate_of(o):
            out.append(res)

    # within-object NMS: top_k proposals often overlap on one physical object
    out.sort(key=lambda r: r["masked_appe"] or r["appe"], reverse=True)
    kept = []
    for r in out:
        if any(_iou_xyxy(r["box"], k["box"]) > nms_iou for k in kept):
            continue
        kept.append(r)
    return kept


def apply_cross_object_nms(objs, results):
    """One physical box accepted by SEVERAL class labels -> keep only the best one.

    Phase 3, DEFAULT OFF (no-op). Human re-audit of all 166 FP (2026-07-23) found
    92 of them were a second label pinned to a box another class had already won
    (Sauce == Sikhye 22, Sauce == saffron 15, ...). Those are a ranking failure,
    not a detection failure: the box is right, the label is a duplicate.

    Winner = rank_appe (nms_rank_blocks = [2,9]), the score already reserved for
    exactly this relative question; falls back to masked_appe when unset. The rule
    is object-agnostic -- no class name, no per-object constant (INV-01..03).
    """
    if not objs or not bool(objs[0].get("cross_object_nms_enabled", False)):
        return results
    thr = float(objs[0].get("cross_object_nms_iou", 0.9))
    live = [(o["name"], results[o["name"]]) for o in objs
            if results.get(o["name"], {}).get("accepted") and results[o["name"]].get("box")]
    live.sort(key=lambda t: -(t[1].get("rank_appe") or t[1].get("masked_appe") or 0.0))
    kept = []
    for nm, r in live:
        loser = next((k for k, kr in kept if _iou_xyxy(r["box"], kr["box"]) >= thr), None)
        if loser is None:
            kept.append((nm, r))
        else:
            r["accepted"] = False
            r["decision"] = "no-object(cross-object-nms)"
            r["cross_object_nms_loser_to"] = loser
    return results


def recognize_frame(groups, prompt_boxes, bgr, rgb, norm_full,
                    model, device, segmentor, pool):
    """2-pass batched frame processing (opt2; probe-verified: decisions and
    scores bit-identical to per-object recognize(), ~2x faster end-to-end).

    Same gates/thresholds as recognize(), restructured so the expensive
    models run ONCE PER FRAME instead of once per object/proposal:
      pass 1  collect every object's proposals -> ONE batched DINOv2 forward
              -> per-object semantic gate
      mask    ONE MobileSAM call with ALL passing bboxes (batched masks
              verified IoU=1.0 vs per-box calls)
      pass 2  per-object masked-appearance gate (winner patch transferred
              to CPU exactly once)
    Returns {object_name: res} with the same res semantics as recognize().
    """
    states = []   # one per object, in groups iteration order
    entries = []  # (state, box, conf, rank, crop) across ALL objects
    for pi, objs_here in groups.items():
        cand = sorted(prompt_boxes.get(pi, []), key=lambda t: t[1], reverse=True)
        for o in objs_here:
            thr = float(o.get("score_threshold", 0.0))
            passed = [(b, s) for (b, s) in cand if s >= thr]
            sel = passed[:int(o.get("top_k", 3))]
            res = {"num_proposals": len(sel),
                   "best_yolo": max((s for _, s in sel), default=0.0),
                   "best_sem": 0.0, "appe": 0.0, "masked_appe": 0.0,
                   "rank_appe": 0.0, "mask_area": 0, "box": None, "mask": None,
                   "rank": 0,
                   "configured_score_threshold": thr,
                   "candidate_count_before_score_threshold": len(cand),
                   "candidate_count_after_score_threshold": len(passed),
                   "decision": "no-object(no-proposal)",
                   "accepted": False}
            st = {"o": o, "res": res, "best": None}
            states.append(st)
            for rank, (box, sc) in enumerate(sel, start=1):
                crop = yi.crop_resize_pad(norm_full, box)
                if crop is None:
                    continue
                entries.append((st, box, sc, rank, crop))

    # ---- ONE DINOv2 pass for the whole frame's crops (all needed blocks) ----
    patch_all = None
    if entries:
        want = sorted({b for st in states for b in st["o"]["_need_blocks"]})
        cls_all, patch_all = dinov2_blocks_forward(
            model, [e[4] for e in entries], device, want)
        for i, (st, box, sc, rank, crop) in enumerate(entries):
            sem = yi.semantic_score(cls_all[i], st["o"]["tcls"],
                                    st["o"]["match_topk"])
            if st["best"] is None or sem > st["best"][0]:
                st["best"] = (sem, box, sc, rank, crop, cls_all[i], i)

    # ---- semantic gate (identical thresholds/ordering) ----
    passing = []
    for st in states:
        o, res, best = st["o"], st["res"], st["best"]
        if res["num_proposals"] == 0:
            continue
        res["best_sem"] = best[0] if best else 0.0
        if not (best and res["best_sem"] >= o["similarity_threshold"]):
            res["decision"] = "no-object(below-sim)"
            continue
        res["box"], res["rank"] = best[1], best[3]
        passing.append(st)

    # ---- ONE MobileSAM call for ALL passing bboxes ----
    mask_states = [st for st in passing if st["o"].get("use_mask", True)]
    masks = [None] * len(mask_states)
    if mask_states:
        masks = yi.segment_boxes(
            segmentor, bgr, [st["best"][1] for st in mask_states], device)
    mask_of = {id(st): m for st, m in zip(mask_states, masks)}

    # ---- pass 2: appearance gate per object ----
    LAST = model_last_block(model)
    for st in passing:
        o, res, best = st["o"], st["res"], st["best"]
        # winner-only device->host copy, per block this object needs
        qblocks = {b: patch_all[b][best[6]].cpu() for b in o["_need_blocks"]}
        if not o.get("use_mask", True):
            res["appe"] = yi.appearance_score(patch_all[LAST][best[6]].cpu(),
                                              o["tappe"], o["match_topk"])
            res["decision"], res["accepted"] = "detected", True
            _apply_hsv_gate(o, res, bgr)           # HSV gate (active reject if enabled; else shadow)
            _apply_tf_gate(o, res)                       # Phase 2.2 training-free rescue (OFF=no-op)
            continue
        mask = mask_of[id(st)]
        x1, y1, x2, y2 = best[1]
        res["mask"] = mask
        res["mask_area"] = int(mask[y1:y2, x1:x2].sum()) if mask is not None else 0
        best_t = int(torch.argmax(o["tcls"] @ best[5]))
        res["masked_appe"] = masked_appe_blocks(qblocks, mask, best[1], pool, o, best_t)
        res["rank_appe"] = res["masked_appe"] if o["_rank_blocks"] == o["_blocks"] else \
            masked_appe_blocks(qblocks, mask, best[1], pool, o, best_t, o["_rank_blocks"])
        if res["masked_appe"] >= _appe_gate_of(o):
            res["decision"], res["accepted"] = "detected", True
        else:
            res["decision"] = "no-object(below-appe)"
        _apply_hsv_gate(o, res, bgr)               # HSV gate (active reject if enabled; else shadow)
        _apply_tf_gate(o, res)                       # Phase 2.2 training-free rescue (OFF=no-op)

    return {st["o"]["name"]: st["res"] for st in states}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="Config-driven multi-object YOLO-World x SAM-6D ISM recognition.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--config", default=DEFAULT_CONFIG)
    p.add_argument("--bag", default="data/ros2_bag/two_table_around")
    p.add_argument("--frames-dir", default="")
    p.add_argument("--output-root",
                   default=os.path.join(REPO_ROOT, "outputs", "yolo_ism_object_n"))
    p.add_argument("--topic", default="/camera/camera/color/image_raw")
    p.add_argument("--stride", type=int, default=0,
                   help="override config stride (0 = use config defaults.stride)")
    p.add_argument("--max-frames", type=int, default=0, help="0 = no cap")
    p.add_argument("--rebuild-features", action="store_true")
    p.add_argument("--device", default="")
    return p.parse_args()


def main():
    args = parse_args()
    defaults, objs = load_config(args.config)

    device = args.device or defaults.get("device", "cuda:0")
    device = device if torch.cuda.is_available() else "cpu"
    stride = args.stride or int(defaults.get("stride", 10))
    bag_name = os.path.basename(os.path.normpath(args.bag))
    out_dir = os.path.join(args.output_root, bag_name)
    os.makedirs(out_dir, exist_ok=True)
    combined_dir = os.path.join(out_dir, "combined")
    os.makedirs(combined_dir, exist_ok=True)

    ckpt = defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT
    print(f"[dinov2] loading {ckpt}")
    model = yi.build_dinov2(ckpt, device)
    objs = prepare_objects(objs, model, device, args.rebuild_features)
    unique_prompts, groups = build_prompt_groups(objs)

    any_mask = any(o.get("use_mask", True) for o in objs)
    segmentor = pool = None
    if any_mask:
        seg_w = _abspath(defaults.get("seg_weights", "mobile_sam.pt"))
        print(f"[seg] loading {seg_w}")
        segmentor = yi.build_segmentor(seg_w, device)
        pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    from ultralytics import YOLOWorld
    weights = defaults.get("weights", "yolov8m-worldv2.pt")
    try:
        yolo = YOLOWorld(weights)
    except Exception as e:
        print(f"[yolo] {weights} failed ({e}); falling back to yolov8s-worldv2.pt")
        yolo = YOLOWorld("yolov8s-worldv2.pt")
    yolo.set_classes(unique_prompts)
    print(f"[yolo] {len(unique_prompts)} unique prompts -> {len(objs)} objects: "
          f"{unique_prompts}")

    # single shared YOLO pass uses the LOWEST per-object threshold so no object
    # is starved; each object re-applies its own score_threshold/top_k in recognize().
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    img_sz = int(defaults.get("imgsz", 640))   # YOLO-World inference resolution (config-driven)
    # template (PLY render) similarity — which objects the shape feature cannot tell
    # apart. Computed once; the colour tie-break reads it instead of a class list.
    tsim = template_similarity(objs)

    per_obj_rows = {o["name"]: [] for o in objs}
    per_obj_count = {o["name"]: 0 for o in objs}
    summary_rows = []
    n_frames = 0

    for frame_idx, name, bgr in yi.resolve_frames(
            args.bag, args.frames_dir, max(1, stride), args.max_frames, args.topic):
        n_frames += 1
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)   # hoisted: shared by all objects

        # Phase 4: with multi_label a box keeps EVERY prompt above conf, not just
        # its top-1 — the deployed top-1 rule was deleting correctly-placed boxes.
        with multi_label_nms(_multi_label_on(objs[0])):
            res = yolo.predict(bgr, conf=min_score, imgsz=img_sz, verbose=False,
                               device=device)
        # boxes grouped by predicted prompt index
        prompt_boxes = {i: [] for i in range(len(unique_prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                cls_i = int(b.cls[j]) if b.cls is not None else 0
                if cls_i in prompt_boxes:   # ignore any out-of-range class id
                    prompt_boxes[cls_i].append(([x1, y1, x2, y2], float(b.conf[j])))

        combined = bgr.copy()
        accepted_names = []
        # opt2: one batched 2-pass call replaces the per-object recognize loop
        # (probe-verified: identical decisions/scores, ~2x faster end-to-end)
        results = recognize_frame_auto(groups, prompt_boxes, bgr, rgb, norm_full,
                                       model, device, segmentor, pool, tsim)
        for pi, objs_here in groups.items():
            for o in objs_here:
                r = results[o["name"]]
                out_path = ""
                if r["accepted"]:
                    per_obj_count[o["name"]] += 1
                    accepted_names.append(o["name"])
                    obj_dir = os.path.join(out_dir, o["name"])
                    os.makedirs(obj_dir, exist_ok=True)
                    out_path = os.path.join(obj_dir, f"frame_{frame_idx:06d}.jpg")
                    ov = yi.draw_result(bgr, r["box"], r["rank"], r["best_yolo"],
                                        r["best_sem"], r["masked_appe"] or r["appe"])
                    if r["mask"] is not None:
                        cnts, _ = cv2.findContours(r["mask"].astype("uint8"),
                                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(ov, cnts, -1, (0, 200, 0), 2)
                    cv2.imwrite(out_path, ov)
                    # annotate object name onto the combined overlay
                    bx = r["box"]
                    cv2.rectangle(combined, (bx[0], bx[1]), (bx[2], bx[3]), (0, 200, 0), 2)
                    cv2.putText(combined, o["name"], (bx[0] + 2, max(12, bx[1] - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
                per_obj_rows[o["name"]].append(
                    [frame_idx, name, r["num_proposals"], round(r["best_yolo"], 4),
                     round(r["best_sem"], 4), round(r["appe"], 4),
                     ";".join(str(v) for v in r["box"]) if r["box"] else "",
                     r["decision"], out_path, round(r["masked_appe"], 4), r["mask_area"],
                     r.get("configured_score_threshold", ""),
                     r.get("candidate_count_before_score_threshold", ""),
                     r.get("candidate_count_after_score_threshold", ""),
                     r.get("hsv_score", ""), r.get("hsv_threshold", ""),
                     r.get("hsv_pass", ""), r.get("would_hsv_reject", ""),
                     r.get("hsv_authority_version", ""),
                     r.get("hsv_template_cache_version", "")])
        cv2.imwrite(os.path.join(combined_dir, f"frame_{frame_idx:06d}.jpg"), combined)
        summary_rows.append([frame_idx, name, len(accepted_names),
                             ";".join(accepted_names)])

    # write per-object CSVs
    for o in objs:
        with open(os.path.join(out_dir, f"{o['name']}_results.csv"), "w", newline="") as f:
            wcsv = csv.writer(f); wcsv.writerow(CSV_FIELDS)
            wcsv.writerows(per_obj_rows[o["name"]])
    # combined summary
    with open(os.path.join(out_dir, "multiobject_summary.csv"), "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["frame_idx", "timestamp", "num_accepted", "accepted_objects"])
        wcsv.writerows(summary_rows)

    print("\n===== SUMMARY =====")
    print(f"bag                 : {bag_name}")
    print(f"frames processed    : {n_frames}")
    print(f"objects (enabled)   : {len(objs)}")
    print(f"{'object':22s} {'accepted':>8s} / frames")
    for o in objs:
        print(f"{o['name']:22s} {per_obj_count[o['name']]:>8d} / {n_frames}")
    total_acc = sum(per_obj_count.values())
    print(f"total accepted (obj-frames): {total_acc}")
    print(f"out : {out_dir}")


if __name__ == "__main__":
    main()
