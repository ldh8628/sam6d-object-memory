#!/usr/bin/env python3
"""GIF: SAM_data relocalized in the SLAM map + object placement.

LEFT  (top-view 2D map, x-z plane):
  - prior SLAM map points (faint) + relocalized camera path
  - current relocalized camera position + heading (camera viewing direction)
  - object positions fused as  T_map_obj = T_map_cam(reloc) @ T_cam_obj(SAM-6D)
RIGHT (SAM input image):
  - rgb frame with the SAM-6D 6D pose drawn as a reprojected 3D bounding box
    (K @ [R|t] of the CAD extent) + object label/score

Frame timeline = the SAM-6D PEM frames (indices map to the color bag).
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sqlite3

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm
from PIL import Image

_FRAME_RE = re.compile(r"frame_(\d+)$")
_PEM_RE = re.compile(r"pem_(.+)$")


# ----------------------------- loaders -----------------------------
def load_color_timestamps(bag_path, hint="/camera/camera/color/image_raw"):
    cands = glob.glob(os.path.join(bag_path, "*.db3")) + glob.glob(os.path.join(bag_path, "*", "*.db3"))
    db3 = sorted(set(cands))[0]
    conn = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    try:
        cur = conn.cursor()
        cur.execute("select id,name from topics")
        rows = cur.fetchall()
        tid = next((i for i, n in rows if n == hint), None)
        if tid is None:
            tid = next(i for i, n in rows if ("color" in n.lower() or "rgb" in n.lower())
                       and "image" in n.lower() and "camera_info" not in n.lower())
        cur.execute("select timestamp from messages where topic_id=? order by timestamp", (tid,))
        return [r[0] / 1e9 for r in cur.fetchall()]
    finally:
        conn.close()


def quat_to_R(qx, qy, qz, qw):
    n = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])


def load_reloc_traj(path):
    ts, T = [], []
    for l in open(path):
        if l.startswith("#") or not l.strip():
            continue
        s = l.split()
        if len(s) < 8:
            continue
        t = float(s[0])
        xyz = np.array([float(s[1]), float(s[2]), float(s[3])])
        R = quat_to_R(*(float(x) for x in s[4:8]))
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = xyz
        ts.append(t); T.append(M)
    idx = np.argsort(ts)
    return np.array(ts)[idx], [T[i] for i in idx]


def load_pem_detections(pem_dir):
    """{frame_idx: [dict(name,R,t_mm,bbox,score)]}"""
    out = {}
    for fd in sorted(glob.glob(os.path.join(pem_dir, "frame_*"))):
        m = _FRAME_RE.search(os.path.basename(fd))
        if not m:
            continue
        fidx = int(m.group(1))
        dets = []
        for pd in sorted(glob.glob(os.path.join(fd, "pem_*"))):
            pm = _PEM_RE.search(os.path.basename(pd))
            if not pm:
                continue
            name = pm.group(1)
            jf = os.path.join(pd, "sam6d_results", "detection_pem.json")
            if not os.path.isfile(jf):
                continue
            items = json.load(open(jf))
            if not isinstance(items, list):
                items = [items]
            for it in items:
                if "R" not in it or "t" not in it:
                    continue
                dets.append(dict(
                    name=name,
                    R=np.array(it["R"], float),
                    t_mm=np.array(it["t"], float),
                    bbox=it.get("bbox"),
                    score=float(it.get("score", 0.0)),
                ))
        if dets:
            out[fidx] = dets
    return out


def load_map_points(pcd, max_pts=60000):
    lines = open(pcd).read().splitlines()
    ds = next(i for i, l in enumerate(lines) if l.startswith("DATA")) + 1
    pts = []
    for l in lines[ds:]:
        s = l.split()
        if len(s) >= 3:
            try:
                pts.append([float(s[0]), float(s[1]), float(s[2])])
            except ValueError:
                pass
    a = np.array(pts)
    # clip outliers (map PCD can carry a few far points)
    lo, hi = np.percentile(a, [1, 99], axis=0)
    a = a[np.all((a >= lo) & (a <= hi), axis=1)]
    if len(a) > max_pts:
        a = a[np.linspace(0, len(a) - 1, max_pts).astype(int)]
    return a


def cad_bbox_corners_mm(cad_path, cache):
    if cad_path in cache:
        return cache[cad_path]
    import trimesh
    mesh = trimesh.load(cad_path, process=False, force="mesh")
    v = np.asarray(mesh.vertices, float)
    lo, hi = v.min(0), v.max(0)
    ext = (hi - lo).max()
    scale = 1000.0 if ext < 5.0 else 1.0  # meters->mm if needed (t is in mm)
    lo, hi = lo * scale, hi * scale
    corners = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
                        [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
                        [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
                        [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]]])
    cache[cad_path] = corners
    return corners


def load_manifest_cadpaths(manifest):
    m = {}
    if not manifest or not os.path.isfile(manifest):
        return m
    import csv
    for row in csv.DictReader(open(manifest)):
        m.setdefault(row["object"], row["cad_path"])
    return m


# ----------------------------- drawing -----------------------------
_BOX_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
              (0, 4), (1, 5), (2, 6), (3, 7)]


def draw_pose_box(img, K, R, t_mm, corners_mm, color, label):
    P = (R @ corners_mm.T).T + t_mm  # camera frame, mm
    z = P[:, 2]
    if not np.all(np.isfinite(P)) or np.min(z) <= 50.0:  # 5cm; behind/too-close -> skip
        return False
    u = K[0, 0] * P[:, 0] / z + K[0, 2]
    v = K[1, 1] * P[:, 1] / z + K[1, 2]
    if not (np.all(np.isfinite(u)) and np.all(np.isfinite(v))):
        return False
    h, w = img.shape[:2]
    lim = 5000  # clip to keep cv2 happy while allowing slightly off-frame boxes
    pts = [(int(np.clip(uu, -lim, lim)), int(np.clip(vv, -lim, lim))) for uu, vv in zip(u, v)]
    for a, b in _BOX_EDGES:
        cv2.line(img, pts[a], pts[b], color, 2, cv2.LINE_AA)
    lx = int(np.clip(min(p[0] for p in pts), 0, w - 1))
    ly = int(np.clip(min(p[1] for p in pts) - 6, 12, h - 1))
    cv2.putText(img, label, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reloc-traj", required=True)
    ap.add_argument("--pem-dir", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--map-pcd", required=True)
    ap.add_argument("--manifest", default="")
    ap.add_argument("--out", default="", help="GIF path (GIF mode)")
    ap.add_argument("--png-dir", default="",
                    help="if set: write one PNG per frame here (per-frame mode, "
                         "no object accumulation, all frames)")
    ap.add_argument("--pose-tol", type=float, default=0.25)
    ap.add_argument("--max-frames", type=int, default=40,
                    help="GIF mode only: subsample PEM frames to at most this many")
    ap.add_argument("--accumulate", dest="accumulate", action="store_true", default=None,
                    help="carry objects across frames (default: on for GIF, off for PNG)")
    ap.add_argument("--no-accumulate", dest="accumulate", action="store_false")
    args = ap.parse_args()
    if not args.out and not args.png_dir:
        ap.error("need --out (GIF) or --png-dir (per-frame PNG)")
    png_mode = bool(args.png_dir)
    accumulate = args.accumulate if args.accumulate is not None else (not png_mode)

    ts_traj, T_traj = load_reloc_traj(args.reloc_traj)
    color_ts = load_color_timestamps(args.bag)
    dets_by_frame = load_pem_detections(args.pem_dir)
    map_pts = load_map_points(args.map_pcd)
    cad_paths = load_manifest_cadpaths(args.manifest)
    cad_cache = {}

    # camera intrinsics from the first frame's camera.json
    cam0 = json.load(open(glob.glob(os.path.join(args.pem_dir, "frame_*", "camera.json"))[0]))
    K = np.array(cam0["cam_K"], float).reshape(3, 3)

    # all PEM frames (with rgb) drive the timeline -> smoother camera motion;
    # detection frames additionally place/refresh objects.
    frames = sorted(int(_FRAME_RE.search(os.path.basename(fd)).group(1))
                    for fd in glob.glob(os.path.join(args.pem_dir, "frame_*"))
                    if _FRAME_RE.search(os.path.basename(fd))
                    and os.path.isfile(os.path.join(fd, "rgb.png")))
    if not png_mode and args.max_frames and len(frames) > args.max_frames:
        # prefer detection frames; subsample to hit the budget (detection frames
        # too if they alone exceed it, so big runs don't blow up the GIF size)
        det_f = [f for f in frames if f in dets_by_frame]
        other = [f for f in frames if f not in dets_by_frame]
        if len(det_f) >= args.max_frames:
            det_f = [det_f[i] for i in np.linspace(0, len(det_f)-1, args.max_frames).astype(int)]
            other = []
        else:
            keep = args.max_frames - len(det_f)
            other = [other[i] for i in np.linspace(0, len(other)-1, keep).astype(int)] if (keep and other) else []
        frames = sorted(set(det_f) | set(other))
    print(f"reloc poses={len(ts_traj)} color_ts={len(color_ts)} "
          f"pem_frames={len(frames)} (with_dets={len(dets_by_frame)})")

    # stable color per object name
    names = sorted({d["name"] for v in dets_by_frame.values() for d in v})
    palette = {n: (np.array(cm.tab20(i % 20)[:3]) * 255) for i, n in enumerate(names)}

    path_xz = np.array([[T[0, 3], T[2, 3]] for T in T_traj])
    xlim = (path_xz[:, 0].min() - 0.6, path_xz[:, 0].max() + 0.6)
    zlim = (path_xz[:, 1].min() - 0.6, path_xz[:, 1].max() + 0.6)

    seen = {}   # name -> list of (x,z)
    pil_frames = []
    n_written = 0

    for fidx in frames:
        if not (0 <= fidx < len(color_ts)):
            continue
        ts = color_ts[fidx]
        k = int(np.searchsorted(ts_traj, ts))
        k = min(max(k, 0), len(ts_traj) - 1)
        if k > 0 and abs(ts_traj[k-1]-ts) < abs(ts_traj[k]-ts):
            k -= 1
        if abs(ts_traj[k] - ts) > args.pose_tol:
            continue
        Tmc = T_traj[k]
        cam_xz = np.array([Tmc[0, 3], Tmc[2, 3]])
        head = Tmc[:3, :3] @ np.array([0, 0, 1.0])   # camera +Z in map
        head_xz = np.array([head[0], head[2]])
        nh = np.linalg.norm(head_xz)
        head_xz = head_xz / nh * 0.5 if nh > 1e-6 else head_xz

        frame_dets = dets_by_frame.get(fidx, [])
        cur_objs = []
        for d in frame_dets:
            Tco = np.eye(4); Tco[:3, :3] = d["R"]; Tco[:3, 3] = d["t_mm"] / 1000.0
            Tmo = Tmc @ Tco
            ox, oz = Tmo[0, 3], Tmo[2, 3]
            if accumulate:
                seen.setdefault(d["name"], []).append((ox, oz))
            cur_objs.append((d["name"], ox, oz))

        # what to render on the map: accumulated history, or only this frame
        if accumulate:
            disp = seen
        else:
            disp = {}
            for nm, ox, oz in cur_objs:
                disp.setdefault(nm, []).append((ox, oz))

        # ---- right panel image ----
        rgb = cv2.imread(os.path.join(args.pem_dir, f"frame_{fidx:06d}", "rgb.png"))
        if rgb is None:
            rgb = np.full((480, 640, 3), 40, np.uint8)
        img = rgb.copy()
        for d in frame_dets:
            col = tuple(int(c) for c in palette[d["name"]][::-1])  # BGR
            cp = cad_paths.get(d["name"])
            lbl = f"{d['name']} {d['score']:.2f}"
            drew_box = False
            if cp and os.path.isfile(cp):
                corners = cad_bbox_corners_mm(cp, cad_cache)
                try:
                    drew_box = bool(draw_pose_box(img, K, d["R"], d["t_mm"], corners, col, lbl))
                except Exception:
                    drew_box = False
            if not drew_box and d["bbox"]:
                x, y, w, h = d["bbox"]
                cv2.rectangle(img, (int(x), int(y)), (int(x+w), int(y+h)), col, 2)
                cv2.putText(img, lbl, (int(x), int(y)-6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # ---- compose figure ----
        fig = plt.figure(figsize=(13, 5.2))
        gs = fig.add_gridspec(1, 2, width_ratios=[1.05, 1.0], wspace=0.18)
        axL = fig.add_subplot(gs[0]); axR = fig.add_subplot(gs[1])

        axL.scatter(map_pts[:, 0], map_pts[:, 2], s=0.4, c="#cccccc", rasterized=True)
        axL.plot(path_xz[:, 0], path_xz[:, 1], "-", color="#9ecae1", lw=1.0, alpha=0.7)
        axL.plot(path_xz[:k+1, 0], path_xz[:k+1, 1], "-", color="#1f77b4", lw=1.6)
        for nm, plist in disp.items():
            p = np.array(plist); mx, mz = p[:, 0].mean(), p[:, 1].mean()
            col = palette[nm] / 255.0
            axL.scatter(mx, mz, s=70, color=col, edgecolor="k", lw=0.6, zorder=5)
        for nm, ox, oz in cur_objs:
            axL.scatter(ox, oz, s=230, marker="*", color=palette[nm]/255.0,
                        edgecolor="k", lw=0.8, zorder=6)
        axL.scatter(*cam_xz, s=120, marker="o", color="red", edgecolor="k", zorder=7)
        axL.arrow(cam_xz[0], cam_xz[1], head_xz[0], head_xz[1], width=0.02,
                  head_width=0.12, head_length=0.12, fc="red", ec="red", zorder=7,
                  length_includes_head=True)
        # legend of objects currently shown
        if disp:
            from matplotlib.lines import Line2D
            handles = [Line2D([0], [0], marker="o", ls="", markersize=7,
                              markerfacecolor=palette[nm]/255.0, markeredgecolor="k",
                              label=nm) for nm in sorted(disp)]
            axL.legend(handles=handles, loc="upper left", fontsize=6.5,
                       framealpha=0.85, title="objects", title_fontsize=7)
        axL.set_xlim(*xlim); axL.set_ylim(*zlim); axL.set_aspect("equal", adjustable="box")
        axL.set_xlabel("x [m]"); axL.set_ylabel("z [m]")
        mode_tag = "this-frame objects" if not accumulate else "accumulated objects"
        axL.set_title(f"2D map | reloc cam pose + {mode_tag}  (frame {fidx}, t={ts-ts_traj[0]:.1f}s)")

        axR.imshow(img_rgb)
        axR.set_xticks([]); axR.set_yticks([])
        det_names = ", ".join(f"{d['name']}({d['score']:.2f})" for d in frame_dets)
        axR.set_title(f"SAM input + 6D pose | {det_names or 'no detection'}", fontsize=9)

        fig.tight_layout()
        fig.canvas.draw()
        buf = np.frombuffer(fig.canvas.buffer_rgba(), np.uint8).reshape(
            fig.canvas.get_width_height()[::-1] + (4,))
        frame_img = Image.fromarray(buf[..., :3].copy())
        plt.close(fig)
        if png_mode:
            os.makedirs(args.png_dir, exist_ok=True)
            tag = "det" if frame_dets else "nodet"
            frame_img.save(os.path.join(args.png_dir, f"frame_{fidx:06d}_{tag}.png"))
            n_written += 1
        else:
            pil_frames.append(frame_img)

    if png_mode:
        if n_written == 0:
            raise SystemExit("no frames rendered")
        print(f"wrote {n_written} per-frame PNGs to {args.png_dir}")
        return
    if not pil_frames:
        raise SystemExit("no frames rendered")
    pil_frames[0].save(args.out, save_all=True, append_images=pil_frames[1:],
                       duration=700, loop=0, disposal=2)
    print(f"wrote {args.out}  ({len(pil_frames)} frames)")


if __name__ == "__main__":
    main()
