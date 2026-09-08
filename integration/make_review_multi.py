#!/usr/bin/env python3
"""make_review_multi.py — one video per (SLAM bag, SAM bag) pair that shows, side
by side, what EACH SLAM algorithm thinks the two cameras were doing.

    +---------------------------+---------------------------+
    |  SLAM camera RGB          |  SAM camera RGB           |
    +---------------------------+---------------------------+
    |  ORB-SLAM3                |  RTAB-Map                 |
    |   2D map, both cameras,   |   2D map, both cameras,   |
    |   rig-offset deviation    |   rig-offset deviation    |
    +---------------------------+---------------------------+

Each algorithm ran independently on each camera's bag, so its two trajectories
sit in different map frames. rig_offset.py solves AX = ZB for that pair, giving
M = T_mapA_mapB; the SAM trajectory is drawn as M . T_mapB_camB(t), i.e. inside
the SLAM camera's map. The gap between the two markers is then the rig offset
the algorithm implies, and the sparkline under each map is how much that offset
wanders — flat means the algorithm is self-consistent.

    /home/ldh9501/miniconda3/envs/sam_yolo/bin/python integration/make_review_multi.py \
        --pair 105023_105018
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from make_review import (COLOR_TOPIC, C_BG, C_SAM, C_SLAM, MapView,  # noqa: E402
                         bag_color_stamps, decode_frames, nearest,
                         put_panel_label, read_pcd_xyz, read_tum)

ROOT = HERE.parent
CONV = ROOT / "data_slam_converted" / "260714"
OUT_ROOT = ROOT / "integration" / "output" / "260714_pairs"

PANEL_W, PANEL_H = 640, 480
MAP_W, MAP_H = 640, 560
TITLE_H, SPARK_H = 30, 74
BAR_H = 44
COL_H = TITLE_H + MAP_H + SPARK_H

ALGO_TRAJ = {
    "orb3": (ROOT / "output_slam/260714_pairs/slam_{s}/orbslam3_noimu/trajectory.txt",
             ROOT / "output_slam/260714_pairs/sam_{m}/orbslam3_noimu/trajectory.txt",
             ROOT / "output_slam/260714_pairs/slam_{s}/orbslam3_noimu/map_points.pcd"),
    "rtabmap": (ROOT / "output_slam/260714_pairs/slam_{s}/rtabmap/trajectory.txt",
                ROOT / "output_slam/260714_pairs/sam_{m}/rtabmap/trajectory.txt",
                ROOT / "output_slam/260714_pairs/slam_{s}/rtabmap/dense_map.pcd"),
}
ALGO_TITLE = {"orb3": "ORB-SLAM3", "rtabmap": "RTAB-Map",
              "hdl_graph_slam": "hdl_graph_slam"}


def log(m):
    print(f"[review-multi] {m}", flush=True)


def interp(ts, T, t, max_gap):
    i = int(np.searchsorted(ts, t))
    if i == 0 or i >= len(ts):
        return None
    t0, t1 = ts[i - 1], ts[i]
    if t1 - t0 > max_gap:
        return None
    u = (t - t0) / (t1 - t0)
    R0, R1 = T[i - 1][:3, :3], T[i][:3, :3]
    U, _, Vt = np.linalg.svd((1 - u) * R0 + u * R1)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = (1 - u) * T[i - 1][:3, 3] + u * T[i][:3, 3]
    return M


class AlgoPanel:
    """One algorithm's column: title, top-down map, deviation sparkline."""

    def __init__(self, algo, fitres, s, m, grid, tau):
        self.algo = algo
        self.title = ALGO_TITLE.get(algo, algo)
        self.ok = bool(fitres and fitres.get("ok"))
        self.note = "" if self.ok else (fitres or {}).get("reason", "not available")
        self.dev = np.full(len(grid), np.nan)
        self.pa = self.pb = None
        if not self.ok:
            return

        pa, pb, pts = (Path(str(p).format(s=s, m=m)) for p in ALGO_TRAJ[algo])
        self.ts_a, self.T_a = read_tum_np(pa)
        self.ts_b, self.T_b = read_tum_np(pb)
        self.M = np.array(fitres["M"])
        self.max_gap = 0.35 if algo == "orb3" else 3.0

        # both cameras expressed in the SLAM camera's map frame
        self.A = [interp(self.ts_a, self.T_a, t + tau, self.max_gap) for t in grid]
        self.B = [interp(self.ts_b, self.T_b, t, self.max_gap) for t in grid]
        self.B = [None if b is None else self.M @ b for b in self.B]

        # rig offset over time and its deviation from the session median
        tx = np.array([(np.linalg.inv(a) @ b)[:3, 3] if (a is not None and b is not None)
                       else [np.nan] * 3 for a, b in zip(self.A, self.B)])
        med = np.nanmedian(tx, axis=0)
        self.dev = np.linalg.norm(tx - med, axis=1)
        self.dev_med = float(np.nanmedian(self.dev))
        self.dev_max = float(np.nanmax(self.dev)) if np.isfinite(self.dev).any() else 0.0

        traj_a = np.array([T[:3, 3] for T in self.T_a])
        traj_b = np.array([(self.M @ T)[:3, 3] for T in self.T_b])
        self.view = MapView(MAP_W, MAP_H, read_pcd_xyz(pts, 12000), [], traj_a, traj_b,
                            legend=((C_SLAM, "SLAM camera (front)"),
                                    (C_SAM, "SAM camera (side)")))

    def render(self, k, trail_n):
        col = np.full((COL_H, MAP_W, 3), C_BG, np.uint8)
        cv2.rectangle(col, (0, 0), (MAP_W, TITLE_H), (44, 44, 44), -1)
        if not self.ok:
            cv2.putText(col, f"{self.title}: {self.note}", (12, 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1, cv2.LINE_AA)
            cv2.putText(col, "no trajectory pair for this algorithm",
                        (MAP_W // 2 - 165, TITLE_H + MAP_H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (170, 170, 170), 1, cv2.LINE_AA)
            return col
        d = self.dev[k]
        cv2.putText(col, self.title, (12, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                    (245, 245, 245), 1, cv2.LINE_AA)
        txt = "rig offset dev: --" if not np.isfinite(d) else \
              f"rig offset dev: {d * 100:5.1f} cm   (median {self.dev_med * 100:.1f})"
        cv2.putText(col, txt, (MAP_W - 330, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (120, 220, 120) if (np.isfinite(d) and d < 0.10) else (120, 190, 255),
                    1, cv2.LINE_AA)

        cams, trails = [], []
        for arr, colr, lab in ((self.A, C_SLAM, "SLAM"), (self.B, C_SAM, "SAM")):
            if arr[k] is None:
                continue
            cams.append((arr[k][:3, 3], arr[k][:3, :3], colr, lab))
            tr = [p[:3, 3] for p in arr[max(0, k - trail_n):k + 1] if p is not None]
            trails.append(np.array(tr) if tr else np.zeros((0, 3)))
        col[TITLE_H:TITLE_H + MAP_H] = self.view.frame(cams, trails)

        # sparkline of the deviation over the whole session
        y0 = TITLE_H + MAP_H
        cv2.rectangle(col, (0, y0), (MAP_W, COL_H), (250, 250, 250), -1)
        cv2.line(col, (0, y0), (MAP_W, y0), (225, 225, 225), 1)
        finite = self.dev[np.isfinite(self.dev)]
        top = max(0.10, float(np.percentile(finite, 99)) if finite.size else 0.10)
        n = len(self.dev)
        pts = [(int(i * (MAP_W - 1) / max(1, n - 1)),
                int(COL_H - 8 - (min(self.dev[i], top) / top) * (SPARK_H - 22)))
               for i in range(n) if np.isfinite(self.dev[i])]
        for i in range(1, len(pts)):
            cv2.line(col, pts[i - 1], pts[i], (90, 90, 200), 1, cv2.LINE_AA)
        cx = int(k * (MAP_W - 1) / max(1, n - 1))
        cv2.line(col, (cx, y0 + 4), (cx, COL_H - 4), (200, 80, 80), 1)
        cv2.putText(col, f"deviation of the rig offset over the session (top = {top*100:.0f} cm)",
                    (8, y0 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1,
                    cv2.LINE_AA)
        return col


def read_tum_np(path):
    ts, T = [], []
    for line in Path(path).read_text().splitlines():
        p = line.split()
        if len(p) != 8:
            continue
        q = np.array([float(v) for v in p[4:8]])
        nq = np.linalg.norm(q)
        if nq == 0 or not np.isfinite(nq):
            continue
        x, y, z, w = q / nq
        M = np.eye(4)
        M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        M[:3, 3] = [float(v) for v in p[1:4]]
        ts.append(float(p[0]))
        T.append(M)
    o = np.argsort(ts, kind="stable")
    ts, T = np.array(ts)[o], np.array(T)[o]
    if len(ts) > 1:
        keep = np.append(np.diff(ts) > 1e-6, True)
        ts, T = ts[keep], T[keep]
    return ts, T


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--pair", required=True, help="e.g. 105023_105018")
    ap.add_argument("--fit", default=str(OUT_ROOT / "rig_offset.json"))
    ap.add_argument("--algos", nargs="+", default=["orb3", "rtabmap", "hdl_graph_slam"])
    ap.add_argument("--fps", type=float, default=12.0)
    ap.add_argument("--trail", type=float, default=6.0)
    ap.add_argument("--start", type=float, default=0.0, help="seconds into the overlap")
    ap.add_argument("--duration", type=float, default=0.0, help="0 = to the end")
    ap.add_argument("--crf", type=int, default=24)
    ap.add_argument("--out-dir", default="")
    a = ap.parse_args()

    s, m = a.pair.split("_")
    fits = json.loads(Path(a.fit).read_text())["pairs"][a.pair]
    out_dir = Path(a.out_dir) if a.out_dir else OUT_ROOT / a.pair
    out_dir.mkdir(parents=True, exist_ok=True)

    slam_bag, sam_bag = CONV / f"{s}__unlabeled" / "SLAM", CONV / f"{s}__unlabeled" / "SAM"
    cs_a = bag_color_stamps(slam_bag, COLOR_TOPIC)
    cs_b = bag_color_stamps(sam_bag, COLOR_TOPIC)

    # clock offset between the two recordings: the fitted tau (they agree within
    # ~0.3 s across algorithms and match the bag start difference)
    taus = [f["tau_s"] for f in fits.values() if f.get("ok")]
    if not taus:
        raise SystemExit(f"no algorithm produced a fit for {a.pair}")
    tau = float(np.median(taus))
    log(f"{a.pair}: tau = {tau:+.2f}s (from {len(taus)} fits: "
        f"{', '.join(f'{t:+.2f}' for t in taus)})")

    t0 = max(cs_b[0], cs_a[0] - tau) + a.start
    t1 = min(cs_b[-1], cs_a[-1] - tau)
    if a.duration > 0:
        t1 = min(t1, t0 + a.duration)
    grid = np.arange(t0, t1, 1.0 / a.fps)
    log(f"{len(grid)} frames @ {a.fps} fps ({grid[-1] - grid[0]:.1f}s of overlap)")

    panels = [AlgoPanel(al, fits.get(al), s, m, grid, tau) for al in a.algos]
    for p in panels:
        log(f"  {p.title}: {'ok' if p.ok else 'SKIP — ' + p.note}"
            + (f"  dev median {p.dev_med * 100:.1f} cm" if p.ok else ""))

    idx_a = [nearest(cs_a, t + tau) for t in grid]
    idx_b = [nearest(cs_b, t) for t in grid]
    log("decoding frames ...")
    fr_a = decode_frames(slam_bag, COLOR_TOPIC, set(idx_a))
    fr_b = decode_frames(sam_bag, COLOR_TOPIC, set(idx_b))

    ncol = len(panels)
    W = max(PANEL_W * 2, MAP_W * ncol)
    H = BAR_H + PANEL_H + COL_H
    mp4 = out_dir / "sync_view.mp4"
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-f", "rawvideo",
         "-pix_fmt", "bgr24", "-s", f"{W}x{H}", "-r", f"{a.fps}", "-i", "-",
         "-c:v", "libx264", "-preset", "faster", "-crf", str(a.crf),
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)],
        stdin=subprocess.PIPE)

    trail_n = int(a.trail * a.fps)
    x_rgb = (W - PANEL_W * 2) // 2
    for k, t in enumerate(grid):
        canvas = np.full((H, W, 3), C_BG, np.uint8)
        cv2.rectangle(canvas, (0, 0), (W, BAR_H), (28, 28, 28), -1)
        cv2.putText(canvas, f"260714 pair {a.pair}   t = {t - grid[0]:6.2f}s   "
                            f"frame {k + 1}/{len(grid)}   clock offset {tau:+.2f}s",
                    (14, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 245, 245), 1,
                    cv2.LINE_AA)
        for j, (frames, iidx, colr, name, stamps) in enumerate((
                (fr_a, idx_a, C_SLAM, "SLAM camera", cs_a),
                (fr_b, idx_b, C_SAM, "SAM camera", cs_b))):
            enc = frames.get(iidx[k])
            panel = (cv2.imdecode(enc, cv2.IMREAD_COLOR) if enc is not None
                     else np.full((PANEL_H, PANEL_W, 3), 60, np.uint8))
            if panel.shape[:2] != (PANEL_H, PANEL_W):
                panel = cv2.resize(panel, (PANEL_W, PANEL_H))
            ref = t + tau if j == 0 else t
            put_panel_label(panel, f"{name}   frame {iidx[k]}   "
                                   f"dt {(stamps[iidx[k]] - ref) * 1000:+.0f} ms", colr)
            canvas[BAR_H:BAR_H + PANEL_H, x_rgb + j * PANEL_W:x_rgb + (j + 1) * PANEL_W] = panel
        for j, p in enumerate(panels):
            canvas[BAR_H + PANEL_H:, j * MAP_W:(j + 1) * MAP_W] = p.render(k, trail_n)
        proc.stdin.write(canvas.tobytes())
        if (k + 1) % 200 == 0:
            log(f"  {k + 1}/{len(grid)}")
    proc.stdin.close()
    if proc.wait() != 0:
        raise SystemExit("ffmpeg failed")
    subprocess.run(["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-i", str(mp4),
                    "-vframes", "1", "-q:v", "4", str(out_dir / "poster.jpg")], check=False)
    log(f"wrote {mp4} ({mp4.stat().st_size / 1e6:.1f} MB)")

    meta = {"pair": a.pair, "tau_s": round(tau, 3), "fps": a.fps,
            "frames": len(grid), "duration_s": round(float(grid[-1] - grid[0]), 2),
            "algos": [{"algo": p.algo, "title": p.title, "ok": p.ok,
                       "note": p.note,
                       "dev_median_cm": round(p.dev_med * 100, 2) if p.ok else None,
                       "dev_max_cm": round(p.dev_max * 100, 2) if p.ok else None,
                       "X_rot_deg": fits[p.algo].get("X_rot_deg") if p.ok else None,
                       "X_t_m": fits[p.algo].get("X_t_m") if p.ok else None}
                      for p in panels]}
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=1))
    log(f"wrote {out_dir / 'meta.json'}")


if __name__ == "__main__":
    main()
