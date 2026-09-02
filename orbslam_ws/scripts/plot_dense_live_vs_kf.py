#!/usr/bin/env python3
"""Top-view (X-Z) of dense maps: live accumulation vs KF re-render (optimized).

Shows the loop/BA-correction effect on the MAP: the live cloud sprays
(double/triple walls), the keyframe-reprojected cloud collapses to thin walls.
Points colored by their true RGB. Handles both PCD rgb encodings
(packed-float 'F' from the live node, uint 'U' from densify).
"""
import struct
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]


def load_pcd_rgb(path, max_pts=250000):
    if not path.exists():
        return np.empty((0, 3)), np.empty((0, 3))
    fields, types, started = [], [], False
    xyz, rgb = [], []
    for line in path.read_text(errors="replace").splitlines():
        if not started:
            t = line.split()
            if t and t[0].upper() == "FIELDS":
                fields = [s.lower() for s in t[1:]]
            elif t and t[0].upper() == "TYPE":
                types = t[1:]
            elif t and t[0].upper() == "DATA":
                started = True
                ri = fields.index("rgb") if "rgb" in fields else -1
                rtype = types[ri] if (ri >= 0 and ri < len(types)) else "U"
            continue
        p = line.split()
        if len(p) < 3:
            continue
        try:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
        except ValueError:
            continue
        xyz.append((x, y, z))
        if ri >= 0 and len(p) > ri:
            tok = p[ri]
            if rtype == "F":               # packed float bits -> uint32
                u = struct.unpack("<I", struct.pack("<f", float(tok)))[0]
            else:                           # already uint
                u = int(float(tok))
            rgb.append(((u >> 16) & 255, (u >> 8) & 255, u & 255))
        else:
            rgb.append((150, 150, 150))
    xyz = np.asarray(xyz, float)
    rgb = np.asarray(rgb, float) / 255.0
    if len(xyz) > max_pts:
        idx = np.random.default_rng(0).choice(len(xyz), max_pts, replace=False)
        xyz, rgb = xyz[idx], rgb[idx]
    return xyz, rgb


PANELS = [
    ("SLAM_three_laps  (live, pre-opt)",  ROOT/"orbslam_ws/output/LC_after/SLAM_three_laps/orbslam3_dense_map.pcd"),
    ("SLAM_three_laps  (KF re-render, opt)", ROOT/"orbslam_ws/output/LC_after/SLAM_three_laps/orbslam3_dense_kf.pcd"),
    ("three_labs_imu  (live, pre-opt)",   ROOT/"orbslam_ws/output/imu_compare/three_labs_imu/orbslam3_dense_map.pcd"),
    ("three_labs_imu  (KF re-render, opt)", ROOT/"orbslam_ws/output/imu_compare/three_labs_imu/orbslam3_dense_kf.pcd"),
]

fig, axes = plt.subplots(2, 2, figsize=(14, 13), dpi=150)
clouds = [load_pcd_rgb(p) for _, p in PANELS]
# shared limits per row (same dataset) for fair visual comparison
for row in (0, 1):
    xs = np.concatenate([clouds[2*row+c][0][:, 0] for c in (0, 1) if len(clouds[2*row+c][0])])
    zs = np.concatenate([clouds[2*row+c][0][:, 2] for c in (0, 1) if len(clouds[2*row+c][0])])
    xlim = (np.percentile(xs, 0.3), np.percentile(xs, 99.7))
    zlim = (np.percentile(zs, 0.3), np.percentile(zs, 99.7))
    for c in (0, 1):
        ax = axes[row][c]
        xyz, rgb = clouds[2*row+c]
        title, _ = PANELS[2*row+c]
        if len(xyz):
            ax.scatter(xyz[:, 0], xyz[:, 2], s=0.25, c=rgb, marker=".", linewidths=0)
        ax.set_xlim(*xlim); ax.set_ylim(*zlim)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X [m]"); ax.set_ylabel("Z [m]")
        ax.set_title(f"{title}\n(n={len(xyz):,} shown)", fontsize=10)
        ax.grid(True, ls=":", alpha=0.3)
fig.suptitle("Dense map top-view — live accumulation (sprays) vs keyframe re-render (collapses to thin walls)",
             fontsize=12)
fig.tight_layout()
out = ROOT/"orbslam_ws/output/imu_compare/dense_live_vs_kf_topview.png"
fig.savefig(out)
print("wrote", out)
for (title, _), (xyz, _) in zip(PANELS, clouds):
    print(f"  {title:42} n={len(xyz)}")
