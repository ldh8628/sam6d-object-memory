#!/usr/bin/env python3
"""Top-view (X-Z) overlay of the 3-way trajectories on SLAM_three_labs_rgbd_imu.

All three runs anchor their origin at the same first camera frame
(camera_color_optical_frame) of the same bag, so we overlay RAW poses
(no Umeyama alignment) — the honest as-estimated comparison.
"""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "orbslam_ws" / "output" / "imu_compare"

RUNS = [
    ("ORB3 IMU-RGBD", OUT / "three_labs_imu" / "trajectory.txt", "#1f77b4"),
    ("ORB3 RGB-D",     OUT / "three_labs_rgbd" / "trajectory.txt", "#2ca02c"),
    ("RTAB-Map RGB-D", ROOT / "rtabmap_ws" / "output" / "imu_compare" / "three_labs_imu" / "trajectory.txt", "#d62728"),
]


def load_xyz(path):
    xyz = []
    for line in open(path):
        p = line.split()
        if len(p) >= 8:
            xyz.append([float(p[1]), float(p[2]), float(p[3])])
    return np.asarray(xyz)


def drift_info(xyz):
    seg = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    path = seg.sum()
    drift = np.linalg.norm(xyz[-1] - xyz[0])
    return drift, drift / path * 100 if path else 0.0


data = [(n, load_xyz(p), c) for n, p, c in RUNS]

# global equal-extent limits
allx = np.concatenate([d[:, 0] for _, d, _ in data])
allz = np.concatenate([d[:, 2] for _, d, _ in data])
pad = 0.1
xlim = (allx.min() - pad, allx.max() + pad)
zlim = (allz.min() - pad, allz.max() + pad)

# ---------- 1) overlay ----------
fig, ax = plt.subplots(figsize=(8.5, 8.5), dpi=150)
for name, xyz, color in data:
    d_m, d_pct = drift_info(xyz)
    ax.plot(xyz[:, 0], xyz[:, 2], "-", c=color, lw=1.5, alpha=0.85,
            label=f"{name}  (n={len(xyz)}, drift {d_m*100:.1f} cm / {d_pct:.2f}%)")
    ax.plot(xyz[0, 0], xyz[0, 2], "o", c=color, ms=11, mec="k", mew=1.0, zorder=5)
    ax.plot(xyz[-1, 0], xyz[-1, 2], "X", c=color, ms=12, mec="k", mew=1.0, zorder=5)
ax.plot([], [], "ko", ms=9, label="start (○)")
ax.plot([], [], "kX", ms=10, label="end (✕)")
ax.set_xlim(*xlim); ax.set_ylim(*zlim)
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("X [m]"); ax.set_ylabel("Z [m]")
ax.set_title("Top-view (X-Z) overlay — SLAM_three_labs_rgbd_imu (raw, same first-frame anchor)")
ax.grid(True, ls=":", alpha=0.5)
ax.legend(loc="upper right", fontsize=8.5, framealpha=0.92)
fig.tight_layout()
fig.savefig(OUT / "threeway_top_view_overlay.png")
plt.close(fig)

# ---------- 2) side-by-side panels ----------
fig, axes = plt.subplots(1, 3, figsize=(16, 5.6), dpi=150)
for ax, (name, xyz, color) in zip(axes, data):
    d_m, d_pct = drift_info(xyz)
    ax.plot(xyz[:, 0], xyz[:, 2], "-", c=color, lw=1.4)
    ax.plot(xyz[0, 0], xyz[0, 2], "o", c="k", ms=10, zorder=5)
    ax.plot(xyz[-1, 0], xyz[-1, 2], "X", c="k", ms=11, zorder=5)
    ax.set_xlim(*xlim); ax.set_ylim(*zlim)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Z [m]")
    ax.set_title(f"{name}\nn={len(xyz)}, drift {d_m*100:.1f} cm / {d_pct:.2f}%", fontsize=10)
    ax.grid(True, ls=":", alpha=0.5)
fig.suptitle("Top-view (X-Z) per method — SLAM_three_labs_rgbd_imu", fontsize=12)
fig.tight_layout()
fig.savefig(OUT / "threeway_top_view_panels.png")
plt.close(fig)

print("wrote:")
print(" ", OUT / "threeway_top_view_overlay.png")
print(" ", OUT / "threeway_top_view_panels.png")
for name, xyz, _ in data:
    d_m, d_pct = drift_info(xyz)
    print(f"  {name:16} n={len(xyz):5d}  X[{xyz[:,0].min():.2f},{xyz[:,0].max():.2f}]  "
          f"Z[{xyz[:,2].min():.2f},{xyz[:,2].max():.2f}]  drift {d_m*100:.1f}cm/{d_pct:.2f}%")
