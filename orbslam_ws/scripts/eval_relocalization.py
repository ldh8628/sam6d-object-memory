#!/usr/bin/env python3
"""Evaluate ORB-SLAM3 relocalization of a query sequence against a prior map.

First step of the SAM_data -> 105023-map localization study:
  * how many query frames obtained a valid pose (tracked fraction),
  * time-to-first-fix,
  * top-view overlay of the query trajectory on the prior map's points.

Usage:
  eval_relocalization.py --reloc-dir <reloc output dir> \
                         --map-dir <mapping output dir> \
                         --n-query-frames 2004 \
                         --out <png path>
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_tum(path: Path):
    ts, xyz = [], []
    if not path.exists():
        return np.array([]), np.zeros((0, 3))
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        p = line.split()
        if len(p) < 8:
            continue
        ts.append(float(p[0]))
        xyz.append([float(p[1]), float(p[2]), float(p[3])])
    return np.array(ts), np.array(xyz).reshape(-1, 3)


def read_pcd_xyz(path: Path, max_pts: int = 400000):
    if not path.exists():
        return np.zeros((0, 3))
    lines = path.read_text().splitlines()
    data_start = None
    for i, l in enumerate(lines):
        if l.startswith("DATA"):
            data_start = i + 1
            break
    if data_start is None:
        return np.zeros((0, 3))
    pts = []
    for l in lines[data_start:]:
        p = l.split()
        if len(p) < 3:
            continue
        try:
            pts.append([float(p[0]), float(p[1]), float(p[2])])
        except ValueError:
            continue
    arr = np.array(pts).reshape(-1, 3)
    if len(arr) > max_pts:
        idx = np.linspace(0, len(arr) - 1, max_pts).astype(int)
        arr = arr[idx]
    return arr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reloc-dir", required=True)
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--n-query-frames", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="Relocalization: SAM_data/105018 localized in SLAM_data/105023 map")
    ap.add_argument("--map-label", default="105023 prior map")
    ap.add_argument("--query-label", default="105018 relocalized trajectory")
    args = ap.parse_args()

    reloc = Path(args.reloc_dir)
    mapd = Path(args.map_dir)

    ts_opt, xyz_opt = read_tum(reloc / "CameraTrajectory.txt")
    ts_live, xyz_live = read_tum(reloc / "CameraTrajectory_live.txt")

    n_tracked = len(ts_live) if len(ts_live) else len(ts_opt)
    print(f"[reloc] optimized poses: {len(ts_opt)}")
    print(f"[reloc] live (front-end) poses: {len(ts_live)}")
    if args.n_query_frames:
        frac = 100.0 * n_tracked / args.n_query_frames
        print(f"[reloc] tracked fraction: {n_tracked}/{args.n_query_frames} = {frac:.1f}%")
    if len(ts_live) >= 2:
        span = ts_live.max() - ts_live.min()
        print(f"[reloc] tracked time span: {span:.1f}s "
              f"(first fix ts={ts_live.min():.3f})")

    # prior map points: prefer sparse map points, fall back to dense
    map_pts = read_pcd_xyz(mapd / "orbslam3_map_points.pcd")
    if len(map_pts) == 0:
        map_pts = read_pcd_xyz(mapd / "orbslam3_dense_map.pcd")
    ts_map, xyz_map = read_tum(mapd / "CameraTrajectory.txt")
    print(f"[map] prior map points: {len(map_pts)}; map trajectory poses: {len(xyz_map)}")

    fig, ax = plt.subplots(figsize=(9, 8))
    if len(map_pts):
        ax.scatter(map_pts[:, 0], map_pts[:, 2], s=0.5, c="#bbbbbb",
                   label=args.map_label, rasterized=True)
    if len(xyz_map):
        ax.plot(xyz_map[:, 0], xyz_map[:, 2], "-", color="#1f77b4",
                lw=1.5, label="map trajectory")
    query_xyz = xyz_opt if len(xyz_opt) else xyz_live
    if len(query_xyz):
        ax.plot(query_xyz[:, 0], query_xyz[:, 2], "-", color="#d62728",
                lw=1.8, label=args.query_label)
        ax.scatter(query_xyz[0, 0], query_xyz[0, 2], c="green", s=60,
                   zorder=5, label="query start")
    # Zoom to the trajectory region (map point clouds can carry far outliers that
    # otherwise blow up the scale). Frame on the union of both trajectories.
    focus = []
    if len(xyz_map):
        focus.append(xyz_map[:, [0, 2]])
    if len(query_xyz):
        focus.append(query_xyz[:, [0, 2]])
    if focus:
        f = np.vstack(focus)
        xmin, xmax = f[:, 0].min(), f[:, 0].max()
        zmin, zmax = f[:, 1].min(), f[:, 1].max()
        mx = 0.15 * max(xmax - xmin, zmax - zmin, 1.0)
        ax.set_xlim(xmin - mx, xmax + mx)
        ax.set_ylim(zmin - mx, zmax + mx)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="best", fontsize=9)
    ax.set_title(args.title)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"[out] wrote {args.out}")


if __name__ == "__main__":
    main()
