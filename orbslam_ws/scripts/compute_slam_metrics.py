#!/usr/bin/env python3
"""GT-free SLAM metrics for the IMU comparison.

Per run computes:
  - Tracking success rate = tracked poses / RGB(color) frames in the bag
  - Tracking gaps / jumps (time/space heuristics, reuse loc_eval.metrics)
  - GT-free RPE proxy = loop-drift (return-to-origin):
        drift_m   = ||p_end - p_start||
        drift_pct = drift_m / path_length * 100
        plus per-step RPE consistency (median frame-to-frame translation)
  - Mean Map Entropy (MME) over the dense point cloud (lower = sharper/consistent)

Run inside any env with numpy + scipy. TUM trajectory + ascii PCD inputs.
"""
from __future__ import annotations

import json
import sqlite3
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parents[1] / "slam_comparison_report" / "loc_eval"))
from metrics import detect_tracking_issues  # noqa: E402

try:
    from scipy.spatial import cKDTree
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False


def load_tum(path: Path):
    t, xyz = [], []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            t.append(float(p[0]))
            xyz.append([float(p[1]), float(p[2]), float(p[3])])
    return np.asarray(t), np.asarray(xyz)


def count_color_frames(bag_dir: Path) -> int:
    db = next(bag_dir.glob("*.db3"), None)
    if db is None:
        return 0
    con = sqlite3.connect(str(db))
    try:
        tid = con.execute("select id from topics where name=?",
                          ("/camera/camera/color/image_raw",)).fetchone()
        if tid:
            return int(con.execute("select count(*) from messages where topic_id=?",
                                   (tid[0],)).fetchone()[0])
        return 0
    finally:
        con.close()


def load_pcd_xyz(path: Path, max_points: int = 200000) -> np.ndarray:
    if not path.exists():
        return np.empty((0, 3))
    pts = []
    started = False
    with path.open(errors="replace") as f:
        for line in f:
            if not started:
                if line.startswith("DATA"):
                    started = True
                continue
            p = line.split()
            if len(p) < 3:
                continue
            try:
                pts.append([float(p[0]), float(p[1]), float(p[2])])
            except ValueError:
                continue
    a = np.asarray(pts)
    if len(a) > max_points:
        idx = np.linspace(0, len(a) - 1, max_points).astype(int)
        a = a[idx]
    return a


def mean_map_entropy(xyz: np.ndarray, radius: float = 0.2, min_neighbors: int = 6):
    """MME (Razlaw et al. 'Look Ma, No Ground Truth'): mean per-point
    differential entropy of the local neighbourhood Gaussian. Lower = sharper."""
    if not HAVE_SCIPY or len(xyz) < min_neighbors + 1:
        return {"mme": None, "n_points_used": 0, "radius_m": radius, "note": "scipy/insufficient"}
    tree = cKDTree(xyz)
    const = 0.5 * np.log((2 * np.pi * np.e) ** 3)
    ents, used = [], 0
    # sample up to 30k query points for speed
    q_idx = np.arange(len(xyz))
    if len(q_idx) > 30000:
        q_idx = np.linspace(0, len(xyz) - 1, 30000).astype(int)
    for i in q_idx:
        nb = tree.query_ball_point(xyz[i], radius)
        if len(nb) < min_neighbors:
            continue
        cov = np.cov(xyz[nb].T)
        det = np.linalg.det(cov)
        if det <= 1e-18:
            continue
        ents.append(const + 0.5 * np.log(det))
        used += 1
    if not ents:
        return {"mme": None, "n_points_used": 0, "radius_m": radius}
    return {"mme": float(np.mean(ents)), "mme_median": float(np.median(ents)),
            "n_points_used": used, "radius_m": radius}


def parse_timing(out_dir: Path):
    """Parse the node's TIMING line: frames fed to the tracker + per-frame cost,
    and whether IMU initialised (VIBA stages present)."""
    log = out_dir / "orbslam3_launch.log"
    info = {"frames_fed": None, "track_ms_mean": None, "track_fps": None,
            "imu_initialised": None}
    if not log.exists():
        return info
    text = log.read_text(errors="replace")
    import re
    m = re.search(r"frames_tracked=(\d+).*?mean=([\d.]+).*?mean_fps=([\d.]+)", text)
    if m:
        info["frames_fed"] = int(m.group(1))
        info["track_ms_mean"] = float(m.group(2))
        info["track_fps"] = float(m.group(3))
    if "subscribed IMU topic" in text:  # IMU mode was active
        info["imu_initialised"] = "start VIBA 1" in text
    return info


def loop_drift(xyz: np.ndarray):
    if len(xyz) < 2:
        return {"drift_m": None, "path_length_m": 0.0, "drift_pct": None}
    seg = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    path = float(seg.sum())
    drift = float(np.linalg.norm(xyz[-1] - xyz[0]))
    return {"drift_m": drift, "path_length_m": path,
            "drift_pct": (drift / path * 100.0) if path > 0 else None,
            "rpe_step_median_m": float(np.median(seg)),
            "rpe_step_p95_m": float(np.percentile(seg, 95))}


def analyse(name: str, out_dir: Path, bag_dir: Path, dense_max: int):
    traj = out_dir / "trajectory.txt"
    t, xyz = load_tum(traj) if traj.exists() else (np.array([]), np.empty((0, 3)))
    n_color = count_color_frames(bag_dir)
    timing = parse_timing(out_dir)
    fed = timing["frames_fed"]
    res = {"name": name, "output": str(out_dir),
           "n_poses": int(len(xyz)), "n_color_frames": n_color,
           "frames_fed": fed,
           "imu_initialised": timing["imu_initialised"],
           # robustness: tracked poses / frames actually fed to the tracker
           "tracking_success_rate": (len(xyz) / fed) if fed else None,
           # throughput: tracked poses / all bag color frames (incl. dropped)
           "pose_yield_vs_bag": (len(xyz) / n_color) if n_color else None,
           "track_ms_mean": timing["track_ms_mean"],
           "track_fps": timing["track_fps"]}
    res.update(loop_drift(xyz))
    if len(t) >= 3:
        ti = detect_tracking_issues(t, xyz)
        res["n_gaps"] = ti["n_gaps"]
        res["n_jumps"] = ti["n_jumps"]
        res["median_dt_s"] = ti.get("median_dt")
    dense = out_dir / "orbslam3_dense_map.pcd"
    res["mme"] = mean_map_entropy(load_pcd_xyz(dense, dense_max))
    return res


def main():
    base = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        SCRIPT_DIR.parent / "output" / "imu_compare")
    data_root = SCRIPT_DIR.parents[1] / "data_slam"
    conds = [
        ("three_labs_imu",  data_root / "SLAM_three_labs_rgbd_imu" / "bag"),
        ("three_labs_rgbd", data_root / "SLAM_three_labs_rgbd_imu" / "bag"),
        ("SLAM_three_laps", data_root / "SLAM_three_laps"),
        ("SLAM_one_lap",    data_root / "SLAM_one_lap"),
    ]
    results = []
    for name, bag in conds:
        out_dir = base / name
        if not (out_dir / "trajectory.txt").exists():
            print(f"[skip] {name}: no trajectory", file=sys.stderr)
            continue
        print(f"[analyse] {name} ...", file=sys.stderr)
        results.append(analyse(name, out_dir, bag, dense_max=200000))
    out_json = base / "metrics_summary.json"
    out_json.write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    print(f"\nwrote {out_json}", file=sys.stderr)


if __name__ == "__main__":
    main()
