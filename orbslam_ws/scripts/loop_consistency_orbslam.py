#!/usr/bin/env python3
"""GT-free Loop-Closure Self-Consistency metric (#2) for an ORB-SLAM3 run.

Method B (odom-vs-optimized):
  CameraTrajectory_live.txt  = LIVE front-end pose stream (pre-loop-closure),
                               written per-frame by the patched publishPose().
  CameraTrajectory.txt (==trajectory.txt) = post-optimization trajectory saved
                               at shutdown.
  -> per-frame displacement (matched by timestamp) = correction the optimizer
     applied = drift it revealed.

Method A (loop-closure gap, pre-optimization):
  Parses "LOOP_GAP|...|gap_m=..." lines that the patched LoopClosing.cc prints
  to stdout (captured in orbslam3_launch.log) at each loop detection, BEFORE
  CorrectLoop(). gap_m = camera-center distance between current & matched KF =
  accumulated drift the loop must absorb.
"""
from __future__ import annotations
import sys, re, json
from pathlib import Path
import numpy as np
try:
    from scipy.spatial.transform import Rotation as Rot
    HAVE = True
except Exception:
    HAVE = False


def load_tum(path: Path):
    ts, T = [], []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        p = line.split()
        if len(p) < 8:
            continue
        ts.append(float(p[0]))
        T.append([float(x) for x in p[1:8]])  # tx ty tz qx qy qz qw
    return np.array(ts), np.array(T)


def stats(a):
    a = np.asarray(a, float)
    if a.size == 0:
        return {}
    return {"n": int(a.size), "mean": float(a.mean()), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def quat_angle_deg(qa, qb):
    if not HAVE:
        return float('nan')
    Ra = Rot.from_quat(qa); Rb = Rot.from_quat(qb)
    rel = Ra.inv() * Rb
    return float(np.degrees(rel.magnitude()))


def main(run_dir: str):
    run = Path(run_dir)
    live_p = run / "CameraTrajectory_live.txt"
    opt_p = run / "CameraTrajectory.txt"
    if not opt_p.exists():
        opt_p = run / "trajectory.txt"
    log_p = run / "orbslam3_launch.log"
    out = {"run": run.name}

    # ---------- Method B ----------
    if live_p.exists() and opt_p.exists():
        ts_l, Tl = load_tum(live_p)
        ts_o, To = load_tum(opt_p)
        # index optimized by rounded timestamp
        omap = {round(t, 6): i for i, t in enumerate(ts_o)}
        dt, dr = [], []
        for i, t in enumerate(ts_l):
            j = omap.get(round(t, 6))
            if j is None:
                continue
            dt.append(float(np.linalg.norm(Tl[i, :3] - To[j, :3])))
            dr.append(quat_angle_deg(Tl[i, 3:7], To[j, 3:7]))
        # path length from optimized
        path_len = float(np.sum(np.linalg.norm(np.diff(To[:, :3], axis=0), axis=1))) if len(To) > 1 else 0.0
        out["method_B_odom_vs_optimized"] = {
            "n_live": int(len(ts_l)), "n_opt": int(len(ts_o)), "n_matched": len(dt),
            "path_length_m": path_len,
            "disp_trans_m": stats(dt), "disp_rot_deg": stats([x for x in dr if x == x]),
            "disp_trans_pct_of_path": (float(np.max(dt)) / path_len * 100) if path_len > 0 and dt else None,
        }
    else:
        out["method_B_odom_vs_optimized"] = {
            "error": f"missing live({live_p.exists()}) or opt({opt_p.exists()}) trajectory"}

    # ---------- Method A ----------
    gaps, srot = [], []
    n_loops = 0
    if log_p.exists():
        for line in log_p.read_text(errors='replace').splitlines():
            if "LOOP_GAP|" not in line:
                continue
            n_loops += 1
            m = re.search(r"gap_m=([-\d.eE]+)", line)
            if m:
                gaps.append(float(m.group(1)))
        n_detected = log_p.read_text(errors='replace').count("*Loop detected")
    else:
        n_detected = None
    out["method_A_loop_gap"] = {
        "n_loop_gap_lines": n_loops,
        "n_loop_detected_msgs": n_detected,
        "gap_trans_pre_opt_m": stats(gaps),
        "gaps_raw_m": [round(g, 4) for g in gaps],
    }

    print(json.dumps(out, indent=2))
    (run / "loop_consistency_metrics.json").write_text(json.dumps(out, indent=2))
    return out


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         "/home/ldh9501/temp_ws/CLI_environment/orbslam_ws/output/imu_compare/three_labs_imu")
