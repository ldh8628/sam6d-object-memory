#!/usr/bin/env python3
"""GT-free 'revisit opportunity' analysis (no re-run).

Quantifies WHY a SLAM closed few loops by separating two things:
  (1) how many physical revisits the trajectory actually contained
      = pairs of poses that are spatially close but temporally distant
  (2) how many loop closures the system actually produced

A large gap between (1) and (2) = the bottleneck is loop *detection*, not the
absence of revisits. Computed purely from saved trajectories.
"""
from __future__ import annotations
import sys, json, sqlite3, struct
from pathlib import Path
import numpy as np


def load_tum(path: Path):
    ts, xyz = [], []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        p = line.split()
        if len(p) < 8:
            continue
        ts.append(float(p[0]))
        xyz.append([float(p[1]), float(p[2]), float(p[3])])
    return np.array(ts), np.array(xyz)


def load_rtab_nodes(db: Path):
    """optimized poses from rtabmap_3d_map_poses.txt-style not needed; use Node odom stamps + opt txt."""
    con = sqlite3.connect(str(db)); c = con.cursor()
    ts, xyz = [], []
    for nid, pose, st in c.execute("select id, pose, stamp from Node where pose is not null order by stamp"):
        v = struct.unpack('<12f', pose)
        T = np.asarray(v, float).reshape(3, 4)
        ts.append(st); xyz.append(T[:, 3])
    con.close()
    return np.array(ts), np.array(xyz)


def analyze(ts, xyz, dt_min=8.0, d_max=0.5, name=""):
    """Count revisit opportunities: pose pairs with |dt|>dt_min and dist<d_max.
    Group consecutive opportunities into distinct 'revisit events' so we don't
    over-count a cluster of nearby frames."""
    n = len(ts)
    if n < 2:
        return {}
    # normalize time to start at 0
    t0 = ts.min()
    t = ts - t0
    pairs = []
    for i in range(n):
        # only look forward, temporally distant
        for j in range(i + 1, n):
            if t[j] - t[i] < dt_min:
                continue
            d = np.linalg.norm(xyz[i] - xyz[j])
            if d < d_max:
                pairs.append((i, j, d, t[j] - t[i]))
    # distinct revisit events: a query frame j that is close to ANY earlier-distant frame
    revisit_query_frames = sorted(set(j for (_, j, _, _) in pairs))
    # cluster query frames that are temporally adjacent (within 2s) into events
    events = 0
    last_t = -1e9
    for j in revisit_query_frames:
        if t[j] - last_t > 2.0:
            events += 1
        last_t = t[j]
    return {
        "name": name,
        "n_poses": int(n),
        "duration_s": float(t.max()),
        "revisit_pairs": len(pairs),
        "revisit_query_frames": len(revisit_query_frames),
        "distinct_revisit_events": int(events),
        "params": {"dt_min_s": dt_min, "d_max_m": d_max},
        "min_pair_dist_m": float(min((d for (_, _, d, _) in pairs), default=float('nan'))),
    }


def main():
    orb_kf = Path("/home/ldh9501/temp_ws/CLI_environment/orbslam_ws/output/imu_compare/three_labs_rgbd/KeyFrameTrajectory.txt")
    orb_cam = Path("/home/ldh9501/temp_ws/CLI_environment/orbslam_ws/output/imu_compare/three_labs_rgbd/CameraTrajectory.txt")
    rtab_db = Path("/home/ldh9501/temp_ws/CLI_environment/rtabmap_ws/output/imu_compare/three_labs_imu/rtabmap.db")

    out = {}
    # ORB keyframes (loop closure operates on keyframes)
    ts, xyz = load_tum(orb_kf)
    out["orb_keyframes"] = analyze(ts, xyz, name="ORB-SLAM3 keyframes (RGB-D)")
    out["orb_keyframes"]["actual_loops"] = 1
    # ORB full camera trajectory (denser sanity check)
    ts2, xyz2 = load_tum(orb_cam)
    out["orb_camera_full"] = analyze(ts2, xyz2, name="ORB-SLAM3 full frames (RGB-D)")
    # RTAB nodes
    if rtab_db.exists():
        tsr, xyzr = load_rtab_nodes(rtab_db)
        out["rtab_nodes"] = analyze(tsr, xyzr, name="RTAB-Map nodes (RGB-D)")
        out["rtab_nodes"]["actual_loops"] = 166

    print(json.dumps(out, indent=2, ensure_ascii=False))
    Path("/home/ldh9501/temp_ws/CLI_environment/orbslam_ws/output/imu_compare/revisit_opportunity.json").write_text(
        json.dumps(out, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
