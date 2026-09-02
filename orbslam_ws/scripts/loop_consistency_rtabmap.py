#!/usr/bin/env python3
"""GT-free Loop-Closure Self-Consistency metric (#2) extracted from a RTAB-Map .db.

Computes, with NO re-run and NO ground truth:
  - Method B (odom-vs-optimized): per-node displacement between the raw odometry
    pose (Node.pose) and the loop-optimized graph pose (rtabmap_3d_map_poses.txt).
    = how much the optimizer had to move each node = drift it revealed.
  - Method A (loop-closure gap + chi2): for every loop/proximity-closure Link,
    the discrepancy between the odom-implied relative pose between the two linked
    nodes and the loop-measured relative transform (Link.transform), weighted by
    the stored 6x6 information_matrix (chi2). = accumulated drift the loop absorbs,
    BEFORE optimization, plus a validity (chi2) signal for wrong-loop detection.

RTAB-Map storage facts used (verified on this db):
  Node.pose            : 12 float32, row-major 3x4 (R|t) = ODOMETRY pose
  Link.type            : 0=Neighbor(odom) 1=GlobalLoopClosure 2=LocalSpaceClosure
  Link.transform       : 12 float32 3x4 = measured relative transform (T_meas)
  Link.information_matrix : 36 float64 6x6  (order assumed [tx,ty,tz, rx,ry,rz])
  rtabmap_3d_map_poses.txt : exported OPTIMIZED graph poses (TUM + id col)
"""
from __future__ import annotations
import sqlite3, struct, sys, json
from pathlib import Path
import numpy as np

try:
    from scipy.spatial.transform import Rotation as Rot
    HAVE = True
except Exception:
    HAVE = False


def mat34_to_T(buf12: bytes) -> np.ndarray:
    v = struct.unpack('<12f', buf12)
    T = np.eye(4)
    T[:3, :4] = np.asarray(v, float).reshape(3, 4)
    return T


def so3_log_angle(R: np.ndarray) -> float:
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(tr)))


def se3_log(T: np.ndarray) -> np.ndarray:
    """Return 6-vec [tx,ty,tz, rx,ry,rz] (rotvec in rad). Small-angle ok."""
    t = T[:3, 3]
    if HAVE:
        rv = Rot.from_matrix(T[:3, :3]).as_rotvec()
    else:
        ang = np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1))
        rv = np.zeros(3)
        if ang > 1e-9:
            w = np.array([T[2, 1] - T[1, 2], T[0, 2] - T[2, 0], T[1, 0] - T[0, 1]])
            rv = ang * w / (2 * np.sin(ang))
    return np.concatenate([t, rv])


def load_tum_with_id(path: Path) -> dict[int, np.ndarray]:
    out = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        p = line.split()
        if len(p) < 9:
            continue
        x, y, z, qx, qy, qz, qw = map(float, p[1:8])
        nid = int(float(p[8]))
        T = np.eye(4)
        if HAVE:
            T[:3, :3] = Rot.from_quat([qx, qy, qz, qw]).as_matrix()
        T[:3, 3] = [x, y, z]
        out[nid] = T
    return out


def stats(a):
    a = np.asarray(a, float)
    if a.size == 0:
        return {}
    return {"n": int(a.size), "mean": float(a.mean()), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def main(run_dir: str):
    run = Path(run_dir)
    db = run / "rtabmap.db"
    opt_txt = run / "rtabmap_3d_map_poses.txt"
    con = sqlite3.connect(str(db)); c = con.cursor()

    # --- odom poses (Node.pose) ---
    odom = {}
    stamp = {}
    for nid, pose, st in c.execute("select id, pose, stamp from Node where pose is not null order by id"):
        odom[nid] = mat34_to_T(pose)
        stamp[nid] = st
    # --- optimized poses ---
    opt = load_tum_with_id(opt_txt)

    # ================= METHOD B: odom vs optimized =================
    common = sorted(set(odom) & set(opt))
    d_trans, d_rot = [], []
    # path length from optimized trajectory (ordered by id)
    opt_ids_sorted = sorted(opt)
    path_len = 0.0
    prev = None
    for nid in opt_ids_sorted:
        t = opt[nid][:3, 3]
        if prev is not None:
            path_len += np.linalg.norm(t - prev)
        prev = t
    for nid in common:
        diff = np.linalg.inv(opt[nid]) @ odom[nid]
        d_trans.append(np.linalg.norm(diff[:3, 3]))
        d_rot.append(so3_log_angle(diff[:3, :3]))
    methodB = {
        "n_common_nodes": len(common),
        "path_length_m": path_len,
        "disp_trans_m": stats(d_trans),
        "disp_rot_deg": stats(d_rot),
        "disp_trans_pct_of_path": (float(np.max(d_trans)) / path_len * 100) if path_len > 0 and d_trans else None,
    }

    # ================= METHOD A: loop-closure gap + chi2 =================
    def link_rows(types):
        q = "select from_id, to_id, type, transform, information_matrix from Link where type in (%s)" % \
            ",".join("?" * len(types))
        return c.execute(q, types).fetchall()

    results = {}
    for label, types in (("global_loop(type1)", [1]), ("local_space(type2)", [2]), ("all_closures(1,2)", [1, 2])):
        gaps_t, gaps_r, chis_pre, chis_post, gaps_t_post = [], [], [], [], []
        used = 0
        skipped = 0
        for f_id, t_id, ttype, tf, info in link_rows(types):
            if f_id not in odom or t_id not in odom or tf is None:
                continue
            T_meas = mat34_to_T(tf)
            # skip degenerate transforms (null/zero rotation block)
            if abs(np.linalg.det(T_meas[:3, :3]) - 1.0) > 1e-2 or \
               abs(np.linalg.det(odom[f_id][:3, :3]) - 1.0) > 1e-2:
                skipped += 1
                continue
            # odom-implied relative transform from_id -> to_id
            T_odom_rel = np.linalg.inv(odom[f_id]) @ odom[t_id]
            res_pre = np.linalg.inv(T_meas) @ T_odom_rel
            gt = np.linalg.norm(res_pre[:3, 3]); gr = so3_log_angle(res_pre[:3, :3])
            gaps_t.append(gt); gaps_r.append(gr)
            # chi2 with stored info (order assumed [t, r])
            if info is not None and len(info) == 288:
                Info = np.asarray(struct.unpack('<36d', info), float).reshape(6, 6)
                xi = se3_log(res_pre)
                chis_pre.append(float(xi @ Info @ xi))
            # post-optimization residual (using optimized poses) for contrast
            if f_id in opt and t_id in opt:
                T_opt_rel = np.linalg.inv(opt[f_id]) @ opt[t_id]
                res_post = np.linalg.inv(T_meas) @ T_opt_rel
                gaps_t_post.append(np.linalg.norm(res_post[:3, 3]))
                if info is not None and len(info) == 288:
                    Info = np.asarray(struct.unpack('<36d', info), float).reshape(6, 6)
                    xi2 = se3_log(res_post)
                    chis_post.append(float(xi2 @ Info @ xi2))
            used += 1
        results[label] = {
            "n_links": used,
            "n_skipped_degenerate": skipped,
            "gap_trans_pre_opt_m": stats(gaps_t),
            "gap_rot_pre_opt_deg": stats(gaps_r),
            "gap_trans_post_opt_m": stats(gaps_t_post),
            "chi2_pre_opt": stats(chis_pre),
            "chi2_post_opt": stats(chis_post),
        }
    con.close()

    out = {"run": run.name, "method_B_odom_vs_optimized": methodB, "method_A_loop_gap": results}
    print(json.dumps(out, indent=2))
    (run / "loop_consistency_metrics.json").write_text(json.dumps(out, indent=2))
    return out


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         "/home/ldh9501/temp_ws/CLI_environment/rtabmap_ws/output/imu_compare/three_labs_imu")
