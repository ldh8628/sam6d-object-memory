#!/usr/bin/env python3
"""Build a per-frame, loop-corrected RTAB-Map trajectory for a 260804 session.

Identical construction to data_slam/0724_chungbuk/densify_rtabmap_trajectory.py
(whose helpers are reused verbatim), only re-pointed at the 260804 output layout
`output_slam/260804_office/rtabmap_dense/<session>/`:

    C_k = optimized_pose_k * odometry_pose_k^-1      (per graph node)
    trajectory(t) = interp(C, t) * odometry(t)

`optimized_pose_k` comes from `rtabmap-export --poses`, `odometry_pose_k` from
the database's `Node.pose` (which stores the odometry pose, not the optimised
one), and `odometry(t)` from the `/rtabmap/odom` recorded by
run_rtab_dense_260804.py during the same run.

    conda activate hdl_graph_slam_humble    # any env with rosbag2_py + numpy
    python3 densify_rtab_260804.py longcircle2
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트
OUT_ROOT = ROOT / "output_slam" / "260804_office" / "rtabmap_dense"

sys.path.insert(0, str(ROOT / "data_slam" / "0724_chungbuk"))
from densify_lidar_trajectory import quat_to_R, slerp                  # noqa: E402
from dump_hdl_graph import R_to_quat                                   # noqa: E402
from densify_rtabmap_trajectory import (node_odom_poses,               # noqa: E402
                                        optimized_poses, read_odom)


def run(session: str) -> bool:
    d = OUT_ROOT / session
    db, bag = d / "rtabmap.db", d / "odom_bag"
    opt_f = d / "rtabmap_dense_poses.txt"
    missing = [p.name for p in (db, bag, opt_f) if not p.exists()]
    if missing:
        print(f"{session}: missing {missing}", file=sys.stderr)
        return False

    odom_by_id = node_odom_poses(db)
    opt = optimized_poses(opt_f)
    pairs = [(s, T, odom_by_id[i]) for s, i, T in opt if i in odom_by_id]
    if not pairs:
        print(f"{session}: no node matched between export and database",
              file=sys.stderr)
        return False

    cts = np.array([p[0] for p in pairs])
    cq, ct = [], []
    for _s, T_opt, T_odom in pairs:
        C = T_opt @ np.linalg.inv(T_odom)
        cq.append(R_to_quat(C[:3, :3]))
        ct.append(C[:3, 3])
    cq, ct = np.array(cq), np.array(ct)

    ots, oT = read_odom(bag)
    if not len(ots):
        print(f"{session}: /odom is empty", file=sys.stderr)
        return False

    rows = []
    for t, O in zip(ots, oT):
        j = int(np.searchsorted(cts, t))
        if j <= 0:
            q, tr = cq[0], ct[0]
        elif j >= len(cts):
            q, tr = cq[-1], ct[-1]
        else:
            span = cts[j] - cts[j - 1]
            u = 0.0 if span <= 0 else float((t - cts[j - 1]) / span)
            q = slerp(cq[j - 1], cq[j], u)
            tr = ct[j - 1] + u * (ct[j] - ct[j - 1])
        C = np.eye(4)
        C[:3, :3] = quat_to_R(q)
        C[:3, 3] = tr
        M = C @ O
        qq = R_to_quat(M[:3, :3])
        rows.append(f"{t:.9f} {M[0,3]:.6f} {M[1,3]:.6f} {M[2,3]:.6f} "
                    f"{qq[0]:.6f} {qq[1]:.6f} {qq[2]:.6f} {qq[3]:.6f}")

    out = d / "trajectory_dense.txt"
    out.write_text("\n".join(rows) + "\n")
    xyz = np.array([[float(v) for v in r.split()[1:4]] for r in rows])
    path = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
    span = ots[-1] - ots[0]
    print(f"{session}: {len(rows)} poses at {len(ots)/max(span,1e-9):.1f} Hz "
          f"from {len(pairs)} node corrections, max gap {np.diff(ots).max():.2f} s, "
          f"path {path:.2f} m -> {out.name}")
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="+")
    args = ap.parse_args()
    ok = sum(run(s) for s in args.sessions)
    print(f"{ok}/{len(args.sessions)} densified")
    return 0 if ok == len(args.sessions) else 1


if __name__ == "__main__":
    raise SystemExit(main())
