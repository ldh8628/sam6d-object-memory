#!/usr/bin/env python3
"""Turn hdl_graph_slam's keyframe graph into a dense reference trajectory.

The dumped graph holds ~160 optimised keyframes over 172 s, which is too coarse
for two things we need:

  * scoring camera SLAM, whose trajectories carry thousands of per-frame poses;
  * checking short events. hdl gates keyframes on *distance travelled*, so a
    stationary stretch produces no keyframes at all — precisely when we most
    want samples (183942's t = 8-12 s window).

Scan-matching odometry, on the other hand, runs at 9.9 Hz but drifts, because
nothing corrects it. The two combine cleanly: every dumped keyframe carries both
its optimised pose and the odometry pose it came from, so

    C_k = estimate_k * odom_k^-1

is the correction the pose graph applied at that instant. Interpolating C
between keyframes (slerp on rotation, lerp on translation) and applying it to
the dense odometry gives a loop-corrected trajectory at the odometry's rate.

    python3 densify_lidar_trajectory.py eight3b_183942
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dump_hdl_graph import parse_keyframe, R_to_quat        # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "slam_comparison" / "output"


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def slerp(q0, q1, u):
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0:                      # take the short way round
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    s = np.sin(th)
    return (np.sin((1 - u) * th) * q0 + np.sin(u * th) * q1) / s


def read_odom(bag: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    ts, T = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/odom":
            continue
        m = deserialize_message(data, Odometry)
        p, o = m.pose.pose.position, m.pose.pose.orientation
        M = np.eye(4)
        M[:3, :3] = quat_to_R([o.x, o.y, o.z, o.w])
        M[:3, 3] = [p.x, p.y, p.z]
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        T.append(M)
    order = np.argsort(ts)
    return np.array(ts)[order], np.array(T)[order]


def read_corrections(dump: Path):
    """C_k = estimate_k * odom_k^-1, as (stamp, quaternion, translation)."""
    stamps, quats, trans = [], [], []
    for d in sorted(dump.iterdir()):
        f = d / "data"
        if not (d.is_dir() and f.exists()):
            continue
        kf = parse_keyframe(f)
        if kf is None or kf.get("odom") is None:
            continue
        C = kf["estimate"] @ np.linalg.inv(kf["odom"])
        stamps.append(kf["stamp"])
        quats.append(R_to_quat(C[:3, :3]))
        trans.append(C[:3, 3])
    order = np.argsort(stamps)
    return (np.array(stamps)[order], np.array(quats)[order], np.array(trans)[order])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="+")
    args = ap.parse_args()

    rc = 0
    for ses in args.sessions:
        gt = OUTPUT / ses / "lidar_gt"
        bag, dump = gt / "hdl_recorded_bag", gt / "graph_dump"
        if not (bag.exists() and dump.exists()):
            print(f"{ses}: missing hdl_recorded_bag or graph_dump")
            rc = 1
            continue

        ots, oT = read_odom(bag)
        cts, cq, ct = read_corrections(dump)
        if not len(ots) or not len(cts):
            print(f"{ses}: no odometry ({len(ots)}) or corrections ({len(cts)})")
            rc = 1
            continue

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

        out = gt / "trajectory_dense.txt"
        out.write_text("\n".join(rows) + "\n")
        xyz = np.array([[float(v) for v in r.split()[1:4]] for r in rows])
        path = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
        print(f"{ses}: {len(rows)} poses at "
              f"{len(ots)/(ots[-1]-ots[0]):.1f} Hz from {len(cts)} keyframe "
              f"corrections, path {path:.2f} m, "
              f"start-end {np.linalg.norm(xyz[-1]-xyz[0]):.3f} m -> {out.name}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
