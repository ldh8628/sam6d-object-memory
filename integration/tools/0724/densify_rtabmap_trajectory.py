#!/usr/bin/env python3
"""Build a per-frame, loop-corrected RTAB-Map trajectory.

Same construction as densify_lidar_trajectory.py, applied to RTAB-Map so the two
camera systems can be compared at the same output rate. RTAB-Map's exported
trajectory is its pose *graph* (~0.9 Hz here, with gaps of up to 18 s where the
platform was nearly still), while its `rgbd_odometry` runs at frame rate. The
graph carries the loop closures; the odometry carries the rate. Combine them:

    C_k = optimized_pose_k * odometry_pose_k^-1     (per graph node)
    trajectory(t) = interp(C, t) * odometry(t)

`optimized_pose_k` comes from the `rtabmap-export --poses` file, `odometry_pose_k`
from the database's `Node.pose` (which stores the odometry pose, not the
optimized one), and `odometry(t)` from the `/odom` recorded by run_rtab_dense.py.

    conda activate rtabmap
    python3 densify_rtabmap_trajectory.py <session> [...]
"""
from __future__ import annotations

import argparse
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parent))
from densify_lidar_trajectory import quat_to_R, slerp            # noqa: E402
from dump_hdl_graph import R_to_quat                             # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "slam_comparison" / "output"


def node_odom_poses(db: Path) -> dict[int, np.ndarray]:
    """Node id -> odometry pose (4x4). Node.pose is a row-major 3x4 float32."""
    con = sqlite3.connect(str(db))
    out = {}
    for nid, blob in con.execute("select id, pose from Node where pose is not null"):
        if blob is None or len(blob) != 48:
            continue
        T = np.eye(4)
        T[:3, :4] = np.asarray(struct.unpack("<12f", blob), float).reshape(3, 4)
        out[int(nid)] = T
    con.close()
    return out


def optimized_poses(path: Path):
    """(stamp, id, 4x4) from `rtabmap-export --poses --poses_format 11`."""
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = line.split()
        if len(v) < 9:
            continue
        stamp = float(v[0])
        t = np.array([float(v[1]), float(v[2]), float(v[3])])
        q = np.array([float(v[4]), float(v[5]), float(v[6]), float(v[7])])
        nid = int(float(v[8]))
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q)
        T[:3, 3] = t
        rows.append((stamp, nid, T))
    rows.sort(key=lambda r: r[0])
    return rows


def read_odom(bag: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    ts, T = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if not topic.endswith("/odom"):
            continue
        m = deserialize_message(data, Odometry)
        p, o = m.pose.pose.position, m.pose.pose.orientation
        if not np.isfinite([p.x, p.y, p.z, o.x, o.y, o.z, o.w]).all():
            continue        # rgbd_odometry publishes a null pose when it is lost
        M = np.eye(4)
        M[:3, :3] = quat_to_R([o.x, o.y, o.z, o.w])
        M[:3, 3] = [p.x, p.y, p.z]
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        T.append(M)
    order = np.argsort(ts)
    return np.array(ts)[order], (np.array(T)[order] if T else np.zeros((0, 4, 4)))


def run(session: str) -> bool:
    d = OUTPUT / session / "rtabmap_dense"
    db, bag = d / "rtabmap.db", d / "odom_bag"
    opt_f = d / "rtabmap_dense_poses.txt"
    if not (db.exists() and bag.exists() and opt_f.exists()):
        print(f"{session}: missing db / odom_bag / poses export")
        return False

    odom_by_id = node_odom_poses(db)
    opt = optimized_poses(opt_f)
    pairs = [(s, T, odom_by_id[i]) for s, i, T in opt if i in odom_by_id]
    if not pairs:
        print(f"{session}: no node matched between export and database")
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
        print(f"{session}: /odom is empty")
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
    gaps = np.diff(ots)
    print(f"{session}: {len(rows)} poses at {len(ots)/max(span,1e-9):.1f} Hz "
          f"from {len(pairs)} node corrections, max gap {gaps.max():.2f} s, "
          f"path {path:.2f} m -> {out.name}")
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="+")
    args = ap.parse_args()
    ok = sum(run(s) for s in args.sessions)
    print(f"\n{ok}/{len(args.sessions)} densified")
    return 0 if ok == len(args.sessions) else 1


if __name__ == "__main__":
    raise SystemExit(main())
