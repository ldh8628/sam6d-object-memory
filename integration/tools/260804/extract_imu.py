#!/usr/bin/env python3
"""Pull the Xsens stream out of a converted 260804 bag into a .npz.

Keeps everything the ground-truth comparison needs: the on-board AHRS
orientation (if the device actually filled it), the raw rates and the raw
accelerations, all on the bag's host-epoch time axis.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def read_imu(bag: str, topic: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    t, q, w, a = [], [], [], []
    while reader.has_next():
        _, raw, _ = reader.read_next()
        m = deserialize_message(raw, Imu)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        q.append([m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w])
        w.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        a.append([m.linear_acceleration.x, m.linear_acceleration.y,
                  m.linear_acceleration.z])
    return (np.asarray(t), np.asarray(q), np.asarray(w), np.asarray(a))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/xsens/imu/data")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    t, q, w, a = read_imu(args.bag, args.topic)
    np.savez(args.out, t=t, q=q, w=w, a=a)

    norms = np.linalg.norm(q, axis=1)
    # an unfilled sensor_msgs/Imu orientation is the zero quaternion, and a
    # driver that reports "no orientation" leaves it identity for every sample
    filled = bool(np.all(norms > 0.5))
    static = bool(filled and np.max(np.abs(q - q[0])) < 1e-6)
    print(f"{Path(args.out).stem}: n={len(t)} dur={t[-1]-t[0]:.2f}s "
          f"rate={len(t)/(t[-1]-t[0]):.1f}Hz")
    print(f"  |q| min/max = {norms.min():.4f}/{norms.max():.4f} "
          f"filled={filled} constant={static}")
    print(f"  q[0] = {np.round(q[0], 4)}   q[-1] = {np.round(q[-1], 4)}")
    print(f"  |acc| median={np.median(np.linalg.norm(a, axis=1)):.3f} "
          f"mean_acc={np.round(a.mean(axis=0), 3)}")
    print(f"  |gyro| max={np.abs(w).max():.3f} rad/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
