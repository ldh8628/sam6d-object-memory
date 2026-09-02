#!/usr/bin/env python3
"""Structural probe of the 260804 converted bags: frame_ids, distortion,
pointcloud geometry, header-vs-bag timestamp offsets, per-topic gaps."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = [
    "/camera/camera/color/image_raw",
    "/camera/camera/color/camera_info",
    "/camera/camera/aligned_depth_to_color/image_raw",
    "/camera/camera/aligned_depth_to_color/camera_info",
    "/velodyne_points",
    "/xsens/imu/data",
]


def reader(bag: Path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    return r


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--json")
    args = ap.parse_args()
    bag = Path(args.bag)

    r = reader(bag)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    stats = {t: {"n": 0, "hdr_ns": [], "bag_ns": [], "frame_id": None}
             for t in TOPICS if t in types}
    samples = {}
    while r.has_next():
        topic, data, t_bag = r.read_next()
        if topic not in stats:
            continue
        s = stats[topic]
        msg = deserialize_message(data, get_message(types[topic]))
        hdr = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        s["n"] += 1
        s["hdr_ns"].append(hdr)
        s["bag_ns"].append(t_bag)
        if s["frame_id"] is None:
            s["frame_id"] = msg.header.frame_id
        if topic == "/velodyne_points" and "pc" not in samples:
            pts = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            pts = pts.reshape(-1, msg.point_step)
            xyz = np.zeros((pts.shape[0], 3), dtype=np.float32)
            offs = {f.name: f.offset for f in msg.fields}
            for i, k in enumerate("xyz"):
                xyz[:, i] = pts[:, offs[k]:offs[k] + 4].copy().view(np.float32).ravel()
            good = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-6)
            g = xyz[good]
            az = np.degrees(np.arctan2(g[:, 1], g[:, 0]))
            el = np.degrees(np.arctan2(g[:, 2], np.linalg.norm(g[:, :2], axis=1)))
            rng = np.linalg.norm(g, axis=1)
            hist = np.histogram(az, bins=36, range=(-180, 180))[0]
            samples["pc"] = {
                "n_points": int(msg.width * msg.height), "n_valid": int(good.sum()),
                "height": int(msg.height), "width": int(msg.width),
                "is_dense": bool(msg.is_dense), "point_step": int(msg.point_step),
                "az_bins_nonempty_of_36": int((hist > 0).sum()),
                "az_min": round(float(az.min()), 1), "az_max": round(float(az.max()), 1),
                "el_min": round(float(el.min()), 1), "el_max": round(float(el.max()), 1),
                "el_unique_approx": int(len(np.unique(np.round(el, 0)))),
                "range_min": round(float(rng.min()), 2),
                "range_max": round(float(rng.max()), 2),
            }
        if topic.endswith("camera_info") and topic not in samples:
            samples[topic] = {"D": [round(float(v), 6) for v in msg.d],
                              "model": msg.distortion_model,
                              "K": [round(float(v), 3) for v in msg.k]}
        if topic.endswith("image_raw") and topic not in samples:
            samples[topic] = {"enc": msg.encoding, "w": msg.width, "h": msg.height,
                              "step": msg.step, "big_endian": int(msg.is_bigendian)}
        if topic == "/xsens/imu/data" and "imu" not in samples:
            samples["imu"] = {
                "orientation_cov0": float(msg.orientation_covariance[0]),
                "ang_cov0": float(msg.angular_velocity_covariance[0]),
                "lin_cov0": float(msg.linear_acceleration_covariance[0]),
                "quat": [round(float(v), 4) for v in
                         (msg.orientation.x, msg.orientation.y,
                          msg.orientation.z, msg.orientation.w)],
                "acc": [round(float(v), 3) for v in
                        (msg.linear_acceleration.x, msg.linear_acceleration.y,
                         msg.linear_acceleration.z)],
            }

    out = {"bag": str(bag), "topics": {}, "samples": samples}
    t0 = min(min(s["hdr_ns"]) for s in stats.values() if s["n"])
    for topic, s in stats.items():
        if not s["n"]:
            continue
        h = np.asarray(s["hdr_ns"], dtype=np.int64)
        b = np.asarray(s["bag_ns"], dtype=np.int64)
        d = np.diff(h) / 1e6
        out["topics"][topic] = {
            "n": s["n"], "frame_id": s["frame_id"],
            "hdr_first_ns": int(h[0]), "hdr_last_ns": int(h[-1]),
            "start_offset_ms": round(float((h[0] - t0) / 1e6), 2),
            "duration_s": round(float((h[-1] - h[0]) / 1e9), 3),
            "hdr_monotonic": bool((np.diff(h) > 0).all()),
            "dt_ms_mean": round(float(d.mean()), 3),
            "dt_ms_std": round(float(d.std()), 3),
            "dt_ms_max": round(float(d.max()), 3),
            "dt_ms_min": round(float(d.min()), 3),
            "gaps_gt_3x": int((d > 3 * np.median(d)).sum()),
            "bag_minus_hdr_ms_mean": round(float(((b - h) / 1e6).mean()), 3),
            "bag_minus_hdr_ms_max": round(float(((b - h) / 1e6).max()), 3),
        }
    txt = json.dumps(out, indent=1)
    print(txt)
    if args.json:
        Path(args.json).write_text(txt)


if __name__ == "__main__":
    main()
