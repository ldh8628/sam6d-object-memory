#!/usr/bin/env python3
"""Ask whether the Xsens AHRS yaw is trustworthy enough to be ground truth.

All three SLAM methods drift against it in the same direction by about the same
amount, which is the signature of a bad reference rather than three independent
failures. Two checks settle it:

  * AHRS yaw vs. its own gyro, integrated. The gyro is the AHRS's own input, so
    any divergence between them is a correction the filter applied - i.e. the
    magnetometer pulling yaw around.
  * the magnetic field magnitude. Indoors, steel and motors distort it; a field
    that swings while the platform merely turns is not a usable heading source.

Roll and pitch are not in question: gravity anchors them and no equivalent
drift mechanism exists.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Vector3Stamped

import importlib.util
_spec = importlib.util.spec_from_file_location(
    "_ev", Path(__file__).resolve().parent / "eval_vs_imu.py")
_ev = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_ev)
quat_to_R, log_so3 = _ev.quat_to_R, _ev.log_so3


def read_mag(bag: str, topic="/xsens/magnetic_field_au"):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    t, v = [], []
    while reader.has_next():
        _, raw, _ = reader.read_next()
        m = deserialize_message(raw, Vector3Stamped)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        v.append([m.vector.x, m.vector.y, m.vector.z])
    return np.asarray(t), np.asarray(v)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", nargs="+", default=["longcircle2", "stablization"])
    ap.add_argument("--converted", default=str(
        Path(__file__).resolve().parents[2] / "data_slam_converted" / "260804_office"))
    ap.add_argument("--imu-dir", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    report = {}
    for s in args.sessions:
        z = np.load(Path(args.imu_dir) / f"imu_{s}.npz")
        t, q, w = z["t"], z["q"], z["w"]
        R = quat_to_R(q)
        dt = np.diff(t)

        # --- gravity axis in the AHRS world frame, from the AHRS itself
        aw = np.einsum('nij,nj->ni', R, z["a"])
        up = aw.mean(0) / np.linalg.norm(aw.mean(0))

        # gravity axis expressed in the body frame at each sample, so both yaw
        # estimates below are projected identically and only the source differs
        up_body = np.einsum('nji,j->ni', R, up)

        # --- yaw as the filter reports it: increments projected on that axis
        inc = log_so3(np.einsum('nji,njk->nik', R[:-1], R[1:]))
        ahrs_yaw = np.degrees(np.concatenate(
            [[0.0], np.cumsum(np.einsum('ni,ni->n', inc, up_body[:-1]))]))

        # --- yaw from the raw gyro alone, no filter corrections
        wz = np.einsum('ni,ni->n', w[:-1], up_body[:-1])
        gyro_yaw = np.degrees(np.concatenate([[0.0], np.cumsum(wz * dt)]))

        diff = ahrs_yaw - gyro_yaw
        slope = np.polyfit(t - t[0], diff, 1)[0] * 60.0

        mt, mv = read_mag(str(Path(args.converted) / s / "ros2_standard"))
        mag = np.linalg.norm(mv, axis=1)

        report[s] = {
            "duration_s": round(float(t[-1] - t[0]), 2),
            "ahrs_yaw_total_deg": round(float(ahrs_yaw[-1]), 2),
            "gyro_yaw_total_deg": round(float(gyro_yaw[-1]), 2),
            "ahrs_minus_gyro_final_deg": round(float(diff[-1]), 2),
            "ahrs_minus_gyro_slope_deg_per_min": round(float(slope), 2),
            "ahrs_minus_gyro_ptp_deg": round(float(np.ptp(diff)), 2),
            "mag_norm_mean_au": round(float(mag.mean()), 4),
            "mag_norm_cv_pct": round(float(100 * mag.std() / mag.mean()), 2),
            "mag_norm_ptp_over_mean_pct": round(float(100 * np.ptp(mag) / mag.mean()), 2),
        }
        r = report[s]
        print(f"{s}: AHRS yaw {r['ahrs_yaw_total_deg']}deg vs gyro {r['gyro_yaw_total_deg']}deg "
              f"-> filter applied {r['ahrs_minus_gyro_final_deg']}deg "
              f"({r['ahrs_minus_gyro_slope_deg_per_min']} deg/min)")
        print(f"    magnetic field: mean {r['mag_norm_mean_au']} au, "
              f"CV {r['mag_norm_cv_pct']}%, peak-to-peak {r['mag_norm_ptp_over_mean_pct']}% of mean")

    Path(args.out).write_text(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
