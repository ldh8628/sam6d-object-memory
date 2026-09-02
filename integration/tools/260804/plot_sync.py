#!/usr/bin/env python3
"""Overlay the three independently-derived yaw-rate signals + IMU sanity."""
from __future__ import annotations

import argparse
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="")
    ap.add_argument("--zoom", nargs=2, type=float, default=None)
    args = ap.parse_args()
    z = np.load(args.npz)
    imu_t, imu_w = z["imu_t"], z["imu_w"]
    t0 = imu_t[0]

    # scale each proxy onto the IMU by least squares so shapes are comparable
    def scaled(t, w, ref_t, ref_w):
        m = np.isfinite(w)
        r = np.interp(t[m], ref_t, ref_w)
        s = float(np.dot(r, w[m]) / np.dot(w[m], w[m]))
        out = np.full_like(w, np.nan)
        out[m] = w[m] * s
        return out, s

    lid, s_l = scaled(z["ly_t"], z["ly_w"], imu_t, imu_w[:, 2])
    cam, s_c = scaled(z["cam_t"], z["cam_w"], imu_t, imu_w[:, 2])

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=False)
    ax = axes[0]
    ax.plot(imu_t - t0, imu_w[:, 2], lw=1.0, color="k", label="Xsens gyro z (200 Hz)")
    ax.plot(z["ly_t"] - t0, lid, lw=1.4, color="tab:red", marker=".", ms=3,
            label=f"Velodyne sweep-shift ×{s_l:.2f} (10 Hz)")
    ax.plot(z["cam_t"] - t0, cam, lw=1.0, color="tab:blue", alpha=0.8,
            label=f"D455 optical flow ×{s_c:.2f} (30 Hz)")
    ax.set_ylabel("yaw rate [rad/s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_title(args.title + "  —  three independent yaw-rate estimates, "
                 "each sensor's own timestamps, no shift applied")

    ax = axes[1]
    lo, hi = args.zoom if args.zoom else (0.0, min(6.0, imu_t[-1] - t0))
    ax.plot(imu_t - t0, imu_w[:, 2], lw=1.2, color="k")
    ax.plot(z["ly_t"] - t0, lid, lw=1.6, color="tab:red", marker="o", ms=4)
    ax.plot(z["cam_t"] - t0, cam, lw=1.2, color="tab:blue", marker=".", ms=4)
    ax.set_xlim(lo, hi)
    ax.set_ylabel("yaw rate [rad/s]")
    ax.set_title(f"zoom {lo:.0f}–{hi:.0f} s")
    ax.grid(alpha=0.3)

    ax = axes[2]
    acc = z["imu_w"]  # placeholder if accel absent
    ax.plot(imu_t - t0, np.degrees(np.cumsum(imu_w[:, 2]) * 0.005),
            color="k", label="integrated Xsens yaw")
    ax.set_ylabel("yaw [deg]")
    ax.set_xlabel("t since first IMU sample [s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print("->", args.out)
    print(json.dumps({"lidar_scale_to_imu": round(s_l, 3),
                      "camera_scale_to_imu": round(s_c, 3),
                      "total_integrated_yaw_deg": round(
                          float(np.degrees(np.sum(imu_w[:, 2]) * 0.005)), 1),
                      "abs_integrated_yaw_deg": round(
                          float(np.degrees(np.sum(np.abs(imu_w[:, 2])) * 0.005)), 1)},
                     indent=1))


if __name__ == "__main__":
    main()
