#!/usr/bin/env python3
"""Cross-sensor time-sync check for the 260804 bags.

Estimates a yaw-rate signal independently from each of the three sensors and
cross-correlates them.  If the recorder stamped all three on one clock, the
peak lag is ~0; a systematic lag shows up as a peak offset.

  * IMU     : angular_velocity, whichever axis correlates (frame unknown)
  * LiDAR   : azimuth/range profile shift between consecutive full sweeps
  * Camera  : median horizontal optical flow / fx  (yaw-dominant motion)

Run under `conda activate rtabmap`.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMU_T = "/xsens/imu/data"
PC_T = "/velodyne_points"
RGB_T = "/camera/camera/color/image_raw"
INFO_T = "/camera/camera/color/camera_info"

NBIN = 360           # 1 deg azimuth bins
MAX_SHIFT = 60       # +-60 deg between consecutive sweeps (600 deg/s @10Hz)


def read_bag(bag: Path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    imu_t, imu_w = [], []
    pc_t, pc_prof = [], []
    rgb_t, rgb_gray = [], []
    fx = None
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == IMU_T:
            m = deserialize_message(data, get_message(types[topic]))
            imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            imu_w.append((m.angular_velocity.x, m.angular_velocity.y,
                          m.angular_velocity.z))
        elif topic == PC_T:
            m = deserialize_message(data, get_message(types[topic]))
            pc_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            pc_prof.append(azimuth_profile(m))
        elif topic == RGB_T:
            m = deserialize_message(data, get_message(types[topic]))
            rgb_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            img = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, 3)
            rgb_gray.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        elif topic == INFO_T and fx is None:
            m = deserialize_message(data, get_message(types[topic]))
            fx = float(m.k[0])
    return (np.array(imu_t), np.array(imu_w), np.array(pc_t),
            np.array(pc_prof), np.array(rgb_t), rgb_gray, fx)


def azimuth_profile(msg) -> np.ndarray:
    """min range per 1-deg azimuth bin, using the near-horizontal rings."""
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    offs = {f.name: f.offset for f in msg.fields}
    xyz = np.empty((raw.shape[0], 3), dtype=np.float32)
    for i, k in enumerate("xyz"):
        xyz[:, i] = raw[:, offs[k]:offs[k] + 4].copy().view(np.float32).ravel()
    ok = np.isfinite(xyz).all(axis=1)
    xyz = xyz[ok]
    rho = np.linalg.norm(xyz[:, :2], axis=1)
    keep = (rho > 0.3) & (np.abs(xyz[:, 2]) < 0.6 * rho)   # |elev| < ~31 deg
    xyz, rho = xyz[keep], rho[keep]
    az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
    idx = np.clip(((az + 180.0) / 360.0 * NBIN).astype(int), 0, NBIN - 1)
    prof = np.full(NBIN, np.nan, dtype=np.float32)
    order = np.argsort(rho)[::-1]          # last write wins -> min range
    prof[idx[order]] = rho[order]
    return prof


def profile_shift(a: np.ndarray, b: np.ndarray) -> float:
    """circular shift (deg) that best aligns profile a onto b, sub-bin refined."""
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < NBIN * 0.3:
        return np.nan
    a0 = np.where(np.isfinite(a), a, np.nanmedian(a))
    b0 = np.where(np.isfinite(b), b, np.nanmedian(b))
    a0 = a0 - a0.mean()
    b0 = b0 - b0.mean()
    shifts = np.arange(-MAX_SHIFT, MAX_SHIFT + 1)
    scores = np.array([np.dot(np.roll(a0, s), b0) for s in shifts])
    k = int(np.argmax(scores))
    if 0 < k < len(shifts) - 1:
        y0, y1, y2 = scores[k - 1], scores[k], scores[k + 1]
        denom = (y0 - 2 * y1 + y2)
        delta = 0.5 * (y0 - y2) / denom if denom != 0 else 0.0
    else:
        delta = 0.0
    return float(shifts[k] + np.clip(delta, -1, 1))


def camera_yaw_rate(t: np.ndarray, grays: list, fx: float):
    """median horizontal LK flow between consecutive frames -> rad/s."""
    out_t, out_w = [], []
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    for i in range(len(grays) - 1):
        g0, g1 = grays[i], grays[i + 1]
        p0 = cv2.goodFeaturesToTrack(g0, maxCorners=400, qualityLevel=0.01,
                                     minDistance=8)
        dt = t[i + 1] - t[i]
        if p0 is None or dt <= 0:
            out_t.append(0.5 * (t[i] + t[i + 1])); out_w.append(np.nan); continue
        p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, **lk)
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None, **lk)
        good = (st.ravel() == 1) & (st2.ravel() == 1)
        good &= np.linalg.norm((p0r - p0).reshape(-1, 2), axis=1) < 1.0
        if good.sum() < 20:
            out_t.append(0.5 * (t[i] + t[i + 1])); out_w.append(np.nan); continue
        dx = (p1 - p0).reshape(-1, 2)[good, 0]
        out_t.append(0.5 * (t[i] + t[i + 1]))
        out_w.append(float(np.median(dx)) / fx / dt)
    return np.array(out_t), np.array(out_w)


def resample(t, v, grid):
    m = np.isfinite(v)
    if m.sum() < 4:
        return np.full_like(grid, np.nan)
    return np.interp(grid, t[m], v[m], left=np.nan, right=np.nan)


def best_lag(ref, sig, dt, max_lag_s=1.0):
    """lag (s) to add to sig's timestamps so it lines up with ref; also corr."""
    m = np.isfinite(ref) & np.isfinite(sig)
    a = np.where(m, ref, 0.0) - np.nanmean(ref[m])
    b = np.where(m, sig, 0.0) - np.nanmean(sig[m])
    n = int(max_lag_s / dt)
    lags = np.arange(-n, n + 1)
    denom = np.sqrt(np.dot(a, a) * np.dot(b, b))
    if denom == 0:
        return np.nan, np.nan, lags, np.zeros_like(lags, dtype=float)
    corr = np.array([np.dot(a, np.roll(b, k)) / denom for k in lags])
    k = int(np.argmax(np.abs(corr)))
    # parabolic refine on the |corr| peak
    frac = 0.0
    if 0 < k < len(lags) - 1:
        y0, y1, y2 = np.abs(corr[[k - 1, k, k + 1]])
        d = (y0 - 2 * y1 + y2)
        frac = 0.5 * (y0 - y2) / d if d != 0 else 0.0
    return float((lags[k] + np.clip(frac, -1, 1)) * dt), float(corr[k]), lags, corr


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--json")
    ap.add_argument("--npz")
    args = ap.parse_args()

    imu_t, imu_w, pc_t, pc_prof, rgb_t, grays, fx = read_bag(Path(args.bag))
    print(f"# imu={len(imu_t)} pc={len(pc_t)} rgb={len(rgb_t)} fx={fx}", flush=True)

    # ---- LiDAR yaw rate ----
    ly_t, ly_w = [], []
    for i in range(len(pc_t) - 1):
        s = profile_shift(pc_prof[i], pc_prof[i + 1])
        dt = pc_t[i + 1] - pc_t[i]
        ly_t.append(0.5 * (pc_t[i] + pc_t[i + 1]))
        ly_w.append(np.radians(s) / dt if np.isfinite(s) else np.nan)
    ly_t, ly_w = np.array(ly_t), np.array(ly_w)

    # ---- camera yaw rate ----
    cam_t, cam_w = camera_yaw_rate(rgb_t, grays, fx)

    # ---- common grid ----
    dt = 0.01
    t0 = max(imu_t[0], ly_t[0], cam_t[0])
    t1 = min(imu_t[-1], ly_t[-1], cam_t[-1])
    grid = np.arange(t0, t1, dt)
    lid = resample(ly_t, ly_w, grid)
    cam = resample(cam_t, cam_w, grid)

    # pick the IMU axis (and sign) most correlated with LiDAR at zero lag
    axes = {}
    for i, name in enumerate("xyz"):
        s = resample(imu_t, imu_w[:, i], grid)
        m = np.isfinite(s) & np.isfinite(lid)
        c = float(np.corrcoef(s[m], lid[m])[0, 1]) if m.sum() > 10 else 0.0
        axes[name] = c
    ax = max(axes, key=lambda k: abs(axes[k]))
    imu = resample(imu_t, imu_w[:, "xyz".index(ax)], grid)

    res = {"bag": args.bag, "fx": fx, "grid_dt_s": dt,
           "n_grid": int(len(grid)), "imu_axis_used": ax,
           "imu_axis_corr_vs_lidar": {k: round(v, 3) for k, v in axes.items()},
           "signal_rms_rad_s": {
               "imu": round(float(np.nanstd(imu)), 4),
               "lidar": round(float(np.nanstd(lid)), 4),
               "camera": round(float(np.nanstd(cam)), 4)}}

    pairs = {"lidar_vs_imu": (imu, lid), "camera_vs_imu": (imu, cam),
             "camera_vs_lidar": (lid, cam)}
    res["lags"] = {}
    curves = {}
    for name, (ref, sig) in pairs.items():
        lag, corr, lags, cc = best_lag(ref, sig, dt)
        res["lags"][name] = {"lag_ms": round(lag * 1e3, 1), "peak_corr": round(corr, 3),
                             "corr_at_zero_lag": round(float(cc[len(cc) // 2]), 3)}
        curves[name] = cc
    res["lag_convention"] = ("positive lag_ms = the second sensor's signal must be "
                             "delayed by that much to match the first, i.e. its "
                             "stamps are that much early")

    print(json.dumps(res, indent=1))
    if args.json:
        Path(args.json).write_text(json.dumps(res, indent=1))
    if args.npz:
        np.savez(args.npz, grid=grid, imu=imu, lidar=lid, camera=cam,
                 imu_t=imu_t, imu_w=imu_w, ly_t=ly_t, ly_w=ly_w,
                 cam_t=cam_t, cam_w=cam_w, **{f"cc_{k}": v for k, v in curves.items()})


if __name__ == "__main__":
    main()
