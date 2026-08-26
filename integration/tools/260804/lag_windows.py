#!/usr/bin/env python3
"""Sliding-window lag between the three yaw-rate signals produced by
sync_check.py.  A constant offset means a fixed pipeline latency (calibratable);
a ramp means the clocks are actually drifting apart."""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np


def lag_of(ref, sig, dt, max_lag_s=0.5):
    m = np.isfinite(ref) & np.isfinite(sig)
    if m.sum() < 0.5 * len(ref):
        return np.nan, np.nan
    a = np.where(m, ref, 0.0)
    b = np.where(m, sig, 0.0)
    a = a - a[m].mean()
    b = b - b[m].mean()
    n = int(max_lag_s / dt)
    lags = np.arange(-n, n + 1)
    den = np.sqrt(np.dot(a, a) * np.dot(b, b))
    if den == 0:
        return np.nan, np.nan
    cc = np.array([np.dot(a, np.roll(b, k)) / den for k in lags])
    k = int(np.argmax(np.abs(cc)))
    frac = 0.0
    if 0 < k < len(lags) - 1:
        y0, y1, y2 = np.abs(cc[[k - 1, k, k + 1]])
        d = y0 - 2 * y1 + y2
        frac = 0.5 * (y0 - y2) / d if d != 0 else 0.0
    return float((lags[k] + np.clip(frac, -1, 1)) * dt * 1e3), float(cc[k])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--win", type=float, default=5.0)
    ap.add_argument("--step", type=float, default=2.0)
    ap.add_argument("--json")
    a = ap.parse_args()
    z = np.load(a.npz)
    grid, dt = z["grid"], 0.01
    sig = {"imu": z["imu"], "lidar": z["lidar"], "camera": z["camera"]}

    nwin, nstep = int(a.win / dt), int(a.step / dt)
    out = {"window_s": a.win, "step_s": a.step, "pairs": {}}
    for name, (r, s) in {"lidar_vs_imu": ("imu", "lidar"),
                         "camera_vs_imu": ("imu", "camera"),
                         "camera_vs_lidar": ("lidar", "camera")}.items():
        rows = []
        for i in range(0, len(grid) - nwin, nstep):
            sl = slice(i, i + nwin)
            lag, corr = lag_of(sig[r][sl], sig[s][sl], dt)
            act = float(np.nanstd(sig[r][sl]))
            rows.append({"t_mid_s": round(float(grid[i] + a.win / 2 - grid[0]), 2),
                         "lag_ms": None if np.isnan(lag) else round(lag, 1),
                         "corr": None if np.isnan(corr) else round(corr, 3),
                         "ref_rms": round(act, 3)})
        # only trust windows where there is real motion and a clean peak
        good = [r_ for r_ in rows
                if r_["lag_ms"] is not None and r_["ref_rms"] > 0.15
                and abs(r_["corr"]) > 0.8]
        lags = np.array([r_["lag_ms"] for r_ in good])
        ts = np.array([r_["t_mid_s"] for r_ in good])
        slope = float(np.polyfit(ts, lags, 1)[0]) if len(lags) > 3 else float("nan")
        out["pairs"][name] = {
            "n_windows": len(rows), "n_trusted": len(good),
            "lag_ms_median": None if not len(lags) else round(float(np.median(lags)), 1),
            "lag_ms_mean": None if not len(lags) else round(float(lags.mean()), 1),
            "lag_ms_std": None if not len(lags) else round(float(lags.std()), 1),
            "lag_ms_min": None if not len(lags) else round(float(lags.min()), 1),
            "lag_ms_max": None if not len(lags) else round(float(lags.max()), 1),
            "drift_ms_per_s": None if np.isnan(slope) else round(slope, 3),
            "windows": rows,
        }
    txt = json.dumps(out, indent=1)
    if a.json:
        open(a.json, "w").write(txt)
    for k, v in out["pairs"].items():
        print(f"{k:18s} n={v['n_trusted']}/{v['n_windows']} "
              f"median={v['lag_ms_median']} ms  mean={v['lag_ms_mean']} "
              f"std={v['lag_ms_std']}  range=[{v['lag_ms_min']},{v['lag_ms_max']}] "
              f"drift={v['drift_ms_per_s']} ms/s")


if __name__ == "__main__":
    sys.exit(main())
