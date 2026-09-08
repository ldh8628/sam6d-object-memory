#!/usr/bin/env python3
"""Fit a constant offset + linear drift between each sensor's yaw-rate signal
and the IMU's, using the raw (non-resampled) sample times from sync_check.py.

Model:  sig(t_k)  ~=  s * imu( t_k + a + b*(t_k - t_mid) ) + c
  a [ms]   constant stamp offset (positive: the sensor's stamps are EARLY)
  b [ms/s] relative clock drift
The IMU reference is boxcar-averaged over each sensor's own integration window
(100 ms for LiDAR, one frame interval for the camera) so we compare like with
like; boxcar averaging is symmetric and adds no phase.

Uncertainty comes from a moving-block bootstrap over 2 s blocks, which keeps the
residual autocorrelation that makes naive per-sample errors far too optimistic.
"""
from __future__ import annotations

import argparse
import json

import numpy as np
from scipy.optimize import least_squares


def imu_boxcar(imu_t, imu_w, centers, win):
    """mean IMU rate over [c-win/2, c+win/2] for each centre, via the integral."""
    csum = np.concatenate([[0.0], np.cumsum(np.diff(imu_t) * 0.5 *
                                            (imu_w[1:] + imu_w[:-1]))])
    lo = np.interp(centers - win / 2, imu_t, csum)
    hi = np.interp(centers + win / 2, imu_t, csum)
    return (hi - lo) / win


def fit(sig_t, sig_w, imu_t, imu_w, win, with_drift=True):
    m = np.isfinite(sig_w)
    st, sw = sig_t[m], sig_w[m]
    # keep only samples with enough IMU margin for the largest shift we allow
    pad = 0.5 + win
    k = (st > imu_t[0] + pad) & (st < imu_t[-1] - pad)
    st, sw = st[k], sw[k]
    tmid = 0.5 * (st[0] + st[-1])

    def resid(p):
        a, b, s, c = p if with_drift else (p[0], 0.0, p[1], p[2])
        shift = a + b * (st - tmid)
        return s * imu_boxcar(imu_t, imu_w, st + shift, win) + c - sw

    p0 = [0.0, 0.0, np.sign(np.corrcoef(
        imu_boxcar(imu_t, imu_w, st, win), sw)[0, 1]), 0.0]
    if not with_drift:
        p0 = [p0[0], p0[2], p0[3]]
    r = least_squares(resid, p0, method="lm", max_nfev=4000)
    a, b, s, c = (r.x if with_drift else (r.x[0], 0.0, r.x[1], r.x[2]))
    res = resid(r.x)
    rms = float(np.sqrt(np.mean(res ** 2)))
    ss = 1 - np.sum(res ** 2) / np.sum((sw - sw.mean()) ** 2)
    return dict(a_ms=a * 1e3, b_ms_per_s=b * 1e3, scale=s, bias=c,
                rms_rad_s=rms, r2=float(ss), n=len(st)), (st, sw)


def bootstrap(sig_t, sig_w, imu_t, imu_w, win, n_boot=60, block_s=2.0, seed=0):
    rng = np.random.default_rng(seed)
    m = np.isfinite(sig_w)
    st, sw = sig_t[m], sig_w[m]
    pad = 0.5 + win
    k = (st > imu_t[0] + pad) & (st < imu_t[-1] - pad)
    st, sw = st[k], sw[k]
    dur = st[-1] - st[0]
    nblk = max(3, int(dur / block_s))
    edges = np.linspace(st[0], st[-1], nblk + 1)
    blocks = [(edges[i], edges[i + 1]) for i in range(nblk)]
    out = []
    for _ in range(n_boot):
        pick = rng.integers(0, nblk, nblk)
        idx = np.concatenate([np.where((st >= blocks[i][0]) & (st < blocks[i][1]))[0]
                              for i in pick])
        if len(idx) < 20:
            continue
        try:
            f, _ = fit(st[idx], sw[idx], imu_t, imu_w, win)
            out.append((f["a_ms"], f["b_ms_per_s"]))
        except Exception:                                     # noqa: BLE001
            continue
    arr = np.asarray(out)
    if not len(arr):
        return {}
    return {"a_ms_std": float(arr[:, 0].std()),
            "a_ms_ci95": [float(np.percentile(arr[:, 0], 2.5)),
                          float(np.percentile(arr[:, 0], 97.5))],
            "b_ms_per_s_std": float(arr[:, 1].std()),
            "b_ms_per_s_ci95": [float(np.percentile(arr[:, 1], 2.5)),
                                float(np.percentile(arr[:, 1], 97.5))],
            "n_boot": int(len(arr))}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--json")
    ap.add_argument("--boot", type=int, default=60)
    args = ap.parse_args()
    z = np.load(args.npz)
    imu_t, imu_w = z["imu_t"], z["imu_w"][:, 2]      # z axis chosen by sync_check
    out = {"npz": args.npz, "note": ("a_ms > 0 => that sensor's stamps are EARLY "
                                     "w.r.t. the Xsens; < 0 => LATE")}
    for name, (t, w, win) in {
            "lidar": (z["ly_t"], z["ly_w"], 0.1),
            "camera": (z["cam_t"], z["cam_w"], 1 / 30.0)}.items():
        f, _ = fit(t, w, imu_t, imu_w, win)
        f0, _ = fit(t, w, imu_t, imu_w, win, with_drift=False)
        b = bootstrap(t, w, imu_t, imu_w, win, n_boot=args.boot)
        out[name] = {"with_drift": {k: round(v, 4) for k, v in f.items()},
                     "offset_only": {k: round(v, 4) for k, v in f0.items()},
                     "bootstrap": {k: (round(v, 3) if isinstance(v, float)
                                       else [round(x, 3) for x in v]
                                       if isinstance(v, list) else v)
                                   for k, v in b.items()}}
    print(json.dumps(out, indent=1))
    if args.json:
        open(args.json, "w").write(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
