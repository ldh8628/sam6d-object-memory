#!/usr/bin/env python3
"""Score ORB-SLAM3 / RTAB-Map / hdl_graph_slam against the Xsens, treated as GT.

The Xsens is a genuine reference for *orientation*: the MTi-630's on-board AHRS
publishes a filtered, gravity-and-magnetometer-anchored quaternion that does not
drift without bound. It is not a reference for *position* — a MEMS IMU has no
position output, and double-integrating its accelerations diverges quadratically
(the script measures that divergence rather than assuming it). So position is
scored the two ways the IMU can legitimately support:

  * vertically, because the AHRS defines the gravity direction and the platform
    ran on a flat floor, so any out-of-plane excursion is real positional error;
  * horizontally, by cross-comparing the three methods after a rigid alignment,
    which bounds their mutual disagreement even though it names no winner.

Every rotation comparison needs the constant IMU->sensor mount rotation X. It is
never measured for this rig, so it is solved for here from the motion itself:
increments obey dR_slam = X' dR_imu X, whose logarithms give a Wahba problem with
a closed-form SVD solution. Residual error after that fit is reported, so a bad X
cannot masquerade as SLAM error.

Run under `conda activate rtabmap` (numpy + scipy + matplotlib).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트
OUT = ROOT / "output_slam" / "260804_office"
METHODS = [("orb3_slam", "ORB-SLAM3", "tab:green"),
           ("rtabmap", "RTAB-Map", "tab:orange"),
           ("hdl_graph_slam", "hdl_graph_slam", "tab:purple")]


# ---------------------------------------------------------------- SO(3) utils
def quat_to_R(q):
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def log_so3(R):
    """(...,3,3) -> (...,3) axis*angle."""
    c = np.clip((np.trace(R, axis1=-2, axis2=-1) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    v = np.stack([R[..., 2, 1] - R[..., 1, 2],
                  R[..., 0, 2] - R[..., 2, 0],
                  R[..., 1, 0] - R[..., 0, 1]], -1)
    s = np.sin(th)
    small = th < 1e-8
    scale = np.where(small, 0.5, th / np.where(small, 1.0, 2.0 * s))
    return v * scale[..., None]


def angle_of(R):
    """(...,3,3) -> rotation angle [rad]."""
    return np.arccos(np.clip((np.trace(R, axis1=-2, axis2=-1) - 1.0) / 2.0, -1.0, 1.0))


def proj_so3(M):
    u, _, vt = np.linalg.svd(M)
    d = np.sign(np.linalg.det(u @ vt))
    return u @ np.diag([1.0, 1.0, d]) @ vt


def kabsch_rot(a, b):
    """rotation Q minimising sum |Q a_i - b_i|^2 for 3-vectors a, b."""
    return proj_so3(b.T @ a)


def R_to_quat(R):
    """(...,3,3) -> (...,4) xyzw, via the numerically safe branch per sample."""
    R = np.asarray(R, float)
    m = R.reshape(-1, 3, 3)
    q = np.empty((len(m), 4))
    tr = m[:, 0, 0] + m[:, 1, 1] + m[:, 2, 2]
    big = tr > 0
    s = np.sqrt(np.maximum(tr[big] + 1.0, 1e-12)) * 2
    q[big] = np.stack([(m[big, 2, 1] - m[big, 1, 2]) / s,
                       (m[big, 0, 2] - m[big, 2, 0]) / s,
                       (m[big, 1, 0] - m[big, 0, 1]) / s, 0.25 * s], -1)
    rest = np.where(~big)[0]
    for i in rest:                              # rare branch; clarity over speed
        d = np.array([m[i, 0, 0], m[i, 1, 1], m[i, 2, 2]])
        k = int(np.argmax(d))
        a, b = (k + 1) % 3, (k + 2) % 3
        s2 = np.sqrt(max(1.0 + d[k] - d[a] - d[b], 1e-12)) * 2
        v = np.zeros(4)
        v[k] = 0.25 * s2
        v[a] = (m[i, a, k] + m[i, k, a]) / s2
        v[b] = (m[i, b, k] + m[i, k, b]) / s2
        v[3] = (m[i, b, a] - m[i, a, b]) / s2
        q[i] = v
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q.reshape(R.shape[:-2] + (4,))


def axis_angle_R(axis, ang):
    """(n,) angles about one fixed axis -> (n,3,3)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    ang = np.asarray(ang, float)[:, None, None]
    return (np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K))


def degauss_yaw(t, q, w, up_world):
    """Strip the magnetometer's yaw correction out of the AHRS orientation.

    Indoors the MTi's heading reference is unusable (the field magnitude here
    swings by 58-73 % peak-to-peak), and the filter's yaw corrections show up as
    an apparent drift of every SLAM method at once. Roll and pitch are kept as
    the AHRS reports them - gravity anchors those and they are not in doubt -
    while yaw is replaced by the integral of the gyro about the gravity axis.
    Over a run this short the gyro's own bias instability is worth well under a
    degree, far below what the magnetometer was injecting.

    Returns the corrected quaternions and how much yaw was removed.
    """
    R = quat_to_R(q)
    up_body = np.einsum('nij,i->nj', R, up_world)      # R' up_world, per sample
    inc = log_so3(np.einsum('nji,njk->nik', R[:-1], R[1:]))
    ahrs = np.concatenate([[0.0], np.cumsum(np.einsum('ni,ni->n', inc, up_body[:-1]))])
    gyro = np.concatenate([[0.0], np.cumsum(
        np.einsum('ni,ni->n', w[:-1], up_body[:-1]) * np.diff(t))])
    delta = ahrs - gyro                                 # what the filter injected
    R_fix = np.einsum('nij,njk->nik', axis_angle_R(up_world, -delta), R)
    return R_to_quat(R_fix), np.degrees(delta)


def slerp_R(t_src, q_src, t_dst):
    """Nearest-neighbour SLERP of a quaternion series onto new stamps."""
    idx = np.clip(np.searchsorted(t_src, t_dst), 1, len(t_src) - 1)
    t0, t1 = t_src[idx - 1], t_src[idx]
    q0, q1 = q_src[idx - 1].copy(), q_src[idx].copy()
    flip = np.sum(q0 * q1, axis=1) < 0
    q1[flip] *= -1.0
    u = np.clip((t_dst - t0) / np.maximum(t1 - t0, 1e-12), 0.0, 1.0)[:, None]
    q = q0 * (1 - u) + q1 * u                    # 200 Hz source: <=2.5 ms gaps,
    q /= np.linalg.norm(q, axis=1, keepdims=True)  # so nlerp == slerp here
    return quat_to_R(q)


# ---------------------------------------------------------------------- I/O
def load_tum(path: Path):
    rows = []
    for line in path.read_text(errors="replace").splitlines():
        p = line.split()
        if len(p) >= 8 and not line.startswith("#"):
            try:
                rows.append([float(v) for v in p[:8]])
            except ValueError:
                continue
    if not rows:
        return None
    a = np.asarray(rows)
    a = a[np.argsort(a[:, 0])]
    return a[:, 0], a[:, 1:4], a[:, 4:8]


# ------------------------------------------------------------- rotation score
def rotation_scores(t, R_slam, R_imu, up_world, deltas=(1.0, 5.0, 10.0)):
    """Absolute + relative rotation error against the AHRS."""
    # --- solve the mount rotation X from increments, on a ~0.2 s stride so the
    #     log vectors carry real rotation rather than quantisation noise
    stride = max(1, int(round(0.2 / max(np.median(np.diff(t)), 1e-6))))
    i0 = np.arange(0, len(t) - stride)
    i1 = i0 + stride
    r_s = log_so3(np.einsum('nji,njk->nik', R_slam[i0], R_slam[i1]))
    r_i = log_so3(np.einsum('nji,njk->nik', R_imu[i0], R_imu[i1]))
    keep = (np.linalg.norm(r_i, axis=1) > np.deg2rad(0.5))
    if keep.sum() < 10:
        keep = np.ones(len(r_i), bool)
    X = kabsch_rot(r_i[keep], r_s[keep]).T          # dR_slam = X' dR_imu X
    fit_res = np.degrees(np.linalg.norm(r_s[keep] - (X.T @ r_i[keep].T).T, axis=1))
    # Observability warning: rotation-only hand-eye needs two non-parallel
    # excited axes. A platform spinning about the vertical excites one, leaving
    # X free up to a rotation about it. sv[1]/sv[0] near 0 means exactly that,
    # so the caller can bound how much the residual X ambiguity can move things.
    sv = np.linalg.svd(r_i[keep], compute_uv=False)
    sv = sv / max(sv[0], 1e-12)

    # --- world alignment A, then GT expressed in the SLAM world frame
    # A = argmin_A sum |R_slam - A R_imu X|^2_F  ->  proj_SO3(sum R_slam X' R_imu')
    A = proj_so3(np.einsum('nij,njk->ik', R_slam, np.einsum('kj,nlk->njl', X, R_imu)))
    R_gt = np.einsum('ij,njk->nik', A, np.einsum('nij,jk->nik', R_imu, X))
    are = np.degrees(angle_of(np.einsum('nji,njk->nik', R_slam, R_gt)))

    out = {
        "n_pairs": int(len(t)),
        "mount_fit_residual_deg_rms": round(float(np.sqrt((fit_res ** 2).mean())), 4),
        "mount_axis_observability_sv": [round(float(v), 4) for v in sv],
        "are_mean_deg": round(float(are.mean()), 3),
        "are_rmse_deg": round(float(np.sqrt((are ** 2).mean())), 3),
        "are_median_deg": round(float(np.median(are)), 3),
        "are_max_deg": round(float(are.max()), 3),
        "are_final_deg": round(float(are[-1]), 3),
    }

    # --- split the residual into gravity-referenced components. ARE is an
    #     unsigned angle, so a slow yaw drift and a constant tilt are
    #     indistinguishable in it; the signed yaw component separates them.
    #     Note the two halves are not equally hard GT: roll/pitch are anchored
    #     by gravity and cannot drift, whereas AHRS yaw leans on the
    #     magnetometer, which indoors is the weaker reference.
    up = A @ up_world
    up = up / np.linalg.norm(up)
    e = log_so3(np.einsum('nij,nkj->nik', R_slam, R_gt))     # world-frame residual
    yaw_e = np.degrees(e @ up)
    tilt_e = np.degrees(np.linalg.norm(e - np.outer(e @ up, up), axis=1))
    out["yaw_err_deg"] = {
        "mean": round(float(yaw_e.mean()), 3),
        "rmse_about_mean": round(float(yaw_e.std()), 3),
        "ptp": round(float(np.ptp(yaw_e)), 3),
        "final_minus_initial": round(float(yaw_e[-1] - yaw_e[0]), 3),
        "drift_deg_per_min": (round(float(np.polyfit(t - t[0], yaw_e, 1)[0] * 60.0), 3)
                              if t[-1] - t[0] > 5 else None)}
    out["tilt_err_deg"] = {
        "mean": round(float(tilt_e.mean()), 3),
        "rmse": round(float(np.sqrt((tilt_e ** 2).mean())), 3),
        "max": round(float(tilt_e.max()), 3),
        "drift_deg_per_min": (round(float(np.polyfit(t - t[0], tilt_e, 1)[0] * 60.0), 3)
                              if t[-1] - t[0] > 5 else None)}

    # --- relative (drift-free) rotation error over fixed time gaps
    rre = {}
    for d in deltas:
        j = np.searchsorted(t, t + d)
        ok = (j < len(t)) & (np.abs(np.take(t, np.clip(j, 0, len(t) - 1)) - (t + d)) < 0.35 * d)
        if ok.sum() < 5:
            continue
        a_, b_ = np.where(ok)[0], j[ok]
        dS = np.einsum('nji,njk->nik', R_slam[a_], R_slam[b_])
        dI = np.einsum('nji,njk->nik', R_imu[a_], R_imu[b_])
        dG = np.einsum('ji,njk->nik', X, np.einsum('nij,jk->nik', dI, X))
        e = np.degrees(angle_of(np.einsum('nji,njk->nik', dS, dG)))
        rre[f"{d:g}s"] = {"n": int(ok.sum()),
                          "rmse_deg": round(float(np.sqrt((e ** 2).mean())), 3),
                          "mean_deg": round(float(e.mean()), 3),
                          "max_deg": round(float(e.max()), 3)}
    out["rre"] = rre
    return out, are, X, A


# ------------------------------------------------------------- position score
def vertical_scores(xyz, A, up_world):
    """Out-of-plane error, using the AHRS gravity direction as the GT normal.

    A maps the AHRS world frame into the method's own world frame, so the GT
    vertical lands in the method's coordinates without needing its convention.
    """
    up_slam = A @ up_world
    up_slam = up_slam / np.linalg.norm(up_slam)
    z = (xyz - xyz[0]) @ up_slam
    horiz = (xyz - xyz[0]) - np.outer(z, up_slam)
    return {
        "vertical_drift_final_m": round(float(z[-1]), 4),
        "vertical_rms_m": round(float(np.sqrt((z ** 2).mean())), 4),
        "vertical_ptp_m": round(float(np.ptp(z)), 4),
        "vertical_max_abs_m": round(float(np.abs(z).max()), 4),
        "horizontal_path_len_m": round(float(
            np.linalg.norm(np.diff(horiz, axis=0), axis=1).sum()), 3),
    }, z


def strapdown(t_i, q_i, a_i):
    """Double-integrate the IMU with an oracle-debiased gravity vector.

    The mean world acceleration over a run that starts and ends at rest *is* the
    gravity vector, so removing it is the most favourable debiasing available
    offline — strictly better than anything a live filter could do. Whatever
    divergence survives is the floor on IMU-only position error.
    """
    R = quat_to_R(q_i)
    aw = np.einsum('nij,nj->ni', R, a_i)
    g = aw.mean(axis=0)
    lin = aw - g
    dt = np.diff(t_i)[:, None]
    v = np.concatenate([[[0, 0, 0]], np.cumsum(0.5 * (lin[:-1] + lin[1:]) * dt, axis=0)])
    p = np.concatenate([[[0, 0, 0]], np.cumsum(0.5 * (v[:-1] + v[1:]) * dt, axis=0)])
    return p, v, g


def skew(v):
    z = np.zeros(len(v))
    return np.stack([np.stack([z, -v[:, 2], v[:, 1]], -1),
                     np.stack([v[:, 2], z, -v[:, 0]], -1),
                     np.stack([-v[:, 1], v[:, 0], z], -1)], -2)


def accel_scale_check(t, xyz, A, t_i, q_i, w_i, a_i, g_vec, fs=30.0, cutoff=2.0):
    """Metric-scale check on translation, using the IMU's specific force.

    This is the one horizontal positional quantity the IMU really observes:
    while the platform turns, the accelerometer sees the centripetal term, so
    the SLAM path's second derivative must match it in magnitude, not merely in
    shape. A fitted scale of 1 means the reconstruction is correctly metric.

    The camera does not sit on the IMU, and at these rates the lever arm is not
    a detail: w^2*r with w~1.2 rad/s and r~0.3 m is ~0.4 m/s^2, the same size as
    the whole signal. So the unknown lever arm r is solved for jointly, which
    keeps the model honest and yields a physically checkable by-product:

        a_cam = s * a_imu + R_wb (skew(alpha) + skew(w)^2) r

    is linear in (s, s*r), so one least-squares solve gives both.

    Only usable for a pose stream fast enough to resolve the motion; at ~1 Hz
    (RTAB-Map, hdl_graph_slam here) the second derivative is pure aliasing.
    """
    from scipy.signal import butter, filtfilt, savgol_filter
    if len(t) < 200 or (t[-1] - t[0]) < 10:
        return None
    grid = np.arange(t[0], t[-1], 1.0 / fs)
    p = np.stack([np.interp(grid, t, xyz[:, k]) for k in range(3)], axis=1)
    win = int(round(0.5 * fs)) | 1
    acc_slam = savgol_filter(p, win, 3, deriv=2, delta=1.0 / fs, axis=0)

    R_i = quat_to_R(q_i)
    aw_s = ((np.einsum('nij,nj->ni', R_i, a_i) - g_vec) @ A.T)      # SLAM world
    acc_imu = np.stack([np.interp(grid, t_i, aw_s[:, k]) for k in range(3)], 1)

    # body-frame rate and its derivative, plus the IMU orientation in SLAM world
    w = np.stack([np.interp(grid, t_i, w_i[:, k]) for k in range(3)], 1)
    w = savgol_filter(w, win, 3, axis=0)
    al = savgol_filter(w, win, 3, deriv=1, delta=1.0 / fs, axis=0)
    R_wb = np.einsum('ij,njk->nik', A, np.stack(
        [quat_to_R(q_i[np.clip(np.searchsorted(t_i, g), 0, len(t_i) - 1)])
         for g in grid]))
    M = np.einsum('nij,njk->nik', R_wb, skew(al) + np.einsum(
        'nij,njk->nik', skew(w), skew(w)))

    b, a = butter(2, cutoff / (fs / 2), btype="low")     # all terms, same band
    acc_slam = filtfilt(b, a, acc_slam, axis=0)
    acc_imu = filtfilt(b, a, acc_imu, axis=0)
    M = filtfilt(b, a, M, axis=0)
    e = int(fs)                                          # drop filter transients
    acc_slam, acc_imu, M = acc_slam[e:-e], acc_imu[e:-e], M[e:-e]

    D = np.concatenate([acc_imu.reshape(-1, 1), M.reshape(-1, 3)], axis=1)
    y = acc_slam.reshape(-1)
    sol, *_ = np.linalg.lstsq(D, y, rcond=None)
    s, r_scaled = float(sol[0]), sol[1:]
    resid = y - D @ sol

    # Is any of this meaningful? Compare the rigid-body signal being sought
    # against the two floors that swamp it. Reporting a scale without this
    # check would dress up noise as a measurement.
    speed = float(np.median(np.linalg.norm(np.diff(p, axis=0), axis=1)) * fs)
    rate = float(np.median(np.linalg.norm(w, axis=1)))
    signal = speed * rate                             # centripetal term sought
    vib = float(np.sqrt((np.linalg.norm(
        (np.einsum('nij,nj->ni', quat_to_R(q_i), a_i) - g_vec), axis=1) ** 2).mean()))
    usable = signal > 3.0 * vib
    return {"usable": usable,
            "verdict": ("ok" if usable else
                        "not usable: platform vibration exceeds the rigid-body "
                        "acceleration being measured, so the fitted scale and "
                        "lever arm below are noise, not measurements"),
            "expected_centripetal_ms2": round(signal, 4),
            "imu_linear_accel_rms_ms2": round(vib, 4),
            "signal_to_vibration": round(signal / vib, 4) if vib else None,
            "scale_slam_over_imu": round(s, 4),
            "lever_arm_m": [round(float(v), 4) for v in (r_scaled / s if s else r_scaled)],
            "correlation_after_fit": round(float(np.corrcoef(y, D @ sol)[0, 1]), 4),
            "rms_signal_ms2": round(float(np.sqrt((y ** 2).mean())), 4),
            "rms_residual_ms2": round(float(np.sqrt((resid ** 2).mean())), 4),
            "band_hz": cutoff, "n": int(len(y) // 3)}


def umeyama_rigid(src, dst):
    """Rigid (no-scale) alignment src -> dst; both sensors are metric."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    R = proj_so3((dst - mu_d).T @ (src - mu_s))
    return R, mu_d - R @ mu_s


# ---------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", nargs="+", default=["longcircle2", "stablization"])
    ap.add_argument("--imu-dir", default=str(OUT / "gt_eval"))
    ap.add_argument("--out", default=str(OUT / "gt_eval"))
    ap.add_argument("--yaw-source", choices=["gyro", "ahrs"], default="gyro",
                    help="gyro = AHRS roll/pitch + gyro-integrated yaw (default, "
                         "the magnetometer is unusable indoors); ahrs = raw filter")
    ap.add_argument("--max-dt", type=float, default=0.02,
                    help="max |t_slam - t_imu| when associating [s]")
    args = ap.parse_args()
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    report = {}
    for session in args.sessions:
        z = np.load(Path(args.imu_dir) / f"imu_{session}.npz")
        t_i, q_i, a_i = z["t"], z["q"], z["a"]
        R_i_all = quat_to_R(q_i)

        # gravity ("up") expressed in the AHRS world frame, from the data itself
        aw_mean = np.einsum('nij,nj->ni', R_i_all, a_i).mean(axis=0)
        up_world = aw_mean / np.linalg.norm(aw_mean)

        # The reference actually used. "gyro" keeps the AHRS roll/pitch but
        # rebuilds yaw from the gyro, because the magnetometer is disturbed
        # indoors; see check_ahrs_yaw.py for the evidence.
        q_gyro, mag_yaw_injected = degauss_yaw(t_i, q_i, z["w"], up_world)
        q_ref = q_i if args.yaw_source == "ahrs" else q_gyro

        p_imu, v_imu, g_vec = strapdown(t_i, q_i, a_i)
        srep = {"imu": {
            "n": int(len(t_i)),
            "rate_hz": round(float(len(t_i) / (t_i[-1] - t_i[0])), 1),
            "gravity_world": [round(float(v), 4) for v in g_vec],
            "gravity_norm": round(float(np.linalg.norm(g_vec)), 4),
            "strapdown_final_error_m": round(float(np.linalg.norm(p_imu[-1])), 2),
            "strapdown_max_excursion_m": round(
                float(np.linalg.norm(p_imu, axis=1).max()), 2),
            "strapdown_at_s": {},
            "yaw_source": args.yaw_source,
            "mag_yaw_injected_total_deg": round(float(mag_yaw_injected[-1]), 2),
            "mag_yaw_injected_rate_deg_per_min": round(float(
                np.polyfit(t_i - t_i[0], mag_yaw_injected, 1)[0] * 60.0), 2),
        }}
        for mark in (1.0, 5.0, 10.0, 25.0, 72.0):
            if t_i[-1] - t_i[0] >= mark:
                k = int(np.searchsorted(t_i, t_i[0] + mark))
                srep["imu"]["strapdown_at_s"][f"{mark:g}"] = round(
                    float(np.linalg.norm(p_imu[k])), 3)

        fig, axes = plt.subplots(1, 3, figsize=(18, 5))
        aligned = {}
        for key, label, colour in METHODS:
            p = OUT / key / session / "trajectory.txt"
            data = load_tum(p) if p.exists() else None
            if data is None:
                srep[key] = {"error": f"missing {p}"}
                continue
            t, xyz, quat = data
            keep = (t >= t_i[0]) & (t <= t_i[-1])
            near = np.abs(t_i[np.clip(np.searchsorted(t_i, t), 0, len(t_i) - 1)] - t)
            keep &= near < args.max_dt
            t, xyz, quat = t[keep], xyz[keep], quat[keep]
            if len(t) < 8:
                srep[key] = {"error": f"only {len(t)} poses overlap the IMU"}
                continue
            R_s = quat_to_R(quat)
            R_ref = slerp_R(t_i, q_ref, t)

            rot, are, X, A = rotation_scores(t, R_s, R_ref, up_world)

            # Is the residual a real attitude error, or just latency? Under a
            # pure time offset the error tracks angular rate, so re-scoring at
            # shifted stamps dips sharply at the true offset. A minimum at ~0
            # says the sync is right and the error is the method's own.
            sweep = []
            for sh in np.arange(-0.12, 0.1201, 0.01):
                ok = (t + sh >= t_i[0]) & (t + sh <= t_i[-1])
                if ok.sum() < 8:
                    continue
                r2, _, _, _ = rotation_scores(t[ok], R_s[ok],
                                              slerp_R(t_i, q_ref, t[ok] + sh), up_world)
                sweep.append((round(float(sh), 3), r2["are_rmse_deg"]))
            best = min(sweep, key=lambda p: p[1]) if sweep else (None, None)
            rot["time_offset_sweep"] = {
                "best_shift_s": best[0], "are_rmse_at_best_deg": best[1],
                "are_rmse_at_zero_deg": dict(sweep).get(0.0),
                "curve": sweep}
            vert, zz = vertical_scores(xyz, A, up_world)
            scal = accel_scale_check(t, xyz, A, t_i, q_ref, z["w"], a_i, g_vec)
            entry = {"poses_scored": int(len(t)),
                     "duration_s": round(float(t[-1] - t[0]), 2),
                     "rotation_vs_imu_gt": rot,
                     "position_vertical_vs_gravity_gt": vert,
                     "translation_scale_vs_imu": scal,
                     "start_end_gap_m": round(float(np.linalg.norm(xyz[-1] - xyz[0])), 4),
                     "path_length_m": round(float(
                         np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()), 3)}
            srep[key] = entry
            aligned[key] = (t, xyz)
            axes[0].plot(t - t[0], are, color=colour, lw=1.2,
                         label=f"{label} (RMSE {rot['are_rmse_deg']:.2f}°)")
            axes[1].plot(t - t[0], zz, color=colour, lw=1.2,
                         label=f"{label} ({vert['vertical_max_abs_m']:.3f} m max)")

        # --- mutual horizontal agreement: rigidly align each pair on shared stamps
        pair = {}
        keys = [k for k, _, _ in METHODS if k in aligned]
        for ia in range(len(keys)):
            for ib in range(ia + 1, len(keys)):
                ka, kb = keys[ia], keys[ib]
                ta, pa = aligned[ka]
                tb, pb = aligned[kb]
                j = np.clip(np.searchsorted(ta, tb), 0, len(ta) - 1)
                ok = np.abs(ta[j] - tb) < 0.05
                if ok.sum() < 8:
                    continue
                src, dst = pb[ok], pa[j[ok]]
                R, tr = umeyama_rigid(src, dst)
                e = np.linalg.norm((src @ R.T + tr) - dst, axis=1)
                pair[f"{ka}__vs__{kb}"] = {
                    "n": int(ok.sum()),
                    "ate_rmse_m": round(float(np.sqrt((e ** 2).mean())), 4),
                    "ate_mean_m": round(float(e.mean()), 4),
                    "ate_max_m": round(float(e.max()), 4)}
        srep["pairwise_rigid_ate"] = pair

        axes[2].plot(t_i - t_i[0], np.linalg.norm(p_imu, axis=1), "k-", lw=1.4,
                     label="IMU strapdown |p| (oracle-debiased)")
        axes[2].set_yscale("log")
        for ax, ttl, yl in ((axes[0], "absolute rotation error vs Xsens AHRS", "deg"),
                            (axes[1], "vertical error vs gravity plane", "m"),
                            (axes[2], "IMU dead-reckoned position magnitude", "m")):
            ax.set_title(f"{session}: {ttl}", fontsize=10)
            ax.set_xlabel("t [s]"); ax.set_ylabel(yl)
            ax.grid(alpha=0.3); ax.legend(fontsize=8)
        fig.tight_layout()
        png = outdir / f"{session}_vs_imu.png"
        fig.savefig(png, dpi=130)
        plt.close(fig)
        srep["figure"] = str(png)
        report[session] = srep
        print(f"-> {png}")

    (outdir / "eval_vs_imu.json").write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
