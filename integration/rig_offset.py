#!/usr/bin/env python3
"""rig_offset.py — is the SAM camera's pose relative to the SLAM camera constant?

Two cameras bolted to one cart must keep a fixed relative pose. Each SLAM
algorithm is run separately on each camera's bag, so the two trajectories live
in DIFFERENT map frames:

    T_mapA_camA(t) . X  =  M . T_mapB_camB(t)          X = T_camA_camB (rig)
                                                       M = T_mapA_mapB (frames)

Both unknowns are recovered together (the robot-world / hand-eye AX = ZB
problem), and then the quantity that actually matters is measured:

    X(t) = T_mapA_camA(t)^-1 . M . T_mapB_camB(t)

If the algorithm's poses were perfect, X(t) would be the same matrix at every
instant. Its spread over time is therefore a GT-free accuracy proxy: it charges
an algorithm for drift, for scale error and for tracking noise, without needing
any ground truth. Comparing that spread between algorithms is the point.

Caveat, stated because it is easy to forget: on a cart that only ever yaws about
the vertical, the component of X along that axis (the height difference) is
unobservable from motion alone, exactly as in plain hand-eye. It is returned as
the minimum-norm solution and marked in the output. It does NOT inflate the
residual — an unobservable direction produces no error signal — so the constancy
verdict stays valid.

    python3 integration/rig_offset.py --a slam/trajectory.txt --b sam/trajectory.txt
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


# --------------------------------------------------------------------------
def read_tum(path):
    ts, T = [], []
    for line in Path(path).read_text().splitlines():
        p = line.split()
        if len(p) != 8:
            continue
        t = float(p[0])
        q = np.array([float(v) for v in p[4:8]])
        n = np.linalg.norm(q)
        if n == 0 or not np.isfinite(n):
            continue
        x, y, z, w = q / n
        M = np.eye(4)
        M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        M[:3, 3] = [float(p[1]), float(p[2]), float(p[3])]
        if not np.isfinite(M).all():
            continue
        ts.append(t)
        T.append(M)
    o = np.argsort(ts, kind="stable")
    ts, T = np.array(ts)[o], np.array(T)[o]
    # ORB-SLAM3 appends a fresh block of poses after a map reset, so a file can
    # carry the same timestamp twice with different poses. Keep the last one
    # (the surviving map's estimate) rather than interpolating between maps.
    if len(ts) > 1:
        keep = np.append(np.diff(ts) > 1e-6, True)
        ts, T = ts[keep], T[keep]
    return ts, T


def rot_log(R):
    """rotation matrix -> rotation vector (axis * angle)."""
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    if th < 1e-8:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (th / (2 * np.sin(th)))


def rot_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def proj_SO3(M):
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def slerp_interp(ts, T, t, max_gap=0.35):
    """pose of trajectory (ts, T) at time t; None outside or across a gap."""
    i = int(np.searchsorted(ts, t))
    if i == 0 or i >= len(ts):
        return None
    t0, t1 = ts[i - 1], ts[i]
    if t1 - t0 > max_gap:
        return None
    u = (t - t0) / (t1 - t0)
    A, B = T[i - 1], T[i]
    R = proj_SO3((1 - u) * A[:3, :3] + u * B[:3, :3])     # chordal, fine for 30-60 ms
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = (1 - u) * A[:3, 3] + u * B[:3, 3]
    return out


def pair_poses(ts_a, T_a, ts_b, T_b, tau, stride=1, max_gap=0.35):
    """(A_i, B_i) sampled at the B timestamps shifted by tau."""
    A, B, stamps = [], [], []
    for k in range(0, len(ts_b), stride):
        t = ts_b[k]
        a = slerp_interp(ts_a, T_a, t + tau, max_gap)
        if a is None:
            continue
        A.append(a)
        B.append(T_b[k])
        stamps.append(t)
    return np.array(A), np.array(B), np.array(stamps)


# --------------------------------------------------------------------------
def solve_axzb(A, B, weights=None):
    """AX = ZB.  Returns (X, M) with X = T_camA_camB, M = T_mapA_mapB.

    Rotation part is the standard linear (Shah / Li-Wang) formulation. From
    R_M = R_A R_X R_B^T and vec(PQS) = (S^T (x) P) vec(Q) with COLUMN-major vec,

        [ R_B (x) R_A ,  -I_9 ] [vec R_X ; vec R_M] = 0,

    so the answer is the right null vector of the stacked matrix, projected onto
    SO(3). This uses every absolute orientation instead of differencing nearby
    poses, which is why it survives the small frame-to-frame rotations here
    (an axis-matching solver on 0.1 s baselines was 11 deg off on a case whose
    answer is known independently).
    """
    n = len(A)
    G = np.zeros((9 * n, 18))
    for i in range(n):
        Ra, Rb = A[i][:3, :3], B[i][:3, :3]
        G[9 * i:9 * i + 9, :9] = np.kron(Rb, Ra)
        G[9 * i:9 * i + 9, 9:] = -np.eye(9)
    if weights is not None:
        G *= np.repeat(weights, 9)[:, None]
    _, _, Vt = np.linalg.svd(G, full_matrices=False)
    v = Vt[-1]
    R_X = proj_SO3(v[:9].reshape(3, 3, order="F"))
    R_M = proj_SO3(v[9:].reshape(3, 3, order="F"))
    # the null vector is sign-ambiguous; proj_SO3 forces det=+1 on each block
    # independently, so check the pair is consistent and flip if not
    if np.mean([rot_angle_deg((A[i][:3, :3] @ R_X @ B[i][:3, :3].T).T @ R_M)
                for i in range(0, n, max(1, n // 50))]) > 90:
        R_X = proj_SO3(-v[:9].reshape(3, 3, order="F"))
        R_M = proj_SO3(-v[9:].reshape(3, 3, order="F"))
    # The 18-vector null solution weighs R_X and R_M against each other, which
    # biases both by a few degrees. Polish with alternating chordal means: each
    # half-step is the closed-form optimal rotation given the other.
    # Alternating chordal means (each half-step optimal given the other), Huber
    # weighted. It only finds a LOCAL optimum, and on the self-test the linear
    # start converged 3.5 deg away from a better basin, so start it twice — from
    # the linear solution and from R_M = I (the "same frame" hypothesis) — and
    # keep whichever ends with the lower cost.
    RA = A[:, :3, :3]
    RB = B[:, :3, :3]
    tA = A[:, :3, 3]
    tB = B[:, :3, 3]

    def errs(R_X_i, R_M_i):
        C = np.einsum("ji,njk,kl,nml->nim", R_M_i, RA, R_X_i, RB)
        tr = np.clip((np.trace(C, axis1=1, axis2=2) - 1) / 2, -1, 1)
        return np.degrees(np.arccos(tr))

    def polish(R_X0, R_M0):
        R_X_i, R_M_i, w = R_X0, R_M0, np.ones(n)
        for it in range(40):
            R_M_i = proj_SO3(np.einsum("n,nij,jk,nlk->il", w, RA, R_X_i, RB) / w.sum())
            R_X_i = proj_SO3(np.einsum("n,nji,jk,nkl->il", w, RA, R_M_i, RB) / w.sum())
            err = errs(R_X_i, R_M_i)
            k = max(1.0, 1.5 * np.median(err))       # Huber knee from the data
            w_new = np.minimum(1.0, k / np.maximum(err, 1e-6))
            if it > 3 and np.abs(w_new - w).max() < 1e-4:
                w = w_new
                break
            w = w_new
        return R_X_i, R_M_i, w

    def solve_t(R_X_i, R_M_i, w):
        # R_A(t) t_X - t_M = R_M t_B(t) - t_A(t); truncated SVD because a cart
        # that only yaws leaves one direction of (t_X, t_M) unobservable.
        G = np.zeros((3 * n, 6))
        G[:, :3] = (RA * w[:, None, None]).reshape(3 * n, 3)
        G[:, 3:] = (-np.eye(3)[None] * w[:, None, None]).reshape(3 * n, 3)
        h = ((tB @ R_M_i.T - tA) * w[:, None]).reshape(3 * n)
        U, s, Vt = np.linalg.svd(G, full_matrices=False)
        keep = s > 1e-2 * s[0]
        sol = Vt[keep].T @ ((U[:, keep].T @ h) / s[keep])
        return sol[:3], sol[3:], float(s[-1] / s[0])

    def score(R_X_i, R_M_i, w):
        """the metric that is actually reported: spread of X(t)'s translation."""
        t_X, t_M, cond = solve_t(R_X_i, R_M_i, w)
        # X(t) translation = R_A^T (R_M t_B + t_M - t_A)
        v = (tB @ R_M_i.T + t_M - tA)
        tx = np.einsum("nji,nj->ni", RA, v)
        return float(np.median(np.linalg.norm(tx - np.median(tx, axis=0), axis=1))), \
            t_X, t_M, cond

    # Multi-start: the alternating solver has several local optima and small
    # input differences flip it between them (RTAB-Map and ORB-SLAM3 poses agree
    # to 4 cm / 1-4 deg on the same bags, yet a 2-start solver put their rig
    # offsets 40 deg apart). Seed it from the linear solution, from R_M = I, and
    # from yaw rotations about the platform's own rotation axis, then keep the
    # candidate that minimises the reported spread, not a proxy cost.
    axis = rot_log(RA[0].T @ RA[n // 2])
    axis = axis / (np.linalg.norm(axis) or 1.0)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R_X_lin = proj_SO3(np.einsum("nji,njl->il", RA, RB) / n)
    # Strongest seed: the two cameras ride the same cart, so their PATHS are the
    # same shape 0.3 m apart while the map is metres across. Aligning the point
    # sets (Umeyama, rotation only) therefore lands M within a few degrees of the
    # truth. Without this seed the alternation settled 40 deg away on RTAB-Map
    # data whose correct solution was verifiable by transplanting ORB-SLAM3's.
    P, Q = tB - tB.mean(0), tA - tA.mean(0)
    U_, _, Vt_ = np.linalg.svd(P.T @ Q)
    R_M_shape = (U_ @ np.diag([1, 1, np.sign(np.linalg.det(U_ @ Vt_))]) @ Vt_).T
    inits = [(R_X, R_M), (R_X_lin, np.eye(3)), (R_X_lin, R_M_shape),
             (R_X, R_M_shape)]
    for a_deg in range(30, 360, 30):
        r = np.radians(a_deg)
        Ry = np.eye(3) + np.sin(r) * K + (1 - np.cos(r)) * (K @ K)
        inits.append((R_X_lin, Ry))
    # Score the seeds THEMSELVES as well as their polished versions: the polish
    # minimises a rotation-only cost, and on RTAB-Map data it walked a good seed
    # (10.9 cm spread, cross-checked by transplanting ORB-SLAM3's solution) into
    # a 51 cm optimum of that proxy cost. Selection is always on the real metric.
    ones = np.ones(n)
    best = None
    for R_X0, R_M0 in inits:
        half = proj_SO3(np.einsum("nji,jk,nkl->il", RA, R_M0, RB) / n)
        for rx, rm, w_i in ((R_X0, R_M0, ones), (half, R_M0, ones),
                            polish(R_X0, R_M0)):
            sc, _, _, _ = score(rx, rm, w_i)
            if best is None or sc < best[0]:
                best = (sc, rx, rm, w_i)
    _, R_X, R_M, w = best
    # The score above only sees X(t)'s TRANSLATION, which does not depend on
    # R_X at all, so R_X would otherwise be whatever the winning candidate
    # happened to carry. Given the selected R_M it has a closed form: the
    # Huber-weighted chordal mean of R_A(t)^T R_M R_B(t).
    w = np.ones(n)
    for _ in range(20):
        R_X = proj_SO3(np.einsum("n,nji,jk,nkl->il", w, RA, R_M, RB) / w.sum())
        err = errs(R_X, R_M)
        k = max(1.0, 1.5 * np.median(err))
        w_new = np.minimum(1.0, k / np.maximum(err, 1e-6))
        if np.abs(w_new - w).max() < 1e-4:
            w = w_new
            break
        w = w_new

    t_X, t_M, cond = solve_t(R_X, R_M, w)
    X, M = np.eye(4), np.eye(4)
    X[:3, :3], X[:3, 3] = R_X, t_X
    M[:3, :3], M[:3, 3] = R_M, t_M
    return X, M, cond


def residuals(A, B, X, M):
    """per-sample X(t) = A(t)^-1 M B(t) and its deviation from the median."""
    Xt = np.array([np.linalg.inv(A[i]) @ M @ B[i] for i in range(len(A))])
    t_med = np.median(Xt[:, :3, 3], axis=0)
    R_med = proj_SO3(np.mean(Xt[:, :3, :3], axis=0))
    d_t = np.linalg.norm(Xt[:, :3, 3] - t_med, axis=1)
    d_R = np.array([rot_angle_deg(R_med.T @ Xt[i, :3, :3]) for i in range(len(Xt))])
    return Xt, t_med, R_med, d_t, d_R


def fit(ts_a, T_a, ts_b, T_b, tau, stride, max_gap=0.35):
    A, B, st = pair_poses(ts_a, T_a, ts_b, T_b, tau, stride, max_gap)
    if len(A) < 30:
        return None
    X, M, cond = solve_axzb(A, B)
    if X is None:
        return None
    Xt, t_med, R_med, d_t, d_R = residuals(A, B, X, M)
    return {"n": len(A), "tau": tau, "X": X, "M": M, "cond": cond,
            "stamps": st, "Xt": Xt, "t_med": t_med, "R_med": R_med,
            "d_t": d_t, "d_R": d_R,
            "score": float(np.percentile(d_t, 90) + 0.02 * np.percentile(d_R, 90))}


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--a", required=True, help="SLAM-camera trajectory (TUM)")
    ap.add_argument("--b", required=True, help="SAM-camera trajectory (TUM)")
    ap.add_argument("--label", default="")
    ap.add_argument("--stride", type=int, default=2)
    ap.add_argument("--tau", type=float, default=None,
                    help="fixed clock offset a<-b [s]; default = scan for it")
    ap.add_argument("--tau-range", type=float, default=8.0)
    ap.add_argument("--max-gap", type=float, default=0.35,
                    help="largest interpolation gap in trajectory A [s]; RTAB-Map "
                         "exports its ~1 Hz pose GRAPH, so it needs a larger value")
    ap.add_argument("--out", default="")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    ts_a, T_a = read_tum(a.a)
    ts_b, T_b = read_tum(a.b)
    if len(ts_a) < 30 or len(ts_b) < 30:
        print(json.dumps({"label": a.label, "ok": False,
                          "reason": f"too few poses ({len(ts_a)}, {len(ts_b)})"}))
        return 1

    best = None
    if a.tau is None:
        # coarse then fine scan; the two recordings may not share a clock
        for step, span, centre in ((0.5, a.tau_range, 0.0), (0.05, 0.6, None)):
            c = best["tau"] if (centre is None and best) else 0.0
            for tau in np.arange(c - span, c + span + 1e-9, step):
                r = fit(ts_a, T_a, ts_b, T_b, float(tau), a.stride, a.max_gap)
                if r and (best is None or r["score"] < best["score"]):
                    best = r
    else:
        best = fit(ts_a, T_a, ts_b, T_b, a.tau, a.stride, a.max_gap)
    if best is None:
        print(json.dumps({"label": a.label, "ok": False, "reason": "fit failed"}))
        return 1

    X, d_t, d_R = best["X"], best["d_t"], best["d_R"]
    axis = rot_log(X[:3, :3])
    ang = np.degrees(np.linalg.norm(axis))
    out = {
        "label": a.label, "ok": True,
        "traj_a": a.a, "traj_b": a.b,
        "n_pairs": int(best["n"]), "tau_s": round(float(best["tau"]), 3),
        "X_camA_camB": [[round(float(v), 6) for v in r] for r in X],
        "M_mapA_mapB": [[round(float(v), 6) for v in r] for r in best["M"]],
        "X_rot_deg": round(float(ang), 3),
        "X_t_m": [round(float(v), 4) for v in X[:3, 3]],
        "X_t_norm_m": round(float(np.linalg.norm(X[:3, 3])), 4),
        "unobservable_ratio": round(best["cond"], 6),
        "constancy": {
            "trans_median_cm": round(float(np.median(d_t)) * 100, 2),
            "trans_p90_cm": round(float(np.percentile(d_t, 90)) * 100, 2),
            "trans_max_cm": round(float(d_t.max()) * 100, 2),
            "rot_median_deg": round(float(np.median(d_R)), 2),
            "rot_p90_deg": round(float(np.percentile(d_R, 90)), 2),
            "rot_max_deg": round(float(d_R.max()), 2),
        },
        "series": {"t": [round(float(v), 3) for v in best["stamps"]],
                   "dev_trans_cm": [round(float(v) * 100, 2) for v in d_t],
                   "dev_rot_deg": [round(float(v), 2) for v in d_R]},
    }
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(out, indent=1))
    if not a.quiet:
        c = out["constancy"]
        print(f"{a.label or 'fit'}: n={out['n_pairs']} tau={out['tau_s']:+.2f}s  "
              f"|X| rot {out['X_rot_deg']:.2f} deg, t {out['X_t_norm_m']:.3f} m "
              f"({', '.join(f'{v:+.3f}' for v in out['X_t_m'])})")
        print(f"    constancy: trans med {c['trans_median_cm']:.1f} / p90 "
              f"{c['trans_p90_cm']:.1f} / max {c['trans_max_cm']:.1f} cm | "
              f"rot med {c['rot_median_deg']:.2f} / p90 {c['rot_p90_deg']:.2f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
