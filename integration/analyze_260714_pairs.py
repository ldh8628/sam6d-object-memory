#!/usr/bin/env python3
"""analyze_260714_pairs.py — per pair, per SLAM algorithm: is the two-camera rig
offset constant?

For every (SLAM bag, SAM bag) pair of the 260714 dataset and every algorithm that
produced a trajectory for BOTH bags, solve AX = ZB (rig_offset.py) and report

  * X   = T_camSLAM_camSAM, the rig offset the algorithm implies
  * tau = the clock offset between the two recordings that best explains the data
  * the spread of X(t) over the session -- the GT-free consistency score

and then, across pairs, whether one algorithm keeps reporting the SAME X (the
rig did not change between sessions, so it should).

    /home/ldh9501/miniconda3/envs/sam_yolo/bin/python integration/analyze_260714_pairs.py
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rig_offset import fit, read_tum, rot_angle_deg, rot_log       # noqa: E402

ROOT = HERE.parent
PAIRS = [("105023", "105018"), ("105319", "105314"), ("105657", "105652"),
         ("110109", "110104"), ("110536", "110532"), ("110637", "110633")]

# algorithm -> (path template for a bag role, sampling stride, max interp gap)
#   RTAB-Map exports its pose GRAPH (~1 Hz), not per-frame poses, hence the
#   larger interpolation gap; see the report for what that costs it.
ALGOS = {
    "orb3": {
        "slam": ROOT / "output_slam/260714_pairs/slam_{s}/orbslam3_noimu/trajectory.txt",
        "sam": ROOT / "output_slam/260714_pairs/sam_{m}/orbslam3_noimu/trajectory.txt",
        "stride": 2, "max_gap": 0.35,
    },
    "rtabmap": {
        "slam": ROOT / "output_slam/260714_pairs/slam_{s}/rtabmap/trajectory.txt",
        "sam": ROOT / "output_slam/260714_pairs/sam_{m}/rtabmap/trajectory.txt",
        "stride": 1, "max_gap": 3.0,
    },
    "hdl_graph_slam": {   # present so the gap is explicit, never runnable here
        "slam": None, "sam": None, "stride": 1, "max_gap": 1.0,
    },
}


def run_one(algo, s, m, tau, tau_range):
    cfg = ALGOS[algo]
    if cfg["slam"] is None:
        return {"ok": False, "reason": "no LiDAR in this dataset"}
    pa = Path(str(cfg["slam"]).format(s=s, m=m))
    pb = Path(str(cfg["sam"]).format(s=s, m=m))
    if not pa.is_file() or not pb.is_file():
        return {"ok": False, "reason": f"missing trajectory ({pa.name}/{pb.name})"}
    ts_a, T_a = read_tum(pa)
    ts_b, T_b = read_tum(pb)
    if len(ts_a) < 30 or len(ts_b) < 30:
        return {"ok": False, "reason": f"too few poses ({len(ts_a)}, {len(ts_b)})",
                "poses": [len(ts_a), len(ts_b)]}

    best = None
    if tau is None:
        for step, span, use_best in ((0.5, tau_range, False), (0.05, 0.6, True)):
            c = best["tau"] if (use_best and best) else 0.0
            for t in np.arange(c - span, c + span + 1e-9, step):
                r = fit(ts_a, T_a, ts_b, T_b, float(t), cfg["stride"], cfg["max_gap"])
                if r and (best is None or r["score"] < best["score"]):
                    best = r
    else:
        best = fit(ts_a, T_a, ts_b, T_b, tau, cfg["stride"], cfg["max_gap"])
    if best is None:
        return {"ok": False, "reason": "fit failed"}

    X, d_t, d_R = best["X"], best["d_t"], best["d_R"]
    return {
        "ok": True, "poses": [len(ts_a), len(ts_b)], "n_pairs": int(best["n"]),
        "tau_s": round(float(best["tau"]), 3),
        "X": [[float(v) for v in r] for r in X],
        "X_rot_deg": round(float(np.degrees(np.linalg.norm(rot_log(X[:3, :3])))), 3),
        "X_t_m": [round(float(v), 4) for v in X[:3, 3]],
        "X_t_norm_m": round(float(np.linalg.norm(X[:3, 3])), 4),
        "M": [[float(v) for v in r] for r in best["M"]],
        "unobservable_ratio": round(best["cond"], 6),
        "dev_trans_cm": {"median": round(float(np.median(d_t)) * 100, 2),
                         "p90": round(float(np.percentile(d_t, 90)) * 100, 2),
                         "max": round(float(d_t.max()) * 100, 2)},
        "dev_rot_deg": {"median": round(float(np.median(d_R)), 2),
                        "p90": round(float(np.percentile(d_R, 90)), 2),
                        "max": round(float(d_R.max()), 2)},
        "series": {"t": [round(float(v), 3) for v in best["stamps"]],
                   "dev_trans_cm": [round(float(v) * 100, 2) for v in d_t],
                   "dev_rot_deg": [round(float(v), 2) for v in d_R]},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algos", nargs="+", default=list(ALGOS))
    ap.add_argument("--pairs", nargs="+", default=[f"{s}_{m}" for s, m in PAIRS])
    ap.add_argument("--tau", type=float, default=None)
    ap.add_argument("--tau-range", type=float, default=8.0)
    ap.add_argument("--out", default=str(ROOT / "integration/output/260714_pairs/rig_offset.json"))
    a = ap.parse_args()

    results = {}
    for s, m in PAIRS:
        key = f"{s}_{m}"
        if key not in a.pairs:
            continue
        results[key] = {}
        for algo in a.algos:
            r = run_one(algo, s, m, a.tau, a.tau_range)
            results[key][algo] = r
            if r["ok"]:
                print(f"{key:14s} {algo:14s} n={r['n_pairs']:5d} tau={r['tau_s']:+6.2f}s  "
                      f"X rot {r['X_rot_deg']:6.2f} deg  |t| {r['X_t_norm_m']:.3f} m  "
                      f"({', '.join(f'{v:+.3f}' for v in r['X_t_m'])})  "
                      f"dev {r['dev_trans_cm']['median']:5.1f} cm / "
                      f"{r['dev_rot_deg']['median']:4.2f} deg (median)", flush=True)
            else:
                print(f"{key:14s} {algo:14s} SKIP — {r['reason']}", flush=True)

    # cross-session consistency: the rig did not change, so X should repeat
    cross = {}
    for algo in a.algos:
        Xs = [np.array(results[k][algo]["X"]) for k in results
              if results[k][algo]["ok"]]
        if len(Xs) < 2:
            continue
        t = np.array([X[:3, 3] for X in Xs])
        R0 = Xs[0][:3, :3]
        cross[algo] = {
            "sessions": len(Xs),
            "t_spread_cm": [round(float(v) * 100, 2) for v in t.std(axis=0)],
            "t_range_cm": round(float(np.linalg.norm(t.max(axis=0) - t.min(axis=0))) * 100, 2),
            "rot_spread_deg": round(float(np.std(
                [rot_angle_deg(R0.T @ X[:3, :3]) for X in Xs])), 3),
            "rot_max_diff_deg": round(float(np.max(
                [rot_angle_deg(Xs[i][:3, :3].T @ Xs[j][:3, :3])
                 for i in range(len(Xs)) for j in range(i + 1, len(Xs))])), 3),
        }
        c = cross[algo]
        print(f"\n{algo}: same rig across {c['sessions']} sessions -> "
              f"rotation spread {c['rot_max_diff_deg']:.2f} deg (max pairwise), "
              f"translation spread {c['t_spread_cm']} cm/axis")

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"pairs": results, "cross_session": cross}, indent=1))
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
