#!/usr/bin/env python3
"""tools/render_mugcup_handle_evidence.py

detail_mugcup_handle_fail.jsonl 에서 handle-flip 사례를 골라 각 사례의
- observed_mask overlay
- 선택된 pose 의 CAD projection overlay
- pgt 로 얻은 정답 pose 의 CAD projection overlay
를 PNG 로 저장한다. bag 을 새로 열지 않고 verify_eval 이 이미 뽑아 놓은
frames.jsonl 은 픽셀을 보관하지 않으므로 (mask polygon 만) bag 을 다시
연다.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np


REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "realtime"))
sys.path.insert(0, str(REPO / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"))

DATA_BAG = REPO / "data" / "longcircle2_sam"


def load_pgt(path):
    d = json.load(open(path))
    out = {}
    for obj, entry in d.items():
        if obj.startswith("_") or not isinstance(entry, dict):
            continue
        if "R" not in entry or "t_m" not in entry:
            continue
        out[obj] = (np.array(entry["R"], dtype=float),
                    np.array(entry["t_m"], dtype=float) * 1000.0)
    return out


def load_traj(path):
    out = []
    with open(path) as f:
        for line in f:
            p = line.strip().split()
            if len(p) < 8: continue
            ts = float(p[0])
            tx, ty, tz = (float(x) for x in p[1:4])
            qx, qy, qz, qw = (float(x) for x in p[4:8])
            xx, yy, zz = qx * qx, qy * qy, qz * qz
            xy, xz, yz = qx * qy, qx * qz, qy * qz
            wx, wy, wz = qw * qx, qw * qy, qw * qz
            R = np.array([[1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                          [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                          [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]])
            out.append((int(round(ts * 1e9)), R, np.array([tx, ty, tz]) * 1000.0))
    out.sort(key=lambda r: r[0])
    return out


def nearest_traj(traj, t_ns, tol_ns=100_000_000):
    if not traj: return None
    best = None
    lo, hi = 0, len(traj) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        d = traj[mid][0] - t_ns
        if best is None or abs(d) < abs(best[0]):
            best = (d, traj[mid])
        if d < 0: lo = mid + 1
        else: hi = mid - 1
    if best is None or abs(best[0]) > tol_ns:
        return None
    return best[1]


def load_bag_by_stamp(bag_dir):
    """Reuse verify_eval.read_frames (which handles topic mapping, color-depth pairing)."""
    sys.path.insert(0, str(REPO / "temp"))
    from verify_eval import read_frames                        # noqa: E402
    K, frame_list = read_frames(bag_dir, stride=1, limit=0)
    frames = {}
    for t, bgr, dep in frame_list:
        frames[int(t)] = {"bgr": bgr, "dep": dep}
    return frames, K


def project_points(R, t_mm, K, pts_mm, w, h):
    P = (R @ pts_mm.T).T + t_mm
    z = P[:, 2]
    valid = z > 1e-3
    u = (P[valid, 0] * K[0, 0] / z[valid] + K[0, 2]).astype(int)
    v = (P[valid, 1] * K[1, 1] / z[valid] + K[1, 2]).astype(int)
    inb = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    return u[inb], v[inb]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detail", required=True,
                    help="analysis/detail_mugcup_handle_fail.jsonl")
    ap.add_argument("--npoint", type=int, required=True,
                    help="only render cases from this npoint value")
    ap.add_argument("--pseudo-gt",
                    default=str(REPO / "data" / "longcircle2_sam" / "self_slam" / "pseudo_ground_truth.json"))
    ap.add_argument("--trajectory",
                    default=str(REPO / "data" / "longcircle2_sam" / "self_slam" / "CameraTrajectory.txt"))
    ap.add_argument("--model-points",
                    default=str(REPO / "assets" / "model_points" / "Mugcup_high.npy"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--max", type=int, default=20,
                    help="render at most N worst-flip cases")
    args = ap.parse_args()

    outdir = Path(args.out); outdir.mkdir(parents=True, exist_ok=True)
    cases = []
    with open(args.detail) as f:
        for line in f:
            r = json.loads(line)
            if r.get("_npoint") != args.npoint:
                continue
            if not r.get("flip"):
                continue
            cases.append(r)
    cases.sort(key=lambda r: -abs(180.0 - (r.get("gt_rot_err_deg") or 0.0)) * -1)
    # sort by highest rot_err first (worst flip)
    cases.sort(key=lambda r: -(r.get("gt_rot_err_deg") or 0.0))
    cases = cases[:args.max]
    print(f"[render] {len(cases)} handle-flip cases from n={args.npoint}")

    pgt = load_pgt(args.pseudo_gt)
    traj = load_traj(args.trajectory)
    mugcup_pts_mm = np.load(args.model_points).astype(np.float32)   # already in mm

    print("[render] scanning bag (once) ...")
    frames, K = load_bag_by_stamp(str(DATA_BAG))
    print(f"[render] bag frames: {len(frames)} K={K is not None}")

    manifest = []
    for c in cases:
        stamp = int(c["stamp_ns"])
        # find closest bag frame by stamp
        best = None
        for ts_bag in frames:
            if best is None or abs(ts_bag - stamp) < abs(best - stamp):
                best = ts_bag
        if best is None or abs(best - stamp) > 50_000_000:
            print(f"[render] no bag frame near {stamp}"); continue
        rec = frames[best]
        bgr = rec.get("bgr"); dep = rec.get("dep")
        if bgr is None or K is None: continue
        h, w = bgr.shape[:2]
        img = bgr.copy()

        # projected estimated pose (red)
        R_est = np.array(c["R"]); t_est = np.array(c["t_mm"])
        ue, ve = project_points(R_est, t_est, K, mugcup_pts_mm, w, h)
        for u, v in zip(ue, ve): cv2.circle(img, (int(u), int(v)), 1, (0, 0, 255), -1)

        # projected gt pose (green)
        pgt_e = pgt.get("Mugcup_high")
        tj = nearest_traj(traj, stamp)
        gt_status = "none"
        if pgt_e is not None and tj is not None:
            R_wc = tj[1]; t_wc = tj[2]
            R_wo, t_wo = pgt_e
            R_co = R_wc.T @ R_wo
            t_co = R_wc.T @ (t_wo - t_wc)
            ug, vg = project_points(R_co, t_co, K, mugcup_pts_mm, w, h)
            for u, v in zip(ug, vg): cv2.circle(img, (int(u), int(v)), 1, (0, 255, 0), -1)
            gt_status = "green"

        # label
        cv2.putText(img, f"i={c['i']} rot_err={c.get('gt_rot_err_deg')}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, f"n={args.npoint} red=est green=gt", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        outp = outdir / f"n{args.npoint}_i{c['i']:04d}_rot{int(c.get('gt_rot_err_deg') or 0)}.png"
        cv2.imwrite(str(outp), img)
        manifest.append({"file": str(outp.name), "i": c["i"], "stamp_ns": stamp,
                         "gt_rot_err_deg": c.get("gt_rot_err_deg"),
                         "gt_trans_err_mm": c.get("gt_trans_err_mm"),
                         "gt_status": gt_status,
                         "selected_proposal6000_index": c.get("selected_proposal6000_index")})
        print(f"[render] {outp.name}")

    with open(outdir / "MANIFEST.json", "w") as f:
        json.dump({"npoint": args.npoint, "count": len(manifest), "cases": manifest},
                  f, indent=2, ensure_ascii=False)
    print(f"[render] wrote {len(manifest)} images → {outdir}")


if __name__ == "__main__":
    main()
