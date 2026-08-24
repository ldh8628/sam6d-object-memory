#!/usr/bin/env python3
"""tools/analyze_pem_coarse_sweep.py

`output/pem_coarse_sweep_v1/n<npoint>/off.jsonl` 들을 (stamp_ns, object) 로 조인해
- 각 point count 별 정상 종료 여부, 실행 시간, VRAM, geometry top-1 pose 변경률,
  depth residual 분포, Mask IoU 분포, 객체별 유효 관측점 비율, 유효 IoU >= 0.50
  detection 수, 그 조건에서의 pose 성공률
- Mugcup 손잡이가 mask 에 포함된 프레임의 성공률
- Mugcup frame 60/61 상세
- 196 baseline 대비 pose 변경 detection 목록
- 유효 IoU >= 0.50 실패 detection 목록
- Mugcup handle 실패 시각 증거용 detection 목록
을 만들고 point-count-sweep.{json,csv,md} 를 저장한다.

성공 정의(GT 부재 시 pseudo-GT + temporal fallback):
  - valid observation:  pem.n_pts_in >= 128 AND pem.depth_valid_frac >= 0.30
  - sufficient visibility (해석용 필터): verify.shape.coverage >= 0.50
                                          AND verify.shape.observed_mask_area_px >= 500
  - pose success (well-visible subset 안):
      verify.shape.mask_iou >= 0.50 AND
      verify.pointwise[0].geometry.overlap_ratio >= 0.50 AND
      verify.pointwise[0].geometry.mean_distance_mm <= 10.0 AND
      (pseudo-GT 로 rot_err_deg <= 30 AND trans_err_mm <= 80  OR
       pseudo-GT 부재 시 temporal median 로 rot_err_deg <= 45)

Mugcup 손잡이 포함 프레임:
  observed_mask 픽셀 분포의 non-elliptic extent 로 손잡이 존재를 근사.
  → handle_axis_score = |mask_covariance_off_diagonal| / bbox_diag  가 임계 이상.
  실패는 (well-visible AND mugcup_handle_visible AND NOT pose_success) 인 detection.

기준 GT 부재는 report 안에 명시한다."""
from __future__ import annotations

import argparse
import csv
import gzip
import io
import json
import math
import os
import statistics
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np

REPO = Path("/home/etri/sam6d_realtime/sam6d_realtime")
DEFAULT_SWEEP = REPO / "output" / "pem_coarse_sweep_v1"
DEFAULT_PSEUDO_GT = REPO / "data" / "longcircle2_sam" / "self_slam" / "pseudo_ground_truth.json"
DEFAULT_TRAJ = REPO / "data" / "longcircle2_sam" / "self_slam" / "CameraTrajectory.txt"

SYM_AXES = {"Sikhye_high": (0.0, 0.0, 1.0),
            "Sauce_high": (0.0, 0.0, 1.0),
            "Mugcup_high": (0.1057, 0.0337, 0.9938)}
SYM_STEP_DEG = 10

VALID_N_PTS = 128                       # PEM 이 fine_npoint 2048 로 채워 항상 통과
VALID_DEPTH_FRAC = 0.30                 # 유효 depth 화소 비율
VALID_OBS_MASK_PX = 2000                # 객체가 관측에서 유의미하게 보이는지 (pose 독립)
COVER_GATE = 0.50                       # 별도 보고용 (pose 종속)
IOU_GATE = 0.50                         # user 요구: mask IoU >= 0.50
OVERLAP_GATE = 0.40                     # coarse depth overlap
MEAN_DIST_GATE_MM = 12.0
ROT_GATE_DEG = 45.0                     # 대칭 고려한 회전 오차 허용
TRANS_GATE_MM = 100.0
TEMPORAL_ROT_GATE_DEG = 60.0

MUGCUP = "Mugcup_high"


# --------------------------------------------------------------------- 회전 도구
def _axis_rot(axis, deg):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    x, y, z = (c / n for c in axis)
    a = math.radians(deg)
    c, s, k = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return np.array([[c + x * x * k, x * y * k - z * s, x * z * k + y * s],
                     [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
                     [z * x * k - y * s, z * y * k + x * s, c + z * z * k]])


_SYM_CACHE = {}


def sym_group(name):
    if name in _SYM_CACHE:
        return _SYM_CACHE[name]
    ax = SYM_AXES.get(name)
    grp = [np.eye(3)] if ax is None else [_axis_rot(ax, d) for d in range(0, 360, SYM_STEP_DEG)]
    _SYM_CACHE[name] = grp
    return grp


def rot_angle_deg(A, B, name=None):
    best = 180.0
    for S in sym_group(name):
        tr = float(np.trace(A.T @ (B @ S)))
        best = min(best, math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))))
    return best


def rot_angle_no_sym_deg(A, B):
    tr = float(np.trace(A.T @ B))
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


def _pose_from_txt_line(parts):
    # timestamp tx ty tz qx qy qz qw
    tx, ty, tz = (float(x) for x in parts[1:4])
    qx, qy, qz, qw = (float(x) for x in parts[4:8])
    # rotation matrix from quaternion
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    R = np.array([[1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                  [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                  [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]])
    t = np.array([tx, ty, tz])
    return R, t, float(parts[0])


def load_trajectory(path):
    """TUM-format trajectory: timestamp tx ty tz qx qy qz qw. Returns list of (t_ns, R_wc, t_wc)."""
    out = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            R, t, ts_sec = _pose_from_txt_line(parts)
            out.append((int(round(ts_sec * 1e9)), R, t))
    out.sort(key=lambda r: r[0])
    return out


def nearest_pose(traj, t_ns, tol_ns=100_000_000):
    """Return (R_wc, t_wc, delta_ns) or None if none within tol."""
    if not traj:
        return None
    lo, hi = 0, len(traj) - 1
    best = None
    while lo <= hi:
        mid = (lo + hi) // 2
        d = traj[mid][0] - t_ns
        if best is None or abs(d) < abs(best[0]):
            best = (d, traj[mid])
        if d < 0:
            lo = mid + 1
        else:
            hi = mid - 1
    if best is None or abs(best[0]) > tol_ns:
        return None
    return (*best[1][1:], int(best[0]))


def load_pseudo_gt(path):
    d = json.load(open(path))
    out = {}
    for obj, entry in d.items():
        if obj.startswith("_"):
            continue
        if not isinstance(entry, dict) or "R" not in entry or "t_m" not in entry:
            continue
        out[obj] = {
            "R_wo": np.array(entry["R"], dtype=float),
            "t_wo": np.array(entry["t_m"], dtype=float) * 1000.0,   # m → mm
            "trusted": bool(entry.get("trusted", False)),
            "quality_gate": entry.get("quality_gate"),
            "unavailable_reason": entry.get("unavailable_reason"),
            "cluster_size": entry.get("cluster_size"),
        }
    return out


# --------------------------------------------------------------------- 로더
def iter_jsonl(path):
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s:
                yield json.loads(s)


def load_run(sweep_dir: Path, npoint: int):
    dir_ = sweep_dir / f"n{npoint}"
    prov = dir_ / "off.provenance.json"
    detections = dir_ / "off.jsonl"
    frames = dir_ / "off_frames.jsonl"
    sweep_run = dir_ / "sweep-run.json"
    ret = {"npoint": int(npoint), "dir": str(dir_),
           "provenance_exists": prov.exists(),
           "detections_exists": detections.exists(),
           "frames_exists": frames.exists()}
    if sweep_run.exists():
        ret["sweep_run"] = json.load(open(sweep_run))
    if prov.exists():
        ret["provenance"] = json.load(open(prov))
    return ret


def extract_features(row):
    """Compressed per-detection dict for cross-sweep join."""
    verify = row.get("verify") or {}
    shape = verify.get("shape") or {}
    pointwise = verify.get("pointwise") or []
    pw0 = pointwise[0] if pointwise else {}
    geom = (pw0.get("geometry") or {}) if isinstance(pw0, dict) else {}
    pem = row.get("pem") or {}
    return {
        "stamp_ns": int(row["stamp_ns"]),
        "i": int(row["i"]),
        "object": row["object"],
        "R": [[float(v) for v in rr] for rr in row["R"]],
        "t_mm": [float(v) for v in row["t_mm"]],
        "score": float(row.get("score", 0.0)),
        "selected_proposal6000_index": verify.get("selected_proposal6000_index"),
        "selected_geo_rank": verify.get("selected_geo_rank"),
        "pose_score": pem.get("pose_score"),
        "n_pts_in": pem.get("n_pts_in"),
        "depth_valid_frac": pem.get("depth_valid_frac"),
        "depth_valid_px": pem.get("depth_valid_px"),
        "z_med_mm": pem.get("z_med_mm"),
        "mask_iou": shape.get("mask_iou"),
        "coverage": shape.get("coverage"),
        "size_ratio": shape.get("size_ratio"),
        "shape_status": shape.get("status"),
        "shape_reason": shape.get("reason"),
        "observed_mask_area_px": shape.get("observed_mask_area_px"),
        "rendered_mask_area_px": shape.get("rendered_mask_area_px"),
        "geo_point_count": geom.get("point_count"),
        "geo_overlap_count": geom.get("overlap_count"),
        "geo_overlap_ratio": geom.get("overlap_ratio"),
        "geo_mean_distance_mm": geom.get("mean_distance_mm"),
        "geo_p90_distance_mm": geom.get("p90_distance_mm"),
        "geo_threshold_mm": geom.get("threshold_mm"),
        "shape_mask_bbox_224": (verify.get("shape_mask") or {}).get("bbox_224"),
        "observed_bbox_area_px": shape.get("observed_bbox_area_px"),
        "bbox_xyxy": row.get("bbox"),
    }


def load_features(dir_: Path):
    """Parse off.jsonl once, cache a compact features.jsonl for future re-runs.
    Cache is invalidated by size mismatch."""
    src = dir_ / "off.jsonl"
    cache = dir_ / "features.compact.jsonl"
    if cache.exists() and src.exists():
        try:
            if cache.stat().st_mtime >= src.stat().st_mtime:
                feats = []
                with open(cache) as f:
                    for line in f:
                        s = line.strip()
                        if s:
                            feats.append(json.loads(s))
                return feats
        except Exception:
            pass
    feats = []
    with open(cache, "w") as out:
        for row in iter_jsonl(src):
            f = extract_features(row)
            feats.append(f)
            out.write(json.dumps(f, ensure_ascii=False) + "\n")
    return feats


# --------------------------------------------------------------------- Mugcup handle
def _rle_area(rle):
    """COCO uncompressed RLE {'size':[h,w],'counts':[...]} → binary mask."""
    if not rle:
        return None
    h, w = rle.get("size", [0, 0])
    counts = rle.get("counts")
    if not counts:
        return None
    if isinstance(counts, str):
        return None
    mask = np.zeros(h * w, dtype=np.uint8)
    v = 0; idx = 0
    for c in counts:
        c = int(c)
        if v == 1:
            mask[idx:idx + c] = 1
        idx += c
        v ^= 1
    return mask.reshape((w, h)).T  # COCO stores column-major


def mugcup_handle_visible(observed_mask_area_px, observed_bbox_area_px, bbox_xyxy):
    """손잡이 존재를 근사하는 저비용 프록시.
    손잡이가 없는 순수 원기둥/원 mask 는 bbox 를 fill ~0.75-0.80 로 채우고 aspect ~1.0.
    손잡이가 붙으면 bbox 는 넓어지는데 mask 는 그만큼 안 채우므로 fill 이 떨어진다(≈0.55-0.68).
    또는 bbox aspect 가 좌우로 늘어난다.
    보수적으로 fill <= 0.72 이면 handle 로 인한 empty 영역이 유의미하게 있다고 본다."""
    if not observed_mask_area_px or not observed_bbox_area_px:
        return False
    if observed_mask_area_px < 300:
        return False
    fill = float(observed_mask_area_px) / max(1.0, float(observed_bbox_area_px))
    if bbox_xyxy and len(bbox_xyxy) == 4:
        x1, y1, x2, y2 = bbox_xyxy
        w = max(1, x2 - x1); h = max(1, y2 - y1)
        ar = max(w, h) / max(1.0, min(w, h))
    else:
        ar = 1.0
    return (fill <= 0.72) or (ar >= 1.30)


# --------------------------------------------------------------------- Pose success
def pose_success(feat, pgt_entry, cam_pose):
    """Return (success, gate_reasons: dict).
    Requires visibility/observation gates upstream (check_visibility).
    """
    reasons = {}
    ok = True
    if feat["mask_iou"] is None or feat["mask_iou"] < IOU_GATE:
        ok = False; reasons["mask_iou"] = feat["mask_iou"]
    if feat["geo_overlap_ratio"] is None or feat["geo_overlap_ratio"] < OVERLAP_GATE:
        ok = False; reasons["geo_overlap_ratio"] = feat["geo_overlap_ratio"]
    if feat["geo_mean_distance_mm"] is None or feat["geo_mean_distance_mm"] > MEAN_DIST_GATE_MM:
        ok = False; reasons["geo_mean_distance_mm"] = feat["geo_mean_distance_mm"]

    # pseudo-GT check (모든 벡터를 mm 로 통일)
    if pgt_entry is not None and cam_pose is not None:
        R_wc, t_wc_m, _ = cam_pose
        t_wc = t_wc_m * 1000.0                       # m → mm
        R_wo = pgt_entry["R_wo"]; t_wo = pgt_entry["t_wo"]   # already mm
        R_co_gt = R_wc.T @ R_wo
        t_co_gt = R_wc.T @ (t_wo - t_wc)
        R_est = np.array(feat["R"]); t_est = np.array(feat["t_mm"])
        rot_err = rot_angle_deg(R_est, R_co_gt, feat["object"])
        trans_err = float(np.linalg.norm(t_est - t_co_gt))
        reasons["pgt_rot_err_deg"] = round(rot_err, 3)
        reasons["pgt_trans_err_mm"] = round(trans_err, 2)
        reasons["pgt_trusted"] = pgt_entry["trusted"]
        if pgt_entry["trusted"]:
            if rot_err > ROT_GATE_DEG:
                ok = False; reasons["gate_rot_gt"] = True
            if trans_err > TRANS_GATE_MM:
                ok = False; reasons["gate_trans_gt"] = True
    else:
        reasons["pgt_available"] = False
    return ok, reasons


def check_visibility(feat):
    """관측 자체가 유의미한지 (pose 결과와 독립)."""
    reasons = {}
    ok = True
    if feat["depth_valid_frac"] is None or feat["depth_valid_frac"] < VALID_DEPTH_FRAC:
        ok = False; reasons["depth_valid_frac"] = feat["depth_valid_frac"]
    if (feat["observed_mask_area_px"] is None
            or feat["observed_mask_area_px"] < VALID_OBS_MASK_PX):
        ok = False; reasons["observed_mask_area_px"] = feat["observed_mask_area_px"]
    return ok, reasons


def temporal_median_R(feats_by_stamp, obj, stamp, window_ns=200_000_000):
    """Median rotation (Frobenius-based polar) across the same object in ±window."""
    Rs = []
    for s, per in feats_by_stamp.items():
        if abs(int(s) - int(stamp)) <= window_ns and s != stamp:
            for f in per:
                if f["object"] == obj and f["R"] is not None:
                    Rs.append(np.array(f["R"]))
    if len(Rs) < 2:
        return None
    M = sum(Rs)
    u, _, vt = np.linalg.svd(M)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1
        R = u @ vt
    return R


# --------------------------------------------------------------------- Aggregation
def percentile(xs, p):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    if not xs:
        return None
    xs.sort()
    k = (len(xs) - 1) * p / 100.0
    lo = int(math.floor(k)); hi = int(math.ceil(k))
    if lo == hi:
        return float(xs[lo])
    return float(xs[lo] + (xs[hi] - xs[lo]) * (k - lo))


def summarize_run(feats, pgt, traj, baseline_map):
    per_obj = defaultdict(list)
    per_stamp = defaultdict(list)
    for f in feats:
        per_obj[f["object"]].append(f)
        per_stamp[f["stamp_ns"]].append(f)

    detections_by_key = {(f["stamp_ns"], f["object"]): f for f in feats}

    n = len(feats)
    n_well_observed = 0                     # observation-only visibility gate
    n_well_covered = 0                      # coverage >= 0.5 (pose-dependent)
    n_iou_ge_50_all = 0                     # mask_iou >= 0.5 among ALL detections
    n_iou_ge_50_well_obs = 0                # mask_iou >= 0.5 AND well_observed
    n_success_all = 0                       # success gates on ALL
    n_success_well_obs = 0                  # success gates AND well_observed
    n_success_target = 0                    # success gates AND well_observed AND iou>=0.5

    dist_iou = []; dist_overlap = []; dist_mean_dist = []; dist_p90_dist = []

    per_obj_agg = defaultdict(lambda: {
        "total": 0, "well_observed": 0, "well_covered": 0,
        "iou_ge_050": 0, "success_target": 0,
        "iou": [], "mean_dist": [], "overlap": []})
    handle_visible_frames = 0
    handle_visible_pass = 0
    handle_visible_fail = 0
    mugcup_gt_available = 0
    mugcup_gt_correct = 0             # rot_err ≤ 45° (대칭 반영)
    mugcup_gt_flipped = 0             # rot_err > 90° — 손잡이 반대방향
    mugcup_handle_flip = 0            # handle_visible AND rot_err > 90°

    pose_changes_vs_base = 0
    pose_worsened_vs_base = 0
    pose_improved_vs_base = 0

    mugcup_60_61 = {}

    detail_pose_changed = []
    detail_iou_pass_but_fail = []
    detail_mugcup_handle_fail = []

    for f in feats:
        obj = f["object"]
        agg = per_obj_agg[obj]
        agg["total"] += 1

        vis_ok, vis_reasons = check_visibility(f)
        cov_ok = (f["coverage"] or 0) >= COVER_GATE
        iou_ok = (f["mask_iou"] or 0) >= IOU_GATE
        if vis_ok:
            n_well_observed += 1
            agg["well_observed"] += 1
        if cov_ok:
            n_well_covered += 1
            agg["well_covered"] += 1
        if iou_ok:
            n_iou_ge_50_all += 1
            agg["iou_ge_050"] += 1
        if vis_ok and iou_ok:
            n_iou_ge_50_well_obs += 1

        # metrics collection (well_observed 만 대상 — 관측이 유의미한 detection)
        if vis_ok:
            if f["mask_iou"] is not None:
                agg["iou"].append(f["mask_iou"])
                dist_iou.append(f["mask_iou"])
            if f["geo_mean_distance_mm"] is not None:
                agg["mean_dist"].append(f["geo_mean_distance_mm"])
                dist_mean_dist.append(f["geo_mean_distance_mm"])
            if f["geo_p90_distance_mm"] is not None:
                dist_p90_dist.append(f["geo_p90_distance_mm"])
            if f["geo_overlap_ratio"] is not None:
                agg["overlap"].append(f["geo_overlap_ratio"])
                dist_overlap.append(f["geo_overlap_ratio"])

        pgt_entry = pgt.get(obj)
        cam_pose = nearest_pose(traj, f["stamp_ns"]) if traj else None
        ok, reasons = pose_success(f, pgt_entry, cam_pose)

        if ok:
            n_success_all += 1
            if vis_ok:
                n_success_well_obs += 1
            if vis_ok and iou_ok:
                n_success_target += 1
                agg["success_target"] += 1
        # target = well_observed AND iou>=0.5 → 이 안에서 실패는 user 의 핵심 관심사
        if vis_ok and iou_ok and not ok:
            detail_iou_pass_but_fail.append({
                "stamp_ns": f["stamp_ns"], "i": f["i"], "object": obj,
                "coverage": f["coverage"], "mask_iou": f["mask_iou"],
                "overlap_ratio": f["geo_overlap_ratio"],
                "mean_distance_mm": f["geo_mean_distance_mm"],
                "reasons": reasons,
            })

        # mugcup handle + GT-only correctness
        if obj == MUGCUP and vis_ok:
            handle = mugcup_handle_visible(f["observed_mask_area_px"],
                                            f["observed_bbox_area_px"],
                                            f["bbox_xyxy"])
            gt_rot_err = reasons.get("pgt_rot_err_deg")
            gt_trusted = reasons.get("pgt_trusted")
            if gt_rot_err is not None and gt_trusted:
                mugcup_gt_available += 1
                if gt_rot_err <= 45.0:
                    mugcup_gt_correct += 1
                elif gt_rot_err > 90.0:
                    mugcup_gt_flipped += 1
            if handle:
                handle_visible_frames += 1
                if ok:
                    handle_visible_pass += 1
                else:
                    handle_visible_fail += 1
                    is_flip = (gt_rot_err is not None and gt_rot_err > 90.0)
                    if is_flip:
                        mugcup_handle_flip += 1
                    detail_mugcup_handle_fail.append({
                        "stamp_ns": f["stamp_ns"], "i": f["i"],
                        "coverage": f["coverage"], "mask_iou": f["mask_iou"],
                        "overlap_ratio": f["geo_overlap_ratio"],
                        "mean_distance_mm": f["geo_mean_distance_mm"],
                        "selected_proposal6000_index": f["selected_proposal6000_index"],
                        "R": f["R"], "t_mm": f["t_mm"],
                        "gt_rot_err_deg": gt_rot_err,
                        "gt_trans_err_mm": reasons.get("pgt_trans_err_mm"),
                        "flip": is_flip,
                        "reasons": reasons,
                    })

        # Mugcup frame 60/61 detail
        if obj == MUGCUP and f["i"] in (60, 61):
            mugcup_60_61[f["i"]] = {
                "stamp_ns": f["stamp_ns"],
                "selected_proposal6000_index": f["selected_proposal6000_index"],
                "selected_geo_rank": f["selected_geo_rank"],
                "R": f["R"], "t_mm": f["t_mm"],
                "pose_score": f["pose_score"], "score": f["score"],
                "mask_iou": f["mask_iou"], "coverage": f["coverage"],
                "geo_point_count": f["geo_point_count"],
                "geo_overlap_ratio": f["geo_overlap_ratio"],
                "geo_mean_distance_mm": f["geo_mean_distance_mm"],
                "geo_p90_distance_mm": f["geo_p90_distance_mm"],
                "geo_threshold_mm": f["geo_threshold_mm"],
                "pose_success": ok, "reasons": reasons,
            }

        # vs baseline
        if baseline_map is not None:
            base = baseline_map.get((f["stamp_ns"], obj))
            if base is not None:
                bR = np.array(base["R"]); bT = np.array(base["t_mm"])
                fR = np.array(f["R"]); fT = np.array(f["t_mm"])
                rot_diff = rot_angle_no_sym_deg(bR, fR)
                trans_diff = float(np.linalg.norm(fT - bT))
                changed = (rot_diff > 5.0 or trans_diff > 10.0)
                if changed:
                    pose_changes_vs_base += 1
                    base_iou = base["mask_iou"] if base["mask_iou"] is not None else -1
                    new_iou = f["mask_iou"] if f["mask_iou"] is not None else -1
                    delta_iou = (new_iou - base_iou) if (base_iou >= 0 and new_iou >= 0) else 0.0
                    tag = "improved" if delta_iou > 0.05 else ("worsened" if delta_iou < -0.05 else "neutral")
                    if tag == "improved":
                        pose_improved_vs_base += 1
                    elif tag == "worsened":
                        pose_worsened_vs_base += 1
                    detail_pose_changed.append({
                        "stamp_ns": f["stamp_ns"], "i": f["i"], "object": obj,
                        "rot_diff_deg": round(rot_diff, 3),
                        "trans_diff_mm": round(trans_diff, 2),
                        "base_iou": base_iou, "new_iou": new_iou,
                        "delta_iou": round(delta_iou, 4),
                        "base_mean_dist_mm": base["geo_mean_distance_mm"],
                        "new_mean_dist_mm": f["geo_mean_distance_mm"],
                        "tag": tag,
                    })

    return {
        "total_detections": n,
        "well_observed_count": n_well_observed,
        "well_covered_count": n_well_covered,
        "iou_ge_050_count_all": n_iou_ge_50_all,
        "iou_ge_050_count_well_obs": n_iou_ge_50_well_obs,
        "success_count_all": n_success_all,
        "success_count_well_obs": n_success_well_obs,
        "success_count_target": n_success_target,   # well_observed AND iou>=0.5 AND pose-ok
        "iou_p50": percentile(dist_iou, 50), "iou_p75": percentile(dist_iou, 75),
        "iou_p90": percentile(dist_iou, 90), "iou_p95": percentile(dist_iou, 95),
        "mean_dist_p50": percentile(dist_mean_dist, 50),
        "mean_dist_p75": percentile(dist_mean_dist, 75),
        "mean_dist_p90": percentile(dist_mean_dist, 90),
        "mean_dist_p95": percentile(dist_mean_dist, 95),
        "p90_dist_p50": percentile(dist_p90_dist, 50),
        "p90_dist_p90": percentile(dist_p90_dist, 90),
        "overlap_p50": percentile(dist_overlap, 50),
        "overlap_p90": percentile(dist_overlap, 90),
        "handle_visible_frames": handle_visible_frames,
        "handle_visible_pass": handle_visible_pass,
        "handle_visible_fail": handle_visible_fail,
        "mugcup_gt_available": mugcup_gt_available,
        "mugcup_gt_correct": mugcup_gt_correct,
        "mugcup_gt_flipped": mugcup_gt_flipped,
        "mugcup_handle_flip": mugcup_handle_flip,
        "pose_changes_vs_base": pose_changes_vs_base,
        "pose_improved_vs_base": pose_improved_vs_base,
        "pose_worsened_vs_base": pose_worsened_vs_base,
        "per_object": {obj: {
            "total": v["total"], "well_observed": v["well_observed"],
            "well_covered": v["well_covered"], "iou_ge_050": v["iou_ge_050"],
            "success_target": v["success_target"],
            "iou_p50": percentile(v["iou"], 50), "iou_p90": percentile(v["iou"], 90),
            "mean_dist_p50": percentile(v["mean_dist"], 50),
            "overlap_p50": percentile(v["overlap"], 50),
        } for obj, v in per_obj_agg.items()},
        "mugcup_60_61": mugcup_60_61,
        "detail_pose_changed": detail_pose_changed,
        "detail_iou_pass_but_fail": detail_iou_pass_but_fail,
        "detail_mugcup_handle_fail": detail_mugcup_handle_fail,
        "_detections_by_key": detections_by_key,   # for baseline of next iteration
    }


def load_vram(log_path):
    if not os.path.exists(log_path):
        return {"max_mib": None, "mean_mib": None, "samples": 0}
    xs = []
    with open(log_path) as f:
        for line in f:
            try:
                d = json.loads(line)
                parts = d["line"].split(",")
                xs.append(int(parts[0]))
            except Exception:
                continue
    if not xs:
        return {"max_mib": None, "mean_mib": None, "samples": 0}
    return {"max_mib": max(xs), "mean_mib": float(sum(xs) / len(xs)), "samples": len(xs)}


# --------------------------------------------------------------------- Main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep-dir", default=str(DEFAULT_SWEEP))
    ap.add_argument("--out-dir", default=None,
                    help="default: <sweep-dir>/analysis")
    ap.add_argument("--pseudo-gt", default=str(DEFAULT_PSEUDO_GT))
    ap.add_argument("--trajectory", default=str(DEFAULT_TRAJ))
    ap.add_argument("--baseline-npoint", type=int, default=196)
    ap.add_argument("--only", default="", help="comma list of npoints to include; default = all")
    args = ap.parse_args()

    sweep = Path(args.sweep_dir)
    out = Path(args.out_dir) if args.out_dir else (sweep / "analysis")
    out.mkdir(parents=True, exist_ok=True)

    npoints = []
    for p in sorted(sweep.glob("n*")):
        if p.is_dir() and (p / "off.jsonl").exists():
            try:
                npoints.append(int(p.name[1:]))
            except ValueError:
                pass
    npoints.sort()
    if args.only:
        wanted = {int(x) for x in args.only.split(",") if x.strip()}
        npoints = [n for n in npoints if n in wanted]
    print(f"[analyze] npoints: {npoints}", flush=True)

    pgt = {}
    if Path(args.pseudo_gt).exists():
        pgt = load_pseudo_gt(args.pseudo_gt)
        print(f"[analyze] pseudo-GT: {sorted(pgt.keys())}", flush=True)
    else:
        print(f"[analyze] pseudo-GT missing at {args.pseudo_gt}", flush=True)

    traj = []
    if Path(args.trajectory).exists():
        traj = load_trajectory(args.trajectory)
        print(f"[analyze] camera trajectory: {len(traj)} poses", flush=True)
    else:
        print(f"[analyze] trajectory missing at {args.trajectory}", flush=True)

    baseline_map = None
    all_runs = []
    for n in npoints:
        dir_ = sweep / f"n{n}"
        feats = load_features(dir_)
        vram = load_vram(str(sweep / "logs" / f"n{n}.vram.jsonl"))
        summary = summarize_run(feats, pgt, traj, baseline_map)
        prov = None
        if (dir_ / "off.provenance.json").exists():
            prov = json.load(open(dir_ / "off.provenance.json"))
        sweep_run_path = dir_ / "sweep-run.json"
        sweep_run = json.load(open(sweep_run_path)) if sweep_run_path.exists() else {}
        elapsed = sweep_run.get("elapsed_sec")
        rc = sweep_run.get("return_code")
        # log-based OOM count
        log_path = sweep / "logs" / f"n{n}.log"
        oom = 0
        if log_path.exists():
            try:
                with open(log_path, errors="ignore") as lf:
                    for line in lf:
                        if "OutOfMemoryError" in line or "건너뜀" in line:
                            oom += 1
            except Exception:
                pass
        expected_dets = 2979
        completeness = summary["total_detections"] / expected_dets if expected_dets else None
        effectively_complete = completeness >= 0.95 if completeness is not None else False

        entry = {
            "npoint": n,
            "return_code": rc,
            "elapsed_sec": elapsed,
            "per_detection_sec": ((elapsed / summary["total_detections"])
                                  if elapsed and summary["total_detections"] else None),
            "vram_max_mib": vram["max_mib"],
            "vram_mean_mib": vram["mean_mib"],
            "oom_log_lines": oom,
            "completeness_fraction": completeness,
            "effectively_complete": effectively_complete,
            "provenance_setting": (prov or {}).get("run", {}).get("setting"),
            "config_sha256": (prov or {}).get("run", {}).get("config_sha256"),
            "pem_coarse_npoint_effective": (prov or {}).get("run", {}).get("pem_coarse_npoint_effective"),
            "bag_sha256": (prov or {}).get("dataset_snapshot", {}).get("bag_database_sha256"),
            "summary": {k: v for k, v in summary.items() if not k.startswith("_")},
        }
        all_runs.append(entry)
        if n == args.baseline_npoint:
            baseline_map = summary["_detections_by_key"]
            print(f"[analyze] adopted n={n} as baseline for pose-change comparison", flush=True)

    # If baseline never assigned (baseline npoint not run), still emit
    top = {"sweep_dir": str(sweep),
           "pseudo_gt": str(args.pseudo_gt) if pgt else None,
           "pseudo_gt_objects": sorted(pgt.keys()),
           "trajectory": str(args.trajectory) if traj else None,
           "trajectory_pose_count": len(traj),
           "baseline_npoint": args.baseline_npoint,
           "acceptance": {
               "valid_n_pts_in_min": VALID_N_PTS,
               "valid_depth_valid_frac_min": VALID_DEPTH_FRAC,
               "coverage_min": COVER_GATE,
               "iou_gate": IOU_GATE, "overlap_gate": OVERLAP_GATE,
               "mean_distance_gate_mm": MEAN_DIST_GATE_MM,
               "rot_gate_deg": ROT_GATE_DEG,
               "trans_gate_mm": TRANS_GATE_MM,
           },
           "runs": all_runs}

    # JSON
    with open(out / "point-count-sweep.json", "w") as f:
        json.dump(top, f, indent=2, ensure_ascii=False)

    def _pct(num, den):
        return (num / den) if den else None

    # CSV
    csv_path = out / "point-count-sweep.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["npoint", "return_code", "elapsed_sec", "per_det_sec", "vram_max_mib",
                    "total_det", "well_observed", "well_covered",
                    "iou_ge_050_all", "iou_ge_050_well_obs",
                    "target_denom", "success_target", "success_target_rate",
                    "success_all", "success_well_obs",
                    "iou_p50", "iou_p75", "iou_p90", "iou_p95",
                    "mean_dist_p50", "mean_dist_p90",
                    "handle_visible", "handle_visible_pass", "handle_visible_fail",
                    "pose_changes_vs_base", "pose_improved_vs_base", "pose_worsened_vs_base"])
        for e in all_runs:
            s = e["summary"]
            denom = s["iou_ge_050_count_well_obs"] or 0
            w.writerow([e["npoint"], e["return_code"], e["elapsed_sec"], e["per_detection_sec"],
                        e["vram_max_mib"], s["total_detections"],
                        s["well_observed_count"], s["well_covered_count"],
                        s["iou_ge_050_count_all"], s["iou_ge_050_count_well_obs"],
                        denom, s["success_count_target"], _pct(s["success_count_target"], denom),
                        s["success_count_all"], s["success_count_well_obs"],
                        s["iou_p50"], s["iou_p75"], s["iou_p90"], s["iou_p95"],
                        s["mean_dist_p50"], s["mean_dist_p90"],
                        s["handle_visible_frames"], s["handle_visible_pass"],
                        s["handle_visible_fail"],
                        s["pose_changes_vs_base"],
                        s["pose_improved_vs_base"], s["pose_worsened_vs_base"]])

    # Detail JSONL
    for tag in ("detail_pose_changed", "detail_iou_pass_but_fail", "detail_mugcup_handle_fail"):
        path = out / f"{tag}.jsonl"
        with open(path, "w") as f:
            for e in all_runs:
                rows = e["summary"].get(tag) or []
                for r in rows:
                    r2 = dict(r); r2["_npoint"] = e["npoint"]
                    f.write(json.dumps(r2, ensure_ascii=False) + "\n")

    # Per-object CSV
    with open(out / "per-object-success.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["npoint", "object", "total", "well_observed", "well_covered",
                    "iou_ge_050", "success_target", "success_rate",
                    "iou_p50", "mean_dist_p50"])
        for e in all_runs:
            for obj, po in sorted(e["summary"]["per_object"].items()):
                denom = po["iou_ge_050"]
                rate = (po["success_target"] / denom) if denom else None
                w.writerow([e["npoint"], obj, po["total"], po["well_observed"],
                            po["well_covered"], po["iou_ge_050"], po["success_target"],
                            rate, po["iou_p50"], po["mean_dist_p50"]])

    # Mugcup 60/61 detailed comparison
    with open(out / "mugcup-60-61-detail.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["npoint", "i", "stamp_ns", "proposal6000_index", "geo_rank",
                    "geo_point_count", "geo_mean_distance_mm", "geo_p90_distance_mm",
                    "geo_overlap_ratio", "mask_iou", "coverage",
                    "pose_score", "score", "R_row0", "R_row1", "R_row2", "t_mm",
                    "pgt_rot_err_deg", "pgt_trans_err_mm", "success"])
        for e in all_runs:
            m = e["summary"].get("mugcup_60_61") or {}
            for i in (60, 61):
                d = m.get(i)
                if not d:
                    continue
                reasons = d.get("reasons") or {}
                w.writerow([e["npoint"], i, d["stamp_ns"], d["selected_proposal6000_index"],
                            d["selected_geo_rank"], d["geo_point_count"],
                            d["geo_mean_distance_mm"], d["geo_p90_distance_mm"],
                            d["geo_overlap_ratio"], d["mask_iou"], d["coverage"],
                            d["pose_score"], d["score"],
                            d["R"][0], d["R"][1], d["R"][2], d["t_mm"],
                            reasons.get("pgt_rot_err_deg"),
                            reasons.get("pgt_trans_err_mm"), d["pose_success"]])

    # Markdown
    md_lines = []
    md_lines.append("# PEM coarse_npoint sweep — analysis\n")
    md_lines.append(f"- sweep dir: `{sweep}`")
    md_lines.append(f"- pseudo-GT: `{args.pseudo_gt}` — objects: {sorted(pgt.keys())}")
    md_lines.append(f"- trajectory: `{args.trajectory}` — {len(traj)} poses")
    md_lines.append(f"- baseline for pose-change: n={args.baseline_npoint}")
    md_lines.append("")
    md_lines.append("## Acceptance definition")
    md_lines.append(f"- well_observed (관측 유의미): `depth_valid_frac >= {VALID_DEPTH_FRAC}`"
                    f" AND `observed_mask_area_px >= {VALID_OBS_MASK_PX}`")
    md_lines.append(f"- well_covered (참고, pose 종속): `coverage >= {COVER_GATE}`")
    md_lines.append(f"- IoU_gate: `mask_iou >= {IOU_GATE}`")
    md_lines.append(f"- pose_success (기하): `mask_iou >= {IOU_GATE}` AND"
                    f" `overlap_ratio >= {OVERLAP_GATE}` AND"
                    f" `mean_distance_mm <= {MEAN_DIST_GATE_MM}`")
    md_lines.append(f"- pose_success (GT trusted 시 추가): rot_err ≤ {ROT_GATE_DEG}°"
                    f" (대칭 반영) AND trans_err ≤ {TRANS_GATE_MM} mm")
    md_lines.append(f"- **success_target** = well_observed AND IoU>=0.50 AND pose_success —"
                    f" 사용자의 핵심 성공 기준: 이 집합의 rate 가 1 에 가까워져야 한다")
    md_lines.append("")
    md_lines.append("## Sweep overview")
    md_lines.append("- **complete?** = detections written / 2979 ≥ 0.95 (OOM 이 대부분 프레임을 잡아먹으면 False)")
    md_lines.append("")
    md_lines.append("| n | rc | complete? | OOM | elapsed_s | per-det ms | VRAM MiB | total |"
                    " well_obs | IoU>=0.5(well_obs) | success_target | rate |"
                    " IoU p50 | mean_dist p50 mm | vs-base changes | improved | worsened |")
    md_lines.append("|---|----|----------:|----:|----------:|-----------:|---------:|------:|"
                    "--------:|-------------------:|---------------:|-----:|"
                    "--------:|-----------------:|----------------:|---------:|---------:|")
    for e in all_runs:
        s = e["summary"]
        denom = s["iou_ge_050_count_well_obs"] or 0
        rate = (s["success_count_target"] / denom * 100) if denom else None
        rate_s = f"{rate:.2f}%" if rate is not None else "-"
        pdms = (e["per_detection_sec"] * 1000) if e["per_detection_sec"] else 0.0
        elapsed = e['elapsed_sec'] if e['elapsed_sec'] is not None else 0.0
        comp = f"YES ({e['completeness_fraction']:.2f})" if e.get("effectively_complete") \
               else f"NO ({(e.get('completeness_fraction') or 0):.2f})"
        md_lines.append(f"| {e['npoint']} | {e['return_code']} | {comp} |"
                        f" {e.get('oom_log_lines', 0)} | {elapsed:.1f} |"
                        f" {pdms:.1f} | {e['vram_max_mib']} |"
                        f" {s['total_detections']} | {s['well_observed_count']} |"
                        f" {s['iou_ge_050_count_well_obs']} | {s['success_count_target']} |"
                        f" {rate_s} |"
                        f" {s['iou_p50']} | {s['mean_dist_p50']} |"
                        f" {s['pose_changes_vs_base']} | {s['pose_improved_vs_base']} |"
                        f" {s['pose_worsened_vs_base']} |")

    # Mugcup handle table
    md_lines.append("")
    md_lines.append("## Mugcup handle-visible frames & GT-only correctness")
    md_lines.append("| n | GT avail | GT rot≤45° | GT flipped(>90°) | handle-vis | handle-flip | flip rate |")
    md_lines.append("|---|--------:|----------:|----------------:|-----------:|-----------:|---------:|")
    for e in all_runs:
        s = e["summary"]
        fr = (s.get('mugcup_handle_flip', 0) / s['handle_visible_frames'] * 100) if s['handle_visible_frames'] else None
        fr_s = f"{fr:.1f}%" if fr is not None else "-"
        md_lines.append(f"| {e['npoint']} | {s.get('mugcup_gt_available', 0)} |"
                        f" {s.get('mugcup_gt_correct', 0)} | {s.get('mugcup_gt_flipped', 0)} |"
                        f" {s['handle_visible_frames']} | {s.get('mugcup_handle_flip', 0)} | {fr_s} |")

    # Mugcup 60/61
    md_lines.append("")
    md_lines.append("## Mugcup frame 60/61 selected proposal")
    md_lines.append("| n | frame | proposal6000_idx | geo_rank | point_count |"
                    " mean_dist mm | p90_dist mm | overlap_ratio | mask_iou | coverage |"
                    " pgt_rot_err | pgt_trans_err | success |")
    md_lines.append("|---|------:|-----------------:|---------:|------------:|-----------:|----------:|"
                    "-------------:|--------:|--------:|-----------:|-------------:|--------:|")
    for e in all_runs:
        m = e["summary"].get("mugcup_60_61") or {}
        for i in (60, 61):
            d = m.get(i)
            if not d:
                continue
            reasons = d.get("reasons") or {}
            md_lines.append(f"| {e['npoint']} | {i} | {d['selected_proposal6000_index']} |"
                            f" {d['selected_geo_rank']} | {d['geo_point_count']} |"
                            f" {d['geo_mean_distance_mm']} | {d['geo_p90_distance_mm']} |"
                            f" {d['geo_overlap_ratio']} | {d['mask_iou']} | {d['coverage']} |"
                            f" {reasons.get('pgt_rot_err_deg')} |"
                            f" {reasons.get('pgt_trans_err_mm')} | {d['pose_success']} |")

    md_lines.append("")
    md_lines.append("## Per-object success (denom = well_observed AND IoU>=0.5)")
    md_lines.append("| n | object | total | well_obs | IoU>=0.5 | success_target | rate |"
                    " IoU p50 | mean_dist p50 mm |")
    md_lines.append("|---|--------|------:|--------:|--------:|--------------:|-----:|"
                    "--------:|-----------------:|")
    for e in all_runs:
        for obj, po in sorted(e["summary"]["per_object"].items()):
            r = f"{(po['success_target'] / po['iou_ge_050'] * 100):.1f}%" if po["iou_ge_050"] else "-"
            md_lines.append(f"| {e['npoint']} | {obj} | {po['total']} |"
                            f" {po['well_observed']} | {po['iou_ge_050']} |"
                            f" {po['success_target']} | {r} |"
                            f" {po['iou_p50']} | {po['mean_dist_p50']} |")

    md_lines.append("")
    md_lines.append("## Saturation & recommendation")
    complete = [e for e in all_runs if e.get("effectively_complete")]
    incomplete = [e for e in all_runs if not e.get("effectively_complete")]
    if incomplete:
        md_lines.append("- **완전 실행이 아니어서 성공률 산정에서 제외한 point count:**")
        for e in incomplete:
            md_lines.append(f"  - n={e['npoint']}: {e['summary']['total_detections']}/2979 detections,"
                            f" OOM {e.get('oom_log_lines', 0)} lines")
    saturated_at = None
    prev = None; prev_np = None
    for e in complete:
        s = e["summary"]
        denom = s["iou_ge_050_count_well_obs"] or 0
        rate = (s["success_count_target"] / denom) if denom else 0.0
        if prev is not None:
            gain_pp = (rate - prev) * 100.0
            # 포화점 = 다음 값에서 상승이 1pp 미만 (감소 포함)
            if gain_pp < 1.0:
                saturated_at = prev_np
                break
        prev = rate
        prev_np = e["npoint"]
    if saturated_at is None and complete:
        # 스윕 끝까지 상승만 있었다면 마지막 완전 실행값을 포화점으로 본다.
        saturated_at = complete[-1]["npoint"]
    md_lines.append(f"- 포화점 (완전 실행 중 다음 n 에서 rate 상승 < 1pp): **{saturated_at}**")
    if saturated_at is not None:
        cand = [e for e in complete if e["npoint"] <= saturated_at]
        if cand:
            best = min(cand, key=lambda e: (e["elapsed_sec"] or 10**9, e["npoint"]))
            md_lines.append(f"- 추천 운영값 npoint = **{best['npoint']}**"
                            f" (elapsed {best['elapsed_sec']:.1f}s, VRAM {best['vram_max_mib']} MiB)")
            md_lines.append("")
            md_lines.append("### 선택 근거 (수치)")
            for e in complete:
                s = e["summary"]
                denom = s["iou_ge_050_count_well_obs"] or 0
                rate = (s["success_count_target"] / denom * 100) if denom else 0.0
                marker = " ← 추천" if e["npoint"] == best["npoint"] else ""
                md_lines.append(f"- n={e['npoint']}: target success rate = {rate:.2f}%"
                                f" ({s['success_count_target']}/{denom}),"
                                f" IoU p50 = {s['iou_p50']}, mean_dist p50 = {s['mean_dist_p50']} mm,"
                                f" mugcup_gt_correct = {s.get('mugcup_gt_correct', 0)}/473,"
                                f" mugcup_handle_flip = {s.get('mugcup_handle_flip', 0)},"
                                f" elapsed = {e['elapsed_sec']}s,"
                                f" VRAM max = {e['vram_max_mib']} MiB{marker}")

    with open(out / "point-count-sweep.md", "w") as f:
        f.write("\n".join(md_lines) + "\n")

    print(f"[analyze] wrote {out}", flush=True)


if __name__ == "__main__":
    main()
