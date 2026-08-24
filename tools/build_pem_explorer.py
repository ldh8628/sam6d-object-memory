#!/usr/bin/env python3
"""Build a reusable, dependency-free PEM pose explorer.

The output is a static bundle (index.html, app.css, app.js, report-data.js,
assets/) and opens directly through file://.  Older top-100 dumps are accepted
in partial mode: absent 6000/300 and pointwise evidence is explicitly marked
``uncollected`` and is never converted to a false result.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import shlex
import shutil
import tempfile
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

try:
    from tools.validate_rgbd_dataset import (COLOR_INFO_TOPIC, COLOR_TOPIC,
                                             DatasetValidationError,
                                             enforce_ground_truth_policy,
                                             validate_evaluation_thresholds,
                                             validate_trusted_reference_poses,
                                             validate_rgbd_dataset)
except ModuleNotFoundError:  # direct `python tools/build_pem_explorer.py`
    from validate_rgbd_dataset import (COLOR_INFO_TOPIC, COLOR_TOPIC,
                                       DatasetValidationError,
                                       enforce_ground_truth_policy,
                                       validate_evaluation_thresholds,
                                       validate_trusted_reference_poses,
                                       validate_rgbd_dataset)

try:
    from tools.build_pem_verification_policy import (
        build_policy as build_scored_policy, detection_fingerprint)
except ModuleNotFoundError:
    from build_pem_verification_policy import (
        build_policy as build_scored_policy, detection_fingerprint)


REPO = Path(__file__).resolve().parents[1]
TEMPLATE_DIR = Path(__file__).resolve().parent / "pem_explorer"
SCHEMA_VERSION = 1
COMPLETION_MARKER = ".pem-explorer-complete"
COMPLETION_MARKER_CONTENT = "PEM_EXPLORER_COMPLETE_V1\n"
COMPLETED_REPORT_MARKERS = ("index.html", "app.css", "app.js", "report-data.js",
                            "verification-policy.js", "README.md", COMPLETION_MARKER)
MIN_FULL_EVIDENCE_FREE_BYTES = 50 * 1024 ** 3
MIN_FULL_EVIDENCE_FREE_INODES = 1_600_000
DEFAULT_OBJECTS = [
    "milk", "choco_hazelnut_high", "Febreze_high", "Mugcup_high", "saffron",
    "Sauce_high", "Sikhye_high", "Bear", "Dinosaur",
]
SYM_AXES = {
    "Sikhye_high": (0.0, 0.0, 1.0),
    "Sauce_high": (0.0, 0.0, 1.0),
    "Mugcup_high": (0.1057, 0.0337, 0.9938),
}


class InputError(ValueError):
    pass


def checked_imwrite(path, image, params=None):
    path = Path(path)
    ok = cv2.imwrite(str(path), image, params or [])
    if not ok or not path.is_file() or path.stat().st_size <= 0:
        raise InputError(f"failed to write image: {path}")


def verification_filter_policy(args):
    """Validate an optional shadow-only texture/IoU rejection policy.

    The policy never changes the geometry-selected pose. It is emitted as a
    small sidecar so a calibrated policy does not need to be embedded in the
    potentially very large report-data.js payload.
    """
    names = ("validator_depth_min", "validator_texture_min", "validator_iou_min")
    values = [getattr(args, name, None) for name in names]
    if not any(value is not None for value in values):
        return {
            "schema_version": 1,
            "enabled": False,
            "mode": "shadow_only",
            "reason": "thresholds_not_configured",
        }
    if not all(value is not None for value in values):
        raise InputError("validator depth/texture/IoU thresholds must be supplied together")
    numeric = []
    for name, value in zip(names, values):
        if isinstance(value, bool):
            raise InputError(f"{name} must be a finite number in [0, 1]")
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise InputError(f"{name} must be a finite number in [0, 1]") from exc
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise InputError(f"{name} must be a finite number in [0, 1]")
        numeric.append(value)
    depth, texture, iou = numeric
    return {
        "schema_version": 1,
        "enabled": True,
        "mode": "shadow_only",
        "selection_method": "geometry_only_unchanged",
        "depth_valid_frac_min": depth,
        "texture_score_min": texture,
        "mask_iou_min": iou,
        "reject_when": "texture_score_below_min_AND_mask_iou_below_min",
        "scope_note": "Thresholds are evaluated only when depth_valid_frac meets the minimum.",
    }


def load_jsonl(path):
    rows = []
    with Path(path).open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise InputError(f"{path}:{number}: invalid JSON: {exc}") from exc
    return rows


def load_detections_spooling_pointwise(path, spool_dir):
    """Keep large pointwise arrays on disk while compact detection rows stay in memory."""
    rows = []
    spool_dir = Path(spool_dir)
    spool_dir.mkdir(parents=True, exist_ok=True)
    with Path(path).open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise InputError(f"{path}:{number}: invalid JSON: {exc}") from exc
            verify = row.get("verify") if isinstance(row.get("verify"), dict) else {}
            diagnostic = (row.get("diagnostic")
                          if isinstance(row.get("diagnostic"), dict) else {})
            verify_pointwise = verify.pop("pointwise", None)
            diagnostic_pointwise = diagnostic.pop("pointwise", None)
            if (verify_pointwise is not None and diagnostic_pointwise is not None and
                    verify_pointwise != diagnostic_pointwise):
                raise InputError(f"{path}:{number}: conflicting pointwise evidence copies")
            pointwise = (verify_pointwise if verify_pointwise is not None
                         else diagnostic_pointwise)
            if pointwise:
                spool_path = spool_dir / f"{number:06d}.json"
                spool_path.write_text(
                    json.dumps(pointwise, separators=(",", ":"), allow_nan=False),
                    encoding="utf-8")
                row["_pointwise_spool"] = str(spool_path)
            rows.append(row)
    return rows


def resolved_verification_filter_policy(args, dataset_id):
    policy = verification_filter_policy(args)
    detections = getattr(args, "validator_score_detections", "") or ""
    candidates = getattr(args, "validator_candidate_analysis", "") or ""
    if bool(detections) != bool(candidates):
        raise InputError("validator score detections and candidate analysis must be supplied together")
    if detections:
        if not policy["enabled"]:
            raise InputError("validator score sources require all three validator thresholds")
        try:
            detection_rows = load_jsonl(detections)
            pose_source = getattr(args, "validator_pose_detections", "") or ""
            pose_rows = load_jsonl(pose_source) if pose_source else detection_rows
            policy = build_scored_policy(
                detection_rows, load_jsonl(candidates),
                policy["depth_valid_frac_min"], policy["texture_score_min"],
                policy["mask_iou_min"], pose_rows)
        except (OSError, ValueError) as exc:
            raise InputError(f"cannot build validator decisions: {exc}") from exc
        policy["score_sources"] = {
            "detections": str(detections), "candidates": str(candidates)}
    policy["dataset_id"] = dataset_id
    return policy


def validate_detection(row, source="detections"):
    required = ("stamp_ns", "object", "R", "t_mm")
    missing = [key for key in required if key not in row]
    if missing:
        raise InputError(f"{source}: missing detection fields: {', '.join(missing)}")
    if np.asarray(row["R"]).shape != (3, 3) or np.asarray(row["t_mm"]).shape != (3,):
        raise InputError(f"{source}: R must be 3x3 and t_mm must have 3 values")
    if "diagnostic" in row and row["diagnostic"] is not None and not isinstance(
            row["diagnostic"], dict):
        raise InputError(f"{source}: detection diagnostic must be an object")


def axis_rotation(axis, degrees):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    return Rotation.from_rotvec(axis * math.radians(degrees)).as_matrix()


def angle_deg(a, b, name=None):
    symmetries = [np.eye(3)]
    if name in SYM_AXES:
        symmetries = [axis_rotation(SYM_AXES[name], d) for d in range(0, 360, 10)]
    return float(min(math.degrees(math.acos(np.clip(
        (np.trace(np.asarray(a).T @ (np.asarray(b) @ sym)) - 1.0) / 2.0, -1.0, 1.0)))
                     for sym in symmetries))


def project_axis(rotation, translation_mm, axis, length_mm, K):
    """Project an object-space axis. Returns [[u0,v0],[u1,v1]] or None."""
    try:
        R = np.asarray(rotation, float)
        t = np.asarray(translation_mm, float)
        axis = np.asarray(axis, float)
        K = np.asarray(K, float)
    except (TypeError, ValueError, OverflowError):
        return None
    if (R.shape != (3, 3) or t.shape != (3,) or axis.shape != (3,) or
            K.shape != (3, 3) or not np.isfinite(R).all() or
            not np.isfinite(t).all() or not np.isfinite(axis).all() or
            not np.isfinite(K).all() or not np.isfinite(length_mm) or
            np.linalg.norm(axis) <= 1e-9 or float(length_mm) <= 0):
        return None
    q = np.stack([t, t + R @ (axis * float(length_mm))])
    if not np.isfinite(q).all() or np.any(q[:, 2] <= 1e-6):
        return None
    uv = (K @ (q / q[:, 2:3]).T).T[:, :2]
    if not np.isfinite(uv).all():
        return None
    return np.round(uv, 3).tolist()


def pose_transform_error(rotation, translation_mm):
    """Return why a selected rigid transform is unsafe to display, or None."""
    try:
        R = np.asarray(rotation, float)
        t = np.asarray(translation_mm, float)
    except (TypeError, ValueError, OverflowError):
        return "nonnumeric_transform"
    if R.shape != (3, 3) or t.shape != (3,):
        return "invalid_shape"
    if not np.isfinite(R).all() or not np.isfinite(t).all():
        return "nonfinite_transform"
    determinant = float(np.linalg.det(R))
    if (not np.isfinite(determinant) or determinant <= 0.0 or
            not np.allclose(R.T @ R, np.eye(3), rtol=0.0, atol=1e-3) or
            not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1e-3)):
        return "degenerate_rotation"
    return None


def pose_matrix(rotation, translation_m):
    result = np.eye(4)
    result[:3, :3] = np.asarray(rotation, float)
    result[:3, 3] = np.asarray(translation_m, float)
    return result


def load_trajectory(path):
    arr = np.loadtxt(path, ndmin=2)
    if arr.shape[1] != 8:
        raise InputError(f"trajectory must have 8 columns: {path}")
    if not np.isfinite(arr).all():
        raise InputError(f"trajectory contains NaN/Inf: {path}")
    if np.any(np.diff(arr[:, 0]) <= 0):
        raise InputError(f"trajectory timestamps must be strictly increasing: {path}")
    norms = np.linalg.norm(arr[:, 4:8], axis=1)
    if np.any(norms < 1e-9) or not np.allclose(norms, 1.0, atol=1e-3, rtol=0.0):
        raise InputError(f"trajectory quaternions must be normalized: {path}")
    poses = []
    for row in arr:
        poses.append(pose_matrix(Rotation.from_quat(row[4:8]).as_matrix(), row[1:4]))
    return arr[:, 0], np.asarray(poses)


def nearest_pose(stamp_ns, times, poses, max_dt=0.025):
    if times is None:
        return None
    stamp = int(stamp_ns) / 1e9
    j = int(np.searchsorted(times, stamp))
    candidates = [k for k in (j - 1, j) if 0 <= k < len(times)]
    if not candidates:
        return None
    best = min(candidates, key=lambda k: abs(float(times[k] - stamp)))
    return poses[best] if abs(float(times[best] - stamp)) <= max_dt else None


def assess_pose(rotation, translation_mm, name, gt, twc,
                rotation_limit=30.0, translation_limit_m=0.10,
                missing_status="gt_missing"):
    if not isinstance(gt, dict) or gt.get("trusted") is not True:
        return {"status": missing_status, "ok": None}
    if twc is None:
        return {"status": "slam_missing", "ok": None}
    tco = pose_matrix(rotation, np.asarray(translation_mm, float) / 1000.0)
    two = twc @ tco
    rerr = angle_deg(np.asarray(gt["R"]), two[:3, :3], name)
    terr = float(np.linalg.norm(np.asarray(gt["t_m"]) - two[:3, 3]))
    return {"status": "available", "ok": bool(rerr <= rotation_limit and terr <= translation_limit_m),
            "rotation_error_deg": round(rerr, 3),
            "translation_error_mm": round(terr * 1000.0, 2)}


def stage_value(diagnostic, name):
    value = (diagnostic or {}).get(name)
    if value is None:
        return {"status": "uncollected", "present": None}
    if not isinstance(value, dict):
        raise InputError(f"{name} diagnostic must be an object")
    if not value:
        return {"status": "uncollected", "present": None}
    if ("present" in value and value["present"] is not None and
            type(value["present"]) is not bool):
        raise InputError(f"{name}.present must be a boolean or null")
    if value.get("status") != "available":
        return {**value, "present": None}
    if type(value.get("present")) is not bool:
        raise InputError(f"{name}.present must be a boolean when status is available")
    return value


def mark_unavailable_reason(value, missing_status):
    """Preserve uncollected states while making a known GT blocker explicit."""
    if missing_status == "extrinsic_missing" and value.get("status") != "uncollected":
        # An embedded "available" result may have been computed against a different
        # camera.  Do not retain any GT-derived O/X or error/count fields.
        return {"status": "extrinsic_missing", "present": None,
                "total": value.get("total"), "suppressed": True,
                "reason": "validated inter-camera extrinsic is unavailable"}
    if missing_status != "gt_missing" and value.get("status") == "gt_missing":
        return {**value, "status": missing_status, "present": None}
    return value


def suppress_stage_for_gt(value, blocker):
    """Remove any O/X payload when the finalized selected pose has no usable GT."""
    if value.get("status") == "uncollected":
        return {"status": "uncollected", "present": None}
    if value.get("status") == blocker and value.get("present") is None:
        return value
    return {"status": blocker, "present": None, "suppressed": True}


def object_state(detection, frame_diagnostic, name):
    if detection is not None:
        return "detected"
    items = ((frame_diagnostic or {}).get("pem_candidates") or [])
    if any(x.get("object") == name and x.get("input_rejection") for x in items):
        return "pem_input_rejected"
    return "not_detected"


def active_objects(config_path):
    cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    objects = [x["name"] for x in cfg.get("objects", [])
               if x.get("enabled", True) and x.get("name") != "Rabbit"]
    if len(objects) != 9:
        raise InputError(f"expected 9 enabled non-Rabbit objects in {config_path}, got {len(objects)}")
    return objects


def load_axes(path, objects):
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    result = {}
    for name in objects:
        item = raw.get("objects", {}).get(name, raw.get("default", {}))
        if not all(k in item for k in ("front_axis", "up_axis", "length_mm")):
            raise InputError(f"axis manifest missing front/up/length for {name}: {path}")
        result[name] = item
    return {"source": raw.get("source", str(path)), "objects": result}


def extract_rgb(bag, wanted, out_dir, color_topic, camera_info_topic):
    """Extract exact timestamped RGB frames and camera K from a ROS2 bag."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    out_dir.mkdir(parents=True, exist_ok=True)
    wanted = set(int(x) for x in wanted)
    found, K, shape = set(), None, None
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(bag)], default_typestore=typestore) as reader:
        color = [c for c in reader.connections if c.topic == color_topic]
        info = [c for c in reader.connections if c.topic == camera_info_topic]
        if not color:
            raise InputError(f"bag has no color topic {color_topic}: {bag}")
        if info:
            for connection, _, raw in reader.messages(connections=info):
                msg = reader.deserialize(raw, connection.msgtype)
                K = np.asarray(msg.k, float).reshape(3, 3)
                break
        for connection, _, raw in reader.messages(connections=color):
            msg = reader.deserialize(raw, connection.msgtype)
            if not hasattr(msg, "step") or not hasattr(msg, "encoding"):
                raise InputError(f"color topic must contain sensor_msgs/Image, not {connection.msgtype}")
            stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if stamp not in wanted:
                continue
            encoding = str(msg.encoding).lower()
            channels = 1 if encoding in ("mono8", "8uc1") else 3
            row_bytes = int(msg.step)
            needed = int(msg.width) * channels
            if row_bytes < needed:
                raise InputError(f"invalid image step={row_bytes}, need at least {needed}")
            rows = np.frombuffer(msg.data, np.uint8).reshape(int(msg.height), row_bytes)
            arr = rows[:, :needed].reshape(int(msg.height), int(msg.width), channels)
            if channels == 1:
                bgr = cv2.cvtColor(arr[:, :, 0], cv2.COLOR_GRAY2BGR)
            elif str(msg.encoding).lower() == "rgb8":
                bgr = cv2.cvtColor(arr[:, :, :3], cv2.COLOR_RGB2BGR)
            else:
                bgr = arr[:, :, :3].copy()
            image_path = out_dir / f"{stamp}.jpg"
            checked_imwrite(image_path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 82])
            found.add(stamp)
            shape = [int(msg.width), int(msg.height)]
    missing = sorted(wanted - found)
    if missing:
        raise InputError(f"bag is missing {len(missing)} requested RGB stamps; first={missing[0]}")
    return K, shape


def render_pose_proxy(frame, detection, candidate, model_points, K, out_path):
    """Render a candidate projection and return explicit projection status."""
    bbox = detection.get("bbox") or [0, 0, frame.shape[1], frame.shape[0]]
    x1, y1, x2, y2 = [int(v) for v in bbox]
    pad = max(8, int(max(x2 - x1, y2 - y1) * 0.35))
    xa, ya = max(0, x1 - pad), max(0, y1 - pad)
    xb, yb = min(frame.shape[1], x2 + pad), min(frame.shape[0], y2 + pad)
    view = frame.copy()
    reason = pose_transform_error(candidate.get("R"), candidate.get("t_mm"))
    if reason is not None:
        crop = np.zeros((224, 224, 3), np.uint8)
        cv2.putText(crop, "PROJECTION UNAVAILABLE", (12, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (50, 180, 255), 1, cv2.LINE_AA)
        cv2.putText(crop, reason[:28], (12, 126),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
        checked_imwrite(out_path, crop)
        return {"status": "unavailable", "reason": reason}
    P = model_points @ np.asarray(candidate["R"], float).T + np.asarray(candidate["t_mm"], float)
    P = P[P[:, 2] > 1.0]
    visible_count = 0
    if len(P):
        uv = (np.asarray(K) @ (P / P[:, 2:3]).T).T[:, :2].astype(int)
        ok = ((uv[:, 0] >= 0) & (uv[:, 0] < view.shape[1]) &
              (uv[:, 1] >= 0) & (uv[:, 1] < view.shape[0]))
        visible_count = int(ok.sum())
        view[uv[ok, 1], uv[ok, 0]] = (30, 210, 255)
    if visible_count == 0:
        reason = "no_positive_depth_points" if len(P) == 0 else "projection_outside_image"
        crop = np.zeros((224, 224, 3), np.uint8)
        cv2.putText(crop, "PROJECTION UNAVAILABLE", (12, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (50, 180, 255), 1, cv2.LINE_AA)
        cv2.putText(crop, reason[:28], (12, 126),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
        checked_imwrite(out_path, crop)
        return {"status": "unavailable", "reason": reason}
    crop = view[ya:yb, xa:xb]
    if not crop.size:
        reason = "empty_detection_crop"
        crop = np.zeros((224, 224, 3), np.uint8)
        cv2.putText(crop, "PROJECTION UNAVAILABLE", (12, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (50, 180, 255), 1, cv2.LINE_AA)
        cv2.putText(crop, reason, (12, 126),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
    checked_imwrite(out_path, cv2.resize(crop, (224, 224)))
    return ({"status": "unavailable", "reason": reason} if reason else
            {"status": "available", "visible_point_count": visible_count})


def render_point_heatmaps(frame, detection, evidence, prefix):
    # uv_224는 PEM이 실제 사용한 정규화 crop 좌표다. 원 실행은 그 crop 자체를
    # 저장하지 않았으므로 ISM bbox 영상 위에 겹치면 정확한 정합처럼 오해된다.
    # 중립 배경에 실제 표본 좌표만 그려 좌표계와 측정값을 보존한다.
    base = np.full((224, 224, 3), 28, np.uint8)
    cv2.rectangle(base, (0, 0), (223, 223), (85, 85, 85), 1)
    geo, tex = base.copy(), base.copy()
    threshold = float(evidence.get("geometry", {}).get("threshold_mm", 10.0))
    for point in evidence.get("geometry_points", evidence.get("points", [])):
        if point.get("uv_224") is None:
            continue
        u, v = [int(x) for x in point["uv_224"]]
        gd = float(point.get("distance_mm", threshold * 2))
        gv = max(0.0, min(1.0, gd / max(threshold, 1e-6)))
        cv2.circle(geo, (u, v), 2, (0, int(255 * (1 - gv)), int(255 * gv)), -1)
    for point in evidence.get("texture_points", evidence.get("points", [])):
        if point.get("uv_224") is None:
            continue
        u, v = [int(x) for x in point["uv_224"]]
        tv = (float(point.get("feature_cosine", 0.0)) + 1.0) * 0.5
        cv2.circle(tex, (u, v), 2, (int(255 * (1 - tv)), int(255 * tv), 40), -1)
    checked_imwrite(prefix.with_name(prefix.name + "_geometry.jpg"), geo)
    checked_imwrite(prefix.with_name(prefix.name + "_texture.jpg"), tex)


def decode_mask_rle(size, runs):
    """Decode the compact row-major RLE emitted by independent PEM verification."""
    if (not isinstance(size, (list, tuple)) or len(size) != 2 or
            not all(isinstance(v, int) and 0 < v <= 4096 for v in size)):
        raise InputError("shape mask size must contain two positive integers")
    flat = np.zeros(int(size[0]) * int(size[1]), np.uint8)
    for run in runs or []:
        if (not isinstance(run, (list, tuple)) or len(run) != 2 or
                not all(isinstance(v, int) for v in run)):
            raise InputError("shape mask RLE entries must be [start, length]")
        start, length = run
        if start < 0 or length <= 0 or start + length > flat.size:
            raise InputError("shape mask RLE entry is out of bounds")
        flat[start:start + length] = 1
    return flat.reshape(int(size[0]), int(size[1])).astype(bool)


def projected_point_mask(candidate, model_points, K, shape_meta):
    size = shape_meta.get("size") or [224, 224]
    h, w = [int(v) for v in size]
    crop = shape_meta.get("crop_bbox_yxyx")
    if not isinstance(crop, list) or len(crop) != 4:
        return np.zeros((h, w), bool)
    y1, y2, x1, x2 = [float(v) for v in crop]
    points = (np.asarray(model_points, float) @ np.asarray(candidate["R"], float).T +
              np.asarray(candidate["t_mm"], float))
    points = points[np.isfinite(points).all(1) & (points[:, 2] > 1e-6)]
    mask = np.zeros((h, w), np.uint8)
    if len(points):
        uv = (np.asarray(K, float) @ (points / points[:, 2:3]).T).T[:, :2]
        u = np.rint((uv[:, 0] - x1) * w / max(x2 - x1, 1.0)).astype(int)
        v = np.rint((uv[:, 1] - y1) * h / max(y2 - y1, 1.0)).astype(int)
        valid = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        mask[v[valid], u[valid]] = 255
    radius = max(0, int(shape_meta.get("point_splat_radius_px", 2)))
    if radius:
        kernel = np.ones((2 * radius + 1, 2 * radius + 1), np.uint8)
        mask = cv2.dilate(mask, kernel)
    return mask > 0


def render_shape_evidence(candidate, model_points, K, shape_meta, prefix,
                          shared_input_path=None):
    observed = decode_mask_rle(shape_meta.get("size"), shape_meta.get("rle"))
    projected = projected_point_mask(candidate, model_points, K, shape_meta)
    intersection = observed & projected
    input_img = np.zeros((*observed.shape, 3), np.uint8)
    render_img = input_img.copy()
    overlap_img = input_img.copy()
    input_img[observed] = (255, 210, 30)
    render_img[projected] = (30, 190, 255)
    overlap_img[observed] = (255, 90, 50)
    overlap_img[projected] = (30, 190, 255)
    overlap_img[intersection] = (70, 240, 120)
    paths = {}
    images = (("shape_input", input_img), ("shape_render", render_img),
              ("shape_overlap", overlap_img))
    for suffix, image in images:
        path = (Path(shared_input_path) if suffix == "shape_input" and shared_input_path
                else prefix.with_name(prefix.name + f"_{suffix}.jpg"))
        if suffix == "shape_input" and path.is_file():
            paths[suffix] = path
            continue
        checked_imwrite(path, image, [cv2.IMWRITE_JPEG_QUALITY, 92])
        paths[suffix] = path
    return paths


def load_reference_inputs(pseudo_gt_path, trajectory_path):
    """Load and validate trusted reference inputs before any report output exists."""
    if not pseudo_gt_path:
        return {}, None, None, {}, {}
    try:
        raw_gt = json.loads(Path(pseudo_gt_path).read_text(encoding="utf-8"))
        gt = validate_trusted_reference_poses(raw_gt, str(pseudo_gt_path))
    except (OSError, json.JSONDecodeError, DatasetValidationError) as exc:
        raise InputError(f"invalid pseudo-GT {pseudo_gt_path}: {exc}") from exc
    times, poses = load_trajectory(trajectory_path)
    metadata = raw_gt.get("_provenance") or {}
    objects = {
        name: value for name, value in raw_gt.items()
        if not name.startswith("_") and isinstance(value, dict)
    }
    return gt, times, poses, metadata, objects


def _build_report(args, out, validation, missing_status, dataset_id, camera_source,
                  reference_inputs):
    pointwise_spool = out / ".pointwise-spool"
    detections = load_detections_spooling_pointwise(args.detections, pointwise_spool)
    if not detections:
        raise InputError(f"no detections in {args.detections}")
    for row in detections:
        validate_detection(row, args.detections)
    frame_rows = load_jsonl(args.frames) if args.frames else []
    if frame_rows and any("stamp_ns" not in x for x in frame_rows):
        raise InputError(f"{args.frames}: every frame requires stamp_ns")
    objects = active_objects(args.objects_config)
    axes = load_axes(args.axes, objects)
    if any(row["object"] == "Rabbit" for row in detections):
        detections = [row for row in detections if row["object"] != "Rabbit"]
    report_fingerprint = detection_fingerprint(detections)
    policy = getattr(args, "verification_filter_policy", verification_filter_policy(args))
    if policy.get("decisions") is not None:
        if policy.get("report_fingerprint") != report_fingerprint:
            raise InputError("validator decisions do not match the report detections fingerprint")

    by_stamp = defaultdict(dict)
    for row in detections:
        if row["object"] not in objects:
            raise InputError(f"detection object is not in the 9-slot manifest: {row['object']}")
        if row["object"] in by_stamp[int(row["stamp_ns"])]:
            raise InputError(f"duplicate detection for stamp/object: {row['stamp_ns']} {row['object']}")
        by_stamp[int(row["stamp_ns"])][row["object"]] = row
    frame_by_stamp = {}
    for row in frame_rows:
        stamp = int(row["stamp_ns"])
        if stamp in frame_by_stamp:
            raise InputError(f"duplicate frame stamp in {args.frames}: {stamp}")
        frame_by_stamp[stamp] = row
    if frame_by_stamp:
        missing_frames = sorted(set(by_stamp) - set(frame_by_stamp))
        if missing_frames:
            raise InputError(f"{len(missing_frames)} detection stamps are absent from {args.frames}; "
                             f"first={missing_frames[0]}")
    frame_source_indices = {}
    report_decision_keys = set()
    for stamp, rows_by_object in by_stamp.items():
        source_frame = frame_by_stamp.get(stamp, {})
        source_index = source_frame.get("i", source_frame.get("frame_seq"))
        detection_indices = {row.get("i") for row in rows_by_object.values()}
        if None in detection_indices or len(detection_indices) != 1:
            raise InputError(f"detections at stamp {stamp} require one shared frame index")
        detection_index = next(iter(detection_indices))
        if source_index is None:
            source_index = detection_index
        frame_source_indices[stamp] = int(source_index)
        report_decision_keys.update(
            f"{stamp}:{int(source_index)}:{name}" for name in rows_by_object)
    policy_decision_keys = set((policy.get("decisions") or {}).keys())
    missing_policy_keys = sorted(policy_decision_keys - report_decision_keys)
    if missing_policy_keys:
        raise InputError(f"{len(missing_policy_keys)} validator decisions have no report row; "
                         f"first={missing_policy_keys[0]}")
    stamps = sorted(frame_by_stamp or by_stamp)
    if not stamps:
        raise InputError("no frames can be indexed")

    assets = out / "assets"
    frame_dir = assets / "frames"
    evidence_dir = assets / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    K, image_shape = extract_rgb(
        validation.bag, stamps, frame_dir, COLOR_TOPIC, COLOR_INFO_TOPIC)
    if K is None:
        raise InputError(f"camera intrinsics not found in bag: {validation.bag}")

    gt, times, poses, reference_meta, reference_objects = reference_inputs
    reference_stamps = {
        name: {int(item["stamp_ns"]) for item in value.get("reference_frames", [])
               if isinstance(item, dict) and "stamp_ns" in item}
        for name, value in reference_objects.items()
    }

    model_cache = {}
    records = []
    capability_counts = {"stage6000": 0, "stage300": 0, "pointwise": 0,
                         "candidate_scores": 0}
    for frame_index, stamp in enumerate(stamps):
        source_frame = frame_by_stamp.get(stamp, {})
        frame_diag = source_frame.get("diagnostics") or {}
        twc = nearest_pose(stamp, times, poses)
        slots = []
        frame_image = cv2.imread(str(frame_dir / f"{stamp}.jpg"))
        if frame_image is None:
            raise InputError(f"failed to read extracted RGB frame: {frame_dir / f'{stamp}.jpg'}")
        for name in objects:
            row = by_stamp.get(stamp, {}).get(name)
            state = object_state(row, frame_diag, name)
            slot = {"object": name, "state": state, "detection": None}
            if row is not None:
                diag = row.get("diagnostic") or {}
                verify = row.get("verify") or {}
                cands = list(verify.get("cands") or [])
                decision_key = f"{stamp}:{frame_source_indices[stamp]}:{name}"
                shadow_selected = (policy.get("decisions", {}).get(
                    decision_key, {}).get("fallbackShadow", {}).get("selected"))
                if shadow_selected is not None and not any(
                        candidate.get("proposal6000_index") ==
                        shadow_selected.get("proposal6000_index")
                        for candidate in cands):
                    cands.append({
                        "rank_geo": shadow_selected.get("rank_geo"),
                        "index300": shadow_selected.get("index300"),
                        "proposal6000_index": shadow_selected.get(
                            "proposal6000_index"),
                        "R": shadow_selected.get("R"),
                        "t_mm": shadow_selected.get("t_mm"),
                        "geo": shadow_selected.get("geometry_score"),
                        "texture_score": shadow_selected.get("texture_score"),
                        "geometry_selected": False,
                        "candidate_valid": shadow_selected.get("candidate_valid"),
                        "shape": {
                            "mask_iou": shadow_selected.get("mask_iou"),
                            "projection_valid": shadow_selected.get(
                                "projection_valid"),
                        },
                        "shadow_evidence_extra": True,
                    })
                capability_counts["candidate_scores"] += bool(cands)
                capability_counts["stage6000"] += bool(diag.get("stage6000"))
                capability_counts["stage300"] += bool(diag.get("stage300"))
                pointwise_path = row.get("_pointwise_spool")
                pointwise = (json.loads(Path(pointwise_path).read_text(encoding="utf-8"))
                             if pointwise_path else
                             diag.get("pointwise") or verify.get("pointwise") or [])
                capability_counts["pointwise"] += bool(pointwise)
                evidence_by_rank = {
                    int(x.get("rank_geo", x.get("rank_s", -1))): x for x in pointwise
                }
                shape_meta = verify.get("shape_mask") or {}
                shared_shape_input = evidence_dir / f"{stamp}_{name}_shape_input.jpg"
                enriched = []
                for candidate in sorted(cands, key=lambda x: int(x.get("rank_geo", 999999))):
                    candidate = dict(candidate)
                    # Legacy combined-score reports remain readable, but the current UI names
                    # these as independent texture fields and never treats them as selection.
                    candidate.setdefault("rank_texture", candidate.get("rank_s"))
                    candidate.setdefault("texture_score", candidate.get("s_feat"))
                    candidate.setdefault("color_score", candidate.get("s_col"))
                    candidate.setdefault("geometry_selected", candidate.get("rank_geo") == 0)
                    candidate["gt"] = assess_pose(candidate["R"], candidate["t_mm"],
                                                    name, gt.get(name), twc,
                                                    args.rotation_threshold_deg,
                                                    args.translation_threshold_mm / 1000.0,
                                                    missing_status)
                    rank = int(candidate.get("rank_geo", len(enriched)))
                    if (shadow_selected is not None and
                            rank == int(shadow_selected.get("rank_geo", -1)) and
                            candidate.get("proposal6000_index") ==
                            shadow_selected.get("proposal6000_index")):
                        candidate["shadow_evidence_extra"] = rank >= args.render_topn
                    candidate["evidence"] = {"mode": "uncollected"}
                    if rank < args.render_topn or candidate.get("shadow_evidence_extra"):
                        shape_evidence = {}
                        stem = f"{stamp}_{name}_{rank:03d}"
                        proxy = evidence_dir / f"{stem}_pose.jpg"
                        if name not in model_cache:
                            p = REPO / "assets" / "model_points" / f"{name}.npy"
                            # Match the 196-point coarse verifier.
                            model_cache[name] = np.load(p).astype(np.float32)[::8] if p.exists() else None
                        if model_cache[name] is not None:
                            projection = render_pose_proxy(
                                frame_image, row, candidate, model_cache[name], K, proxy)
                            candidate["evidence"] = {
                                "mode": ("pose_projection_proxy" if projection["status"] == "available"
                                         else "projection_unavailable"),
                                "pose": f"assets/evidence/{proxy.name}",
                                "projection_status": projection["status"],
                                "projection_reason": projection.get("reason"),
                                "note": "CAD pose projection proxy; exact score overlap was not collected",
                            }
                            if shape_meta.get("rle") is not None:
                                if (projection["status"] == "available" and
                                        candidate.get("shape", {}).get(
                                            "projection_valid") is not False):
                                    shape_paths = render_shape_evidence(
                                        candidate, model_cache[name], K, shape_meta,
                                        evidence_dir / stem, shared_shape_input)
                                    shape_evidence = {
                                        "shape_input": f"assets/evidence/{shape_paths['shape_input'].name}",
                                        "shape_render": f"assets/evidence/{shape_paths['shape_render'].name}",
                                        "shape_overlap": f"assets/evidence/{shape_paths['shape_overlap'].name}",
                                    }
                                else:
                                    observed = decode_mask_rle(
                                        shape_meta.get("size"), shape_meta.get("rle"))
                                    if not shared_shape_input.is_file():
                                        input_image = np.zeros((*observed.shape, 3), np.uint8)
                                        input_image[observed] = (255, 210, 30)
                                        checked_imwrite(
                                            shared_shape_input, input_image,
                                            [cv2.IMWRITE_JPEG_QUALITY, 92])
                                    shape_evidence = {
                                        "shape_input": f"assets/evidence/{shared_shape_input.name}",
                                        "shape_unavailable_reason": (
                                            projection.get("reason") or
                                            "candidate_projection_invalid"),
                                    }
                                candidate["evidence"].update(shape_evidence)
                        else:
                            candidate["evidence"] = {
                                "mode": "projection_unavailable",
                                "projection_status": "unavailable",
                                "projection_reason": "model_points_missing",
                            }
                        exact = evidence_by_rank.get(rank)
                        if exact:
                            prefix = evidence_dir / stem
                            render_point_heatmaps(frame_image, row, exact, prefix)
                            candidate["evidence"] = {
                                "mode": "sampled_pointwise",
                                "pose": f"assets/evidence/{proxy.name}" if proxy.exists() else None,
                                "projection_status": candidate["evidence"].get(
                                    "projection_status", "unavailable"),
                                "projection_reason": candidate["evidence"].get(
                                    "projection_reason"),
                                "geometry": f"assets/evidence/{stem}_geometry.jpg",
                                "texture": f"assets/evidence/{stem}_texture.jpg",
                                "metrics": {"geometry": exact.get("geometry"),
                                            "texture": exact.get("texture")},
                            }
                            candidate["evidence"].update(shape_evidence)
                    enriched.append(candidate)
                if shadow_selected is not None:
                    matching = [candidate for candidate in enriched
                                if candidate.get("proposal6000_index") ==
                                shadow_selected.get("proposal6000_index")]
                    if len(matching) != 1 or int(matching[0].get("rank_geo", -1)) != int(
                            shadow_selected.get("rank_geo", -2)):
                        raise InputError(
                            "shadow fallback candidate is absent from report candidates: "
                            f"{decision_key}:rank_geo={shadow_selected.get('rank_geo')}")
                axis_cfg = axes["objects"][name]
                pose_error = pose_transform_error(row["R"], row["t_mm"])
                selected_gt = ({"status": "pose_invalid", "ok": None}
                               if pose_error else assess_pose(
                                   row["R"], row["t_mm"], name, gt.get(name), twc,
                                   args.rotation_threshold_deg,
                                   args.translation_threshold_mm / 1000.0,
                                   missing_status))
                stage6000 = mark_unavailable_reason(
                    stage_value(diag, "stage6000"), missing_status)
                stage300 = mark_unavailable_reason(
                    stage_value(diag, "stage300"), missing_status)
                front_line = (None if pose_error else project_axis(
                    row["R"], row["t_mm"], axis_cfg["front_axis"],
                    axis_cfg["length_mm"], K))
                up_line = (None if pose_error else project_axis(
                    row["R"], row["t_mm"], axis_cfg["up_axis"],
                    axis_cfg["length_mm"], K))
                if pose_error is None and front_line is None:
                    pose_error = "front_axis_unprojectable"
                if pose_error is None and up_line is None:
                    pose_error = "up_axis_unprojectable"
                if pose_error:
                    selected_gt = {"status": "pose_invalid", "ok": None}
                    slot["state"] = "pem_pose_invalid"
                if selected_gt["status"] != "available":
                    stage6000 = suppress_stage_for_gt(
                        stage6000, selected_gt["status"])
                    stage300 = suppress_stage_for_gt(
                        stage300, selected_gt["status"])
                verify_summary = {k: verify.get(k) for k in (
                    "selection_method", "selected_geo_rank",
                    "selected_proposal6000_index", "texture", "shape", "shape_mask",
                    # Legacy fields are retained only so older dumps remain readable.
                    "verdict", "conf", "margin", "s_feat", "s_col", "n_modes",
                    "geo_rank_used", "geo_ratio") if k in verify}
                det = {k: row.get(k) for k in ("score", "R", "t_mm", "bbox", "ism", "pem")}
                is_reference_member = stamp in reference_stamps.get(name, set())
                reference_trusted = bool(reference_objects.get(name, {}).get("trusted"))
                det.update({"verify": verify_summary, "gt": selected_gt,
                            "stage6000": stage6000, "stage300": stage300,
                            "candidates": enriched,
                            "reference_role": (
                                "trusted_reference_member" if is_reference_member and reference_trusted
                                else "untrusted_cluster_member" if is_reference_member
                                else "evaluation_frame"),
                            "pose_status": ({"status": "invalid", "reason": pose_error}
                                            if pose_error else {"status": "available"}),
                            "front_line": front_line, "up_line": up_line})
                slot["detection"] = det
                if pointwise_path:
                    Path(pointwise_path).unlink()
            else:
                rejected = [x for x in frame_diag.get("pem_candidates", [])
                            if x.get("object") == name and x.get("input_rejection")]
                if rejected:
                    slot["rejection"] = rejected[-1]
            slots.append(slot)
        records.append({"index": frame_index, "stamp_ns": str(stamp),
                        "source_index": frame_source_indices.get(stamp),
                        "image": f"assets/frames/{stamp}.jpg", "slots": slots})

    if pointwise_spool.exists():
        shutil.rmtree(pointwise_spool)

    trusted = sorted(name for name, value in gt.items()
                     if isinstance(value, dict) and value.get("trusted") is True)
    has_trusted_gt = bool(trusted)
    has_reference = bool(args.pseudo_gt)
    capabilities = {key: count == len(detections)
                    for key, count in capability_counts.items()}
    data = {
        "schema_version": SCHEMA_VERSION,
        "title": args.title,
        "dataset": {"id": dataset_id,
                    "camera_source": camera_source,
                    "camera_role": validation.camera_role,
                    "manifest": str(validation.manifest_path or ""),
                    "bag": str(validation.bag), "detections": str(args.detections),
                    "report_fingerprint": report_fingerprint,
                    "frames": str(args.frames or ""), "frame_count": len(records),
                    "detection_count": len(detections), "partial": not all(capabilities.values())},
        "objects": objects, "axes": axes, "camera": {"K": np.asarray(K).round(8).tolist(),
                                                        "image_shape": image_shape},
        "ground_truth": {"kind": "same-camera ORB-SLAM3 pseudo-GT" if has_reference else "none",
                         "status": ("available" if has_trusted_gt else
                                    "untrusted" if has_reference else missing_status),
                         "source": str(args.pseudo_gt or ""), "trusted_objects": trusted,
                         "provenance": reference_meta,
                         "objects": {name: {
                             "trusted": bool(value.get("trusted")),
                             "unavailable_reason": value.get("unavailable_reason"),
                             "detections": value.get("detections"),
                             "cluster_size": value.get("cluster_size"),
                             "cluster_share": value.get("cluster_share"),
                             "quality_gate": value.get("quality_gate"),
                             "center": value.get("center"),
                         } for name, value in sorted(reference_objects.items())},
                         "rotation_threshold_deg": args.rotation_threshold_deg,
                         "translation_threshold_mm": args.translation_threshold_mm,
                         "notice": ("Same-camera ORB-SLAM3 pseudo-GT; exploratory self-consistency, "
                                    "not externally measured ground truth. Reference-member frames "
                                    "are labelled and are not independent accuracy trials."
                                    if has_trusted_gt else
                                    "Same-camera ORB-SLAM3 pseudo-GT was supplied, but no object "
                                    "passed its trust gates; O/X is unavailable."
                                    if has_reference else
                                    "No ground truth: SAM-to-SLAM extrinsic is unavailable "
                                    "(extrinsic_missing); O/X is unavailable."
                                    if missing_status == "extrinsic_missing" else
                                    "No ground truth was supplied; O/X is unavailable.")},
        "capabilities": capabilities,
        "capability_counts": {key: {"available": count, "total": len(detections)}
                              for key, count in capability_counts.items()},
        "verification_filter": policy,
        "frames": records,
    }
    for filename in ("index.html", "app.css", "app.js"):
        shutil.copy2(TEMPLATE_DIR / filename, out / filename)
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    (out / "report-data.js").write_text(
        "window.PEM_EXPLORER_DATA=" + payload + ";\n", encoding="utf-8")
    policy_payload = json.dumps(policy, ensure_ascii=False, separators=(",", ":"))
    (out / "verification-policy.js").write_text(
        "window.PEM_VERIFICATION_POLICY=" + policy_payload + ";\n", encoding="utf-8")
    (out / "README.md").write_text(
        f"# {args.title}\n\nOpen `index.html` directly in a browser.\n\n"
        f"- frames: {len(records)}\n- detections: {len(detections)}\n"
        f"- dataset: {data['dataset']['id']}\n"
        f"- camera source: {data['dataset']['camera_source']}\n"
        f"- partial mode: {data['dataset']['partial']}\n"
        f"- GT: {data['ground_truth']['notice']}\n\n"
        "Regenerate with:\n\n```bash\n" + shlex.join(args.command) + "\n```\n",
        encoding="utf-8")
    (out / COMPLETION_MARKER).write_text(COMPLETION_MARKER_CONTENT, encoding="utf-8")
    return data


def _validate_replace_target(target, replace_existing=False):
    """Allow replacement only for a real, completed PEM report directory."""
    target = Path(target)
    if target.is_symlink():
        raise InputError(f"refusing to replace symlink output: {target}")
    if not target.exists():
        return
    if not replace_existing:
        raise InputError(f"output already exists; refusing to overwrite completed report: {target}")
    if not target.is_dir():
        raise InputError(f"replacement target is not a directory: {target}")
    for name in COMPLETED_REPORT_MARKERS:
        marker = target / name
        if marker.is_symlink() or not marker.is_file() or marker.stat().st_size <= 0:
            raise InputError(f"replacement target is not a completed PEM report: {marker}")
    assets = target / "assets"
    if assets.is_symlink() or not assets.is_dir():
        raise InputError(f"replacement target is not a completed PEM report: {assets}")
    try:
        index = (target / "index.html").read_text(encoding="utf-8")
        with (target / "report-data.js").open("rb") as handle:
            report_prefix = handle.read(128)
        completion = (target / COMPLETION_MARKER).read_text(encoding="utf-8")
    except OSError as exc:
        raise InputError(f"cannot inspect replacement target {target}: {exc}") from exc
    if ("PEM Pose Explorer" not in index or "report-data.js" not in index or
            not report_prefix.startswith(b"window.PEM_EXPLORER_DATA=") or
            completion != COMPLETION_MARKER_CONTENT):
        raise InputError(f"replacement target lacks PEM completion markers: {target}")


def _publish_staged_report(stage, target, replace_existing=False):
    _validate_replace_target(target, replace_existing)
    if not target.exists():
        os.replace(stage, target)
        _fsync_directory(target.parent)
        return
    _exchange_directories(stage, target)
    _fsync_directory(target.parent)
    shutil.rmtree(stage)


def _fsync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _exchange_directories(first, second):
    """Atomically exchange two same-filesystem directories on Linux."""
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise InputError("atomic report replacement requires renameat2(RENAME_EXCHANGE)")
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p,
                          ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    result = renameat2(-100, os.fsencode(first), -100, os.fsencode(second), 2)
    if result != 0:
        error = ctypes.get_errno()
        raise InputError(
            f"atomic report directory exchange failed: {os.strerror(error)}")


def _validate_evidence_asset(stage, relative_path):
    if not isinstance(relative_path, str) or not relative_path.startswith("assets/"):
        raise InputError(f"invalid evidence asset link: {relative_path!r}")
    relative = Path(relative_path)
    if ".." in relative.parts:
        raise InputError(f"evidence asset escapes report directory: {relative_path}")
    path = stage / relative
    if not path.is_file() or path.stat().st_size <= 0:
        raise InputError(f"missing evidence asset: {path}")


def _validate_staged_evidence(data, stage, render_topn):
    """Fail publication when requested evidence is incomplete or links are stale."""
    full_evidence = render_topn >= 100
    for frame in data["frames"]:
        for slot in frame["slots"]:
            detection = slot.get("detection")
            if not detection:
                continue
            candidates = detection.get("candidates") or []
            rendered = [candidate for candidate in candidates
                        if int(candidate.get("rank_geo", 10 ** 9)) < render_topn]
            validated = rendered + [
                candidate for candidate in candidates
                if candidate.get("shadow_evidence_extra") and candidate not in rendered]
            if full_evidence:
                ranks = sorted(int(candidate.get("rank_geo", -1))
                               for candidate in rendered)
                if ranks != list(range(100)):
                    raise InputError(
                        f"full evidence requires exact rank_geo 0-99 at "
                        f"{frame['stamp_ns']}:{slot['object']}")
            shared_shape_inputs = set()
            for candidate in validated:
                evidence = candidate.get("evidence") or {}
                if evidence.get("mode") == "uncollected":
                    raise InputError(
                        f"candidate evidence uncollected at {frame['stamp_ns']}:"
                        f"{slot['object']}:rank_geo={candidate.get('rank_geo')}")
                for key in ("pose", "geometry", "texture", "shape_input",
                            "shape_render", "shape_overlap"):
                    if evidence.get(key):
                        _validate_evidence_asset(stage, evidence[key])
                if evidence.get("shape_input"):
                    shared_shape_inputs.add(evidence["shape_input"])
                exact_required = full_evidence and int(candidate.get("rank_geo", -1)) < 100
                shape_required = full_evidence or candidate.get("shadow_evidence_extra")
                if exact_required:
                    if not evidence.get("geometry") or not evidence.get("texture"):
                        raise InputError(
                            f"full pointwise evidence missing at {frame['stamp_ns']}:"
                            f"{slot['object']}:rank_geo={candidate.get('rank_geo')}")
                    metrics = evidence.get("metrics") or {}
                    if (not isinstance(metrics.get("geometry"), dict) or
                            not isinstance(metrics.get("texture"), dict)):
                        raise InputError("full pointwise evidence is missing exact metrics")
                if full_evidence or candidate.get("shadow_evidence_extra"):
                    if evidence.get("projection_status") == "available":
                        if not evidence.get("pose"):
                            raise InputError("available projection is missing its pose image")
                    elif not evidence.get("projection_reason"):
                        raise InputError("unavailable projection is missing an explicit reason")
                if shape_required:
                    if not evidence.get("shape_input"):
                        raise InputError("full evidence is missing the shared input mask")
                    shape_available = bool(evidence.get("shape_render") and
                                           evidence.get("shape_overlap"))
                    if not shape_available and not evidence.get(
                            "shape_unavailable_reason"):
                        raise InputError("unavailable shape evidence is missing a reason")
            if full_evidence and len(shared_shape_inputs) != 1:
                raise InputError(
                    f"candidate input mask is not shared at "
                    f"{frame['stamp_ns']}:{slot['object']}")


def _validate_full_evidence_capacity(path):
    free_bytes = shutil.disk_usage(path).free
    if free_bytes < MIN_FULL_EVIDENCE_FREE_BYTES:
        raise InputError(
            "render-topn=100 requires at least 50 GiB free before staging; "
            f"available={free_bytes / 1024 ** 3:.1f} GiB")
    filesystem = os.statvfs(path)
    if filesystem.f_files and filesystem.f_favail < MIN_FULL_EVIDENCE_FREE_INODES:
        raise InputError(
            "render-topn=100 requires at least 1,600,000 free inodes before staging; "
            f"available={filesystem.f_favail}")


def build(args):
    try:
        validation = validate_rgbd_dataset(
            args.bag, getattr(args, "dataset_manifest", ""), require_manifest=True)
        enforce_ground_truth_policy(validation, args.pseudo_gt, args.trajectory)
        (args.rotation_threshold_deg,
         args.translation_threshold_mm) = validate_evaluation_thresholds(
             args.rotation_threshold_deg, args.translation_threshold_mm)
    except DatasetValidationError as exc:
        raise InputError(f"dataset validation failed: {exc}") from exc

    requested = getattr(args, "gt_unavailable_reason", "auto") or "auto"
    manifest_reason = validation.gt_unavailable_reason
    missing_status = manifest_reason or ("gt_missing" if requested == "auto" else requested)
    if requested != "auto" and manifest_reason and requested != manifest_reason:
        raise InputError(f"GT reason {requested!r} contradicts manifest {manifest_reason!r}")
    if missing_status not in ("gt_missing", "extrinsic_missing"):
        raise InputError(f"unsupported GT unavailable reason: {missing_status}")
    if args.pseudo_gt and missing_status != "gt_missing":
        raise InputError("pseudo-GT cannot be combined with an unavailable GT reason")

    requested_dataset_id = getattr(args, "dataset_id", "") or ""
    requested_camera_source = getattr(args, "camera_source", "") or ""
    if requested_dataset_id and requested_dataset_id != validation.dataset_id:
        raise InputError(f"dataset id {requested_dataset_id!r} contradicts manifest "
                         f"{validation.dataset_id!r}")
    if requested_camera_source and requested_camera_source != validation.camera_source:
        raise InputError(f"camera source {requested_camera_source!r} contradicts manifest "
                         f"{validation.camera_source!r}")
    dataset_id = validation.dataset_id
    camera_source = validation.camera_source
    render_topn = getattr(args, "render_topn", 10)
    if isinstance(render_topn, bool) or not isinstance(render_topn, int) or render_topn < 0:
        raise InputError("render-topn must be a nonnegative integer")
    args.verification_filter_policy = resolved_verification_filter_policy(args, dataset_id)
    if args.color_topic != COLOR_TOPIC:
        raise InputError(f"color topic {args.color_topic!r} contradicts trusted manifest "
                         f"canonical topic {COLOR_TOPIC!r}")
    if args.camera_info_topic != COLOR_INFO_TOPIC:
        raise InputError(f"camera-info topic {args.camera_info_topic!r} contradicts trusted "
                         f"manifest canonical topic {COLOR_INFO_TOPIC!r}")
    reference_inputs = load_reference_inputs(args.pseudo_gt, args.trajectory)
    target = Path(args.out)
    target.parent.mkdir(parents=True, exist_ok=True)
    if render_topn >= 100:
        _validate_full_evidence_capacity(target.parent)
    _validate_replace_target(target, getattr(args, "replace_existing", False))
    stage = Path(tempfile.mkdtemp(prefix=f".{target.name}.staging-", dir=target.parent))
    try:
        data = _build_report(args, stage, validation, missing_status,
                             dataset_id, camera_source, reference_inputs)
        for required in COMPLETED_REPORT_MARKERS:
            path = stage / required
            if not path.is_file() or path.stat().st_size <= 0:
                raise InputError(f"staged report is incomplete: {path}")
        _validate_staged_evidence(data, stage, render_topn)
        _publish_staged_report(stage, target, getattr(args, "replace_existing", False))
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    print(json.dumps({"out": str(target), "frames": len(data["frames"]),
                      "detections": data["dataset"]["detection_count"],
                      "capabilities": data["capabilities"]}, ensure_ascii=False, indent=2))
    return data


def parser():
    ap = argparse.ArgumentParser(description="Build a reusable static PEM pose explorer")
    ap.add_argument("--bag", required=True, help="ROS2 bag directory")
    ap.add_argument("--detections", required=True, help="PEM detection JSONL")
    ap.add_argument("--frames", default="", help="optional frame/diagnostic JSONL")
    ap.add_argument("--pseudo-gt", default="", help="optional SLAM map pseudo-GT JSON")
    ap.add_argument("--trajectory", default="", help="optional T_wc trajectory txt")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="SAM-6D PEM Pose Explorer")
    ap.add_argument("--dataset-id", default="", help="stable dataset identifier shown in the UI")
    ap.add_argument("--camera-source", default="", help="human-readable camera/source provenance")
    ap.add_argument("--objects-config", default=str(REPO / "configs/yolo_ism_objects.yaml"))
    ap.add_argument("--axes", default=str(TEMPLATE_DIR / "default_axes.json"))
    ap.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    ap.add_argument("--dataset-manifest", default="",
                    help="optional spelling of the required in-bag dataset_manifest.json")
    ap.add_argument("--gt-unavailable-reason", default="auto",
                    choices=("auto", "gt_missing", "extrinsic_missing"),
                    help="why O/X is unavailable when no pseudo-GT is supplied")
    ap.add_argument("--replace-existing", action="store_true",
                    help="atomically replace an existing completed report after staging succeeds")
    ap.add_argument("--render-topn", type=int, default=10,
                    help="candidate pose/evidence images to render per detection")
    ap.add_argument("--rotation-threshold-deg", type=float, default=30.0)
    ap.add_argument("--translation-threshold-mm", type=float, default=100.0)
    ap.add_argument("--validator-depth-min", type=float, default=None,
                    help="enable shadow rejection only at or above this depth-valid fraction")
    ap.add_argument("--validator-texture-min", type=float, default=None,
                    help="geometry winner texture support threshold; supply all validator args")
    ap.add_argument("--validator-iou-min", type=float, default=None,
                    help="geometry winner 2D mask IoU threshold; supply all validator args")
    ap.add_argument("--validator-score-detections", default="",
                    help="scored detection JSONL used to publish exact shadow decisions")
    ap.add_argument("--validator-candidate-analysis", default="",
                    help="full-stage300 candidate analysis JSONL aligned with score detections")
    ap.add_argument("--validator-pose-detections", default="",
                    help="optional aligned legacy diagnostic JSONL when analysis lacks candidate R/t")
    return ap


def main():
    ap = parser()
    args = ap.parse_args()
    import sys
    args.command = [sys.executable, "tools/build_pem_explorer.py", *sys.argv[1:]]
    try:
        build(args)
    except (OSError, InputError, ValueError) as exc:
        ap.error(str(exc))


if __name__ == "__main__":
    main()
