#!/usr/bin/env python3
"""Evaluate SAM-6D poses in a SLAM map frame and preserve failure cases.

The dataset has no externally measured object poses.  This tool therefore builds
an explicitly labelled *pseudo ground truth*: detections are transformed from the
camera frame into the optimized ORB-SLAM3 map frame, clustered per object, and the
largest stable pose cluster is robustly averaged.  Metrics are only reported for
objects whose reference cluster passes minimum size/share gates.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


SYM_AXES = {
    "Sikhye_high": (0.0, 0.0, 1.0),
    "Sauce_high": (0.0, 0.0, 1.0),
    "Mugcup_high": (0.1057, 0.0337, 0.9938),
}


def axis_rotation(axis, degrees):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    return Rotation.from_rotvec(axis * math.radians(degrees)).as_matrix()


def symmetries(name):
    axis = SYM_AXES.get(name)
    if axis is None:
        return [np.eye(3)]
    return [axis_rotation(axis, d) for d in range(0, 360, 10)]


def angle_deg(a, b, name=None):
    values = []
    for sym in symmetries(name):
        rel = a.T @ (b @ sym)
        values.append(math.degrees(math.acos(np.clip((np.trace(rel) - 1.0) / 2.0, -1.0, 1.0))))
    return float(min(values))


def mean_rotation(mats, weights=None):
    if len(mats) == 1:
        return mats[0]
    return Rotation.from_matrix(np.asarray(mats)).mean(weights=weights).as_matrix()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def pose_matrix(rotation, translation):
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = translation
    return out


def load_trajectory(path):
    try:
        arr = np.loadtxt(path, ndmin=2)
    except (OSError, ValueError) as exc:
        raise ValueError(f"trajectory cannot be parsed: {exc}") from exc
    if arr.ndim != 2 or arr.shape[1] != 8 or not np.isfinite(arr).all():
        raise ValueError(f"trajectory must have 8 columns: {path}")
    if len(arr) < 2 or np.any(np.diff(arr[:, 0]) <= 0):
        raise ValueError(f"trajectory timestamps must be strictly increasing: {path}")
    norms = np.linalg.norm(arr[:, 4:8], axis=1)
    if np.any(norms < 1e-9) or not np.allclose(norms, 1.0, atol=1e-3, rtol=0.0):
        raise ValueError(f"trajectory quaternions must be normalized: {path}")
    mats = []
    for row in arr:
        mats.append(pose_matrix(Rotation.from_quat(row[4:8]).as_matrix(), row[1:4]))
    return arr[:, 0], np.asarray(mats)


def nearest_pose(stamp_ns, timestamps, poses, max_dt=0.025):
    stamp = stamp_ns / 1e9
    j = int(np.searchsorted(timestamps, stamp))
    candidates = [k for k in (j - 1, j) if 0 <= k < len(timestamps)]
    if not candidates:
        return None, None
    best = min(candidates, key=lambda k: abs(timestamps[k] - stamp))
    dt = abs(float(timestamps[best] - stamp))
    return (poses[best], dt) if dt <= max_dt else (None, dt)


def load_jsonl(path):
    rows = []
    with Path(path).open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def _read_json_object(path, label):
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def validate_generation_inputs(dataset_manifest, dataset_id, trajectory,
                               trajectory_provenance, reference_paths):
    """Fail closed unless trajectory and compact references attest one dataset."""
    manifest_path = Path(dataset_manifest).resolve()
    manifest = _read_json_object(manifest_path, "dataset manifest")
    if manifest.get("dataset_id") != dataset_id:
        raise ValueError("dataset manifest dataset_id mismatch")
    camera = manifest.get("camera")
    bag_info = manifest.get("bag")
    if not isinstance(camera, dict) or not camera.get("optical_frame"):
        raise ValueError("dataset manifest camera.optical_frame is required")
    if not isinstance(bag_info, dict) or not isinstance(bag_info.get("database"), str):
        raise ValueError("dataset manifest bag.database is required")

    provenance_path = Path(trajectory_provenance).resolve()
    provenance = _read_json_object(provenance_path, "trajectory provenance")
    source = provenance.get("source")
    trajectory_info = provenance.get("trajectory")
    if (provenance.get("status") != "accepted" or not isinstance(source, dict)
            or not isinstance(trajectory_info, dict)):
        raise ValueError("trajectory provenance must be accepted and structured")
    if source.get("dataset_id") != dataset_id:
        raise ValueError("trajectory provenance dataset_id mismatch")
    if source.get("optical_frame") != camera.get("optical_frame"):
        raise ValueError("trajectory provenance optical_frame mismatch")
    bag_root = manifest_path.parent
    database_name = source.get("bag_database")
    if (not isinstance(database_name, str) or Path(database_name).name != database_name
            or database_name != bag_info["database"]):
        raise ValueError("trajectory provenance bag database path mismatch")
    database = bag_root / database_name
    metadata = bag_root / "metadata.yaml"
    if (not database.is_file()
            or sha256(database) != source.get("bag_database_sha256")):
        raise ValueError("trajectory provenance bag database hash mismatch")
    if not metadata.is_file() or sha256(metadata) != source.get("metadata_sha256"):
        raise ValueError("trajectory provenance metadata hash mismatch")
    trajectory_path = Path(trajectory).resolve()
    if (trajectory_info.get("file") != trajectory_path.name
            or trajectory_info.get("sha256") != sha256(trajectory_path)
            or trajectory_info.get("format") !=
            "TUM T_wc: timestamp tx ty tz qx qy qz qw"):
        raise ValueError("trajectory file/hash/convention does not match provenance")

    reference_provenance = []
    for reference in reference_paths:
        reference_path = Path(reference).resolve()
        sidecar_path = reference_path.with_suffix(
            reference_path.suffix + ".provenance.json")
        sidecar = _read_json_object(sidecar_path, "reference provenance")
        output = sidecar.get("output")
        source_info = sidecar.get("source")
        snapshot = sidecar.get("dataset_snapshot")
        diagnostic_attestation = sidecar.get("diagnostic_provenance")
        if (sidecar.get("kind") != "compact PEM pose reference"
                or sidecar.get("dataset_id") != dataset_id
                or not isinstance(output, dict) or not isinstance(source_info, dict)
                or not isinstance(snapshot, dict)
                or not isinstance(diagnostic_attestation, dict)):
            raise ValueError("reference provenance kind/dataset/structure mismatch")
        if (snapshot.get("bag_database") != database_name
                or snapshot.get("bag_database_sha256") != sha256(database)
                or snapshot.get("metadata_sha256") != sha256(metadata)):
            raise ValueError("reference dataset snapshot mismatch")
        if (Path(str(output.get("file", ""))).resolve() != reference_path
                or output.get("sha256") != sha256(reference_path)
                or output.get("fields") != [
                    "stamp_ns", "i", "object", "score", "R", "t_mm", "bbox"]):
            raise ValueError("reference output path/hash/schema mismatch")
        source_path = Path(str(source_info.get("file", ""))).resolve()
        if not source_path.is_file() or sha256(source_path) != source_info.get("sha256"):
            raise ValueError("reference source path/hash mismatch")
        diagnostic_path = Path(str(diagnostic_attestation.get("file", ""))).resolve()
        diagnostic_content = diagnostic_attestation.get("content")
        if (not diagnostic_path.is_file()
                or sha256(diagnostic_path) != diagnostic_attestation.get("sha256")
                or _read_json_object(diagnostic_path, "diagnostic provenance") !=
                diagnostic_content or not isinstance(diagnostic_content, dict)):
            raise ValueError("diagnostic provenance file/hash/content mismatch")
        diagnostic_outputs = diagnostic_content.get("outputs")
        diagnostic_detections = (diagnostic_outputs.get("detections")
                                 if isinstance(diagnostic_outputs, dict) else None)
        if (diagnostic_content.get("kind") != "SAM-6D diagnostic run"
                or diagnostic_content.get("dataset_id") != dataset_id
                or diagnostic_content.get("optical_frame") != camera.get("optical_frame")
                or diagnostic_content.get("dataset_snapshot") != snapshot
                or not isinstance(diagnostic_detections, dict)
                or Path(str(diagnostic_detections.get("file", ""))).resolve() != source_path
                or diagnostic_detections.get("sha256") != source_info.get("sha256")):
            raise ValueError("diagnostic provenance does not bind this reference source")
        reference_provenance.append({
            "file": str(sidecar_path), "sha256": sha256(sidecar_path),
            "content": sidecar,
        })
    return manifest, provenance, reference_provenance


def validate_reference_rows(rows):
    seen = set()
    for number, row in enumerate(rows, 1):
        try:
            key = (str(row["object"]), int(row["stamp_ns"]))
            rotation = np.asarray(row["R"], dtype=float)
            translation = np.asarray(row["t_mm"], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid reference row {number}: {exc}") from exc
        if key in seen:
            raise ValueError(f"duplicate reference object/stamp: {key}")
        seen.add(key)
        if (rotation.shape != (3, 3) or translation.shape != (3,)
                or not np.isfinite(rotation).all() or not np.isfinite(translation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3, rtol=0.0)
                or not math.isclose(float(np.linalg.det(rotation)), 1.0,
                                    abs_tol=1e-3, rel_tol=0.0)):
            raise ValueError(f"reference row {number} has an invalid rigid pose")


def enrich(rows, timestamps, trajectory):
    result = []
    for row in rows:
        twc, dt = nearest_pose(int(row["stamp_ns"]), timestamps, trajectory)
        if twc is None:
            continue
        tco = pose_matrix(np.asarray(row["R"], dtype=float),
                          np.asarray(row["t_mm"], dtype=float) / 1000.0)
        two = twc @ tco
        result.append({**row, "map_R": two[:3, :3], "map_t": two[:3, 3],
                       "slam_dt_ms": round(dt * 1000.0, 3)})
    return result


def _annotate_one_crop(row, image):
    enriched = dict(row)
    reason = None
    try:
        bbox = np.asarray(row.get("bbox", []), dtype=float)
    except (TypeError, ValueError):
        bbox = np.asarray([], dtype=float)
    if (bbox.shape != (4,) or not np.isfinite(bbox).all()
            or bbox[2] <= bbox[0] or bbox[3] <= bbox[1]):
        reason = "bbox_missing_or_invalid"
    elif image is None:
        reason = "frame_image_missing"
    else:
        height, width = image.shape[:2]
        x1, y1, x2, y2 = [int(round(v)) for v in bbox]
        x1, x2 = max(0, min(width, x1)), max(0, min(width, x2))
        y1, y2 = max(0, min(height, y1)), max(0, min(height, y2))
        crop = image[y1:y2, x1:x2]
        if crop.size == 0 or x2 <= x1 or y2 <= y1:
            reason = "bbox_empty_after_clip"
        else:
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            enriched["bbox_area_px"] = float((x2 - x1) * (y2 - y1))
            enriched["crop_sharpness"] = float(cv2.Laplacian(
                gray, cv2.CV_64F).var())
    enriched["quality_rejection"] = reason
    return enriched


def annotate_crop_quality(records, image_dir):
    """Legacy helper retained for unit tests and older callers."""
    image_dir = Path(image_dir)
    cache = {}
    result = []
    for row in records:
        stamp = int(row["stamp_ns"])
        if stamp not in cache:
            cache[stamp] = cv2.imread(str(image_dir / f"{stamp}.jpg"))
        result.append(_annotate_one_crop(row, cache[stamp]))
    return result


def annotate_crop_quality_from_bag(records, bag):
    """Measure quality from the attested bag rather than a mutable JPEG cache."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    result = [dict(row) for row in records]
    by_stamp = defaultdict(list)
    for index, row in enumerate(result):
        by_stamp[int(row["stamp_ns"])].append(index)
    seen = set()
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(bag)], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections
                       if connection.topic == "/camera/camera/color/image_raw"]
        for connection, _bag_stamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            stamp = (int(message.header.stamp.sec) * 1_000_000_000
                     + int(message.header.stamp.nanosec))
            if stamp not in by_stamp:
                continue
            if stamp in seen:
                raise ValueError(f"duplicate RGB header stamp in bag: {stamp}")
            seen.add(stamp)
            image = np.frombuffer(message.data, np.uint8).reshape(
                message.height, message.width, -1)
            if message.encoding == "rgb8":
                image = image[:, :, ::-1]
            for index in by_stamp[stamp]:
                result[index] = _annotate_one_crop(result[index], image)
    for stamp, indices in by_stamp.items():
        if stamp not in seen:
            for index in indices:
                result[index] = _annotate_one_crop(result[index], None)
    return result


def adaptive_quality_gate(records, min_samples=10, quantiles=None):
    """Select jointly large and sharp crops, relaxing only as needed per object."""
    quantiles = (np.linspace(0.70, 0.0, 15) if quantiles is None
                 else np.asarray(quantiles, dtype=float))
    if (quantiles.ndim != 1 or len(quantiles) == 0 or not np.isfinite(quantiles).all()
            or np.any((quantiles < 0.0) | (quantiles > 1.0))):
        raise ValueError("quality quantiles must be a nonempty finite 1-D array in [0, 1]")
    valid = [row for row in records
             if row.get("quality_rejection") is None
             and np.isfinite(float(row.get("bbox_area_px", np.nan)))
             and np.isfinite(float(row.get("crop_sharpness", np.nan)))]
    invalid_reasons = defaultdict(int)
    for row in records:
        if row.get("quality_rejection"):
            invalid_reasons[row["quality_rejection"]] += 1
    summary = {
        "input_count": len(records), "valid_quality_count": len(valid),
        "minimum_requested": int(min_samples),
        "invalid_reasons": dict(sorted(invalid_reasons.items())),
        "policy": "joint bbox-area and crop-sharpness quantile; q70 down by q05",
    }
    if not valid:
        return [], {**summary, "status": "insufficient_quality", "selected_count": 0}
    areas = np.asarray([row["bbox_area_px"] for row in valid], dtype=float)
    sharpness = np.asarray([row["crop_sharpness"] for row in valid], dtype=float)
    target = min(int(min_samples), len(valid))
    selected, selected_q, area_gate, sharp_gate = [], None, None, None
    for q in quantiles:
        area_gate = float(np.quantile(areas, q))
        sharp_gate = float(np.quantile(sharpness, q))
        selected = [row for row in valid
                    if row["bbox_area_px"] >= area_gate
                    and row["crop_sharpness"] >= sharp_gate]
        if len(selected) >= target:
            selected_q = float(q)
            break
    status = "selected" if len(valid) >= min_samples and len(selected) >= min_samples else "insufficient_quality"
    return selected, {
        **summary, "status": status,
        "selected_quantile": round(selected_q, 2) if selected_q is not None else None,
        "bbox_area_threshold_px": float(area_gate) if area_gate is not None else None,
        "crop_sharpness_threshold": float(sharp_gate) if sharp_gate is not None else None,
        "selected_count": len(selected),
    }


def dbscan(records, trans_gate=0.08, rot_gate=30.0, min_samples=3):
    translations = np.asarray([r["map_t"] for r in records])
    rotations = np.asarray([r["map_R"] for r in records])
    trans_distance = np.linalg.norm(translations[:, None, :] - translations[None, :, :], axis=2)
    # trace(Ra.T @ Rb) is the element-wise dot product.  Max over declared
    # object symmetries before converting to an angle.
    trace_max = np.full((len(records), len(records)), -np.inf)
    for sym in symmetries(records[0]["object"]):
        equivalent = rotations @ sym
        trace_max = np.maximum(trace_max, np.einsum("aij,bij->ab", rotations, equivalent))
    cosine = np.clip((trace_max - 1.0) / 2.0, -1.0, 1.0)
    rot_distance = np.degrees(np.arccos(cosine))
    adjacent = (trans_distance <= trans_gate) & (rot_distance <= rot_gate)

    labels = np.full(len(records), -99, dtype=int)  # -99 unseen, -1 noise
    cluster_id = 0
    for i in range(len(records)):
        if labels[i] != -99:
            continue
        seed = np.flatnonzero(adjacent[i]).tolist()
        if len(seed) < min_samples:
            labels[i] = -1
            continue
        labels[i] = cluster_id
        queue = deque(seed)
        queued = set(seed)
        while queue:
            j = queue.popleft()
            if labels[j] == -1:
                labels[j] = cluster_id
            if labels[j] != -99:
                continue
            labels[j] = cluster_id
            more = np.flatnonzero(adjacent[j]).tolist()
            if len(more) >= min_samples:
                for k in more:
                    if k not in queued:
                        queued.add(k)
                        queue.append(k)
        cluster_id += 1
    return labels


def _pairwise_pose_distance(records):
    translations = np.asarray([row["map_t"] for row in records], dtype=float)
    rotations = np.asarray([row["map_R"] for row in records], dtype=float)
    trans_distance = np.linalg.norm(
        translations[:, None, :] - translations[None, :, :], axis=2)
    trace_max = np.full((len(records), len(records)), -np.inf)
    for sym in symmetries(records[0]["object"]):
        equivalent = rotations @ sym
        trace_max = np.maximum(
            trace_max, np.einsum("aij,bij->ab", rotations, equivalent))
    rot_distance = np.degrees(np.arccos(np.clip((trace_max - 1.0) / 2.0, -1.0, 1.0)))
    return trans_distance, rot_distance


def _align_to(reference, rotation, name):
    choices = [rotation @ sym for sym in symmetries(name)]
    return min(choices, key=lambda value: angle_deg(reference, value))


def _direct_neighborhood(records, trans_gate, rot_gate):
    trans_distance, rot_distance = _pairwise_pose_distance(records)
    adjacent = (trans_distance <= trans_gate) & (rot_distance <= rot_gate)
    sizes = adjacent.sum(axis=1)
    largest = int(sizes.max())
    candidates = np.flatnonzero(sizes == largest)

    def tie_key(index):
        members = adjacent[index]
        normalized = (trans_distance[index, members] / max(trans_gate, 1e-12)
                      + rot_distance[index, members] / max(rot_gate, 1e-12))
        return (float(np.median(normalized)), int(records[index]["stamp_ns"]), int(index))

    center = int(min(candidates, key=tie_key))
    return center, np.flatnonzero(adjacent[center]), {
        "maximum_direct_neighbor_count": largest,
        "tied_center_count": int(len(candidates)),
        "tie_break": "lowest median normalized direct-neighbor distance, then earliest stamp",
    }


def build_pseudo_gt(records, min_cluster=5, min_share=0.50, trans_gate=0.08,
                    rot_gate=30.0, quality_filter=False, min_quality_samples=10):
    by_object = defaultdict(list)
    for row in records:
        by_object[row["object"]].append(row)
    ground_truth = {}
    assignments = {}
    for name, group in sorted(by_object.items()):
        quality_group, quality = (adaptive_quality_gate(group, min_quality_samples)
                                  if quality_filter else
                                  (group, {"status": "not_applied", "input_count": len(group),
                                           "valid_quality_count": len(group),
                                           "selected_count": len(group)}))
        if not quality_group:
            ground_truth[name] = {
                "trusted": False, "unavailable_reason": "insufficient_quality",
                "detections": len(group), "quality_gate": quality,
                "cluster_size": 0, "cluster_share": 0.0,
                "reference_frames": [], "representative_frames": [],
            }
            assignments[name] = {
                int(row["stamp_ns"]): {
                    "status": "quality_rejected",
                    "reasons": [row.get("quality_rejection") or "insufficient_quality"],
                    "bbox_area_px": row.get("bbox_area_px"),
                    "crop_sharpness": row.get("crop_sharpness"),
                } for row in group
            }
            continue
        center, idx, neighborhood = _direct_neighborhood(
            quality_group, trans_gate, rot_gate)
        center_r = quality_group[center]["map_R"]
        aligned = [_align_to(center_r, quality_group[i]["map_R"], name) for i in idx]
        ref_r = mean_rotation(aligned)
        ref_t = np.median(np.asarray([quality_group[i]["map_t"] for i in idx]), axis=0)
        # Remove-only fixed point: every serialized member stays a direct center
        # neighbor and also satisfies the final robust-reference gates.
        while len(idx):
            new_idx = np.asarray([
                i for i in idx
                if angle_deg(ref_r, quality_group[i]["map_R"], name) <= rot_gate
                and np.linalg.norm(ref_t - quality_group[i]["map_t"]) <= trans_gate
            ], dtype=int)
            if np.array_equal(new_idx, idx):
                break
            idx = new_idx
            if len(idx):
                aligned = [_align_to(ref_r, quality_group[i]["map_R"], name) for i in idx]
                ref_r = mean_rotation(aligned)
                ref_t = np.median(
                    np.asarray([quality_group[i]["map_t"] for i in idx]), axis=0)
        share = len(idx) / len(quality_group)
        member_error = [(angle_deg(ref_r, quality_group[i]["map_R"], name),
                         float(np.linalg.norm(ref_t - quality_group[i]["map_t"])), i) for i in idx]
        representatives = sorted(member_error, key=lambda x: (x[0] + 300.0 * x[1],
                                                               int(quality_group[x[2]]["stamp_ns"])))[:5]
        quality_ok = quality.get("status") != "insufficient_quality"
        trusted = quality_ok and len(idx) >= min_cluster and share >= min_share
        unavailable_reason = (None if trusted else
                              "insufficient_quality" if not quality_ok else
                              "cluster_too_small" if len(idx) < min_cluster else
                              "cluster_share_too_low")
        ground_truth[name] = {
            "R": ref_r.tolist(),
            "t_m": ref_t.tolist(),
            "detections": len(group),
            "quality_gate": quality,
            "cluster_size": int(len(idx)),
            "cluster_share": round(float(share), 4),
            "trusted": trusted,
            "unavailable_reason": unavailable_reason,
            "rotation_gate_deg": float(rot_gate),
            "translation_gate_m": float(trans_gate),
            "center": {
                "stamp_ns": int(quality_group[center]["stamp_ns"]),
                "frame_index": int(quality_group[center].get("i", -1)),
                **neighborhood,
            },
            "reference_frames": [{
                "stamp_ns": int(quality_group[i]["stamp_ns"]),
                "frame_index": int(quality_group[i].get("i", -1)),
                "bbox_area_px": round(float(quality_group[i].get("bbox_area_px", 0.0)), 3),
                "crop_sharpness": round(float(quality_group[i].get("crop_sharpness", 0.0)), 3),
                "rotation_residual_deg": round(angle_deg(
                    ref_r, quality_group[i]["map_R"], name), 3),
                "translation_residual_m": round(float(np.linalg.norm(
                    ref_t - quality_group[i]["map_t"])), 4),
            } for i in idx],
            "representative_frames": [{
                "stamp_ns": int(quality_group[i]["stamp_ns"]),
                "frame_index": int(quality_group[i].get("i", -1)),
                "score": float(quality_group[i].get("score", 0.0)),
                "rotation_residual_deg": round(rot, 3),
                "translation_residual_m": round(trans, 4),
            } for rot, trans, i in representatives],
        }
        member_stamps = {int(quality_group[i]["stamp_ns"]) for i in idx}
        quality_stamps = {int(row["stamp_ns"]) for row in quality_group}
        assignments[name] = {}
        area_gate = quality.get("bbox_area_threshold_px")
        sharp_gate = quality.get("crop_sharpness_threshold")
        for row in group:
            stamp = int(row["stamp_ns"])
            reasons = []
            if row.get("quality_rejection"):
                reasons.append(row["quality_rejection"])
            elif stamp not in quality_stamps:
                if area_gate is not None and row.get("bbox_area_px", -np.inf) < area_gate:
                    reasons.append("bbox_area_below_threshold")
                if sharp_gate is not None and row.get("crop_sharpness", -np.inf) < sharp_gate:
                    reasons.append("crop_sharpness_below_threshold")
            status = ("reference_member" if stamp in member_stamps else
                      "quality_selected" if stamp in quality_stamps else
                      "quality_rejected")
            if status == "quality_rejected" and not reasons:
                reasons.append("joint_quality_threshold_failed")
            assignments[name][stamp] = {
                "status": status,
                "reasons": reasons,
                "bbox_area_px": row.get("bbox_area_px"),
                "crop_sharpness": row.get("crop_sharpness"),
                "bbox_area_threshold_px": area_gate,
                "crop_sharpness_threshold": sharp_gate,
            }
    return ground_truth, assignments


def evaluate_static_pose_memory(records, ground_truth, train_fraction=0.50):
    """Evaluate a causal SLAM-map pose memory for objects fixed in the scene.

    Anchors are learned only from the chronological first half.  The second half
    is then answered by projecting the stored map pose, so this metric must be
    reported separately from standalone per-frame PEM accuracy.
    """
    # Reference inputs may contain the same detection from multiple controlled
    # methods.  Keep one object/stamp observation to avoid accidental weighting.
    unique = {}
    for row in records:
        unique[(row["object"], int(row["stamp_ns"]))] = row
    rows = sorted(unique.values(), key=lambda row: int(row["stamp_ns"]))
    stamps = sorted({int(row["stamp_ns"]) for row in rows})
    if len(stamps) < 2:
        return {"train_fraction": train_fraction, "anchors": {}, "overall": {"n": 0}}
    cutoff = stamps[min(int(len(stamps) * train_fraction), len(stamps) - 1)]
    train = [row for row in rows if int(row["stamp_ns"]) < cutoff]
    test = [row for row in rows if int(row["stamp_ns"]) >= cutoff]
    anchors, _ = build_pseudo_gt(train)

    outcomes, per_object, anchor_checks = [], defaultdict(list), {}
    for row in test:
        anchor = anchors.get(row["object"])
        gt = ground_truth.get(row["object"])
        if not anchor or not anchor["trusted"] or not gt or not gt["trusted"]:
            continue
        rot = angle_deg(np.asarray(gt["R"]), np.asarray(anchor["R"]), row["object"])
        trans = float(np.linalg.norm(np.asarray(gt["t_m"]) - np.asarray(anchor["t_m"])))
        value = {"rotation_error_deg": rot, "translation_error_m": trans}
        outcomes.append(value)
        per_object[row["object"]].append(value)
        anchor_checks[row["object"]] = value

    def summary(values):
        if not values:
            return {"n": 0, "rotation_le_30_pct": 0.0, "pose_success_pct": 0.0}
        rot = np.asarray([value["rotation_error_deg"] for value in values])
        trans = np.asarray([value["translation_error_m"] for value in values])
        return {
            "n": len(values),
            "rotation_le_30_pct": round(float(100 * np.mean(rot <= 30.0)), 2),
            "pose_success_pct": round(float(100 * np.mean((rot <= 30.0) & (trans <= .10))), 2),
            "gross_gt_45_pct": round(float(100 * np.mean(rot > 45.0)), 2),
            "rotation_median_deg": round(float(np.median(rot)), 2),
            "translation_median_m": round(float(np.median(trans)), 4),
        }
    return {
        "description": "anchors learned from chronological first half; evaluated on second half",
        "train_fraction": train_fraction,
        "cutoff_stamp_ns": cutoff,
        "train_detections": len(train),
        "test_detections": len(test),
        "covered_test_detections": len(outcomes),
        "anchors": anchors,
        "overall": summary(outcomes),
        "anchor_agreement": summary(list(anchor_checks.values())),
        "per_object": {name: summary(values) for name, values in sorted(per_object.items())},
    }


def evaluate_causal_pose_memory(records, ground_truth, min_cluster=10):
    """Learn each static object anchor online, then score only later frames.

    An anchor is frozen when the observations seen so far contain a stable
    cluster of at least ``min_cluster`` poses occupying at least half of that
    object's history.  No future pose is used to initialize an anchor.
    """
    unique = {}
    for row in records:
        unique[(row["object"], int(row["stamp_ns"]))] = row
    rows = sorted(unique.values(), key=lambda row: int(row["stamp_ns"]))
    pending, anchors, initialized = defaultdict(list), {}, {}
    outcomes, per_object, anchor_checks = [], defaultdict(list), {}
    eligible_seen = 0
    for row in rows:
        name = row["object"]
        gt = ground_truth.get(name)
        if not gt or not gt["trusted"]:
            continue
        eligible_seen += 1
        if name not in anchors:
            pending[name].append(row)
            if len(pending[name]) >= min_cluster:
                candidate, _ = build_pseudo_gt(
                    pending[name], min_cluster=min_cluster, min_share=0.50)
                if candidate.get(name, {}).get("trusted"):
                    anchors[name] = candidate[name]
                    initialized[name] = {
                        "stamp_ns": int(row["stamp_ns"]),
                        "observations_seen": len(pending[name]),
                        "cluster_size": candidate[name]["cluster_size"],
                        "cluster_share": candidate[name]["cluster_share"],
                    }
            continue
        anchor = anchors[name]
        rot = angle_deg(np.asarray(gt["R"]), np.asarray(anchor["R"]), name)
        trans = float(np.linalg.norm(np.asarray(gt["t_m"]) - np.asarray(anchor["t_m"])))
        value = {"rotation_error_deg": rot, "translation_error_m": trans}
        outcomes.append(value)
        per_object[name].append(value)
        anchor_checks[name] = value

    def summary(values):
        if not values:
            return {"n": 0, "rotation_le_30_pct": 0.0, "pose_success_pct": 0.0}
        rot = np.asarray([value["rotation_error_deg"] for value in values])
        trans = np.asarray([value["translation_error_m"] for value in values])
        return {
            "n": len(values),
            "rotation_le_30_pct": round(float(100 * np.mean(rot <= 30.0)), 2),
            "pose_success_pct": round(float(100 * np.mean((rot <= 30.0) & (trans <= .10))), 2),
            "gross_gt_45_pct": round(float(100 * np.mean(rot > 45.0)), 2),
            "rotation_median_deg": round(float(np.median(rot)), 2),
            "translation_median_m": round(float(np.median(trans)), 4),
        }
    return {
        "description": "causal per-object initialization; frozen anchor used only on later frames",
        "minimum_stable_cluster": min_cluster,
        "eligible_detections_seen": eligible_seen,
        "post_initialization_outputs": len(outcomes),
        "initialized": initialized,
        "anchors": anchors,
        "overall": summary(outcomes),
        "anchor_agreement": summary(list(anchor_checks.values())),
        "per_object": {name: summary(values) for name, values in sorted(per_object.items())},
    }


def evaluate(name, records, ground_truth, reference_members=None,
             rot_ok=30.0, trans_ok=0.10):
    reference_members = reference_members or set()
    cases, per_object = [], defaultdict(list)
    for row in records:
        gt = ground_truth.get(row["object"])
        if not gt or not gt["trusted"]:
            continue
        rot = angle_deg(np.asarray(gt["R"]), row["map_R"], row["object"])
        trans = float(np.linalg.norm(np.asarray(gt["t_m"]) - row["map_t"]))
        case = {
            "method": name,
            "stamp_ns": int(row["stamp_ns"]),
            "frame_index": int(row.get("i", -1)),
            "object": row["object"],
            "score": float(row.get("score", 0.0)),
            "rotation_error_deg": round(rot, 3),
            "translation_error_m": round(trans, 4),
            "rotation_ok": rot <= rot_ok,
            "pose_ok": rot <= rot_ok and trans <= trans_ok,
            "category": ("wrong_rotation" if rot > rot_ok else
                         "wrong_translation" if trans > trans_ok else "correct"),
            "reference_role": ("reference_member"
                               if (row["object"], int(row["stamp_ns"])) in reference_members
                               else "evaluation_frame"),
            **({"bbox": row["bbox"]} if "bbox" in row else {}),
            **({"ism": row["ism"]} if "ism" in row else {}),
            **({"pem": row["pem"]} if "pem" in row else {}),
            **({"verify": {k: v for k, v in row["verify"].items() if k != "cands"}}
               if "verify" in row else {}),
        }
        cases.append(case)
        per_object[row["object"]].append(case)

    def summarize(group):
        if not group:
            return {"n": 0}
        r = np.asarray([v["rotation_error_deg"] for v in group])
        t = np.asarray([v["translation_error_m"] for v in group])
        return {
            "n": len(group),
            "rotation_median_deg": round(float(np.median(r)), 2),
            "rotation_le_15_pct": round(float(100 * np.mean(r <= 15.0)), 2),
            "rotation_le_30_pct": round(float(100 * np.mean(r <= rot_ok)), 2),
            "rotation_gt_45_pct": round(float(100 * np.mean(r > 45.0)), 2),
            "translation_median_m": round(float(np.median(t)), 4),
            "pose_success_pct": round(float(100 * np.mean((r <= rot_ok) & (t <= trans_ok))), 2),
        }

    evaluation_cases = [case for case in cases
                        if case["reference_role"] == "evaluation_frame"]
    reference_cases = [case for case in cases
                       if case["reference_role"] == "reference_member"]
    return {"overall": summarize(cases),
            "evaluation_only": summarize(evaluation_cases),
            "reference_members": summarize(reference_cases),
            "per_object": {obj: summarize(vals) for obj, vals in sorted(per_object.items())}}, cases


def candidate_sweep(rows, timestamps, trajectory, ground_truth, reference_members=None):
    reference_members = reference_members or set()
    samples = []
    for row in rows:
        if (row["object"], int(row["stamp_ns"])) in reference_members:
            continue
        gt = ground_truth.get(row["object"])
        candidates = (row.get("verify") or {}).get("cands") or []
        if not gt or not gt["trusted"] or not candidates:
            continue
        twc, _ = nearest_pose(int(row["stamp_ns"]), timestamps, trajectory)
        if twc is None:
            continue
        sample = {"object": row["object"], "i": int(row.get("i", -1)), "candidates": []}
        for cand in candidates:
            tco = pose_matrix(np.asarray(cand["R"], float), np.asarray(cand["t_mm"], float) / 1000.0)
            two = twc @ tco
            sample["candidates"].append({
                **cand,
                "rot_error": angle_deg(np.asarray(gt["R"]), two[:3, :3], row["object"]),
                "trans_error": float(np.linalg.norm(np.asarray(gt["t_m"]) - two[:3, 3])),
            })
        samples.append(sample)

    def select(sample, w_geo, w_col, guard):
        cands = sample["candidates"]
        geo = np.asarray([c["geo"] for c in cands], float)
        feat = np.asarray([c["s_feat"] for c in cands], float)
        col = np.asarray([0.0 if c.get("s_col") is None else c["s_col"] for c in cands], float)
        z = lambda x: (x - x.mean()) / (x.std() + 1e-9)
        score = z(feat) + w_geo * z(geo) + w_col * z(col)
        eligible = geo >= guard * geo.max()
        eligible[int(np.argmax(geo))] = True
        return cands[int(np.argmax(np.where(eligible, score, -np.inf)))]

    def metric(subset, params):
        chosen = [select(s, *params) for s in subset]
        return chosen_metric(chosen)

    def chosen_metric(chosen):
        if not chosen:
            return {"n": 0, "rotation_le_30_pct": 0.0, "pose_success_pct": 0.0}
        rot = np.asarray([c["rot_error"] for c in chosen])
        trans = np.asarray([c["trans_error"] for c in chosen])
        return {"n": len(chosen),
                "rotation_le_30_pct": round(float(100 * np.mean(rot <= 30.0)), 2),
                "pose_success_pct": round(float(100 * np.mean((rot <= 30.0) & (trans <= .10))), 2),
                "gross_gt_45_pct": round(float(100 * np.mean(rot > 45.0)), 2),
                "rotation_median_deg": round(float(np.median(rot)), 2)}

    def availability_oracle(subset):
        if not subset:
            return {"n": 0, "rotation_candidate_available_pct": 0.0,
                    "pose_candidate_available_pct": 0.0}
        rotation_available = [any(c["rot_error"] <= 30.0 for c in s["candidates"])
                              for s in subset]
        pose_available = [any(c["rot_error"] <= 30.0 and c["trans_error"] <= .10
                              for c in s["candidates"]) for s in subset]
        return {
            "n": len(subset),
            "rotation_candidate_available_pct": round(100 * float(np.mean(rotation_available)), 2),
            "pose_candidate_available_pct": round(100 * float(np.mean(pose_available)), 2),
        }

    train = [s for s in samples if s["i"] % 10 < 5]
    test = [s for s in samples if s["i"] % 10 >= 5]
    params_grid = []
    for w_geo in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0):
        for w_col in (0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0):
            for guard in (0.0, 0.80, 0.85, 0.90, 0.95):
                params_grid.append((w_geo, w_col, guard))
    grid = []
    for params in params_grid:
        m = metric(train, params)
        grid.append((m["rotation_le_30_pct"], m["pose_success_pct"], -m["gross_gt_45_pct"], params))
    grid.sort(reverse=True)
    best = grid[0][3] if grid else (2.0, 1.0, 0.9)
    current = (2.0, 1.0, 0.9)
    geometry = (1e6, 0.0, 0.0)
    # Per-object weights expose whether one global texture coefficient is the
    # bottleneck.  Parameters are selected on the tune split only.
    object_params, object_test_chosen = {}, []
    for obj in sorted({s["object"] for s in samples}):
        obj_train = [s for s in train if s["object"] == obj]
        obj_test = [s for s in test if s["object"] == obj]
        ranked = []
        for params in params_grid:
            m = metric(obj_train, params)
            ranked.append((m["rotation_le_30_pct"], m["pose_success_pct"],
                           -m.get("gross_gt_45_pct", 100.0), params))
        ranked.sort(reverse=True)
        chosen_params = ranked[0][3]
        object_params[obj] = {"w_geo": chosen_params[0], "w_col": chosen_params[1],
                              "geo_guard": chosen_params[2],
                              "train": metric(obj_train, chosen_params),
                              "test": metric(obj_test, chosen_params)}
        object_test_chosen.extend(select(s, *chosen_params) for s in obj_test)

    # Diagnostic hindsight selection by a continuous composite error.  This is
    # not a binary success upper bound; candidate_availability_oracle below is.
    anchor_chosen = []
    for sample in test:
        anchor_chosen.append(min(sample["candidates"],
                                 key=lambda c: c["rot_error"] / 30.0 + c["trans_error"] / .10))
    return {
        "samples": len(samples),
        "split": ("exploratory frame_index mod 10: 0-4 tune, 5-9 test; "
                  "temporally interleaved, not an independent sequence"),
        "geometry_only": {"params": geometry, "train": metric(train, geometry),
                          "test": metric(test, geometry)},
        "current": {"params": current, "train": metric(train, current),
                    "test": metric(test, current)},
        "best_train_params": {"w_geo": best[0], "w_col": best[1], "geo_guard": best[2]},
        "best": {"train": metric(train, best), "test": metric(test, best)},
        "object_specific": {"params": object_params,
                            "combined_test": chosen_metric(object_test_chosen)},
        "composite_hindsight_selector": chosen_metric(anchor_chosen),
        "candidate_availability_oracle": {
            "train": availability_oracle(train), "test": availability_oracle(test)},
        "top_five": [{"w_geo": p[0], "w_col": p[1], "geo_guard": p[2],
                      "train": metric(train, p), "test": metric(test, p)}
                     for _, _, _, p in grid[:5]],
    }


def markdown_report(gt, metrics, sweep, pose_memory, causal_memory):
    lines = ["# longcircle2 SAM-6D pose evaluation", "",
             "The reference is a SLAM-map-frame pseudo ground truth, not a measured external GT. "
             "The evaluated feature/texture predictions also contribute to this reference, so the "
             "numbers are exploratory self-consistency measurements, not independent accuracy.", "",
             "## Reference quality", "",
             "| object | detections | stable cluster | share | trusted |",
             "|---|---:|---:|---:|:---:|"]
    for obj, row in sorted(gt.items()):
        lines.append(f"| {obj} | {row['detections']} | {row['cluster_size']} | "
                     f"{100*row['cluster_share']:.1f}% | {'yes' if row['trusted'] else 'no'} |")
    for name, result in metrics.items():
        o = result["overall"]
        e = result.get("evaluation_only", {"n": 0})
        r = result.get("reference_members", {"n": 0})
        lines += ["", f"## {name}", "",
                  f"Full self-consistency (not independent): {o.get('n', 0)} poses; rotation ≤30°: "
                  f"{o.get('rotation_le_30_pct', 0):.2f}%; pose success (≤30°, ≤10 cm): "
                  f"{o.get('pose_success_pct', 0):.2f}%.",
                  f"Evaluation-only (reference members excluded): {e.get('n', 0)} poses; "
                  f"rotation ≤30°: {e.get('rotation_le_30_pct', 0):.2f}%; pose success: "
                  f"{e.get('pose_success_pct', 0):.2f}%.",
                  f"Reference-member cohort: {r.get('n', 0)} poses; rotation ≤30°: "
                  f"{r.get('rotation_le_30_pct', 0):.2f}%; pose success: "
                  f"{r.get('pose_success_pct', 0):.2f}%.", "",
                  "| object | n | rot≤30° | pose success | gross>45° |",
                  "|---|---:|---:|---:|---:|"]
        for obj, row in result["per_object"].items():
            lines.append(f"| {obj} | {row['n']} | {row['rotation_le_30_pct']:.1f}% | "
                         f"{row['pose_success_pct']:.1f}% | {row['rotation_gt_45_pct']:.1f}% |")
    if sweep:
        lines += ["", "## Candidate score sweep", "",
                  f"Candidate samples: {sweep['samples']}. {sweep['split']}.", "",
                  f"Current exploratory test rotation ≤30°: "
                  f"{sweep['current']['test']['rotation_le_30_pct']:.2f}%.",
                  f"Best exploratory split rotation ≤30°: "
                  f"{sweep['best']['test']['rotation_le_30_pct']:.2f}% "
                  f"with `{sweep['best_train_params']}`.",
                  f"Binary candidate-availability oracle pose success: "
                  f"{sweep['candidate_availability_oracle']['test']['pose_candidate_available_pct']:.2f}%."]
    if pose_memory:
        value = pose_memory["anchor_agreement"]
        lines += ["", "## Static SLAM pose memory (separate operating mode)", "",
                  f"Chronological first-half anchors cover "
                  f"{pose_memory['covered_test_detections']}/{pose_memory['test_detections']} "
                  f"second-half detections. Anchor/reference agreement: "
                  f"{value.get('pose_success_pct', 0):.2f}%.", "",
                  "This compares map anchors with the full-sequence pseudo reference; repeated "
                  "projections are not independent pose trials or standalone PEM accuracy."]
    if causal_memory:
        value = causal_memory["anchor_agreement"]
        lines += ["", "## Causal static SLAM pose memory", "",
                  f"Initialized {len(causal_memory['initialized'])} trusted objects and produced "
                  f"{causal_memory['post_initialization_outputs']} later-frame projections. "
                  f"Anchor/reference agreement: "
                  f"{value.get('pose_success_pct', 0):.2f}%.", "",
                  "Each anchor is frozen after a 10-pose majority cluster; future frames are not "
                  "used for its initialization. The five anchor comparisons are not 450 "
                  "independent accuracy trials. This is an offline static-scene prototype."]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--trajectory-provenance", required=True)
    parser.add_argument("--dataset-manifest", required=True)
    parser.add_argument("--reference", nargs="+", required=True,
                        help="detection jsonl files used to build pseudo GT")
    parser.add_argument("--frame-image-dir", default="",
                        help="deprecated; quality images are decoded from the attested bag")
    parser.add_argument("--dataset-id", default="longcircle2_sam")
    parser.add_argument("--method", action="append", nargs=2, metavar=("NAME", "JSONL"), required=True)
    parser.add_argument("--candidate-jsonl", default="")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    manifest, trajectory_provenance, reference_provenance = validate_generation_inputs(
        args.dataset_manifest, args.dataset_id, args.trajectory,
        args.trajectory_provenance, args.reference)
    ts, trajectory = load_trajectory(args.trajectory)
    raw_reference = []
    for path in args.reference:
        raw_reference.extend(load_jsonl(path))
    validate_reference_rows(raw_reference)
    reference = enrich(raw_reference, ts, trajectory)
    reference = annotate_crop_quality_from_bag(
        reference, Path(args.dataset_manifest).resolve().parent)
    gt, assignments = build_pseudo_gt(
        reference, min_cluster=5, min_share=0.50, trans_gate=0.08,
        rot_gate=30.0, quality_filter=True, min_quality_samples=10)
    matched_keys = {(row["object"], int(row["stamp_ns"])) for row in reference}
    raw_by_object = defaultdict(list)
    for row in raw_reference:
        name, stamp = row["object"], int(row["stamp_ns"])
        raw_by_object[name].append(row)
        if (name, stamp) not in matched_keys:
            assignments.setdefault(name, {})[stamp] = {
                "status": "slam_missing", "reasons": ["slam_missing"],
                "bbox_area_px": None, "crop_sharpness": None,
                "bbox_area_threshold_px": None,
                "crop_sharpness_threshold": None,
            }
    for name, rows in raw_by_object.items():
        missing_count = sum(
            (name, int(row["stamp_ns"])) not in matched_keys for row in rows)
        if name not in gt:
            gt[name] = {
                "trusted": False, "unavailable_reason": "slam_missing",
                "detections": len(rows), "quality_gate": {
                    "status": "slam_missing", "input_count": len(rows),
                    "valid_quality_count": 0, "selected_count": 0,
                    "invalid_reasons": {"slam_missing": len(rows)},
                },
                "cluster_size": 0, "cluster_share": 0.0,
                "reference_frames": [], "representative_frames": [],
            }
        else:
            quality = gt[name]["quality_gate"]
            matched_count = int(quality.get("input_count", 0))
            quality["input_count"] = len(rows)
            quality["slam_matched_count"] = matched_count
            quality["slam_missing_count"] = missing_count
            if missing_count:
                invalid = dict(quality.get("invalid_reasons") or {})
                invalid["slam_missing"] = missing_count
                quality["invalid_reasons"] = dict(sorted(invalid.items()))
            gt[name]["detections"] = len(rows)
    pose_memory = evaluate_static_pose_memory(reference, gt)
    causal_memory = evaluate_causal_pose_memory(reference, gt)
    reference_members = {
        (name, int(frame["stamp_ns"]))
        for name, value in gt.items()
        for frame in value.get("reference_frames", [])
    }

    metrics, failures = {}, []
    for name, path in args.method:
        method_rows = load_jsonl(path)
        result, cases = evaluate(
            name, enrich(method_rows, ts, trajectory), gt, reference_members)
        metrics[name] = result
        failures.extend(case for case in cases if case["category"] != "correct")

    sweep = None
    if args.candidate_jsonl:
        sweep = candidate_sweep(
            load_jsonl(args.candidate_jsonl), ts, trajectory, gt, reference_members)
    provenance = {"file": str(Path(args.trajectory_provenance)),
                  "sha256": sha256(args.trajectory_provenance),
                  "content": trajectory_provenance}
    serial_gt = {
        **gt,
        "_provenance": {
            "trusted": False,
            "schema_version": 1,
            "kind": "same-camera ORB-SLAM3 pseudo-GT",
            "dataset_id": args.dataset_id,
            "optical_frame": manifest["camera"]["optical_frame"],
            "dataset_manifest": {
                "file": str(Path(args.dataset_manifest)),
                "dataset_id": args.dataset_id,
                "note": "mutable manifest path; bag/metadata hashes are validated upstream",
            },
            "trajectory": str(Path(args.trajectory)),
            "trajectory_sha256": sha256(args.trajectory),
            "trajectory_provenance": provenance,
            "reference_inputs": [{"file": str(Path(path)), "sha256": sha256(path)}
                                 for path in args.reference],
            "reference_provenance": reference_provenance,
            "quality_image_source": "attested bag color topic",
            "reference_policy": {
                "quality": "joint per-object bbox-area and crop-sharpness q70..q00",
                "minimum_quality_samples": 10,
                "rotation_gate_deg": 30.0,
                "translation_gate_m": 0.08,
                "minimum_cluster": 5,
                "minimum_cluster_share": 0.50,
                "selection": "maximum direct neighborhood; no transitive DBSCAN",
            },
            "evaluation_warning": (
                "Exploratory self-consistency reference built from SAM-6D outputs; "
                "not externally measured ground truth."),
        },
    }
    (out / "pseudo_ground_truth.json").write_text(
        json.dumps(serial_gt, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "metrics.json").write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "reference_assignments.json").write_text(
        json.dumps(assignments, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "failure_cases.jsonl").write_text(
        "".join(json.dumps(r, ensure_ascii=False) + "\n" for r in sorted(
            failures, key=lambda x: (-x["rotation_error_deg"], -x["translation_error_m"]))),
        encoding="utf-8")
    if sweep is not None:
        (out / "candidate_sweep.json").write_text(
            json.dumps(sweep, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "static_pose_memory.json").write_text(
        json.dumps(pose_memory, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "causal_pose_memory.json").write_text(
        json.dumps(causal_memory, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "REPORT.md").write_text(
        markdown_report(gt, metrics, sweep, pose_memory, causal_memory), encoding="utf-8")
    print((out / "REPORT.md").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
