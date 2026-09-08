#!/usr/bin/env python3
"""Localize one SAM-6D dataset and replay its poses through ObjectMemory."""
from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(HERE), str(ROOT / "sam6d_ws/realtime")]
sys.path.append(str(ROOT / "objectmemory_ws/object_memory/src"))

from camera_extrinsic_localization import (  # noqa: E402
    rpy_to_matrix, run_launch, trajectory_ok, write_config, write_settings,
)
from rig_offset import read_tum, rot_angle_deg, slerp_interp  # noqa: E402
from run_integration import write_topview_svg  # noqa: E402
from slam_pose_memory import (  # noqa: E402
    ObjectAnchorManager, pose_distance, pose_matrix,
)
from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus  # noqa: E402
from core.transforms import predict_current_camera_object_pose  # noqa: E402
from pipeline.object_memory_runner import run_object_memory  # noqa: E402

SYMMETRY_AXES = {
    "Sikhye_high": [0.0, 0.0, 1.0],
    "Sauce_high": [0.0, 0.0, 1.0],
    "Mugcup_high": [0.1057, 0.0337, 0.9938],
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_sam_run(path: Path) -> dict:
    """Cheap streaming checks for the immutable exhaustive Explorer output."""
    manifest = json.loads((path / "explorer_manifest.json").read_text())
    if not manifest.get("completed") or manifest.get("capture_profile") != "exhaustive_visualization":
        raise ValueError(f"SAM run is not complete: {path}")
    required = ("frames.jsonl", "detections.jsonl", "explorer_index.jsonl",
                "candidates.bin", "replay.bin", "preview.mp4")
    if any(not (path / name).is_file() for name in required):
        raise ValueError(f"SAM run is missing a required output: {path}")
    expected_candidates = (int(manifest["candidate_count"])
                           * int(manifest["candidate_record_bytes"]))
    if (path / "candidates.bin").stat().st_size != expected_candidates:
        raise ValueError(f"candidate binary size mismatch: {path}")

    frames = 0
    frame_stamps = []
    first = last = previous = None
    with (path / "frames.jsonl").open() as stream:
        for frames, line in enumerate(stream, 1):
            row = json.loads(line)
            stamp, seq = int(row["stamp_ns"]), int(row["frame_seq"])
            if seq != frames - 1 or (previous is not None and stamp <= previous):
                raise ValueError(f"non-contiguous frame stream: {path}")
            first, last, previous = first or stamp, stamp, stamp
            frame_stamps.append(stamp)
    if (frames != int(manifest["frames_processed"])
            or frames != int(manifest["bag_frame_count"])
            or first != int(manifest["first_stamp_ns"])
            or last != int(manifest["last_stamp_ns"])):
        raise ValueError(f"frame/manifest mismatch: {path}")

    detections, objects = 0, Counter()
    with (path / "detections.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            seq, stamp = int(row["frame_seq"]), int(row["stamp_ns"])
            if not 0 <= seq < len(frame_stamps) or stamp != frame_stamps[seq]:
                raise ValueError(f"detection/frame mismatch: {path}")
            R, t = np.asarray(row["R"], float), np.asarray(row["t_mm"], float)
            if (R.shape != (3, 3) or t.shape != (3,) or not np.isfinite(R).all()
                    or not np.isfinite(t).all() or abs(np.linalg.det(R) - 1) > 2e-3
                    or not np.allclose(R.T @ R, np.eye(3), atol=2e-3)):
                raise ValueError(f"invalid detection pose in {path}")
            detections += 1
            objects[row["object"]] += 1
    run_meta = json.loads((path / "run_meta.json").read_text())
    if (run_meta.get("summary", {}).get("completed") is not True
            or int(run_meta["summary"]["detections"]) != detections):
        raise ValueError(f"detection/run_meta mismatch: {path}")
    preview = manifest["preview_video"]
    if (not math.isfinite(float(preview["fps"])) or float(preview["fps"]) <= 0
            or int(preview["frame_count"]) != frames
            or int(preview["size_bytes"]) != (path / "preview.mp4").stat().st_size
            or preview["sha256"] != sha256(path / "preview.mp4")):
        raise ValueError(f"preview mismatch: {path}")
    return {
        "ok": True, "path": str(path), "bytes": sum(p.stat().st_size for p in path.rglob("*") if p.is_file()),
        "frames": frames, "detections": detections, "objects": dict(sorted(objects.items())),
        "first_stamp_ns": first, "last_stamp_ns": last,
    }


def load_urdf(path: Path) -> np.ndarray:
    origin = ET.parse(path).find(".//joint[@name='slam_cam_to_sam_cam']/origin")
    if origin is None:
        raise ValueError(f"slam_cam_to_sam_cam origin missing: {path}")
    xyz = np.asarray([float(v) for v in origin.attrib["xyz"].split()])
    rpy = [float(v) for v in origin.attrib["rpy"].split()]
    if xyz.shape != (3,) or len(rpy) != 3 or not np.isfinite([*xyz, *rpy]).all():
        raise ValueError(f"invalid camera extrinsic: {path}")
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rpy_to_matrix(*rpy), xyz
    return result


def localize_slam_camera(converted: Path, map_results: Path, out: Path,
                         force: bool) -> Path:
    info = json.loads((converted / "info.json").read_text())
    source_extrinsic = json.loads((map_results / "camera_extrinsic.json").read_text())
    map_dataset = source_extrinsic["dataset"]
    map_dir = map_results / "map_slam"
    atlas_name = f"map_{map_dataset}"
    atlas_source = map_dir / f"{atlas_name}.osa"
    if not atlas_source.is_file() or not trajectory_ok(
            map_dir / "CameraTrajectory.txt", info["SLAM"]["duration_s"]):
        raise ValueError(f"SLAM map is incomplete: {map_dir}")
    out.mkdir(parents=True, exist_ok=True)
    atlas = out / atlas_source.name
    if not atlas.exists():
        shutil.copy2(atlas_source, atlas)
    settings, config = out / "orb_settings.yaml", out / "orb_config.yaml"
    write_settings(info, "SLAM", settings, "System.LoadAtlasFromFile", atlas_name, 0.05)
    write_config(info, "SLAM", config, converted / "SLAM", settings, out, 1.0, 12.0,
                 True, dense_map=True)
    trajectory = out / "CameraTrajectory.txt"
    dense_map = out / "orbslam3_dense_map.pcd"
    if (force or not trajectory_ok(trajectory, info["SLAM"]["duration_s"])
            or not dense_map.is_file()):
        trajectory.unlink(missing_ok=True)
        dense_map.unlink(missing_ok=True)
        run_launch(config, out, info["SLAM"]["duration_s"] + 600.0)
    if (not trajectory_ok(trajectory, info["SLAM"]["duration_s"])
            or not dense_map.is_file()):
        raise ValueError(f"only-localization produced no usable trajectory/dense map: {out}")
    shutil.copyfile(trajectory, out / "trajectory.txt")
    return trajectory


def nearest_pose(stamp_ns: int, timestamps: np.ndarray, poses: np.ndarray,
                 max_dt_s: float = 0.1):
    stamp = stamp_ns / 1e9
    at = int(np.searchsorted(timestamps, stamp))
    if 0 < at < len(timestamps) and timestamps[at] - timestamps[at - 1] <= 0.2:
        return slerp_interp(timestamps, poses, stamp, max_gap=0.2), stamp_ns, 0.0
    candidates = [i for i in (at - 1, at) if 0 <= i < len(timestamps)]
    if not candidates:
        return None, None, None
    best = min(candidates, key=lambda i: abs(float(timestamps[i] - stamp)))
    dt = abs(float(timestamps[best] - stamp))
    return ((poses[best], int(round(timestamps[best] * 1e9)), dt)
            if dt <= max_dt_s else (None, None, dt))


def _json_matrix(value, digits=8):
    return np.asarray(value, float).round(digits).tolist()


def realtime_inference_frames(frame_rows: list) -> set[int]:
    """Keep the first frame arriving after the single SAM-6D worker becomes free."""
    selected = set()
    busy_until_ns = -1
    for row in frame_rows:
        stamp = int(row["stamp_ns"])
        if stamp < busy_until_ns:
            continue
        inference_ms = float(row["inference_ms"])
        if not math.isfinite(inference_ms) or inference_ms < 0:
            raise ValueError(f"invalid SAM-6D inference time at frame {row['frame_seq']}")
        selected.add(int(row["frame_seq"]))
        busy_until_ns = stamp + math.ceil(inference_ms * 1_000_000)
    return selected


def _corrected_pose(memory, decision):
    """Serialize the fused pose only for a successfully matched active object."""
    if (memory is None or memory.camera_T_map_cam is None or decision is None
            or decision.decision_type.value not in {"short_term_match", "long_term_match"}):
        return None
    snapshot = next((item for item in memory.objects
                     if item.object_id == decision.object_id), None)
    if snapshot is None or snapshot.status.value != "active":
        return None
    T = predict_current_camera_object_pose(memory.camera_T_map_cam, snapshot.T_map_obj)
    return {
        "R": [[round(T[i][j], 8) for j in range(3)] for i in range(3)],
        "t_mm": [round(T[i][3] * 1000, 4) for i in range(3)],
    }


def _predicted_poses(objects, T_map_cam):
    """Project confirmed map landmarks into the current SAM camera frame."""
    if T_map_cam is None:
        return []
    camera = tuple(tuple(float(v) for v in row) for row in T_map_cam)
    predictions = []
    for item in objects:
        if item["status"] not in {"active", "remembered"}:
            continue
        landmark = tuple(tuple(float(v) for v in row)
                         for row in item["fused_T_map_obj"])
        T = predict_current_camera_object_pose(camera, landmark)
        predictions.append({
            "object_id": item["object_id"], "object_name": item["object_name"],
            "status": item["status"],
            "R": [[round(T[i][j], 8) for j in range(3)] for i in range(3)],
            "t_mm": [round(T[i][3] * 1000, 4) for i in range(3)],
        })
    return predictions


def load_dense_topview(path: Path, center_y: float, half_span_m: float,
                       max_points: int = 30_000) -> list:
    """Read an ASCII ORB3 dense PCD and retain a wall-height x-z slice."""
    point_count = data_line = None
    with path.open(encoding="ascii", errors="replace") as stream:
        for line_no, line in enumerate(stream, 1):
            if line.startswith("POINTS "):
                point_count = int(line.split()[1])
            if line.startswith("DATA "):
                if line.strip() != "DATA ascii":
                    raise ValueError(f"dense map must be ASCII PCD: {path}")
                data_line = line_no
                break
        if point_count is None or data_line is None:
            raise ValueError(f"invalid dense map PCD header: {path}")
        stride = max(1, math.ceil(point_count / max_points))
        points = []
        for index, line in enumerate(stream):
            if index % stride:
                continue
            values = line.split()
            if len(values) < 3:
                continue
            x, y, z = (float(value) for value in values[:3])
            if (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
                    and abs(y - center_y) <= half_span_m):
                points.append([round(x, 4), round(z, 4)])
    if not points:
        raise ValueError("dense map height filter removed every point")
    return points


def write_timeline(path: Path, frame_rows: list, raw_by_frame: dict,
                   frame_results: list, transitions: list,
                   anchor_by_detection: dict, timestamps: np.ndarray,
                   slam_poses: np.ndarray, X: np.ndarray) -> list:
    """Join existing per-detection processing onto the complete video frame axis."""
    results = {row.frame_idx: row for row in frame_results}
    stamp_seq = [(int(row["stamp_ns"]), int(row["frame_seq"])) for row in frame_rows]
    transition_by_seq = {}
    transition_names = {
        "active": "activated", "lost": "lost", "remembered": "remembered",
        "deleted": "deleted",
    }
    for object_id, transition in transitions:
        if transition.stamp is None:
            continue
        stamp_ns = int(round(transition.stamp * 1e9))
        nearest = min(stamp_seq, key=lambda item: abs(item[0] - stamp_ns))
        if abs(nearest[0] - stamp_ns) > 1_000:
            raise ValueError("ObjectMemory transition is outside the frame stream")
        seq = nearest[1]
        event = "returned" if transition.reason == "redetected" else transition_names.get(
            transition.to_status.value, transition.to_status.value)
        transition_by_seq.setdefault(seq, []).append({
            "object_id": object_id, "event": event,
            "from": transition.from_status.value, "to": transition.to_status.value,
            "reason": transition.reason,
        })

    current_objects = []
    current_anchors = {}
    camera_path = []
    prediction_frames = prediction_count = 0
    with path.open("w", encoding="utf-8") as stream:
        for source_frame in frame_rows:
            seq, stamp = int(source_frame["frame_seq"]), int(source_frame["stamp_ns"])
            memory = results.get(seq)
            decisions = ({item.detection_id: item for item in memory.decisions}
                         if memory else {})
            events = list(transition_by_seq.get(seq, ()))
            detections = []
            for detection_id, raw in raw_by_frame.get(seq, ()):
                decision = decisions.get(detection_id)
                if decision and decision.decision_type.value == "new_tentative":
                    events.append({"object_id": decision.object_id, "event": "created",
                                   "from": None, "to": "tentative", "reason": "first_detection"})
                anchor = anchor_by_detection.get(detection_id)
                if anchor:
                    if anchor["state"] == "registered":
                        current_anchors[raw["object"]] = {
                            "object_name": raw["object"], "state": "registered",
                            "xyz": [anchor["T_map_obj"][i][3] for i in range(3)],
                            "T_map_obj": anchor["T_map_obj"],
                        }
                    else:
                        current_anchors.pop(raw["object"], None)
                detections.append({
                    "detection_id": detection_id, "object_name": raw["object"],
                    "bbox": raw.get("bbox"), "R": raw["R"], "t_mm": raw["t_mm"],
                    "corrected_pose": _corrected_pose(memory, decision),
                    "quality": {
                        "score": raw.get("score"),
                        "depth_valid_frac": (raw.get("pem") or {}).get("depth_valid_frac"),
                        "mask_iou": raw.get("mask_iou"),
                        "texture_score": raw.get("texture_score"),
                        "cluster_occupancy": raw.get("cluster_occupancy"),
                    },
                    "object_memory": None if decision is None else {
                        "object_id": decision.object_id,
                        "decision": decision.decision_type.value,
                        "reject_reason": decision.reject_reason,
                    },
                    "anchor": anchor,
                })

            if memory is not None:
                current_objects = [{
                    "object_id": item.object_id, "object_name": item.object_name,
                    "status": item.status.value, "visible": item.visible,
                    "observation_count": item.observation_count,
                    "fused_T_map_obj": _json_matrix(item.T_map_obj),
                    "observed_T_map_obj": (None if item.T_map_obj_meas is None
                                            else _json_matrix(item.T_map_obj_meas)),
                    "residual_translation_mm": (None if item.residual_t is None
                                                   else round(item.residual_t * 1000, 4)),
                    "residual_rotation_deg": (None if item.residual_deg is None
                                                else round(item.residual_deg, 4)),
                } for item in memory.objects]
            display_objects = current_objects if memory is not None else [
                {**item, "visible": False, "observed_T_map_obj": None,
                 "residual_translation_mm": None, "residual_rotation_deg": None}
                for item in current_objects]

            T_map_slam, pose_stamp, dt = nearest_pose(stamp, timestamps, slam_poses)
            camera = T_map_cam = None
            if T_map_slam is not None:
                T_map_cam = T_map_slam @ X
                camera = {
                    "xyz": T_map_cam[:3, 3].round(6).tolist(),
                    "forward": T_map_cam[:3, 2].round(6).tolist(),
                    "slam_stamp_ns": str(pose_stamp), "slam_dt_ms": round(dt * 1000, 3),
                }
                camera_path.append(camera["xyz"])
            predictions = _predicted_poses(display_objects, T_map_cam)
            prediction_count += len(predictions)
            prediction_frames += bool(predictions)
            row = {
                "frame_seq": seq, "stamp_ns": str(stamp),
                "timestamp_s": round((stamp - int(frame_rows[0]["stamp_ns"])) / 1e9, 6),
                "camera": camera, "sam_inferred": source_frame["sam_inferred"],
                "memory_processed": memory is not None,
                "detections": detections, "objects": display_objects,
                "predictions": predictions,
                "transitions": events, "anchors": list(current_anchors.values()),
            }
            stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
    return camera_path, {"frames": prediction_frames, "poses": prediction_count}


def run_memory(dataset: str, map_dataset: str, trajectory: Path, urdf: Path,
               sam_run: Path, converted: Path, extrinsic_json: Path, out: Path,
               map_height_half_span_m: float,
               realtime: bool = False) -> dict:
    timestamps, slam_poses = read_tum(trajectory)
    if len(timestamps) < 30:
        raise ValueError(f"too few localized SLAM poses: {trajectory}")
    X = load_urdf(urdf)
    source_extrinsic = json.loads(extrinsic_json.read_text())
    expected_X = np.asarray(source_extrinsic["X_slam_camera_to_sam_camera"], float)
    if np.max(np.abs(X - expected_X)) > 2e-6:
        raise ValueError(f"URDF/estimated extrinsic mismatch: {urdf}")

    manager = ObjectAnchorManager(map_dataset, {"symmetry_axes": SYMMETRY_AXES})
    out.mkdir(parents=True, exist_ok=True)
    stream_path = out / "pose_stream.jsonl"
    frame_rows = []
    with (sam_run / "frames.jsonl").open() as source:
        for line in source:
            row = json.loads(line)
            frame_rows.append({"stamp_ns": int(row["stamp_ns"]),
                               "frame_seq": int(row["frame_seq"]),
                               "inference_ms": (row.get("ms") or {}).get("total")})
    inferred_frames = (realtime_inference_frames(frame_rows) if realtime else
                       {row["frame_seq"] for row in frame_rows})
    for row in frame_rows:
        row["sam_inferred"] = row["frame_seq"] in inferred_frames
    frame_timestamps = [int(row["stamp_ns"]) / 1e9 for row in frame_rows]
    detections_by_frame = {seq: [] for seq in inferred_frames}
    raw_by_frame = {}
    anchor_by_detection = {}
    counts = Counter()
    events = []
    with (sam_run / "detections.jsonl").open() as source, stream_path.open("w") as stream:
        for line in source:
            row = json.loads(line)
            counts["source_detections"] += 1
            if int(row["frame_seq"]) not in inferred_frames:
                counts["skipped_detections"] += 1
                continue
            counts["detections"] += 1
            stamp = int(row["stamp_ns"])
            name = row["object"]
            frame_seq = int(row["frame_seq"])
            T_sam_obj = pose_matrix(row["R"], np.asarray(row["t_mm"], float) / 1000.0)
            detection_id = counts["detections"]
            # The source row carries several KB of candidate diagnostics. The
            # timeline needs only the accepted pose and compact quality fields.
            raw_by_frame.setdefault(frame_seq, []).append((detection_id, {
                "object": name, "score": row.get("score"), "R": row["R"],
                "t_mm": row["t_mm"], "bbox": row.get("bbox"),
                "pem": {"depth_valid_frac": (row.get("pem") or {}).get(
                    "depth_valid_frac")},
                "mask_iou": row.get("mask_iou"),
                "texture_score": row.get("texture_score"),
                "cluster_occupancy": row.get("cluster_occupancy"),
            }))
            detections_by_frame.setdefault(frame_seq, []).append(Sam6DDetection(
                stamp=stamp / 1e9, frame_id="sam_camera",
                detection_id=detection_id, object_name=name,
                T_cam_obj=tuple(tuple(float(v) for v in matrix_row) for matrix_row in T_sam_obj),
                score=float(row.get("score", 0.0)), bbox=tuple(row.get("bbox") or ()),
                depth_quality=(row.get("pem") or {}).get("depth_valid_frac"),
            ))
            T_map_slam, pose_stamp, dt = nearest_pose(stamp, timestamps, slam_poses)
            if T_map_slam is None:
                counts["no_slam_pose"] += 1
                continue
            counts["localized"] += 1
            T_map_sam = T_map_slam @ X
            T_map_obj = T_map_sam @ T_sam_obj
            was_registered = name in manager.anchors
            update = manager.observe(name, T_map_sam, T_sam_obj, "TRACKING_OK",
                                     pose_stamp, stamp)
            is_registered = name in manager.anchors
            event = ("registered" if not was_registered and is_registered else
                     "released" if was_registered and not is_registered else None)
            if event:
                counts[event] += 1
                events.append({"stamp_ns": stamp, "object": name, "event": event,
                               "details": update})
            anchor = manager.anchors.get(name)
            angle = translation = None
            anchored_pose = None
            if anchor is not None:
                angle, translation = pose_distance(
                    anchor, T_map_obj, SYMMETRY_AXES.get(name), 10)
                anchored_pose = np.linalg.inv(T_map_sam) @ anchor
                counts["anchor_outputs"] += 1
            compact = {
                "stamp_ns": stamp, "frame_seq": frame_seq,
                "detection_id": detection_id, "object": name,
                "slam_dt_ms": round(dt * 1000, 3), "anchor_state": "registered" if is_registered else "collecting",
                "event": event, "raw_map_xyz_m": T_map_obj[:3, 3].round(6).tolist(),
                "anchor_error_rotation_deg": None if angle is None else round(angle, 4),
                "anchor_error_translation_mm": None if translation is None else round(translation * 1000, 4),
                "output_R": row["R"] if anchored_pose is None else anchored_pose[:3, :3].round(6).tolist(),
                "output_t_mm": row["t_mm"] if anchored_pose is None else (anchored_pose[:3, 3] * 1000).round(4).tolist(),
                "pose_source": "sam6d" if anchored_pose is None else "slam_anchor",
            }
            anchor_by_detection[detection_id] = {
                "state": compact["anchor_state"], "event": event,
                "error_rotation_deg": compact["anchor_error_rotation_deg"],
                "error_translation_mm": compact["anchor_error_translation_mm"],
                "T_map_obj": None if anchor is None else _json_matrix(anchor),
            }
            stream.write(json.dumps(compact, ensure_ascii=False) + "\n")

    anchors = []
    for object_id, (name, anchor) in enumerate(sorted(manager.anchors.items()), 1):
        anchors.append({
            "object_id": object_id, "object_name": name, "status": "registered",
            "xyz": anchor[:3, 3].round(6).tolist(),
            "T_map_obj": anchor.round(8).tolist(),
        })
    objectmemory_poses = [SlamCameraPose(
        stamp=float(stamp), frame_id="map", child_frame_id="sam_camera",
        T_map_cam=tuple(tuple(float(v) for v in line) for line in pose @ X),
        tracking_status=TrackingStatus.OK, source_slam_id="orbslam3_only_localization",
    ) for stamp, pose in zip(timestamps, slam_poses)]
    camera = json.loads((converted / "info.json").read_text())["SAM"]["camera"]
    K = camera["K"]
    persistent = run_object_memory(
        objectmemory_poses, detections_by_frame, frame_timestamps,
        time_tolerance=0.1, assoc_trans_gate_m=0.20, assoc_rot_gate_deg=-1.0,
        cam_K=(tuple(K[:3]), tuple(K[3:6]), tuple(K[6:9])),
        img_size=(int(camera["width"]), int(camera["height"])),
        quality_weighting=True,
    )
    objects = []
    for landmark in sorted(persistent.store.all_landmarks(), key=lambda item: item.object_id):
        T = np.asarray(landmark.T_map_obj)
        objects.append({
            "object_id": landmark.object_id, "object_name": landmark.object_name,
            "status": landmark.status.value, "confidence": round(landmark.confidence, 4),
            "observations": landmark.observation_count, "missed": landmark.missed_count,
            "xyz": T[:3, 3].round(6).tolist(), "T_map_obj": T.round(8).tolist(),
            "pose_history": [dataclasses.asdict(sample) for sample in landmark.pose_history],
        })
    persistent_stats = {
        "frames_with_detections": persistent.frames_total,
        "frames_fused": persistent.frames_with_slam,
        "frames_dropped_no_slam_pose": persistent.frames_without_slam,
        "detections": persistent.detections_total, "instances": len(objects),
        "live_instances": sum(row["status"] != "deleted" for row in objects),
    }
    timeline_path = out / "object_memory_timeline.jsonl"
    camera_path, overlay_stats = write_timeline(
        timeline_path, frame_rows, raw_by_frame, persistent.frame_results,
        persistent.store.transition_log(), anchor_by_detection, timestamps, slam_poses, X)
    dense_path = trajectory.parent / "orbslam3_dense_map.pcd"
    dense_xz = load_dense_topview(
        dense_path, float(np.median(slam_poses[:, 1, 3])), map_height_half_span_m)
    scene_points = [point for point in camera_path]
    scene_points.extend(row["xyz"] for row in objects if row["status"] != "deleted")
    if not scene_points:
        raise ValueError("no localized camera or object poses for explorer map")
    xs = [point[0] for point in scene_points] + [point[0] for point in dense_xz]
    zs = [point[2] for point in scene_points] + [point[1] for point in dense_xz]
    preview = json.loads((sam_run / "explorer_manifest.json").read_text())["preview_video"]
    result = {
        "dataset": dataset, "map_id": map_dataset,
        "sam_inference": {
            "mode": "recorded_latency_realtime" if realtime else "exhaustive",
            "input_frames": len(frame_rows), "inferred_frames": len(inferred_frames),
            "skipped_frames": len(frame_rows) - len(inferred_frames),
        },
        "formula": "T_map_obj = T_map_camSLAM(t) * X_camSLAM_camSAM(URDF) * T_camSAM_obj",
        "corrected_pose": {
            "formula": "T_cam_obj_corrected = inverse(T_map_cam) * T_map_obj_fused",
            "minimum_status": "active",
        },
        "inputs": {"trajectory": str(trajectory), "extrinsic_urdf": str(urdf),
                   "sam_detections": str(sam_run / "detections.jsonl")},
        "extrinsic": X.round(9).tolist(), "stats": dict(counts),
        "trajectory": {"poses": len(timestamps), "first_s": float(timestamps[0]),
                       "last_s": float(timestamps[-1])},
        "events": events, "anchors": anchors,
        "persistent_stats": persistent_stats, "objects": objects,
        "explorer": {
            "schema_version": 1,
            "coordinate_system": "existing map coordinates; top view uses x-z",
            "frame_count": len(frame_rows),
            "frame_range": [0, len(frame_rows) - 1],
            "stamp_range_ns": [str(frame_rows[0]["stamp_ns"]), str(frame_rows[-1]["stamp_ns"])],
            "map_bounds_xz": {"min": [min(xs), min(zs)], "max": [max(xs), max(zs)]},
            "dense_map": {"source": str(dense_path.resolve()), "points_xz": dense_xz,
                          "height_center_m": round(float(np.median(slam_poses[:, 1, 3])), 4),
                          "height_half_span_m": map_height_half_span_m},
            "camera": {"K": K, "width": int(camera["width"]),
                       "height": int(camera["height"])},
            "map_pose_overlay": overlay_stats,
            "preview": {"path": str((sam_run / "preview.mp4").resolve()),
                        "fps": float(preview["fps"]), "frame_count": int(preview["frame_count"]),
                        "width": int(preview["width"]), "height": int(preview["height"])},
            "timeline": timeline_path.name,
        },
    }
    (out / "object_map.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    live = [row for row in objects if row["status"] != "deleted"]
    write_topview_svg(camera_path, live,
                      out / "map_topview.svg", f"{dataset} — ObjectMemory")
    write_report(out / "object_memory_report.md", result)
    return result


def write_report(path: Path, result: dict) -> None:
    stats = result["stats"]
    persistent = result["persistent_stats"]
    overlay = result["explorer"]["map_pose_overlay"]
    lines = [f"# ObjectMemory — {result['dataset']}", "", f"- map: `{result['map_id']}`",
             f"- detections: {stats.get('detections', 0)}",
             f"- localized: {stats.get('localized', 0)}",
             f"- no SLAM pose: {stats.get('no_slam_pose', 0)}",
             f"- anchor outputs: {stats.get('anchor_outputs', 0)}",
             f"- register/release events: {stats.get('registered', 0)}/{stats.get('released', 0)}",
             f"- map-pose overlay: {overlay['frames']} frames / {overlay['poses']} poses",
             f"- persistent frames fused: {persistent['frames_fused']}/{persistent['frames_with_detections']}",
             f"- persistent live instances: {persistent['live_instances']}/{persistent['instances']}",
             "", "## Persistent landmarks (`objectmemory_ws`)", "",
             "| id | object | status | observations | confidence | map xyz (m) |",
             "| --- | --- | --- | --- | --- | --- |"]
    for row in result["objects"]:
        lines.append(f"| {row['object_id']} | {row['object_name']} | {row['status']} | "
                     f"{row['observations']} | {row['confidence']} | {row['xyz']} |")
    lines.extend(["", "## Registered anchors (`slam_pose_memory`)", "",
                  "| object | map xyz (m) |", "| --- | --- |"])
    for row in result["anchors"]:
        lines.append(f"| {row['object_name']} | {row['xyz']} |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def self_test() -> None:
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "x.urdf"
        path.write_text("""<robot><joint name="slam_cam_to_sam_cam"><origin xyz="1 2 3" rpy="0 0 0"/></joint></robot>""")
        assert np.allclose(load_urdf(path)[:3, 3], [1, 2, 3])
    manager = ObjectAnchorManager("test", {"symmetry_axes": SYMMETRY_AXES})
    pose = pose_matrix(np.eye(3), [0, 0, 1])
    for stamp in range(20):
        manager.observe("milk", np.eye(4), pose, "TRACKING_OK", stamp, stamp)
    assert "milk" in manager.anchors
    assert rot_angle_deg(manager.anchors["milk"][:3, :3]) < 1e-8
    poses = np.repeat(np.eye(4)[None], 2, axis=0)
    poses[1, 0, 3] = 1.0
    interpolated, pose_stamp, dt = nearest_pose(
        50_000_000, np.asarray([0.0, 0.1]), poses)
    assert np.allclose(interpolated[:3, 3], [0.5, 0, 0])
    assert pose_stamp == 50_000_000 and dt == 0.0
    from types import SimpleNamespace
    camera = np.eye(4)
    camera[2, 3] = 0.25
    camera = tuple(tuple(float(v) for v in row) for row in camera)
    fused = tuple(tuple(float(v) for v in row) for row in pose)
    active = SimpleNamespace(object_id=7, status=SimpleNamespace(value="active"),
                             T_map_obj=fused)
    memory = SimpleNamespace(camera_T_map_cam=camera, objects=[active])
    matched = SimpleNamespace(decision_type=SimpleNamespace(value="short_term_match"),
                              object_id=7)
    corrected = _corrected_pose(memory, matched)
    assert corrected["t_mm"] == [0.0, 0.0, 750.0]
    predictions = _predicted_poses([{
        "object_id": 7, "object_name": "milk", "status": "active",
        "fused_T_map_obj": fused,
    }], np.asarray(camera))
    assert predictions[0]["t_mm"] == [0.0, 0.0, 750.0]
    assert _predicted_poses([{**predictions[0], "status": "lost",
                              "fused_T_map_obj": fused}], camera) == []
    rows = [{"frame_seq": i, "stamp_ns": i * 33_000_000, "inference_ms": 60}
            for i in range(5)]
    assert realtime_inference_frames(rows) == {0, 2, 4}
    active.status.value = "tentative"
    assert _corrected_pose(memory, matched) is None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--force-localize", action="store_true")
    parser.add_argument("--dataset-dir", type=Path,
                        help="dataset folder containing converted/ and sam6d/")
    parser.add_argument("--map-dir", type=Path,
                        help="map dataset folder containing camera_extrinsic/")
    parser.add_argument("--map-height-half-span", type=float, default=0.8,
                        help="dense-map wall slice half-height in metres (default: 0.8)")
    parser.add_argument("--realtime", action="store_true",
                        help="retired; use run_object_memory_rosbag.py")
    args = parser.parse_args()
    if args.map_height_half_span <= 0:
        parser.error("--map-height-half-span must be positive")
    self_test()
    if args.self_test:
        print("self-test: PASS")
        return 0
    if args.realtime:
        parser.error("--realtime offline replay is retired; use "
                     "integration/run_object_memory_rosbag.py")
    if not args.dataset_dir or not args.map_dir:
        parser.error("--dataset-dir and --map-dir are required unless --self-test is used")
    try:
        dataset_dir = args.dataset_dir.expanduser().resolve()
        map_dir = args.map_dir.expanduser().resolve()
        converted, sam_run = dataset_dir / "converted", dataset_dir / "sam6d"
        map_results = map_dir / "camera_extrinsic"
        urdf = map_results / "camera_extrinsic.urdf"
        extrinsic_json = map_results / "camera_extrinsic.json"
        if not (converted / "info.json").is_file() or not sam_run.is_dir():
            raise ValueError(f"expected converted/info.json and sam6d/ under {dataset_dir}")
        if not urdf.is_file() or not extrinsic_json.is_file():
            raise ValueError(f"camera extrinsic result is incomplete: {map_results}")
        dataset = dataset_dir.name
        map_dataset = json.loads(extrinsic_json.read_text())["dataset"]
        print(f"[{dataset}] validating SAM-6D", flush=True)
        validation = validate_sam_run(sam_run)
        root = dataset_dir / "object_memory"
        print(f"[{dataset}] only-localizing SLAM camera with map {map_dataset}", flush=True)
        trajectory = localize_slam_camera(
            converted, map_results, root / "localize_slam", args.force_localize)
        print(f"[{dataset}] applying ObjectMemory", flush=True)
        memory = run_memory(dataset, map_dataset, trajectory, urdf, sam_run,
                            converted, extrinsic_json, root / "fused",
                            args.map_height_half_span, args.realtime)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    summary_path = root / "run_summary.json"
    summary_path.write_text(json.dumps(
        {"sam_validation": validation, "object_memory": memory},
        indent=2, ensure_ascii=False) + "\n")
    print(f"complete: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
