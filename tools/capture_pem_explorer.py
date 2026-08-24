#!/usr/bin/env python3
"""Sequential, no-drop PEM Explorer capture from a trusted RGB-D SQLite bag.

This command deliberately bypasses ROS and the latest-frame shared-memory channel.
It persists compact candidates, replay indices and one bitset Mask PNG per attempted
frame; RGB, depth, candidate renders and feature tensors remain in the source bag/RAM.
"""
from __future__ import annotations

import argparse
import json
import math
import sqlite3
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


REPO = Path(__file__).resolve().parents[1]
REALTIME = REPO / "realtime"
TOOLS = Path(__file__).resolve().parent
if str(REALTIME) not in sys.path:
    sys.path.insert(0, str(REALTIME))
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from pem_explorer_record import ExplorerRecorder, provenance_entry  # noqa: E402
from validate_rgbd_dataset import (  # noqa: E402
    COLOR_INFO_TOPIC,
    COLOR_TOPIC,
    DEPTH_TOPIC,
    DatasetValidationError,
    enforce_ground_truth_policy,
    validate_rgbd_dataset,
    validate_trusted_reference_poses,
)


CAPTURE_PROFILE = "exhaustive_visualization"
FIXED_OUTPUT = REPO / "output" / "longcircle2_visualization"


class CaptureError(RuntimeError):
    pass


def _resolve_repo_path(value):
    path = Path(value)
    return (path if path.is_absolute() else REPO / path).resolve()


def load_capture_config(config_path):
    raw_config_path = Path(config_path)
    if raw_config_path.is_symlink():
        raise CaptureError("capture config must be a real file under the repository")
    config_path = raw_config_path.resolve()
    if REPO.resolve() not in config_path.parents:
        raise CaptureError("capture config must be a real file under the repository")
    try:
        cfg = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise CaptureError(f"invalid capture config {config_path}: {exc}") from exc
    explorer = dict((cfg.get("output", {}).get("pem_explorer") or {}))
    diagnostic = dict((cfg.get("runtime", {}).get("pem_diagnostic") or {}))
    explorer_diag = dict(diagnostic.get("explorer_v2") or {})
    if not explorer.get("enabled") or not diagnostic.get("enabled") or not explorer_diag.get("enabled"):
        raise CaptureError("full capture requires output.pem_explorer and PEM explorer diagnostics")
    if explorer.get("capture_profile") != CAPTURE_PROFILE:
        raise CaptureError(f"output.pem_explorer.capture_profile must be {CAPTURE_PROFILE}")
    if explorer_diag.get("capture_profile") != CAPTURE_PROFILE:
        raise CaptureError(
            f"runtime.pem_diagnostic.explorer_v2.capture_profile must be {CAPTURE_PROFILE}")
    output = _resolve_repo_path(cfg.get("output", {}).get("dir", ""))
    if output != FIXED_OUTPUT.resolve():
        raise CaptureError(f"full capture output is fixed at {FIXED_OUTPUT}")
    bag = _resolve_repo_path(explorer.get("source_bag") or cfg.get("bag", {}).get("path", ""))
    data_root = (REPO / "data").resolve()
    if bag != data_root and data_root not in bag.parents:
        raise CaptureError("full capture source bag must remain under repository data/")
    declared_bag = cfg.get("bag", {}).get("path")
    if declared_bag and _resolve_repo_path(declared_bag) != bag:
        raise CaptureError("source_bag and bag.path must identify the same bag")
    ism_config = (cfg.get("ism") or {}).get("config")
    if ism_config:
        ism_path = _resolve_repo_path(ism_config)
        if (REPO.resolve() not in ism_path.parents or ism_path.is_symlink() or
                not ism_path.is_file()):
            raise CaptureError("ISM config must be a real file under the repository")
    return cfg, explorer, output, bag, config_path


def _header_stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec)


def iter_trusted_bag_frames(validation, topics):
    """Yield exact RGB/depth/CameraInfo tuples in ascending trusted bag timestamp."""
    try:
        from cv_bridge import CvBridge
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise CaptureError(
            "ROS 2 Python image type support is required for offline bag decoding") from exc

    required = {
        "rgb": topics.get("rgb", COLOR_TOPIC),
        "depth": topics.get("depth", DEPTH_TOPIC),
        "caminfo": topics.get("caminfo", COLOR_INFO_TOPIC),
    }
    expected = {"rgb": COLOR_TOPIC, "depth": DEPTH_TOPIC, "caminfo": COLOR_INFO_TOPIC}
    if required != expected:
        raise CaptureError(f"capture topics must be the trusted standard topics: {expected}")

    connection = sqlite3.connect(
        f"file:{validation.database.resolve().as_posix()}?mode=ro", uri=True)
    try:
        topic_rows = connection.execute("SELECT id,name,type FROM topics").fetchall()
        topic_info = {name: (int(topic_id), msgtype)
                      for topic_id, name, msgtype in topic_rows}
        missing = sorted(set(required.values()) - set(topic_info))
        if missing:
            raise CaptureError(f"trusted bag topics disappeared after validation: {missing}")
        classes = {name: get_message(topic_info[topic][1])
                   for name, topic in required.items()}
        cursors = {
            name: connection.execute(
                "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topic_info[topic][0],))
            for name, topic in required.items()
        }
        bridge = CvBridge()
        while True:
            rows = {name: cursor.fetchone() for name, cursor in cursors.items()}
            ended = {name for name, row in rows.items() if row is None}
            if ended:
                if len(ended) != len(rows):
                    raise CaptureError(f"trusted bag topic streams ended unevenly: {sorted(ended)}")
                break
            storage_stamps = {int(row[0]) for row in rows.values()}
            if len(storage_stamps) != 1:
                stamps = {name: int(row[0]) for name, row in rows.items()}
                raise CaptureError(f"trusted bag streams lost timestamp alignment: {stamps}")
            storage_stamp = storage_stamps.pop()
            messages = {
                name: deserialize_message(row[1], classes[name])
                for name, row in rows.items()
            }
            header_stamps = {name: _header_stamp_ns(message)
                             for name, message in messages.items()}
            if set(header_stamps.values()) != {storage_stamp}:
                raise CaptureError(
                    f"header/storage timestamp mismatch at {storage_stamp}: {header_stamps}")
            rgb = np.asarray(bridge.imgmsg_to_cv2(
                messages["rgb"], desired_encoding="rgb8"))
            depth = np.asarray(bridge.imgmsg_to_cv2(
                messages["depth"], desired_encoding="passthrough"))
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise CaptureError(f"decoded RGB shape is invalid at {storage_stamp}: {rgb.shape}")
            if depth.ndim != 2 or depth.shape != rgb.shape[:2]:
                raise CaptureError(
                    f"decoded aligned depth shape mismatch at {storage_stamp}: {depth.shape}")
            if depth.dtype != np.uint16:
                raise CaptureError(
                    f"decoded depth must be uint16 millimetres at {storage_stamp}, "
                    f"got {depth.dtype}")
            K = np.asarray(messages["caminfo"].k, dtype=np.float64).reshape(3, 3)
            if not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
                raise CaptureError(f"invalid CameraInfo intrinsics at {storage_stamp}")
            yield {
                "stamp_ns": storage_stamp,
                "depth_stamp_ns": header_stamps["depth"],
                "bgr": cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                "depth": depth,
                "K": K,
            }
    finally:
        connection.close()


def _quaternion_rotation(qx, qy, qz, qw):
    q = np.asarray([qx, qy, qz, qw], dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise CaptureError("trajectory contains an invalid quaternion")
    x, y, z, w = q / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _pose_matrix(rotation, translation):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.asarray(rotation, dtype=float)
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix


class TrustedReferenceProvider:
    """Convert attested same-camera map-object pseudo-GT to frame-local camera poses."""

    def __init__(self, validation, max_dt_ms=25.0):
        self.enabled = False
        self.times = np.empty(0, dtype=float)
        self.poses = np.empty((0, 4, 4), dtype=float)
        self.objects = {}
        self.max_dt_s = float(max_dt_ms) / 1000.0
        self.provenance = {}
        policy = validation.gt_policy or {}
        self_slam = policy.get("self_slam") or {}
        if not validation.allows_pseudo_gt or not self_slam:
            return
        trajectory = (validation.bag / self_slam["trajectory_path"]).resolve()
        pseudo_gt = (validation.bag / self_slam["pseudo_gt_path"]).resolve()
        enforce_ground_truth_policy(validation, str(pseudo_gt), str(trajectory))
        try:
            raw = json.loads(pseudo_gt.read_text(encoding="utf-8"))
            values = np.loadtxt(trajectory, ndmin=2)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise CaptureError(f"cannot load attested same-camera references: {exc}") from exc
        self.objects = validate_trusted_reference_poses(raw, str(pseudo_gt))
        if (values.ndim != 2 or values.shape[1] != 8 or len(values) < 2
                or not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0)):
            raise CaptureError("attested trajectory must be finite, increasing TUM poses")
        poses = []
        for row in values:
            poses.append(_pose_matrix(_quaternion_rotation(*row[4:8]), row[1:4]))
        self.times = values[:, 0]
        self.poses = np.asarray(poses)
        self.enabled = True
        self.provenance = {
            "policy": "same_camera_self_slam_reference",
            "role": "pseudo-GT reference/evaluation split",
            "trajectory": provenance_entry(trajectory),
            "pseudo_gt": provenance_entry(pseudo_gt),
            "trajectory_convention": "TUM T_wc",
            "max_match_dt_ms": float(max_dt_ms),
        }

    def for_stamp(self, stamp_ns):
        if not self.enabled:
            return {}
        stamp = int(stamp_ns) / 1e9
        at = int(np.searchsorted(self.times, stamp))
        candidates = [i for i in (at - 1, at) if 0 <= i < len(self.times)]
        if not candidates:
            return {}
        best = min(candidates, key=lambda i: abs(float(self.times[i] - stamp)))
        if abs(float(self.times[best] - stamp)) > self.max_dt_s:
            return {}
        camera_from_map = np.linalg.inv(self.poses[best])
        result = {}
        for name, pose in self.objects.items():
            map_from_object = _pose_matrix(pose["R"], pose["t_m"])
            camera_from_object = camera_from_map @ map_from_object
            result[name] = {
                "R": camera_from_object[:3, :3],
                "t_mm": camera_from_object[:3, 3] * 1000.0,
            }
        return result


def _public_frame_diagnostics(diagnostics):
    out = dict(diagnostics)
    out["pem_candidates"] = []
    for attempt in diagnostics.get("pem_candidates", []):
        out["pem_candidates"].append({
            key: value for key, value in attempt.items()
            if not key.startswith("explorer_") and key not in {"decision", "crop_bbox_yxyx"}
        })
    return out


def _capture_provenance(config_path, validation, core, references):
    pem = REPO / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
    result = {
        "config": provenance_entry(config_path),
        "bag": provenance_entry(validation.bag),
        "dataset_manifest": provenance_entry(validation.manifest_path),
        "checkpoint": provenance_entry(pem / "checkpoints" / "sam-6d-pem-base.pth"),
        "templates": {name: provenance_entry(REPO / "assets" / "pem_templates" /
                                               f"{name}.pt") for name in core._tem},
        "cad": {name: provenance_entry(REPO / "assets" / "model_points" /
                                         f"{name}.npy") for name in core._pts},
    }
    if references.provenance:
        result["same_camera_reference"] = references.provenance
    return result


def capture(config_path):
    from sam6d_core import Sam6DCore
    import verify_config as VC

    cfg, explorer_cfg, output, bag, config_path = load_capture_config(config_path)
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"refusing to replace existing full capture: {output}")
    validation = validate_rgbd_dataset(bag, require_manifest=True)
    if validation.pair_count <= 0:
        raise CaptureError("trusted bag contains no RGB-D frames")
    references = TrustedReferenceProvider(
        validation, (cfg.get("runtime", {}).get("pem_diagnostic") or {}).get(
            "reference_max_dt_ms", 25.0))

    runtime = cfg.get("runtime", {})
    core = Sam6DCore(
        cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml"),
        cfg.get("ism", {}).get("objects", []),
        runtime.get("device", "cuda:0"), runtime.get("det_score_thresh", 0.2),
        appe_rerank=runtime.get("appe_rerank"),
        verify=runtime.get("verify", VC.UNSET),
        pem_diagnostic=runtime.get("pem_diagnostic"))

    recording = dict(explorer_cfg)
    recording.update({
        "bag_frame_count": validation.pair_count,
        "expected_first_stamp_ns": validation.first_stamp_ns,
        "expected_last_stamp_ns": validation.last_stamp_ns,
        "completion_tolerance_ns": 0,
        "completion_tolerance_policy": "exact_count_and_first_last_stamp",
    })
    provenance = _capture_provenance(config_path, validation, core, references)
    output.mkdir(parents=True, exist_ok=False)
    recorder = ExplorerRecorder(output, provenance, recording)
    detections = open(output / "detections.jsonl", "x", encoding="utf-8", buffering=1)
    frames = open(output / "frames.jsonl", "x", encoding="utf-8", buffering=1)
    meta_path = output / "run_meta.json"
    meta = {
        "started_wall": time.time(), "mode": "offline_trusted_bag_sequential",
        "capture_profile": CAPTURE_PROFILE, "dataset": validation.summary(),
        "objects": [item["name"] for item in core.objs], "config": cfg,
    }
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n",
                         encoding="utf-8")

    processed = accepted = 0
    started = time.monotonic()
    completed = False
    try:
        for frame_seq, frame in enumerate(iter_trusted_bag_frames(
                validation, cfg.get("topics", {}))):
            t0 = time.time()
            rows, ms, n_boxes, label = core.process(
                frame["bgr"], frame["depth"], frame["K"], want_mask=True,
                diagnostic_references=references.for_stamp(frame["stamp_ns"]))
            t1 = time.time()
            recorder.record_frame(
                frame["stamp_ns"], frame["depth_stamp_ns"], frame_seq,
                frame["K"], frame["depth"].shape,
                core.last_frame_diag.get("pem_candidates", []), label)
            for row in rows:
                detections.write(json.dumps({
                    **row, "stamp_ns": frame["stamp_ns"], "frame_seq": frame_seq,
                }, ensure_ascii=False) + "\n")
            frame_row = {
                "stamp_ns": frame["stamp_ns"], "depth_stamp_ns": frame["depth_stamp_ns"],
                "frame_seq": frame_seq, "n_accept": len(rows), "n_boxes": n_boxes,
                "ms": ms, "t_start_wall": t0, "t_done_wall": t1,
                "objects": [row["object"] for row in rows],
                "diagnostics": _public_frame_diagnostics(core.last_frame_diag),
            }
            frames.write(json.dumps(frame_row, ensure_ascii=False) + "\n")
            processed += 1
            accepted += len(rows)
            print(f"#{frame_seq}/{validation.pair_count - 1} "
                  f"{len(rows)} accepted, {ms['total']} ms", flush=True)
        if processed != validation.pair_count:
            raise CaptureError(
                f"processed {processed} frames, expected {validation.pair_count}")
        if recorder._first_stamp_ns != validation.first_stamp_ns:
            raise CaptureError("processed first stamp does not match trusted bag")
        if recorder._last_stamp_ns != validation.last_stamp_ns:
            raise CaptureError("processed last stamp does not match trusted bag")
        completed = True
    finally:
        elapsed = max(1e-9, time.monotonic() - started)
        meta["summary"] = {
            "bag_frames": validation.pair_count, "frames_processed": processed,
            "detections": accepted, "elapsed_s": round(elapsed, 3),
            "hz_processed": round(processed / elapsed, 4), "completed": completed,
        }
        meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n",
                             encoding="utf-8")
        detections.close()
        frames.close()
        recorder.close(completed=completed)
    return meta["summary"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    try:
        summary = capture(args.config)
    except (CaptureError, DatasetValidationError, FileExistsError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
