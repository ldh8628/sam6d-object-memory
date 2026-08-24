#!/usr/bin/env python3
"""Localhost PEM Explorer server for compact and legacy SAM-6D reports."""
from __future__ import annotations

import argparse
import json
import math
import mimetypes
import sqlite3
import statistics
import sys
import threading
from collections import defaultdict, deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PurePosixPath
from urllib.parse import parse_qs, unquote, urlsplit

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
DATA_ROOT = (REPO / "data").resolve()
sys.path.insert(0, str(REPO / "realtime"))
from pem_explorer_record import (  # noqa: E402
    CANDIDATE_BYTES, FLAG_TEXTURE_MEASURED, SCHEMA, read_attempt,
)

STATIC = Path(__file__).resolve().parent / "pem_explorer_live"
DEFAULT_OBJECTS = [
    "milk", "choco_hazelnut_high", "Febreze_high", "Mugcup_high", "saffron",
    "Sauce_high", "Sikhye_high", "Bear", "Dinosaur",
]
POLICY = {
    "depth_valid_frac_min": 0.8,
    "texture_score_min": 0.449562,
    "mask_iou_min": 0.420998,
    "cluster_rotation_deg": 20.0,
    "cluster_translation_mm": 25.0,
    "correct_rotation_deg": 30.0,
    "correct_translation_mm": 100.0,
}
SYMMETRY_AXES = {
    "Sikhye_high": (0.0, 0.0, 1.0),
    "Sauce_high": (0.0, 0.0, 1.0),
    "Mugcup_high": (0.1057, 0.0337, 0.9938),
}


def json_safe(value):
    if hasattr(value, "tolist"):
        return json_safe(value.tolist())
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def safe_run(root, name):
    if not name or name in {".", ".."} or "/" in name or "\\" in name:
        raise ValueError("run must be one direct child of output root")
    root = Path(root).resolve()
    candidate = root / name
    if candidate.is_symlink():
        raise PermissionError("symlink runs are not served")
    resolved = candidate.resolve(strict=False)
    if resolved.parent != root:
        raise PermissionError("run escapes output root")
    return resolved


def classify_run(path):
    path = Path(path)
    manifest_path = path / "explorer_manifest.json"
    if manifest_path.is_file() and not manifest_path.is_symlink():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            return {"kind": "incomplete", "reason": f"invalid manifest: {exc}"}
        if (manifest.get("schema") != SCHEMA or manifest.get("schema_version") != 2 or
                manifest.get("candidate_record_bytes") != CANDIDATE_BYTES):
            return {"kind": "incomplete", "reason": "unsupported explorer schema"}
        required = ("explorer_index.jsonl", "candidates.bin", "replay.bin")
        if (not manifest.get("completed") or
                any(not (path / name).is_file() or (path / name).is_symlink()
                    for name in required)):
            return {"kind": "incomplete",
                    "reason": "run is not finalized; rerun the capture from the start",
                    "manifest": manifest}
        try:
            expected = int(manifest.get("candidate_count", -1)) * CANDIDATE_BYTES
        except (TypeError, ValueError):
            expected = -1
        if expected < 0 or (path / "candidates.bin").stat().st_size != expected:
            return {"kind": "incomplete", "reason": "candidate binary size mismatch",
                    "manifest": manifest}
        try:
            _validate_completed_v2(path, manifest)
        except (OSError, ValueError, KeyError, TypeError, PermissionError) as exc:
            return {"kind": "incomplete", "reason": f"invalid completed run: {exc}",
                    "manifest": manifest}
        return {"kind": "explorer_v2", "manifest": manifest}
    if (path / "detections.jsonl").is_file():
        return {"kind": "partial", "reason": "300개 후보 정보 미수집"}
    if (path / "index.html").is_file():
        return {"kind": "legacy", "url": f"/legacy/{path.name}/index.html"}
    return {"kind": "unknown", "reason": "recognized result files not found"}


def _run_priority(row):
    name = row["name"]
    if name == "longcircle2_visualization" and row["kind"] == "explorer_v2":
        return (0, name)
    if row["kind"] == "explorer_v2":
        return (1 if name == "longcircle2_manual" else 2, name)
    if row["kind"] == "legacy":
        return (3 if name == "pem_explorer" else 4, name)
    return (5, name)


def discover_runs(root):
    root = Path(root).resolve()
    if not root.is_dir():
        return []
    rows = []
    for path in root.iterdir():
        if path.is_symlink() or not path.is_dir():
            continue
        state = classify_run(path)
        if state["kind"] != "unknown":
            rows.append({"name": path.name, **state})
    return sorted(rows, key=_run_priority)


def _load_jsonl(path):
    if not path.is_file() or path.is_symlink():
        return []
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, 1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"{path.name}:{line_no} must contain an object")
            rows.append(value)
    return rows


def load_index(run_dir):
    rows = _load_jsonl(Path(run_dir) / "explorer_index.jsonl")
    for attempt, row in enumerate(rows):
        if int(row.get("attempt_id", -1)) != attempt:
            raise ValueError("explorer index attempt ids must be contiguous")
    return rows


def _validate_completed_v2(run_dir, manifest):
    """Validate append-only ranges and profile completion before exposing a run."""
    run_dir = Path(run_dir)
    rows = load_index(run_dir)
    if int(manifest.get("attempts", -1)) != len(rows):
        raise ValueError("manifest attempt count does not match index")
    candidate_size = (run_dir / "candidates.bin").stat().st_size
    replay_size = (run_dir / "replay.bin").stat().st_size
    candidate_end = replay_end = candidate_count = 0
    for row in rows:
        c_offset = int(row["candidate_offset"]); c_bytes = int(row["candidate_bytes"])
        r_offset = int(row["replay_offset"]); r_bytes = int(row["replay_bytes"])
        if c_offset != candidate_end or r_offset != replay_end:
            raise ValueError("binary ranges are not contiguous append-only records")
        if min(c_offset, c_bytes, r_offset, r_bytes) < 0 or c_bytes % CANDIDATE_BYTES:
            raise ValueError("invalid binary offset or candidate record size")
        count = int(row.get("candidate_count", -1))
        if count < 0 or count * CANDIDATE_BYTES != c_bytes:
            raise ValueError("index candidate count does not match its binary range")
        counts = row.get("replay_counts") or {}
        source = int(counts.get("source", -1)); fps = int(counts.get("fps", -1))
        cad = int(counts.get("cad", -1))
        if min(source, fps, cad) < 0:
            raise ValueError("invalid replay counts")
        expected_replay = source * 4 + fps * 2 + (fps + 7) // 8 + cad * 2
        if r_bytes != expected_replay:
            raise ValueError("index replay counts do not match its binary range")
        mask_asset = row.get("mask_asset")
        if mask_asset:
            relative = Path(mask_asset)
            target = run_dir / relative
            resolved = target.resolve(strict=False)
            if (relative.is_absolute() or ".." in relative.parts or target.is_symlink() or
                    not target.is_file() or run_dir.resolve() not in resolved.parents):
                raise PermissionError("unsafe or missing mask asset")
        candidate_end += c_bytes; replay_end += r_bytes; candidate_count += count
    if candidate_end != candidate_size or replay_end != replay_size:
        raise ValueError("binary file has missing or trailing records")
    if candidate_count != int(manifest.get("candidate_count", -1)):
        raise ValueError("manifest candidate count does not match index")

    profile = manifest.get("capture_profile")
    if profile not in {"exhaustive_visualization", "realtime_inference"}:
        return  # Backward-compatible v2 run created before capture profiles existed.
    for name in ("frames.jsonl", "detections.jsonl"):
        target = run_dir / name
        if target.is_symlink() or not target.is_file():
            raise ValueError(f"profile run is missing {name}")
    frames = _load_jsonl(run_dir / "frames.jsonl")
    _load_jsonl(run_dir / "detections.jsonl")
    if len(frames) != int(manifest.get("frames_processed", -1)):
        raise ValueError("processed frame count does not match frames.jsonl")
    stamps = [int(frame["stamp_ns"]) for frame in frames]
    sequences = [int(frame["frame_seq"]) for frame in frames]
    if sequences != list(range(len(frames))) or any(
            later <= earlier for earlier, later in zip(stamps, stamps[1:])):
        raise ValueError("processed frames are not contiguous and timestamp ordered")
    if stamps and (stamps[0] != int(manifest.get("first_stamp_ns", -1)) or
                   stamps[-1] != int(manifest.get("last_stamp_ns", -1))):
        raise ValueError("processed frame boundary stamps do not match manifest")
    if profile == "exhaustive_visualization":
        if len(frames) != int(manifest.get("bag_frame_count", -1)):
            raise ValueError("exhaustive frame count does not match bag frame count")
        for row in rows:
            count = int(row.get("candidate_count", 0))
            measured = int(row.get("texture_measured_count", 0))
            if count not in {0, 300} or (count and measured != count):
                raise ValueError("exhaustive attempt does not contain 300 measured textures")


def _contained_data_path(value):
    path = Path(value)
    if not path.is_absolute():
        path = REPO / path
    if path.is_symlink():
        raise PermissionError("symlink bag paths are not served")
    resolved = path.resolve()
    if resolved != DATA_ROOT and DATA_ROOT not in resolved.parents:
        raise PermissionError("source bag must remain under repository data/")
    if not resolved.exists():
        raise FileNotFoundError(resolved)
    return resolved


def _contained_repo_file(value, label):
    path = Path(value)
    if not path.is_absolute():
        path = REPO / path
    if path.is_symlink():
        raise PermissionError(f"symlink {label} paths are not served")
    resolved = path.resolve()
    if REPO.resolve() not in resolved.parents or not resolved.is_file():
        raise PermissionError(f"{label} must be a real file under the repository")
    return resolved


def _run_config(state):
    entry = state.get("manifest", {}).get("provenance", {}).get("config", {})
    raw = entry.get("path") if isinstance(entry, dict) else None
    if not raw:
        return {}
    path = _contained_repo_file(raw, "config provenance")
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _run_bag(state, cfg):
    manifest = state.get("manifest", {})
    raw = ((manifest.get("recording") or {}).get("source_bag") or
           (cfg.get("bag") or {}).get("path"))
    if not raw:
        entry = (manifest.get("provenance") or {}).get("bag") or {}
        raw = entry.get("path")
    if not raw:
        raise FileNotFoundError("source bag provenance is missing")
    return _contained_data_path(raw)


def _bag_database(bag):
    bag = Path(bag)
    files = sorted(bag.glob("*.db3")) if bag.is_dir() else [bag]
    if len(files) != 1 or files[0].is_symlink():
        raise ValueError("expected one real SQLite bag database")
    return files[0]


def _bag_stamps(bag, rgb_topic):
    db = _bag_database(bag)
    con = sqlite3.connect(f"file:{db.resolve().as_posix()}?mode=ro", uri=True)
    try:
        row = con.execute("SELECT id,type FROM topics WHERE name=?", (rgb_topic,)).fetchone()
        if row is None:
            raise ValueError(f"RGB topic missing from bag: {rgb_topic}")
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        message_class = get_message(row[1])
        stamps = []
        for (blob,) in con.execute(
                "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],)):
            message = deserialize_message(blob, message_class)
            stamps.append(int(message.header.stamp.sec) * 1_000_000_000 +
                          int(message.header.stamp.nanosec))
        if any(later <= earlier for earlier, later in zip(stamps, stamps[1:])):
            raise ValueError("RGB header stamps are not strictly increasing")
        return stamps
    finally:
        con.close()


def _object_names(cfg, attempts):
    requested = list((cfg.get("ism") or {}).get("objects") or [])
    if requested:
        return requested
    config = (cfg.get("ism") or {}).get("config")
    if config:
        path = _contained_repo_file(config, "ISM config")
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        names = [item.get("name") for item in raw.get("objects", [])
                 if item.get("enabled", True) and item.get("name") != "Rabbit"]
        if names:
            return names
    present = list(dict.fromkeys(row.get("object") for row in attempts if row.get("object")))
    return present or list(DEFAULT_OBJECTS)


def _axis_rotation(axis, degrees):
    """Return an object-frame Rodrigues rotation without adding a scipy dependency."""
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    angle = math.radians(degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    one_minus = 1.0 - cosine
    return np.asarray([
        [cosine + x * x * one_minus,
         x * y * one_minus - z * sine,
         x * z * one_minus + y * sine],
        [y * x * one_minus + z * sine,
         cosine + y * y * one_minus,
         y * z * one_minus - x * sine],
        [z * x * one_minus - y * sine,
         z * y * one_minus + x * sine,
         cosine + z * z * one_minus],
    ])


def _rotation_error(a, b, object_name=None):
    first = np.asarray(a, float)
    second = np.asarray(b, float)
    symmetries = (np.eye(3),)
    if object_name in SYMMETRY_AXES:
        symmetries = tuple(_axis_rotation(SYMMETRY_AXES[object_name], degrees)
                           for degrees in range(0, 360, 10))
    errors = []
    for symmetry in symmetries:
        trace = float(np.trace(first.T @ (second @ symmetry)))
        errors.append(math.degrees(math.acos(
            max(-1.0, min(1.0, (trace - 1.0) * 0.5)))))
    return min(errors)


def _valid_rotation(value):
    rotation = np.asarray(value, float)
    return (rotation.shape == (3, 3) and np.isfinite(rotation).all() and
            np.linalg.norm(rotation.T @ rotation - np.eye(3)) <= 1e-2 and
            np.linalg.det(rotation) > 0.99)


def _pose_distance(a, b, object_name=None):
    if (not _valid_rotation(a.get("R")) or not _valid_rotation(b.get("R")) or
            not np.isfinite(np.asarray(a.get("t_m"), float)).all() or
            not np.isfinite(np.asarray(b.get("t_m"), float)).all()):
        return math.inf, math.inf
    return (_rotation_error(a["R"], b["R"], object_name),
            float(np.linalg.norm(np.asarray(a["t_m"], float) -
                                 np.asarray(b["t_m"], float)) * 1000.0))


def _candidate_correct(candidate, reference, object_name=None):
    if not reference:
        return {"status": "unavailable", "correct": None}
    if (not _valid_rotation(candidate.get("R")) or
            not _valid_rotation(reference.get("R")) or
            not np.isfinite(np.asarray(candidate.get("t_m"), float)).all() or
            not np.isfinite(np.asarray(reference.get("t_mm"), float)).all()):
        return {"status": "invalid_pose", "correct": None}
    rot = _rotation_error(candidate["R"], reference["R"], object_name)
    trans = float(np.linalg.norm(np.asarray(candidate["t_m"], float) -
                                 np.asarray(reference["t_mm"], float) / 1000.0) * 1000.0)
    return {"status": "available", "rotation_error_deg": rot,
            "translation_error_mm": trans,
            "correct": rot <= POLICY["correct_rotation_deg"] and
                       trans <= POLICY["correct_translation_mm"]}


def _legacy_shadow(row, candidates, depth_valid_frac=None):
    top = next((c for c in candidates if int(c["rank_geo"]) == 0), None)
    result = {"policy": dict(POLICY), "geometry_top1": top, "status": "unavailable",
              "fallback": None, "excluded_similar_count": 0}
    if top is None:
        return result
    measured = bool(top.get("texture_measured"))
    top_projection_valid = bool(int(top.get("flags", 0)) & 2)
    top_scores_finite = all(math.isfinite(float(top.get(key, math.nan)))
                            for key in ("texture", "mask_iou"))
    depth_ok = (depth_valid_frac is not None and
                float(depth_valid_frac) >= POLICY["depth_valid_frac_min"])
    if (not measured or depth_valid_frac is None or not top_projection_valid or
            not top_scores_finite or not _valid_rotation(top.get("R"))):
        result["reason"] = "depth_texture_or_projection_unmeasured"
        return result
    withheld = (depth_ok and float(top["texture"]) < POLICY["texture_score_min"] and
                float(top["mask_iou"]) < POLICY["mask_iou_min"])
    result["status"] = "withheld" if withheld else "accepted"
    if not withheld:
        return result
    excluded = []
    for candidate in candidates:
        rot, trans = _pose_distance(top, candidate, row.get("object"))
        if (rot <= POLICY["cluster_rotation_deg"] and
                trans <= POLICY["cluster_translation_mm"]):
            excluded.append(candidate)
    result["excluded_similar_count"] = len(excluded)
    excluded_ids = {int(candidate["index300"]) for candidate in excluded}
    eligible = [candidate for candidate in candidates
                if int(candidate["index300"]) not in excluded_ids
                and candidate.get("texture_measured")
                and bool(int(candidate.get("flags", 0)) & 2)
                and _valid_rotation(candidate.get("R"))
                and math.isfinite(float(candidate.get("texture", math.nan)))
                and math.isfinite(float(candidate.get("mask_iou", math.nan)))
                and float(candidate["texture"]) >= POLICY["texture_score_min"]
                and float(candidate["mask_iou"]) >= POLICY["mask_iou_min"]]
    if eligible:
        result["fallback"] = min(eligible, key=lambda item: int(item["rank_geo"]))
    return result


def _attempt_depth_fraction(run_dir, row):
    stored = row.get("depth_valid_frac")
    if stored is not None:
        return float(stored)
    stamp = int(row["stamp_ns"])
    name = row.get("object")
    for frame in _load_jsonl(Path(run_dir) / "frames.jsonl"):
        if int(frame.get("stamp_ns", -1)) != stamp:
            continue
        for attempt in (frame.get("diagnostics") or {}).get("pem_candidates", []):
            if attempt.get("object") != name:
                continue
            mask = int(attempt.get("mask_px", 0) or 0)
            valid = int(attempt.get("valid_depth_px", 0) or 0)
            return float(valid) / float(mask) if mask > 0 else None
        break
    return None


class ExplorerServer(ThreadingHTTPServer):
    def __init__(self, address, root, no_model=False):
        raw_root = Path(root)
        if raw_root.is_symlink() or not raw_root.is_dir():
            raise ValueError("--root must be a real output directory, not a symlink")
        self.root = raw_root.resolve()
        self.no_model = no_model
        self.index_cache = {}
        self.report_cache = {}
        self.config_cache = {}
        self.frame_sources = {}
        self.analyzers = {}
        self.analyzer_errors = {}
        self.references = {}
        self.analysis_lock = threading.Lock()
        self.timings = defaultdict(lambda: deque(maxlen=200))
        super().__init__(address, Handler)

    def state(self, run):
        return classify_run(safe_run(self.root, run))

    def rows(self, run):
        if run not in self.index_cache:
            self.index_cache[run] = load_index(safe_run(self.root, run))
        return self.index_cache[run]

    def config(self, run, state):
        if run not in self.config_cache:
            self.config_cache[run] = _run_config(state)
        return self.config_cache[run]

    def bag_context(self, run, state):
        cfg = self.config(run, state)
        bag = _run_bag(state, cfg)
        topics = cfg.get("topics") or {
            "rgb": "/camera/camera/color/image_raw",
            "depth": "/camera/camera/aligned_depth_to_color/image_raw",
            "caminfo": "/camera/camera/color/camera_info",
        }
        return bag, topics

    def report(self, run, state):
        if run in self.report_cache:
            return self.report_cache[run]
        run_dir = safe_run(self.root, run)
        attempts = self.rows(run)
        cfg = self.config(run, state)
        bag, topics = self.bag_context(run, state)
        stamps = _bag_stamps(bag, topics["rgb"])
        processed = {int(row["stamp_ns"]): row for row in _load_jsonl(run_dir / "frames.jsonl")}
        detections = _load_jsonl(run_dir / "detections.jsonl")
        detection_map = {(int(row["stamp_ns"]), row.get("object")): row for row in detections}
        attempt_map = {(int(row["stamp_ns"]), row.get("object")): row for row in attempts}
        attempt_by_stamp = {}
        for attempt in attempts:
            attempt_by_stamp.setdefault(int(attempt["stamp_ns"]), attempt)
        objects = _object_names(cfg, attempts)
        profile = state["manifest"].get("capture_profile", "realtime_inference")
        frames = []
        status_counts = defaultdict(int)
        for source_index, stamp in enumerate(stamps):
            frame = processed.get(stamp)
            is_processed = frame is not None
            slots = []
            frame_status = ("processed_no_detection" if is_processed else
                            "unprocessed_realtime_drop" if profile == "realtime_inference"
                            else "unprocessed_capture_gap")
            for name in objects:
                attempt = attempt_map.get((stamp, name))
                detection = detection_map.get((stamp, name))
                if not is_processed:
                    status = frame_status
                elif attempt is not None:
                    status = "pem_passed" if attempt.get("accepted") else "pem_rejected"
                elif detection is not None:
                    status = "pem_passed"
                else:
                    status = "not_detected"
                if status == "pem_passed":
                    frame_status = "pem_passed"
                elif status == "pem_rejected" and frame_status != "pem_passed":
                    frame_status = "pem_rejected"
                slots.append({"object": name, "status": status,
                              "attempt_id": None if attempt is None else attempt["attempt_id"],
                              "accepted_pose": detection,
                              "rejection_reason": None if attempt is None else
                                                  attempt.get("rejection_reason")})
            status_counts[frame_status] += 1
            # Nanosecond ROS stamps exceed JavaScript's exact integer range. Keep the
            # wire representation decimal so frame URLs never round to a nearby stamp.
            frames.append({"index": source_index, "source_index": source_index,
                           "stamp_ns": str(stamp), "processed": is_processed,
                           "frame_seq": None if frame is None else frame.get("frame_seq"),
                           "status": frame_status, "n_boxes": 0 if frame is None else
                                     frame.get("n_boxes", 0),
                           "K": (attempt_by_stamp.get(stamp) or {}).get("K"),
                           "image_size": (attempt_by_stamp.get(stamp) or {}).get("image_size"),
                           "slots": slots})
        dataset_manifest = {}
        manifest_path = bag / "dataset_manifest.json" if bag.is_dir() else bag.parent / "dataset_manifest.json"
        if manifest_path.is_file() and not manifest_path.is_symlink():
            dataset_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        payload = {
            "schema_version": 1, "run": run, "kind": state["kind"],
            "capture_profile": profile, "completed": True,
            "dataset": {"id": dataset_manifest.get("dataset_id", bag.name),
                        "camera": dataset_manifest.get("camera", {}),
                        "bag_frame_count": len(stamps),
                        "processed_frame_count": len(processed)},
            "objects": objects, "frames": frames, "status_counts": dict(status_counts),
            "policy": dict(POLICY), "manifest": state["manifest"],
        }
        self.report_cache[run] = payload
        return payload

    def frame_source(self, run, state):
        if run not in self.frame_sources:
            from pem_explorer_live.analyzer import BagFrameCache
            bag, topics = self.bag_context(run, state)
            self.frame_sources[run] = BagFrameCache(bag, topics)
        return self.frame_sources[run]

    def reference(self, run, state, stamp_ns, name):
        key = (run, int(stamp_ns))
        if key not in self.references:
            try:
                from capture_pem_explorer import TrustedReferenceProvider
                from validate_rgbd_dataset import validate_rgbd_dataset
                bag, _ = self.bag_context(run, state)
                provider_key = (run, "provider")
                provider = self.references.get(provider_key)
                if provider is None:
                    provider = TrustedReferenceProvider(
                        validate_rgbd_dataset(bag, require_manifest=True))
                    self.references[provider_key] = provider
                self.references[key] = provider.for_stamp(stamp_ns)
            except Exception:
                self.references[key] = {}
        return self.references[key].get(name)

    def reference_role(self, run, state, stamp_ns, name):
        self.reference(run, state, stamp_ns, name)
        provider = self.references.get((run, "provider"))
        if provider is None or name not in provider.objects:
            return "gt_unavailable"
        members = {int(frame.get("stamp_ns"))
                   for frame in provider.objects[name].get("reference_frames", [])
                   if isinstance(frame, dict) and frame.get("stamp_ns") is not None}
        return ("trusted_reference_member" if int(stamp_ns) in members
                else "evaluation_frame")

    def analyzer_for(self, run, state):
        if self.no_model or run in self.analyzer_errors:
            return None
        if run not in self.analyzers:
            cfg = self.config(run, state)  # validates provenance before model loading
            config = state.get("manifest", {}).get("provenance", {}).get("config", {}).get("path")
            if not config:
                return None
            from pem_explorer_live.analyzer import PemLiveAnalyzer
            self.analyzers.clear()
            try:
                bag = _run_bag(state, cfg)
                self.analyzers[run] = PemLiveAnalyzer(REPO, config,
                                                       state["manifest"].get("provenance"),
                                                       bag_path=bag)
            except Exception as exc:
                self.analyzer_errors[run] = f"{type(exc).__name__}: {exc}"
                return None
        return self.analyzers[run]


class Handler(BaseHTTPRequestHandler):
    server: ExplorerServer

    def _json(self, payload, status=200):
        blob = json.dumps(json_safe(payload), ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(blob)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(blob)

    def _error(self, status, message):
        self._json({"error": message}, status)

    def do_GET(self):
        host = self.headers.get("Host", "")
        if urlsplit("//" + host).hostname not in {"127.0.0.1", "localhost", "::1"}:
            return self._error(HTTPStatus.FORBIDDEN, "localhost Host required")
        origin = self.headers.get("Origin")
        if origin and urlsplit(origin).hostname not in {"127.0.0.1", "localhost", "::1"}:
            return self._error(HTTPStatus.FORBIDDEN, "cross-origin request refused")
        try:
            self._get()
        except ValueError as exc:
            self._error(HTTPStatus.BAD_REQUEST, str(exc))
        except PermissionError as exc:
            self._error(HTTPStatus.FORBIDDEN, str(exc))
        except FileNotFoundError as exc:
            self._error(HTTPStatus.NOT_FOUND, str(exc))
        except (IndexError, KeyError) as exc:
            self._error(HTTPStatus.BAD_REQUEST, str(exc))
        except Exception as exc:
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, f"{type(exc).__name__}: {exc}")

    def _get(self):
        parsed = urlsplit(self.path); path = unquote(parsed.path)
        if path == "/api/runs":
            return self._json({"runs": discover_runs(self.server.root),
                               "analysis_enabled": not self.server.no_model})
        if path.startswith("/api/run/"):
            parts = PurePosixPath(path).parts
            if len(parts) < 4:
                raise ValueError("missing run")
            run = parts[3]; run_dir = safe_run(self.server.root, run)
            state = classify_run(run_dir)
            action = parts[4] if len(parts) > 4 else "summary"
            if action == "summary":
                payload = {"name": run, **state}
                if state["kind"] == "explorer_v2":
                    payload["attempts"] = self.server.rows(run)
                elif state["kind"] == "partial":
                    payload["detections"] = _load_jsonl(run_dir / "detections.jsonl")
                return self._json(payload)
            if state["kind"] != "explorer_v2":
                return self._error(HTTPStatus.CONFLICT,
                                   state.get("reason", "analysis unavailable"))
            if action == "report":
                return self._json(self.server.report(run, state))
            query = parse_qs(parsed.query)
            if action == "frame":
                stamp = int(query.get("stamp_ns", ["-1"])[0])
                valid = {int(row["stamp_ns"]) for row in self.server.report(run, state)["frames"]}
                if stamp not in valid:
                    raise ValueError("stamp is not a frame in this run's source bag")
                rgb = self.server.frame_source(run, state).get_rgb(stamp)
                import cv2
                ok, encoded = cv2.imencode(".jpg", rgb[:, :, ::-1],
                                           [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not ok:
                    raise OSError("JPEG encoding failed")
                return self._bytes(encoded.tobytes(), "image/jpeg")
            if action in {"candidates", "analysis"}:
                # Stored evidence is meaningful only with the attested source bag;
                # validate indirect config/provenance paths before reading an attempt.
                self.server.bag_context(run, state)
            attempt_id = int(query.get("attempt", ["-1"])[0])
            rows = self.server.rows(run)
            if attempt_id < 0 or attempt_id >= len(rows):
                raise ValueError("attempt id out of range")
            row = rows[attempt_id]
            candidates, replay = read_attempt(run_dir, row)
            legacy_record = "capture_profile" not in state["manifest"]
            for candidate in candidates:
                measured = (bool(int(candidate["flags"]) & FLAG_TEXTURE_MEASURED)
                            if not legacy_record else math.isfinite(float(candidate["texture"])))
                candidate["texture_measured"] = measured
                candidate["selected_actual"] = (
                    row.get("selected_index300") is not None and
                    int(candidate["index300"]) == int(row["selected_index300"]))
                candidate["geometry_top1"] = int(candidate["rank_geo"]) == 0
                candidate["gt"] = _candidate_correct(
                    candidate, self.server.reference(
                        run, state, row["stamp_ns"], row["object"]), row["object"])
            candidates.sort(key=lambda item: int(item["rank_geo"]))
            if action == "candidates":
                depth = _attempt_depth_fraction(run_dir, row)
                shadow = _legacy_shadow(row, candidates, depth)
                shadow["depth_valid_frac"] = depth
                return self._json({"attempt": row, "candidates": candidates,
                                   "actual": {"final_pose": row.get("final_pose"),
                                              "selected_index300": row.get("selected_index300"),
                                              "accepted": row.get("accepted"),
                                              "rejection_reason": row.get("rejection_reason")},
                                   "legacy_shadow": shadow, "policy": POLICY,
                                   "reference_role": self.server.reference_role(
                                       run, state, row["stamp_ns"], row["object"])})
            if action == "analysis":
                index = int(query.get("candidate", ["-1"])[0])
                candidate = next((item for item in candidates
                                  if int(item["index300"]) == index), None)
                if candidate is None:
                    raise ValueError("candidate index not found")
                with self.server.analysis_lock:
                    analyzer = self.server.analyzer_for(run, state)
                    if analyzer is None:
                        detail = self.server.analyzer_errors.get(
                            run, "dynamic analysis disabled (--no-model or invalid provenance)")
                        return self._error(HTTPStatus.SERVICE_UNAVAILABLE, detail)
                    result = analyzer.analyze(run_dir, row, candidate, replay)
                category = "warm" if result.get("feature_cache_hit") else "cold"
                self.server.timings[(row["object"], category)].append(result["timing_ms"])
                summaries = {}
                for kind in ("cold", "warm"):
                    ordered = sorted(self.server.timings[(row["object"], kind)])
                    if ordered:
                        summaries[kind] = {"samples": len(ordered),
                                           "p50_ms": statistics.median(ordered),
                                           "p90_ms": ordered[min(len(ordered) - 1,
                                                                  int(0.9 * len(ordered)))]}
                result["timing_summary"] = summaries
                result["texture_measured_at_capture"] = candidate["texture_measured"]
                return self._json(result)
            raise ValueError("unknown API action")
        if path.startswith("/legacy/"):
            parts = PurePosixPath(path).parts
            if len(parts) < 4:
                raise ValueError("legacy asset missing")
            run_dir = safe_run(self.server.root, parts[2])
            target = run_dir.joinpath(*parts[3:])
            resolved = target.resolve(strict=False)
            if target.is_symlink() or not target.is_file() or run_dir.resolve() not in resolved.parents:
                raise PermissionError("unsafe legacy asset")
            return self._file(target)
        assets = {
            "/": "index.html", "/index.html": "index.html",
            "/explorer.html": "explorer.html", "/app.js": "app.js",
            "/app.css": "app.css", "/worker.js": "worker.js",
            "/landing.js": "landing.js", "/landing.css": "landing.css",
        }
        if path in assets:
            return self._file(STATIC / assets[path])
        raise FileNotFoundError(path)

    def _bytes(self, blob, mime):
        self.send_response(200); self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(blob)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers(); self.wfile.write(blob)

    def _file(self, path):
        blob = Path(path).read_bytes()
        mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
        self.send_response(200); self.send_header("Content-Type", mime)
        self.send_header("X-Content-Type-Options", "nosniff")
        if mime == "text/html":
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self'; "
                "img-src 'self' data: blob:; connect-src 'self'; worker-src 'self'; "
                "frame-src 'self'")
        self.send_header("Content-Length", str(len(blob)))
        self.end_headers(); self.wfile.write(blob)

    def log_message(self, fmt, *args):
        sys.stderr.write("[pem-explorer] " + fmt % args + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default="output")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-model", action="store_true")
    args = parser.parse_args()
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        parser.error("PEM Explorer is localhost-only")
    server = ExplorerServer((args.host, args.port), args.root, args.no_model)
    print(f"PEM Explorer: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
