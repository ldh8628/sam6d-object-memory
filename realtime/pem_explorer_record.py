#!/usr/bin/env python3
"""Compact, append-only recording contract for PEM Explorer v2.

The recorder deliberately stores no RGB, depth, rendered candidate image, or feature
tensor.  Those assets are replayed from the source bag by the localhost explorer.
"""
from __future__ import annotations

import hashlib
import json
import os
import struct
import time
from pathlib import Path

import numpy as np


SCHEMA = "pem-explorer-v2"
CANDIDATE_FIELDS = (
    "R00", "R01", "R02", "R10", "R11", "R12", "R20", "R21", "R22",
    "tx_m", "ty_m", "tz_m", "geometry", "mask_iou", "texture", "coverage",
    "size_ratio", "proposal6000", "index300", "rank_geo", "rank_texture", "flags",
)
# 17 float32 + uint32 + 4 uint16 = exactly 80 bytes.
CANDIDATE_STRUCT = struct.Struct("<17fI4H")
CANDIDATE_BYTES = CANDIDATE_STRUCT.size

FLAG_POSE_VALID = 1 << 0
FLAG_PROJECTION_VALID = 1 << 1
FLAG_MASK_PASS = 1 << 2
FLAG_TEXTURE_PASS = 1 << 3
FLAG_CLUSTER_MEMBER = 1 << 4
FLAG_SELECTED = 1 << 5


def _jsonable(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def sha256_file(path, chunk_size=4 * 1024 * 1024):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            block = stream.read(chunk_size)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def provenance_entry(path, hash_content=True):
    path = Path(path).resolve()
    entry = {"path": str(path), "exists": path.is_file() or path.is_dir()}
    if path.is_file():
        entry.update({"size": path.stat().st_size,
                      "sha256": sha256_file(path) if hash_content else None})
    elif path.is_dir():
        files = []
        for child in sorted(p for p in path.iterdir() if p.is_file()):
            files.append({"name": child.name, "size": child.stat().st_size,
                          "sha256": sha256_file(child) if hash_content else None})
        entry["files"] = files
    return entry


def pack_candidates(payload):
    """Return fixed-width candidate bytes without any numeric re-rounding."""
    rotations = np.asarray(payload["R"], dtype="<f4")
    translations = np.asarray(payload["t_m"], dtype="<f4")
    count = rotations.shape[0]
    if rotations.shape != (count, 3, 3) or translations.shape != (count, 3):
        raise ValueError("candidate R/t shape mismatch")
    scores = [np.asarray(payload.get(key, np.full(count, np.nan)), dtype="<f4")
              for key in ("geometry", "mask_iou", "texture", "coverage", "size_ratio")]
    integers = [np.asarray(payload.get(key), dtype=np.uint32 if key == "proposal6000"
                           else np.uint16)
                for key in ("proposal6000", "index300", "rank_geo", "rank_texture", "flags")]
    out = bytearray(count * CANDIDATE_BYTES)
    for i in range(count):
        values = [*rotations[i].reshape(-1), *translations[i],
                  *(score[i] for score in scores), int(integers[0][i]),
                  *(int(value[i]) for value in integers[1:])]
        CANDIDATE_STRUCT.pack_into(out, i * CANDIDATE_BYTES, *values)
    return bytes(out)


def unpack_candidates(blob):
    if len(blob) % CANDIDATE_BYTES:
        raise ValueError("truncated candidates.bin record")
    rows = []
    for values in CANDIDATE_STRUCT.iter_unpack(blob):
        rows.append({
            "R": np.asarray(values[:9], dtype="<f4").reshape(3, 3),
            "t_m": np.asarray(values[9:12], dtype="<f4"),
            "geometry": np.float32(values[12]), "mask_iou": np.float32(values[13]),
            "texture": np.float32(values[14]), "coverage": np.float32(values[15]),
            "size_ratio": np.float32(values[16]), "proposal6000": values[17],
            "index300": values[18], "rank_geo": values[19],
            "rank_texture": values[20], "flags": values[21],
        })
    return rows


def pack_replay(replay):
    source = np.asarray(replay["source_pixel_index"], dtype="<u4")
    fps = np.asarray(replay["coarse_fps_index"], dtype="<u2")
    valid = np.asarray(replay["coarse_valid"], dtype=np.uint8)
    cad = np.asarray(replay["cad_sample_index"], dtype="<u2")
    if fps.size != valid.size:
        raise ValueError("coarse FPS index/valid length mismatch")
    valid_bits = np.packbits(valid, bitorder="little")
    return source.tobytes() + fps.tobytes() + valid_bits.tobytes() + cad.tobytes()


def unpack_replay(blob, counts):
    n_source = int(counts["source"]); n_fps = int(counts["fps"]); n_cad = int(counts["cad"])
    n_valid_bytes = (n_fps + 7) // 8
    expected = n_source * 4 + n_fps * 2 + n_valid_bytes + n_cad * 2
    if len(blob) != expected:
        raise ValueError(f"replay record size {len(blob)} != {expected}")
    at = 0
    source = np.frombuffer(blob[at:at + n_source * 4], dtype="<u4").copy(); at += n_source * 4
    fps = np.frombuffer(blob[at:at + n_fps * 2], dtype="<u2").copy(); at += n_fps * 2
    valid = np.unpackbits(
        np.frombuffer(blob[at:at + n_valid_bytes], dtype=np.uint8),
        bitorder="little")[:n_fps].copy(); at += n_valid_bytes
    cad = np.frombuffer(blob[at:], dtype="<u2").copy()
    return {"source_pixel_index": source, "coarse_fps_index": fps,
            "coarse_valid": valid.astype(bool), "cad_sample_index": cad}


class ExplorerRecorder:
    """Append PEM attempts and atomically maintain an incomplete/complete manifest."""

    def __init__(self, run_dir, provenance, config=None):
        self.run_dir = Path(run_dir).resolve()
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.mask_dir = self.run_dir / "masks"
        self.mask_dir.mkdir(exist_ok=True)
        self.index_path = self.run_dir / "explorer_index.jsonl"
        self.candidate_path = self.run_dir / "candidates.bin"
        self.replay_path = self.run_dir / "replay.bin"
        self.manifest_path = self.run_dir / "explorer_manifest.json"
        existing = [path for path in (self.index_path, self.candidate_path,
                                      self.replay_path, self.manifest_path) if path.exists()]
        if existing:
            raise FileExistsError(
                "refusing to replace Explorer run files: "
                + ", ".join(path.name for path in existing))
        self._index = open(self.index_path, "xb")
        self._candidates = open(self.candidate_path, "xb")
        self._replay = open(self.replay_path, "xb")
        self._attempt = 0
        self._frames = set()
        self._last_stamp_ns = None
        self._max_bytes_per_attempt = int((config or {}).get(
            "max_bytes_per_attempt", 0) or 0)
        self._manifest = {
            "schema": SCHEMA, "schema_version": 2, "completed": False,
            "created_wall": time.time(), "completed_wall": None,
            "candidate_record_bytes": CANDIDATE_BYTES,
            "candidate_fields": list(CANDIDATE_FIELDS),
            "replay_layout": "source:u32,fps:u16,valid:packed-little-bits,cad:u16",
            "attempts": 0, "candidate_count": 0, "bytes_per_full_attempt": None,
            "provenance": provenance, "recording": dict(config or {}),
        }
        self._write_manifest()

    def _write_manifest(self):
        temp = self.manifest_path.with_suffix(".json.tmp")
        temp.write_text(json.dumps(self._manifest, indent=2, ensure_ascii=False,
                                   default=_jsonable) + "\n", encoding="utf-8")
        os.replace(temp, self.manifest_path)

    def record_frame(self, stamp_ns, depth_stamp_ns, frame_seq, K, image_shape, attempts,
                     label_image=None):
        self._last_stamp_ns = int(stamp_ns)
        mask_name = None
        frame_key = (int(frame_seq), int(stamp_ns))
        if attempts and label_image is not None and frame_key not in self._frames:
            import cv2
            mask_name = f"masks/{int(frame_seq)}_{int(stamp_ns)}.png"
            image = np.asarray(label_image)
            if image.dtype not in (np.uint8, np.uint16):
                raise ValueError("label image must be uint8 or uint16")
            if not cv2.imwrite(str(self.run_dir / mask_name), image):
                raise OSError(f"failed to write {mask_name}")
            self._frames.add(frame_key)
        elif attempts and (self.run_dir / "masks" /
                           f"{int(frame_seq)}_{int(stamp_ns)}.png").is_file():
            mask_name = f"masks/{int(frame_seq)}_{int(stamp_ns)}.png"

        for label, attempt in enumerate(attempts, start=1):
            payload = attempt.get("explorer_candidates")
            replay = attempt.get("explorer_replay")
            c_offset = self._candidates.tell(); r_offset = self._replay.tell()
            candidate_blob = pack_candidates(payload) if payload else b""
            replay_blob = pack_replay(replay) if replay else b""
            if (self._max_bytes_per_attempt and
                    len(candidate_blob) + len(replay_blob) > self._max_bytes_per_attempt):
                raise ValueError(
                    f"attempt exceeds max_bytes_per_attempt={self._max_bytes_per_attempt}")
            self._candidates.write(candidate_blob); self._replay.write(replay_blob)
            counts = ({"source": len(replay["source_pixel_index"]),
                       "fps": len(replay["coarse_fps_index"]),
                       "cad": len(replay["cad_sample_index"])} if replay else
                      {"source": 0, "fps": 0, "cad": 0})
            decision = dict(attempt.get("decision") or {})
            row = {
                "attempt_id": self._attempt, "stamp_ns": int(stamp_ns),
                "rgb_stamp_ns": int(stamp_ns),
                "depth_stamp_ns": int(depth_stamp_ns),
                "frame_seq": int(frame_seq), "object": attempt["object"],
                "bbox_xyxy": [int(v) for v in attempt.get("bbox", [])],
                "crop_bbox_yxyx": [float(v) for v in attempt.get("crop_bbox_yxyx", [])],
                "K": np.asarray(K, dtype=np.float32).reshape(-1).tolist(),
                "image_size": [int(image_shape[0]), int(image_shape[1])],
                "mask_asset": mask_name, "mask_label": label,
                "mask_bit": 1 << (label - 1),
                "candidate_offset": c_offset, "candidate_bytes": len(candidate_blob),
                "candidate_count": len(candidate_blob) // CANDIDATE_BYTES,
                "replay_offset": r_offset, "replay_bytes": len(replay_blob),
                "replay_counts": counts,
                "selected_index300": decision.get("selected_index300"),
                "selected_proposal6000": decision.get("selected_proposal6000_index"),
                "accepted": bool(decision.get("accepted", False)),
                "mask_survivors": int(decision.get("mask_survivors", 0) or 0),
                "texture_survivors": int(decision.get("texture_survivors", 0) or 0),
                "cluster_size": int(decision.get("cluster_size", 0) or 0),
                "cluster_occupancy": float(decision.get("cluster_occupancy", 0.0) or 0.0),
                "rejection_reason": (attempt.get("input_rejection") or
                                     decision.get("rejection_reason")),
            }
            self._index.write((json.dumps(row, ensure_ascii=False) + "\n").encode("utf-8"))
            self._attempt += 1
            self._manifest["candidate_count"] += row["candidate_count"]
            if row["candidate_count"] == 300:
                total = len(candidate_blob) + len(replay_blob)
                self._manifest["bytes_per_full_attempt"] = total
        self._manifest["attempts"] = self._attempt
        for stream in (self._index, self._candidates, self._replay):
            stream.flush()

    def close(self, completed=False):
        for stream in (self._index, self._candidates, self._replay):
            if not stream.closed:
                stream.flush(); os.fsync(stream.fileno()); stream.close()
        expected = self._manifest["recording"].get("expected_last_stamp_ns")
        tolerance = int(self._manifest["recording"].get("completion_tolerance_ns", 0) or 0)
        reached_end = (expected is None or (self._last_stamp_ns is not None and
                       self._last_stamp_ns + tolerance >= int(expected)))
        completed = bool(completed and reached_end)
        self._manifest["last_stamp_ns"] = self._last_stamp_ns
        self._manifest["completed"] = completed
        self._manifest["completion_reason"] = (
            "source_end_reached" if completed else
            "source_end_not_reached" if not reached_end else "interrupted_or_failed")
        self._manifest["completed_wall"] = time.time() if completed else None
        self._write_manifest()


def read_attempt(run_dir, row):
    run_dir = Path(run_dir)
    candidate_offset = int(row["candidate_offset"])
    candidate_bytes = int(row["candidate_bytes"])
    replay_offset = int(row["replay_offset"])
    replay_bytes = int(row["replay_bytes"])
    if (candidate_offset < 0 or replay_offset < 0 or candidate_bytes < 0 or
            replay_bytes < 0 or candidate_bytes % CANDIDATE_BYTES):
        raise ValueError("invalid binary offset or record size")
    if candidate_offset + candidate_bytes > (run_dir / "candidates.bin").stat().st_size:
        raise ValueError("candidate range exceeds candidates.bin")
    if replay_offset + replay_bytes > (run_dir / "replay.bin").stat().st_size:
        raise ValueError("replay range exceeds replay.bin")
    with open(run_dir / "candidates.bin", "rb") as stream:
        stream.seek(candidate_offset); candidates = unpack_candidates(
            stream.read(candidate_bytes))
    with open(run_dir / "replay.bin", "rb") as stream:
        stream.seek(replay_offset); replay = unpack_replay(
            stream.read(replay_bytes), row["replay_counts"])
    if len(candidates) != int(row.get("candidate_count", len(candidates))):
        raise ValueError("candidate count does not match binary range")
    return candidates, replay
