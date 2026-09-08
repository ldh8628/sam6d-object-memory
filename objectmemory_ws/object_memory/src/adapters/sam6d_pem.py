"""Load SAM-6D PEM per-frame 6D poses into Sam6DDetection records.

Two on-disk layouts are supported and auto-detected:

  nested (build_pem_inputs.py style):
    <pem_bag_dir>/frame_<idx>/pem_<Object>/sam6d_results/detection_pem.json

  flat (rgbd_imu_sdk_bag/<bag>/output/pem style, produced by the SAM-6D run):
    <pem_bag_dir>/frame_<idx>_<Object>.json

Both hold the same BOP list (R, t, score, bbox, ...). Each entry provides a 3x3
rotation R and a translation t (in millimetres). T_cam_obj is assembled from
(R, t/1000) so translations are in metres, matching the SLAM trajectory. The
object name and frame index come from the folder names (nested) or the file
name (flat).
"""

from __future__ import annotations

import glob
import json
import os
import re
from typing import Dict, List

from core.models import Sam6DDetection
from core.transforms import make_transform

_FRAME_RE = re.compile(r"frame_(\d+)$")
_PEM_RE = re.compile(r"pem_(.+)$")
# flat layout: frame_<idx>_<Object>.json  (object name may contain underscores)
_FLAT_RE = re.compile(r"frame_(\d+)_(.+)\.json$")


def _stamp_for(frame_idx: int, timestamps: List[float] = None) -> float:
    if timestamps is not None and 0 <= frame_idx < len(timestamps):
        return timestamps[frame_idx]
    return float(frame_idx)


def _load_detection_file(
    json_path: str,
    object_name: str,
    frame_idx: int,
    stamp: float,
    mm_to_m: bool,
) -> List[Sam6DDetection]:
    with open(json_path, "r", encoding="utf-8") as f:
        items = json.load(f)
    if not isinstance(items, list):
        items = [items]

    detections: List[Sam6DDetection] = []
    scale = 0.001 if mm_to_m else 1.0
    for local_id, it in enumerate(items):
        if "R" not in it or "t" not in it:
            continue  # ISM-only entry without a pose; skip.
        R = tuple(tuple(float(v) for v in row) for row in it["R"])
        t = tuple(float(v) * scale for v in it["t"])
        T_cam_obj = make_transform(R, t)
        # detection_id must be unique per frame; combine frame and local index.
        detection_id = frame_idx * 1000 + local_id
        detections.append(
            Sam6DDetection(
                stamp=stamp,
                frame_id="camera",
                detection_id=detection_id,
                object_name=object_name,
                T_cam_obj=T_cam_obj,
                score=float(it.get("score", 0.0)),
                bbox=tuple(it["bbox"]) if it.get("bbox") is not None else None,
                class_id=it.get("category_id"),
            )
        )
    return detections


def load_pem_detections(
    pem_bag_dir: str,
    timestamps: List[float] = None,
    mm_to_m: bool = True,
) -> Dict[int, List[Sam6DDetection]]:
    """Return {frame_idx: [Sam6DDetection, ...]} for one bag's PEM output tree.

    If ``timestamps`` (from bag_frame_index.load_frame_timestamps) is provided,
    each detection's stamp is set to the frame's bag timestamp; otherwise stamp
    is left as the frame index (float) so callers can fill it in later.
    """
    pem_bag_dir = os.path.expanduser(pem_bag_dir)
    if not os.path.isdir(pem_bag_dir):
        raise FileNotFoundError(f"pem dir not found: {pem_bag_dir}")

    out: Dict[int, List[Sam6DDetection]] = {}

    # --- nested layout: frame_<idx>/pem_<Object>/sam6d_results/detection_pem.json
    for frame_dir in sorted(glob.glob(os.path.join(pem_bag_dir, "frame_*"))):
        if not os.path.isdir(frame_dir):
            continue  # a flat frame_<idx>_<obj>.json file, handled below
        m = _FRAME_RE.search(os.path.basename(frame_dir))
        if not m:
            continue
        frame_idx = int(m.group(1))
        stamp = _stamp_for(frame_idx, timestamps)

        frame_dets: List[Sam6DDetection] = []
        for pem_dir in sorted(glob.glob(os.path.join(frame_dir, "pem_*"))):
            pm = _PEM_RE.search(os.path.basename(pem_dir))
            if not pm:
                continue
            object_name = pm.group(1)
            json_path = os.path.join(pem_dir, "sam6d_results", "detection_pem.json")
            if not os.path.isfile(json_path):
                continue
            frame_dets.extend(
                _load_detection_file(
                    json_path, object_name, frame_idx, stamp, mm_to_m
                )
            )
        if frame_dets:
            out.setdefault(frame_idx, []).extend(frame_dets)

    # --- flat layout: frame_<idx>_<Object>.json
    for json_path in sorted(glob.glob(os.path.join(pem_bag_dir, "frame_*_*.json"))):
        if not os.path.isfile(json_path):
            continue
        fm = _FLAT_RE.search(os.path.basename(json_path))
        if not fm:
            continue
        frame_idx = int(fm.group(1))
        object_name = fm.group(2)
        stamp = _stamp_for(frame_idx, timestamps)
        dets = _load_detection_file(json_path, object_name, frame_idx, stamp, mm_to_m)
        if dets:
            out.setdefault(frame_idx, []).extend(dets)

    return out
