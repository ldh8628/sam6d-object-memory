#!/usr/bin/env python3
"""Compare ism_associated and fov_always runs made from the same recording."""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def _jsonl(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()]


def _rotation_delta(a, b):
    relative = np.asarray(a, float).T @ np.asarray(b, float)
    return math.degrees(math.acos(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)))


def _metrics(frames, detections):
    by_object = defaultdict(list)
    for row in detections:
        by_object[row["object"]].append(row)
    rotation_steps, translation_steps = [], []
    for values in by_object.values():
        values.sort(key=lambda row: (row.get("stamp_ns", 0), row.get("frame_seq", 0)))
        for previous, current in zip(values, values[1:]):
            rotation_steps.append(_rotation_delta(previous["R"], current["R"]))
            translation_steps.append(float(np.linalg.norm(
                np.asarray(previous["t_mm"], float) - np.asarray(current["t_mm"], float))))
    labelled = [row for row in detections if isinstance(row.get("correct"), bool)]
    covered_frames = {row.get("frame_seq") for row in detections}
    anchor_rows = [row for row in detections if row.get("pose_source") == "slam_anchor"]
    recovery = [row for row in anchor_rows
                if (row.get("anchor_diagnostic") or {}).get("ism_present") is False]
    return {
        "frame_count": len(frames),
        "output_count": len(detections),
        "output_coverage": len(covered_frames) / max(len(frames), 1),
        "anchor_output_count": len(anchor_rows),
        "ism_missing_recovery_count": len(recovery),
        "labelled_output_count": len(labelled),
        "output_accuracy": (sum(row["correct"] for row in labelled) / len(labelled)
                            if labelled else None),
        "incorrect_output_count": (sum(not row["correct"] for row in labelled)
                                   if labelled else None),
        "pose_stability": {
            "rotation_step_median_deg": (float(np.median(rotation_steps))
                                         if rotation_steps else None),
            "translation_step_median_mm": (float(np.median(translation_steps))
                                            if translation_steps else None),
        },
    }


def summarize(run_dir):
    run_dir = Path(run_dir)
    frames = _jsonl(run_dir / "frames.jsonl")
    detections = _jsonl(run_dir / "detections.jsonl")
    result = _metrics(frames, detections)
    object_ids = sorted({row["object"] for row in detections})
    result["by_object"] = {
        object_id: _metrics(frames, [row for row in detections
                                    if row["object"] == object_id])
        for object_id in object_ids
    }
    return result


def _run_identity(run_dir):
    path = Path(run_dir) / "run_meta.json"
    if not path.exists():
        return None
    meta = json.loads(path.read_text(encoding="utf-8"))
    cfg = meta.get("config") or {}
    bag = cfg.get("bag") or {}
    topics = cfg.get("topics") or {}
    return {
        "objects": meta.get("objects"),
        "bag_path": bag.get("path"),
        "bag_rate": bag.get("rate"),
        "rgb_topic": topics.get("rgb"),
        "depth_topic": topics.get("depth"),
    }


def compare(ism_associated_dir, fov_always_dir):
    associated_identity = _run_identity(ism_associated_dir)
    always_identity = _run_identity(fov_always_dir)
    if ((associated_identity is None) != (always_identity is None)
            or (associated_identity is not None
                and associated_identity != always_identity)):
        raise ValueError("A/B runs do not describe the same recording/input configuration")
    return {"ism_associated": summarize(ism_associated_dir),
            "fov_always": summarize(fov_always_dir)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ism-associated", required=True)
    parser.add_argument("--fov-always", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    result = compare(args.ism_associated, args.fov_always)
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
