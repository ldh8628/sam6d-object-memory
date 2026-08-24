#!/usr/bin/env python3
"""Apply learned static SLAM anchors to later SAM-6D detections."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "realtime"))
sys.path.insert(0, str(REPO / "tools"))

from evaluate_slam_pose import load_jsonl, load_trajectory, nearest_pose  # noqa: E402
from slam_pose_memory import StaticSlamPoseMemory  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--memory", required=True)
    parser.add_argument("--detections", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    payload = json.loads(Path(args.memory).read_text(encoding="utf-8"))
    memory = StaticSlamPoseMemory(payload["anchors"])
    initialized = payload["initialized"]
    timestamps, trajectory = load_trajectory(args.trajectory)
    output = []
    for row in load_jsonl(args.detections):
        name = row["object"]
        init = initialized.get(name)
        if init is None or int(row["stamp_ns"]) <= int(init["stamp_ns"]):
            continue
        twc, dt = nearest_pose(int(row["stamp_ns"]), timestamps, trajectory)
        if twc is None:
            continue
        tco = memory.camera_pose(name, twc)
        clean = dict(row)
        if clean.get("verify"):
            clean["verify"] = {key: value for key, value in clean["verify"].items()
                               if key != "cands"}
        output.append({
            **clean,
            "R": tco[:3, :3].tolist(),
            "t_mm": (tco[:3, 3] * 1000.0).tolist(),
            "raw_R": row["R"],
            "raw_t_mm": row["t_mm"],
            "stabilization": {
                "mode": "static_slam_pose_memory",
                "anchor_initialized_stamp_ns": int(init["stamp_ns"]),
                "slam_dt_ms": round(float(dt * 1000.0), 3),
            },
        })
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in output),
                   encoding="utf-8")
    print(f"wrote {len(output)} stabilized poses -> {out}")


if __name__ == "__main__":
    main()
