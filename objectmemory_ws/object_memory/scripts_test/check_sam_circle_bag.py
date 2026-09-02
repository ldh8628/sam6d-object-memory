#!/usr/bin/env python3
"""SAM_circle ros2 bag usability checker (read-only).

Inspects a real ros2 bag to decide whether it is usable as an RGB-D/IMU
source for object_memory development. It never copies bag contents and never
creates fake bag / dataset / SAM-output folders.

Verdict values (Korean, per spec):
    사용 가능      - RGB/depth/camera_info present AND SLAM/TF AND SAM output present
    부분 사용 가능  - RGB/depth/camera_info present, but SLAM pose or SAM output missing
    사용 불가      - folder/metadata missing, or timestamps/topics unreadable
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import sqlite3
import subprocess
import sys

DEFAULT_BAG_PATH = "~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle"

VERDICT_OK = "사용 가능"
VERDICT_PARTIAL = "부분 사용 가능"
VERDICT_UNUSABLE = "사용 불가"


# ---------------------------------------------------------------------------
# discovery helpers
# ---------------------------------------------------------------------------
def find_metadata(bag_path: str):
    candidates = [
        os.path.join(bag_path, "metadata.yaml"),
        os.path.join(bag_path, "bag", "metadata.yaml"),
    ]
    candidates.extend(sorted(glob.glob(os.path.join(bag_path, "*", "metadata.yaml"))))
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def find_db3(bag_dir: str):
    matches = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    return matches[0] if matches else None


def try_ros2_bag_info(bag_dir: str):
    if shutil.which("ros2") is None:
        return False, "ros2 not on PATH"
    try:
        proc = subprocess.run(
            ["ros2", "bag", "info", bag_dir],
            capture_output=True,
            text=True,
            timeout=60,
        )
        if proc.returncode == 0:
            return True, proc.stdout
        return False, proc.stderr or f"exit {proc.returncode}"
    except Exception as exc:  # pragma: no cover - environment dependent
        return False, str(exc)


# ---------------------------------------------------------------------------
# topic classification
# ---------------------------------------------------------------------------
def classify_topic(name: str) -> set:
    n = name.lower()
    tags = set()
    if "camera_info" in n:
        tags.add("camera_info")
    elif "depth" in n and "image" in n:
        tags.add("depth")
    elif ("color" in n or "rgb" in n) and "image" in n:
        tags.add("rgb")
    if "imu" in n:
        tags.add("imu")
    if name == "/tf":
        tags.add("tf")
    if name == "/tf_static":
        tags.add("tf_static")
    # SAM-6D / object pose output heuristics.
    if any(k in n for k in ("sam6d", "object_pose", "detection", "object_memory")):
        tags.add("sam_output")
    # SLAM pose heuristics.
    if any(k in n for k in ("slam", "/pose", "odom", "trajectory")):
        tags.add("slam_pose")
    return tags


# ---------------------------------------------------------------------------
# core check
# ---------------------------------------------------------------------------
def check_bag(bag_path: str, max_topic_samples: int = 5) -> dict:
    resolved_input = os.path.expanduser(bag_path)
    result = {
        "bag_path": bag_path,
        "resolved_input": resolved_input,
        "resolved_bag_dir": None,
        "folder_exists": False,
        "metadata_exists": False,
        "metadata_path": None,
        "ros2_bag_info_ok": False,
        "ros2_bag_info_detail": "",
        "verdict": VERDICT_UNUSABLE,
        "reasons": [],
        "topics": [],
        "has_rgb": False,
        "has_depth": False,
        "has_camera_info": False,
        "has_imu": False,
        "has_tf": False,
        "has_tf_static": False,
        "has_slam_pose": False,
        "has_sam_output": False,
        "created_fake_folders": False,
    }

    if not os.path.isdir(resolved_input):
        result["reasons"].append("bag folder does not exist")
        return result
    result["folder_exists"] = True

    metadata_path = find_metadata(resolved_input)
    if metadata_path is None:
        result["reasons"].append("metadata.yaml not found")
        return result
    result["metadata_exists"] = True
    result["metadata_path"] = metadata_path
    bag_dir = os.path.dirname(metadata_path)
    result["resolved_bag_dir"] = bag_dir

    info_ok, info_detail = try_ros2_bag_info(bag_dir)
    result["ros2_bag_info_ok"] = info_ok
    result["ros2_bag_info_detail"] = info_detail

    db3 = find_db3(bag_dir)
    if db3 is None:
        result["reasons"].append("no .db3 sqlite file found; cannot read timestamps")
        return result

    try:
        conn = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
        cur = conn.cursor()
        cur.execute("select id, name, type from topics")
        topic_rows = cur.fetchall()
        for topic_id, name, ttype in topic_rows:
            cur.execute(
                "select count(*), min(timestamp), max(timestamp) "
                "from messages where topic_id = ?",
                (topic_id,),
            )
            count, ts_min, ts_max = cur.fetchone()
            cur.execute(
                "select timestamp from messages where topic_id = ? "
                "order by timestamp limit ?",
                (topic_id, max_topic_samples),
            )
            samples = [row[0] for row in cur.fetchall()]
            tags = classify_topic(name)
            result["topics"].append(
                {
                    "id": topic_id,
                    "name": name,
                    "type": ttype,
                    "count": count or 0,
                    "ts_min": ts_min,
                    "ts_max": ts_max,
                    "ts_samples": samples,
                    "tags": sorted(tags),
                }
            )
        conn.close()
    except Exception as exc:
        result["reasons"].append(f"sqlite read failed: {exc}")
        return result

    for t in result["topics"]:
        has_data = (t["count"] or 0) > 0
        for tag in t["tags"]:
            if tag == "rgb" and has_data:
                result["has_rgb"] = True
            elif tag == "depth" and has_data:
                result["has_depth"] = True
            elif tag == "camera_info" and has_data:
                result["has_camera_info"] = True
            elif tag == "imu" and has_data:
                result["has_imu"] = True
            elif tag == "tf":
                result["has_tf"] = True
            elif tag == "tf_static":
                result["has_tf_static"] = True
            elif tag == "slam_pose" and has_data:
                result["has_slam_pose"] = True
            elif tag == "sam_output" and has_data:
                result["has_sam_output"] = True

    core_rgbd = result["has_rgb"] and result["has_depth"] and result["has_camera_info"]
    slam_available = result["has_tf"] or result["has_tf_static"] or result["has_slam_pose"]
    sam_available = result["has_sam_output"]

    if not core_rgbd:
        result["verdict"] = VERDICT_UNUSABLE
        missing = [
            n
            for n, present in (
                ("rgb", result["has_rgb"]),
                ("depth", result["has_depth"]),
                ("camera_info", result["has_camera_info"]),
            )
            if not present
        ]
        result["reasons"].append(f"missing core RGB-D topics: {missing}")
    elif slam_available and sam_available:
        result["verdict"] = VERDICT_OK
        result["reasons"].append(
            "RGB-D/camera_info present with SLAM/TF and SAM object output"
        )
    else:
        result["verdict"] = VERDICT_PARTIAL
        gaps = []
        if not slam_available:
            gaps.append("SLAM pose / TF")
        if not sam_available:
            gaps.append("SAM-6D object output")
        result["reasons"].append(
            "RGB-D/camera_info present, but missing: " + ", ".join(gaps)
        )

    return result


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------
def _fmt_bool(v: bool) -> str:
    return "yes" if v else "no"


def write_report(result: dict, output_report: str) -> None:
    out_dir = os.path.dirname(os.path.abspath(output_report))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    lines = []
    lines.append("# Story 0 SAM_circle Bag Usability Check")
    lines.append("")
    lines.append(f"- bag path: `{result['bag_path']}`")
    lines.append(f"- resolved bag dir: `{result['resolved_bag_dir']}`")
    lines.append(f"- folder exists: {_fmt_bool(result['folder_exists'])}")
    lines.append(f"- metadata exists: {_fmt_bool(result['metadata_exists'])}")
    lines.append(f"- ros2 bag info: {'ok' if result['ros2_bag_info_ok'] else 'failed/unavailable'}")
    lines.append(f"- **verdict: {result['verdict']}**")
    lines.append("")
    lines.append("## Reasons")
    for r in result["reasons"] or ["(none)"]:
        lines.append(f"- {r}")
    lines.append("")
    lines.append("## Source Topic Summary")
    lines.append("")
    lines.append(f"- RGB: {_fmt_bool(result['has_rgb'])}")
    lines.append(f"- depth: {_fmt_bool(result['has_depth'])}")
    lines.append(f"- camera_info: {_fmt_bool(result['has_camera_info'])}")
    lines.append(f"- IMU: {_fmt_bool(result['has_imu'])}")
    lines.append(f"- /tf: {_fmt_bool(result['has_tf'])}")
    lines.append(f"- /tf_static: {_fmt_bool(result['has_tf_static'])}")
    lines.append(f"- SLAM pose topic: {_fmt_bool(result['has_slam_pose'])}")
    lines.append(f"- SAM-6D object output: {_fmt_bool(result['has_sam_output'])}")
    lines.append("")
    lines.append("## Topic Table")
    lines.append("")
    if result["topics"]:
        lines.append("| name | type | count | ts_min | ts_max | ts_samples |")
        lines.append("| --- | --- | --- | --- | --- | --- |")
        for t in result["topics"]:
            samples = ", ".join(str(s) for s in t["ts_samples"])
            lines.append(
                f"| `{t['name']}` | {t['type']} | {t['count']} | "
                f"{t['ts_min']} | {t['ts_max']} | {samples} |"
            )
    else:
        lines.append("(no topics read)")
    lines.append("")
    lines.append("## Policy Check")
    lines.append("")
    lines.append(
        f"- fake folders created: {_fmt_bool(result['created_fake_folders'])} "
        "(no fake bag/dataset/SAM-output folders were created)"
    )
    lines.append("")

    with open(output_report, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="SAM_circle bag usability checker")
    parser.add_argument("--bag-path", default=DEFAULT_BAG_PATH)
    parser.add_argument(
        "--output-report",
        default="reports/story0_sam_circle_bag_check.md",
    )
    parser.add_argument("--max-topic-samples", type=int, default=5)
    args = parser.parse_args(argv)

    result = check_bag(args.bag_path, max_topic_samples=args.max_topic_samples)
    write_report(result, args.output_report)

    print(f"verdict: {result['verdict']}")
    for r in result["reasons"]:
        print(f"  - {r}")
    print(f"report: {args.output_report}")

    return 0 if result["verdict"] in (VERDICT_OK, VERDICT_PARTIAL) else 1


if __name__ == "__main__":
    sys.exit(main())
