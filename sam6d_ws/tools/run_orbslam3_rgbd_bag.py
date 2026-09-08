#!/usr/bin/env python3
"""Run the installed ORB-SLAM3 RGB-D node and attest its same-bag trajectory."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sqlite3
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

try:
    from tools.validate_rgbd_dataset import COLOR_TOPIC, validate_rgbd_dataset
except ModuleNotFoundError:
    from validate_rgbd_dataset import COLOR_TOPIC, validate_rgbd_dataset


REPO = Path(__file__).resolve().parents[1]
DEFAULT_BAG = REPO / "data" / "longcircle2_sam"
DEFAULT_SETTINGS = REPO / "configs" / "orbslam3_longcircle2_sam_rgbd.yaml"
DEFAULT_OUTPUT = REPO / "output" / "longcircle2_sam_self_slam"
DEFAULT_SETUP = REPO.parent / "orbslam_ws" / "install" / "setup.bash"
DEFAULT_VOCABULARY = REPO.parent / "orbslam_ws" / "src" / "ORB_SLAM3" / "Vocabulary" / "ORBvoc.txt"
OPTICAL_FRAME = "camera_color_optical_frame"


class RunError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def color_stamps(database: Path) -> np.ndarray:
    con = sqlite3.connect(f"file:{database.as_posix()}?mode=ro", uri=True)
    try:
        rows = con.execute(
            "SELECT m.timestamp FROM messages m JOIN topics t ON t.id=m.topic_id "
            "WHERE t.name=? ORDER BY m.timestamp", (COLOR_TOPIC,)).fetchall()
    finally:
        con.close()
    return np.asarray([row[0] for row in rows], dtype=np.int64) / 1e9


def trajectory_stats(path: Path, frame_stamps: np.ndarray, max_dt_s: float) -> dict:
    if not path.is_file() or path.stat().st_size <= 0:
        raise RunError(f"trajectory is missing or empty: {path}")
    try:
        values = np.loadtxt(path, ndmin=2)
    except (OSError, ValueError) as exc:
        raise RunError(f"trajectory cannot be parsed: {exc}") from exc
    if values.ndim != 2 or values.shape[1] != 8 or not np.isfinite(values).all():
        raise RunError("trajectory must be a finite TUM matrix with 8 columns")
    if len(values) < 2 or np.any(np.diff(values[:, 0]) <= 0):
        raise RunError("trajectory timestamps must be strictly increasing")
    quat_norm = np.linalg.norm(values[:, 4:8], axis=1)
    if np.any(quat_norm < 1e-9) or not np.allclose(quat_norm, 1.0, atol=1e-3, rtol=0.0):
        raise RunError("trajectory quaternions must be normalized and nonzero")

    times = values[:, 0]
    positions = np.searchsorted(times, frame_stamps)
    distances = np.full(len(frame_stamps), np.inf)
    for offset in (-1, 0):
        indices = positions + offset
        valid = (indices >= 0) & (indices < len(times))
        distances[valid] = np.minimum(
            distances[valid], np.abs(times[indices[valid]] - frame_stamps[valid]))
    matched = distances <= max_dt_s
    gaps = np.diff(times)
    return {
        "format": "TUM T_wc: timestamp tx ty tz qx qy qz qw",
        "pose_count": int(len(values)),
        "first_timestamp_s": float(times[0]),
        "last_timestamp_s": float(times[-1]),
        "strictly_monotonic": True,
        "finite": True,
        "quaternion_norm_max_error": float(np.max(np.abs(quat_norm - 1.0))),
        "bag_frame_count": int(len(frame_stamps)),
        "matched_frame_count": int(matched.sum()),
        "match_gate_ms": float(max_dt_s * 1000.0),
        "matched_frame_coverage": float(matched.mean()),
        "nearest_dt_median_ms": float(np.median(distances[matched]) * 1000.0)
        if matched.any() else None,
        "nearest_dt_max_ms": float(np.max(distances[matched]) * 1000.0)
        if matched.any() else None,
        "max_tracking_gap_s": float(gaps.max()) if len(gaps) else 0.0,
    }


def launch_config(args, output: Path) -> dict:
    return {
        "dataset": "longcircle2_sam",
        "bag": {"path": str(args.bag.resolve()), "play": True, "clock": True,
                "rate": args.rate, "start_delay": 12.0},
        "orbslam3": {"vocabulary_path": str(args.vocabulary.resolve()),
                     "settings_path": str(args.settings.resolve())},
        "topics": {"rgb": COLOR_TOPIC,
                   "depth": "/camera/camera/aligned_depth_to_color/image_raw"},
        "frames": {"world": "map", "camera": OPTICAL_FRAME},
        "runtime": {"sync_queue_size": 60, "publish_map_points": False,
                    "save_map_points_on_shutdown": False},
        "dense_map": {"enabled": False, "save_pcd": False, "save_ply": False},
        "output": {"dir": str(output.resolve()),
                   "map_points_path": "orbslam3_map_points.pcd"},
        "visualization": {"enabled": False},
    }


def require_file(path: Path, label: str) -> None:
    if not path.is_file() or path.stat().st_size <= 0:
        raise RunError(f"{label} is missing or empty: {path}")


def build_launch_command(ros_setup: Path, run_config: Path) -> list[str]:
    launch_shell = (
        "source \"$1\" && exec ros2 launch orbslam3_ros2 orb_slam.launch.py \"$2\"")
    return ["/bin/bash", "-lc", launch_shell, "orbslam3-run",
            str(ros_setup.resolve()), f"config:={run_config}"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    ap.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    ap.add_argument("--vocabulary", type=Path, default=DEFAULT_VOCABULARY)
    ap.add_argument("--ros-setup", type=Path, default=DEFAULT_SETUP)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--max-match-dt-ms", type=float, default=25.0)
    ap.add_argument("--min-coverage", type=float, default=0.50)
    ap.add_argument("--validate-only", action="store_true")
    args = ap.parse_args()

    if not math.isfinite(args.rate) or args.rate <= 0:
        ap.error("--rate must be finite and positive")
    if not math.isfinite(args.max_match_dt_ms) or args.max_match_dt_ms <= 0:
        ap.error("--max-match-dt-ms must be finite and positive")
    if not math.isfinite(args.min_coverage) or not 0 <= args.min_coverage <= 1:
        ap.error("--min-coverage must be between 0 and 1")
    for path, label in ((args.settings, "ORB settings"),
                        (args.vocabulary, "ORB vocabulary"),
                        (args.ros_setup, "ROS workspace setup")):
        require_file(path, label)

    validation = validate_rgbd_dataset(args.bag, require_manifest=True)
    if validation.dataset_id != "longcircle2_sam":
        raise RunError(f"refusing unexpected dataset: {validation.dataset_id}")
    if validation.manifest.get("camera", {}).get("optical_frame") != OPTICAL_FRAME:
        raise RunError("dataset optical frame does not match the ORB camera frame")

    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    trajectory = output / "CameraTrajectory.txt"
    run_config = output / "launch-config.yaml"
    log_path = output / "orbslam3.log"
    if not args.validate_only:
        if trajectory.exists():
            raise RunError(f"refusing to overwrite existing trajectory: {trajectory}")
        run_config.write_text(yaml.safe_dump(launch_config(args, output), sort_keys=False),
                              encoding="utf-8")
        command = build_launch_command(args.ros_setup, run_config)
        with log_path.open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=REPO, stdout=log,
                                    stderr=subprocess.STDOUT, check=False)
        if result.returncode != 0:
            raise RunError(f"ORB-SLAM3 failed with exit code {result.returncode}; see {log_path}")
    else:
        require_file(run_config, "saved launch config")
        try:
            saved_config = yaml.safe_load(run_config.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise RunError(f"saved launch config cannot be parsed: {exc}") from exc
        if saved_config != launch_config(args, output):
            raise RunError("saved launch config does not match current inputs")

    stats = trajectory_stats(
        trajectory, color_stamps(validation.database), args.max_match_dt_ms / 1000.0)
    status = "accepted" if stats["matched_frame_coverage"] >= args.min_coverage else "rejected"
    provenance = {
        "schema_version": 1,
        "kind": "same-camera ORB-SLAM3 trajectory",
        "status": status,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source": {
            "dataset_id": validation.dataset_id,
            "bag_database": validation.database.name,
            "bag_database_sha256": sha256(validation.database),
            "metadata_sha256": sha256(validation.bag / "metadata.yaml"),
            "pair_count": validation.pair_count,
            "optical_frame": OPTICAL_FRAME,
            "rgb_topic": COLOR_TOPIC,
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
        },
        "orbslam3": {
            "settings": args.settings.name,
            "settings_sha256": sha256(args.settings),
            "vocabulary": args.vocabulary.name,
            "vocabulary_sha256": sha256(args.vocabulary),
            "launch_config": run_config.name,
            "launch_config_sha256": sha256(run_config),
            "workspace_setup": str(args.ros_setup),
            "workspace_setup_sha256": sha256(args.ros_setup),
            "command": "ros2 launch orbslam3_ros2 orb_slam.launch.py "
                       f"config:={run_config}",
        },
        "trajectory": {**stats, "file": trajectory.name,
                       "sha256": sha256(trajectory)},
        "acceptance": {"minimum_matched_frame_coverage": args.min_coverage,
                       "status": status},
    }
    provenance_path = output / "provenance.json"
    provenance_path.write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")
    print(json.dumps(provenance, ensure_ascii=False, indent=2))
    if status != "accepted":
        raise RunError(
            f"trajectory coverage {stats['matched_frame_coverage']:.3f} is below "
            f"{args.min_coverage:.3f}")


if __name__ == "__main__":
    main()
