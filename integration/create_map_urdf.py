#!/usr/bin/env python3
"""Record two RealSense cameras, then build the SLAM atlas and camera URDF."""
from __future__ import annotations

import argparse
import json
import math
import os
import select
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from camera_publish import (  # noqa: E402
    SAM_CAMERA,
    SLAM_CAMERA,
    role_contract,
    topics,
    wait_camera_frame,
    write_roles,
)
from camera_extrinsic_localization import ORB_WS, write_settings  # noqa: E402
from converting_launch.downgrade_bag_metadata import (  # noqa: E402
    TARGET_VERSION,
    TOPIC_KEYS,
    TOP_KEYS,
)
from run_object_memory_rosbag import (  # noqa: E402
    _conda_command,
    _request_shutdown,
    _stop,
    _wait_orb_ready,
)


SYSTEM_PYTHON = Path("/usr/bin/python3")


def _publisher_command(args: argparse.Namespace, roles_out: Path) -> list[str]:
    command = [str(SYSTEM_PYTHON), str(HERE / "camera_publish.py"),
               "--ros-domain-id", str(args.ros_domain_id),
               "--roles-out", str(roles_out),
               "--camera-timeout", str(args.camera_timeout)]
    if args.hardware_sync:
        command.append("--hardware-sync")
    return command


def _wait_roles(path: Path, process: subprocess.Popen) -> dict[str, str]:
    while not path.is_file():
        if process.poll() is not None:
            raise RuntimeError(f"camera publisher exited before role selection (rc={process.returncode})")
        time.sleep(0.1)
    selected = json.loads(path.read_text(encoding="utf-8"))
    return role_contract(selected["slam_serial"], selected["sam_serial"])


def _camera_info(camera: str, domain_id: int) -> dict:
    command = _conda_command("realsense", [
        "ros2", "topic", "echo", "--no-daemon", "--once",
        "--qos-reliability", "best_effort", topics(camera)["caminfo"],
        "sensor_msgs/msg/CameraInfo",
    ], domain_id=domain_id)
    message = subprocess.run(command, check=True, capture_output=True, text=True,
                             timeout=20).stdout.split("\n---", 1)[0]
    info = yaml.safe_load(message)
    if not isinstance(info, dict):
        raise ValueError(f"invalid CameraInfo on {topics(camera)['caminfo']}")
    camera_info = {
        "width": int(info["width"]),
        "height": int(info["height"]),
        "K": [float(value) for value in info["k"]],
        "D": [float(value) for value in info["d"]],
        "distortion_model": str(info["distortion_model"]),
    }
    if not _valid_camera_info(camera_info):
        raise ValueError(f"invalid CameraInfo on {topics(camera)['caminfo']}")
    return camera_info


def _record_topics() -> list[str]:
    return [topics(camera)[key]
            for camera in (SLAM_CAMERA, SAM_CAMERA)
            for key in ("rgb", "depth", "caminfo", "depth_caminfo")]


def _map_viewer_command(args: argparse.Namespace, dataset: Path,
                        camera_info: dict) -> list[str]:
    output = dataset / "live_map_preview"
    output.mkdir()
    settings = output / "orb_settings.yaml"
    config = output / "orb_config.yaml"
    info = {"SLAM": {
        "camera": camera_info,
        "color_hz": 30.0,
        "color_topic": topics(SLAM_CAMERA)["rgb"],
        "depth_topic": topics(SLAM_CAMERA)["depth"],
    }}
    write_settings(info, "SLAM", settings, "System.SaveAtlasToFile",
                   "live_map_preview", args.baseline)
    config.write_text(yaml.safe_dump({
        "dataset": f"{dataset.name}_live_map_preview",
        "bag": {"play": False, "clock": False, "rate": 1.0},
        "orbslam3": {
            "vocabulary_path": str(ORB_WS / "src/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
            "settings_path": str(settings),
        },
        "topics": {
            "rgb": topics(SLAM_CAMERA)["rgb"],
            "depth": topics(SLAM_CAMERA)["depth"],
        },
        "frames": {"world": "map", "camera": "slam_camera_color_optical_frame"},
        "runtime": {
            "enable_viewer": True,
            "localization_mode": False,
            "publish_map_points": False,
            "save_map_points_on_shutdown": False,
        },
        "dense_map": {"enabled": False},
        "output": {"dir": str(output), "map_points_path": "orbslam3_map_points.pcd"},
        "visualization": {"enabled": False},
    }, sort_keys=False), encoding="utf-8")
    return _conda_command(
        "orbslam3",
        ["ros2", "launch", "orbslam3_ros2", "orb_slam.launch.py", f"config:={config}"],
        ORB_WS, ORB_WS / "install/setup.bash", args.ros_domain_id)


def _metadata_payload(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(
            document.get("rosbag2_bagfile_information"), dict):
        raise ValueError(f"invalid rosbag metadata: {path}")
    return document["rosbag2_bagfile_information"]


def _valid_camera_info(info: dict) -> bool:
    try:
        values = [*info.get("K", []), *info.get("D", [])]
        return (int(info.get("width", 0)) > 0 and int(info.get("height", 0)) > 0
                and len(info.get("K", [])) == 9 and bool(info.get("D"))
                and all(math.isfinite(float(value)) for value in values))
    except (AttributeError, TypeError, ValueError):
        return False


def write_info(capture: Path, camera_infos: dict[str, dict], destination: Path,
               name: str, min_duration: float = 30.0) -> dict:
    metadata = _metadata_payload(capture / "metadata.yaml")
    duration_ns = int(metadata["duration"]["nanoseconds"])
    start_ns = int(metadata["starting_time"]["nanoseconds_since_epoch"])
    if duration_ns < min_duration * 1e9:
        raise ValueError(
            f"recorded rosbag is too short: {duration_ns / 1e9:.3f}s < {min_duration:.3f}s")
    counts = {
        entry["topic_metadata"]["name"]: int(entry["message_count"])
        for entry in metadata["topics_with_message_count"]
    }
    result = {
        "session": name,
        "generated_by": "integration/create_map_urdf.py",
        "generated_at": datetime.now().isoformat(timespec="seconds"),
    }
    duration_s = duration_ns / 1e9
    for role, camera in (("SLAM", SLAM_CAMERA), ("SAM", SAM_CAMERA)):
        expected = {topics(camera)[key] for key in
                    ("rgb", "depth", "caminfo", "depth_caminfo")}
        missing = sorted(expected - counts.keys())
        if missing:
            raise ValueError(f"recorded rosbag is missing {role} topics: {missing}")
        empty = sorted(topic for topic in expected if counts[topic] <= 0)
        if empty:
            raise ValueError(f"recorded rosbag has empty {role} topics: {empty}")
        if not _valid_camera_info(camera_infos[role]):
            raise ValueError(f"recorded rosbag has invalid {role} CameraInfo")
        color_count = counts[topics(camera)["rgb"]]
        result[role] = {
            "path": role,
            "topics": {topic: counts[topic] for topic in sorted(expected)},
            "duration_s": round(duration_s, 3),
            "t_start_ns": start_ns,
            "color_topic": topics(camera)["rgb"],
            "depth_topic": topics(camera)["depth"],
            "caminfo_topic": topics(camera)["caminfo"],
            "color_count": color_count,
            "color_hz": round(color_count / duration_s, 2),
            "camera": camera_infos[role],
        }
    destination.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def make_humble_metadata(capture: Path) -> None:
    metadata_path = capture / "metadata.yaml"
    data = _metadata_payload(metadata_path)
    if int(data.get("version", 0)) <= TARGET_VERSION:
        return
    backup = capture / "metadata.jazzy.yaml"
    if backup.exists() and backup.read_bytes() != metadata_path.read_bytes():
        raise RuntimeError(f"refusing to replace existing Jazzy metadata backup: {backup}")
    if not backup.exists():
        temporary_backup = capture / ".metadata.jazzy.yaml.tmp"
        temporary_backup.write_bytes(metadata_path.read_bytes())
        temporary_backup.replace(backup)
    compatible = {key: data[key] for key in TOP_KEYS if key in data}
    compatible["version"] = TARGET_VERSION
    compatible.setdefault("compression_format", "")
    compatible.setdefault("compression_mode", "")
    for entry in compatible.get("topics_with_message_count", []):
        topic = entry["topic_metadata"]
        entry["topic_metadata"] = {key: topic[key] for key in TOPIC_KEYS if key in topic}
        if not isinstance(entry["topic_metadata"].get("offered_qos_profiles"), str):
            entry["topic_metadata"]["offered_qos_profiles"] = ""
    compatible["relative_file_paths"] = [
        os.path.basename(path) for path in compatible.get("relative_file_paths", [])]
    for file_info in compatible.get("files", []):
        file_info["path"] = os.path.basename(file_info["path"])
    temporary = capture / ".metadata.yaml.tmp"
    temporary.write_text(yaml.safe_dump(
        {"rosbag2_bagfile_information": compatible}, sort_keys=False), encoding="utf-8")
    temporary.replace(metadata_path)


def _ensure_role_links(converted: Path, capture: Path) -> None:
    for role in ("SLAM", "SAM"):
        link = converted / role
        if link.is_symlink() and link.resolve() == capture.resolve():
            continue
        if link.exists() or link.is_symlink():
            raise RuntimeError(f"refusing to replace existing role bag: {link}")
        link.symlink_to(capture.name, target_is_directory=True)


def _validate_capture(capture: Path, info_path: Path, roles_path: Path,
                      min_duration: float) -> None:
    role_data = json.loads(roles_path.read_text(encoding="utf-8"))
    role_contract(role_data["slam_serial"], role_data["sam_serial"])
    info = json.loads(info_path.read_text(encoding="utf-8"))
    metadata = _metadata_payload(capture / "metadata.yaml")
    duration = int(metadata["duration"]["nanoseconds"]) / 1e9
    counts = {entry["topic_metadata"]["name"]: int(entry["message_count"])
              for entry in metadata["topics_with_message_count"]}
    missing = [topic for topic in _record_topics() if counts.get(topic, 0) <= 0]
    if duration < min_duration or missing:
        raise ValueError(
            f"existing capture is incomplete: duration={duration:.3f}s, topics={missing}")
    for role in ("SLAM", "SAM"):
        if role not in info or not _valid_camera_info(info[role].get("camera", {})):
            raise ValueError(f"existing info.json has invalid {role} CameraInfo")


def _record(args: argparse.Namespace, dataset: Path, converted: Path) -> None:
    roles_path = dataset / "camera_roles.pending.json"
    roles_path.unlink(missing_ok=True)
    publisher = subprocess.Popen(_publisher_command(args, roles_path), start_new_session=True)
    recorder = None
    map_viewer = None
    try:
        roles = _wait_roles(roles_path, publisher)
        camera_processes = [("camera publisher", publisher)]
        for camera in (SLAM_CAMERA, SAM_CAMERA):
            for key in ("rgb", "depth"):
                wait_camera_frame(topics(camera)[key], args.ros_domain_id,
                                  args.camera_timeout, camera_processes)
        camera_infos = {
            "SLAM": _camera_info(SLAM_CAMERA, args.ros_domain_id),
            "SAM": _camera_info(SAM_CAMERA, args.ros_domain_id),
        }
        if args.map_viewer:
            map_viewer = subprocess.Popen(
                _map_viewer_command(args, dataset, camera_infos["SLAM"]),
                start_new_session=True)
            camera_processes.append(("ORB map viewer", map_viewer))
            _wait_orb_ready(args.ros_domain_id, args.camera_timeout, map_viewer)
        capture = converted / "capture"
        command = _conda_command("sam6d", [
            "ros2", "bag", "record", "--storage", "sqlite3",
            "--output", str(capture), *_record_topics(),
        ], domain_id=args.ros_domain_id)
        recorder = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, start_new_session=True)
        started = time.monotonic()
        while time.monotonic() - started < args.min_duration:
            if recorder.poll() is not None:
                raise RuntimeError(f"rosbag recorder exited early (rc={recorder.returncode})")
            if publisher.poll() is not None:
                raise RuntimeError(f"camera publisher exited during recording (rc={publisher.returncode})")
            if map_viewer is not None and map_viewer.poll() is not None:
                raise RuntimeError(
                    f"ORB map viewer exited during recording (rc={map_viewer.returncode})")
            remaining = math.ceil(args.min_duration - (time.monotonic() - started))
            print(f"\r최소 녹화 시간까지 {remaining:3d}초", end="", flush=True)
            time.sleep(min(1.0, max(0.05, args.min_duration - (time.monotonic() - started))))
        print("\r최소 녹화 시간 충족. 맵 촬영이 끝났으면 Enter를 누르세요.   ", flush=True)
        while True:
            if recorder.poll() is not None:
                raise RuntimeError(f"rosbag recorder exited early (rc={recorder.returncode})")
            if publisher.poll() is not None:
                raise RuntimeError(
                    f"camera publisher exited during recording (rc={publisher.returncode})")
            if map_viewer is not None and map_viewer.poll() is not None:
                raise RuntimeError(
                    f"ORB map viewer exited during recording (rc={map_viewer.returncode})")
            readable, _writable, _errors = select.select([sys.stdin], [], [], 0.5)
            if not readable:
                continue
            if sys.stdin.readline() == "":
                raise RuntimeError("recording requires interactive Enter confirmation")
            break
        if not _stop(recorder):
            raise RuntimeError("rosbag recorder cleanup failed")
        recorder = None
        map_viewer_cleanup_failed = not _stop(map_viewer)
        map_viewer = None
        if not (capture / "metadata.yaml").is_file():
            raise RuntimeError(f"rosbag metadata was not finalized: {capture}")
        write_info(capture, camera_infos, converted / "info.json", dataset.name,
                   args.min_duration)
        make_humble_metadata(capture)
        _ensure_role_links(converted, capture)
        write_roles(dataset / "camera_roles.json", roles)
        roles_path.unlink(missing_ok=True)
        print(f"[record] {capture} ({roles['slam_serial']} / {roles['sam_serial']})", flush=True)
        if map_viewer_cleanup_failed:
            raise RuntimeError("ORB map viewer cleanup failed; capture was preserved")
    finally:
        stopped = (_stop(map_viewer), _stop(recorder), _stop(publisher))
        if not all(stopped):
            if sys.exc_info()[0] is None:
                raise RuntimeError("capture child process cleanup failed")
            print("WARNING: capture child process cleanup failed",
                  file=sys.stderr, flush=True)


def run(args: argparse.Namespace) -> None:
    dataset = (ROOT / "output" / args.name).resolve()
    converted = dataset / "converted"
    capture = converted / "capture"
    if dataset.exists() and not args.force:
        raise ValueError(f"refusing to replace existing dataset: {dataset} (--force to reuse it)")
    converted.mkdir(parents=True, exist_ok=True)
    if capture.exists():
        required = (dataset / "camera_roles.json", converted / "info.json",
                    capture / "metadata.yaml")
        if not args.force or not all(path.is_file() for path in required):
            raise ValueError(f"refusing to overwrite existing capture: {capture}")
        _validate_capture(capture, converted / "info.json",
                          dataset / "camera_roles.json", args.min_duration)
        make_humble_metadata(capture)
        _ensure_role_links(converted, capture)
        print(f"[reuse] 기존 capture 보존: {capture}", flush=True)
    else:
        _record(args, dataset, converted)

    command = [sys.executable, str(HERE / "camera_extrinsic_localization.py"),
               f"--inputs:={converted}",
               f"--outputs:={dataset / 'camera_extrinsic'}",
               f"--baseline:={args.baseline}"]
    if args.force:
        command.append("--force")
    subprocess.run(command, check=True)
    print(f"complete: {dataset}", flush=True)


def self_test() -> None:
    metadata = {
        "rosbag2_bagfile_information": {
            "version": 9,
            "storage_identifier": "sqlite3",
            "duration": {"nanoseconds": 31_000_000_000},
            "starting_time": {"nanoseconds_since_epoch": 123},
            "message_count": 80,
            "topics_with_message_count": [
                {"topic_metadata": {
                    "name": topic, "type": "sensor_msgs/msg/Image",
                    "serialization_format": "cdr", "offered_qos_profiles": []},
                 "message_count": 310}
                for topic in _record_topics()
            ],
            "relative_file_paths": ["capture_0.db3"],
            "files": [{"path": "capture_0.db3"}],
        }
    }
    camera = {"width": 640, "height": 480,
              "K": [1.0, 0.0, 2.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0],
              "D": [0.0] * 5, "distortion_model": "plumb_bob"}
    with tempfile.TemporaryDirectory() as directory:
        converted = Path(directory)
        capture = converted / "capture"
        capture.mkdir()
        (capture / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
        info = write_info(capture, {"SLAM": camera, "SAM": camera},
                          converted / "info.json", "demo")
        assert info["SLAM"]["color_hz"] == 10.0
        assert info["SAM"]["camera"]["width"] == 640
        try:
            write_info(capture, {"SLAM": camera, "SAM": camera},
                       converted / "short.json", "demo", min_duration=32)
        except ValueError:
            pass
        else:
            raise AssertionError("short capture accepted")
        make_humble_metadata(capture)
        assert _metadata_payload(capture / "metadata.yaml")["version"] == 5
        assert (capture / "metadata.jazzy.yaml").is_file()
        write_roles(converted / "camera_roles.json", {
            "slam_serial": "123", "sam_serial": "456"})
        _validate_capture(capture, converted / "info.json",
                          converted / "camera_roles.json", 30.0)
        _ensure_role_links(converted, capture)
        assert (converted / "SLAM").resolve() == capture.resolve()
        assert (converted / "SAM").resolve() == capture.resolve()
        dataset = converted / "viewer"
        dataset.mkdir()
        command = _map_viewer_command(
            argparse.Namespace(baseline=0.095, ros_domain_id=72), dataset, camera)
        config = yaml.safe_load(
            (dataset / "live_map_preview/orb_config.yaml").read_text(encoding="utf-8"))
        assert config["bag"]["rate"] == 1.0
        assert config["runtime"]["enable_viewer"] is True
        assert config["runtime"]["publish_map_points"] is False
        assert "ROS_DOMAIN_ID=72" in command[2]


def main() -> int:
    signal.signal(signal.SIGTERM, _request_shutdown)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _request_shutdown)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name")
    parser.add_argument("--min-duration", type=float, default=30.0)
    parser.add_argument("--baseline", type=float, default=0.095)
    parser.add_argument("--hardware-sync", action="store_true")
    parser.add_argument("--map-viewer", action="store_true",
                        help="신규 촬영 중 ORB-SLAM3 실시간 sparse-map viewer 표시")
    parser.add_argument("--ros-domain-id", type=int, default=72)
    parser.add_argument("--camera-timeout", type=float, default=30.0)
    parser.add_argument("--force", action="store_true",
                        help="기존 capture는 보존하고 Atlas/URDF만 다시 생성")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    self_test()
    if args.self_test:
        print("self-test: PASS")
        return 0
    if not args.name or Path(args.name).name != args.name or args.name in (".", ".."):
        parser.error("--name must be one dataset directory name")
    if not math.isfinite(args.min_duration) or args.min_duration < 30:
        parser.error("--min-duration must be at least 30 seconds")
    if not math.isfinite(args.baseline) or args.baseline <= 0:
        parser.error("--baseline must be positive")
    if not math.isfinite(args.camera_timeout) or args.camera_timeout <= 0:
        parser.error("--camera-timeout must be positive")
    try:
        run(args)
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError,
            subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
