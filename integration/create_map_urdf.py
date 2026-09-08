#!/usr/bin/env python3
"""Record two RealSense cameras, then build the SLAM atlas and camera URDF."""
from __future__ import annotations

import argparse
import json
import math
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
    runtime_domain,
    topics,
    wait_camera_frame,
    write_roles,
)
from camera_extrinsic_localization import (  # noqa: E402
    ORB_ENV,
    ORB_INSTALL,
    ORB_WS,
    write_settings,
)
from run_object_memory_rosbag import (  # noqa: E402
    _conda_command,
    _request_shutdown,
    _stop,
    _wait_orb_ready,
)


from rgbd_capture import recorder_args
from input_check import InputCheckSession, ensure_camera_baseline


SYSTEM_PYTHON = Path("/usr/bin/python3")


def _publisher_command(args: argparse.Namespace, roles_out: Path) -> list[str]:
    command = [str(SYSTEM_PYTHON), str(HERE / "camera_publish.py"),
               "--ros-domain-id", str(args.ros_domain_id),
               "--roles-out", str(roles_out),
               "--camera-timeout", str(args.camera_timeout)]
    if args.hardware_sync:
        command.append("--hardware-sync")
    if getattr(args, "slam_serial", None):
        command += ["--slam-serial", args.slam_serial, "--sam-serial", args.sam_serial]
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
                        camera_info: dict, serial: str = "") -> list[str]:
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
            "rgbd": topics(SLAM_CAMERA)["rgbd"],
            "rgb": topics(SLAM_CAMERA)["rgb"],
            "depth": topics(SLAM_CAMERA)["depth"],
        },
        "frames": {"world": "map", "camera": "slam_camera_color_optical_frame"},
        "runtime": {
            "enable_viewer": True,
            "input_camera": SLAM_CAMERA, "input_serial": serial,
            "input_queue_size": 60, "input_qos_depth": 120,
            "localization_mode": False,
            "publish_map_points": False,
            "save_map_points_on_shutdown": False,
        },
        "dense_map": {"enabled": False},
        "output": {"dir": str(output), "map_points_path": "orbslam3_map_points.pcd"},
        "visualization": {"enabled": False},
    }, sort_keys=False), encoding="utf-8")
    return _conda_command(
        ORB_ENV,
        ["ros2", "launch", "orbslam3_ros2", "orb_slam.launch.py", f"config:={config}"],
        ORB_WS, ORB_INSTALL / "setup.bash", args.ros_domain_id)


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
    input_only = args.input_check_only
    roles_path = dataset / "camera_roles.pending.json"
    roles_path.unlink(missing_ok=True)
    publisher = subprocess.Popen(_publisher_command(args, roles_path), start_new_session=True)
    recorder = map_viewer = input_session = None
    error = None
    camera_infos = None
    raw_capture = None if input_only else converted / "capture_raw"
    try:
        roles = _wait_roles(roles_path, publisher)
        if input_only:
            write_roles(dataset / "camera_roles.json", roles)
            roles_path.unlink(missing_ok=True)
        camera_processes = [("camera publisher", publisher)]
        if not input_only:
            for camera in (SLAM_CAMERA, SAM_CAMERA):
                wait_camera_frame(topics(camera)["rgbd"], args.ros_domain_id,
                                  args.camera_timeout, camera_processes)
            camera_infos = {"SLAM": _camera_info(SLAM_CAMERA, args.ros_domain_id),
                            "SAM": _camera_info(SAM_CAMERA, args.ros_domain_id)}
        baseline = args.input_baseline
        if not input_only:
            baseline = ensure_camera_baseline(baseline, dataset, args.ros_domain_id,
                {SLAM_CAMERA:roles["slam_serial"], SAM_CAMERA:roles["sam_serial"]},
                camera_processes, args.camera_timeout, settings={"view":False,"hardware_sync":args.hardware_sync})
        input_session = InputCheckSession(dataset, args.ros_domain_id,
            {SLAM_CAMERA:roles["slam_serial"], SAM_CAMERA:roles["sam_serial"]},
            seconds=args.input_check_seconds, record=not input_only, capture=raw_capture,
            consumers={SLAM_CAMERA:["orb"]} if args.map_viewer and not input_only else {},
            baseline=baseline, is_baseline=input_only and args.input_baseline is None,
            compatibility=not input_only,
            settings={"view":args.map_viewer and not input_only,"hardware_sync":args.hardware_sync})
        if args.map_viewer and not input_only:
            map_viewer = subprocess.Popen(
                _map_viewer_command(args, dataset, camera_infos["SLAM"], roles["slam_serial"]),
                start_new_session=True)
            camera_processes.append(("ORB map viewer", map_viewer))
            _wait_orb_ready(args.ros_domain_id, args.camera_timeout, map_viewer)
        if not input_only:
            recorder = subprocess.Popen(_conda_command("sam6d", recorder_args(raw_capture),
                domain_id=args.ros_domain_id), stdin=subprocess.DEVNULL, start_new_session=True)
            camera_processes.append(("recorder", recorder))
        input_session.wait_ready(camera_processes, args.camera_timeout)
        input_session.arm()
        started = time.monotonic()
        print("[input] 준비 10초 후 카메라 입력만 검사" if input_only else
              "[record] 준비 10초 후 입력 검사 시작; 원본 RGBD MCAP 기록 중", flush=True)
        prompted = False
        while not input_session.done():
            failed = [(name,p.returncode) for name,p in camera_processes if p.poll() is not None]
            if failed:
                raise RuntimeError(f"recording process exited: {failed}")
            if not args.input_check_seconds and time.monotonic()-started >= 10+args.min_duration:
                if not prompted:
                    print("최소 녹화 시간 충족. 맵 촬영이 끝났으면 Enter를 누르세요.", flush=True)
                    prompted = True
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if readable:
                    if sys.stdin.readline() == "":
                        raise RuntimeError("recording requires interactive Enter confirmation")
                    break
            else:
                time.sleep(.1)
    except BaseException as exc:
        error = repr(exc)
        raise
    finally:
        if input_session:
            input_session.close_window()
        # Camera children close concurrently; allow their full 30+5+2s escalation budget.
        stopped = [_stop(publisher, timeouts=(40.0, 5.0, 2.0))]
        if input_session:
            time.sleep(2)
            try:
                input_session.drain_consumers()
            except Exception as exc:
                stopped.append(False)
                error = str(error or "") + " drain: " + repr(exc)
        stopped += [_stop(recorder), _stop(map_viewer)]
        if input_session:
            result = input_session.finish(drained=all(stopped), error=error)
        if not all(stopped) and sys.exc_info()[0] is None:
            raise RuntimeError("capture child process cleanup failed")
    if input_only:
        print(f"[input] {result['status']}: {dataset / 'input_integrity.json'}", flush=True)
        if result["status"] != "PASS":
            raise RuntimeError(f"camera input integrity is {result['status']}")
        return
    capture = converted / "capture"
    subprocess.run(_conda_command("sam6d", ["python", str(HERE / "rgbd_capture.py"),
        str(raw_capture), "--to-sqlite", str(capture)], domain_id=args.ros_domain_id), check=True)
    write_info(capture, camera_infos, converted / "info.json", dataset.name, args.min_duration)
    _ensure_role_links(converted, capture)
    write_roles(dataset / "camera_roles.json", roles)
    roles_path.unlink(missing_ok=True)
    if result["status"] != "PASS":
        raise RuntimeError(f"capture preserved but dataset integrity is {result['status']}; "
                           f"see {dataset / 'input_integrity.json'}")
    print(f"[record] 원본 {raw_capture}; 호환 {capture}", flush=True)


def run(args: argparse.Namespace) -> None:
    if getattr(args, "two_host_config", None):
        from two_host_map import run as run_two_host
        return run_two_host(args)
    dataset = (ROOT / "output" / args.name).resolve()
    converted = dataset / "converted"
    capture = converted / "capture"
    if dataset.exists() and args.input_check_only:
        raise ValueError(f"input check requires a new dataset: {dataset}")
    if dataset.exists() and not args.force:
        raise ValueError(f"refusing to replace existing dataset: {dataset} (--force to reuse it)")
    if args.input_check_only:
        dataset.mkdir(parents=True)
        _record(args, dataset, converted)
        return
    converted.mkdir(parents=True, exist_ok=True)
    if capture.exists():
        required = (dataset / "camera_roles.json", converted / "info.json",
                    capture / "metadata.yaml")
        if not args.force or not all(path.is_file() for path in required):
            raise ValueError(f"refusing to overwrite existing capture: {capture}")
        integrity = dataset / "input_integrity.json"
        if integrity.exists() and json.loads(integrity.read_text())["status"] != "PASS":
            raise ValueError("refusing map creation from failed/incomplete input capture")
        _validate_capture(capture, converted / "info.json",
                          dataset / "camera_roles.json", args.min_duration)
        _ensure_role_links(converted, capture)
        print(f"[reuse] 기존 capture 보존: {capture}", flush=True)
    else:
        _record(args, dataset, converted)

    command = [sys.executable, str(HERE / "camera_extrinsic_localization.py"),
               f"--inputs:={converted}",
               f"--outputs:={dataset / 'camera_extrinsic'}",
               f"--baseline:={args.baseline}",
               f"--ros-domain-id:={args.ros_domain_id}"]
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
        assert _metadata_payload(capture / "metadata.yaml")["version"] == 9
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
        # Exercise both launcher branches without opening cameras or running ROS.
        from contextlib import redirect_stdout
        from io import StringIO
        from unittest.mock import Mock, patch
        for input_only, status in ((False, "PASS"), (True, "PASS"), (True, "FAIL"), (True, "INCOMPLETE")):
            args = argparse.Namespace(name=f"check_{input_only}_{status}", force=False,
                input_check_only=input_only, input_check_seconds=1 if input_only else 30,
                input_baseline=None, slam_serial="123", sam_serial="456",
                hardware_sync=False, map_viewer=input_only, ros_domain_id=72,
                camera_timeout=30, min_duration=30, baseline=0.095)
            session = Mock()
            session.done.side_effect = [False, True]
            session.finish.return_value = {"status": status}
            factory = Mock(return_value=session)
            baseline_factory = Mock(return_value=converted / "baseline.json")
            publisher = Mock()
            publisher.poll.return_value = None
            with patch.multiple(sys.modules[__name__], ROOT=converted,
                    InputCheckSession=factory, _stop=Mock(return_value=True),
                    ensure_camera_baseline=baseline_factory,
                    _wait_roles=Mock(return_value=role_contract("123", "456")),
                    wait_camera_frame=Mock(), _camera_info=Mock(return_value=camera),
                    write_info=Mock(), _ensure_role_links=Mock()), \
                    patch.object(subprocess, "Popen", return_value=publisher) as popen, \
                    patch.object(subprocess, "run") as execute, patch.object(time, "sleep"), \
                    redirect_stdout(StringIO()):
                try:
                    run(args)
                except RuntimeError as exc:
                    assert status != "PASS" and str(exc) == f"camera input integrity is {status}"
                else:
                    assert status == "PASS"
            assert factory.call_args.kwargs["record"] is (not input_only)
            assert factory.call_args.kwargs["compatibility"] is (not input_only)
            assert factory.call_args.kwargs["is_baseline"] is input_only
            assert baseline_factory.call_count == (0 if input_only else 1)
            assert factory.call_args.kwargs["baseline"] == (None if input_only else converted / "baseline.json")
            assert factory.call_args.kwargs["consumers"] == {}
            assert popen.call_count == (1 if input_only else 2)
            assert execute.call_count == (0 if input_only else 2)
            assert "--slam-serial" in popen.call_args_list[0].args[0]
            assert "--sam-serial" in popen.call_args_list[0].args[0]
            session.close_window.assert_called_once()
            session.finish.assert_called_once_with(drained=True, error=None)
            result_dir = converted / "output" / args.name
            assert (result_dir / "camera_roles.json").is_file()
            assert (result_dir / "converted").exists() is (not input_only)


def main() -> int:
    signal.signal(signal.SIGTERM, _request_shutdown)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _request_shutdown)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name")
    parser.add_argument("--two-host-config", type=Path,
                        help="두 호스트 설정: SLAM master 원격 녹화와 mode 3 동기 검증")
    parser.add_argument("--min-duration", type=float, default=30.0)
    parser.add_argument("--baseline", type=float, default=0.095)
    parser.add_argument("--hardware-sync", action="store_true")
    parser.add_argument("--map-viewer", action="store_true",
                        help="신규 촬영 중 ORB-SLAM3 실시간 sparse-map viewer 표시")
    parser.add_argument("--ros-domain-id", type=int,
                        help="기본값: 실행마다 자동 선택")
    parser.add_argument("--camera-timeout", type=float, default=30.0)
    parser.add_argument("--force", action="store_true",
                        help="기존 capture는 보존하고 Atlas/URDF만 다시 생성")
    parser.add_argument("--input-check-seconds", type=float, default=0,
                        help="10초 준비 후 N초 검사 및 자동 종료")
    parser.add_argument("--input-check-only", action="store_true",
                        help="카메라 입력만 검사하고 종료 (양수 검사 시간과 두 serial 필수)")
    parser.add_argument("--input-baseline", type=Path)
    parser.add_argument("--slam-serial")
    parser.add_argument("--sam-serial")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    self_test()
    if args.self_test:
        print("self-test: PASS")
        return 0
    if (args.slam_serial is None) != (args.sam_serial is None):
        parser.error("--slam-serial and --sam-serial must be provided together")
    if not math.isfinite(args.input_check_seconds) or args.input_check_seconds < 0:
        parser.error("--input-check-seconds must be finite and nonnegative")
    if args.input_check_only and not args.two_host_config and (args.input_check_seconds <= 0 or args.slam_serial is None):
        parser.error("--input-check-only requires positive --input-check-seconds and both camera serials")
    if not args.input_check_only and args.input_check_seconds and args.input_check_seconds < args.min_duration:
        parser.error("--input-check-seconds must be at least --min-duration")
    if not args.name or Path(args.name).name != args.name or args.name in (".", ".."):
        parser.error("--name must be one dataset directory name")
    if not math.isfinite(args.min_duration) or args.min_duration < 30:
        parser.error("--min-duration must be at least 30 seconds")
    if not math.isfinite(args.baseline) or args.baseline <= 0:
        parser.error("--baseline must be positive")
    if not math.isfinite(args.camera_timeout) or args.camera_timeout <= 0:
        parser.error("--camera-timeout must be positive")
    try:
        if args.two_host_config:
            from two_host import cli_domain
            args.ros_domain_id = cli_domain(args)
        else:
            args.ros_domain_id = runtime_domain(args.ros_domain_id)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if not args.two_host_config:
        print(f"[ROS] Jazzy domain {args.ros_domain_id} (localhost)", flush=True)
    try:
        run(args)
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError,
            subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
