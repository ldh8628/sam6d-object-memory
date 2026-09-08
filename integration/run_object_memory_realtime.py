#!/usr/bin/env python3
"""Run two RealSense cameras + ORB3 + SAM-6D + ObjectMemory live."""
from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from camera_publish import (
    CAMERA_NAMESPACE,
    SAM_CAMERA,
    SLAM_CAMERA,
    role_contract,
    runtime_domain,
    topics as _topics,
    wait_camera_frame as _wait_camera_frame,
)
from rgbd_capture import record_topics as _record_topics, recorder_args
from input_check import InputCheckSession, ensure_camera_baseline
from run_object_memory_rosbag import (
    ROOT,
    ORB_ENV,
    ORB_INSTALL,
    ORB_WS,
    SAM_WS,
    _conda_command,
    _layout,
    _request_shutdown,
    _stop,
    _wait_orb_ready,
    _write_run_configs,
)
from slam_pose_memory import valid_se3


def _urdf_transform(path: Path) -> np.ndarray:
    root = ET.parse(path).getroot()
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if (parent is not None and child is not None
                and joint.get("type") == "fixed"
                and parent.get("link") == "slam_camera_color_optical_frame"
                and child.get("link") == "sam_camera_color_optical_frame"):
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None
                                      else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None
                                      else "0 0 0").split()]
            if len(xyz) != 3 or len(rpy) != 3:
                break
            roll, pitch, yaw = rpy
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            matrix = np.eye(4)
            matrix[:3, :3] = [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]
            matrix[:3, 3] = xyz
            if valid_se3(matrix):
                return matrix
            break
    raise ValueError(
        f"URDF needs a valid slam_camera_color_optical_frame -> "
        f"sam_camera_color_optical_frame fixed joint: {path}")


def _publisher_command(args, roles_out: Path) -> list[str]:
    command = ["/usr/bin/python3", str(ROOT / "integration/camera_publish.py"),
               "--ros-domain-id", str(args.ros_domain_id),
               "--roles-out", str(roles_out),
               "--camera-timeout", str(args.camera_timeout)]
    if args.hardware_sync:
        command.append("--hardware-sync")
    if args.slam_serial is not None:
        roles = role_contract(args.slam_serial, args.sam_serial)
        command += ["--slam-serial", roles["slam_serial"],
                    "--sam-serial", roles["sam_serial"]]
    return command


def _recorder_command(output: Path, domain_id: int) -> list[str]:
    return _conda_command("sam6d", recorder_args(output / "capture"), domain_id=domain_id)


def _wait_roles(path: Path, process: subprocess.Popen) -> dict[str, str]:
    while not path.is_file():
        if process.poll() is not None:
            raise RuntimeError(
                f"camera publisher exited before role selection (rc={process.returncode})")
        time.sleep(0.1)
    roles = json.loads(path.read_text(encoding="utf-8"))
    return role_contract(roles["slam_serial"], roles["sam_serial"])


def _validate_roles(map_root: Path, selected: dict[str, str]) -> None:
    path = map_root / "camera_roles.json"
    if not path.is_file():
        print(f"WARNING: camera role contract is missing; cannot validate: {path}",
              file=sys.stderr, flush=True)
        return
    saved = json.loads(path.read_text(encoding="utf-8"))
    expected = role_contract(saved["slam_serial"], saved["sam_serial"])
    if selected != expected:
        raise ValueError(
            "camera roles differ from calibration: "
            f"expected SLAM={expected['slam_serial']} SAM={expected['sam_serial']}, "
            f"selected SLAM={selected['slam_serial']} SAM={selected['sam_serial']}")


def _write_live_configs(map_root: Path, urdf: Path, output: Path, view: bool,
                        slam_wait_ms: float, pose_match_ms: float,
                        slam_serial: str, sam_serial: str, record: bool = False):
    converted, map_results, extrinsic, atlas = _layout(map_root, map_root)
    extrinsic = dict(extrinsic)
    extrinsic["X_slam_camera_to_sam_camera"] = _urdf_transform(urdf).tolist()
    _info, orb_config, sam_config = _write_run_configs(
        converted, extrinsic, atlas, output, 1.0, view, slam_wait_ms)

    orb = yaml.safe_load(orb_config.read_text(encoding="utf-8"))
    orb["topics"].update({
        "rgbd": _topics(SLAM_CAMERA)["rgbd"],
        "rgb": _topics(SLAM_CAMERA)["rgb"],
        "depth": _topics(SLAM_CAMERA)["depth"],
    })
    orb["runtime"].update({"input_require_publishers": True, "input_queue_size": 60, "input_qos_depth": 120,
                           "input_camera": SLAM_CAMERA, "input_serial": slam_serial})
    orb["frames"]["camera"] = "slam_camera_color_optical_frame"
    orb_config.write_text(yaml.safe_dump(orb, sort_keys=False), encoding="utf-8")

    sam = yaml.safe_load(sam_config.read_text(encoding="utf-8"))
    sam["topics"] = {key: _topics(SAM_CAMERA)[key]
                     for key in ("rgbd", "rgb", "depth", "caminfo")}
    sam["runtime"].update({"use_sim_time": False, "idle_exit_s": 0,
                           "qos_depth": 120, "input_queue_size": 60, "publish_native_camera_info": False})
    sam["output"]["live_recording"]["enabled"] = record
    sam["slam"]["pose_match_tolerance_ms"] = pose_match_ms
    sam["input"] = {
        "type": "realsense",
        "namespace": f"/{CAMERA_NAMESPACE}",
        "slam_camera": {"name": SLAM_CAMERA, "serial": slam_serial},
        "sam_camera": {"name": SAM_CAMERA, "serial": sam_serial},
        "urdf": str(urdf),
        "atlas": str(atlas),
    }
    map_candidates = (
        map_results / "map_slam/orbslam3_dense_map.pcd",
        map_root / "object_memory/localize_slam/orbslam3_dense_map.pcd",
        map_results / "map_slam/orbslam3_map_points.pcd",
    )
    topdown_map = next((path for path in map_candidates if path.is_file()), None)
    if topdown_map is not None:
        sam["output"]["topdown_map"] = str(topdown_map)
    sam_config.write_text(yaml.safe_dump(sam, sort_keys=False), encoding="utf-8")
    return map_results, extrinsic, orb_config, sam_config


def run(args) -> None:
    map_root = args.map_dir.resolve()
    roles_path = map_root / "camera_roles.json"
    if args.slam_serial is None and roles_path.is_file():
        saved = json.loads(roles_path.read_text(encoding="utf-8"))
        selected = role_contract(saved["slam_serial"], saved["sam_serial"])
        args = argparse.Namespace(**vars(args))
        args.slam_serial, args.sam_serial = selected["slam_serial"], selected["sam_serial"]
    map_results = map_root / "camera_extrinsic"
    if not map_results.is_dir():
        map_results = map_root
    urdf = (args.urdf.resolve() if args.urdf else
            map_results / "camera_extrinsic.urdf")
    if not urdf.is_file():
        raise ValueError(f"camera extrinsic URDF is missing: {urdf}")

    default_output = (ROOT / "output" / map_root.name / "object_memory" /
                      f"realtime_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    output = args.output.resolve() if args.output else default_output
    if output.exists():
        raise ValueError(f"refusing to replace existing run: {output}")
    processes: list[tuple[str, subprocess.Popen]] = []

    def start(name: str, command: list[str]) -> None:
        processes.append((name, subprocess.Popen(command, start_new_session=True)))

    temporary = tempfile.TemporaryDirectory()
    input_session = None
    run_error = None
    try:
        roles_out = Path(temporary.name) / "camera_roles.json"
        start("Camera publisher", _publisher_command(args, roles_out))
        selected = _wait_roles(roles_out, processes[0][1])
        _validate_roles(map_root, selected)
        for camera in (SLAM_CAMERA, SAM_CAMERA):
            for key in ("rgbd",):
                _wait_camera_frame(_topics(camera)[key], args.ros_domain_id,
                                   args.camera_timeout, processes)

        output.mkdir(parents=True)
        baseline = ensure_camera_baseline(args.input_baseline, output, args.ros_domain_id,
            {SLAM_CAMERA:selected["slam_serial"], SAM_CAMERA:selected["sam_serial"]},
            processes, args.camera_timeout, settings={"view":False,"hardware_sync":args.hardware_sync})
        _map_results, extrinsic, orb_config, sam_config = _write_live_configs(
            map_root, urdf, output, args.view, args.slam_wait_ms,
            args.pose_match_ms, selected["slam_serial"], selected["sam_serial"], args.record)
        (output / "camera_roles.json").write_text(json.dumps(selected, indent=2))
        input_session = InputCheckSession(output, args.ros_domain_id,
            {SLAM_CAMERA: selected["slam_serial"], SAM_CAMERA: selected["sam_serial"]},
            seconds=args.input_check_seconds, record=args.record, capture=output/"capture",
            consumers={SLAM_CAMERA:["orb"], SAM_CAMERA:["sam"]}, baseline=baseline,
            settings={"view":args.view, "hardware_sync":args.hardware_sync})
        start("ORB3", _conda_command(
            ORB_ENV, ["ros2", "launch", "orbslam3_ros2",
                         "orb_slam.launch.py", f"config:={orb_config}"], ORB_WS,
            ORB_INSTALL / "setup.bash", args.ros_domain_id))
        start("SAM-6D", _conda_command(
            "sam6d", ["ros2", "launch", "realtime/launch/sam6d_split.launch.py",
                      f"config:={sam_config}"], SAM_WS,
            domain_id=args.ros_domain_id))
        start("ObjectMemory", _conda_command(
            "sam6d", [
                "python", str(ROOT / "objectmemory_ws/object_memory/ros/object_memory_node.py"),
                "--ros-args", "-p", "pose_topic:=/sam6d/camera_pose",
                "-p", "detections_topic:=/sam6d/detections",
                "-p", f"caminfo_topic:={_topics(SAM_CAMERA)['caminfo']}",
                "-p", "landmarks_topic:=/object_memory/landmarks",
                "-p", f"map_frame:={extrinsic['dataset']}",
                "-p", f"output_path:={output / 'object_memory.json'}",
            ], ROOT, domain_id=args.ros_domain_id))
        ready = output / "READY"
        deadline = time.monotonic() + args.ready_timeout
        while not ready.is_file():
            failed = [(name, process.returncode) for name, process in processes
                      if process.poll() is not None]
            if failed:
                raise RuntimeError(f"process exited before READY: {failed}")
            if time.monotonic() >= deadline:
                raise TimeoutError("SAM-6D model READY timeout")
            time.sleep(0.5)
        _wait_orb_ready(args.ros_domain_id, max(1.0, deadline - time.monotonic()))
        if args.record:
            recorder = subprocess.Popen(_recorder_command(output, args.ros_domain_id),
                                        stdin=subprocess.DEVNULL, start_new_session=True)
            processes.append(("Recorder", recorder))
        input_session.wait_ready(processes, args.camera_timeout)
        input_session.arm()
        print(f"READY: {output}", flush=True)
        print(f"  SLAM RGBD:  {_topics(SLAM_CAMERA)['rgbd']}", flush=True)
        print(f"  SAM RGBD:   {_topics(SAM_CAMERA)['rgbd']}", flush=True)
        print("Ctrl-C로 종료하면 결과를 마무리합니다.", flush=True)
        while not input_session.done():
            failed = [(name, process.returncode) for name, process in processes
                      if process.poll() is not None]
            if failed:
                raise RuntimeError(f"live process exited: {failed}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[stop] 실시간 입력 종료", flush=True)
    except Exception as exc:
        run_error = repr(exc)
        raise
    finally:
        by_name = dict(processes)
        stopped = []
        if input_session is not None:
            input_session.close_window()
        # Close the source first; leave DDS readers and ROS context alive for the tail.
        if "Camera publisher" in by_name:
            # Allow the camera children's full 30+5+2s concurrent cleanup budget.
            stopped.append(_stop(by_name.pop("Camera publisher"), timeouts=(40.0, 5.0, 2.0)))
        if input_session is not None:
            time.sleep(2.0)
            try:
                input_session.drain_consumers()
            except Exception as exc:
                run_error = str(run_error or "") + " drain: " + repr(exc)
                stopped.append(False)
        for name in ("Recorder", "SAM-6D", "ObjectMemory", "ORB3"):
            if name in by_name:
                stopped.append(_stop(by_name[name]))
        temporary.cleanup()
        if input_session is not None:
            result = input_session.finish(drained=all(stopped), error=run_error)
            print(f"[input] {result['status']}: {output / 'input_integrity.json'}", flush=True)
            if result['status'] != 'PASS' and sys.exc_info()[0] is None:
                raise RuntimeError(f"input integrity is {result['status']}; see {output / 'input_integrity.json'}")
        if not all(stopped) and sys.exc_info()[0] is None:
            raise RuntimeError("ROS input drain or child process cleanup failed")
        print(f"[cleanup] 결과: {output}", flush=True)


def self_test() -> None:
    assert _topics(SLAM_CAMERA)["rgb"] == "/camera/slam_camera/color/image_raw"
    assert _topics(SAM_CAMERA)["rgbd"] == "/camera/sam_camera/rgbd"
    expected_topics = _record_topics()
    with tempfile.TemporaryDirectory() as directory:
        recorder_command = _recorder_command(Path(directory), 73)
        assert "ROS_DOMAIN_ID=73" in recorder_command[2]
        assert "--storage mcap" in recorder_command[2]
        assert all(topic in recorder_command[2] for topic in expected_topics)
        assert not any(topic.endswith("image_raw") for topic in expected_topics)
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        urdf = root / "camera_extrinsic.urdf"
        urdf.write_text("""<robot name="test"><link name="a"/><link name="b"/>
<joint name="x" type="fixed">
  <parent link="slam_camera_color_optical_frame"/>
  <child link="sam_camera_color_optical_frame"/>
  <origin xyz="0.1 -0.2 0.3" rpy="0 0 0"/>
</joint></robot>""")
        matrix = _urdf_transform(urdf)
        assert valid_se3(matrix) and np.allclose(matrix[:3, 3], [0.1, -0.2, 0.3])
        (root / "camera_roles.json").write_text(json.dumps({
            "slam_serial": "123", "sam_serial": "456"}))
        _validate_roles(root, {"slam_serial": "123", "sam_serial": "456"})
        try:
            _validate_roles(root, {"slam_serial": "456", "sam_serial": "123"})
        except ValueError:
            pass
        else:
            raise AssertionError("calibration role mismatch accepted")


def main() -> int:
    signal.signal(signal.SIGTERM, _request_shutdown)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _request_shutdown)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path,
                        help="converted/와 camera_extrinsic/가 있는 eightcircle 결과 루트")
    parser.add_argument("--urdf", type=Path,
                        help="기본값: MAP_DIR/camera_extrinsic/camera_extrinsic.urdf")
    parser.add_argument("--slam-serial")
    parser.add_argument("--sam-serial")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--ros-domain-id", type=int,
                        help="기본값: 실행마다 자동 선택")
    parser.add_argument("--ready-timeout", type=float, default=600.0)
    parser.add_argument("--camera-timeout", type=float, default=30.0,
                        help="각 RealSense의 첫 영상을 기다리는 최대 시간")
    parser.add_argument("--slam-wait-ms", type=float, default=1000.0,
                        help="SAM frame이 대응 SLAM pose를 기다리는 최대 시간")
    parser.add_argument("--pose-match-ms", type=float, default=50.0,
                        help="서로 다른 카메라 timestamp의 최대 허용 차이")
    parser.add_argument("--hardware-sync", action="store_true",
                        help="sync cable 연결 시 SLAM=master, SAM=slave")
    parser.add_argument("--record", action="store_true",
                        help="두 카메라 native RGBD·metadata 및 ORB 상태를 MCAP으로 녹화")
    parser.add_argument("--view", action="store_true")
    parser.add_argument("--input-check-seconds", type=float, default=0,
                        help="10초 준비 후 N초 입력 검사 및 자동 종료 (0: 수동 종료)")
    parser.add_argument("--input-baseline", type=Path, help="동일 설정 카메라 기준 input_integrity.json")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    self_test()
    if args.self_test:
        print("self-test: PASS")
        return 0
    if not math.isfinite(args.input_check_seconds) or args.input_check_seconds < 0:
        parser.error("--input-check-seconds must be finite and nonnegative")
    if args.map_dir is None:
        parser.error("--map-dir is required")
    if (args.slam_serial is None) != (args.sam_serial is None):
        parser.error("--slam-serial and --sam-serial must be provided together")
    if not np.isfinite(args.slam_wait_ms) or args.slam_wait_ms < 0:
        parser.error("--slam-wait-ms must be non-negative")
    if not np.isfinite(args.pose_match_ms) or args.pose_match_ms < 0:
        parser.error("--pose-match-ms must be non-negative")
    if not np.isfinite(args.camera_timeout) or args.camera_timeout <= 0:
        parser.error("--camera-timeout must be positive")
    if not np.isfinite(args.ready_timeout) or args.ready_timeout <= 0:
        parser.error("--ready-timeout must be positive")
    try:
        args.ros_domain_id = runtime_domain(args.ros_domain_id)
    except ValueError as exc:
        parser.error(str(exc))
    print(f"[ROS] Jazzy domain {args.ros_domain_id} (localhost)", flush=True)
    try:
        run(args)
    except (ET.ParseError, OSError, ValueError, KeyError, RuntimeError, TimeoutError,
            subprocess.TimeoutExpired) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
