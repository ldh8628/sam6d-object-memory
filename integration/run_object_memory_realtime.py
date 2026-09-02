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
    topics as _topics,
    wait_camera_frame as _wait_camera_frame,
)
from run_object_memory_rosbag import (
    ROOT,
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
                        slam_serial: str, sam_serial: str):
    converted, map_results, extrinsic, atlas = _layout(map_root, map_root)
    extrinsic = dict(extrinsic)
    extrinsic["X_slam_camera_to_sam_camera"] = _urdf_transform(urdf).tolist()
    _info, orb_config, sam_config = _write_run_configs(
        converted, extrinsic, atlas, output, 1.0, view, slam_wait_ms)

    orb = yaml.safe_load(orb_config.read_text(encoding="utf-8"))
    orb["topics"].update({
        "rgb": _topics(SLAM_CAMERA)["rgb"],
        "depth": _topics(SLAM_CAMERA)["depth"],
    })
    orb["frames"]["camera"] = "slam_camera_color_optical_frame"
    orb_config.write_text(yaml.safe_dump(orb, sort_keys=False), encoding="utf-8")

    sam = yaml.safe_load(sam_config.read_text(encoding="utf-8"))
    sam["topics"] = {key: _topics(SAM_CAMERA)[key]
                     for key in ("rgbd", "rgb", "depth", "caminfo")}
    sam["runtime"].update({"use_sim_time": False, "idle_exit_s": 0})
    sam["slam"]["pose_match_tolerance_ms"] = pose_match_ms
    sam["input"] = {
        "type": "realsense",
        "namespace": f"/{CAMERA_NAMESPACE}",
        "slam_camera": {"name": SLAM_CAMERA, "serial": slam_serial},
        "sam_camera": {"name": SAM_CAMERA, "serial": sam_serial},
        "urdf": str(urdf),
        "atlas": str(atlas),
    }
    sam_config.write_text(yaml.safe_dump(sam, sort_keys=False), encoding="utf-8")
    return map_results, extrinsic, orb_config, sam_config


def run(args) -> None:
    map_root = args.map_dir.resolve()
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
    try:
        roles_out = Path(temporary.name) / "camera_roles.json"
        start("Camera publisher", _publisher_command(args, roles_out))
        selected = _wait_roles(roles_out, processes[0][1])
        _validate_roles(map_root, selected)
        for camera in (SLAM_CAMERA, SAM_CAMERA):
            for key in ("rgb", "depth"):
                _wait_camera_frame(_topics(camera)[key], args.ros_domain_id,
                                   args.camera_timeout, processes)

        output.mkdir(parents=True)
        _map_results, extrinsic, orb_config, sam_config = _write_live_configs(
            map_root, urdf, output, args.view, args.slam_wait_ms,
            args.pose_match_ms, selected["slam_serial"], selected["sam_serial"])
        start("ORB3", _conda_command(
            "orbslam3", ["ros2", "launch", "orbslam3_ros2",
                         "orb_slam.launch.py", f"config:={orb_config}"], ORB_WS,
            ORB_WS / "install/setup.bash", args.ros_domain_id))
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
        print(f"READY: {output}", flush=True)
        print(f"  SLAM RGB-D: {_topics(SLAM_CAMERA)['rgb']} + "
              f"{_topics(SLAM_CAMERA)['depth']}", flush=True)
        print(f"  SAM RGBD:   {_topics(SAM_CAMERA)['rgbd']}", flush=True)
        print("Ctrl-C로 종료하면 결과를 마무리합니다.", flush=True)
        while True:
            failed = [(name, process.returncode) for name, process in processes
                      if process.poll() is not None]
            if failed:
                raise RuntimeError(f"live process exited: {failed}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[stop] 실시간 입력 종료", flush=True)
    finally:
        by_name = dict(processes)
        shutdown_order = ("Camera publisher", "SAM-6D", "ObjectMemory", "ORB3")
        stopped = [_stop(by_name[name]) for name in shutdown_order if name in by_name]
        temporary.cleanup()
        if not all(stopped):
            if sys.exc_info()[0] is None:
                raise RuntimeError("ROS child process cleanup failed")
            print("WARNING: ROS child process cleanup failed", file=sys.stderr, flush=True)
        print(f"[cleanup] 결과: {output}", flush=True)


def self_test() -> None:
    assert _topics(SLAM_CAMERA)["rgb"] == "/camera/slam_camera/color/image_raw"
    assert _topics(SAM_CAMERA)["rgbd"] == "/camera/sam_camera/rgbd"
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
    parser.add_argument("--ros-domain-id", type=int, default=72)
    parser.add_argument("--ready-timeout", type=float, default=600.0)
    parser.add_argument("--camera-timeout", type=float, default=30.0,
                        help="각 RealSense의 첫 영상을 기다리는 최대 시간")
    parser.add_argument("--slam-wait-ms", type=float, default=1000.0,
                        help="SAM frame이 대응 SLAM pose를 기다리는 최대 시간")
    parser.add_argument("--pose-match-ms", type=float, default=50.0,
                        help="서로 다른 카메라 timestamp의 최대 허용 차이")
    parser.add_argument("--hardware-sync", action="store_true",
                        help="sync cable 연결 시 SLAM=master, SAM=slave")
    parser.add_argument("--view", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    self_test()
    if args.self_test:
        print("self-test: PASS")
        return 0
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
        run(args)
    except (ET.ParseError, OSError, ValueError, KeyError, RuntimeError, TimeoutError,
            subprocess.TimeoutExpired) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
