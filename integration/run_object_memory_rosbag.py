#!/usr/bin/env python3
"""Run ORB3 + SAM-6D + ObjectMemory from paired ROS2 bags in real time."""
from __future__ import annotations

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SAM_WS = ROOT / "sam6d_ws"
ORB_WS = ROOT / "orbslam_ws"
CONDA_SH = Path.home() / "miniconda3/etc/profile.d/conda.sh"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(SAM_WS / "realtime"))

from camera_extrinsic_localization import (  # noqa: E402
    ORB_ENV,
    ORB_INSTALL,
    write_config,
    write_settings,
)
from camera_publish import runtime_domain  # noqa: E402
from slam_pose_memory import valid_se3  # noqa: E402


def _layout(dataset_dir: Path, map_dir: Path):
    converted = dataset_dir / "converted" if (dataset_dir / "converted").is_dir() else dataset_dir
    for source in (dataset_dir, dataset_dir / "capture", converted / "capture_raw"):
        metadata_path = source / "metadata.yaml"
        if metadata_path.is_file():
            metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
            if any(t["topic_metadata"]["type"] == "realsense2_camera_msgs/msg/RGBD"
                   for t in metadata["topics_with_message_count"]):
                converted = source.parent / (source.name + "_replay_layout")
                subprocess.run(_conda_command("sam6d", ["python", str(HERE/"rgbd_capture.py"),
                    str(source), "--prepare-replay", str(converted)]), check=True)
                break
    camera_extrinsic = (map_dir / "camera_extrinsic"
                        if (map_dir / "camera_extrinsic").is_dir() else map_dir)
    info_path = converted / "info.json"
    extrinsic_path = camera_extrinsic / "camera_extrinsic.json"
    if not info_path.is_file() or not (converted / "SLAM").is_dir() or not (converted / "SAM").is_dir():
        raise ValueError(f"expected info.json, SLAM/, and SAM/ under {converted}")
    if not extrinsic_path.is_file():
        raise ValueError(f"camera extrinsic is missing: {extrinsic_path}")
    extrinsic = json.loads(extrinsic_path.read_text(encoding="utf-8"))
    map_name = str(extrinsic["dataset"])
    atlas = camera_extrinsic / "map_slam" / f"map_{map_name}.osa"
    if not atlas.is_file():
        raise ValueError(f"ORB3 atlas is missing: {atlas}")
    return converted, camera_extrinsic, extrinsic, atlas


def _paired_delays(info: dict, rate: float = 1.0, base_s: float = 1.0):
    starts = {name: int(info[name]["t_start_ns"]) for name in ("SLAM", "SAM")}
    first = min(starts.values())
    return {name: base_s + (stamp - first) / 1e9 / rate
            for name, stamp in starts.items()}


def _write_run_configs(converted: Path, extrinsic: dict, atlas: Path,
                       output: Path, rate: float, view: bool = False,
                       slam_wait_ms: float = 1000.0):
    info = json.loads((converted / "info.json").read_text(encoding="utf-8"))
    matrix = np.asarray(extrinsic["X_slam_camera_to_sam_camera"], dtype=float)
    if not valid_se3(matrix):
        raise ValueError("camera_extrinsic.json contains an invalid SE(3)")

    orb_out = output / "orbslam3"
    orb_out.mkdir(parents=True)
    atlas_link = orb_out / atlas.name
    atlas_link.symlink_to(atlas.resolve())
    settings = orb_out / "orb_settings.yaml"
    orb_config = orb_out / "orb_config.yaml"
    write_settings(info, "SLAM", settings, "System.LoadAtlasFromFile",
                   atlas.stem, 0.05)
    write_config(info, "SLAM", orb_config, converted / "SLAM", settings,
                 orb_out, rate, 0.0, True, dense_map=False)
    orb = yaml.safe_load(orb_config.read_text(encoding="utf-8"))
    orb["bag"]["play"] = False
    orb["topics"]["rgb"] = "/slam_camera/color/image_raw"
    orb["topics"]["depth"] = "/slam_camera/aligned_depth_to_color/image_raw"
    if info["SLAM"].get("rgbd_topic"):
        orb["topics"]["rgbd"] = "/slam_camera/rgbd"
    orb["runtime"]["input_require_publishers"] = False
    orb["runtime"]["publish_map_points"] = False
    orb["dense_map"]["enabled"] = False
    orb_config.write_text(yaml.safe_dump(orb, sort_keys=False), encoding="utf-8")

    base = yaml.safe_load((SAM_WS / "realtime/run_live_split.yaml").read_text(
        encoding="utf-8")) or {}
    base["topics"] = {
        "rgb": "/camera/camera/color/image_raw",
        "depth": "/camera/camera/aligned_depth_to_color/image_raw",
        "caminfo": "/camera/camera/color/camera_info",
    }
    if info["SAM"].get("rgbd_topic"):
        base["topics"]["rgbd"] = "/camera/camera/rgbd"
        base.setdefault("runtime", {})["publish_native_camera_info"] = True
    slam = base.setdefault("slam", {})
    slam.update({
        "pose_topic": "/orbslam3/pose",
        "tracking_topic": "/orbslam3/tracking_state",
        "sam_pose_topic": "/sam6d/camera_pose",
        "pose_match_tolerance_ms": 50.0,
        "timestamp_tolerance_ms": slam_wait_ms,
        "map_id": str(extrinsic["dataset"]),
        "T_slam_camera_sam_camera": matrix.tolist(),
    })
    base["anchor"] = {"enabled": False}
    base["object_memory"] = {
        "enabled": True, "landmarks_topic": "/object_memory/landmarks"}
    runtime = base.setdefault("runtime", {})
    runtime.update({"use_sim_time": True, "idle_exit_s": 5,
                    "qos_reliability": "reliable", "qos_depth": 120})
    base["bag"] = {"play": False}
    out = base.setdefault("output", {})
    out.update({"dir": str(output), "diagnostics": True, "view": view,
                "overlay_topic": "/sam6d/overlay"})
    out["live_recording"] = {
        "enabled": True, "depth_policy": "processed",
        "rgb_fps": float(info["SAM"].get("color_hz", 30.0)), "nvenc_cq": 32}
    sam_config = output / "run_config.yaml"
    sam_config.write_text(yaml.safe_dump(base, sort_keys=False), encoding="utf-8")
    return info, orb_config, sam_config


def _conda_command(environment: str, command: list[str], cwd: Path | None = None,
                   source: Path | None = None, domain_id: int = 72):
    parts = ["unset PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION",
             "unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH",
             f"source {shlex.quote(str(CONDA_SH))}",
             f"conda activate {shlex.quote(environment)}"]
    if source is not None:
        parts.append(f"source {shlex.quote(str(source))}")
    parts += [f"export ROS_DOMAIN_ID={int(domain_id)}",
              "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST",
              "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
              f"export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(HERE / 'fastdds_input.xml'))}"]
    if cwd is not None:
        parts.append(f"cd {shlex.quote(str(cwd))}")
    parts.append(shlex.join(command))
    return ["bash", "-lc", "; ".join(parts)]


def _group_alive(pgid: int):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def _request_shutdown(_signum, _frame):
    raise KeyboardInterrupt


def _stop(process, timeouts=(30.0, 5.0, 2.0)):
    """Stop the whole spawned session, even if its launch parent already exited."""
    if process is None:
        return True
    pgid = process.pid
    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
        process.poll()  # reap the direct child when possible
        if not _group_alive(pgid):
            return True
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + timeout
        while _group_alive(pgid) and time.monotonic() < deadline:
            process.poll()
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                # A repeated Ctrl-C must not interrupt cleanup and orphan children.
                continue
    process.poll()
    return not _group_alive(pgid)


def _play_pair(converted: Path, delays: dict, rate: float, domain_id: int):
    info = json.loads((converted / "info.json").read_text())
    # Shared role symlinks represent one bag and must be played exactly once.
    groups = {}
    for role in ("SLAM", "SAM"):
        source = (converted / role).resolve()
        entry = groups.setdefault(source, {"roles": [], "delay": delays[role]})
        entry["roles"].append(role)
        entry["delay"] = min(entry["delay"], delays[role])
    clock_owner = min(groups, key=lambda source: groups[source]["delay"])
    commands = []
    for source, entry in groups.items():
        remaps, selected = [], []
        for role in entry["roles"]:
            data = info[role]
            target = "/slam_camera" if role == "SLAM" else "/camera/camera"
            pairs = [(data["rgbd_topic"], target+"/rgbd")] if data.get("rgbd_topic") else [
                (data["color_topic"], target+"/color/image_raw"),
                (data["depth_topic"], target+"/aligned_depth_to_color/image_raw"),
                (data.get("caminfo_topic", data["color_topic"].replace("image_raw", "camera_info")), target+"/color/camera_info"),
                (data["depth_topic"].replace("image_raw", "camera_info"), target+"/aligned_depth_to_color/camera_info")]
            for original, destination in pairs:
                selected.append(original)
                if original != destination: remaps.append(original+":="+destination)
        command = ["ros2", "bag", "play", str(source), "--rate", str(rate),
                   "--delay", f"{entry['delay']:.6f}", "--read-ahead-queue-size", "120",
                   "--disable-keyboard-controls", "--topics", *selected]
        if remaps: command += ["--remap", *remaps]
        if source == clock_owner: command.append("--clock")
        commands.append(shlex.join(command))
    if len(commands) == 1:
        script = "exec " + commands[0]
    else:
        script = (f"{commands[0]} & p0=$!; {commands[1]} & p1=$!; "
                  "wait $p0; r0=$?; wait $p1; r1=$?; test $r0 -eq 0 -a $r1 -eq 0")
    return subprocess.Popen(_conda_command(
        ORB_ENV, ["bash", "-c", script], domain_id=domain_id), start_new_session=True)


def _wait_orb_ready(domain_id: int, timeout: float, orb=None):
    command = _conda_command(ORB_ENV, [
        "ros2", "topic", "echo", "--no-daemon", "--once",
        "--qos-reliability", "reliable", "--qos-durability", "transient_local",
        "/orbslam3/ready", "std_msgs/msg/String",
    ], domain_id=domain_id)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if orb is not None and orb.poll() is not None:
            raise RuntimeError(f"ORB3 exited before READY: {orb.returncode}")
        probe = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            if probe.wait(timeout=min(5.0, max(0.1, deadline - time.monotonic()))) == 0:
                return
        except subprocess.TimeoutExpired:
            pass
        finally:
            _stop(probe, timeouts=(0.5, 0.5, 0.5))
    raise TimeoutError("ORB3 Atlas READY timeout")


def _validate_completed_output(output: Path):
    meta = json.loads((output / "run_meta.json").read_text(encoding="utf-8"))
    expected = int(meta.get("summary", {}).get("frames_processed", -1))
    recorded = sum(1 for line in (output / "frames.jsonl").open(encoding="utf-8")
                   if line.strip())
    fused = int(json.loads((output / "object_memory.json").read_text(
        encoding="utf-8"))["frame"])
    if expected < 0 or recorded != expected or fused != expected:
        raise RuntimeError(
            f"incomplete result delivery: inference={expected}, recorded={recorded}, "
            f"object_memory={fused}")


def run(args):
    dataset_dir, map_dir = args.dataset_dir.resolve(), args.map_dir.resolve()
    converted, _map_results, extrinsic, atlas = _layout(dataset_dir, map_dir)
    dataset_name = dataset_dir.name if (dataset_dir / "converted").is_dir() else converted.name
    output = (args.output.resolve() if args.output else
              ROOT / "output" / dataset_name / "object_memory" /
              f"live_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    if output.exists():
        raise ValueError(f"refusing to replace existing run: {output}")
    output.mkdir(parents=True)
    info, orb_config, sam_config = _write_run_configs(
        converted, extrinsic, atlas, output, args.rate, args.view,
        args.slam_wait_ms)
    (output / "run_meta.json").write_text(json.dumps({
        "mode": "ros2_bag_subscribe_realtime", "dataset": dataset_name,
        "converted": str(converted), "map": str(atlas), "rate": args.rate,
        "started_at": datetime.now().isoformat(timespec="seconds")},
        indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    orb = memory = sam = player = None
    try:
        orb = subprocess.Popen(_conda_command(
            ORB_ENV, ["ros2", "launch", "orbslam3_ros2", "orb_slam.launch.py",
                         f"config:={orb_config}"], ORB_WS,
            ORB_INSTALL / "setup.bash", args.ros_domain_id), start_new_session=True)
        _wait_orb_ready(args.ros_domain_id, args.ready_timeout, orb)
        sam = subprocess.Popen(_conda_command(
            "sam6d", ["ros2", "launch", "realtime/launch/sam6d_split.launch.py",
                      f"config:={sam_config}"], SAM_WS,
            domain_id=args.ros_domain_id), start_new_session=True)
        memory = subprocess.Popen(_conda_command(
            "sam6d", ["python", str(ROOT / "objectmemory_ws/object_memory/ros/object_memory_node.py"),
                      "--ros-args", "-p", "pose_topic:=/sam6d/camera_pose",
                      "-p", "detections_topic:=/sam6d/detections",
                      "-p", "caminfo_topic:=/camera/camera/color/camera_info",
                      "-p", "landmarks_topic:=/object_memory/landmarks",
                      "-p", f"map_frame:={extrinsic['dataset']}",
                      "-p", f"output_path:={output / 'object_memory.json'}"],
            ROOT, domain_id=args.ros_domain_id), start_new_session=True)

        ready = output / "READY"
        deadline = time.monotonic() + args.ready_timeout
        while not ready.is_file():
            failed = [(name, process.returncode) for name, process in
                      (("ORB3", orb), ("SAM-6D", sam), ("ObjectMemory", memory))
                      if process.poll() is not None]
            if failed:
                raise RuntimeError(f"process exited before READY: {failed}")
            if time.monotonic() >= deadline:
                raise TimeoutError("SAM-6D model READY timeout")
            time.sleep(0.5)
        if args.subscribe_only:
            print(f"READY_TO_PLAY: {output}", flush=True)
            sam.wait()
        else:
            delays = _paired_delays(info, args.rate)
            print(f"[play] SLAM delay={delays['SLAM']:.3f}s, SAM delay={delays['SAM']:.3f}s",
                  flush=True)
            player = _play_pair(converted, delays, args.rate, args.ros_domain_id)
            if player.wait() != 0:
                raise RuntimeError("paired ros2 bag play failed")
            sam.wait(timeout=45)
    finally:
        stopped = [_stop(process) for process in (player, sam, memory, orb)]
        if not all(stopped):
            raise RuntimeError("ROS child process cleanup failed")
        print("[cleanup] ORB3, SAM-6D, ObjectMemory, rosbag 종료 완료", flush=True)
    _validate_completed_output(output)
    print(f"complete: {output}")


def play_only(args):
    dataset = args.dataset_dir.resolve()
    converted = dataset / "converted" if (dataset / "converted").is_dir() else dataset
    info_path = converted / "info.json"
    if not info_path.is_file() or not all((converted / name).is_dir()
                                           for name in ("SLAM", "SAM")):
        raise ValueError(f"expected info.json, SLAM/, and SAM/ under {converted}")
    info = json.loads(info_path.read_text(encoding="utf-8"))
    delays = _paired_delays(info, args.rate)
    print(f"[play] SLAM delay={delays['SLAM']:.3f}s, SAM delay={delays['SAM']:.3f}s",
          flush=True)
    player = _play_pair(converted, delays, args.rate, args.ros_domain_id)
    try:
        if player.wait() != 0:
            raise RuntimeError("paired ros2 bag play failed")
    finally:
        if not _stop(player):
            raise RuntimeError("rosbag player cleanup failed")
        print("[cleanup] rosbag 종료 완료", flush=True)


def self_test():
    info = {"SLAM": {"t_start_ns": 1_000_000_000},
            "SAM": {"t_start_ns": 1_040_000_000}}
    assert _paired_delays(info) == {"SLAM": 1.0, "SAM": 1.04}
    assert _paired_delays(info, 2.0) == {"SLAM": 1.0, "SAM": 1.02}
    command = _conda_command("x", ["printf", "%s", "a b"], domain_id=7)
    assert command[:2] == ["bash", "-lc"] and "ROS_DOMAIN_ID=7" in command[2]
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in command[2]


def _self_test_stop():
    child = subprocess.Popen([
        sys.executable, "-c",
        "import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)"
    ], start_new_session=True)
    time.sleep(0.1)
    assert _stop(child, timeouts=(0.1, 0.1, 1.0))


def main():
    signal.signal(signal.SIGTERM, _request_shutdown)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _request_shutdown)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", type=Path)
    parser.add_argument("--map-dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--ready-timeout", type=float, default=600.0)
    parser.add_argument("--ros-domain-id", type=int,
                        help="기본값: 실행마다 자동 선택")
    parser.add_argument("--slam-wait-ms", type=float, default=1000.0,
                        help="SAM frame이 같은 시각의 SLAM pose를 기다리는 최대 시간")
    parser.add_argument("--view", action="store_true",
                        help="입력 영상에 ObjectMemory map pose를 실시간 표시")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--subscribe-only", action="store_true",
                      help="ORB3, SAM-6D, ObjectMemory와 UI만 실행하고 외부 bag 입력 대기")
    mode.add_argument("--play-only", action="store_true",
                      help="SLAM/SAM bag 두 개만 동기화하여 ros2 bag play")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    self_test()
    if args.self_test:
        _self_test_stop()
        print("self-test: PASS")
        return 0
    if args.dataset_dir is None or (args.map_dir is None and not args.play_only):
        parser.error("--dataset-dir and --map-dir are required (play-only needs dataset only)")
    if not np.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be positive")
    if not np.isfinite(args.slam_wait_ms) or args.slam_wait_ms < 0:
        parser.error("--slam-wait-ms must be non-negative")
    try:
        args.ros_domain_id = runtime_domain(args.ros_domain_id)
    except ValueError as exc:
        parser.error(str(exc))
    print(f"[ROS] Jazzy domain {args.ros_domain_id} (localhost)", flush=True)
    try:
        play_only(args) if args.play_only else run(args)
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError,
            subprocess.TimeoutExpired) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
