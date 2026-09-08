#!/usr/bin/env python3
"""수신 프로세스 + 추론 프로세스를 한 번에 띄운다 (+ bag 재생).

    conda activate sam6d
    ros2 launch realtime/launch/sam6d_split.launch.py config:=<run yaml>

왜 나누는가: 고속 영상 구독과 무거운 추론을 **같은 파이썬 인터프리터**에 두면 GIL 때문에
추론이 초 단위로 굶는다(NOTES 19~21절). 나누면 그 정체가 사라진다(22절 실측: 1 초 초과
프레임 8.4% → 0%, 최대 9.6 s → 0.43 s).

순서: 수신 노드 → 추론 프로세스(모델 적재) → 추론이 READY 를 쓰면 → bag 재생.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

REPO = Path(__file__).resolve().parents[2]


def _abs(v, base=REPO):
    p = Path(str(v)).expanduser()
    return p if p.is_absolute() else base / p


def _setup(context, *args, **kwargs):
    del args, kwargs
    cfg_path = _abs(LaunchConfiguration("config").perform(context))
    if not cfg_path.is_file():
        raise RuntimeError(f"설정 파일이 없다: {cfg_path}")
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    out_dir = _abs(cfg.get("output", {}).get("dir", "output/rt_split"))
    recording = cfg.get("output", {}).get("live_recording", {}) or {}
    explorer_on = bool((cfg.get("output", {}).get("pem_explorer") or {}).get("enabled"))
    if explorer_on and out_dir.exists():
        protected = (
            "explorer_manifest.json", "explorer_index.jsonl", "candidates.bin",
            "replay.bin", "detections.jsonl", "frames.jsonl", "run_meta.json",
            "receiver.jsonl", "masks",
        )
        existing = [name for name in protected if (out_dir / name).exists()]
        if existing:
            raise RuntimeError(
                f"Explorer output already contains run data; refusing replacement: "
                f"{out_dir} ({', '.join(existing)})")
    if bool(recording.get("enabled", False)) and out_dir.exists():
        protected = ("live_manifest.json", "rgb.mp4", "rgb_frames.jsonl",
                     "depth.mkv", "depth_frames.jsonl")
        existing = [name for name in protected if (out_dir / name).exists()]
        if existing:
            raise RuntimeError(
                f"live output already contains recording data; refusing replacement: "
                f"{out_dir} ({', '.join(existing)})")
    out_dir.mkdir(parents=True, exist_ok=True)
    ready = out_dir / "READY"
    try:
        ready.unlink()
    except FileNotFoundError:
        pass

    recv = ExecuteProcess(
        cmd=[sys.executable, str(REPO / "realtime" / "sam6d_receiver_node.py"),
             "--config", str(cfg_path)],
        cwd=str(out_dir), output="screen", sigterm_timeout="15")
    infer = ExecuteProcess(
        cmd=[sys.executable, str(REPO / "realtime" / "sam6d_infer.py"),
             "--config", str(cfg_path)],
        cwd=str(out_dir), output="screen", sigterm_timeout="30",
        additional_env={name: "1" for name in (
            "OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS",
            "NUMEXPR_NUM_THREADS", "OPENCV_FOR_THREADS_NUM")})
    # External bag playback (bag.play:false) still relies on inference idle_exit_s.
    # Always propagate a clean inference exit to the whole launch so the receiver is
    # not left running after the compact recorder has finalized its manifest.
    acts = [RegisterEventHandler(OnProcessExit(
                target_action=infer, on_exit=[TimerAction(
                    period=5.0, actions=[EmitEvent(event=Shutdown())])])),
            recv, infer]

    if bool(cfg.get("output", {}).get("view", False)):
        viewer = ["nice", "-n", "5", sys.executable,
                  str(REPO / "realtime" / "sam6d_viewer.py")]
        if bool(cfg.get("object_memory", {}).get("enabled", False)):
            viewer += ["--overlay-topic", str(cfg.get("output", {}).get(
                "overlay_topic", "/sam6d/overlay"))]
            guard_file = str((cfg.get("slam") or {}).get("two_host_guard_file", ""))
            if guard_file:
                viewer += ["--two-host-guard-file", guard_file]
            topdown_map = str(cfg.get("output", {}).get("topdown_map", "")).strip()
            if topdown_map:
                slam = cfg.get("slam", {}) or {}
                memory = cfg.get("object_memory", {}) or {}
                viewer += [
                    "--map-pcd", str(_abs(topdown_map)),
                    "--slam-pose-topic", str(slam.get("pose_topic", "/orbslam3/pose")),
                    "--sam-pose-topic", str(slam.get("sam_pose_topic", "/sam6d/camera_pose")),
                    "--tracking-topic", str(slam.get(
                        "tracking_topic", "/orbslam3/tracking_state")),
                    "--landmarks-topic", str(memory.get(
                        "landmarks_topic", "/object_memory/landmarks")),
                ]
        acts.append(ExecuteProcess(
            cmd=viewer,
            cwd=str(out_dir), output="screen", sigterm_timeout="10"))
    if bool(recording.get("enabled", False)):
        acts.append(ExecuteProcess(
            cmd=["nice", "-n", "10", sys.executable,
                 str(REPO / "realtime" / "sam6d_live_recorder.py"),
                 "--config", str(cfg_path)],
            cwd=str(out_dir), output="screen", sigterm_timeout="30"))

    bag = cfg.get("bag", {}) or {}
    if bool(bag.get("play", False)):
        bag_path = _abs(bag.get("path", ""))
        if not bag.get("path") or not bag_path.exists():
            raise RuntimeError(f"bag.path 가 없다: {bag_path}")
        gate = ExecuteProcess(
            cmd=["bash", "-c",
                 f'for i in $(seq 1 {int(bag.get("ready_timeout", 600))}); do '
                 f'[ -f "{ready}" ] && exit 0; sleep 1; done; exit 1'],
            output="screen")
        play = ["ros2", "bag", "play", str(bag_path),
                "--rate", str(bag.get("rate", 1.0))]
        if bag.get("clock", True):
            play.append("--clock")
        player = ExecuteProcess(cmd=play, output="screen")
        acts += [gate,
                 RegisterEventHandler(OnProcessExit(target_action=gate,
                                                    on_exit=[player]))]
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="실행 설정 YAML"),
        OpaqueFunction(function=_setup),
    ])
