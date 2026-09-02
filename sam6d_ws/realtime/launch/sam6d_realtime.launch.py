#!/usr/bin/env python3
"""설정 파일 하나로 SAM-6D 실시간 노드를 띄운다 (+ bag 재생까지).

    conda activate sam6d
    ros2 launch realtime/launch/sam6d_realtime.launch.py config:=realtime/run_bag_example.yaml

순서: 노드 기동 -> 준비 완료 대기 -> bag 재생 -> bag 끝나면 노드에 SIGINT -> launch 종료.

준비 완료를 **고정 시간으로 기다리지 않는다.** SAM-6D 는 DINOv2 + MobileSAM + PEM +
객체별 템플릿을 올리느라 수십 초가 걸리고 그 시간이 객체 수·디스크 상태에 따라 변한다.
고정 지연이면 앞 구간 프레임을 통째로 버린다. `ros2 topic echo --once /sam6d/status`
는 첫 메시지가 올 때까지 블록하므로, 노드가 "준비됐다"고 말한 바로 그 순간 재생이 시작된다.

`bag.play: false` 로 두면 노드만 띄운다(실제 카메라 운용).
"""
from __future__ import annotations

import os
import signal
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration

REPO = Path(__file__).resolve().parents[2]
NODE = REPO / "realtime" / "sam6d_realtime_node.py"


def _abs(value, base=REPO):
    p = Path(str(value)).expanduser()
    return p if p.is_absolute() else base / p


def _setup(context, *args, **kwargs):
    del args, kwargs
    cfg_arg = LaunchConfiguration("config").perform(context)
    if not cfg_arg:
        raise RuntimeError("config 인자가 필요하다: config:=<run yaml>")
    cfg_path = _abs(cfg_arg)
    if not cfg_path.is_file():
        raise RuntimeError(f"설정 파일이 없다: {cfg_path}")
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}

    out_dir = _abs(cfg.get("output", {}).get("dir", "output/rt_run"))
    out_dir.mkdir(parents=True, exist_ok=True)
    bag = cfg.get("bag", {}) or {}
    play = bool(bag.get("play", False))

    node = ExecuteProcess(
        cmd=[sys.executable, str(NODE), "--config", str(cfg_path)],
        cwd=str(out_dir), output="screen",
        # SIGINT 로 끝나야 요약과 jsonl 이 닫힌다. 강제 kill 금지.
        sigterm_timeout="30", sigkill_timeout="60",
    )
    actions = [node]
    if not play:
        return actions

    bag_path = _abs(bag.get("path", ""), REPO)
    if not bag_path.exists():
        raise RuntimeError(f"bag.path 가 없다: {bag_path}")

    # 타입을 반드시 명시할 것: 퍼블리셔가 아직 없으면 `ros2 topic echo` 는 타입을 못 찾고
    # 기다리지 않고 즉시 exit 1 로 죽는다 -> 게이트가 무력화되고 bag 이 먼저 출발한다.
    gate = ExecuteProcess(
        cmd=["timeout", str(bag.get("ready_timeout", 600)),
             "ros2", "topic", "echo", "--once", "/sam6d/status", "std_msgs/msg/String"],
        output="log")
    play_cmd = ["ros2", "bag", "play", str(bag_path), "--rate", str(bag.get("rate", 1.0))]
    if bag.get("clock", True):
        play_cmd.append("--clock")
    for src, dst in (bag.get("remap", {}) or {}).items():
        play_cmd += ["--remap", f"{src}:={dst}"]
    player = ExecuteProcess(cmd=play_cmd, output="screen")

    actions += [
        gate,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[player])),
        RegisterEventHandler(OnProcessExit(
            target_action=player,
            on_exit=[EmitEvent(event=SignalProcess(signal_number=signal.SIGINT,
                                                   process_matcher=matches_action(node)))])),
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason="SAM-6D 실시간 실행 완료"))])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="", description="실행 설정 YAML"),
        OpaqueFunction(function=_setup),
    ])
