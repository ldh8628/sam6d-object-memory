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
                            OpaqueFunction, RegisterEventHandler)
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
        cwd=str(out_dir), output="screen", sigterm_timeout="30")
    acts = [recv, infer]

    bag = cfg.get("bag", {}) or {}
    if bool(bag.get("play", False)):
        gate = ExecuteProcess(
            cmd=["bash", "-c",
                 f'for i in $(seq 1 {int(bag.get("ready_timeout", 600))}); do '
                 f'[ -f "{ready}" ] && exit 0; sleep 1; done; exit 1'],
            output="screen")
        play = ["ros2", "bag", "play", str(_abs(bag["path"])),
                "--rate", str(bag.get("rate", 1.0))]
        if bag.get("clock", True):
            play.append("--clock")
        player = ExecuteProcess(cmd=play, output="screen")
        acts += [gate,
                 RegisterEventHandler(OnProcessExit(target_action=gate,
                                                    on_exit=[player])),
                 # 추론 프로세스는 입력이 끊기면 스스로 끝난다(idle_exit_s) → 그때 전체 종료
                 RegisterEventHandler(OnProcessExit(target_action=infer,
                                                    on_exit=[EmitEvent(event=Shutdown())]))]
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="실행 설정 YAML"),
        OpaqueFunction(function=_setup),
    ])
