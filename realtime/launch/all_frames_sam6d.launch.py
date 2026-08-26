#!/usr/bin/env python3
"""Run the no-drop, sequential SAM-6D PEM Explorer capture."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


CAPTURE = Path(__file__).resolve().parents[2] / "tools" / "capture_pem_explorer.py"


def _propagate_failure(event, _context):
    if event.returncode:
        raise RuntimeError(f"sequential SAM-6D capture failed with exit code {event.returncode}")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="exhaustive capture YAML"),
        ExecuteProcess(
            cmd=["python", str(CAPTURE), "--config", LaunchConfiguration("config")],
            output="screen", on_exit=_propagate_failure),
    ])
