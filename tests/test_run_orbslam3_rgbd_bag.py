import importlib.util
from pathlib import Path

import numpy as np
import pytest


PATH = Path(__file__).resolve().parents[1] / "tools" / "run_orbslam3_rgbd_bag.py"
SPEC = importlib.util.spec_from_file_location("run_orbslam3_rgbd_bag", PATH)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def test_trajectory_stats_reports_same_stamp_coverage(tmp_path):
    trajectory = tmp_path / "CameraTrajectory.txt"
    trajectory.write_text(
        "1.000 0 0 0 0 0 0 1\n"
        "1.100 0.1 0 0 0 0 0 1\n"
        "1.200 0.2 0 0 0 0 0 1\n", encoding="utf-8")

    stats = MOD.trajectory_stats(
        trajectory, np.asarray([1.0, 1.1, 1.22]), max_dt_s=0.025)

    assert stats["pose_count"] == 3
    assert stats["matched_frame_count"] == 3
    assert stats["matched_frame_coverage"] == 1.0
    assert stats["max_tracking_gap_s"] == pytest.approx(0.1)


@pytest.mark.parametrize("content", [
    "1.0 0 0 0 0 0 0 1\n1.0 0 0 0 0 0 0 1\n",
    "1.0 0 0 0 0 0 0 0\n2.0 0 0 0 0 0 0 0\n",
    "1.0 0 0 0 nan 0 0 1\n2.0 0 0 0 0 0 0 1\n",
])
def test_trajectory_stats_rejects_unsafe_tum_input(tmp_path, content):
    trajectory = tmp_path / "bad.txt"
    trajectory.write_text(content, encoding="utf-8")

    with pytest.raises(MOD.RunError):
        MOD.trajectory_stats(trajectory, np.asarray([1.0]), max_dt_s=0.025)


def test_launch_command_sources_the_requested_workspace(tmp_path):
    setup = tmp_path / "custom workspace" / "setup.bash"
    config = tmp_path / "launch config.yaml"

    command = MOD.build_launch_command(setup, config)

    assert command[-2] == str(setup.resolve())
    assert command[-1] == f"config:={config}"
    assert "source \"$1\"" in command[2]
