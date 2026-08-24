import json
from pathlib import Path

import numpy as np
import pytest

from realtime.slam_pose_memory import StaticSlamPoseMemory, pose_matrix


def test_projects_map_anchor_back_to_camera(tmp_path: Path):
    anchor = pose_matrix(np.eye(3), [2.0, 0.0, 1.0])
    twc = pose_matrix(np.eye(3), [0.5, 0.0, 0.0])
    path = tmp_path / "memory.json"
    path.write_text(json.dumps({"anchors": {
        "milk": {"R": anchor[:3, :3].tolist(), "t_m": anchor[:3, 3].tolist(),
                 "trusted": True},
        "ignored": {"R": np.eye(3).tolist(), "t_m": [0, 0, 0], "trusted": False},
    }}), encoding="utf-8")

    memory = StaticSlamPoseMemory.from_evaluation(path)
    result = memory.camera_pose("milk", twc)

    assert memory.objects() == ("milk",)
    assert np.allclose(result, np.linalg.inv(twc) @ anchor)
    assert memory.camera_pose("unknown", twc) is None


def test_rejects_invalid_transforms():
    with pytest.raises(ValueError, match=r"invalid SE\(3\) anchor"):
        StaticSlamPoseMemory({"milk": {"R": np.zeros((3, 3)).tolist(),
                                               "t_m": [0, 0, 0], "trusted": True}})
    memory = StaticSlamPoseMemory({"milk": {"R": np.eye(3).tolist(),
                                             "t_m": [0, 0, 0], "trusted": True}})
    bad = np.eye(4)
    bad[0, 0] = np.nan
    with pytest.raises(ValueError, match="finite 4x4"):
        memory.camera_pose("milk", bad)
