import json
from pathlib import Path

import numpy as np
import pytest

from realtime.slam_pose_memory import (ObjectAnchorManager, StaticSlamPoseMemory,
                                       pose_matrix)


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


def _observe(manager, object_id, pose, stamp):
    return manager.observe(object_id, np.eye(4), pose, "TRACKING_OK", stamp, stamp)


def test_anchor_registers_at_16_of_20_but_not_15_of_20():
    converged = pose_matrix(np.eye(3), [0.0, 0.0, 1.0])
    outlier = pose_matrix(np.eye(3), [0.2, 0.0, 1.0])
    manager = ObjectAnchorManager("map-a")
    result = None
    for index in range(20):
        result = _observe(manager, "milk", converged if index < 16 else outlier, index)
    assert result["registered"] is True
    assert "milk" in manager.anchors
    assert any(np.array_equal(manager.anchors["milk"], pose)
               for pose in [converged] * 16 + [outlier] * 4)

    manager = ObjectAnchorManager("map-a")
    for index in range(20):
        result = _observe(manager, "milk", converged if index < 15 else outlier, index)
    assert result["registered"] is False
    assert "milk" not in manager.anchors


def test_anchor_resets_on_map_change_and_releases_on_four_of_five_mismatches():
    pose = pose_matrix(np.eye(3), [0.0, 0.0, 1.0])
    moved = pose_matrix(np.eye(3), [0.2, 0.0, 1.0])
    manager = ObjectAnchorManager("map-a")
    for index in range(20):
        _observe(manager, "milk", pose, index)
    assert manager.set_map("map-b") is True
    assert not manager.anchors and not manager.histories

    for index in range(20):
        _observe(manager, "milk", pose, index)
    for mismatch in [True, True, False, True, True]:
        result = manager.validate_shadow("milk", moved if mismatch else pose)
    assert result["released"] is True
    assert "milk" not in manager.anchors


def test_low_mask_iou_alone_does_not_release_anchor():
    pose = pose_matrix(np.eye(3), [0.0, 0.0, 1.0])
    manager = ObjectAnchorManager("map-a")
    for index in range(20):
        _observe(manager, "milk", pose, index)
    for _ in range(8):
        result = manager.validate_shadow("milk", pose, mask_only_failure=True)
    assert result["registered"] is True
    assert "milk" in manager.anchors
    assert len(manager.validations["milk"]) == 0


def test_anchor_output_modes_differ_when_ism_is_missing():
    pose = pose_matrix(np.eye(3), [0.0, 0.0, 1.0])
    points = np.asarray([[x, y, 0.0] for x in (-0.1, 0.1) for y in (-0.1, 0.1)])
    K = np.asarray([[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]])
    manager = ObjectAnchorManager("map-a")
    manager.anchors["milk"] = pose
    associated, associated_diag = manager.output_decision(
        "milk", np.eye(4), points, K, (100, 100), mode="ism_associated")
    always, always_diag = manager.output_decision(
        "milk", np.eye(4), points, K, (100, 100), mode="fov_always")
    assert associated is None
    assert associated_diag["reason"] == "ism_association_missing"
    assert np.allclose(always, pose)
    assert always_diag["output"] is True


@pytest.mark.parametrize("config", [
    {"window": 0},
    {"window": 20, "register_count": 21},
    {"release_window": 5, "release_count": 6},
    {"sym_step_deg": 7},
    {"fov_fraction_min": 1.1},
    {"timestamp_tolerance_ms": -1},
])
def test_anchor_rejects_impossible_state_machine_config(config):
    with pytest.raises(ValueError):
        ObjectAnchorManager("map-a", config)
