import importlib.util
from pathlib import Path

import numpy as np


PATH = Path(__file__).resolve().parents[1] / "tools" / "evaluate_slam_pose.py"
SPEC = importlib.util.spec_from_file_location("evaluate_slam_pose", PATH)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def test_declared_axial_symmetry_is_folded():
    turn = MOD.axis_rotation((0.0, 0.0, 1.0), 170)
    assert MOD.angle_deg(np.eye(3), turn, "Sikhye_high") < 1e-6
    assert MOD.angle_deg(np.eye(3), turn, "milk") > 160.0


def test_nearest_pose_enforces_time_gate():
    times = np.array([1.0, 2.0])
    poses = np.stack([np.eye(4), np.eye(4)])
    pose, dt = MOD.nearest_pose(1_010_000_000, times, poses, max_dt=0.02)
    assert pose is not None and abs(dt - 0.01) < 1e-9
    pose, _ = MOD.nearest_pose(1_100_000_000, times, poses, max_dt=0.02)
    assert pose is None


def test_pseudo_gt_selects_largest_stable_cluster():
    rows = []
    for i in range(8):
        rows.append({"object": "milk", "stamp_ns": i, "i": i, "score": 0.9,
                     "map_R": np.eye(3), "map_t": np.array([1.0 + i * 0.001, 0.0, 0.0])})
    wrong = MOD.axis_rotation((0, 1, 0), 180)
    for i in range(3):
        rows.append({"object": "milk", "stamp_ns": 100 + i, "i": 100 + i, "score": 0.9,
                     "map_R": wrong, "map_t": np.array([1.0, 0.0, 0.0])})
    gt, _ = MOD.build_pseudo_gt(rows, min_cluster=5, min_share=0.25)
    assert gt["milk"]["trusted"] is True
    assert gt["milk"]["cluster_size"] == 8
    assert MOD.angle_deg(np.asarray(gt["milk"]["R"]), np.eye(3), "milk") < 1e-6


def test_direct_neighborhood_does_not_chain_beyond_rotation_gate():
    rows = []
    for i, degrees in enumerate((0, 20, 40, 60)):
        rows.append({"object": "milk", "stamp_ns": i, "i": i,
                     "map_R": MOD.axis_rotation((0, 1, 0), degrees),
                     "map_t": np.zeros(3)})
    gt, assignments = MOD.build_pseudo_gt(
        rows, min_cluster=1, min_share=0.1, rot_gate=30.0)

    assert gt["milk"]["center"]["maximum_direct_neighbor_count"] == 3
    assert gt["milk"]["cluster_size"] <= 3
    member_stamps = [row["stamp_ns"] for row in gt["milk"]["reference_frames"]]
    center_stamp = gt["milk"]["center"]["stamp_ns"]
    center = next(row for row in rows if row["stamp_ns"] == center_stamp)
    assert all(MOD.angle_deg(center["map_R"], next(
        row["map_R"] for row in rows if row["stamp_ns"] == stamp), "milk") <= 30.0
               for stamp in member_stamps)
    assert {value["status"] for value in assignments["milk"].values()} <= {
        "reference_member", "quality_selected", "quality_rejected"}


def test_adaptive_quality_gate_relaxes_joint_threshold_and_records_it():
    rows = []
    for i in range(20):
        rows.append({"object": "milk", "stamp_ns": i,
                     "bbox_area_px": float(i + 1),
                     "crop_sharpness": float(i + 1),
                     "quality_rejection": None})
    selected, quality = MOD.adaptive_quality_gate(rows, min_samples=10)

    assert len(selected) >= 10
    assert quality["selected_quantile"] == 0.5
    assert quality["status"] == "selected"


def test_quality_filtered_pseudo_gt_is_untrusted_when_valid_samples_are_sparse():
    rows = [{"object": "milk", "stamp_ns": i, "i": i,
             "map_R": np.eye(3), "map_t": np.zeros(3),
             "bbox_area_px": 100.0, "crop_sharpness": 100.0,
             "quality_rejection": None} for i in range(4)]
    gt, _ = MOD.build_pseudo_gt(rows, quality_filter=True, min_quality_samples=10)

    assert gt["milk"]["trusted"] is False
    assert gt["milk"]["unavailable_reason"] == "insufficient_quality"


def test_quality_assignments_record_each_failed_axis():
    rows = [{"object": "milk", "stamp_ns": i, "i": i,
             "map_R": np.eye(3), "map_t": np.zeros(3),
             "bbox_area_px": float(i + 1), "crop_sharpness": float(20 - i),
             "quality_rejection": None} for i in range(20)]
    _gt, assignments = MOD.build_pseudo_gt(
        rows, quality_filter=True, min_quality_samples=10, min_cluster=1,
        min_share=0.0)

    rejected = [value for value in assignments["milk"].values()
                if value["status"] == "quality_rejected"]
    assert rejected
    assert all(value["reasons"] for value in rejected)
    assert all("bbox_area_threshold_px" in value and
               "crop_sharpness_threshold" in value for value in rejected)


def test_load_trajectory_rejects_nonmonotonic_and_bad_quaternion(tmp_path):
    path = tmp_path / "trajectory.txt"
    path.write_text("2 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n", encoding="utf-8")
    with np.testing.assert_raises_regex(ValueError, "strictly increasing"):
        MOD.load_trajectory(path)
    path.write_text("1 0 0 0 0 0 0 2\n2 0 0 0 0 0 0 2\n", encoding="utf-8")
    with np.testing.assert_raises_regex(ValueError, "normalized"):
        MOD.load_trajectory(path)


def test_validate_reference_rows_rejects_duplicate_and_nonrigid():
    row = {"object": "milk", "stamp_ns": 1, "R": np.eye(3).tolist(),
           "t_mm": [0, 0, 1]}
    with np.testing.assert_raises_regex(ValueError, "duplicate"):
        MOD.validate_reference_rows([row, row])
    bad = {**row, "stamp_ns": 2, "R": np.zeros((3, 3)).tolist()}
    with np.testing.assert_raises_regex(ValueError, "invalid rigid pose"):
        MOD.validate_reference_rows([bad])


def test_evaluate_separates_reference_members():
    rows = [{"object": "milk", "stamp_ns": i, "i": i, "score": 1.0,
             "map_R": np.eye(3), "map_t": np.zeros(3)} for i in (1, 2)]
    gt = {"milk": {"trusted": True, "R": np.eye(3).tolist(), "t_m": [0, 0, 0]}}
    result, cases = MOD.evaluate("geometry", rows, gt, {("milk", 1)})

    assert result["overall"]["n"] == 2
    assert result["evaluation_only"]["n"] == 1
    assert result["reference_members"]["n"] == 1
    assert {case["reference_role"] for case in cases} == {
        "reference_member", "evaluation_frame"}


def test_causal_memory_scores_only_after_anchor_initialization():
    rows = [{"object": "milk", "stamp_ns": i, "i": i, "score": 0.9,
             "map_R": np.eye(3), "map_t": np.array([1.0, 0.0, 0.0])}
            for i in range(12)]
    gt, _ = MOD.build_pseudo_gt(rows, min_cluster=5, min_share=0.5)

    result = MOD.evaluate_causal_pose_memory(rows, gt, min_cluster=3)

    assert result["initialized"]["milk"]["observations_seen"] == 3
    assert result["post_initialization_outputs"] == 9
    assert result["overall"]["pose_success_pct"] == 100.0
