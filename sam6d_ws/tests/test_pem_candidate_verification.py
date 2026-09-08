import sys
from pathlib import Path

import pytest
import torch


ROOT = Path(__file__).resolve().parents[1]
PEM = ROOT / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
sys.path.insert(0, str(PEM / "model" / "pointnet2"))
sys.path.insert(0, str(PEM / "utils"))

from model_utils import (  # noqa: E402
    _candidate_shape_metrics,
    _compact_distribution,
    _stage6000_score_analysis,
    independent_candidate_verify,
    sequential_candidate_select,
    validate_refined_poses,
)


def test_missing_mask_rejects_detection_without_inventing_pose():
    rotations = torch.eye(3).reshape(1, 1, 3, 3).repeat(1, 3, 1, 1)
    translations = torch.tensor([[[0.0, 0.0, 0.0],
                                  [0.2, 0.0, 0.0],
                                  [-0.2, 0.0, 0.0]]])
    geometry = torch.tensor([[1.0, 3.0, 2.0]])
    points = torch.tensor([[[-0.1, 0.0, 1.0], [0.1, 0.0, 1.0]]])
    features = torch.tensor([[[1.0, 0.0], [0.0, 1.0]]])
    appearance = {
        "topk": 3,
        "stride": 1,
        "verify": {"enabled": True, "texture_max_rank": 0},
        "dense_pm": points,
        "dense_fm": features,
        "dense_po": points,
        "dense_fo": features,
        "radius": torch.ones(1),
        "_proposal_ids": torch.tensor([[101, 102, 103]]),
    }
    info = {}
    selected = independent_candidate_verify(
        rotations, translations, geometry, appearance, info)
    assert selected.tolist() == [1]
    assert info["verify"][0]["selection_method"] == "geometry_mask_texture_convergence"
    assert info["verify"][0]["accepted"] is False
    assert info["verify"][0]["rejection_reason"] == "candidate_projection_invalid"
    assert info["verify"][0]["selected_proposal6000_index"] is None


def _selector_shape(mask_iou, valid=None):
    mask_iou = torch.tensor([mask_iou], dtype=torch.float32)
    return {"mask_iou": mask_iou,
            "projection_valid": (torch.ones_like(mask_iou, dtype=torch.bool)
                                 if valid is None else torch.tensor([valid]))}


def test_sequential_threshold_boundaries_and_stage_reasons():
    rotations = torch.eye(3).reshape(1, 1, 3, 3).repeat(1, 3, 1, 1)
    translations = torch.tensor([[[0.0, 0.0, 1.0], [0.01, 0.0, 1.0],
                                  [0.02, 0.0, 1.0]]])
    geometry = torch.tensor([[3.0, 2.0, 1.0]])
    texture = torch.tensor([[0.449562, 0.449562, 0.449561]])
    shape = _selector_shape([0.420998, 0.420997, 0.8])
    selected, rows = sequential_candidate_select(
        rotations, translations, geometry, texture, shape, ["Bear"], {
            "mask_iou_min": 0.420998, "texture_min_score": 0.449562,
            "cluster_rotation_deg": 20, "cluster_translation_mm": 25,
            "cluster_min_occupancy": 0.5,
        })
    assert selected.tolist() == [0]
    assert rows[0]["accepted"] is True
    assert rows[0]["mask_survivors"] == 2
    assert rows[0]["texture_survivors"] == 1

    _, rows = sequential_candidate_select(
        rotations, translations, geometry, torch.full_like(texture, 0.1),
        _selector_shape([0.8, 0.8, 0.8]), ["Bear"], {})
    assert rows[0]["rejection_reason"] == "texture_filter_empty"

    _, rows = sequential_candidate_select(
        rotations, translations, geometry, torch.full_like(texture, 0.8),
        _selector_shape([0.1, 0.2, 0.3]), ["Bear"], {})
    assert rows[0]["rejection_reason"] == "mask_filter_empty"

    separated = torch.tensor([[[0.0, 0.0, 1.0], [0.1, 0.0, 1.0],
                               [0.2, 0.0, 1.0]]])
    _, rows = sequential_candidate_select(
        rotations, separated, geometry, torch.full_like(texture, 0.8),
        _selector_shape([0.8, 0.8, 0.8]), ["Bear"], {})
    assert rows[0]["rejection_reason"] == "convergence_insufficient"


def test_invalid_projection_and_nonfinite_geometry_are_rejected_before_mask():
    rotations = torch.eye(3).reshape(1, 1, 3, 3)
    translations = torch.tensor([[[0.0, 0.0, 1.0]]])
    _, rows = sequential_candidate_select(
        rotations, translations, torch.tensor([[float("nan")]]),
        torch.tensor([[1.0]]), _selector_shape([1.0]), ["Bear"], {})
    assert rows[0]["rejection_reason"] == "candidate_projection_invalid"


def test_symmetry_aware_dominant_cluster_selects_best_actual_geometry_candidate():
    quarter = torch.tensor([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0]])
    rotations = torch.stack([torch.eye(3), quarter, torch.eye(3)])[None]
    translations = torch.tensor([[[0.0, 0.0, 1.0], [0.01, 0.0, 1.0],
                                  [0.2, 0.0, 1.0]]])
    selected, rows = sequential_candidate_select(
        rotations, translations, torch.tensor([[3.0, 2.0, 1.0]]),
        torch.tensor([[0.8, 0.8, 0.8]]), _selector_shape([0.8, 0.8, 0.8]),
        ["symmetric"], {"symmetry_axes": {"symmetric": [0, 0, 1]},
                        "sym_step_deg": 90, "cluster_rotation_deg": 20,
                        "cluster_translation_mm": 25, "cluster_min_occupancy": 0.5})
    assert rows[0]["cluster_size"] == 2
    assert rows[0]["cluster_occupancy"] == pytest.approx(2 / 3)
    assert selected.tolist() == [0]


@pytest.mark.parametrize(("pose_score", "iou", "feature", "accepted", "reason"), [
    (float("nan"), 0.8, 1.0, False, "fine_refinement_failed"),
    (1.0, 0.2, 1.0, False, "final_mask_below_threshold"),
    (1.0, 0.8, -1.0, False, "final_texture_below_threshold"),
    (1.0, 0.8, 1.0, True, None),
])
def test_fine_pose_has_no_coarse_fallback(monkeypatch, pose_score, iou, feature,
                                          accepted, reason):
    import model_utils as module
    monkeypatch.setattr(module, "_candidate_shape_metrics", lambda *args: {
        "projection_valid": torch.tensor([[True]]),
        "mask_iou": torch.tensor([[iou]]),
    })
    rows = [{"accepted": True, "rejection_reason": None}]
    appearance = {
        "verify": {"enabled": True, "mask_iou_min": 0.420998,
                   "texture_min_score": 0.449562},
        "dense_pm": torch.tensor([[[0.0, 0.0, 0.0]]]),
        "dense_fm": torch.tensor([[[1.0, 0.0]]]),
        "dense_po": torch.tensor([[[0.0, 0.0, 0.0]]]),
        "dense_fo": torch.tensor([[[feature, 0.0]]]),
        "radius": torch.ones(1), "_projection_model_pts": torch.zeros(1, 1, 3),
    }
    validate_refined_poses(
        torch.eye(3)[None], torch.tensor([[0.0, 0.0, 1.0]]),
        torch.tensor([pose_score]), appearance, rows)
    assert rows[0]["accepted"] is accepted
    assert rows[0]["rejection_reason"] == reason


def test_geometry_tie_uses_same_first_index_as_production_max():
    rotations = torch.eye(3).reshape(1, 1, 3, 3).repeat(1, 3, 1, 1)
    translations = torch.zeros(1, 3, 3)
    geometry = torch.ones(1, 3)
    points = torch.tensor([[[-0.1, 0.0, 1.0], [0.1, 0.0, 1.0]]])
    features = torch.tensor([[[1.0, 0.0], [0.0, 1.0]]])
    info = {}
    selected = independent_candidate_verify(rotations, translations, geometry, {
        "topk": 3, "stride": 1,
        "dense_pm": points, "dense_fm": features,
        "dense_po": points, "dense_fo": features,
        "radius": torch.ones(1),
        "_proposal_ids": torch.tensor([[101, 102, 103]]),
        "names": ["Bear"],
        "diagnostic_ref_valid": torch.tensor([True]),
        "diagnostic_ref_R": torch.eye(3).reshape(1, 3, 3),
        "diagnostic_ref_t": torch.zeros(1, 3),
        "diagnostic": {"enabled": True,
                       "rotation_threshold_deg": 30.0,
                       "translation_threshold_mm": 100.0,
                       "score_analysis": {"enabled": True}},
    }, info)
    assert selected.tolist() == [0]
    assert info["score_analysis"][0]["stage300"]["selected_index300"] == 0
    assert info["verify"][0]["selected_proposal6000_index"] == 101


def test_shape_metrics_reject_render_smaller_than_input_and_bound_iou():
    rotations = torch.eye(3).reshape(1, 1, 3, 3)
    translations = torch.tensor([[[0.0, 0.0, 1.0]]])
    model = torch.tensor([[[-0.1, -0.1, 0.0], [0.1, -0.1, 0.0],
                           [0.1, 0.1, 0.0], [-0.1, 0.1, 0.0]]])
    observed = torch.zeros(1, 224, 224, dtype=torch.bool)
    observed[:, 80:145, 80:145] = True
    appearance = {
        "shape_mask": observed,
        "K": torch.tensor([[[100.0, 0.0, 112.0],
                             [0.0, 100.0, 112.0],
                             [0.0, 0.0, 1.0]]]),
        "crop_bbox_yxyx": torch.tensor([[0.0, 224.0, 0.0, 224.0]]),
        "verify": {"projection_chunk": 1, "point_splat_radius_px": 2},
    }
    metrics = _candidate_shape_metrics(
        rotations, translations, model, appearance)
    assert metrics["size_ratio"].item() > 1.0
    assert 0.0 <= metrics["mask_iou"].item() <= 1.0
    assert 0.0 <= metrics["coverage"].item() <= 1.0
    assert metrics["projection_valid"].item() is True


def test_shape_projection_behind_camera_is_explicitly_invalid():
    rotations = torch.eye(3).reshape(1, 1, 3, 3)
    translations = torch.tensor([[[0.0, 0.0, -2.0]]])
    model = torch.tensor([[[0.0, 0.0, 0.0], [0.1, 0.1, 0.0]]])
    observed = torch.zeros(1, 224, 224, dtype=torch.bool)
    observed[:, 100:120, 100:120] = True
    metrics = _candidate_shape_metrics(rotations, translations, model, {
        "shape_mask": observed,
        "K": torch.eye(3).reshape(1, 3, 3),
        "crop_bbox_yxyx": torch.tensor([[0.0, 224.0, 0.0, 224.0]]),
        "verify": {},
    })
    assert metrics["projection_valid"].item() is False
    assert metrics["rendered_bbox_area_px"].item() == 0.0


def test_shape_projection_wholly_off_crop_is_invalid():
    rotations = torch.eye(3).reshape(1, 1, 3, 3)
    translations = torch.tensor([[[100.0, 0.0, 1.0]]])
    model = torch.tensor([[[0.0, 0.0, 0.0], [0.1, 0.1, 0.0]]])
    observed = torch.zeros(1, 224, 224, dtype=torch.bool)
    observed[:, 100:120, 100:120] = True
    metrics = _candidate_shape_metrics(rotations, translations, model, {
        "shape_mask": observed,
        "K": torch.tensor([[[100.0, 0.0, 112.0],
                              [0.0, 100.0, 112.0], [0.0, 0.0, 1.0]]]),
        "crop_bbox_yxyx": torch.tensor([[0.0, 224.0, 0.0, 224.0]]),
        "verify": {},
    })
    assert metrics["projection_valid"].item() is False
    assert metrics["rendered_bbox_area_px"].item() == 0.0
    assert metrics["rendered_mask_area_px"].item() == 0.0


def test_score_analysis_measures_all_candidates_without_changing_winner():
    rotations = torch.eye(3).reshape(1, 1, 3, 3).repeat(1, 3, 1, 1)
    translations = torch.tensor([[[0.0, 0.0, 0.0],
                                  [0.2, 0.0, 0.0],
                                  [-0.2, 0.0, 0.0]]])
    geometry = torch.tensor([[1.0, 3.0, 2.0]])
    points = torch.tensor([[[-0.1, 0.0, 1.0], [0.1, 0.0, 1.0]]])
    features = torch.tensor([[[1.0, 0.0], [0.0, 1.0]]])
    base = {
        "topk": 1, "stride": 1,
        "dense_pm": points, "dense_fm": features,
        "dense_po": points, "dense_fo": features,
        "radius": torch.ones(1),
        "_proposal_ids": torch.tensor([[101, 102, 103]]),
        "names": ["Bear"],
        "diagnostic_ref_valid": torch.tensor([True]),
        "diagnostic_ref_R": torch.eye(3).reshape(1, 3, 3),
        "diagnostic_ref_t": torch.zeros(1, 3),
    }
    selected_off = independent_candidate_verify(
        rotations, translations, geometry, base, {})
    info = {}
    selected_on = independent_candidate_verify(rotations, translations, geometry, {
        **base,
        "diagnostic": {
            "enabled": True,
            "rotation_threshold_deg": 30.0,
            "translation_threshold_mm": 100.0,
            "score_analysis": {"enabled": True},
        },
    }, info)
    assert selected_off.tolist() == selected_on.tolist() == [1]
    stage = info["score_analysis"][0]["stage300"]
    assert stage["selection_method"] == "geometry_only"
    assert stage["candidate_count"] == 3
    assert len(stage["candidates"]) == 3
    assert [candidate["rank_geo"] for candidate in stage["candidates"]] == [0, 1, 2]
    assert all(len(candidate["R"]) == 3 and all(len(row) == 3 for row in candidate["R"])
               for candidate in stage["candidates"])
    assert stage["candidates"][0]["t_mm"] == [200.0, 0.0, 0.0]
    assert sum(x["correct"] for x in stage["candidates"]) == 1
    assert all(x["mask_iou"] is None for x in stage["candidates"])
    assert all(x["missing_reason"] == "projection_metadata_or_mask_unavailable"
               for x in stage["candidates"])


def test_stage6000_analysis_is_aggregate_and_symmetry_aware():
    rotations = torch.eye(3).reshape(1, 1, 3, 3).repeat(1, 4, 1, 1)
    translations = torch.tensor([[[0.0, 0.0, 0.0], [0.01, 0.0, 0.0],
                                  [0.2, 0.0, 0.0], [0.3, 0.0, 0.0]]])
    residual = torch.tensor([[0.0, 0.001, 0.02, 0.5]])
    rows = _stage6000_score_analysis(rotations, translations, residual, {
        "names": ["Bear"],
        "diagnostic": {"enabled": True, "rotation_threshold_deg": 30.0,
                       "translation_threshold_mm": 100.0},
        "diagnostic_ref_valid": torch.tensor([True]),
        "diagnostic_ref_R": torch.eye(3).reshape(1, 3, 3),
        "diagnostic_ref_t": torch.zeros(1, 3),
    }, torch.ones(1))
    row = rows[0]
    assert row["score_name"] == "initial_3point_residual"
    assert "not a PEM geometry score" in row["score_semantics"]
    assert row["candidate_count"] == 4
    assert row["correct"]["count"] == 2
    assert row["incorrect"]["count"] == 2
    assert "values" not in row["overall"]


def test_compact_residual_histogram_count_invariant():
    value = _compact_distribution(torch.tensor(
        [0.0, 0.001, 0.999, 1.0, 2.0, float("nan"), -0.1]))
    assert value["count"] == 5
    assert value["missing_count"] == 2
    assert sum(value["bins"]) + value["overflow"] == value["count"]
