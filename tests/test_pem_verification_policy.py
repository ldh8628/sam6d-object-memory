import importlib.util
import math
from pathlib import Path

import pytest


PATH = Path(__file__).resolve().parents[1] / "tools" / "build_pem_verification_policy.py"
SPEC = importlib.util.spec_from_file_location("build_pem_verification_policy", PATH)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def scored_row(stamp, texture, iou, correct):
    detection = {
        "stamp_ns": stamp, "i": stamp, "object": "milk",
        "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t_mm": [0, 0, 1000],
        "pem": {"depth_valid_frac": 0.9},
    }
    analysis = {
        "stamp_ns": stamp, "frame_index": stamp, "object": "milk",
        "reference_role": "evaluation_frame",
        "stage6000": {"gt_status": "available"},
        "stage300": {"candidates": [{
            "rank_geo": 0,
            "geometry_selected": True, "texture_score": texture,
            "mask_iou": iou, "correct": correct,
        }]},
    }
    return detection, analysis


def test_policy_keeps_geometry_and_separates_time_blocked_rejections():
    first = scored_row(10, 0.6, 0.2, True)
    second = scored_row(20, 0.3, 0.3, False)
    policy = MOD.build_policy(
        [first[0], second[0]], [first[1], second[1]], 0.8, 0.45, 0.42)

    assert policy["selection_method"] == "geometry_only_unchanged"
    assert policy["decisions"]["10:10:milk"]["status"] == "accepted"
    assert policy["decisions"]["10:10:milk"]["split"] == "tune"
    assert policy["decisions"]["20:20:milk"]["status"] == "withheld"
    assert policy["decisions"]["20:20:milk"]["split"] == "holdout"
    assert policy["summary"]["holdout"]["true_reject"] == 1


def test_policy_preserves_boundary_score_precision():
    row = scored_row(10, 0.449561999, 0.2, False)
    policy = MOD.build_policy([row[0]], [row[1]], 0.8, 0.449562, 0.420998)
    decision = policy["decisions"]["10:10:milk"]
    assert decision["texture"] == 0.449561999
    assert decision["textureFailed"] is True


def test_policy_fails_on_unaligned_sources():
    first = scored_row(10, 0.6, 0.2, True)
    second = scored_row(20, 0.3, 0.3, False)
    with pytest.raises(MOD.PolicyInputError, match="row key mismatch"):
        MOD.build_policy([first[0]], [second[1]], 0.8, 0.45, 0.42)


def test_policy_rejects_boolean_scores_and_duplicate_keys():
    first = scored_row(10, 0.6, 0.2, True)
    first[0]["pem"]["depth_valid_frac"] = True
    with pytest.raises(MOD.PolicyInputError, match="duplicate source key"):
        valid = scored_row(10, 0.6, 0.2, True)
        MOD.build_policy([valid[0], valid[0]], [valid[1], valid[1]], 0.8, 0.45, 0.42)
    with pytest.raises(MOD.PolicyInputError, match="invalid depth_valid_frac"):
        MOD.build_policy([first[0]], [first[1]], 0.8, 0.45, 0.42)


@pytest.mark.parametrize("threshold", [True, float("nan"), float("inf"), -0.1, 1.1])
def test_library_policy_validates_thresholds(threshold):
    first = scored_row(10, 0.6, 0.2, True)
    with pytest.raises(MOD.PolicyInputError, match="finite and in"):
        MOD.build_policy([first[0]], [first[1]], threshold, 0.45, 0.42)


def test_duplicate_ineligible_source_rows_are_rejected():
    first = scored_row(10, 0.6, 0.2, True)
    first[1]["reference_role"] = "trusted_reference_member"
    with pytest.raises(MOD.PolicyInputError, match="duplicate source key"):
        MOD.build_policy([first[0], first[0]], [first[1], first[1]], 0.8, 0.45, 0.42)


@pytest.mark.parametrize("rank", [None, 1.5, "1", True])
def test_candidate_rank_identity_must_be_an_explicit_integer(rank):
    row = scored_row(10, 0.6, 0.2, True)
    if rank is None:
        row[1]["stage300"]["candidates"][0].pop("rank_geo")
    else:
        row[1]["stage300"]["candidates"][0]["rank_geo"] = rank
    with pytest.raises(MOD.PolicyInputError, match="rank_geo"):
        MOD.build_policy([row[0]], [row[1]], 0.8, 0.45, 0.42)


def test_detection_fingerprint_is_order_independent_and_pose_sensitive():
    first = scored_row(10, 0.6, 0.2, True)[0]
    second = scored_row(20, 0.3, 0.3, False)[0]
    assert MOD.detection_fingerprint([first, second]) == MOD.detection_fingerprint(
        [second, first])
    changed = {**second, "t_mm": [1, 0, 1000]}
    assert MOD.detection_fingerprint([first, second]) != MOD.detection_fingerprint(
        [first, changed])


def candidate(rank, rotation=None, translation=None, texture=0.6, iou=0.6,
              correct=True, selected=False):
    return {
        "rank_geo": rank, "index300": rank,
        "proposal6000_index": 100 + rank,
        "R": rotation or [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "t_mm": translation or [rank * 100, 0, 1000],
        "geometry_score": 10 - rank,
        "texture_score": texture, "mask_iou": iou,
        "geometry_selected": selected, "candidate_valid": True,
        "projection_valid": True,
        "correct": correct,
    }


def fallback_record(object_name="Bear", candidates=None):
    candidates = candidates or [
        candidate(0, translation=[0, 0, 1000], texture=0.2, iou=0.2,
                  selected=True),
        candidate(1, translation=[10, 0, 1000]),
        candidate(2, translation=[100, 0, 1000]),
    ]
    return {
        "object": object_name, "winner": candidates[0], "candidates": candidates,
        "texture_min": 0.449562, "iou_min": 0.420998,
    }


def test_fallback_excludes_boundary_cluster_and_keeps_original_geometry_order():
    record = fallback_record(candidates=[
        candidate(0, translation=[0, 0, 1000], texture=0.2, iou=0.2,
                  selected=True),
        candidate(1, translation=[25, 0, 1000]),
        candidate(2, translation=[80, 0, 1000]),
        candidate(3, translation=[120, 0, 1000]),
    ])
    fallback = MOD._fallback_candidate(record, 20.0, 25.0)
    assert fallback["excluded_cluster_size"] == 2  # Top-1 and exact 25 mm boundary.
    assert fallback["selected"]["rank_geo"] == 2
    assert fallback["selected"]["proposal6000_index"] == 102
    assert fallback["selected"]["R"] == record["candidates"][2]["R"]
    assert fallback["selected"]["t_mm"] == record["candidates"][2]["t_mm"]


def test_fallback_symmetry_aware_cluster_and_no_candidate_reason():
    quarter_turn = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    record = fallback_record("Sikhye_high", [
        candidate(0, translation=[0, 0, 1000], texture=0.2, iou=0.2,
                  selected=True),
        candidate(1, rotation=quarter_turn, translation=[0, 0, 1000]),
    ])
    assert math.isclose(MOD.symmetry_aware_rotation_distance_deg(
        record["winner"]["R"], quarter_turn, "Sikhye_high"), 0.0, abs_tol=1e-6)
    fallback = MOD._fallback_candidate(record, 10.0, 25.0)
    assert fallback["status"] == "none"
    assert fallback["reason"] == "no_candidate_pose_outside_cluster"
    assert fallback["excluded_cluster_size"] == 2


def test_rotation_distance_rejects_finite_non_so3_matrix():
    with pytest.raises(MOD.PolicyInputError, match=r"valid SO\(3\)"):
        MOD.symmetry_aware_rotation_distance_deg(
            [[2, 0, 0], [0, 1, 0], [0, 0, 1]],
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "Bear")


def test_fallback_requires_both_channels_and_reports_none():
    record = fallback_record(candidates=[
        candidate(0, translation=[0, 0, 1000], texture=0.2, iou=0.2,
                  selected=True),
        candidate(1, translation=[100, 0, 1000], texture=0.8, iou=0.2),
        candidate(2, translation=[200, 0, 1000], texture=0.2, iou=0.8),
    ])
    fallback = MOD._fallback_candidate(record, 20.0, 25.0)
    assert fallback["selected"] is None
    assert fallback["reason"] == "no_candidate_passes_texture_and_iou"


def test_fallback_does_not_skip_higher_rank_when_gt_label_is_missing():
    candidates = [
        candidate(0, translation=[0, 0, 1000], texture=0.2, iou=0.2,
                  selected=True),
        candidate(1, translation=[100, 0, 1000]),
        candidate(2, translation=[200, 0, 1000]),
    ]
    candidates[1]["correct"] = None
    with pytest.raises(MOD.PolicyInputError, match="GT unavailable at rank_geo=1"):
        MOD._fallback_candidate(fallback_record(candidates=candidates), 20.0, 25.0)


def test_fallback_reports_candidate_pose_unavailable():
    invalid = candidate(1, translation=[100, 0, 1000])
    invalid["projection_valid"] = False
    fallback = MOD._fallback_candidate(fallback_record(candidates=[
        candidate(0, translation=[0, 0, 1000], texture=0.2, iou=0.2,
                  selected=True), invalid,
    ]), 20.0, 25.0)
    assert fallback["selected"] is None
    assert fallback["reason"] == "candidate_pose_unavailable"


def policy_rows(stamp, holdout_alternative_correct=True):
    detection_row, analysis = scored_row(stamp, 0.2, 0.2, True)
    alternatives = [
        candidate(0, translation=[0, 0, 1000], texture=0.2, iou=0.2,
                  selected=True, correct=True),
        candidate(1, translation=[100, 0, 1000], texture=0.8, iou=0.8,
                  correct=holdout_alternative_correct),
    ]
    analysis["stage300"]["candidates"] = alternatives
    return detection_row, analysis


def test_tune_threshold_is_not_changed_by_holdout_outcomes():
    tune_a = policy_rows(10)
    tune_b = policy_rows(20)
    holdout_a = policy_rows(30, True)
    holdout_b = policy_rows(40, True)
    detections = [row[0] for row in (tune_a, tune_b, holdout_a, holdout_b)]
    analyses = [row[1] for row in (tune_a, tune_b, holdout_a, holdout_b)]
    first = MOD.build_policy(detections, analyses, 0.8, 0.449562, 0.420998)

    changed_holdout_a = policy_rows(30, False)
    changed_holdout_b = policy_rows(40, False)
    changed_analyses = [tune_a[1], tune_b[1],
                        changed_holdout_a[1], changed_holdout_b[1]]
    second = MOD.build_policy(detections, changed_analyses, 0.8, 0.449562, 0.420998)

    assert first["fallback_shadow"]["selected_thresholds"] == second[
        "fallback_shadow"]["selected_thresholds"]
    assert first["fallback_shadow"]["summary"]["holdout"] != second[
        "fallback_shadow"]["summary"]["holdout"]
    selected_tune = next(row["tune"] for row in first["fallback_shadow"]["tune_sweep"]
                         if row["selected"])
    assert all("holdout" not in row for row in first[
        "fallback_shadow"]["tune_sweep"])
    assert selected_tune["accepted_accuracy"] >= 0.90
    assert selected_tune["correct_to_incorrect"] == 0
