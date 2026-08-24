import sys
from pathlib import Path

import numpy as np
import pytest
import torch


ROOT = Path(__file__).resolve().parents[1]
PEM = ROOT / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
sys.path.insert(0, str(PEM / "model" / "pointnet2"))
sys.path.insert(0, str(PEM / "utils"))

import model_utils  # noqa: E402
from realtime.pem_explorer_record import CANDIDATE_BYTES, pack_candidates  # noqa: E402


TEXTURE_MEASURED = 1 << 6


def _shape_metrics(candidate_count):
    mask_iou = torch.full((1, candidate_count), 0.1)
    mask_iou[0, 0] = 0.8
    return {
        "projection_valid": torch.ones(1, candidate_count, dtype=torch.bool),
        "mask_iou": mask_iou,
        "coverage": torch.full((1, candidate_count), 0.5),
        "size_ratio": torch.ones(1, candidate_count),
        "rendered_mask_area_px": torch.full((1, candidate_count), 4),
        "observed_mask_area_px": torch.tensor([4]),
        "observed_bbox_area_px": torch.tensor([4.0]),
        "rendered_bbox_area_px": torch.full((1, candidate_count), 4.0),
        "observed_mask_rle": [[0, 4]],
        "observed_bbox_224": [[0, 0, 1, 1]],
        "mask_size": [2, 2],
        "point_splat_radius_px": 1,
        "closing_radius_px": 0,
    }


def _inputs(profile=None, *, production=True):
    candidate_count = 3
    rotations = torch.eye(3).reshape(1, 1, 3, 3).repeat(1, candidate_count, 1, 1)
    translations = torch.zeros(1, candidate_count, 3)
    geometry = torch.tensor([[3.0, 2.0, 1.0]])
    points = torch.tensor([[[-0.1, 0.0, 1.0], [0.1, 0.0, 1.0]]])
    explorer = {"enabled": True}
    if profile is not None:
        explorer["capture_profile"] = profile
    appearance = {
        "topk": 1,
        "stride": 1,
        "verify": {
            "enabled": production,
            "mask_iou_min": 0.420998,
            "texture_min_score": 0.449562,
        },
        "diagnostic": {"enabled": True, "explorer_v2": explorer},
        "dense_pm": points,
        "dense_fm": torch.tensor([[[1.0, 0.0], [0.0, 1.0]]]),
        "dense_po": points,
        "dense_fo": torch.tensor([[[1.0, 0.0], [0.0, 1.0]]]),
        "radius": torch.ones(1),
        "_proposal_ids": torch.tensor([[10, 11, 12]]),
        "source_pixel_index": torch.tensor([[1, 2]]),
        "cad_sample_index": torch.tensor([[3, 4]]),
        "coarse_fps_idx": torch.tensor([[0, 1]]),
        "_geo_point_valid": torch.tensor([[True, True]]),
        "crop_bbox_yxyx": torch.tensor([[0.0, 0.0, 2.0, 2.0]]),
    }
    return rotations, translations, geometry, appearance


def _run(monkeypatch, profile=None, *, production=True):
    monkeypatch.setattr(
        model_utils, "_candidate_shape_metrics",
        lambda *_args, **_kwargs: _shape_metrics(3))
    rotations, translations, geometry, appearance = _inputs(
        profile, production=production)
    info = {}
    selected = model_utils.independent_candidate_verify(
        rotations, translations, geometry, appearance, info)
    return selected, info


def test_production_profiles_preserve_selection_and_bound_shadow_texture(monkeypatch):
    real_cdist = torch.cdist
    measured_batch_sizes = []

    def tracked_cdist(*args, **kwargs):
        measured_batch_sizes.append(args[0].shape[0])
        return real_cdist(*args, **kwargs)

    monkeypatch.setattr(model_utils.torch, "cdist", tracked_cdist)
    selected_full, full = _run(
        monkeypatch, "exhaustive_visualization", production=True)
    full_measurements = measured_batch_sizes[:]
    measured_batch_sizes.clear()
    selected_realtime, realtime = _run(
        monkeypatch, "realtime_inference", production=True)
    realtime_measurements = measured_batch_sizes[:]
    measured_batch_sizes.clear()

    off_inputs = _inputs(production=True)
    off_inputs[3].pop("diagnostic")
    off = {}
    selected_off = model_utils.independent_candidate_verify(*off_inputs, off)

    assert torch.equal(selected_full, selected_realtime)
    assert torch.equal(selected_full, selected_off)
    assert full["verify"] == realtime["verify"]
    assert full["verify"] == off["verify"]
    assert full_measurements == [1, 2]
    assert realtime_measurements == [1]
    assert measured_batch_sizes == [1]

    full_payload = full["pem_explorer"][0]
    realtime_payload = realtime["pem_explorer"][0]
    assert np.isfinite(full_payload["texture"]).tolist() == [True, True, True]
    assert np.isfinite(realtime_payload["texture"]).tolist() == [True, False, False]
    assert (full_payload["flags"] & TEXTURE_MEASURED).tolist() == [64, 64, 64]
    assert (realtime_payload["flags"] & TEXTURE_MEASURED).tolist() == [64, 0, 0]
    assert CANDIDATE_BYTES == 80
    assert len(pack_candidates(realtime_payload)) == 3 * CANDIDATE_BYTES


def test_realtime_profile_does_not_expand_geometry_only_texture_measurement(monkeypatch):
    selected_full, full = _run(
        monkeypatch, "exhaustive_visualization", production=False)
    selected_realtime, realtime = _run(
        monkeypatch, "realtime_inference", production=False)

    assert torch.equal(selected_full, selected_realtime)
    assert np.isfinite(full["pem_explorer"][0]["texture"]).sum() == 3
    assert np.isfinite(realtime["pem_explorer"][0]["texture"]).sum() == 1
    assert (realtime["pem_explorer"][0]["flags"] & TEXTURE_MEASURED).tolist() == [64, 0, 0]


def test_missing_profile_preserves_exhaustive_behavior(monkeypatch):
    _, info = _run(monkeypatch, production=True)
    assert np.isfinite(info["pem_explorer"][0]["texture"]).all()


def test_unknown_capture_profile_is_rejected():
    rotations, translations, geometry, appearance = _inputs("not-a-profile")
    with pytest.raises(ValueError, match="capture_profile"):
        model_utils.independent_candidate_verify(
            rotations, translations, geometry, appearance, {})
