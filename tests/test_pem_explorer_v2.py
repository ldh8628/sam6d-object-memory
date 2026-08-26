import json
from pathlib import Path

import numpy as np
import pytest

from realtime.pem_explorer_record import (
    CANDIDATE_BYTES, FLAG_TEXTURE_MEASURED, ExplorerRecorder, pack_candidates,
    pack_replay, read_attempt, unpack_candidates,
)
from tools.serve_pem_explorer import classify_run, discover_runs, json_safe, safe_run


def candidate_payload(count=300):
    rng = np.random.default_rng(42)
    return {
        "R": rng.normal(size=(count, 3, 3)).astype(np.float32),
        "t_m": rng.normal(size=(count, 3)).astype(np.float32),
        "geometry": rng.normal(size=count).astype(np.float32),
        "mask_iou": rng.random(count).astype(np.float32),
        "texture": rng.normal(size=count).astype(np.float32),
        "coverage": rng.random(count).astype(np.float32),
        "size_ratio": rng.random(count).astype(np.float32),
        "proposal6000": np.arange(count, dtype=np.uint32) + 5000,
        "index300": np.arange(count, dtype=np.uint16),
        "rank_geo": np.arange(count, dtype=np.uint16)[::-1],
        "rank_texture": np.arange(count, dtype=np.uint16),
        "flags": np.arange(count, dtype=np.uint16) & 63,
    }


def replay_payload():
    return {
        "source_pixel_index": np.arange(2048, dtype=np.uint32) * 3,
        "coarse_fps_index": np.arange(196, dtype=np.uint16),
        "coarse_valid": np.arange(196) % 3 != 0,
        "cad_sample_index": np.arange(1024, dtype=np.uint16) * 7,
    }


def test_candidate_binary_is_fixed_width_and_float32_bit_exact():
    payload = candidate_payload()
    # Preserve a non-canonical float32 NaN payload as well as ordinary values.
    payload["texture"].view(np.uint32)[7] = np.uint32(0x7FC01234)
    blob = pack_candidates(payload)
    assert CANDIDATE_BYTES == 80
    assert len(blob) == 300 * 80
    rows = unpack_candidates(blob)
    for key in ("geometry", "mask_iou", "texture", "coverage", "size_ratio"):
        actual = np.asarray([row[key] for row in rows], np.float32)
        assert np.array_equal(actual.view(np.uint32), payload[key].view(np.uint32))
    actual_r = np.stack([row["R"] for row in rows])
    actual_t = np.stack([row["t_m"] for row in rows])
    assert np.array_equal(actual_r.view(np.uint32), payload["R"].view(np.uint32))
    assert np.array_equal(actual_t.view(np.uint32), payload["t_m"].view(np.uint32))


def test_full_attempt_round_trip_is_under_40_kib_and_rejection_is_indexed(tmp_path):
    run = tmp_path / "rejected-run"
    recorder = ExplorerRecorder(run, {"test": True})
    attempt = {
        "object": "Bear", "bbox": [1, 2, 30, 40], "crop_bbox_yxyx": [2, 40, 1, 30],
        "explorer_candidates": candidate_payload(), "explorer_replay": replay_payload(),
        "decision": {"accepted": False, "rejection_reason": "texture_filter_empty",
                     "mask_survivors": 11, "texture_survivors": 0},
    }
    recorder.record_frame(123, 124, 4, np.eye(3), (48, 64), [attempt], None)
    recorder.close(completed=True)
    row = json.loads((run / "explorer_index.jsonl").read_text())
    assert row["accepted"] is False
    assert row["rejection_reason"] == "texture_filter_empty"
    assert row["candidate_bytes"] + row["replay_bytes"] <= 40 * 1024
    candidates, replay = read_attempt(run, row)
    assert len(candidates) == 300
    assert np.array_equal(replay["source_pixel_index"], replay_payload()["source_pixel_index"])
    manifest = json.loads((run / "explorer_manifest.json").read_text())
    assert manifest["completed"] is True
    assert manifest["bytes_per_full_attempt"] <= 40 * 1024


def test_one_uint16_mask_png_preserves_overlapping_object_bits(tmp_path):
    import cv2
    run = tmp_path / "overlap"
    recorder = ExplorerRecorder(run, {})
    labels = np.zeros((3, 4), np.uint16)
    labels[1, 1] = 1
    labels[1, 2] = 2
    labels[1, 3] = 3
    attempts = [{"object": "Bear"}, {"object": "Dinosaur"}]
    recorder.record_frame(99, 100, 0, np.eye(3), labels.shape, attempts, labels)
    recorder.close(completed=True)
    rows = [json.loads(line) for line in (run / "explorer_index.jsonl").read_text().splitlines()]
    stored = cv2.imread(str(run / "masks" / "0_99.png"), cv2.IMREAD_UNCHANGED)
    assert stored.dtype == np.uint16
    first = (stored & rows[0]["mask_bit"]) != 0
    second = (stored & rows[1]["mask_bit"]) != 0
    assert first[1].tolist() == [False, True, False, True]
    assert second[1].tolist() == [False, False, True, True]


def test_uint16_mask_rejects_more_than_sixteen_attempts(tmp_path):
    recorder = ExplorerRecorder(tmp_path / "too-many-mask-bits", {})
    with pytest.raises(ValueError, match="at most 16"):
        recorder.record_frame(
            1, 1, 0, np.eye(3), (2, 2),
            [{"object": f"object-{index}"} for index in range(17)],
            np.zeros((2, 2), np.uint16))
    recorder.close(completed=False)


def test_missing_profile_is_preserved_as_legacy_exhaustive_metadata(tmp_path):
    run = tmp_path / "legacy-default"
    recorder = ExplorerRecorder(run, {})
    recorder.close(completed=True)
    manifest = json.loads((run / "explorer_manifest.json").read_text())
    assert "capture_profile" not in manifest
    assert manifest["shadow_texture_measurement"] is True


def test_binary_reader_rejects_out_of_range_and_misaligned_ranges(tmp_path):
    run = tmp_path / "bounds"
    recorder = ExplorerRecorder(run, {})
    recorder.record_frame(1, 2, 0, np.eye(3), (2, 2), [{
        "object": "Bear", "explorer_candidates": candidate_payload(1),
        "explorer_replay": replay_payload(),
    }])
    recorder.close(completed=True)
    row = json.loads((run / "explorer_index.jsonl").read_text())
    broken = dict(row, candidate_bytes=row["candidate_bytes"] - 1)
    with pytest.raises(ValueError, match="record size"):
        read_attempt(run, broken)
    broken = dict(row, replay_offset=10**9)
    with pytest.raises(ValueError, match="exceeds replay.bin"):
        read_attempt(run, broken)


def test_run_classification_and_hostile_paths_are_fail_closed(tmp_path):
    root = tmp_path / "output"; root.mkdir()
    partial = root / "partial"; partial.mkdir(); (partial / "detections.jsonl").write_text("\n")
    legacy = root / "legacy"; legacy.mkdir(); (legacy / "index.html").write_text("legacy")
    incomplete = root / "incomplete"; incomplete.mkdir()
    (incomplete / "explorer_manifest.json").write_text(json.dumps({
        "schema": "pem-explorer-v2", "completed": False}))
    complete = root / "v2"
    recorder = ExplorerRecorder(complete, {}, {"capture_profile": "realtime_inference"})
    recorder.close(completed=True)
    (complete / "frames.jsonl").write_text("")
    (complete / "detections.jsonl").write_text("")
    outside = tmp_path / "outside"; outside.mkdir()
    (root / "escape").symlink_to(outside, target_is_directory=True)

    kinds = {row["name"]: row["kind"] for row in discover_runs(root)}
    assert kinds == {"incomplete": "incomplete", "legacy": "legacy",
                     "partial": "partial", "v2": "explorer_v2"}
    assert classify_run(partial)["reason"] == "300개 후보 정보 미수집"
    for hostile in ("../outside", "/tmp", "a/b", "a\\b", ".."):
        with pytest.raises((ValueError, PermissionError)):
            safe_run(root, hostile)
    with pytest.raises(PermissionError):
        safe_run(root, "escape")


def test_completed_run_with_wrong_schema_or_binary_size_is_incomplete(tmp_path):
    run = tmp_path / "v2"
    recorder = ExplorerRecorder(run, {})
    recorder.close(completed=True)
    manifest_path = run / "explorer_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["candidate_record_bytes"] = CANDIDATE_BYTES + 4
    manifest_path.write_text(json.dumps(manifest))
    assert classify_run(run)["kind"] == "incomplete"

    manifest["candidate_record_bytes"] = CANDIDATE_BYTES
    manifest["candidate_count"] = 1
    manifest_path.write_text(json.dumps(manifest))
    state = classify_run(run)
    assert state["kind"] == "incomplete"
    assert "binary size" in state["reason"]


def test_source_end_and_existing_run_guards_are_fail_closed(tmp_path):
    run = tmp_path / "short"
    recorder = ExplorerRecorder(run, {}, {
        "expected_last_stamp_ns": 1000, "completion_tolerance_ns": 5,
    })
    recorder.record_frame(900, 901, 0, np.eye(3), (2, 2), [], None)
    recorder.close(completed=True)
    manifest = json.loads((run / "explorer_manifest.json").read_text())
    assert manifest["completed"] is False
    assert manifest["completion_reason"] == "source_end_not_reached"
    with pytest.raises(FileExistsError, match="refusing to replace"):
        ExplorerRecorder(run, {})


def test_exhaustive_manifest_requires_exact_count_and_boundary_stamps(tmp_path):
    config = {
        "capture_profile": "exhaustive_visualization", "bag_frame_count": 2,
        "expected_first_stamp_ns": 100, "expected_last_stamp_ns": 200,
    }
    run = tmp_path / "complete-full"
    recorder = ExplorerRecorder(run, {"same_camera_reference": {"kind": "pseudo-GT"}}, config)
    recorder.record_frame(100, 100, 0, np.eye(3), (2, 2), [], None)
    recorder.record_frame(200, 200, 1, np.eye(3), (2, 2), [], None)
    recorder.close(completed=True)
    manifest = json.loads((run / "explorer_manifest.json").read_text())
    assert manifest["completed"] is True
    assert manifest["capture_profile"] == "exhaustive_visualization"
    assert manifest["shadow_texture_measurement"] is True
    assert manifest["bag_frame_count"] == manifest["frames_processed"] == 2
    assert manifest["first_stamp_ns"] == 100
    assert manifest["last_stamp_ns"] == 200

    short = tmp_path / "short-full"
    recorder = ExplorerRecorder(short, {}, config)
    recorder.record_frame(100, 100, 0, np.eye(3), (2, 2), [], None)
    recorder.close(completed=True)
    manifest = json.loads((short / "explorer_manifest.json").read_text())
    assert manifest["completed"] is False
    assert manifest["completion_reason"] == "source_end_not_reached"


def test_exhaustive_completion_rejects_unmeasured_texture_candidates(tmp_path):
    config = {
        "capture_profile": "exhaustive_visualization", "bag_frame_count": 1,
        "expected_first_stamp_ns": 100, "expected_last_stamp_ns": 100,
    }
    payload = candidate_payload(3)
    payload["flags"] = np.asarray(
        [FLAG_TEXTURE_MEASURED, 0, FLAG_TEXTURE_MEASURED], np.uint16)
    run = tmp_path / "texture-incomplete"
    recorder = ExplorerRecorder(run, {}, config)
    recorder.record_frame(100, 100, 0, np.eye(3), (2, 2), [{
        "object": "Bear", "explorer_candidates": payload,
        "explorer_replay": replay_payload(),
    }])
    recorder.close(completed=True)
    manifest = json.loads((run / "explorer_manifest.json").read_text())
    assert manifest["completed"] is False
    assert manifest["shadow_texture_complete"] is False
    assert manifest["completion_reason"] == "shadow_texture_incomplete"


def test_index_preserves_final_rejected_pose_top1_stage_and_measurement_count(tmp_path):
    payload = candidate_payload(3)
    payload["flags"] = np.asarray([FLAG_TEXTURE_MEASURED, 0,
                                   FLAG_TEXTURE_MEASURED], np.uint16)
    final_pose = {"valid": True, "R": np.eye(3).tolist(),
                  "t_mm": [1.0, 2.0, 3.0], "stage": "fine_refined"}
    recorder = ExplorerRecorder(tmp_path / "metadata", {}, {
        "capture_profile": "realtime_inference",
    })
    recorder.record_frame(1, 1, 7, np.eye(3), (2, 2), [{
        "object": "Bear", "explorer_candidates": payload,
        "explorer_replay": replay_payload(),
        "decision": {"accepted": False, "selected_index300": 2,
                     "selected_proposal6000_index": 5002,
                     "rejection_reason": "final_texture_below_threshold"},
        "geometry_top1_index300": 1, "geometry_top1_proposal6000": 5001,
        "final_pose": final_pose,
        "stage_summary": {"stage6000": {"status": "available", "present": True}},
    }], np.ones((2, 2), np.uint16))
    recorder.close(completed=True)
    row = json.loads((tmp_path / "metadata" / "explorer_index.jsonl").read_text())
    assert row["accepted"] is False
    assert row["selected_index300"] == 2
    assert row["geometry_top1_index300"] == 1
    assert row["final_pose"] == final_pose
    assert row["stage_summary"]["stage6000"]["present"] is True
    assert row["texture_measured_count"] == 2


def test_nonfinite_numpy_values_are_valid_json_nulls():
    safe = json_safe({"values": np.asarray([1.0, np.nan, np.inf], np.float32)})
    assert safe == {"values": [1.0, None, None]}


def test_split_launch_registers_infer_exit_handler_even_for_external_bag():
    source = Path("realtime/launch/sam6d_split.launch.py").read_text(encoding="utf-8")
    handler = "RegisterEventHandler(OnProcessExit(\n                target_action=infer"
    assert source.index(handler) < source.index("if bool(bag.get(\"play\", False))")

    infer = Path("realtime/sam6d_infer.py").read_text(encoding="utf-8")
    assert "output and diagnostic capture_profile must match exactly" in infer


def test_all_frames_launch_is_one_sequential_capture_and_propagates_failure():
    source = Path("realtime/launch/all_frames_sam6d.launch.py").read_text(
        encoding="utf-8")
    assert source.count("ExecuteProcess(") == 1
    assert '"tools" / "capture_pem_explorer.py"' in source
    assert '"--config", LaunchConfiguration("config")' in source
    assert "if event.returncode:" in source
    assert "raise RuntimeError" in source
    for forbidden in ("sam6d_receiver_node.py", "shm_channel", "ros2", "bag", "--rate"):
        assert forbidden not in source


def test_replay_layout_has_no_candidate_images_or_features():
    blob = pack_replay(replay_payload())
    assert len(blob) == 2048 * 4 + 196 * 2 + (196 + 7) // 8 + 1024 * 2
    assert b"PNG" not in blob and b"feature" not in blob


def test_crop_indices_round_trip_to_exact_full_frame_pixels():
    import sys
    pem = Path(__file__).resolve().parents[1] / "sam6d_master/SAM-6D/Pose_Estimation_Model"
    sys.path.insert(0, str(pem / "provider")); sys.path.insert(0, str(pem / "utils"))
    sys.path.insert(0, str(pem))
    import run_inference_custom as ric
    crop = np.array([0, 4, 5, 19], dtype=np.int64)
    full = ric.global_pixel_indices(crop, [2, 6, 3, 8], 12)
    assert full.tolist() == [27, 31, 39, 67]
    yy, xx = full // 12, full % 12
    recovered = (yy - 2) * 5 + (xx - 3)
    assert np.array_equal(recovered, crop)


def test_opt_in_payload_preserves_selected_candidate_and_raw_float32():
    torch = pytest.importorskip("torch")
    import sys
    pem = Path(__file__).resolve().parents[1] / "sam6d_master/SAM-6D/Pose_Estimation_Model"
    sys.path.insert(0, str(pem / "model/pointnet2")); sys.path.insert(0, str(pem / "utils"))
    import model_utils
    rotations = torch.eye(3).reshape(1, 1, 3, 3).repeat(1, 3, 1, 1)
    translations = torch.zeros(1, 3, 3)
    geometry = torch.tensor([[1.25, 9.5, -2.0]], dtype=torch.float32)
    points = torch.tensor([[[-0.1, 0.0, 1.0], [0.1, 0.0, 1.0]]])
    base = {"topk": 3, "stride": 1, "verify": {"enabled": True},
            "dense_pm": points, "dense_fm": torch.ones(1, 2, 2),
            "dense_po": points, "dense_fo": torch.ones(1, 2, 2),
            "radius": torch.ones(1), "_proposal_ids": torch.tensor([[7, 8, 9]])}
    info_off = {}
    selected_off = model_utils.independent_candidate_verify(
        rotations, translations, geometry, base, info_off)
    info = {}
    selected_on = model_utils.independent_candidate_verify(rotations, translations, geometry, {
        **base, "diagnostic": {"enabled": True, "explorer_v2": {"enabled": True}},
        "source_pixel_index": torch.tensor([[1, 2]]),
        "cad_sample_index": torch.tensor([[3, 4]]),
        "coarse_fps_idx": torch.tensor([[0, 1]]),
        "_geo_point_valid": torch.tensor([[True, False]]),
    }, info)
    assert torch.equal(selected_on, selected_off)
    assert info["verify"] == info_off["verify"]
    payload = info["pem_explorer"][0]
    assert np.array_equal(payload["geometry"].view(np.uint32),
                          geometry.numpy()[0].view(np.uint32))
    assert payload["proposal6000"].tolist() == [7, 8, 9]


def test_coarse_forward_propagates_explorer_payload(monkeypatch):
    torch = pytest.importorskip("torch")
    import sys
    from types import SimpleNamespace
    pem = Path(__file__).resolve().parents[1] / "sam6d_master/SAM-6D/Pose_Estimation_Model"
    for path in (pem / "model/pointnet2", pem / "utils", pem / "model"):
        sys.path.insert(0, str(path))
    import coarse_point_matching as coarse_module

    class PairIdentity(torch.nn.Module):
        def forward(self, f1, _geo1, f2, _geo2):
            return f1, f2

    cfg = SimpleNamespace(nblock=1, sim_type="cosine", temp=1.0,
                          normalize_feat=True, nproposal1=6000, nproposal2=300)
    layer = coarse_module.CoarsePointMatching.__new__(
        coarse_module.CoarsePointMatching)
    torch.nn.Module.__init__(layer)
    layer.cfg = cfg
    layer.return_feat = False
    layer.nblock = 1
    layer.in_proj = torch.nn.Identity()
    layer.out_proj = torch.nn.Identity()
    layer.bg_token = torch.nn.Parameter(torch.zeros(1, 1, 2))
    layer.transformers = torch.nn.ModuleList([PairIdentity()])
    layer.eval()
    payload = [{"sentinel": "recorder-contract"}]

    monkeypatch.setattr(
        coarse_module, "compute_feature_similarity",
        lambda f1, f2, *_args: torch.zeros(f1.shape[0], f1.shape[1], f2.shape[1]))

    def fake_coarse(_atten, _p1, _p2, _model, _n1, _n2, appe=None, info=None):
        assert appe is not None
        info["pem_explorer"] = payload
        return torch.eye(3).unsqueeze(0), torch.zeros(1, 3)

    monkeypatch.setattr(coarse_module, "compute_coarse_Rt", fake_coarse)
    points = torch.zeros(1, 2, 3)
    features = torch.ones(1, 2, 2)
    geometry = torch.zeros(1, 2, 2)
    result = layer(points, features, geometry, points, features, geometry,
                   torch.ones(1), {"model": points, "appe_rerank": {"enabled": True}})
    assert result["pem_explorer"] is payload
