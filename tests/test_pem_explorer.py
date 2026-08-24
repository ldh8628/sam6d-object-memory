import importlib.util
import json
import shlex
from argparse import Namespace
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml


PATH = Path(__file__).resolve().parents[1] / "tools" / "build_pem_explorer.py"
SPEC = importlib.util.spec_from_file_location("build_pem_explorer", PATH)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


@pytest.fixture(autouse=True)
def validated_dataset_gate(monkeypatch):
    validation = SimpleNamespace(
        bag=Path("dummy"),
        dataset_id="fixture", camera_role="fixture_camera",
        camera_source="fixture camera", manifest_path=None,
        manifest={}, gt_unavailable_reason="", allows_pseudo_gt=True,
    )
    monkeypatch.setattr(MOD, "validate_rgbd_dataset", lambda *_args, **_kwargs: validation)


def detection(stamp=10, diagnostic=None, pointwise=False):
    verify = {"cands": [{"R": np.eye(3).tolist(), "t_mm": [0, 0, 1000],
                         "geo": 2.0, "s_feat": 0.4, "s_col": 0.2, "s": 1.5,
                         "rank_geo": 0, "rank_s": 0, "elig": True}]}
    if pointwise:
        verify["pointwise"] = [{"rank_geo": 0, "rank_s": 0,
                                "geometry": {"point_count": 1},
                                "texture": {"feature_mean": 0.4}, "points": []}]
    return {"stamp_ns": stamp, "i": 0, "object": "milk", "score": 0.9,
            "R": np.eye(3).tolist(), "t_mm": [0, 0, 1000], "bbox": [10, 10, 30, 30],
            "verify": verify, **({"diagnostic": diagnostic} if diagnostic else {})}


def write_fixture(tmp_path, row):
    det = tmp_path / "detections.jsonl"
    det.write_text(json.dumps(row) + "\n", encoding="utf-8")
    frames = tmp_path / "frames.jsonl"
    frames.write_text(json.dumps({"stamp_ns": row["stamp_ns"], "i": 0,
                                  "diagnostics": {"pem_candidates": []}}) + "\n",
                      encoding="utf-8")
    cfg = tmp_path / "objects.yaml"
    cfg.write_text("objects:\n" + "".join(
        f"  - name: {name}\n    enabled: true\n" for name in MOD.DEFAULT_OBJECTS),
        encoding="utf-8")
    axes = tmp_path / "axes.json"
    axes.write_text(json.dumps({"source": "test", "default": {
        "front_axis": [1, 0, 0], "up_axis": [0, -1, 0], "length_mm": 100}}),
        encoding="utf-8")
    return det, frames, cfg, axes


def test_detection_loader_spools_one_legacy_pointwise_copy(tmp_path):
    pointwise = [{"rank_geo": 42, "points": [{"uv_224": [1, 2]}]}]
    row = detection()
    row["verify"]["pointwise"] = pointwise
    row["diagnostic"] = {"pointwise": pointwise}
    source = tmp_path / "large.jsonl"
    source.write_text(json.dumps(row) + "\n", encoding="utf-8")
    rows = MOD.load_detections_spooling_pointwise(source, tmp_path / "spool")
    assert "pointwise" not in rows[0]["verify"]
    assert "pointwise" not in rows[0]["diagnostic"]
    assert json.loads(Path(rows[0]["_pointwise_spool"]).read_text()) == pointwise


def test_detection_loader_rejects_conflicting_pointwise_copies(tmp_path):
    row = detection()
    row["verify"]["pointwise"] = [{"rank_geo": 1}]
    row["diagnostic"] = {"pointwise": [{"rank_geo": 2}]}
    source = tmp_path / "bad.jsonl"
    source.write_text(json.dumps(row) + "\n", encoding="utf-8")
    with pytest.raises(MOD.InputError, match="conflicting pointwise"):
        MOD.load_detections_spooling_pointwise(source, tmp_path / "spool")


def fake_extract(_bag, wanted, out_dir, _topic, _info_topic):
    out_dir.mkdir(parents=True, exist_ok=True)
    for stamp in wanted:
        cv2.imwrite(str(out_dir / f"{stamp}.jpg"), np.zeros((48, 64, 3), np.uint8))
    return np.array([[100, 0, 32], [0, 100, 24], [0, 0, 1]], float), [64, 48]


def args_for(tmp_path, det, frames, cfg, axes):
    return Namespace(bag="dummy", detections=str(det), frames=str(frames), pseudo_gt="",
                     trajectory="", out=str(tmp_path / "report"), title="test explorer",
                     dataset_id="fixture", camera_source="fixture camera",
                     gt_unavailable_reason="gt_missing",
                     objects_config=str(cfg), axes=str(axes),
                     color_topic=MOD.COLOR_TOPIC, camera_info_topic=MOD.COLOR_INFO_TOPIC,
                     render_topn=0,
                     rotation_threshold_deg=30.0, translation_threshold_mm=100.0,
                     command=["python", "tools/build_pem_explorer.py"])


def test_axis_projection_uses_object_to_camera_rotation():
    line = MOD.project_axis(np.eye(3), [0, 0, 1000], [1, 0, 0], 100,
                            [[100, 0, 50], [0, 100, 40], [0, 0, 1]])
    assert line == [[50.0, 40.0], [60.0, 40.0]]


def test_shape_mask_rle_and_projection_evidence_are_supported(tmp_path):
    mask = np.zeros((8, 8), bool)
    mask[2:6, 2:6] = True
    runs = [[int(i), 1] for i in np.flatnonzero(mask)]
    assert np.array_equal(MOD.decode_mask_rle([8, 8], runs), mask)
    candidate = {"R": np.eye(3).tolist(), "t_mm": [0, 0, 1000]}
    points = np.array([[-100, -100, 0], [100, -100, 0],
                       [100, 100, 0], [-100, 100, 0]], float)
    shape_meta = {"size": [8, 8], "rle": runs,
                  "crop_bbox_yxyx": [0, 8, 0, 8], "point_splat_radius_px": 1}
    projected = MOD.projected_point_mask(
        candidate, points, np.array([[10, 0, 4], [0, 10, 4], [0, 0, 1]], float),
        shape_meta)
    assert projected.shape == (8, 8) and projected.any()
    paths = MOD.render_shape_evidence(
        candidate, points, np.array([[10, 0, 4], [0, 10, 4], [0, 0, 1]], float),
        shape_meta, tmp_path / "candidate")
    assert all(path.is_file() for path in paths.values())


def test_detection_input_mask_is_shared_across_candidate_evidence(tmp_path):
    mask = np.zeros((8, 8), bool)
    mask[2:6, 2:6] = True
    runs = [[int(i), 1] for i in np.flatnonzero(mask)]
    points = np.array([[-100, -100, 0], [100, -100, 0],
                       [100, 100, 0], [-100, 100, 0]], float)
    shape_meta = {"size": [8, 8], "rle": runs,
                  "crop_bbox_yxyx": [0, 8, 0, 8], "point_splat_radius_px": 1}
    K = np.array([[10, 0, 4], [0, 10, 4], [0, 0, 1]], float)
    shared = tmp_path / "detection_shape_input.jpg"
    first = MOD.render_shape_evidence(
        {"R": np.eye(3).tolist(), "t_mm": [0, 0, 1000]},
        points, K, shape_meta, tmp_path / "candidate_0", shared)
    second = MOD.render_shape_evidence(
        {"R": np.eye(3).tolist(), "t_mm": [10, 0, 1000]},
        points, K, shape_meta, tmp_path / "candidate_1", shared)
    assert first["shape_input"] == second["shape_input"] == shared
    assert len(list(tmp_path.glob("*shape_input.jpg"))) == 1


@pytest.mark.parametrize(("candidate", "reason"), [
    ({"R": np.eye(3).tolist(), "t_mm": [0, 0, -1000]},
     "no_positive_depth_points"),
    ({"R": np.zeros((3, 3)).tolist(), "t_mm": [0, 0, 1000]},
     "degenerate_rotation"),
])
def test_pose_proxy_renders_explicit_projection_failure(tmp_path, candidate, reason):
    output = tmp_path / "projection.jpg"
    result = MOD.render_pose_proxy(
        np.zeros((48, 64, 3), np.uint8), {"bbox": [10, 10, 30, 30]}, candidate,
        np.array([[0, 0, 0], [1, 1, 1]], float),
        np.array([[100, 0, 32], [0, 100, 24], [0, 0, 1]], float), output)
    assert result == {"status": "unavailable", "reason": reason}
    assert output.is_file() and cv2.imread(str(output)).shape == (224, 224, 3)


def test_gt_missing_is_not_false_and_available_pose_can_be_true():
    missing = MOD.assess_pose(np.eye(3), [0, 0, 1000], "milk", None, np.eye(4))
    assert missing == {"status": "gt_missing", "ok": None}
    gt = {"trusted": True, "R": np.eye(3).tolist(), "t_m": [0, 0, 1]}
    result = MOD.assess_pose(np.eye(3), [0, 0, 1000], "milk", gt, np.eye(4))
    assert result["status"] == "available" and result["ok"] is True
    untrusted = {**gt, "trusted": "true"}
    assert MOD.assess_pose(np.eye(3), [0, 0, 1000], "milk", untrusted, np.eye(4)) == {
        "status": "gt_missing", "ok": None,
    }


def test_truthy_string_pseudo_gt_is_not_reported_as_available(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    pseudo = tmp_path / "pseudo.json"
    pseudo.write_text(json.dumps({
        "milk": {"trusted": "true", "R": np.eye(3).tolist(), "t_m": [0, 0, 1]},
    }), encoding="utf-8")
    trajectory = tmp_path / "trajectory.txt"
    trajectory.write_text("0.00000001 0 0 0 0 0 0 1\n", encoding="utf-8")
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.pseudo_gt, args.trajectory = str(pseudo), str(trajectory)
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    data = MOD.build(args)
    assert data["ground_truth"]["status"] == "untrusted"
    assert data["ground_truth"]["kind"] == "same-camera ORB-SLAM3 pseudo-GT"
    assert data["ground_truth"]["trusted_objects"] == []
    found = data["frames"][0]["slots"][0]["detection"]
    assert found["gt"] == {"status": "gt_missing", "ok": None}


def test_builder_labels_reference_members_and_exposes_gt_quality(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    pseudo = tmp_path / "pseudo.json"
    pseudo.write_text(json.dumps({
        "milk": {
            "trusted": True, "R": np.eye(3).tolist(), "t_m": [0, 0, 1],
            "detections": 20, "cluster_size": 10, "cluster_share": 0.5,
            "quality_gate": {"selected_count": 10, "selected_quantile": 0.5},
            "reference_frames": [{"stamp_ns": 10, "frame_index": 0}],
        },
        "_provenance": {"kind": "same-camera ORB-SLAM3 pseudo-GT",
                         "dataset_id": "fixture"},
    }), encoding="utf-8")
    trajectory = tmp_path / "trajectory.txt"
    trajectory.write_text("0.00000001 0 0 0 0 0 0 1\n", encoding="utf-8")
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.pseudo_gt, args.trajectory = str(pseudo), str(trajectory)
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)

    data = MOD.build(args)

    found = data["frames"][0]["slots"][0]["detection"]
    assert found["reference_role"] == "trusted_reference_member"
    assert data["ground_truth"]["kind"] == "same-camera ORB-SLAM3 pseudo-GT"
    assert data["ground_truth"]["objects"]["milk"]["cluster_size"] == 10
    assert data["ground_truth"]["provenance"]["dataset_id"] == "fixture"


def test_builder_rejects_nonfinite_threshold_before_publication(tmp_path):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.rotation_threshold_deg = float("nan")
    with pytest.raises(MOD.InputError, match="finite nonnegative"):
        MOD.build(args)
    assert not Path(args.out).exists()


def test_states_distinguish_no_detection_rejection_and_uncollected():
    assert MOD.object_state(None, {}, "milk") == "not_detected"
    diagnostic = {"pem_candidates": [{"object": "milk", "input_rejection": "radius_inliers<4"}]}
    assert MOD.object_state(None, diagnostic, "milk") == "pem_input_rejected"
    assert MOD.stage_value({}, "stage6000") == {"status": "uncollected", "present": None}
    with pytest.raises(MOD.InputError, match="present must be a boolean"):
        MOD.stage_value({"stage6000": {"status": "available", "present": "false"}},
                        "stage6000")
    for malformed in (False, 0, "", []):
        with pytest.raises(MOD.InputError, match="diagnostic must be an object"):
            MOD.stage_value({"stage6000": malformed}, "stage6000")


def test_partial_bundle_keeps_nine_slots_and_marks_capabilities(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    data = MOD.build(args_for(tmp_path, det, frames, cfg, axes))
    assert len(data["objects"]) == 9 and "Rabbit" not in data["objects"]
    assert len(data["frames"][0]["slots"]) == 9
    assert data["frames"][0]["slots"][0]["state"] == "detected"
    assert sum(s["state"] == "not_detected" for s in data["frames"][0]["slots"]) == 8
    assert data["dataset"]["partial"] is True
    assert isinstance(data["frames"][0]["stamp_ns"], str)
    assert data["frames"][0]["slots"][0]["detection"]["stage6000"]["status"] == "uncollected"
    for name in ("index.html", "app.css", "app.js", "report-data.js",
                 "verification-policy.js"):
        assert (tmp_path / "report" / name).is_file()
    assert (tmp_path / "report" / MOD.COMPLETION_MARKER).read_text(
        encoding="utf-8") == MOD.COMPLETION_MARKER_CONTENT


def test_shadow_verification_policy_is_validated_and_published(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.validator_depth_min = 0.8
    args.validator_texture_min = 0.449562
    args.validator_iou_min = 0.420998
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)

    data = MOD.build(args)

    policy = data["verification_filter"]
    assert policy == {
        "schema_version": 1,
        "enabled": True,
        "mode": "shadow_only",
        "selection_method": "geometry_only_unchanged",
        "depth_valid_frac_min": 0.8,
        "texture_score_min": 0.449562,
        "mask_iou_min": 0.420998,
        "reject_when": "texture_score_below_min_AND_mask_iou_below_min",
        "scope_note": "Thresholds are evaluated only when depth_valid_frac meets the minimum.",
        "dataset_id": "fixture",
    }
    sidecar = (tmp_path / "report" / "verification-policy.js").read_text(
        encoding="utf-8")
    assert sidecar.startswith("window.PEM_VERIFICATION_POLICY=")
    assert '"texture_score_min":0.449562' in sidecar
    index = (tmp_path / "report" / "index.html").read_text(encoding="utf-8")
    app = (tmp_path / "report" / "app.js").read_text(encoding="utf-8")
    assert '<script src="verification-policy.js"></script>' in index
    assert 'value="holdout_withheld">Holdout 보류' in index
    assert "texture_and_iou_below_threshold" in app
    assert "기하 Top-1 pose는 변경하지 않음" in app


@pytest.mark.parametrize("missing", ["depth", "texture", "iou"])
def test_shadow_verification_policy_requires_all_thresholds(missing):
    args = SimpleNamespace(
        validator_depth_min=None if missing == "depth" else 0.8,
        validator_texture_min=None if missing == "texture" else 0.4,
        validator_iou_min=None if missing == "iou" else 0.4,
    )
    with pytest.raises(MOD.InputError, match="must be supplied together"):
        MOD.verification_filter_policy(args)


@pytest.mark.parametrize("value", [-0.1, 1.1, float("nan"), float("inf")])
def test_shadow_verification_policy_rejects_invalid_threshold(value):
    args = SimpleNamespace(validator_depth_min=0.8,
                           validator_texture_min=value,
                           validator_iou_min=0.4)
    with pytest.raises(MOD.InputError, match=r"finite number in \[0, 1\]"):
        MOD.verification_filter_policy(args)


def test_scored_shadow_decisions_are_resolved_before_atomic_report_build(tmp_path):
    scored = tmp_path / "scored.jsonl"
    scored.write_text(json.dumps({
        "stamp_ns": 10, "i": 0, "object": "milk",
        "R": np.eye(3).tolist(), "t_mm": [0, 0, 1000],
        "pem": {"depth_valid_frac": 0.9},
    }) + "\n", encoding="utf-8")
    candidates = tmp_path / "candidates.jsonl"
    candidates.write_text(json.dumps({
        "stamp_ns": 10, "frame_index": 0, "object": "milk",
        "reference_role": "evaluation_frame",
        "stage6000": {"gt_status": "available"},
        "stage300": {"candidates": [{
            "rank_geo": 0,
            "geometry_selected": True, "texture_score": 0.3,
            "mask_iou": 0.2, "correct": False,
        }]},
    }) + "\n", encoding="utf-8")
    args = SimpleNamespace(
        validator_depth_min=0.8, validator_texture_min=0.449562,
        validator_iou_min=0.420998,
        validator_score_detections=str(scored),
        validator_candidate_analysis=str(candidates),
    )

    policy = MOD.resolved_verification_filter_policy(args, "fixture")

    assert policy["dataset_id"] == "fixture"
    assert policy["report_fingerprint"].startswith("sha256:")
    assert policy["decisions"]["10:0:milk"]["status"] == "withheld"
    assert policy["summary"]["holdout"]["true_reject"] == 1


def test_report_rejects_scored_policy_for_different_detection_pose(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.verification_filter_policy = {
        "schema_version": 1, "enabled": True, "decisions": {},
        "report_fingerprint": "sha256:not-the-report",
    }
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)

    with pytest.raises(MOD.InputError, match="fingerprint"):
        MOD._build_report(args, tmp_path / "staged", SimpleNamespace(bag=Path("dummy")),
                          "gt_missing", "fixture", "fixture camera", ({}, None, None, {}, {}))


def test_report_renders_existing_shadow_candidate_outside_top100(tmp_path, monkeypatch):
    row = detection()
    row["verify"]["cands"].append({
        "rank_geo": 142, "proposal6000_index": 4242,
        "R": np.eye(3).tolist(), "t_mm": [100, 0, 1000],
        "geo": 1.0, "texture_score": 0.7,
        "shape": {"mask_iou": 0.6, "projection_valid": True},
    })
    det, frames, cfg, axes = write_fixture(tmp_path, row)
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.verification_filter_policy = {
        "schema_version": 1, "enabled": True,
        "report_fingerprint": MOD.detection_fingerprint([row]),
        "decisions": {"10:0:milk": {"fallbackShadow": {"selected": {
            "rank_geo": 142, "proposal6000_index": 4242,
            "R": np.eye(3).tolist(), "t_mm": [100, 0, 1000],
            "geometry_score": 1.0, "texture_score": 0.7,
            "mask_iou": 0.6, "candidate_valid": True,
            "projection_valid": True,
        }}}},
    }
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    data = MOD._build_report(
        args, tmp_path / "staged", SimpleNamespace(bag=Path("dummy"),
            camera_role="fixture", manifest_path=None), "gt_missing",
        "fixture", "fixture camera", ({}, None, None, {}, {}))
    candidates = next(slot["detection"]["candidates"]
                      for slot in data["frames"][0]["slots"] if slot["detection"])
    shadow = next(candidate for candidate in candidates
                  if candidate.get("proposal6000_index") == 4242)
    assert shadow["rank_geo"] == 142
    assert shadow["shadow_evidence_extra"] is True
    assert shadow["evidence"]["mode"] != "uncollected"


def test_full_diagnostic_capabilities_are_detected(tmp_path, monkeypatch):
    stage = {"status": "available", "present": True, "match_count": 3, "total": 6000}
    row = detection(diagnostic={"stage6000": stage,
                                "stage300": {**stage, "total": 300}}, pointwise=True)
    det, frames, cfg, axes = write_fixture(tmp_path, row)
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    data = MOD.build(args_for(tmp_path, det, frames, cfg, axes))
    assert data["capabilities"] == {"stage6000": True, "stage300": True,
                                     "pointwise": True, "candidate_scores": True}
    assert data["dataset"]["partial"] is False


def test_bad_detection_schema_is_rejected():
    with pytest.raises(MOD.InputError, match="missing detection fields"):
        MOD.validate_detection({"stamp_ns": 1, "object": "milk"}, "fixture")


def test_truthy_nonobject_detection_diagnostic_fails_before_publication(
        tmp_path, monkeypatch):
    row = detection()
    row["diagnostic"] = "available"
    det, frames, cfg, axes = write_fixture(tmp_path, row)
    called = []
    monkeypatch.setattr(MOD, "extract_rgb", lambda *_args: called.append(True))
    args = args_for(tmp_path, det, frames, cfg, axes)
    with pytest.raises(MOD.InputError, match="detection diagnostic must be an object"):
        MOD.build(args)
    assert called == []
    assert not Path(args.out).exists()


def test_detection_frame_mismatch_is_rejected(tmp_path, monkeypatch):
    row = detection(stamp=10)
    det, frames, cfg, axes = write_fixture(tmp_path, row)
    frames.write_text(json.dumps({"stamp_ns": 11, "diagnostics": {}}) + "\n")
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    with pytest.raises(MOD.InputError, match="detection stamps are absent"):
        MOD.build(args_for(tmp_path, det, frames, cfg, axes))


def test_mixed_capabilities_remain_partial(tmp_path, monkeypatch):
    stage = {"status": "available", "present": True, "match_count": 1, "total": 6000}
    first = detection(stamp=10, diagnostic={"stage6000": stage,
                                             "stage300": {**stage, "total": 300}},
                      pointwise=True)
    second = detection(stamp=11)
    det, frames, cfg, axes = write_fixture(tmp_path, first)
    det.write_text(json.dumps(first) + "\n" + json.dumps(second) + "\n", encoding="utf-8")
    frames.write_text("\n".join(json.dumps({"stamp_ns": stamp, "i": i, "diagnostics": {}})
                                 for i, stamp in enumerate((10, 11))) + "\n", encoding="utf-8")
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    data = MOD.build(args_for(tmp_path, det, frames, cfg, axes))
    assert data["dataset"]["partial"] is True
    assert data["capabilities"]["stage6000"] is False
    assert data["capability_counts"]["stage6000"] == {"available": 1, "total": 2}


def test_trajectory_rejects_non_finite_and_duplicate_times(tmp_path):
    path = tmp_path / "trajectory.txt"
    path.write_text("1 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n", encoding="utf-8")
    with pytest.raises(MOD.InputError, match="timestamps must be strictly increasing"):
        MOD.load_trajectory(path)
    path.write_text("1 nan 0 0 0 0 0 1\n", encoding="utf-8")
    with pytest.raises(MOD.InputError, match="NaN/Inf"):
        MOD.load_trajectory(path)


def test_extrinsic_missing_is_explicit_and_never_false(tmp_path, monkeypatch):
    stage = {"status": "available", "present": True, "match_count": 3, "total": 6000}
    row = detection(diagnostic={"stage6000": stage,
                                "stage300": {**stage, "total": 300}}, pointwise=True)
    det, frames, cfg, axes = write_fixture(tmp_path, row)
    validation = SimpleNamespace(
        bag=Path("dummy"),
        dataset_id="longcircle2_sam", camera_role="dedicated_sam_pose_camera",
        camera_source="dedicated SAM camera", manifest_path=Path("dataset_manifest.json"),
        manifest={"ground_truth": {"allow_pseudo_gt": False}},
        gt_unavailable_reason="extrinsic_missing", allows_pseudo_gt=False,
    )
    monkeypatch.setattr(MOD, "validate_rgbd_dataset", lambda *_args, **_kwargs: validation)
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.dataset_id = "longcircle2_sam"
    args.camera_source = "dedicated SAM camera"
    args.gt_unavailable_reason = "extrinsic_missing"
    data = MOD.build(args)
    found = data["frames"][0]["slots"][0]["detection"]
    assert data["dataset"]["id"] == "longcircle2_sam"
    assert data["dataset"]["camera_source"] == "dedicated SAM camera"
    assert data["ground_truth"]["status"] == "extrinsic_missing"
    assert found["gt"] == {"status": "extrinsic_missing", "ok": None}
    assert found["stage6000"]["status"] == "extrinsic_missing"
    assert found["stage300"]["status"] == "extrinsic_missing"
    assert found["stage6000"].get("present") is None
    assert "match_count" not in found["stage6000"]
    assert "GT 없음(extrinsic_missing)" in (tmp_path / "report" / "app.js").read_text()


def test_extrinsic_missing_rejects_pseudo_gt_injection(tmp_path):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.pseudo_gt = str(tmp_path / "pseudo.json")
    args.trajectory = str(tmp_path / "trajectory.txt")
    args.gt_unavailable_reason = "extrinsic_missing"
    with pytest.raises(MOD.InputError, match="cannot be combined"):
        MOD.build(args)


def test_landing_defaults_to_sam_and_preserves_slam_link():
    root = PATH.parents[1]
    landing = root / "output" / "pem_explorer" / "index.html"
    html = landing.read_text(encoding="utf-8")
    assert '<iframe id="report-frame" src="longcircle2_sam/index.html"' in html
    assert "../longcircle2_pem_explorer/index.html" in html
    assert "../pem_score_distributions/index.html" in html
    assert "SAM camera · same-camera ORB-SLAM3 pseudo-GT" in html
    assert "longcircle2_sam_no_gt_20260823/index.html" in html
    for report in (root / "output" / "pem_explorer" / "longcircle2_sam",
                   root / "output" / "longcircle2_pem_explorer"):
        assert (report / MOD.COMPLETION_MARKER).read_text(
            encoding="utf-8") == MOD.COMPLETION_MARKER_CONTENT


def test_generated_sam_dataset_count_and_source_regression():
    root = PATH.parents[1]
    metadata = yaml.safe_load(
        (root / "data" / "longcircle2_sam" / "metadata.yaml").read_text(encoding="utf-8")
    )["rosbag2_bagfile_information"]
    topic_counts = {x["topic_metadata"]["name"]: x["message_count"]
                    for x in metadata["topics_with_message_count"]}
    expected = {
        "/camera/camera/color/image_raw",
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/color/camera_info",
        "/camera/camera/aligned_depth_to_color/camera_info",
    }
    assert metadata["message_count"] == 8520
    assert set(topic_counts) == expected
    assert set(topic_counts.values()) == {2130}

    report = root / "output" / "pem_explorer" / "longcircle2_sam"
    readme = (report / "README.md").read_text(encoding="utf-8")
    with (report / "report-data.js").open(encoding="utf-8") as handle:
        prefix = handle.read(20_000)
    assert "- frames: 2130" in readme
    assert "Dedicated SAM D435i color camera" in readme
    assert '"id":"longcircle2_sam"' in prefix
    assert '"frame_count":2130' in prefix
    assert '"kind":"same-camera ORB-SLAM3 pseudo-GT"' in prefix
    assert '"status":"available"' in prefix
    assert ('"trusted_objects":["Bear","Dinosaur","Mugcup_high",'
            '"Sauce_high","Sikhye_high","milk","saffron"]') in prefix
    assert (report / MOD.COMPLETION_MARKER).read_text(
        encoding="utf-8") == MOD.COMPLETION_MARKER_CONTENT


def test_late_build_failure_preserves_existing_report(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    target = tmp_path / "report"
    target.mkdir()
    for name in MOD.COMPLETED_REPORT_MARKERS:
        content = ("<title>PEM Pose Explorer</title><script src='report-data.js'></script>"
                   if name == "index.html" else
                   "window.PEM_EXPLORER_DATA={};" if name == "report-data.js" else
                   MOD.COMPLETION_MARKER_CONTENT if name == MOD.COMPLETION_MARKER else
                   "marker")
        (target / name).write_text(content, encoding="utf-8")
    (target / "assets").mkdir()
    sentinel = target / "sentinel.txt"
    sentinel.write_text("completed report", encoding="utf-8")
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.replace_existing = True

    def fail_after_partial_extract(_bag, wanted, out_dir, _topic, _info_topic):
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / f"{next(iter(wanted))}.jpg").write_bytes(b"partial")
        raise MOD.InputError("simulated late failure")

    monkeypatch.setattr(MOD, "extract_rgb", fail_after_partial_extract)
    with pytest.raises(MOD.InputError, match="simulated late failure"):
        MOD.build(args)
    assert sentinel.read_text(encoding="utf-8") == "completed report"
    assert not list(tmp_path.glob(".report.staging-*"))


def test_completed_report_is_replaced_with_one_atomic_directory_exchange(tmp_path):
    target = tmp_path / "report"
    target.mkdir()
    for name in MOD.COMPLETED_REPORT_MARKERS:
        content = ("<title>PEM Pose Explorer</title><script src='report-data.js'></script>"
                   if name == "index.html" else
                   "window.PEM_EXPLORER_DATA={};" if name == "report-data.js" else
                   MOD.COMPLETION_MARKER_CONTENT if name == MOD.COMPLETION_MARKER else
                   "marker")
        (target / name).write_text(content, encoding="utf-8")
    (target / "assets").mkdir()
    (target / "old.txt").write_text("old", encoding="utf-8")
    stage = tmp_path / ".report.staging-test"
    stage.mkdir()
    (stage / "new.txt").write_text("new", encoding="utf-8")

    MOD._publish_staged_report(stage, target, replace_existing=True)

    assert (target / "new.txt").read_text(encoding="utf-8") == "new"
    assert not (target / "old.txt").exists()
    assert not stage.exists()


def test_exchange_failure_leaves_completed_report_and_stage_untouched(
        tmp_path, monkeypatch):
    target = tmp_path / "report"
    target.mkdir()
    for name in MOD.COMPLETED_REPORT_MARKERS:
        content = ("<title>PEM Pose Explorer</title><script src='report-data.js'></script>"
                   if name == "index.html" else
                   "window.PEM_EXPLORER_DATA={};" if name == "report-data.js" else
                   MOD.COMPLETION_MARKER_CONTENT if name == MOD.COMPLETION_MARKER else
                   "marker")
        (target / name).write_text(content, encoding="utf-8")
    (target / "assets").mkdir()
    (target / "old.txt").write_text("old", encoding="utf-8")
    stage = tmp_path / ".report.staging-test"
    stage.mkdir()
    (stage / "new.txt").write_text("new", encoding="utf-8")
    monkeypatch.setattr(
        MOD, "_exchange_directories",
        lambda *_args: (_ for _ in ()).throw(MOD.InputError("exchange unavailable")))

    with pytest.raises(MOD.InputError, match="exchange unavailable"):
        MOD._publish_staged_report(stage, target, replace_existing=True)
    assert (target / "old.txt").read_text(encoding="utf-8") == "old"
    assert (stage / "new.txt").read_text(encoding="utf-8") == "new"


def test_full_evidence_disk_preflight_preserves_existing_report(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    target = tmp_path / "report"
    target.mkdir()
    for name in MOD.COMPLETED_REPORT_MARKERS:
        content = ("<title>PEM Pose Explorer</title><script src='report-data.js'></script>"
                   if name == "index.html" else
                   "window.PEM_EXPLORER_DATA={};" if name == "report-data.js" else
                   MOD.COMPLETION_MARKER_CONTENT if name == MOD.COMPLETION_MARKER else
                   "marker")
        (target / name).write_text(content, encoding="utf-8")
    (target / "assets").mkdir()
    sentinel = target / "sentinel.txt"
    sentinel.write_text("old report", encoding="utf-8")
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.render_topn = 100
    args.replace_existing = True
    monkeypatch.setattr(MOD.shutil, "disk_usage", lambda _path: SimpleNamespace(free=49 * 1024 ** 3))
    with pytest.raises(MOD.InputError, match="at least 50 GiB"):
        MOD.build(args)
    assert sentinel.read_text(encoding="utf-8") == "old report"
    assert not list(tmp_path.glob(".report.staging-*"))


def test_full_evidence_preflight_rejects_inode_exhaustion(tmp_path, monkeypatch):
    monkeypatch.setattr(
        MOD.shutil, "disk_usage",
        lambda _path: SimpleNamespace(free=100 * 1024 ** 3))
    monkeypatch.setattr(
        MOD.os, "statvfs",
        lambda _path: SimpleNamespace(f_files=2_000_000, f_favail=1_599_999))
    with pytest.raises(MOD.InputError, match="1,600,000 free inodes"):
        MOD._validate_full_evidence_capacity(tmp_path)


def test_full_evidence_validator_requires_every_requested_asset(tmp_path):
    stage = tmp_path / "stage"
    stage.mkdir()
    data = {"frames": [{"stamp_ns": "10", "slots": [{
        "object": "Bear", "detection": {"candidates": [{
            "rank_geo": 42, "evidence": {"mode": "sampled_pointwise",
                "projection_status": "available", "pose": "assets/missing.jpg"},
        }]},
    }]}]}
    with pytest.raises(MOD.InputError, match="missing evidence asset"):
        MOD._validate_staged_evidence(data, stage, 43)


def test_full_evidence_validator_requires_exact_ranks_and_one_shared_mask(tmp_path):
    stage = tmp_path / "stage"
    (stage / "assets").mkdir(parents=True)
    for name in ("common.jpg", "other.jpg"):
        (stage / "assets" / name).write_bytes(b"evidence")
    def entry(rank, shape_input="assets/common.jpg"):
        return {"rank_geo": rank, "evidence": {
            "mode": "sampled_pointwise", "projection_status": "available",
            "pose": "assets/common.jpg", "geometry": "assets/common.jpg",
            "texture": "assets/common.jpg", "shape_input": shape_input,
            "shape_render": "assets/common.jpg", "shape_overlap": "assets/common.jpg",
            "metrics": {"geometry": {}, "texture": {}},
        }}
    data = {"frames": [{"stamp_ns": "10", "slots": [{
        "object": "Bear", "detection": {"candidates": [entry(i) for i in range(99)]},
    }]}]}
    with pytest.raises(MOD.InputError, match="exact rank_geo 0-99"):
        MOD._validate_staged_evidence(data, stage, 100)
    data["frames"][0]["slots"][0]["detection"]["candidates"] = [
        entry(i, "assets/other.jpg" if i == 99 else "assets/common.jpg")
        for i in range(100)]
    with pytest.raises(MOD.InputError, match="input mask is not shared"):
        MOD._validate_staged_evidence(data, stage, 100)


def test_ui_uses_rank_geo_identity_and_exposes_shadow_navigation():
    app = (PATH.parents[1] / "tools" / "pem_explorer" / "app.js").read_text(
        encoding="utf-8")
    assert "cands.find(c=>Number(c.rank_geo)===selectedCandidateRank)" in app
    assert 'data-rank="${c.rank_geo}"' in app
    assert 'query.get("rank_geo")' in app
    assert 'query.get("object")' in app
    assert "visibleCandidates.push(fallbackCandidate)" in app
    assert "ev.shape_metrics||candidate?.shape" in app
    assert "decision.split!==expectedSplit" in app
    assert "제외된 유사 후보" in app
    assert "대체 후보 행으로 이동" in app


def test_checked_image_write_fails_closed(tmp_path, monkeypatch):
    monkeypatch.setattr(MOD.cv2, "imwrite", lambda *_args, **_kwargs: False)
    with pytest.raises(MOD.InputError, match="failed to write image"):
        MOD.checked_imwrite(tmp_path / "bad.jpg", np.zeros((2, 2, 3), np.uint8))


def test_unreadable_extracted_frame_does_not_publish(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    args = args_for(tmp_path, det, frames, cfg, axes)

    def invalid_extract(_bag, wanted, out_dir, _topic, _info_topic):
        out_dir.mkdir(parents=True, exist_ok=True)
        for stamp in wanted:
            (out_dir / f"{stamp}.jpg").write_bytes(b"not a jpeg")
        return np.eye(3), [64, 48]

    monkeypatch.setattr(MOD, "extract_rgb", invalid_extract)
    with pytest.raises(MOD.InputError, match="failed to read extracted RGB"):
        MOD.build(args)
    assert not (tmp_path / "report").exists()
    assert not list(tmp_path.glob(".report.staging-*"))


def test_manifest_reason_masks_embedded_available_stage(tmp_path, monkeypatch):
    stage = {"status": "available", "present": True, "match_count": 9, "total": 6000}
    row = detection(diagnostic={"stage6000": stage,
                                "stage300": {**stage, "total": 300}}, pointwise=True)
    det, frames, cfg, axes = write_fixture(tmp_path, row)
    validation = SimpleNamespace(
        bag=Path("dummy"),
        dataset_id="longcircle2_sam", camera_role="dedicated_sam_pose_camera",
        camera_source="manifest SAM camera", manifest_path=Path("dataset_manifest.json"),
        manifest={"ground_truth": {"allow_pseudo_gt": False}},
        gt_unavailable_reason="extrinsic_missing", allows_pseudo_gt=False,
    )
    monkeypatch.setattr(MOD, "validate_rgbd_dataset", lambda *_args, **_kwargs: validation)
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.dataset_id = ""
    args.camera_source = ""
    args.gt_unavailable_reason = "auto"
    data = MOD.build(args)
    found = data["frames"][0]["slots"][0]["detection"]
    assert data["dataset"]["camera_role"] == "dedicated_sam_pose_camera"
    assert found["stage6000"] == {
        "status": "extrinsic_missing", "present": None, "total": 6000,
        "suppressed": True,
        "reason": "validated inter-camera extrinsic is unavailable",
    }


def test_dataset_preflight_failure_publishes_nothing(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())

    def invalid_dataset(*_args, **_kwargs):
        raise MOD.DatasetValidationError("topic count mismatch")

    monkeypatch.setattr(MOD, "validate_rgbd_dataset", invalid_dataset)
    with pytest.raises(MOD.InputError, match="topic count mismatch"):
        MOD.build(args_for(tmp_path, det, frames, cfg, axes))
    assert not (tmp_path / "report").exists()
    assert not list(tmp_path.glob(".report.staging-*"))


def test_builder_requires_manifest_and_rejects_manifest_identity_conflicts(
        tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    args = args_for(tmp_path, det, frames, cfg, axes)
    calls = []

    def missing_manifest(*_args, **kwargs):
        calls.append(kwargs)
        raise MOD.DatasetValidationError("trusted dataset manifest is required")

    monkeypatch.setattr(MOD, "validate_rgbd_dataset", missing_manifest)
    with pytest.raises(MOD.InputError, match="trusted dataset manifest is required"):
        MOD.build(args)
    assert calls == [{"require_manifest": True}]

    validation = SimpleNamespace(
        bag=Path("dummy"),
        dataset_id="manifest-id", camera_role="manifest-role",
        camera_source="manifest-camera", manifest_path=Path("dataset_manifest.json"),
        manifest={}, gt_unavailable_reason="", allows_pseudo_gt=True,
    )
    monkeypatch.setattr(MOD, "validate_rgbd_dataset", lambda *_args, **_kwargs: validation)
    with pytest.raises(MOD.InputError, match="dataset id .* contradicts manifest"):
        MOD.build(args)
    args.dataset_id = "manifest-id"
    with pytest.raises(MOD.InputError, match="camera source .* contradicts manifest"):
        MOD.build(args)


@pytest.mark.parametrize(
    ("rotation", "translation", "reason"),
    [
        (np.zeros((3, 3)).tolist(), [0, 0, 1000], "degenerate_rotation"),
        (np.eye(3).tolist(), [0, 0, 0], "front_axis_unprojectable"),
        (np.eye(3).tolist(), [0, 0, float("nan")], "nonfinite_transform"),
        ([["x", 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 1000],
         "nonnumeric_transform"),
        ([[10 ** 1000, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 1000],
         "nonnumeric_transform"),
    ],
)
def test_invalid_selected_pose_has_distinct_state_but_candidates_remain_accessible(
        tmp_path, monkeypatch, rotation, translation, reason):
    row = detection(diagnostic={
        "stage6000": {"status": "available", "present": True},
        "stage300": {"status": "available", "present": True},
    })
    row["R"] = rotation
    row["t_mm"] = translation
    det, frames, cfg, axes = write_fixture(tmp_path, row)
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    data = MOD.build(args_for(tmp_path, det, frames, cfg, axes))
    slot = data["frames"][0]["slots"][0]
    assert slot["state"] == "pem_pose_invalid"
    assert slot["detection"]["pose_status"] == {"status": "invalid", "reason": reason}
    assert slot["detection"]["gt"] == {"status": "pose_invalid", "ok": None}
    assert slot["detection"]["stage6000"] == {
        "status": "pose_invalid", "present": None, "suppressed": True}
    assert slot["detection"]["candidates"][0]["rank_s"] == 0
    app = (tmp_path / "report" / "app.js").read_text(encoding="utf-8")
    assert 'pem_pose_invalid:"PEM 최종 pose 무효"' in app
    assert "node.disabled=!slot.detection" in app
    assert "후보 상세은 그대로 확인할 수 있습니다" in app


def test_up_projection_failure_alone_marks_selected_pose_invalid(tmp_path, monkeypatch):
    row = detection()
    row["t_mm"] = [0, 0, 50]
    det, frames, cfg, axes = write_fixture(tmp_path, row)
    axes.write_text(json.dumps({"source": "test", "default": {
        "front_axis": [1, 0, 0], "up_axis": [0, 0, -1], "length_mm": 100,
    }}), encoding="utf-8")
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    data = MOD.build(args_for(tmp_path, det, frames, cfg, axes))
    slot = data["frames"][0]["slots"][0]
    assert slot["detection"]["front_line"] is not None
    assert slot["detection"]["up_line"] is None
    assert slot["state"] == "pem_pose_invalid"
    assert slot["detection"]["pose_status"]["reason"] == "up_axis_unprojectable"


@pytest.mark.parametrize("kind", ["symlink", "file", "unrelated_dir"])
def test_replace_existing_rejects_unsafe_targets(tmp_path, kind):
    target = tmp_path / "report"
    if kind == "symlink":
        source = tmp_path / "source"
        source.mkdir()
        target.symlink_to(source, target_is_directory=True)
        match = "symlink output"
    elif kind == "file":
        target.write_text("not a report", encoding="utf-8")
        match = "not a directory"
    else:
        target.mkdir()
        (target / "notes.txt").write_text("unrelated", encoding="utf-8")
        match = "not a completed PEM report"
    with pytest.raises(MOD.InputError, match=match):
        MOD._validate_replace_target(target, replace_existing=True)


def test_replace_existing_requires_explicit_ownership_marker(tmp_path):
    target = tmp_path / "legacy-report"
    target.mkdir()
    old_markers = [name for name in MOD.COMPLETED_REPORT_MARKERS
                   if name != MOD.COMPLETION_MARKER]
    for name in old_markers:
        content = ("<title>PEM Pose Explorer</title><script src='report-data.js'></script>"
                   if name == "index.html" else
                   "window.PEM_EXPLORER_DATA={};" if name == "report-data.js" else
                   "legacy")
        (target / name).write_text(content, encoding="utf-8")
    (target / "assets").mkdir()
    with pytest.raises(MOD.InputError, match="not a completed PEM report"):
        MOD._validate_replace_target(target, replace_existing=True)


def test_builder_uses_validated_bag_and_rejects_topic_overrides(tmp_path, monkeypatch):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    validation = SimpleNamespace(
        bag=tmp_path / "canonical-bag", dataset_id="fixture", camera_role="fixture_camera",
        camera_source="fixture camera", manifest_path=Path("dataset_manifest.json"),
        manifest={}, gt_unavailable_reason="", allows_pseudo_gt=True,
    )
    monkeypatch.setattr(MOD, "validate_rgbd_dataset", lambda *_args, **_kwargs: validation)
    seen = []

    def capture_extract(bag, wanted, out_dir, topic, info_topic):
        seen.append((bag, topic, info_topic))
        return fake_extract(bag, wanted, out_dir, topic, info_topic)

    monkeypatch.setattr(MOD, "extract_rgb", capture_extract)
    args = args_for(tmp_path, det, frames, cfg, axes)
    MOD.build(args)
    assert seen == [(validation.bag, MOD.COLOR_TOPIC, MOD.COLOR_INFO_TOPIC)]

    args.out = str(tmp_path / "bad-color")
    args.color_topic = "/external/color"
    with pytest.raises(MOD.InputError, match="contradicts trusted manifest canonical topic"):
        MOD.build(args)
    args.color_topic = MOD.COLOR_TOPIC
    args.camera_info_topic = "/external/camera_info"
    with pytest.raises(MOD.InputError, match="contradicts trusted manifest canonical topic"):
        MOD.build(args)


def test_slam_readme_command_contract_builds_temp_report(tmp_path, monkeypatch):
    readme_path = PATH.parents[1] / "output" / "longcircle2_pem_explorer" / "README.md"
    readme = readme_path.read_text(encoding="utf-8")
    command = readme.split("```bash\n", 1)[1].split("\n```", 1)[0]
    tokens = shlex.split(command)
    assert tokens[:7] == [
        "/home/etri/miniconda3/bin/conda", "run", "--no-capture-output",
        "-n", "sam6d", "python", "tools/build_pem_explorer.py",
    ]
    args = MOD.parser().parse_args(tokens[7:])
    assert args.dataset_manifest == "data/longcircle2/dataset_manifest.json"
    assert args.dataset_id == "longcircle2"
    assert args.camera_source == (
        "SLAM reference RGB-D color camera; ROS2 recording with Xsens and Velodyne auxiliaries")
    assert args.replace_existing is True
    assert args.color_topic == MOD.COLOR_TOPIC
    assert args.camera_info_topic == MOD.COLOR_INFO_TOPIC

    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    args.detections, args.frames = str(det), str(frames)
    args.pseudo_gt = args.trajectory = ""
    args.out = str(tmp_path / "report")
    args.objects_config, args.axes = str(cfg), str(axes)
    args.render_topn = 0
    args.command = tokens
    validation = SimpleNamespace(
        bag=Path("canonical-longcircle2"), dataset_id="longcircle2",
        camera_role="slam_reference_camera", camera_source=args.camera_source,
        manifest_path=Path("data/longcircle2/dataset_manifest.json"), manifest={},
        gt_unavailable_reason="", allows_pseudo_gt=True,
    )
    monkeypatch.setattr(MOD, "validate_rgbd_dataset", lambda *_args, **_kwargs: validation)
    monkeypatch.setattr(MOD, "extract_rgb", fake_extract)
    data = MOD.build(args)
    assert data["dataset"]["id"] == "longcircle2"
    assert Path(args.out, "report-data.js").is_file()


@pytest.mark.parametrize(("rotation", "translation", "message"), [
    ([[1, 0], [0, 1]], [0, 0, 1], "R 3x3"),
    ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 1], "t_m length 3"),
    ([[1, 0, 0], [0, float("nan"), 0], [0, 0, 1]], [0, 0, 1], "finite"),
    ([[2, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 1], r"SO\(3\)"),
    ([[-1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 1], r"SO\(3\)"),
    ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, float("inf")], "finite"),
])
def test_builder_rejects_malformed_trusted_pose_before_output(
        tmp_path, monkeypatch, rotation, translation, message):
    det, frames, cfg, axes = write_fixture(tmp_path, detection())
    pseudo = tmp_path / "pseudo.json"
    pseudo.write_text(json.dumps({
        "milk": {"trusted": True, "R": rotation, "t_m": translation},
    }), encoding="utf-8")
    trajectory = tmp_path / "trajectory.txt"
    trajectory.write_text("0.00000001 0 0 0 0 0 0 1\n", encoding="utf-8")
    args = args_for(tmp_path, det, frames, cfg, axes)
    args.pseudo_gt, args.trajectory = str(pseudo), str(trajectory)
    called = []
    monkeypatch.setattr(MOD, "extract_rgb", lambda *_args: called.append(True))
    with pytest.raises(MOD.InputError, match=message):
        MOD.build(args)
    assert called == []
    assert not Path(args.out).exists()
