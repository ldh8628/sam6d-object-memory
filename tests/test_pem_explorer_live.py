import json
import threading
from collections import OrderedDict
from http.client import HTTPConnection
from pathlib import Path

import numpy as np
import pytest
import torch

from realtime.pem_explorer_record import ExplorerRecorder
from tools.pem_explorer_live.analyzer import _strided_texture_features
from tools import serve_pem_explorer as server_module
from tools.serve_pem_explorer import (
    ExplorerServer, _candidate_correct, _legacy_shadow, _live_depth_png, _live_report,
    classify_run, discover_runs,
)


def _write_live_run(run, completed=False):
    run.mkdir()
    manifest = {
        "format": "live_replay", "schema": "sam6d-live-replay",
        "schema_version": 1, "completed": completed,
        "resolution": {"width": 2, "height": 2},
        "camera_info": {"frame_id": "camera_color_optical_frame", "k": [1] * 9},
        "recording": {"fps": 30, "depth_policy": "processed"},
    }
    (run / "live_manifest.json").write_text(json.dumps(manifest))
    rgb = [{"frame_index": i, "stamp_ns": stamp, "source_seq": source}
           for i, (stamp, source) in enumerate(((100, 0), (200, 2), (300, 3)))]
    (run / "rgb_frames.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rgb))
    (run / "depth_frames.jsonl").write_text(
        json.dumps({"frame_index": 0, "stamp_ns": 100}) + "\n")
    frames = [
        {"stamp_ns": 100, "source_seq": 0, "frame_seq": 0, "diagnostics": {}},
        {"stamp_ns": 200, "source_seq": 2, "frame_seq": 1, "diagnostics": {"rejections": [{
            "object": "Bear", "rejection_reason": "mask_filter_empty",
        }]}},
    ]
    (run / "frames.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in frames))
    detections = [
        {"stamp_ns": 100, "source_seq": 0, "object": "Bear", "score": 0.8,
         "R": np.eye(3).tolist(), "t_mm": [0, 0, 500]},
        {"stamp_ns": 150, "source_seq": 1, "object": "Dinosaur", "score": 0.7,
         "R": np.eye(3).tolist(), "t_mm": [0, 0, 600]},
    ]
    (run / "detections.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in detections))
    (run / "rgb.mp4").write_bytes(b"0123456789")
    (run / "depth.mkv").write_bytes(b"depth")
    return manifest


def test_live_replay_joins_statuses_and_preserves_incomplete_state(tmp_path):
    run = tmp_path / "live_20260827_120000"
    manifest = _write_live_run(run)
    state = classify_run(run)
    assert state["kind"] == "live_replay"
    assert state["reason"] == "recording is incomplete"
    report = _live_report(run, manifest)
    assert report["completed"] is False
    assert [frame["status"] for frame in report["frames"]] == [
        "pem_passed", "pem_rejected", "unprocessed_realtime_drop"]
    assert report["frames"][0]["stamp_ns"] == "100"
    assert report["depth_frames"][0]["stamp_ns"] == "100"
    assert [row["object"] for row in report["detections"]] == ["Bear", "Dinosaur"]


def test_corrupt_completed_live_run_is_exposed_as_incomplete(tmp_path):
    run = tmp_path / "live"; _write_live_run(run, completed=True)
    state = classify_run(run)
    assert state["kind"] == "live_replay"
    assert state["manifest"]["completed"] is False
    assert "invalid completed recording" in state["reason"]


def test_live_rgb_endpoint_supports_http_byte_ranges(tmp_path):
    root = tmp_path / "output"; root.mkdir()
    _write_live_run(root / "live")
    httpd = ExplorerServer(("127.0.0.1", 0), root, no_model=True)
    worker = threading.Thread(target=httpd.serve_forever, daemon=True); worker.start()
    try:
        client = HTTPConnection("127.0.0.1", httpd.server_port)
        client.request("GET", "/api/run/live/rgb", headers={"Range": "bytes=2-5"})
        response = client.getresponse()
        assert response.status == 206
        assert response.getheader("Content-Range") == "bytes 2-5/10"
        assert response.read() == b"2345"
        client.close()
    finally:
        httpd.shutdown(); worker.join(); httpd.server_close()


def test_live_depth_rejects_unrecorded_frame_before_decoding(tmp_path):
    run = tmp_path / "live"; manifest = _write_live_run(run)
    with pytest.raises(ValueError, match="out of range"):
        _live_depth_png(run, manifest, 1, [{"frame_index": 0, "stamp_ns": "100"}])


def test_report_joins_bag_frames_and_marks_realtime_drop(monkeypatch, tmp_path):
    root = tmp_path / "output"
    root.mkdir()
    run = root / "longcircle2_manual"
    recorder = ExplorerRecorder(run, {}, {
        "capture_profile": "realtime_inference", "bag_frame_count": 3,
    })
    recorder.record_frame(100, 100, 0, np.eye(3), (2, 2), [{
        "object": "Bear", "decision": {"accepted": False,
                                           "rejection_reason": "mask_filter_empty"},
    }])
    recorder.record_frame(300, 300, 1, np.eye(3), (2, 2), [], None)
    recorder.close(completed=True)
    (run / "frames.jsonl").write_text(
        json.dumps({"stamp_ns": 100, "frame_seq": 0, "n_boxes": 1}) + "\n" +
        json.dumps({"stamp_ns": 300, "frame_seq": 1, "n_boxes": 0}) + "\n")
    (run / "detections.jsonl").write_text("")

    monkeypatch.setattr(server_module, "_run_config", lambda _state: {
        "ism": {"objects": ["Bear", "Dinosaur"]},
        "topics": {"rgb": "/rgb", "depth": "/depth", "caminfo": "/info"},
    })
    monkeypatch.setattr(server_module, "_run_bag", lambda _state, _cfg: tmp_path)
    monkeypatch.setattr(server_module, "_bag_stamps", lambda _bag, _topic: [100, 200, 300])

    httpd = ExplorerServer(("127.0.0.1", 0), root, no_model=True)
    try:
        report = httpd.report("longcircle2_manual", classify_run(run))
    finally:
        httpd.server_close()
    assert report["dataset"]["bag_frame_count"] == 3
    assert report["dataset"]["processed_frame_count"] == 2
    assert report["frames"][0]["status"] == "pem_rejected"
    assert report["frames"][1]["status"] == "unprocessed_realtime_drop"
    assert report["frames"][2]["status"] == "processed_no_detection"
    assert len(report["frames"][0]["slots"]) == 2


def test_legacy_shadow_uses_inclusive_pose_cluster_and_geometry_fallback():
    eye = np.eye(3, dtype=np.float32)
    candidates = [
        {"R": eye, "t_m": np.zeros(3), "rank_geo": 0, "index300": 7,
         "texture": 0.1, "mask_iou": 0.1, "texture_measured": True, "flags": 2},
        {"R": eye, "t_m": np.array([0.025, 0, 0]), "rank_geo": 1,
         "index300": 8, "texture": 0.9, "mask_iou": 0.9,
         "texture_measured": True, "flags": 2},
        {"R": eye, "t_m": np.array([0.026, 0, 0]), "rank_geo": 2,
         "index300": 9, "texture": 0.8, "mask_iou": 0.8,
         "texture_measured": True, "flags": 2},
    ]
    result = _legacy_shadow({}, candidates, depth_valid_frac=0.8)
    assert result["status"] == "withheld"
    assert result["excluded_similar_count"] == 2
    assert result["fallback"]["index300"] == 9


def test_gt_and_shadow_pose_distance_are_symmetry_aware():
    eye = np.eye(3, dtype=np.float32)
    z180 = np.diag([-1.0, -1.0, 1.0]).astype(np.float32)
    reference = {"R": eye, "t_mm": [0.0, 0.0, 0.0]}
    symmetric = {"R": z180, "t_m": np.zeros(3), "rank_geo": 0,
                 "index300": 1, "texture": 0.1, "mask_iou": 0.1,
                 "texture_measured": True, "flags": 2}
    assert _candidate_correct(symmetric, reference, "Sikhye_high")["correct"]
    assert not _candidate_correct(symmetric, reference, "Bear")["correct"]

    outside = {"R": eye, "t_m": np.array([0.026, 0.0, 0.0]), "rank_geo": 1,
               "index300": 2, "texture": 0.9, "mask_iou": 0.9,
               "texture_measured": True, "flags": 2}
    result = _legacy_shadow({"object": "Sikhye_high"}, [symmetric, outside], 0.8)
    assert result["excluded_similar_count"] == 1
    assert result["fallback"]["index300"] == 2


def test_texture_replay_uses_capture_stride_exactly():
    points = torch.arange(18).reshape(6, 3)
    features = torch.arange(24).reshape(6, 4)
    selected_points, selected_features = _strided_texture_features(points, features, 2)
    assert torch.equal(selected_points, points[::2])
    assert torch.equal(selected_features, features[::2])


def test_shadow_is_unavailable_for_nonfinite_or_unprojectable_top1():
    candidate = {"R": np.eye(3), "t_m": np.zeros(3), "rank_geo": 0,
                 "index300": 0, "texture": 0.5, "mask_iou": np.nan,
                 "texture_measured": True, "flags": 0}
    result = _legacy_shadow({}, [candidate], 0.9)
    assert result["status"] == "unavailable"
    assert "projection" in result["reason"]


def test_indirect_config_paths_cannot_escape_repository_or_data(tmp_path, monkeypatch):
    outside = tmp_path / "outside.yaml"
    outside.write_text("objects: []\n", encoding="utf-8")
    with pytest.raises(PermissionError, match="ISM config"):
        server_module._object_names({"ism": {"config": str(outside)}}, [])

    monkeypatch.setattr(server_module, "DATA_ROOT", tmp_path / "data")
    (tmp_path / "data").mkdir()
    with pytest.raises(PermissionError, match="source bag"):
        server_module._run_bag(
            {"manifest": {"recording": {"source_bag": str(tmp_path / "outside-bag")}}},
            {})


def test_bag_timeline_uses_header_stamps_and_rgb_endpoint_needs_no_depth(monkeypatch,
                                                                         tmp_path):
    class Query:
        def __init__(self, values): self.values = values
        def fetchone(self): return self.values[0]
        def __iter__(self): return iter(self.values)

    class Connection:
        def execute(self, query, _params):
            if "FROM topics" in query:
                return Query([(7, "sensor_msgs/msg/Image")])
            return Query([(b"first",), (b"second",)])
        def close(self): pass

    stamps = {b"first": 101, b"second": 205}
    monkeypatch.setattr(server_module, "_bag_database", lambda _bag: tmp_path / "bag.db3")
    monkeypatch.setattr(server_module.sqlite3, "connect", lambda *_a, **_k: Connection())
    import rclpy.serialization
    import rosidl_runtime_py.utilities
    monkeypatch.setattr(rosidl_runtime_py.utilities, "get_message", lambda _name: object)
    monkeypatch.setattr(
        rclpy.serialization, "deserialize_message",
        lambda blob, _cls: type("Message", (), {"header": type("Header", (), {
            "stamp": type("Stamp", (), {"sec": 0, "nanosec": stamps[blob]})(),
        })()})())
    assert server_module._bag_stamps(tmp_path, "/rgb") == [101, 205]

    from tools.pem_explorer_live.analyzer import BagFrameCache
    cache = BagFrameCache.__new__(BagFrameCache)
    cache.cache = OrderedDict()
    calls = []
    cache._decode = lambda key, stamp: calls.append((key, stamp)) or np.ones((1, 1, 3))
    assert cache.get_rgb(101).shape == (1, 1, 3)
    assert calls == [("rgb", 101)]


def test_landing_and_live_explorer_keep_legacy_dom_contract():
    static = Path("tools/pem_explorer_live")
    landing = (static / "index.html").read_text(encoding="utf-8")
    explorer = (static / "explorer.html").read_text(encoding="utf-8")
    app = (static / "app.js").read_text(encoding="utf-8")
    assert all(token in landing for token in ("dataset-select", "report-frame"))
    assert all(token in explorer for token in (
        "frame-slider", "frame-search", "object-grid", "details", "pose-canvas"))
    assert all(token in app for token in (
        "실시간 드롭·미처리", "Geometry Top-1", "data.candidates.length", "stage6000",
        "openSlotStatus", "저장된 PEM 시도 없음", "provenance_mismatch",
        "pseudo-GT", "drawAcceptedAxes", "accepted_pose"))
    assert all(token in app for token in (
        "response.blob()", "(frameIndex + 0.5)", 'addEventListener("seeked"'))
    assert "if (hasCandidates) analyze" not in app
    assert "node.disabled" not in app
    assert all(token in app for token in (
        "startLiveReplay", "BigInt(frame.stamp_ns)", "held", "show-depth",
        "unprocessed_realtime_drop"))
    assert all(token in explorer for token in ("live-video", "live-timeline"))
    assert "live_replay" in (static / "landing.js").read_text(encoding="utf-8")


def test_completed_run_validation_is_cached(monkeypatch, tmp_path):
    run = tmp_path / "cached"
    recorder = ExplorerRecorder(run, {})
    recorder.close(completed=True)
    calls = []
    original = server_module._validate_completed_v2

    def counted(*args):
        calls.append(True)
        return original(*args)

    monkeypatch.setattr(server_module, "_validate_completed_v2", counted)
    assert classify_run(run)["kind"] == "explorer_v2"
    assert classify_run(run)["kind"] == "explorer_v2"
    assert len(calls) == 1


def test_discovery_prioritizes_full_then_realtime_then_legacy(tmp_path):
    root = tmp_path / "output"; root.mkdir()
    for name, profile in (("longcircle2_manual", "realtime_inference"),
                          ("longcircle2_visualization", "exhaustive_visualization")):
        config = {"capture_profile": profile}
        if profile == "exhaustive_visualization":
            config.update({"bag_frame_count": 1, "expected_first_stamp_ns": 1,
                           "expected_last_stamp_ns": 1})
        recorder = ExplorerRecorder(root / name, {}, config)
        if profile == "exhaustive_visualization":
            recorder.record_frame(1, 1, 0, np.eye(3), (1, 1), [], None)
        recorder.close(completed=True)
        frames = ([{"stamp_ns": 1, "frame_seq": 0}]
                  if profile == "exhaustive_visualization" else [])
        (root / name / "frames.jsonl").write_text(
            "".join(json.dumps(frame) + "\n" for frame in frames))
        (root / name / "detections.jsonl").write_text("")
    legacy = root / "pem_explorer"; legacy.mkdir(); (legacy / "index.html").write_text("ok")
    assert [row["name"] for row in discover_runs(root)][:3] == [
        "longcircle2_visualization", "longcircle2_manual", "pem_explorer"]
