import json
import threading
from types import SimpleNamespace

import numpy as np
import yaml

from realtime import sam6d_live_recorder as MOD


class FakeEncoder:
    def __init__(self, *_args, **kwargs):
        self.kind = _args[3]
        self.cq = int(kwargs.get("cq", _args[4] if len(_args) > 4 else 32))
        self.items = []
        self.enqueued = self.written = self.dropped = 0
        self.error = None
        self.closing = threading.Event()

    def put(self, image, metadata):
        self.items.append((image.copy(), metadata))
        self.enqueued += 1
        self.written += 1
        return True

    def close(self):
        pass


def test_processed_depth_uses_exact_rgb_stamp_cache(tmp_path, monkeypatch):
    monkeypatch.setattr(MOD, "_Encoder", FakeEncoder)
    output = tmp_path / "live"
    config = tmp_path / "run.yaml"
    config.write_text(yaml.safe_dump({
        "topics": {"rgbd": "/camera/camera/rgbd"},
        "output": {"dir": str(output), "live_recording": {
            "enabled": True, "depth_policy": "processed"}},
    }))
    recorder = MOD.LiveRecorder(config)
    reader = SimpleNamespace(last_source_seq=7, last_depth_stamp_ns=101)
    rgb = np.zeros((2, 3, 3), np.uint8)
    depth = np.full((2, 3), 9, np.uint16)
    recorder._on_frame((rgb, depth, np.eye(3), 100, 1.0), reader)
    recorder._on_result({"stamp_ns": 99, "frame_seq": 3})
    recorder._on_result({"stamp_ns": 100, "frame_seq": 5})
    assert recorder.processed_depth_cache_miss == 1
    assert recorder.source_missed == 7
    assert recorder.processed_results_missed == 4
    assert len(recorder.depth.items) == 1
    assert recorder.depth.items[0][1] == {
        "source_seq": 7, "stamp_ns": 100, "depth_stamp_ns": 101,
        "policy": "processed", "frame_seq": 5,
    }


def test_manifest_is_atomic_and_declares_incomplete_until_finalize(tmp_path, monkeypatch):
    monkeypatch.setattr(MOD, "_Encoder", FakeEncoder)
    config = tmp_path / "run.yaml"
    config.write_text(yaml.safe_dump({"output": {
        "dir": str(tmp_path / "live"),
        "live_recording": {"enabled": True, "depth_policy": "full"}}}))
    recorder = MOD.LiveRecorder(config)
    manifest = json.loads(recorder.manifest_path.read_text())
    assert manifest["format"] == "live_replay"
    assert manifest["completed"] is False
    assert manifest["finalized"] is False
    assert not list(recorder.output.glob(".live_manifest.json.*.tmp"))


def test_ffmpeg_commands_fail_closed_on_nvenc_and_limit_ffv1_threads(tmp_path):
    rgb = object.__new__(MOD._Encoder)
    rgb.kind, rgb.fps, rgb.cq, rgb.output = "rgb", 30.0, 32, tmp_path / "rgb.mp4"
    depth = object.__new__(MOD._Encoder)
    depth.kind, depth.fps, depth.cq, depth.output = "depth", 30.0, 32, tmp_path / "depth.mkv"
    rgb_cmd = rgb._command(640, 480)
    depth_cmd = depth._command(640, 480)
    assert "h264_nvenc" in rgb_cmd and rgb_cmd[rgb_cmd.index("-cq") + 1] == "32"
    assert "libx264" not in rgb_cmd
    assert "ffv1" in depth_cmd and depth_cmd[depth_cmd.index("-threads") + 1] == "2"


def test_recorder_refuses_to_replace_existing_live_assets(tmp_path, monkeypatch):
    monkeypatch.setattr(MOD, "_Encoder", FakeEncoder)
    output = tmp_path / "live"; output.mkdir()
    (output / "rgb.mp4").write_bytes(b"existing")
    config = tmp_path / "run.yaml"
    config.write_text(yaml.safe_dump({"output": {"dir": str(output),
        "live_recording": {"enabled": True}}}))
    import pytest
    with pytest.raises(FileExistsError, match="refusing to replace"):
        MOD.LiveRecorder(config)
