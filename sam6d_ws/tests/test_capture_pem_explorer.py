import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml


PATH = Path(__file__).resolve().parents[1] / "tools" / "capture_pem_explorer.py"
SPEC = importlib.util.spec_from_file_location("capture_pem_explorer", PATH)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def capture_config(output, bag, profile="exhaustive_visualization"):
    return {
        "topics": {
            "rgb": "/camera/camera/color/image_raw",
            "depth": "/camera/camera/aligned_depth_to_color/image_raw",
            "caminfo": "/camera/camera/color/camera_info",
        },
        "runtime": {"pem_diagnostic": {
            "enabled": True,
            "explorer_v2": {"enabled": True, "capture_profile": profile},
        }},
        "output": {
            "dir": str(output),
            "pem_explorer": {"enabled": True, "capture_profile": profile,
                             "source_bag": str(bag)},
        },
        "bag": {"path": str(bag)},
    }


def test_full_config_is_fixed_profile_and_existing_output_fails_before_model(tmp_path,
                                                                            monkeypatch):
    output = tmp_path / "output" / "longcircle2_visualization"
    bag = tmp_path / "data" / "bag"; bag.mkdir(parents=True)
    config_path = tmp_path / "full.yaml"
    config_path.write_text(yaml.safe_dump(capture_config(output, bag)), encoding="utf-8")
    monkeypatch.setattr(MOD, "REPO", tmp_path)
    cfg, explorer, resolved_output, resolved_bag, resolved_config = MOD.load_capture_config(
        config_path)
    assert cfg["runtime"]["pem_diagnostic"]["explorer_v2"]["capture_profile"] == (
        "exhaustive_visualization")
    assert explorer["capture_profile"] == "exhaustive_visualization"
    assert resolved_output == output.resolve()
    assert resolved_bag == bag.resolve()
    assert resolved_config == config_path.resolve()

    output.mkdir(parents=True)
    with pytest.raises(FileExistsError, match="refusing to replace"):
        MOD.capture(config_path)


def test_direct_paths_override_dataset_specific_yaml(tmp_path, monkeypatch):
    config_path = tmp_path / "generic.yaml"
    config_path.write_text(yaml.safe_dump(capture_config("", "")), encoding="utf-8")
    monkeypatch.setattr(MOD, "REPO", tmp_path)
    output, bag = tmp_path / "session/sam6d", tmp_path / "session/converted/SAM"
    _, _, resolved_output, resolved_bag, _ = MOD.load_capture_config(
        config_path, bag, output)
    assert resolved_output == output.resolve()
    assert resolved_bag == bag.resolve()


@pytest.mark.parametrize("mutation", ["output_profile", "diagnostic_profile", "output_path"])
def test_capture_config_rejects_non_exhaustive_or_external_output(tmp_path, monkeypatch,
                                                                  mutation):
    output = tmp_path / "output" / "expected"
    bag = tmp_path / "data" / "bag"; bag.mkdir(parents=True)
    cfg = capture_config(output, bag)
    monkeypatch.setattr(MOD, "REPO", tmp_path)
    if mutation == "output_profile":
        cfg["output"]["pem_explorer"]["capture_profile"] = "realtime_inference"
    elif mutation == "diagnostic_profile":
        cfg["runtime"]["pem_diagnostic"]["explorer_v2"]["capture_profile"] = (
            "realtime_inference")
    else:
        cfg["output"]["dir"] = str(tmp_path / "different")
    path = tmp_path / f"{mutation}.yaml"
    path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    with pytest.raises(MOD.CaptureError):
        MOD.load_capture_config(path)


def test_reference_provider_transforms_map_pose_into_current_camera_pose():
    provider = MOD.TrustedReferenceProvider.__new__(MOD.TrustedReferenceProvider)
    provider.enabled = True
    provider.times = np.asarray([1.0, 2.0])
    provider.poses = np.asarray([
        MOD._pose_matrix(np.eye(3), [1.0, 0.0, 0.0]),
        MOD._pose_matrix(np.eye(3), [2.0, 0.0, 0.0]),
    ])
    provider.objects = {"Bear": {"R": np.eye(3), "t_m": [1.5, 0.0, 1.0]}}
    provider.max_dt_s = 0.025
    references = provider.for_stamp(1_000_000_000)
    assert np.allclose(references["Bear"]["R"], np.eye(3))
    assert np.allclose(references["Bear"]["t_mm"], [500.0, 0.0, 1000.0])
    assert provider.for_stamp(1_500_000_000) == {}


def test_public_frame_diagnostics_never_serializes_candidate_arrays():
    diagnostics = {"pem_candidates": [{
        "object": "Bear", "bbox": [1, 2, 3, 4],
        "explorer_candidates": {"texture": np.ones(300)},
        "explorer_replay": {"source_pixel_index": np.arange(2048)},
        "decision": {"accepted": False}, "final_pose": {"valid": False},
    }]}
    public = MOD._public_frame_diagnostics(diagnostics)
    assert public["pem_candidates"] == [{
        "object": "Bear", "bbox": [1, 2, 3, 4], "final_pose": {"valid": False},
    }]


def test_rosbags_image_decoder_handles_bgr_and_uint16_depth():
    bgr = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    depth = np.arange(6, dtype=np.uint16).reshape(2, 3)
    color_msg = SimpleNamespace(height=2, width=3, step=9, encoding="bgr8",
                                is_bigendian=0, data=bgr.reshape(-1))
    depth_msg = SimpleNamespace(height=2, width=3, step=6, encoding="16UC1",
                                is_bigendian=0, data=depth.view(np.uint8).reshape(-1))
    assert np.array_equal(MOD._decode_image(color_msg, "color"), bgr)
    assert np.array_equal(MOD._decode_image(depth_msg, "depth"), depth)


def test_preview_writer_creates_all_intra_h264_and_rejects_count_mismatch(tmp_path):
    writer = MOD.PreviewWriter(tmp_path / "preview.mp4", 64, 48, 29.4928449)
    for value in (0, 80, 160):
        writer.write(np.full((48, 64, 3), value, np.uint8))
    metadata = writer.close(expected_count=3)
    assert metadata["codec"] == "h264"
    assert metadata["all_intra"] is True
    assert metadata["frame_count"] == 3
    assert metadata["width"] == 64 and metadata["height"] == 48
    assert len(metadata["sha256"]) == 64

    short = MOD.PreviewWriter(tmp_path / "short.mp4", 64, 48, 30.0)
    short.write(np.zeros((48, 64, 3), np.uint8))
    with pytest.raises(MOD.CaptureError, match="received 1 frames, expected 2"):
        short.close(expected_count=2)
