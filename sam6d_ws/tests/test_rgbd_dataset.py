import json
import hashlib
import sqlite3
import sys
from dataclasses import replace
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
import yaml

from temp.verify_eval import (FrameSyncError, enforce_pending_bound,
                              enforce_sync_complete, match_pending_rgbd)
import temp.verify_eval as verify_eval
from tools.validate_rgbd_dataset import (DatasetValidationError, STANDARD_TOPICS,
                                         enforce_ground_truth_policy,
                                         validate_evaluation_thresholds,
                                         validate_trusted_reference_poses,
                                         validate_rgbd_dataset)


def make_bag(tmp_path, stamps=(100, 200, 300), depth_stamps=None, with_manifest=True):
    bag = tmp_path / "bag"
    bag.mkdir(parents=True)
    db = bag / "bag_0.db3"
    con = sqlite3.connect(db)
    con.executescript("""
        CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT);
        CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB);
    """)
    for topic_id, (name, msgtype) in enumerate(STANDARD_TOPICS.items(), 1):
        con.execute("INSERT INTO topics VALUES(?,?,?)", (topic_id, name, msgtype))
        topic_stamps = depth_stamps if name.endswith("aligned_depth_to_color/image_raw") else stamps
        for stamp in topic_stamps or stamps:
            con.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
                        (topic_id, stamp, b"fixture"))
    con.commit()
    con.close()
    info = {
        "version": 5, "storage_identifier": "sqlite3",
        "duration": {"nanoseconds": stamps[-1] - stamps[0]},
        "starting_time": {"nanoseconds_since_epoch": stamps[0]},
        "message_count": len(stamps) * 4,
        "topics_with_message_count": [
            {"topic_metadata": {"name": name, "type": msgtype,
                                "serialization_format": "cdr", "offered_qos_profiles": ""},
             "message_count": len(stamps)}
            for name, msgtype in STANDARD_TOPICS.items()
        ],
        "relative_file_paths": [db.name],
    }
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump({"rosbag2_bagfile_information": info}, sort_keys=False), encoding="utf-8")
    if with_manifest:
        manifest = {
            "schema_version": 1, "dataset_id": "fixture",
            "trust": {"status": "validated"},
            "camera": {"role": "sam_camera", "source": "fixture"},
            "bag": {
                "database": db.name, "storage_id": "sqlite3",
                "message_count": len(stamps) * 4, "pair_count": len(stamps),
                "duration_ns": stamps[-1] - stamps[0],
                "first_stamp_ns": stamps[0], "last_stamp_ns": stamps[-1],
                "topics": [
                    {"name": name, "type": msgtype, "count": len(stamps),
                     "unique_stamp_count": len(stamps), "first_stamp_ns": stamps[0],
                     "last_stamp_ns": stamps[-1]}
                    for name, msgtype in STANDARD_TOPICS.items()
                ],
                "auxiliary_topics": [],
            },
            "ground_truth": {
                "policy": "validated_inter_camera_extrinsic_required",
                "allow_pseudo_gt": False, "unavailable_reason": "extrinsic_missing",
                "inter_camera_extrinsic": {"status": "missing"},
            },
        }
        (bag / "dataset_manifest.json").write_text(
            json.dumps(manifest), encoding="utf-8")
    return bag


def test_validator_accepts_complete_manifest_and_enforces_gt_policy(tmp_path):
    validation = validate_rgbd_dataset(make_bag(tmp_path), require_manifest=True)
    assert validation.pair_count == 3
    assert validation.camera_role == "sam_camera"
    assert validation.gt_unavailable_reason == "extrinsic_missing"
    with pytest.raises(DatasetValidationError, match="forbids pseudo-GT"):
        enforce_ground_truth_policy(validation, "pseudo.json", "trajectory.txt")


def test_validator_rejects_incomplete_topic_and_nonidentical_stamp_set(tmp_path):
    bag = make_bag(tmp_path / "count")
    con = sqlite3.connect(bag / "bag_0.db3")
    con.execute("DELETE FROM messages WHERE id=(SELECT MAX(id) FROM messages)")
    con.commit(); con.close()
    with pytest.raises(DatasetValidationError, match="count mismatch"):
        validate_rgbd_dataset(bag)

    bag = make_bag(tmp_path / "stamps", depth_stamps=(100, 250, 300), with_manifest=False)
    with pytest.raises(DatasetValidationError, match="stamp set differs"):
        validate_rgbd_dataset(bag)


def test_validator_rejects_manifest_count_drift(tmp_path):
    bag = make_bag(tmp_path)
    path = bag / "dataset_manifest.json"
    manifest = json.loads(path.read_text())
    manifest["bag"]["pair_count"] = 2
    path.write_text(json.dumps(manifest))
    with pytest.raises(DatasetValidationError, match="bag.pair_count"):
        validate_rgbd_dataset(bag)


def test_required_manifest_cannot_be_missing_or_replaced_externally(tmp_path):
    bag = make_bag(tmp_path / "missing", with_manifest=False)
    with pytest.raises(DatasetValidationError, match="trusted dataset manifest is required"):
        validate_rgbd_dataset(bag, require_manifest=True)

    external_bag = make_bag(tmp_path / "external")
    with pytest.raises(DatasetValidationError, match="must be the in-bag file"):
        validate_rgbd_dataset(bag, external_bag / "dataset_manifest.json",
                              require_manifest=True)

    external = tmp_path / "external.json"
    external.write_text("{}", encoding="utf-8")
    (bag / "dataset_manifest.json").symlink_to(external)
    with pytest.raises(DatasetValidationError, match="must not be a symlink"):
        validate_rgbd_dataset(bag, require_manifest=True)


def test_validator_rejects_self_consistent_truncated_bag_against_manifest(tmp_path):
    bag = make_bag(tmp_path)
    con = sqlite3.connect(bag / "bag_0.db3")
    con.execute("DELETE FROM messages WHERE timestamp=300")
    con.commit()
    con.close()
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = metadata["rosbag2_bagfile_information"]
    info["message_count"] = 8
    info["duration"]["nanoseconds"] = 100
    for topic in info["topics_with_message_count"]:
        topic["message_count"] = 2
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="manifest bag.message_count"):
        validate_rgbd_dataset(bag, require_manifest=True)


def test_validator_rejects_truthy_string_gt_policy_and_defaults_to_deny(tmp_path):
    bag = make_bag(tmp_path)
    manifest_path = bag / "dataset_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["ground_truth"]["allow_pseudo_gt"] = "false"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="must be a boolean"):
        validate_rgbd_dataset(bag, require_manifest=True)

    valid = validate_rgbd_dataset(make_bag(tmp_path / "default-deny"),
                                  require_manifest=True)
    assert replace(valid, manifest={}).allows_pseudo_gt is False

    policy_bag = make_bag(tmp_path / "bad-policy")
    policy_path = policy_bag / "dataset_manifest.json"
    policy_manifest = json.loads(policy_path.read_text(encoding="utf-8"))
    policy_manifest["ground_truth"]["policy"] = "trust_me"
    policy_path.write_text(json.dumps(policy_manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="policy must be one of"):
        validate_rgbd_dataset(policy_bag, require_manifest=True)


def test_declared_auxiliary_topics_are_exact_and_duplicate_names_fail(tmp_path):
    bag = make_bag(tmp_path / "declared")
    db = bag / "bag_0.db3"
    con = sqlite3.connect(db)
    con.execute("INSERT INTO topics VALUES(?,?,?)", (9, "/aux", "std_msgs/msg/UInt32"))
    con.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
                (9, 150, b"fixture"))
    con.commit()
    con.close()
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = metadata["rosbag2_bagfile_information"]
    info["message_count"] += 1
    info["topics_with_message_count"].append({
        "topic_metadata": {"name": "/aux", "type": "std_msgs/msg/UInt32",
                           "serialization_format": "cdr", "offered_qos_profiles": ""},
        "message_count": 1,
    })
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    manifest_path = bag / "dataset_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["bag"]["message_count"] += 1
    manifest["bag"]["auxiliary_topics"] = [{
        "name": "/aux", "type": "std_msgs/msg/UInt32", "count": 1,
        "unique_stamp_count": 1, "first_stamp_ns": 150, "last_stamp_ns": 150,
    }]
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    assert validate_rgbd_dataset(bag, require_manifest=True).pair_count == 3

    manifest["bag"]["auxiliary_topics"] = []
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="auxiliary topics do not exactly match"):
        validate_rgbd_dataset(bag, require_manifest=True)

    manifest["bag"]["auxiliary_topics"] = [dict(manifest["bag"]["topics"][0])]
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="duplicate manifest topic"):
        validate_rgbd_dataset(bag, require_manifest=True)


def test_duplicate_sqlite_topic_name_is_rejected(tmp_path):
    bag = make_bag(tmp_path)
    con = sqlite3.connect(bag / "bag_0.db3")
    con.execute("INSERT INTO topics VALUES(?,?,?)", (9, next(iter(STANDARD_TOPICS)),
                                                     "sensor_msgs/msg/Image"))
    con.commit()
    con.close()
    with pytest.raises(DatasetValidationError, match="duplicate SQLite topic"):
        validate_rgbd_dataset(bag, require_manifest=True)


def test_sqlite_orphan_message_topic_id_is_rejected(tmp_path):
    bag = make_bag(tmp_path)
    con = sqlite3.connect(bag / "bag_0.db3")
    con.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
                (999, 400, b"orphan"))
    con.commit()
    con.close()
    with pytest.raises(DatasetValidationError, match=r"1 orphan message\(s\)"):
        validate_rgbd_dataset(bag, require_manifest=True)


def test_trusted_pose_overflow_is_controlled_validation_error():
    huge = 10 ** 1000
    references = {
        "milk": {"trusted": True,
                 "R": [[huge, 0, 0], [0, 1, 0], [0, 0, 1]],
                 "t_m": [0, 0, 1]},
    }
    with pytest.raises(DatasetValidationError, match="must contain numeric R/t_m"):
        validate_trusted_reference_poses(references, "fixture")


def test_inter_camera_allow_is_unsupported_but_same_camera_gt_is_allowed(tmp_path):
    bag = make_bag(tmp_path / "inter")
    path = bag / "dataset_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    gt = manifest["ground_truth"]
    gt.update({"allow_pseudo_gt": True, "unavailable_reason": "",
               "inter_camera_extrinsic": {"status": "validated", "from": "a",
                                              "to": "b", "source": "fixture"}})
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="inter-camera pseudo-GT is unsupported"):
        validate_rgbd_dataset(bag, require_manifest=True)

    bag = make_bag(tmp_path / "same")
    path = bag / "dataset_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["camera"]["role"] = "slam_reference_camera"
    manifest["camera"]["relation_to_slam_camera"] = "same_camera"
    manifest["ground_truth"] = {"policy": "same_camera_reference",
                                  "allow_pseudo_gt": True}
    path.write_text(json.dumps(manifest), encoding="utf-8")
    validation = validate_rgbd_dataset(bag, require_manifest=True)
    enforce_ground_truth_policy(validation, "pseudo.json", "trajectory.txt")
    assert validation.allows_pseudo_gt is True


def test_dedicated_camera_self_slam_requires_attested_same_dataset_files(tmp_path):
    bag = make_bag(tmp_path)
    manifest_path = bag / "dataset_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["camera"].update({
        "role": "dedicated_sam_pose_camera",
        "optical_frame": "camera_color_optical_frame",
        "relation_to_slam_camera": "distinct_unregistered_camera",
    })
    derivative = bag / "self_slam"
    derivative.mkdir()
    trajectory = derivative / "CameraTrajectory.txt"
    trajectory.write_text("1.0 0 0 0 0 0 0 1\n", encoding="utf-8")
    settings = derivative / "settings.yaml"
    settings.write_text("fixture\n", encoding="utf-8")
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    provenance = derivative / "provenance.json"
    provenance.write_text(json.dumps({
        "status": "accepted",
        "source": {"dataset_id": "fixture", "optical_frame": "camera_color_optical_frame",
                   "bag_database": "bag_0.db3"},
        "trajectory": {"file": trajectory.name, "sha256": digest(trajectory)},
    }), encoding="utf-8")
    pseudo = derivative / "pseudo.json"
    pseudo.write_text(json.dumps({"_provenance": {
        "dataset_id": "fixture", "trajectory_sha256": digest(trajectory)}}), encoding="utf-8")
    manifest["ground_truth"] = {
        "policy": "same_camera_self_slam_reference", "allow_pseudo_gt": True,
        "self_slam": {
            "source_dataset_id": "fixture", "optical_frame": "camera_color_optical_frame",
            "trajectory_path": "self_slam/CameraTrajectory.txt",
            "trajectory_sha256": digest(trajectory),
            "provenance_path": "self_slam/provenance.json",
            "provenance_sha256": digest(provenance),
            "pseudo_gt_path": "self_slam/pseudo.json", "pseudo_gt_sha256": digest(pseudo),
            "settings_path": "self_slam/settings.yaml", "settings_sha256": digest(settings),
        },
    }
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    validation = validate_rgbd_dataset(bag, require_manifest=True)
    enforce_ground_truth_policy(validation, pseudo, trajectory)

    manifest["ground_truth"]["self_slam"]["trajectory_sha256"] = "0" * 64
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="sha256 mismatch"):
        validate_rgbd_dataset(bag, require_manifest=True)


@pytest.mark.parametrize(("role", "relation"), [
    ("dedicated_sam_pose_camera", "same_camera"),
    ("slam_reference_camera", "distinct_unregistered_camera"),
])
def test_distinct_camera_cross_fields_cannot_enable_same_camera_gt(
        tmp_path, role, relation):
    bag = make_bag(tmp_path)
    path = bag / "dataset_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["camera"].update({"role": role, "relation_to_slam_camera": relation})
    manifest["ground_truth"] = {"policy": "same_camera_reference",
                                  "allow_pseudo_gt": True}
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="distinct/dedicated camera"):
        validate_rgbd_dataset(bag, require_manifest=True)


def test_same_camera_policy_requires_explicit_role_and_relation(tmp_path):
    bag = make_bag(tmp_path)
    path = bag / "dataset_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["camera"].update({"role": "sam_camera",
                                "relation_to_slam_camera": "same_camera"})
    manifest["ground_truth"] = {"policy": "same_camera_reference",
                                  "allow_pseudo_gt": True}
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="requires camera.role"):
        validate_rgbd_dataset(bag, require_manifest=True)


@pytest.mark.parametrize("value", [None, [], ""])
def test_manifest_extrinsic_key_is_strict_object_even_when_falsy(tmp_path, value):
    bag = make_bag(tmp_path)
    path = bag / "dataset_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["ground_truth"]["inter_camera_extrinsic"] = value
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="inter_camera_extrinsic must be an object"):
        validate_rgbd_dataset(bag, require_manifest=True)


@pytest.mark.parametrize("name", [None, "", "   ", 123])
def test_manifest_topic_names_must_be_nonempty_strings(tmp_path, name):
    bag = make_bag(tmp_path)
    path = bag / "dataset_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["bag"]["topics"][0]["name"] = name
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(DatasetValidationError, match="topic name must be a nonempty string"):
        validate_rgbd_dataset(bag, require_manifest=True)


@pytest.mark.parametrize("rotation,translation", [
    (float("nan"), 1), (1, float("inf")), (-1, 1), (1, -1), (True, 1),
])
def test_evaluation_thresholds_must_be_finite_and_nonnegative(rotation, translation):
    with pytest.raises(DatasetValidationError, match="finite nonnegative"):
        validate_evaluation_thresholds(rotation, translation)


def test_verify_eval_entrypoint_requires_in_bag_manifest(monkeypatch):
    torch_module = ModuleType("torch")
    core_module = ModuleType("sam6d_core")
    core_module.Sam6DCore = object
    config_module = ModuleType("verify_config")
    config_module.build_appe_cfg = lambda: None
    monkeypatch.setitem(sys.modules, "torch", torch_module)
    monkeypatch.setitem(sys.modules, "sam6d_core", core_module)
    monkeypatch.setitem(sys.modules, "verify_config", config_module)
    calls = []

    def reject(*_args, **kwargs):
        calls.append(kwargs)
        raise DatasetValidationError("trusted dataset manifest is required")

    monkeypatch.setattr(verify_eval, "validate_rgbd_dataset", reject)
    args = SimpleNamespace(bag="dummy", dataset_manifest="", pseudo_gt="", trajectory="")
    with pytest.raises(SystemExit, match="trusted dataset manifest is required"):
        verify_eval.run(args)
    assert calls == [{"require_manifest": True}]


def test_verify_eval_uses_validated_bag_and_strict_trusted_gt(tmp_path, monkeypatch):
    torch_module = ModuleType("torch")
    core_module = ModuleType("sam6d_core")
    core_module.Sam6DCore = object
    config_module = ModuleType("verify_config")
    config_module.build_appe_cfg = lambda: None
    monkeypatch.setitem(sys.modules, "torch", torch_module)
    monkeypatch.setitem(sys.modules, "sam6d_core", core_module)
    monkeypatch.setitem(sys.modules, "verify_config", config_module)
    canonical = tmp_path / "canonical-bag"
    validation = SimpleNamespace(
        bag=canonical, pair_count=3, allows_pseudo_gt=True,
        summary=lambda: {"dataset_id": "fixture"},
    )
    monkeypatch.setattr(verify_eval, "validate_rgbd_dataset",
                        lambda *_args, **_kwargs: validation)
    monkeypatch.setattr(verify_eval, "enforce_ground_truth_policy", lambda *_args: None)
    seen = []

    def no_frames(bag, stride, limit, expected_pair_count=None):
        seen.append((bag, expected_pair_count))
        return None, []

    monkeypatch.setattr(verify_eval, "read_frames", no_frames)
    args = SimpleNamespace(
        bag="caller-bag", dataset_manifest="", pseudo_gt="", trajectory="",
        rotation_threshold_deg=30, translation_threshold_mm=100, stride=1, limit=0,
    )
    with pytest.raises(SystemExit, match="프레임/카메라 정보를 못 읽었다"):
        verify_eval.run(args)
    assert seen == [(canonical, 3)]
    monkeypatch.delitem(sys.modules, "torch", raising=False)

    pseudo = tmp_path / "pseudo.json"
    pseudo.write_text(json.dumps({
        "truthy": {"trusted": "true", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                   "t_m": [0, 0, 1]},
        "exact": {"trusted": True, "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                  "t_m": [0, 0, 1]},
    }), encoding="utf-8")
    trajectory = tmp_path / "trajectory.txt"
    trajectory.write_text("1 0 0 0 0 0 0 1\n", encoding="utf-8")
    refs = verify_eval.DiagnosticReferenceProvider(pseudo, trajectory).at(1_000_000_000)
    assert set(refs) == {"exact"}


def test_verify_reference_provider_rejects_malformed_trusted_pose(tmp_path):
    pseudo = tmp_path / "pseudo.json"
    pseudo.write_text(json.dumps({
        "bad": {"trusted": True, "R": [[2, 0, 0], [0, 1, 0], [0, 0, 1]],
                "t_m": [0, 0, 1]},
    }), encoding="utf-8")
    trajectory = tmp_path / "trajectory.txt"
    trajectory.write_text("1 0 0 0 0 0 0 1\n", encoding="utf-8")
    with pytest.raises(ValueError, match=r"trusted pose.*SO\(3\)"):
        verify_eval.DiagnosticReferenceProvider(pseudo, trajectory)


def test_verify_reference_provider_rejects_nonmonotonic_trajectory(tmp_path):
    pseudo = tmp_path / "pseudo.json"
    pseudo.write_text("{}", encoding="utf-8")
    trajectory = tmp_path / "trajectory.txt"
    trajectory.write_text(
        "2 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n", encoding="utf-8")

    with pytest.raises(ValueError, match="strictly increasing"):
        verify_eval.DiagnosticReferenceProvider(pseudo, trajectory)


def test_pending_matcher_is_bounded_and_fails_on_drop_or_eof():
    color = (100, "rgb", "bgr8")
    matches, pending, dropped = match_pending_rgbd([color], [], 20)
    assert matches == [] and pending == [color] and dropped == []
    matches, pending, dropped = match_pending_rgbd(pending, [(100, "depth")], 20)
    assert len(matches) == 1 and pending == [] and dropped == []

    _, _, dropped = match_pending_rgbd([(100, "rgb", "bgr8")], [(200, "depth")], 20)
    assert len(dropped) == 1
    with pytest.raises(FrameSyncError, match="exceeded 2"):
        enforce_pending_bound([color, color, color], 2)
    with pytest.raises(FrameSyncError, match="unmatched color"):
        enforce_sync_complete([color])
    with pytest.raises(FrameSyncError, match="dropped 1"):
        enforce_sync_complete([], 1)


def test_real_sam_manifest_and_sqlite_are_still_complete():
    root = Path(__file__).resolve().parents[1]
    validation = validate_rgbd_dataset(root / "data" / "longcircle2_sam", require_manifest=True)
    assert validation.pair_count == 2130
    assert validation.first_stamp_ns == 1785831506201433600
    assert validation.last_stamp_ns == 1785831578388433664
    assert validation.allows_pseudo_gt is True
    assert validation.gt_policy["policy"] == "same_camera_self_slam_reference"


def test_real_slam_manifest_declares_all_auxiliaries_and_allows_same_camera_gt():
    root = Path(__file__).resolve().parents[1]
    validation = validate_rgbd_dataset(root / "data" / "longcircle2",
                                       require_manifest=True)
    assert validation.pair_count == 2166
    assert validation.camera_role == "slam_reference_camera"
    assert validation.allows_pseudo_gt is True
    auxiliary = validation.manifest["bag"]["auxiliary_topics"]
    assert len(auxiliary) == 8
    assert sum(item["count"] for item in auxiliary) == 69345
