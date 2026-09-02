#!/usr/bin/env python3
"""Fail-closed validation for standard aligned RGB-D ROS2 datasets."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path

import yaml
import numpy as np


COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
COLOR_INFO_TOPIC = "/camera/camera/color/camera_info"
DEPTH_INFO_TOPIC = "/camera/camera/aligned_depth_to_color/camera_info"
STANDARD_TOPICS = {
    COLOR_TOPIC: "sensor_msgs/msg/Image",
    DEPTH_TOPIC: "sensor_msgs/msg/Image",
    COLOR_INFO_TOPIC: "sensor_msgs/msg/CameraInfo",
    DEPTH_INFO_TOPIC: "sensor_msgs/msg/CameraInfo",
}
GT_POLICIES = {
    "validated_inter_camera_extrinsic_required",
    "same_camera_reference",
    "same_camera_self_slam_reference",
    "ground_truth_disabled",
}


class DatasetValidationError(RuntimeError):
    """The bag cannot be trusted for inference or publication."""


@dataclass(frozen=True)
class DatasetValidation:
    bag: Path
    database: Path
    manifest_path: Path | None
    manifest: dict
    dataset_id: str
    camera_role: str
    camera_source: str
    pair_count: int
    first_stamp_ns: int
    last_stamp_ns: int
    topic_stats: dict

    @property
    def gt_policy(self):
        return self.manifest.get("ground_truth", {})

    @property
    def gt_unavailable_reason(self):
        return self.gt_policy.get("unavailable_reason", "")

    @property
    def allows_pseudo_gt(self):
        # Missing or malformed policy must never enable a cross-camera GT path.
        return self.gt_policy.get("allow_pseudo_gt") is True

    def summary(self):
        return {
            "dataset_id": self.dataset_id,
            "camera_role": self.camera_role,
            "manifest": str(self.manifest_path or ""),
            "database": str(self.database),
            "pair_count": self.pair_count,
            "first_stamp_ns": self.first_stamp_ns,
            "last_stamp_ns": self.last_stamp_ns,
            "gt_policy": self.gt_policy or {"policy": "unspecified"},
        }


def _fail(message):
    raise DatasetValidationError(message)


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _attested_derivative(root, relative, expected_hash, label):
    if not isinstance(relative, str) or not relative.strip():
        _fail(f"{label}.path must be a nonempty relative path")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        _fail(f"{label}.sha256 must be a 64-character digest")
    root = Path(root).resolve()
    unresolved = root / relative
    if Path(relative).is_absolute() or ".." in Path(relative).parts:
        _fail(f"{label}.path must remain inside the dataset")
    component = root
    for part in Path(relative).parts:
        component = component / part
        if component.is_symlink():
            _fail(f"{label}.path must not contain symlinks: {component}")
    resolved = unresolved.resolve(strict=False)
    try:
        resolved.relative_to(root)
    except ValueError:
        _fail(f"{label}.path escapes the dataset: {resolved}")
    if not resolved.is_file() or resolved.stat().st_size <= 0:
        _fail(f"{label}.path is missing or empty: {resolved}")
    actual = _sha256(resolved)
    if actual != expected_hash:
        _fail(f"{label}.sha256 mismatch: expected={expected_hash}, actual={actual}")
    return resolved


def discover_manifest(bag, explicit=""):
    """Return only the regular, in-bag trust manifest.

    An explicit argument is retained for CLI compatibility, but it may only
    name the canonical file.  A symlink or an external override would detach
    validation policy from the bag being inspected and is therefore rejected.
    """
    bag = Path(bag).resolve()
    canonical = bag / "dataset_manifest.json"
    if explicit:
        supplied = Path(explicit)
        if supplied.resolve(strict=False) != canonical.resolve(strict=False):
            _fail(f"dataset manifest must be the in-bag file: {canonical}")
    if canonical.is_symlink():
        _fail(f"dataset manifest must not be a symlink: {canonical}")
    if not canonical.exists():
        if explicit:
            _fail(f"dataset manifest does not exist: {canonical}")
        return None
    if not canonical.is_file():
        _fail(f"dataset manifest is not a regular file: {canonical}")
    return canonical


def _load_metadata(bag):
    path = bag / "metadata.yaml"
    if not path.is_file():
        _fail(f"bag metadata is missing: {path}")
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        info = raw["rosbag2_bagfile_information"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        _fail(f"invalid bag metadata {path}: {exc}")
    relatives = info.get("relative_file_paths") or []
    if len(relatives) != 1:
        _fail(f"expected exactly one SQLite database in metadata, got {len(relatives)}")
    database = (bag / relatives[0]).resolve()
    try:
        database.relative_to(bag.resolve())
    except ValueError:
        _fail(f"metadata database escapes bag directory: {database}")
    if not database.is_file() or database.stat().st_size <= 0:
        _fail(f"bag database is missing or empty: {database}")
    return info, database


def _metadata_topics(info):
    result = {}
    for item in info.get("topics_with_message_count") or []:
        topic = item.get("topic_metadata") or {}
        name = topic.get("name")
        if not name or name in result:
            _fail(f"invalid or duplicate metadata topic: {name!r}")
        result[name] = {"type": topic.get("type"), "count": int(item.get("message_count", -1))}
    return result


def _sqlite_topics(database):
    try:
        con = sqlite3.connect(f"file:{database.as_posix()}?mode=ro", uri=True)
        try:
            check = con.execute("PRAGMA quick_check").fetchone()
            if not check or check[0] != "ok":
                _fail(f"SQLite quick_check failed: {check}")
            orphan_count = int(con.execute(
                "SELECT COUNT(*) FROM messages m LEFT JOIN topics t "
                "ON t.id=m.topic_id WHERE t.id IS NULL"
            ).fetchone()[0])
            if orphan_count:
                _fail(f"SQLite contains {orphan_count} orphan message(s) with no topic")
            rows = con.execute(
                "SELECT t.id,t.name,t.type,COUNT(m.id),MIN(m.timestamp),MAX(m.timestamp),"
                "COUNT(DISTINCT m.timestamp) FROM topics t LEFT JOIN messages m "
                "ON m.topic_id=t.id GROUP BY t.id ORDER BY t.name"
            ).fetchall()
            names = [row[1] for row in rows]
            duplicates = sorted({name for name in names if names.count(name) > 1})
            if duplicates:
                _fail(f"duplicate SQLite topic name(s): {', '.join(duplicates)}")
            stats = {
                name: {"topic_id": int(topic_id), "type": msgtype, "count": int(count),
                       "first_stamp_ns": int(first) if first is not None else None,
                       "last_stamp_ns": int(last) if last is not None else None,
                       "unique_stamp_count": int(unique)}
                for topic_id, name, msgtype, count, first, last, unique in rows
            }
            stamps = {}
            for name in STANDARD_TOPICS:
                item = stats.get(name)
                if item:
                    stamps[name] = {int(x[0]) for x in con.execute(
                        "SELECT timestamp FROM messages WHERE topic_id=?", (item["topic_id"],))}
            total = int(con.execute("SELECT COUNT(*) FROM messages").fetchone()[0])
        finally:
            con.close()
    except (sqlite3.Error, OSError) as exc:
        _fail(f"cannot validate SQLite database {database}: {exc}")
    return stats, stamps, total


def _validate_manifest(path, actual, info, database, pair_count, first, last, total):
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        _fail(f"invalid dataset manifest {path}: {exc}")
    if not isinstance(manifest, dict):
        _fail(f"dataset manifest root must be an object: {path}")
    if manifest.get("schema_version") != 1:
        _fail(f"unsupported dataset manifest schema: {manifest.get('schema_version')!r}")
    if not isinstance(manifest.get("dataset_id"), str) or not manifest["dataset_id"].strip():
        _fail("dataset manifest dataset_id must be a nonempty string")
    trust = manifest.get("trust") or {}
    if not isinstance(trust, dict):
        _fail("dataset manifest trust must be an object")
    if trust.get("status") != "validated":
        _fail("dataset manifest trust.status must be 'validated'")
    camera = manifest.get("camera") or {}
    if not isinstance(camera, dict):
        _fail("dataset manifest camera must be an object")
    for key in ("role", "source"):
        if not isinstance(camera.get(key), str) or not camera[key].strip():
            _fail(f"dataset manifest camera.{key} must be a nonempty string")
    relation = camera.get("relation_to_slam_camera")
    if ("relation_to_slam_camera" in camera and
            (not isinstance(relation, str) or not relation.strip())):
        _fail("dataset manifest camera.relation_to_slam_camera must be a nonempty string")
    bag_expected = manifest.get("bag") or {}
    if not isinstance(bag_expected, dict):
        _fail("dataset manifest bag must be an object")
    scalar_checks = {
        "database": database.name,
        "storage_id": info.get("storage_identifier"),
        "message_count": total,
        "pair_count": pair_count,
        "first_stamp_ns": first,
        "last_stamp_ns": last,
    }
    for key, value in scalar_checks.items():
        if bag_expected.get(key) != value:
            _fail(f"manifest bag.{key}={bag_expected.get(key)!r}, actual={value!r}")
    duration = int((info.get("duration") or {}).get("nanoseconds", -1))
    if bag_expected.get("duration_ns") != duration:
        _fail(f"manifest bag.duration_ns={bag_expected.get('duration_ns')!r}, actual={duration}")
    topic_items = bag_expected.get("topics")
    auxiliary_items = bag_expected.get("auxiliary_topics")
    if not isinstance(topic_items, list) or not all(isinstance(item, dict)
                                                    for item in topic_items):
        _fail("dataset manifest bag.topics must be an array of objects")
    if not isinstance(auxiliary_items, list) or not all(isinstance(item, dict)
                                                       for item in auxiliary_items):
        _fail("dataset manifest bag.auxiliary_topics must be an array of objects")
    manifest_topic_names = [item.get("name") for item in topic_items + auxiliary_items]
    if any(not isinstance(name, str) or not name.strip() for name in manifest_topic_names):
        _fail("every manifest topic name must be a nonempty string")
    duplicate_names = sorted({name for name in manifest_topic_names
                              if manifest_topic_names.count(name) > 1}, key=str)
    if duplicate_names:
        _fail(f"duplicate manifest topic name(s): {', '.join(map(str, duplicate_names))}")
    expected_standard = {item.get("name"): item for item in topic_items}
    expected_auxiliary = {item.get("name"): item for item in auxiliary_items}
    if set(expected_standard) != set(STANDARD_TOPICS):
        _fail("manifest bag.topics must list exactly the four standard aligned RGB-D topics")
    if set(expected_auxiliary) != set(actual) - set(STANDARD_TOPICS):
        _fail("manifest auxiliary topics do not exactly match metadata/SQLite topics")
    expected_topics = {**expected_standard, **expected_auxiliary}
    if set(expected_topics) != set(actual):
        _fail("manifest topic set does not exactly match metadata/SQLite topics")
    for name, stat in actual.items():
        expected = expected_topics[name]
        for key in ("type", "count", "unique_stamp_count", "first_stamp_ns", "last_stamp_ns"):
            if expected.get(key) != stat[key]:
                _fail(f"manifest topic {name} {key}={expected.get(key)!r}, actual={stat[key]!r}")
    gt = manifest.get("ground_truth") or {}
    if not isinstance(gt, dict):
        _fail("dataset manifest ground_truth must be an object")
    policy = gt.get("policy")
    if policy not in GT_POLICIES:
        _fail(f"dataset manifest ground_truth.policy must be one of {sorted(GT_POLICIES)}")
    if type(gt.get("allow_pseudo_gt")) is not bool:
        _fail("dataset manifest ground_truth.allow_pseudo_gt must be a boolean")
    allow_pseudo_gt = gt["allow_pseudo_gt"]
    if ("inter_camera_extrinsic" in gt and
            not isinstance(gt["inter_camera_extrinsic"], dict)):
        _fail("dataset manifest ground_truth.inter_camera_extrinsic must be an object")
    extrinsic = gt.get("inter_camera_extrinsic") or {}
    role = camera["role"]
    distinct_camera = (role == "dedicated_sam_pose_camera" or
                       relation == "distinct_unregistered_camera")
    if distinct_camera and policy == "same_camera_reference":
        _fail("distinct/dedicated camera manifests cannot use same-camera pseudo-GT")
    if (distinct_camera and allow_pseudo_gt and
            policy != "same_camera_self_slam_reference"):
        _fail("distinct/dedicated camera manifests can enable pseudo-GT only through "
              "same-camera self-SLAM provenance")
    if policy == "same_camera_reference" and (
            role != "slam_reference_camera" or relation != "same_camera"):
        _fail("same_camera_reference requires camera.role='slam_reference_camera' "
              "and relation_to_slam_camera='same_camera'")
    if policy == "validated_inter_camera_extrinsic_required":
        if allow_pseudo_gt:
            _fail("inter-camera pseudo-GT is unsupported until an approved transform "
                  "application path is implemented")
        else:
            if gt.get("unavailable_reason") != "extrinsic_missing":
                _fail("pseudo-GT-disabled inter-camera datasets must declare extrinsic_missing")
            if extrinsic.get("status") == "validated":
                _fail("manifest contradicts itself: validated extrinsic but pseudo-GT disabled")
    elif policy == "same_camera_reference":
        if not allow_pseudo_gt:
            _fail("same_camera_reference policy must allow pseudo-GT")
        if gt.get("unavailable_reason") not in (None, ""):
            _fail("same-camera pseudo-GT cannot declare an unavailable_reason")
    elif policy == "same_camera_self_slam_reference":
        if not allow_pseudo_gt:
            _fail("same_camera_self_slam_reference policy must allow pseudo-GT")
        if role != "dedicated_sam_pose_camera":
            _fail("same_camera_self_slam_reference requires the dedicated SAM camera role")
        if gt.get("unavailable_reason") not in (None, ""):
            _fail("same-camera self-SLAM pseudo-GT cannot declare an unavailable_reason")
        self_slam = gt.get("self_slam")
        if not isinstance(self_slam, dict):
            _fail("same_camera_self_slam_reference requires ground_truth.self_slam")
        if self_slam.get("source_dataset_id") != manifest.get("dataset_id"):
            _fail("self-SLAM source_dataset_id must match dataset_id")
        if self_slam.get("optical_frame") != camera.get("optical_frame"):
            _fail("self-SLAM optical_frame must match camera.optical_frame")
        root = path.parent
        trajectory_path = _attested_derivative(
            root, self_slam.get("trajectory_path"), self_slam.get("trajectory_sha256"),
            "ground_truth.self_slam.trajectory")
        provenance_path = _attested_derivative(
            root, self_slam.get("provenance_path"), self_slam.get("provenance_sha256"),
            "ground_truth.self_slam.provenance")
        _attested_derivative(
            root, self_slam.get("pseudo_gt_path"), self_slam.get("pseudo_gt_sha256"),
            "ground_truth.self_slam.pseudo_gt")
        _attested_derivative(
            root, self_slam.get("settings_path"), self_slam.get("settings_sha256"),
            "ground_truth.self_slam.settings")
        try:
            provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            _fail(f"invalid self-SLAM provenance: {exc}")
        if not isinstance(provenance, dict):
            _fail("self-SLAM provenance must be a JSON object")
        source = provenance.get("source")
        trajectory = provenance.get("trajectory")
        if not isinstance(source, dict) or not isinstance(trajectory, dict):
            _fail("self-SLAM provenance source/trajectory must be objects")
        if provenance.get("status") != "accepted":
            _fail("self-SLAM provenance status must be accepted")
        if source.get("dataset_id") != manifest.get("dataset_id"):
            _fail("self-SLAM provenance dataset_id mismatch")
        if source.get("optical_frame") != camera.get("optical_frame"):
            _fail("self-SLAM provenance optical_frame mismatch")
        if source.get("bag_database") != database.name:
            _fail("self-SLAM provenance bag database mismatch")
        if trajectory.get("sha256") != self_slam.get("trajectory_sha256"):
            _fail("self-SLAM provenance trajectory digest mismatch")
        if trajectory_path.name != trajectory.get("file"):
            _fail("self-SLAM provenance trajectory filename mismatch")
    elif policy == "ground_truth_disabled":
        if allow_pseudo_gt:
            _fail("ground_truth_disabled policy cannot allow pseudo-GT")
        if not isinstance(gt.get("unavailable_reason"), str) or not gt["unavailable_reason"].strip():
            _fail("ground_truth_disabled policy requires an unavailable_reason")
        if extrinsic.get("status") == "validated":
            _fail("ground_truth_disabled policy cannot declare a validated extrinsic")
    return manifest


def validate_rgbd_dataset(bag, manifest_path="", require_manifest=False):
    """Validate metadata, SQLite and optional trust manifest before any output work."""
    bag = Path(bag).resolve()
    if not bag.is_dir():
        _fail(f"bag directory does not exist: {bag}")
    info, database = _load_metadata(bag)
    metadata_topics = _metadata_topics(info)
    sqlite_topics, stamp_sets, total = _sqlite_topics(database)
    if set(metadata_topics) != set(sqlite_topics):
        _fail("metadata and SQLite topic sets differ")
    actual_all = {}
    for name, stat in sqlite_topics.items():
        meta = metadata_topics[name]
        if meta["type"] != stat["type"]:
            _fail(f"metadata/SQLite type mismatch for {name}: "
                  f"{meta['type']} != {stat['type']}")
        if meta["count"] != stat["count"]:
            _fail(f"metadata/SQLite count mismatch for {name}: "
                  f"{meta['count']} != {stat['count']}")
        actual_all[name] = {key: stat[key] for key in
                            ("type", "count", "unique_stamp_count",
                             "first_stamp_ns", "last_stamp_ns")}
    actual = {}
    for name, msgtype in STANDARD_TOPICS.items():
        meta = metadata_topics.get(name)
        stat = sqlite_topics.get(name)
        if meta is None or stat is None:
            _fail(f"required aligned RGB-D topic is missing: {name}")
        if meta["type"] != msgtype or stat["type"] != msgtype:
            _fail(f"topic type mismatch for {name}: metadata={meta['type']}, sqlite={stat['type']}")
        if meta["count"] != stat["count"]:
            _fail(f"metadata/SQLite count mismatch for {name}: {meta['count']} != {stat['count']}")
        if stat["count"] <= 0 or stat["unique_stamp_count"] != stat["count"]:
            _fail(f"topic must have nonzero unique stamps for every message: {name}")
        actual[name] = {key: stat[key] for key in
                        ("type", "count", "unique_stamp_count", "first_stamp_ns", "last_stamp_ns")}
    counts = {x["count"] for x in actual.values()}
    firsts = {x["first_stamp_ns"] for x in actual.values()}
    lasts = {x["last_stamp_ns"] for x in actual.values()}
    if len(counts) != 1 or len(firsts) != 1 or len(lasts) != 1:
        _fail("the four standard topics do not share count and first/last stamps")
    reference = stamp_sets[next(iter(STANDARD_TOPICS))]
    for name, stamps in stamp_sets.items():
        if stamps != reference:
            _fail(f"topic stamp set differs from color images: {name}")
    pair_count, first, last = counts.pop(), firsts.pop(), lasts.pop()
    metadata_total = int(info.get("message_count", -1))
    if metadata_total != total:
        _fail(f"total message count mismatch: metadata={metadata_total}, sqlite={total}")

    manifest_file = discover_manifest(bag, manifest_path)
    if require_manifest and manifest_file is None:
        _fail(f"trusted dataset manifest is required: {bag / 'dataset_manifest.json'}")
    manifest = (_validate_manifest(manifest_file, actual_all, info, database, pair_count,
                                   first, last, total) if manifest_file else {})
    camera = manifest.get("camera") or {}
    return DatasetValidation(
        bag=bag, database=database, manifest_path=manifest_file, manifest=manifest,
        dataset_id=manifest.get("dataset_id", bag.name),
        camera_role=camera.get("role", "unspecified"),
        camera_source=camera.get("source", "unspecified"),
        pair_count=pair_count, first_stamp_ns=first, last_stamp_ns=last,
        topic_stats=actual,
    )


def enforce_ground_truth_policy(validation, pseudo_gt="", trajectory=""):
    if bool(pseudo_gt) != bool(trajectory):
        _fail("--pseudo-gt and --trajectory must be supplied together")
    if pseudo_gt and not validation.allows_pseudo_gt:
        reason = validation.gt_unavailable_reason or "dataset_policy"
        _fail(f"dataset {validation.dataset_id} forbids pseudo-GT/trajectory: {reason}")
    if not pseudo_gt:
        return
    gt_policy = getattr(validation, "gt_policy", None)
    if gt_policy is None:
        gt_policy = (getattr(validation, "manifest", {}) or {}).get("ground_truth", {})
    if gt_policy.get("policy") != "same_camera_self_slam_reference":
        return
    self_slam = gt_policy.get("self_slam") or {}
    declared_trajectory = (
        validation.bag / self_slam.get("trajectory_path", "")).resolve(strict=False)
    declared_pseudo_gt = (
        validation.bag / self_slam.get("pseudo_gt_path", "")).resolve(strict=False)
    if Path(trajectory).resolve(strict=False) != declared_trajectory:
        _fail(f"trajectory must be the attested same-camera file: {declared_trajectory}")
    if Path(pseudo_gt).resolve(strict=False) != declared_pseudo_gt:
        _fail(f"pseudo-GT must be the attested same-camera file: {declared_pseudo_gt}")
    try:
        pseudo = json.loads(Path(pseudo_gt).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        _fail(f"invalid pseudo-GT provenance: {exc}")
    provenance = pseudo.get("_provenance") if isinstance(pseudo, dict) else None
    if not isinstance(provenance, dict):
        _fail("same-camera pseudo-GT requires _provenance")
    if provenance.get("dataset_id") != validation.dataset_id:
        _fail("pseudo-GT provenance dataset_id mismatch")
    if provenance.get("trajectory_sha256") != self_slam.get("trajectory_sha256"):
        _fail("pseudo-GT provenance trajectory digest mismatch")


def validate_evaluation_thresholds(rotation_threshold_deg, translation_threshold_mm):
    """Return finite, nonnegative evaluation thresholds or fail closed."""
    result = []
    for name, value in (("rotation_threshold_deg", rotation_threshold_deg),
                        ("translation_threshold_mm", translation_threshold_mm)):
        if isinstance(value, bool):
            _fail(f"{name} must be a finite nonnegative number")
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise DatasetValidationError(
                f"{name} must be a finite nonnegative number") from exc
        if not math.isfinite(number) or number < 0:
            _fail(f"{name} must be a finite nonnegative number")
        result.append(number)
    return tuple(result)


def validate_trusted_reference_poses(references, source="pseudo-GT"):
    """Return only explicitly trusted, finite rigid reference poses."""
    if not isinstance(references, dict):
        _fail(f"{source} root must be an object")
    trusted = {}
    for name, value in references.items():
        if not isinstance(value, dict) or value.get("trusted") is not True:
            continue
        try:
            rotation = np.asarray(value.get("R"), dtype=float)
            translation = np.asarray(value.get("t_m"), dtype=float)
        except (TypeError, ValueError, OverflowError) as exc:
            raise DatasetValidationError(
                f"{source} trusted pose {name!r} must contain numeric R/t_m") from exc
        if rotation.shape != (3, 3) or translation.shape != (3,):
            _fail(f"{source} trusted pose {name!r} requires R 3x3 and t_m length 3")
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            _fail(f"{source} trusted pose {name!r} must be finite")
        determinant = float(np.linalg.det(rotation))
        if (not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1e-3) or
                not np.allclose(rotation.T @ rotation, np.eye(3),
                                rtol=0.0, atol=1e-3)):
            _fail(f"{source} trusted pose {name!r} R must be SO(3)")
        trusted[name] = value
    return trusted


def main():
    ap = argparse.ArgumentParser(description="Validate a standard aligned RGB-D dataset")
    ap.add_argument("bag")
    ap.add_argument("--manifest", default="",
                    help="optional spelling of the required in-bag dataset_manifest.json")
    ap.add_argument("--require-manifest", action="store_true")
    args = ap.parse_args()
    try:
        result = validate_rgbd_dataset(args.bag, args.manifest, args.require_manifest)
    except DatasetValidationError as exc:
        ap.error(str(exc))
    print(json.dumps(result.summary(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
