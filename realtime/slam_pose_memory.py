"""Static-object pose stabilization using an already learned SLAM-map anchor.

This module is deliberately separate from standalone PEM scoring.  It is useful
when objects are fixed in the scene and a clean initialization interval has
provided one trusted object pose in the SLAM map frame.
"""
from __future__ import annotations

import json
import math
from collections import defaultdict, deque
from pathlib import Path

import numpy as np


# External pose convention: +X side, +Y rear, +Z up.  SAM-6D itself stays in
# each CAD's native frame; only published poses/boxes cross this boundary.
_CAD_FROM_CANONICAL = {
    "milk": np.diag([-1.0, -1.0, 1.0]),
    "choco_hazelnut_high": np.asarray([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ]),
}


def canonical_object_rotation(object_name, rotation):
    """Convert a native-CAD object rotation to the shared external frame."""
    return np.asarray(rotation, dtype=float) @ _CAD_FROM_CANONICAL.get(
        object_name, np.eye(3))


def canonical_object_points(object_name, points):
    """Express native-CAD points in the shared external object frame."""
    return np.asarray(points) @ _CAD_FROM_CANONICAL.get(object_name, np.eye(3))


def tracking_state_ok(value):
    return str(value).strip().lower() in {"tracking", "tracking_ok"}


def valid_se3(value, atol=1e-3):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        return False
    rotation = value[:3, :3]
    return (np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6)
            and np.allclose(rotation.T @ rotation, np.eye(3), atol=atol)
            and np.linalg.det(rotation) > 0.99)


def _axis_symmetries(axis, step_deg=10):
    values = [np.eye(3)]
    if axis is None:
        return values
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not np.isfinite(norm) or norm <= 0:
        return values
    x, y, z = axis / norm
    for degrees in range(int(step_deg), 360, int(step_deg)):
        angle = math.radians(degrees)
        c, s, k = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
        values.append(np.asarray([
            [c + x*x*k, x*y*k - z*s, x*z*k + y*s],
            [y*x*k + z*s, c + y*y*k, y*z*k - x*s],
            [z*x*k - y*s, z*y*k + x*s, c + z*z*k],
        ]))
    return values


def pose_distance(a, b, symmetry_axis=None, symmetry_step_deg=10):
    """Return symmetry-aware rotation degrees and translation metres."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    relative = a[:3, :3].T @ b[:3, :3]
    best_trace = max(float(np.trace(relative @ symmetry))
                     for symmetry in _axis_symmetries(symmetry_axis,
                                                       symmetry_step_deg))
    cosine = np.clip((best_trace - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(cosine)), float(np.linalg.norm(a[:3, 3] - b[:3, 3]))


def interpolate_se3(stamp_ns, left_stamp_ns, left, right_stamp_ns, right,
                    max_span_s=None):
    """Interpolate a map-camera pose at ``stamp_ns`` using NumPy only."""
    left, right = np.asarray(left, float), np.asarray(right, float)
    stamp_ns, left_stamp_ns, right_stamp_ns = map(
        int, (stamp_ns, left_stamp_ns, right_stamp_ns))
    if (not valid_se3(left) or not valid_se3(right)
            or not left_stamp_ns <= stamp_ns <= right_stamp_ns
            or left_stamp_ns >= right_stamp_ns):
        return None
    span_ns = right_stamp_ns - left_stamp_ns
    if max_span_s is not None and span_ns > float(max_span_s) * 1e9:
        return None
    weight = (stamp_ns - left_stamp_ns) / span_ns
    if weight == 0.0:
        return left.copy()
    if weight == 1.0:
        return right.copy()

    relative = left[:3, :3].T @ right[:3, :3]
    angle = math.acos(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    if angle < 1e-12:
        rotation = left[:3, :3].copy()
    else:
        if math.pi - angle < 1e-6:
            values, vectors = np.linalg.eig(relative)
            axis = np.real(vectors[:, np.argmin(np.abs(values - 1.0))])
            axis /= np.linalg.norm(axis)
        else:
            axis = np.asarray([relative[2, 1] - relative[1, 2],
                               relative[0, 2] - relative[2, 0],
                               relative[1, 0] - relative[0, 1]]) / (2.0 * math.sin(angle))
        x, y, z = axis
        skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
        partial = angle * weight
        rotation = left[:3, :3] @ (
            np.eye(3) + math.sin(partial) * skew
            + (1.0 - math.cos(partial)) * (skew @ skew))

    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = ((1.0 - weight) * left[:3, 3]
                     + weight * right[:3, 3])
    return result


def repair_isolated_pose(left_stamp_ns, left, stamp_ns, pose,
                         right_stamp_ns, right, translation_m=0.25,
                         rotation_deg=15.0, max_span_s=0.5,
                         max_endpoint_speed_m_s=2.0,
                         max_endpoint_speed_deg_s=180.0):
    """Replace only a one-sample spike confirmed by two continuous neighbours."""
    pose = np.asarray(pose, float)
    expected = interpolate_se3(
        stamp_ns, left_stamp_ns, left, right_stamp_ns, right, max_span_s)
    if expected is None or not valid_se3(pose):
        return pose.copy(), False
    span_s = (int(right_stamp_ns) - int(left_stamp_ns)) / 1e9
    endpoint_rotation, endpoint_translation = pose_distance(left, right)
    if (endpoint_translation / span_s > float(max_endpoint_speed_m_s)
            or endpoint_rotation / span_s > float(max_endpoint_speed_deg_s)):
        return pose.copy(), False
    left_s = (int(stamp_ns) - int(left_stamp_ns)) / 1e9
    right_s = (int(right_stamp_ns) - int(stamp_ns)) / 1e9
    left_rotation, left_translation = pose_distance(left, pose)
    right_rotation, right_translation = pose_distance(pose, right)
    if (left_translation / left_s <= float(max_endpoint_speed_m_s)
            and right_translation / right_s <= float(max_endpoint_speed_m_s)
            and left_rotation / left_s <= float(max_endpoint_speed_deg_s)
            and right_rotation / right_s <= float(max_endpoint_speed_deg_s)):
        return pose.copy(), False
    residual_rotation, residual_translation = pose_distance(pose, expected)
    repaired = (residual_translation > float(translation_m)
                or residual_rotation > float(rotation_deg))
    return (expected if repaired else pose.copy()), repaired


def dominant_pose_cluster(poses, rotation_deg, translation_m, symmetry_axis=None,
                          symmetry_step_deg=10):
    """Return largest centre-neighbourhood; ties prefer the earlier observation."""
    best = []
    for centre, pose in enumerate(poses):
        members = []
        for index, other in enumerate(poses):
            angle, translation = pose_distance(
                pose, other, symmetry_axis, symmetry_step_deg)
            if angle <= rotation_deg and translation <= translation_m:
                members.append(index)
        if len(members) > len(best):
            best = members
    return best


def pose_medoid(poses, indices, symmetry_axis=None, symmetry_step_deg=10,
                rotation_scale_deg=20.0, translation_scale_m=0.05):
    """Choose an actual observation minimizing total normalized pose distance."""
    best_index, best_cost = None, None
    for candidate in indices:
        cost = 0.0
        for other in indices:
            angle, translation = pose_distance(
                poses[candidate], poses[other], symmetry_axis, symmetry_step_deg)
            cost += angle / rotation_scale_deg + translation / translation_scale_m
        if best_cost is None or cost < best_cost:
            best_index, best_cost = candidate, cost
    return best_index


class ObjectAnchorManager:
    """Session-only SLAM-map anchors learned from valid SAM-6D observations."""

    DEFAULTS = {
        "window": 20,
        "register_count": 16,
        "register_rotation_deg": 20.0,
        "register_translation_mm": 50.0,
        "release_window": 5,
        "release_count": 4,
        # Static map objects do not move because SAM-6D suddenly flips 45/100/180°.
        # Keep the trusted orientation; only a sustained physical displacement
        # releases the anchor. 150 mm stays above this rig's 35–45 mm p90 residual.
        "release_rotation_deg": 180.0,
        "release_translation_mm": 150.0,
        "timestamp_tolerance_ms": 100.0,
        "anchor_output_mode": "ism_associated",
        "ism_overlap_min": 0.5,
        "ism_depth_tolerance_mm": 100.0,
        "fov_fraction_min": 0.2,
        "sym_step_deg": 10,
        "symmetry_axes": {},
    }

    def __init__(self, map_id, config=None):
        self.config = dict(self.DEFAULTS, **(config or {}))
        mode = self.config["anchor_output_mode"]
        if mode not in ("ism_associated", "fov_always"):
            raise ValueError("anchor_output_mode must be ism_associated or fov_always")
        window = int(self.config["window"])
        register_count = int(self.config["register_count"])
        release_window = int(self.config["release_window"])
        release_count = int(self.config["release_count"])
        sym_step = int(self.config["sym_step_deg"])
        if window < 1 or not 1 <= register_count <= window:
            raise ValueError("register_count must be in [1, window]")
        if release_window < 1 or not 1 <= release_count <= release_window:
            raise ValueError("release_count must be in [1, release_window]")
        if sym_step < 1 or 360 % sym_step:
            raise ValueError("sym_step_deg must be a positive divisor of 360")
        for key in ("register_rotation_deg", "register_translation_mm",
                    "release_rotation_deg", "release_translation_mm",
                    "timestamp_tolerance_ms", "ism_depth_tolerance_mm"):
            if not np.isfinite(float(self.config[key])) or float(self.config[key]) < 0:
                raise ValueError(f"{key} must be finite and non-negative")
        for key in ("ism_overlap_min", "fov_fraction_min"):
            if not 0.0 <= float(self.config[key]) <= 1.0:
                raise ValueError(f"{key} must be in [0, 1]")
        self.map_id = str(map_id)
        self.histories = defaultdict(lambda: deque(maxlen=window))
        self.validations = defaultdict(
            lambda: deque(maxlen=release_window))
        self.anchors = {}

    def set_map(self, map_id):
        map_id = str(map_id)
        if map_id == self.map_id:
            return False
        self.map_id = map_id
        self.histories.clear()
        self.validations.clear()
        self.anchors.clear()
        return True

    def slam_pose_valid(self, camera_to_map, tracking_state, pose_stamp_ns, rgb_stamp_ns):
        delta_ms = abs(int(pose_stamp_ns) - int(rgb_stamp_ns)) / 1e6
        return (str(tracking_state) == "TRACKING_OK" and valid_se3(camera_to_map)
                and delta_ms <= float(self.config["timestamp_tolerance_ms"]))

    def _axis(self, object_id):
        return (self.config.get("symmetry_axes") or {}).get(object_id)

    def observe(self, object_id, camera_to_map, camera_to_object, tracking_state,
                pose_stamp_ns, rgb_stamp_ns):
        """Add one valid observation and register after a full 20-observation window."""
        if not self.slam_pose_valid(camera_to_map, tracking_state,
                                    pose_stamp_ns, rgb_stamp_ns):
            return {"accepted": False, "reason": "slam_pose_invalid"}
        camera_to_object = np.asarray(camera_to_object, float)
        if not valid_se3(camera_to_object):
            return {"accepted": False, "reason": "object_pose_invalid"}
        map_to_object = np.asarray(camera_to_map, float) @ camera_to_object
        if object_id in self.anchors:
            return self.validate_shadow(object_id, map_to_object)
        history = self.histories[object_id]
        history.append(map_to_object.copy())
        required_window = int(self.config["window"])
        if len(history) < required_window:
            return {"accepted": True, "registered": False,
                    "valid_observations": len(history)}
        poses = list(history)
        cluster = dominant_pose_cluster(
            poses, float(self.config["register_rotation_deg"]),
            float(self.config["register_translation_mm"]) / 1000.0,
            self._axis(object_id), int(self.config["sym_step_deg"]))
        if len(cluster) < int(self.config["register_count"]):
            return {"accepted": True, "registered": False,
                    "valid_observations": len(history), "cluster_size": len(cluster)}
        medoid = pose_medoid(
            poses, cluster, self._axis(object_id), int(self.config["sym_step_deg"]),
            float(self.config["register_rotation_deg"]),
            float(self.config["register_translation_mm"]) / 1000.0)
        self.anchors[object_id] = poses[medoid].copy()
        self.validations[object_id].clear()
        return {"accepted": True, "registered": True, "cluster_size": len(cluster),
                "medoid_observation": int(medoid), "map_id": self.map_id}

    def validate_shadow(self, object_id, map_to_object, mask_only_failure=False):
        """Release on 4/5 pose disagreements; a low Mask IoU alone is ignored."""
        if object_id not in self.anchors:
            return {"registered": False, "reason": "anchor_missing"}
        if mask_only_failure:
            return {"registered": True, "validation_ignored": "low_mask_iou_only"}
        if not valid_se3(map_to_object):
            return {"registered": True, "validation_ignored": "invalid_shadow_pose"}
        angle, translation = pose_distance(
            self.anchors[object_id], map_to_object, self._axis(object_id),
            int(self.config["sym_step_deg"]))
        mismatch = (angle > float(self.config["release_rotation_deg"]) or
                    translation > float(self.config["release_translation_mm"]) / 1000.0)
        validations = self.validations[object_id]
        validations.append(bool(mismatch))
        release = (len(validations) == int(self.config["release_window"])
                   and sum(validations) >= int(self.config["release_count"]))
        if release:
            del self.anchors[object_id]
            validations.clear()
            self.histories[object_id].clear()
        return {"registered": not release, "released": release,
                "rotation_deg": angle, "translation_mm": translation * 1000.0,
                "mismatch": bool(mismatch), "validation_count": len(validations)}

    def camera_pose(self, object_id, camera_to_map):
        if object_id not in self.anchors or not valid_se3(camera_to_map):
            return None
        return np.linalg.inv(np.asarray(camera_to_map, float)) @ self.anchors[object_id]

    @staticmethod
    def _project(points_m, camera_to_object, K, image_shape):
        points = np.asarray(points_m, float)
        pose = np.asarray(camera_to_object, float)
        K = np.asarray(K, float).reshape(3, 3)
        h, w = image_shape[:2]
        camera = points @ pose[:3, :3].T + pose[:3, 3]
        z = camera[:, 2]
        finite = np.isfinite(camera).all(1) & (z > 1e-6)
        u = K[0, 0] * camera[:, 0] / np.maximum(z, 1e-6) + K[0, 2]
        v = K[1, 1] * camera[:, 1] / np.maximum(z, 1e-6) + K[1, 2]
        inside = finite & (u >= 0) & (u < w) & (v >= 0) & (v < h)
        mask = np.zeros((h, w), np.uint8)
        if inside.any():
            mask[np.rint(v[inside]).astype(int).clip(0, h - 1),
                 np.rint(u[inside]).astype(int).clip(0, w - 1)] = 1
            try:
                import cv2
                mask = cv2.morphologyEx(
                    cv2.dilate(mask, np.ones((5, 5), np.uint8)), cv2.MORPH_CLOSE,
                    np.ones((3, 3), np.uint8))
            except ImportError:
                pass
        return mask.astype(bool), inside, z

    def output_decision(self, object_id, camera_to_map, points_m, K, image_shape,
                        ism_mask=None, depth_mm=None, mode=None):
        pose = self.camera_pose(object_id, camera_to_map)
        if pose is None:
            return None, {"output": False, "reason": "anchor_or_slam_pose_invalid"}
        rendered, inside, z = self._project(points_m, pose, K, image_shape)
        fov_fraction = float(inside.mean()) if inside.size else 0.0
        mode = mode or self.config["anchor_output_mode"]
        diag = {"pose_source": "slam_anchor", "map_id": self.map_id,
                "anchor_state": "registered", "anchor_output_mode": mode,
                "fov_fraction": fov_fraction}
        if mode == "fov_always":
            allowed = fov_fraction >= float(self.config["fov_fraction_min"])
            return (pose if allowed else None), {
                **diag, "output": allowed,
                "reason": None if allowed else "anchor_fov_below_threshold",
                "ism_present": ism_mask is not None,
            }
        if ism_mask is None:
            return None, {**diag, "output": False, "reason": "ism_association_missing"}
        ism = np.asarray(ism_mask, bool)
        intersection = int((rendered & ism).sum())
        overlap = intersection / max(int(rendered.sum()), 1)
        observed = np.asarray(depth_mm)[ism & (np.asarray(depth_mm) > 0)] if depth_mm is not None else []
        expected_depth = float(np.median(z[inside]) * 1000.0) if inside.any() else math.nan
        observed_depth = float(np.median(observed)) if len(observed) else math.nan
        depth_delta = abs(expected_depth - observed_depth)
        allowed = (overlap >= float(self.config["ism_overlap_min"])
                   and math.isfinite(depth_delta)
                   and depth_delta <= float(self.config["ism_depth_tolerance_mm"]))
        return (pose if allowed else None), {
            **diag, "output": allowed, "anchor_ism_overlap": overlap,
            "anchor_depth_delta_mm": depth_delta if math.isfinite(depth_delta) else None,
            "reason": None if allowed else "anchor_ism_association_failed",
        }


def pose_matrix(rotation, translation):
    value = np.eye(4, dtype=float)
    value[:3, :3] = np.asarray(rotation, dtype=float)
    value[:3, 3] = np.asarray(translation, dtype=float)
    return value


def project_pose_axes(camera_to_object, K, axis_length_m=0.08):
    """Return image pixels for an object's origin and x/y/z axis endpoints."""
    pose, camera = np.asarray(camera_to_object, float), np.asarray(K, float)
    if (not valid_se3(pose) or camera.shape != (3, 3)
            or not np.isfinite(camera).all() or axis_length_m <= 0):
        return None
    points = np.vstack((np.zeros(3), np.eye(3) * float(axis_length_m)))
    points = points @ pose[:3, :3].T + pose[:3, 3]
    if not np.isfinite(points).all() or np.any(points[:, 2] <= 1e-6):
        return None
    pixels = np.column_stack((camera[0, 0] * points[:, 0] / points[:, 2] + camera[0, 2],
                              camera[1, 1] * points[:, 1] / points[:, 2] + camera[1, 2]))
    return pixels if np.isfinite(pixels).all() else None


def project_pose_box(camera_to_object, K, extent_m):
    """Return the eight projected corners of a CAD axis-aligned extent."""
    pose, camera = np.asarray(camera_to_object, float), np.asarray(K, float)
    extent = np.asarray(extent_m, float)
    if (not valid_se3(pose) or camera.shape != (3, 3) or extent.shape != (2, 3)
            or not np.isfinite(camera).all() or not np.isfinite(extent).all()
            or np.any(extent[1] <= extent[0])):
        return None
    lo, hi = extent
    points = np.asarray([[x, y, z] for x in (lo[0], hi[0])
                         for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    points = points @ pose[:3, :3].T + pose[:3, 3]
    if np.any(points[:, 2] <= 1e-6):
        return None
    pixels = np.column_stack((camera[0, 0] * points[:, 0] / points[:, 2] + camera[0, 2],
                              camera[1, 1] * points[:, 1] / points[:, 2] + camera[1, 2]))
    return pixels if np.isfinite(pixels).all() else None


class StaticSlamPoseMemory:
    """Project trusted map-frame object anchors into the current camera frame."""

    def __init__(self, anchors):
        self.anchors = {}
        for name, row in anchors.items():
            if not row.get("trusted", False):
                continue
            value = pose_matrix(row["R"], row["t_m"])
            rotation = value[:3, :3]
            if (not np.isfinite(value).all() or
                    not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3) or
                    np.linalg.det(rotation) < 0.99):
                raise ValueError(f"invalid SE(3) anchor for {name}")
            self.anchors[name] = value

    @classmethod
    def from_evaluation(cls, path):
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        anchors = payload.get("anchors", payload)
        return cls(anchors)

    def objects(self):
        return tuple(sorted(self.anchors))

    def camera_pose(self, object_name, camera_to_map):
        """Return ``T_camera_object`` from ``T_map_camera`` (ORB trajectory T_wc)."""
        if object_name not in self.anchors:
            return None
        twc = np.asarray(camera_to_map, dtype=float)
        if (twc.shape != (4, 4) or not np.isfinite(twc).all() or
                not np.allclose(twc[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6)):
            raise ValueError("camera_to_map must be a finite 4x4 T_wc matrix")
        return np.linalg.inv(twc) @ self.anchors[object_name]
