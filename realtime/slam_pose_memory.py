"""Static-object pose stabilization using an already learned SLAM-map anchor.

This module is deliberately separate from standalone PEM scoring.  It is useful
when objects are fixed in the scene and a clean initialization interval has
provided one trusted object pose in the SLAM map frame.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def pose_matrix(rotation, translation):
    value = np.eye(4, dtype=float)
    value[:3, :3] = np.asarray(rotation, dtype=float)
    value[:3, 3] = np.asarray(translation, dtype=float)
    return value


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
