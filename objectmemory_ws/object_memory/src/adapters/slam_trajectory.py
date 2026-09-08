"""Load a SLAM camera trajectory (TUM format) into SlamCameraPose records.

TUM line format (space separated):

    timestamp tx ty tz qx qy qz qw

Each row becomes a SlamCameraPose whose T_map_cam is the SE(3) pose of the
camera in the map frame. Rows are returned sorted by timestamp.
"""

from __future__ import annotations

import os
from typing import List

from core.models import SlamCameraPose, TrackingStatus
from core.transforms import make_transform_from_quat


def load_slam_trajectory(
    path: str,
    source_slam_id: str = "orbslam3",
    frame_id: str = "map",
    child_frame_id: str = "camera",
    tracking_status: TrackingStatus = TrackingStatus.OK,
) -> List[SlamCameraPose]:
    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"trajectory file not found: {path}")

    poses: List[SlamCameraPose] = []
    with open(path, "r", encoding="utf-8") as f:
        for lineno, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                raise ValueError(
                    f"{path}:{lineno}: expected 8 TUM columns, got {len(parts)}"
                )
            ts, tx, ty, tz, qx, qy, qz, qw = (float(p) for p in parts)
            T_map_cam = make_transform_from_quat((tx, ty, tz), (qx, qy, qz, qw))
            poses.append(
                SlamCameraPose(
                    stamp=ts,
                    frame_id=frame_id,
                    child_frame_id=child_frame_id,
                    T_map_cam=T_map_cam,
                    tracking_status=tracking_status,
                    source_slam_id=source_slam_id,
                )
            )
    poses.sort(key=lambda p: p.stamp)
    return poses
