"""PoseBuffer -- a time-indexed window of SLAM camera poses (the "수첩").

Real-time object memory ingests two asynchronous streams: a FAST SLAM pose
stream (~30 Hz, low latency) and a SLOW SAM-6D detection stream (~few Hz, high
and variable latency). A SAM-6D result is stamped with the CAPTURE time of the
frame it was computed from -- always in the past by the time it arrives. To fuse
it into the map correctly we must multiply by the camera pose AT THAT CAPTURE
TIME, not the pose "now":

    T_map_obj = T_map_cam(capture_stamp) @ T_cam_obj      # time alignment

PoseBuffer records every incoming SlamCameraPose keyed by stamp, keeps only a
bounded recent window (so memory does not grow), and answers ``lookup(stamp)``
by returning the pose at that past instant -- interpolating between the two
bracketing samples when asked (decision (1) of the real-time design), or falling
back to nearest-within-tolerance. A stamp older than the window, or with no
sample within tolerance, returns None -> the caller drops that detection
(INV-008: no pose => do not corrupt memory).

Unlike the offline runner's ``_nearest_pose`` (which may peek at a FUTURE pose,
+/-1 sample), a streaming lookup only ever sees samples already added -- i.e. the
past. That is the whole difference between "batch merge" and "real-time".
"""

from __future__ import annotations

import bisect
from typing import List, Optional

from core.models import SlamCameraPose
from core.transforms import (
    make_transform_from_quat,
    quat_nlerp,
    rotation_to_quat,
)

DEFAULT_WINDOW_S = 10.0     # keep this many seconds of recent poses
DEFAULT_TOL_S = 0.05        # nearest-sample acceptance tolerance
DEFAULT_MAX_GAP_S = 0.20    # do not interpolate across a gap wider than this
                            # (a tracking dropout must not be bridged silently)


def _rotation_of(T) -> tuple:
    return ((T[0][0], T[0][1], T[0][2]),
            (T[1][0], T[1][1], T[1][2]),
            (T[2][0], T[2][1], T[2][2]))


def _translation_of(T) -> tuple:
    return (T[0][3], T[1][3], T[2][3])


def interpolate_pose(a: SlamCameraPose, b: SlamCameraPose,
                     stamp: float) -> SlamCameraPose:
    """Pose at ``stamp`` between bracketing samples a (<=stamp) and b (>=stamp).

    Translation is linearly interpolated; rotation uses normalized-lerp (the
    same short-path quaternion blend the codebase already uses for fusion). The
    result carries the metadata of the nearer sample; tracking_status is taken
    from the nearer sample too (a conservative choice near a state change)."""
    span = b.stamp - a.stamp
    w = 0.0 if span <= 0.0 else (stamp - a.stamp) / span
    ta, tb = _translation_of(a.T_map_cam), _translation_of(b.T_map_cam)
    trans = tuple(ta[i] * (1.0 - w) + tb[i] * w for i in range(3))
    qa = rotation_to_quat(_rotation_of(a.T_map_cam))
    qb = rotation_to_quat(_rotation_of(b.T_map_cam))
    quat = quat_nlerp(qa, qb, w)
    near = a if w <= 0.5 else b
    return SlamCameraPose(
        stamp=stamp,
        frame_id=near.frame_id,
        child_frame_id=near.child_frame_id,
        T_map_cam=make_transform_from_quat(trans, quat),
        tracking_status=near.tracking_status,
        source_slam_id=near.source_slam_id,
        confidence=near.confidence,
        map_version=near.map_version,
    )


class PoseBuffer:
    """Bounded, time-ordered store of camera poses with a stamped lookup."""

    def __init__(self, window_s: float = DEFAULT_WINDOW_S,
                 tol: float = DEFAULT_TOL_S,
                 interpolate: bool = True,
                 max_gap_s: float = DEFAULT_MAX_GAP_S):
        self.window_s = window_s
        self.tol = tol
        self.interpolate = interpolate
        self.max_gap_s = max_gap_s
        self._stamps: List[float] = []
        self._poses: List[SlamCameraPose] = []

    def __len__(self) -> int:
        return len(self._poses)

    def latest_stamp(self) -> Optional[float]:
        return self._stamps[-1] if self._stamps else None

    def add(self, pose: SlamCameraPose) -> None:
        """Insert a pose (kept sorted by stamp) and trim the old tail."""
        i = bisect.bisect_right(self._stamps, pose.stamp)
        self._stamps.insert(i, pose.stamp)
        self._poses.insert(i, pose)
        self._trim()

    def _trim(self) -> None:
        if not self._stamps:
            return
        cutoff = self._stamps[-1] - self.window_s
        # drop everything strictly older than the window's start.
        keep = bisect.bisect_left(self._stamps, cutoff)
        if keep > 0:
            del self._stamps[:keep]
            del self._poses[:keep]

    def lookup(self, stamp: float,
               tol: Optional[float] = None) -> Optional[SlamCameraPose]:
        """Camera pose at ``stamp`` (a past instant), or None.

        With interpolation on and both bracketing samples present within
        ``max_gap_s``, returns the interpolated pose. Otherwise returns the
        nearest sample within ``tol`` (mirrors the offline ``_nearest_pose``
        contract), or None if nothing qualifies."""
        if not self._stamps:
            return None
        tol = self.tol if tol is None else tol
        i = bisect.bisect_left(self._stamps, stamp)
        # exact hit
        if i < len(self._stamps) and self._stamps[i] == stamp:
            return self._poses[i]
        left = i - 1
        right = i
        have_left = left >= 0
        have_right = right < len(self._stamps)
        if (self.interpolate and have_left and have_right):
            gap = self._stamps[right] - self._stamps[left]
            if gap <= self.max_gap_s:
                return interpolate_pose(self._poses[left], self._poses[right],
                                        stamp)
        # fall back to nearest within tolerance
        best = None
        best_dt = tol
        for j in (left, right):
            if 0 <= j < len(self._stamps):
                dt = abs(self._stamps[j] - stamp)
                if dt <= best_dt:
                    best_dt = dt
                    best = self._poses[j]
        return best
