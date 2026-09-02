"""In-memory persistent object store for object_memory.

ObjectMemoryStore is the single owner of persistent ``object_id``. It creates
tentative landmarks, auto-increments identities, applies validated lifecycle
transitions, and exposes diagnostic summaries. It performs no association
matching.
"""

from __future__ import annotations

from dataclasses import fields as dataclass_fields
from typing import Dict, List, Optional, Tuple

from core.models import Matrix4, ObjectLandmark, TrackingStatus
from core.state_machine import (
    ObjectStatus,
    StateTransition,
    build_transition,
    coerce_status,
)

_LANDMARK_FIELDS = {f.name for f in dataclass_fields(ObjectLandmark)}


class ObjectMemoryStore:
    def __init__(self, starting_object_id: int = 1):
        if starting_object_id < 1:
            raise ValueError("starting_object_id must be >= 1")
        self._next_object_id = starting_object_id
        self._landmarks: Dict[int, ObjectLandmark] = {}
        self._transition_log: List[Tuple[int, StateTransition]] = []

    # ----- creation -------------------------------------------------------
    def create_tentative(
        self,
        object_name: str,
        T_map_obj: Matrix4,
        first_seen_time: float,
        confidence: float = 0.0,
        class_id: Optional[int] = None,
        last_detection_id: Optional[int] = None,
        last_sam6d_score: Optional[float] = None,
        source_slam_status: Optional[TrackingStatus] = None,
    ) -> ObjectLandmark:
        object_id = self._next_object_id
        self._next_object_id += 1
        landmark = ObjectLandmark(
            object_id=object_id,
            object_name=object_name,
            status=ObjectStatus.tentative,
            T_map_obj=T_map_obj,
            confidence=confidence,
            first_seen_time=first_seen_time,
            last_seen_time=first_seen_time,
            observation_count=1,
            missed_count=0,
            last_detection_id=last_detection_id,
            last_sam6d_score=last_sam6d_score,
            source_slam_status=source_slam_status,
            class_id=class_id,
        )
        self._landmarks[object_id] = landmark
        return landmark

    # ----- queries --------------------------------------------------------
    def get(self, object_id: int) -> Optional[ObjectLandmark]:
        return self._landmarks.get(object_id)

    def require(self, object_id: int) -> ObjectLandmark:
        if object_id not in self._landmarks:
            raise KeyError(f"unknown object_id: {object_id}")
        return self._landmarks[object_id]

    def all_landmarks(self) -> List[ObjectLandmark]:
        return list(self._landmarks.values())

    def by_status(self, status) -> List[ObjectLandmark]:
        target = coerce_status(status)
        return [lm for lm in self._landmarks.values() if lm.status is target]

    def landmarks_by_name(self, object_name: str) -> List[ObjectLandmark]:
        """All landmarks (any status) of a class, ordered by object_id."""
        return sorted(
            (lm for lm in self._landmarks.values()
             if lm.object_name == object_name),
            key=lambda lm: lm.object_id,
        )

    def active_objects(self) -> List[ObjectLandmark]:
        return self.by_status(ObjectStatus.active)

    def remembered_or_lost_objects(self) -> List[ObjectLandmark]:
        return [
            lm
            for lm in self._landmarks.values()
            if lm.status in (ObjectStatus.remembered, ObjectStatus.lost)
        ]

    # ----- mutation -------------------------------------------------------
    def update_landmark(self, object_id: int, **changes) -> ObjectLandmark:
        landmark = self.require(object_id)
        if "object_id" in changes:
            raise ValueError("object_id cannot be changed")
        if "status" in changes:
            raise ValueError(
                "status cannot be changed directly; use apply_transition"
            )
        unknown = set(changes) - _LANDMARK_FIELDS
        if unknown:
            raise ValueError(f"unknown landmark fields: {sorted(unknown)}")
        for key, value in changes.items():
            setattr(landmark, key, value)
        return landmark

    def apply_transition(
        self,
        object_id: int,
        to_status,
        reason: str,
        stamp: Optional[float] = None,
        metadata: Optional[dict] = None,
    ) -> StateTransition:
        landmark = self.require(object_id)
        transition = build_transition(
            landmark.status, to_status, reason, stamp=stamp, metadata=metadata
        )
        landmark.status = transition.to_status
        landmark.transition_history.append(transition)
        self._transition_log.append((object_id, transition))
        return transition

    # ----- diagnostics ----------------------------------------------------
    def transition_log(self) -> List[Tuple[int, StateTransition]]:
        return list(self._transition_log)

    def debug_summary(self) -> dict:
        status_counts: Dict[str, int] = {}
        for lm in self._landmarks.values():
            status_counts[lm.status.value] = (
                status_counts.get(lm.status.value, 0) + 1
            )
        return {
            "object_count": len(self._landmarks),
            "next_object_id": self._next_object_id,
            "status_counts": status_counts,
            "transition_count": len(self._transition_log),
        }
