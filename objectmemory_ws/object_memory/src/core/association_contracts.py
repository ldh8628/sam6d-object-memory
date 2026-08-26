"""Association input/output contracts for object_memory.

These dataclasses define the shape of data flowing into and out of the (future)
association stage. No matching logic is implemented here; later stories
(Story 4 short-term, Story 5 long-term) will consume these contracts.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from core.models import (
    AssociationDecision,
    ObjectLandmark,
    Sam6DDetection,
    SlamCameraPose,
)


@dataclass
class AssociationInput:
    detections: List[Sam6DDetection] = field(default_factory=list)
    slam_pose: Optional[SlamCameraPose] = None
    active_tracks: List[ObjectLandmark] = field(default_factory=list)
    memory_candidates: List[ObjectLandmark] = field(default_factory=list)


@dataclass
class AssociationOutput:
    decisions: List[AssociationDecision] = field(default_factory=list)
    updated_landmarks: List[ObjectLandmark] = field(default_factory=list)
    debug_summary: dict = field(default_factory=dict)
