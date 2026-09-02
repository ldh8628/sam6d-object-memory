"""Core data models for object_memory.

These dataclasses define the contracts exchanged between SLAM adapters,
SAM-6D detection producers, and the persistent object memory. They contain no
matching logic; validation is limited to structural invariants.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from core.state_machine import ObjectStatus, StateTransition

Matrix4 = tuple  # 4x4 nested tuple of floats (see core.transforms)


class TrackingStatus(str, Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    LOST = "LOST"
    RELOCALIZING = "RELOCALIZING"
    LOOP_CLOSING = "LOOP_CLOSING"
    RESET = "RESET"


class DecisionType(str, Enum):
    short_term_match = "short_term_match"
    long_term_match = "long_term_match"
    new_tentative = "new_tentative"
    rejected = "rejected"
    ambiguous = "ambiguous"


@dataclass
class SlamCameraPose:
    stamp: float
    frame_id: str
    child_frame_id: str
    T_map_cam: Matrix4
    tracking_status: TrackingStatus
    source_slam_id: str
    confidence: Optional[float] = None
    map_version: Optional[int] = None


@dataclass
class Sam6DDetection:
    stamp: float
    frame_id: str
    detection_id: int
    object_name: str
    T_cam_obj: Matrix4
    score: float
    bbox: Optional[tuple] = None
    depth_quality: Optional[float] = None
    model_id: Optional[str] = None
    size_extent: Optional[tuple] = None
    class_id: Optional[int] = None


@dataclass
class FusedPoseSample:
    """관측 하나를 받아들인 직후의 융합 포즈 한 점."""

    stamp: float
    observation_count: int
    xyz: tuple
    quat_xyzw: tuple


@dataclass
class ObjectLandmark:
    object_id: int
    object_name: str
    status: ObjectStatus
    T_map_obj: Matrix4
    confidence: float
    first_seen_time: float
    last_seen_time: float
    observation_count: int = 1
    missed_count: int = 0
    T_map_obj_smoothed: Optional[Matrix4] = None
    last_detection_id: Optional[int] = None
    last_sam6d_score: Optional[float] = None
    source_slam_status: Optional[TrackingStatus] = None
    class_id: Optional[int] = None
    transition_history: List[StateTransition] = field(default_factory=list)
    #: 관측을 받아들일 때마다의 융합 포즈. fuse_pose 의 가중치가 1/(n+1) 이라 융합 포즈는
    #: 1/n 으로 수렴하는데, 최종값만 저장하면 그 수렴 과정이 보이지 않는다. 항목당 수십
    #: 바이트라 검출 1,159 건 기준 0.15 MB 수준이다.
    pose_history: List["FusedPoseSample"] = field(default_factory=list)
    #: 회전 모드 다수결용 봉우리 목록 [(quat_xyzw, 표), ...] (표 내림차순).
    #: core.fusion.update_rotation_modes 가 제자리에서 갱신한다. 판정에 쓰이는 것은
    #: 맨 앞 원소뿐이고, 점유율(=자세 신뢰도)을 얻으려면 fusion.mode_occupancy 를 쓴다.
    rot_modes: List[tuple] = field(default_factory=list)


@dataclass
class AssociationDecision:
    decision_type: DecisionType
    detection_id: int
    score: float
    object_id: Optional[int] = None
    reject_reason: Optional[str] = None
    debug_info: dict = field(default_factory=dict)

    def __post_init__(self):
        if not isinstance(self.decision_type, DecisionType):
            self.decision_type = DecisionType(self.decision_type)
        if not (0.0 <= self.score <= 1.0):
            raise ValueError("score must be in [0, 1]")
        if self.decision_type in (
            DecisionType.short_term_match,
            DecisionType.long_term_match,
        ):
            if self.object_id is None:
                raise ValueError(
                    f"{self.decision_type.value} requires object_id"
                )
        if self.decision_type is DecisionType.rejected:
            if not self.reject_reason:
                raise ValueError("rejected decision requires reject_reason")
        # detection_id and object_id are deliberately separate fields.
        if self.object_id is not None and self.object_id == self.detection_id:
            # Not an error, but a strong smell: identity must not be aliased.
            # We record it for diagnostics rather than raising.
            self.debug_info.setdefault(
                "note", "object_id equals detection_id value"
            )
