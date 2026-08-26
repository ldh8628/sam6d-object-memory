"""Object lifecycle state machine for object_memory.

Defines the ObjectStatus lifecycle enum, the StateTransition record, and the
allowed transitions between lifecycle states. Matching/association logic lives
elsewhere; this module only governs valid status changes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class ObjectStatus(str, Enum):
    tentative = "tentative"
    active = "active"
    lost = "lost"
    remembered = "remembered"
    merged = "merged"
    deleted = "deleted"


@dataclass
class StateTransition:
    from_status: ObjectStatus
    to_status: ObjectStatus
    reason: str
    stamp: Optional[float] = None
    metadata: dict = field(default_factory=dict)


# Explicit allowed transitions. `any -> deleted` is handled separately.
_ALLOWED = {
    (ObjectStatus.tentative, ObjectStatus.active),
    (ObjectStatus.tentative, ObjectStatus.deleted),
    (ObjectStatus.active, ObjectStatus.lost),
    (ObjectStatus.lost, ObjectStatus.active),
    (ObjectStatus.lost, ObjectStatus.remembered),
    (ObjectStatus.remembered, ObjectStatus.active),
    (ObjectStatus.active, ObjectStatus.merged),
    (ObjectStatus.remembered, ObjectStatus.merged),
}


def coerce_status(status) -> ObjectStatus:
    """Accept an ObjectStatus or its string value and return an ObjectStatus."""
    if isinstance(status, ObjectStatus):
        return status
    if isinstance(status, str):
        try:
            return ObjectStatus(status)
        except ValueError as exc:
            raise ValueError(f"unknown status: {status!r}") from exc
    raise ValueError(f"cannot coerce {status!r} to ObjectStatus")


def is_transition_allowed(from_status, to_status) -> bool:
    src = coerce_status(from_status)
    dst = coerce_status(to_status)
    if dst is ObjectStatus.deleted:
        return True  # any -> deleted
    return (src, dst) in _ALLOWED


def build_transition(
    from_status,
    to_status,
    reason: str,
    stamp: Optional[float] = None,
    metadata: Optional[dict] = None,
) -> StateTransition:
    src = coerce_status(from_status)
    dst = coerce_status(to_status)
    if not reason or not str(reason).strip():
        raise ValueError("transition reason must be non-empty")
    if not is_transition_allowed(src, dst):
        raise ValueError(f"transition not allowed: {src.value} -> {dst.value}")
    return StateTransition(
        from_status=src,
        to_status=dst,
        reason=str(reason),
        stamp=stamp,
        metadata=dict(metadata) if metadata else {},
    )
