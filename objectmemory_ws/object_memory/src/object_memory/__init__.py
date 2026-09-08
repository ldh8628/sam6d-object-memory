"""Optional compatibility re-export package.

The actual implementation lives in ``core``. This package only re-exposes the
public contracts for convenience; it holds no logic of its own.
"""

from core.models import (
    AssociationDecision,
    DecisionType,
    ObjectLandmark,
    Sam6DDetection,
    SlamCameraPose,
    TrackingStatus,
)
from core.state_machine import ObjectStatus

__all__ = [
    "AssociationDecision",
    "DecisionType",
    "ObjectLandmark",
    "ObjectStatus",
    "Sam6DDetection",
    "SlamCameraPose",
    "TrackingStatus",
]
