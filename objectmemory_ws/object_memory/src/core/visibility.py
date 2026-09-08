"""Expected probability of detection P_D from landmark visibility.

Because object_memory knows both the camera pose (SLAM) and the landmark pose
(map frame), it can decide whether a landmark is inside the camera view THIS
frame. A landmark outside the view gets P_D = 0, so a missed detection carries
no existence penalty (see core.existence).

With intrinsics: reproject the landmark into the image and require the pixel
to fall inside the frame with a relative margin. Without intrinsics: fall back
to an angular field-of-view test around the optical axis.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

PD_BASE = 0.5            # expected in-view detection rate. Calibrated to the
                        # measured recall (9-bag audit 0.62 overall, but harder
                        # objects like loop1 choco run ~0.48). At 0.6 the filter
                        # over-penalised normal detector misses and flickered
                        # real-but-intermittent objects to 'lost'; 0.5 matches
                        # reality. Safe because Story-5 quality weighting, not
                        # the miss penalty, is what suppresses phantoms.
FOV_MARGIN = 0.05        # ignore detections this close to the image border
FALLBACK_HALF_ANGLE_DEG = 35.0

Matrix4 = Sequence[Sequence[float]]


def _point_in_camera(T_map_cam: Matrix4, T_map_obj: Matrix4) -> Tuple[float, float, float]:
    """Landmark position expressed in the camera frame: R^T (p_obj - t_cam)."""
    dx = T_map_obj[0][3] - T_map_cam[0][3]
    dy = T_map_obj[1][3] - T_map_cam[1][3]
    dz = T_map_obj[2][3] - T_map_cam[2][3]
    # rows of R^T are columns of R
    x = T_map_cam[0][0] * dx + T_map_cam[1][0] * dy + T_map_cam[2][0] * dz
    y = T_map_cam[0][1] * dx + T_map_cam[1][1] * dy + T_map_cam[2][1] * dz
    z = T_map_cam[0][2] * dx + T_map_cam[1][2] * dy + T_map_cam[2][2] * dz
    return x, y, z


def expected_p_d(
    T_map_cam: Matrix4,
    T_map_obj: Matrix4,
    cam_K: Optional[Sequence[Sequence[float]]] = None,
    img_size: Optional[Tuple[int, int]] = None,
    pd_base: float = PD_BASE,
    margin: float = FOV_MARGIN,
    fallback_half_angle_deg: float = FALLBACK_HALF_ANGLE_DEG,
) -> float:
    """Expected detection probability of a landmark for this camera pose."""
    x, y, z = _point_in_camera(T_map_cam, T_map_obj)
    if z <= 0.0:
        return 0.0
    if cam_K is not None and img_size is not None:
        w, h = img_size
        u = cam_K[0][0] * x / z + cam_K[0][2]
        v = cam_K[1][1] * y / z + cam_K[1][2]
        mu, mv = w * margin, h * margin
        if mu <= u <= w - mu and mv <= v <= h - mv:
            return pd_base
        return 0.0
    angle = math.degrees(math.atan2(math.hypot(x, y), z))
    return pd_base if angle <= fallback_half_angle_deg else 0.0
