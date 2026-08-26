"""Rigid transform utilities for object_memory (standard library only).

Transforms are represented as immutable 4x4 nested tuples of floats:

    (
        (r00, r01, r02, tx),
        (r10, r11, r12, ty),
        (r20, r21, r22, tz),
        (0.0, 0.0, 0.0, 1.0),
    )

Core formulas provided:

    T_map_obj             = T_map_cam * T_cam_obj
    T_cam_curr_obj_pred   = inverse(T_map_cam_curr) * T_map_obj_prev
"""

from __future__ import annotations

import math

Matrix3 = tuple  # 3x3 nested tuple of floats
Matrix4 = tuple  # 4x4 nested tuple of floats


def _validate_matrix4(transform) -> None:
    if not isinstance(transform, tuple) or len(transform) != 4:
        raise ValueError("transform must be a 4-row tuple")
    for row in transform:
        if not isinstance(row, tuple) or len(row) != 4:
            raise ValueError("transform must have 4 columns per row")


def make_transform(rotation, translation) -> Matrix4:
    """Build a 4x4 transform from a 3x3 rotation and a 3-vector translation."""
    if len(rotation) != 3 or any(len(r) != 3 for r in rotation):
        raise ValueError("rotation must be 3x3")
    if len(translation) != 3:
        raise ValueError("translation must have length 3")
    r = rotation
    t = translation
    return (
        (float(r[0][0]), float(r[0][1]), float(r[0][2]), float(t[0])),
        (float(r[1][0]), float(r[1][1]), float(r[1][2]), float(t[1])),
        (float(r[2][0]), float(r[2][1]), float(r[2][2]), float(t[2])),
        (0.0, 0.0, 0.0, 1.0),
    )


def identity_transform() -> Matrix4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def compose_transform(a: Matrix4, b: Matrix4) -> Matrix4:
    """Return the matrix product a * b (apply b first, then a)."""
    _validate_matrix4(a)
    _validate_matrix4(b)
    out = []
    for i in range(4):
        row = []
        for j in range(4):
            row.append(
                a[i][0] * b[0][j]
                + a[i][1] * b[1][j]
                + a[i][2] * b[2][j]
                + a[i][3] * b[3][j]
            )
        out.append(tuple(row))
    return tuple(out)


def invert_transform(transform: Matrix4) -> Matrix4:
    """Rigid-body inverse: R^T for rotation, -R^T t for translation."""
    _validate_matrix4(transform)
    r = transform
    # R^T
    rt = (
        (r[0][0], r[1][0], r[2][0]),
        (r[0][1], r[1][1], r[2][1]),
        (r[0][2], r[1][2], r[2][2]),
    )
    t = (r[0][3], r[1][3], r[2][3])
    # -R^T t
    nt = (
        -(rt[0][0] * t[0] + rt[0][1] * t[1] + rt[0][2] * t[2]),
        -(rt[1][0] * t[0] + rt[1][1] * t[1] + rt[1][2] * t[2]),
        -(rt[2][0] * t[0] + rt[2][1] * t[1] + rt[2][2] * t[2]),
    )
    return make_transform(rt, nt)


def _translation(transform: Matrix4):
    _validate_matrix4(transform)
    return (transform[0][3], transform[1][3], transform[2][3])


def transform_distance_translation(a: Matrix4, b: Matrix4) -> float:
    """Euclidean distance between the translation components of a and b."""
    ta = _translation(a)
    tb = _translation(b)
    return math.sqrt(
        (ta[0] - tb[0]) ** 2 + (ta[1] - tb[1]) ** 2 + (ta[2] - tb[2]) ** 2
    )


def transform_distance_rotation_deg(a: Matrix4, b: Matrix4) -> float:
    """Relative rotation angle between a and b, in degrees.

    Computes R_rel = R_a^T R_b and returns the geodesic angle
    arccos((trace(R_rel) - 1) / 2).
    """
    _validate_matrix4(a)
    _validate_matrix4(b)
    # R_a^T R_b, restricted to the 3x3 rotation blocks.
    trace = 0.0
    for i in range(3):
        # diagonal element (i, i) of R_a^T R_b = sum_k R_a[k][i] * R_b[k][i]
        trace += sum(a[k][i] * b[k][i] for k in range(3))
    cos_theta = (trace - 1.0) / 2.0
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return math.degrees(math.acos(cos_theta))


def quat_to_rotation(qx: float, qy: float, qz: float, qw: float) -> Matrix3:
    """Convert a (qx, qy, qz, qw) quaternion into a 3x3 rotation matrix.

    The quaternion is normalized first; a zero quaternion raises ValueError.
    """
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        raise ValueError("zero-norm quaternion")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return (
        (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ),
        (
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ),
        (
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ),
    )


def make_transform_from_quat(translation, quat) -> Matrix4:
    """Build a 4x4 transform from a translation and a (qx, qy, qz, qw) quat."""
    return make_transform(quat_to_rotation(*quat), translation)


def rotation_to_quat(rotation: Matrix3):
    """Convert a 3x3 rotation matrix into a normalized (qx, qy, qz, qw) quat."""
    m = rotation
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        raise ValueError("degenerate rotation matrix")
    return (qx / n, qy / n, qz / n, qw / n)


def quat_nlerp(qa, qb, w: float):
    """Normalized linear interpolation between two (qx,qy,qz,qw) quaternions.

    ``w`` is the weight of ``qb`` (w=0 -> qa, w=1 -> qb). ``qb`` is flipped into
    the same hemisphere as ``qa`` first so the short path is taken.
    """
    dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]
    if dot < 0.0:
        qb = (-qb[0], -qb[1], -qb[2], -qb[3])
    q = tuple(qa[i] * (1.0 - w) + qb[i] * w for i in range(4))
    n = math.sqrt(sum(c * c for c in q))
    if n == 0.0:
        return qa
    return tuple(c / n for c in q)


def compute_T_map_obj(T_map_cam: Matrix4, T_cam_obj: Matrix4) -> Matrix4:
    """T_map_obj = T_map_cam * T_cam_obj."""
    return compose_transform(T_map_cam, T_cam_obj)


def predict_current_camera_object_pose(
    T_map_cam_curr: Matrix4, T_map_obj_prev: Matrix4
) -> Matrix4:
    """T_cam_curr_obj_pred = inverse(T_map_cam_curr) * T_map_obj_prev."""
    return compose_transform(invert_transform(T_map_cam_curr), T_map_obj_prev)
