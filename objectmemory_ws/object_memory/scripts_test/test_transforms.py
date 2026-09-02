"""Unit tests for core.transforms (in-memory synthetic poses only)."""

import math

from core.transforms import (
    compose_transform,
    compute_T_map_obj,
    identity_transform,
    invert_transform,
    make_transform,
    make_transform_from_quat,
    predict_current_camera_object_pose,
    quat_to_rotation,
    transform_distance_rotation_deg,
    transform_distance_translation,
)

I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rot_z(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _almost_equal_m4(a, b, tol=1e-9):
    for i in range(4):
        for j in range(4):
            assert abs(a[i][j] - b[i][j]) < tol, (i, j, a[i][j], b[i][j])


def test_identity_compose():
    T = make_transform(_rot_z(30), (1.0, 2.0, 3.0))
    _almost_equal_m4(compose_transform(identity_transform(), T), T)
    _almost_equal_m4(compose_transform(T, identity_transform()), T)


def test_inverse_round_trip():
    T = make_transform(_rot_z(47), (1.5, -2.0, 0.7))
    _almost_equal_m4(compose_transform(T, invert_transform(T)), identity_transform())
    _almost_equal_m4(compose_transform(invert_transform(T), T), identity_transform())


def test_T_map_obj_formula():
    T_map_cam = make_transform(I3, (10.0, 0.0, 0.0))
    T_cam_obj = make_transform(I3, (0.5, 1.0, 2.0))
    T_map_obj = compute_T_map_obj(T_map_cam, T_cam_obj)
    # pure translation compose -> translations add.
    assert abs(T_map_obj[0][3] - 10.5) < 1e-9
    assert abs(T_map_obj[1][3] - 1.0) < 1e-9
    assert abs(T_map_obj[2][3] - 2.0) < 1e-9


def test_predict_current_camera_object_pose_formula():
    T_map_cam_curr = make_transform(_rot_z(20), (3.0, 1.0, 0.0))
    T_cam_obj = make_transform(_rot_z(10), (0.2, 0.4, 1.0))
    T_map_obj_prev = compute_T_map_obj(T_map_cam_curr, T_cam_obj)
    pred = predict_current_camera_object_pose(T_map_cam_curr, T_map_obj_prev)
    # If the object did not move, predicted cam->obj must equal the true cam->obj.
    _almost_equal_m4(pred, T_cam_obj)


def test_translation_distance():
    a = make_transform(I3, (0.0, 0.0, 0.0))
    b = make_transform(I3, (3.0, 4.0, 0.0))
    assert abs(transform_distance_translation(a, b) - 5.0) < 1e-9


def test_rotation_distance_deg():
    a = make_transform(_rot_z(10), (0.0, 0.0, 0.0))
    b = make_transform(_rot_z(55), (5.0, 5.0, 5.0))
    assert abs(transform_distance_rotation_deg(a, b) - 45.0) < 1e-6
    assert abs(transform_distance_rotation_deg(a, a)) < 1e-6


def test_quat_identity():
    R = quat_to_rotation(0.0, 0.0, 0.0, 1.0)
    for i in range(3):
        for j in range(3):
            assert abs(R[i][j] - (1.0 if i == j else 0.0)) < 1e-9


def test_quat_matches_rot_z():
    # 90 deg about z: qz = sin(45), qw = cos(45).
    s = math.sin(math.radians(45))
    T_quat = make_transform_from_quat((1.0, 2.0, 3.0), (0.0, 0.0, s, s))
    T_rot = make_transform(_rot_z(90), (1.0, 2.0, 3.0))
    _almost_equal_m4(T_quat, T_rot, tol=1e-9)


def test_quat_zero_norm_rejected():
    import pytest

    with pytest.raises(ValueError):
        quat_to_rotation(0.0, 0.0, 0.0, 0.0)
