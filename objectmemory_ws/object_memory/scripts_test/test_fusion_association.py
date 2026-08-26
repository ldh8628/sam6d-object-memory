"""Tests for short-term geometric association + incremental-mean pose fusion."""

import math

from core.fusion import fuse_pose
from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus
from core.state_machine import ObjectStatus
from core.transforms import (
    identity_transform,
    make_transform,
    make_transform_from_quat,
    quat_nlerp,
    quat_to_rotation,
    rotation_to_quat,
    transform_distance_rotation_deg,
    transform_distance_translation,
)
from pipeline.object_memory_runner import run_object_memory


# ----- transforms ---------------------------------------------------------
def _rot_z(deg):
    return quat_to_rotation(0.0, 0.0, math.sin(math.radians(deg) / 2),
                            math.cos(math.radians(deg) / 2))


def test_rotation_to_quat_roundtrip():
    for deg in (0, 30, 90, 179):
        R = _rot_z(deg)
        q = rotation_to_quat(R)
        R2 = quat_to_rotation(*q)
        for i in range(3):
            for j in range(3):
                assert abs(R[i][j] - R2[i][j]) < 1e-9


def test_quat_nlerp_endpoints_and_midpoint():
    qa = rotation_to_quat(_rot_z(0))
    qb = rotation_to_quat(_rot_z(90))
    assert quat_nlerp(qa, qb, 0.0) == qa
    mid = quat_nlerp(qa, qb, 0.5)
    # midpoint should be a unit quaternion near a 45-deg rotation.
    assert abs(math.sqrt(sum(c * c for c in mid)) - 1.0) < 1e-9
    R = quat_to_rotation(*mid)
    ang = transform_distance_rotation_deg(identity_transform(), make_transform(R, (0, 0, 0)))
    assert 40.0 < ang < 50.0


# ----- fusion -------------------------------------------------------------
def test_fuse_first_obs_returns_measurement():
    meas = make_transform(_rot_z(10), (1.0, 2.0, 3.0))
    assert fuse_pose(identity_transform(), 0, meas) == meas


def test_fuse_incremental_mean_translation():
    # seed at x=0, then fuse x=1 with n_prev=1 -> mean 0.5
    prev = make_transform(_rot_z(0), (0.0, 0.0, 0.0))
    fused = fuse_pose(prev, 1, make_transform(_rot_z(0), (1.0, 0.0, 0.0)))
    assert abs(fused[0][3] - 0.5) < 1e-9


def test_fusion_reduces_jitter():
    # noisy measurements around a true point; fused variance << raw variance.
    true = (2.0, -1.0, 5.0)
    noise = [0.1, -0.12, 0.08, -0.09, 0.11, -0.07, 0.05, -0.06]
    fused = make_transform(_rot_z(0), true)
    n = 0
    raw_devs, fused_devs = [], []
    for k in noise:
        meas = make_transform(_rot_z(0), (true[0] + k, true[1] - k, true[2] + k))
        fused = fuse_pose(fused, n, meas)
        n += 1
        raw_devs.append(abs(k))
        fused_devs.append(abs(fused[0][3] - true[0]))
    assert max(fused_devs[3:]) < max(raw_devs) / 2.0


# ----- association (via runner) -------------------------------------------
def _pose(stamp, xyz=(0.0, 0.0, 0.0)):
    return SlamCameraPose(stamp=stamp, frame_id="map", child_frame_id="cam",
                          T_map_cam=make_transform(_rot_z(0), xyz),
                          tracking_status=TrackingStatus.OK,
                          source_slam_id="unit")


def _det(did, name, xyz, score=0.9):
    # camera at identity -> T_cam_obj == T_map_obj; translation in metres.
    return Sam6DDetection(stamp=0.0, frame_id="cam", detection_id=did,
                          object_name=name,
                          T_cam_obj=make_transform(_rot_z(0), xyz), score=score)


def test_jitter_within_gate_matches_and_fuses():
    poses = [_pose(float(i)) for i in range(4)]
    ts = [float(i) for i in range(4)]
    dets = {i: [_det(i, "milk", (1.0 + 0.02 * (i % 2), 0.0, 1.0))] for i in range(4)}
    r = run_object_memory(poses, dets, ts)
    assert len(r.store.all_landmarks()) == 1
    lm = r.store.all_landmarks()[0]
    assert lm.status is ObjectStatus.active
    assert lm.observation_count == 4  # all four accepted


def test_outlier_beyond_gate_spawns_separate_instance_without_corrupting_pose():
    # multi-instance: a detection beyond the gate of every live instance of its
    # class no longer corrupts the matched pose NOR is silently dropped - it
    # SPAWNS its own tentative instance (which, unreinforced, stays tentative).
    poses = [_pose(float(i)) for i in range(4)]
    ts = [float(i) for i in range(4)]
    dets = {
        0: [_det(0, "milk", (1.0, 0.0, 1.0))],
        1: [_det(1, "milk", (1.0, 0.0, 1.0))],   # promote to active, fused ~1.0
        2: [_det(2, "milk", (9.0, 0.0, 1.0))],   # gross outlier -> new instance
        3: [_det(3, "milk", (1.0, 0.0, 1.0))],   # re-associates to instance #1
    }
    r = run_object_memory(poses, dets, ts, assoc_trans_gate_m=0.15)
    milk = r.store.landmarks_by_name("milk")
    assert len(milk) == 2                              # primary + outlier spawn
    primary, outlier = milk[0], milk[1]
    assert abs(primary.T_map_obj[0][3] - 1.0) < 0.05   # outlier did NOT move it
    assert primary.observation_count == 3              # frames 0,1,3
    assert abs(outlier.T_map_obj[0][3] - 9.0) < 1e-6   # spawned at the outlier
    assert outlier.observation_count == 1              # one-shot, not promoted
    spawned = [dc for fr in r.frame_results for dc in fr.decisions
               if dc.decision_type.value == "new_tentative"]
    assert len(spawned) == 2                           # first sighting + outlier


def test_low_score_rejected():
    poses = [_pose(0.0), _pose(1.0)]
    ts = [0.0, 1.0]
    dets = {0: [_det(0, "milk", (1.0, 0.0, 1.0), score=0.9)],
            1: [_det(1, "milk", (1.0, 0.0, 1.0), score=0.05)]}
    r = run_object_memory(poses, dets, ts, score_floor=0.2)
    lm = r.store.all_landmarks()[0]
    assert lm.observation_count == 1  # low-score frame not fused
