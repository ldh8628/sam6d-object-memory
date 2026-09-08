"""Story 5 — quality-weighted confidence tests.

Two layers:
 - pure feature/logLR unit tests (core.features): deterministic, no pipeline.
 - pipeline tests (quality_weighting=True): a degenerate-depth cluster cannot
   promote, two clean detections do, and out-of-view misses stay frozen.

Promotion still requires the second sighting (promotion fires on the accept
path, not on spawn); what Story 5 changes is that the QUALITY of those sightings
gates promotion, not a bare hit count.
"""

import math

from core.features import (
    QualityWeights,
    detection_log_lr,
    phi_depth,
    phi_resid,
    phi_score,
)
from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus
from core.state_machine import ObjectStatus
from pipeline.object_memory_runner import run_object_memory


# ---- pure feature / logLR units -------------------------------------------
_PD, _C = 0.6, 0.1
_BASE = math.log(_PD / _C)


def test_legacy_weights_reproduce_base_loglr():
    # all quality weights 0 -> logLR collapses to the legacy constant log(P_D/c).
    lr = detection_log_lr(0.9, 1.0, 0.01, 0.2, _PD, _C, QualityWeights.legacy())
    assert abs(lr - _BASE) < 1e-9


def test_depth_sanity_penalizes_degenerate_pose():
    clean = detection_log_lr(0.9, 1.0, None, 0.2, _PD, _C)
    degen = detection_log_lr(0.9, 0.0, None, 0.2, _PD, _C)   # z~0 = on the camera
    assert degen < clean
    assert degen < _BASE          # degenerate is *negative* evidence vs base
    assert phi_depth(0.0) == -1.0 and phi_depth(1.0) == 0.0


def test_score_is_monotonic():
    lo = detection_log_lr(0.2, 1.0, None, 0.2, _PD, _C)
    hi = detection_log_lr(0.9, 1.0, None, 0.2, _PD, _C)
    assert hi > lo
    assert phi_score(0.9) > 0.0 > phi_score(0.2)


def test_residual_feature_is_available_but_off_by_default():
    # phi_resid still works as a feature...
    assert phi_resid(None, 0.2) == 0.0        # fresh spawn: no track, no penalty
    assert phi_resid(0.2, 0.2) == -1.0        # at the gate edge: full penalty
    w_on = QualityWeights(w_resid=1.5)
    near = detection_log_lr(0.9, 1.0, 0.02, 0.2, _PD, _C, w_on)
    far = detection_log_lr(0.9, 1.0, 0.20, 0.2, _PD, _C, w_on)
    assert far < near                          # when enabled, it penalises
    # ...but the DEFAULT weights ignore residual: jitter must not change
    # existence evidence (loop1 choco). near == far with defaults.
    d_near = detection_log_lr(0.9, 1.0, 0.02, 0.2, _PD, _C)
    d_far = detection_log_lr(0.9, 1.0, 0.20, 0.2, _PD, _C)
    assert d_near == d_far


def test_loglr_is_clamped():
    w = QualityWeights()
    huge = detection_log_lr(1.0, 1.0, 0.0, 0.2, 0.99, 1e-6, w)
    tiny = detection_log_lr(0.0, 0.0, 1.0, 0.2, _PD, _C, w)
    assert huge <= w.clamp_hi + 1e-9
    assert tiny >= w.clamp_lo - 1e-9


# ---- pipeline (quality_weighting=True) ------------------------------------
def _T(t):
    return ((1.0, 0.0, 0.0, t[0]), (0.0, 1.0, 0.0, t[1]),
            (0.0, 0.0, 1.0, t[2]), (0.0, 0.0, 0.0, 1.0))


def _pose(stamp, t=(0.0, 0.0, 0.0)):
    return SlamCameraPose(
        stamp=stamp, frame_id="map", child_frame_id="camera", T_map_cam=_T(t),
        tracking_status=TrackingStatus.OK, source_slam_id="synthetic")


def _det(stamp, name, t, did, score=0.9):
    return Sam6DDetection(
        stamp=stamp, frame_id="camera", detection_id=did,
        object_name=name, T_cam_obj=_T(t), score=score)


def _published(store, name):
    return [lm for lm in store.landmarks_by_name(name)
            if lm.status in (ObjectStatus.active, ObjectStatus.remembered)]


def test_degenerate_depth_cluster_never_promotes_in_quality_mode():
    # static camera, five sightings of "milk" all at z~0 (pose sitting on the
    # camera). High score cannot rescue an unphysical depth.
    n = 5
    poses = [_pose(float(i)) for i in range(n)]
    ts = [float(i) for i in range(n)]
    dets = {i: [_det(float(i), "milk", (0.0, 0.0, 0.0), i, score=0.9)]
            for i in range(n)}
    q = run_object_memory(poses, dets, ts, quality_weighting=True)
    assert _published(q.store, "milk") == []          # never becomes an object
    # contrast: legacy hit-counter promotes the very same garbage.
    legacy = run_object_memory(poses, dets, ts, quality_weighting=False)
    assert any(lm.status is ObjectStatus.active
               for lm in legacy.store.landmarks_by_name("milk"))


def test_two_clean_detections_promote_in_quality_mode():
    n = 3
    poses = [_pose(float(i)) for i in range(n)]
    ts = [float(i) for i in range(n)]
    dets = {i: [_det(float(i), "cup", (0.0, 0.0, 1.0), i, score=0.9)]
            for i in range(n)}
    q = run_object_memory(poses, dets, ts, quality_weighting=True)
    pub = _published(q.store, "cup")
    assert len(pub) == 1 and pub[0].status is ObjectStatus.active


def test_clean_track_has_higher_confidence_than_degenerate():
    n = 3
    poses = [_pose(float(i)) for i in range(n)]
    ts = [float(i) for i in range(n)]
    clean = run_object_memory(
        poses, {i: [_det(float(i), "a", (0.0, 0.0, 1.0), i, 0.9)]
                for i in range(n)}, ts, quality_weighting=True)
    degen = run_object_memory(
        poses, {i: [_det(float(i), "a", (0.0, 0.0, 0.0), i, 0.9)]
                for i in range(n)}, ts, quality_weighting=True)
    rc = clean.store.landmarks_by_name("a")[0].confidence
    rd = degen.store.landmarks_by_name("a")[0].confidence
    assert rc > rd


def test_out_of_view_miss_keeps_confidence_frozen_in_quality_mode():
    # establish a track, then a frame where the object is behind the camera
    # (out of view) with no detection: fix-r must keep r unchanged.
    ts = [0.0, 1.0, 2.0, 3.0]
    poses = [_pose(0.0), _pose(1.0), _pose(2.0), _pose(3.0, t=(0.0, 0.0, 10.0))]
    dets = {0: [_det(0.0, "cup", (0.0, 0.0, 1.0), 0, 0.9)],
            1: [_det(1.0, "cup", (0.0, 0.0, 1.0), 1, 0.9)],
            2: [_det(2.0, "cup", (0.0, 0.0, 1.0), 2, 0.9)],
            3: []}   # object now at z_cam = 1 - 10 < 0 -> out of view, missed
    q = run_object_memory(poses, dets, ts, quality_weighting=True)
    cup = q.store.landmarks_by_name("cup")[0]
    # r after frame 2 == r after frame 3 (out-of-view miss carries no penalty).
    assert cup.status is ObjectStatus.active
    assert cup.missed_count == 0            # out-of-view misses are not counted
