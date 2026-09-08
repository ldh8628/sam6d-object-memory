"""Phase A -- streaming engine tests (PoseBuffer + real-time driver).

Two layers:
 - PoseBuffer units: exact lookup, interpolation, out-of-window, nearest.
 - driver: streaming with time alignment ON and a realistic latency reproduces
   the batch runner; turning time alignment OFF shifts a landmark by (camera
   speed * latency) -- the smearing the design predicts.
"""
import math

from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus
from core.pose_buffer import PoseBuffer, interpolate_pose
from core.state_machine import ObjectStatus
from pipeline.object_memory_runner import run_object_memory
from run_realtime_object_memory import run_streaming


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


def _survivors(res):
    return sorted((lm.object_name,
                   round(lm.T_map_obj[0][3], 5),
                   round(lm.T_map_obj[1][3], 5),
                   round(lm.T_map_obj[2][3], 5), lm.status.value)
                  for lm in res.store.all_landmarks()
                  if lm.status in (ObjectStatus.active, ObjectStatus.remembered))


# ---- PoseBuffer units ------------------------------------------------------
def test_buffer_exact_lookup():
    b = PoseBuffer()
    b.add(_pose(1.0, (1.0, 0.0, 0.0)))
    b.add(_pose(2.0, (2.0, 0.0, 0.0)))
    p = b.lookup(2.0)
    assert p is not None and p.T_map_cam[0][3] == 2.0


def test_buffer_interpolates_translation():
    b = PoseBuffer(interpolate=True, max_gap_s=2.0)
    b.add(_pose(1.0, (0.0, 0.0, 0.0)))
    b.add(_pose(2.0, (10.0, 0.0, 0.0)))
    p = b.lookup(1.25)                       # 25% of the way
    assert abs(p.T_map_cam[0][3] - 2.5) < 1e-9


def test_buffer_out_of_window_returns_none():
    b = PoseBuffer(window_s=1.0)
    b.add(_pose(1.0))
    b.add(_pose(3.0))                        # trims anything older than 2.0
    assert b.lookup(1.0) is None            # 1.0 fell out of the window

def test_buffer_nearest_when_no_bracket():
    b = PoseBuffer(interpolate=True, tol=0.05)
    b.add(_pose(1.0, (1.0, 0.0, 0.0)))
    # only a left sample, within tol -> returned; beyond tol -> None
    assert b.lookup(1.03) is not None
    assert b.lookup(5.0) is None


def test_interpolate_pose_midpoint():
    a, c = _pose(0.0, (0.0, 0.0, 0.0)), _pose(1.0, (4.0, 2.0, 0.0))
    mid = interpolate_pose(a, c, 0.5)
    assert abs(mid.T_map_cam[0][3] - 2.0) < 1e-9
    assert abs(mid.T_map_cam[1][3] - 1.0) < 1e-9


# ---- streaming driver ------------------------------------------------------
def _scene(v, dt=0.05, n=61, keyframes=(10, 20, 30, 40)):
    """Camera slides along +x at speed v with realistic 20 Hz poses; one fixed
    object in the map, seen on a few sparse keyframes (like slow SAM output)."""
    poses = [_pose(i * dt, (v * i * dt, 0.0, 0.0)) for i in range(n)]
    ts = [i * dt for i in range(n)]
    X, Z = 5.0, 1.0
    dets = {k: [_det(ts[k], "obj", (X - v * ts[k], 0.0, Z), k, 0.9)]
            for k in keyframes}
    return poses, dets, ts, X, Z


def test_streaming_matches_batch_with_alignment():
    poses, dets, ts, _, _ = _scene(v=0.3)
    batch = run_object_memory(poses, dets, ts, quality_weighting=True)
    stream = run_streaming(poses, dets, ts, latency=0.15, time_align=True,
                           interpolate=False, quality_weighting=True)
    assert _survivors(batch) == _survivors(stream)


def test_time_alignment_recovers_true_position():
    # object truly at x=5. With alignment, the fused landmark sits at x=5.
    poses, dets, ts, X, Z = _scene(v=0.3)
    stream = run_streaming(poses, dets, ts, latency=0.5, time_align=True,
                           interpolate=True, quality_weighting=True)
    surv = [lm for lm in stream.store.all_landmarks()
            if lm.status in (ObjectStatus.active, ObjectStatus.remembered)]
    assert len(surv) == 1
    assert abs(surv[0].T_map_obj[0][3] - X) < 1e-6


def test_no_alignment_smears_by_speed_times_latency():
    # fusing at ARRIVAL pose instead of CAPTURE pose shifts the landmark by
    # ~ v*L downstream of the true position.
    v, L = 0.3, 0.5
    poses, dets, ts, X, Z = _scene(v=v)
    off = run_streaming(poses, dets, ts, latency=L, time_align=False,
                        interpolate=True, quality_weighting=True)
    surv = [lm for lm in off.store.all_landmarks()
            if lm.status in (ObjectStatus.active, ObjectStatus.remembered)]
    assert len(surv) == 1
    err = surv[0].T_map_obj[0][3] - X
    assert err > 0.1                       # clearly displaced, not at the truth
    assert abs(err - v * L) < 0.05         # and displaced by ~ v*L
