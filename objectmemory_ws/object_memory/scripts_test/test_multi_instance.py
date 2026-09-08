"""Multi-instance association tests (Story 4/5 minimal form).

Real SAM datasets are single-instance-per-class, so genuine coexistence is
validated here with synthetic detections: two instances of one class held
distinct, one-to-one per-frame assignment, revisit keeping identity, and
near-duplicate re-observations NOT fragmenting into many instances.
"""

from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus
from core.state_machine import ObjectStatus
from pipeline.object_memory_runner import run_object_memory

I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _T(t):
    return ((1.0, 0.0, 0.0, t[0]),
            (0.0, 1.0, 0.0, t[1]),
            (0.0, 0.0, 1.0, t[2]),
            (0.0, 0.0, 0.0, 1.0))


def _pose(stamp, tx=0.0):
    return SlamCameraPose(
        stamp=stamp, frame_id="map", child_frame_id="camera",
        T_map_cam=_T((tx, 0.0, 0.0)),
        tracking_status=TrackingStatus.OK, source_slam_id="synthetic",
    )


def _det(stamp, name, t, did):
    return Sam6DDetection(
        stamp=stamp, frame_id="camera", detection_id=did,
        object_name=name, T_cam_obj=_T(t), score=0.9,
    )


def test_two_coexisting_instances_of_same_class_kept_distinct():
    # two "milk" at map locations 2 m apart (>> gate), both visible every frame.
    n = 5
    poses = [_pose(float(i)) for i in range(n)]
    ts = [float(i) for i in range(n)]
    dets = {i: [_det(float(i), "milk", (0.0, 0.0, 1.0), i * 10),
                _det(float(i), "milk", (0.0, 0.0, 3.0), i * 10 + 1)]
            for i in range(n)}
    r = run_object_memory(poses, dets, ts)
    milk = r.store.landmarks_by_name("milk")
    assert len(milk) == 2                                  # two distinct ids
    assert {round(m.T_map_obj[2][3]) for m in milk} == {1, 3}
    assert all(m.status is ObjectStatus.active for m in milk)
    assert all(m.observation_count == n for m in milk)     # never cross-fed
    assert sorted(r.live_ids_by_name["milk"]) == [m.object_id for m in milk]


def test_two_detections_same_frame_assigned_one_to_one():
    # seed two instances, then a frame with two detections (listed in the OTHER
    # order) must bind each to its nearest instance, not double-claim one.
    poses = [_pose(float(i)) for i in range(3)]
    ts = [float(i) for i in range(3)]
    dets = {
        0: [_det(0.0, "cup", (0.0, 0.0, 1.0), 1),
            _det(0.0, "cup", (0.0, 0.0, 3.0), 2)],
        1: [_det(1.0, "cup", (0.0, 0.0, 1.0), 3),
            _det(1.0, "cup", (0.0, 0.0, 3.0), 4)],
        # frame 2: same two objects, detections given in reversed order + jitter
        2: [_det(2.0, "cup", (0.0, 0.0, 3.02), 5),
            _det(2.0, "cup", (0.0, 0.0, 0.98), 6)],
    }
    r = run_object_memory(poses, dets, ts)
    cup = r.store.landmarks_by_name("cup")
    assert len(cup) == 2                                   # no third spawn
    assert all(c.observation_count == 3 for c in cup)      # each fed once/frame
    assert {round(c.T_map_obj[2][3]) for c in cup} == {1, 3}


def test_revisit_reassociates_same_instance_not_new():
    # instance formed, camera drives away (object out of view), then returns to
    # the SAME spot -> must re-acquire the same object_id, not spawn a new one.
    frames = [("milk", (0.0, 0.0, 1.0), 0.0)] * 4         # form + promote
    # leave: camera 50 m away, a different object visible there
    frames += [("other", (0.0, 0.0, 1.0), 50.0)] * 8
    frames += [("milk", (0.0, 0.0, 1.0), 0.0)]            # return
    poses, ts, dets = [], [], {}
    for i, (nm, t, tx) in enumerate(frames):
        poses.append(_pose(float(i), tx))
        ts.append(float(i))
        dets[i] = [_det(float(i), nm, t, i)]
    r = run_object_memory(poses, dets, ts)
    milk = r.store.landmarks_by_name("milk")
    assert len(milk) == 1                                 # no duplicate spawn
    assert milk[0].object_id == 1                         # same identity
    assert milk[0].status is ObjectStatus.active
    assert milk[0].observation_count == 5                 # 4 forming + 1 return
    # out-of-view absence never killed it -> no delete/re-seed churn.
    assert milk[0].missed_count == 0


def test_jittered_reobservations_stay_single_instance():
    # a single object seen with small (<gate) pose noise must remain ONE
    # instance, not fragment (the core fragmentation-reduction property).
    import math
    n = 20
    poses = [_pose(float(i)) for i in range(n)]
    ts = [float(i) for i in range(n)]
    dets = {}
    for i in range(n):
        # deterministic jitter well under the 0.15 m gate (no RNG in scripts).
        jx = 0.05 * math.sin(i)
        jz = 0.05 * math.cos(i)
        dets[i] = [_det(float(i), "milk", (jx, 0.0, 1.0 + jz), i)]
    r = run_object_memory(poses, dets, ts)
    milk = r.store.landmarks_by_name("milk")
    assert len(milk) == 1                                 # no fragmentation
    assert milk[0].status is ObjectStatus.active
    assert milk[0].observation_count == n


def test_rotation_gate_disabled_allows_negative_value():
    # assoc_rot_gate_deg < 0 (or None) disables the rotation gate; a large
    # rotation difference within the translation gate still associates.
    n = 3
    poses = [_pose(float(i)) for i in range(n)]
    ts = [float(i) for i in range(n)]
    # 90-deg yaw about Z on the observed pose; same translation.
    R90 = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    T_rot = (tuple(R90[0]) + (0.0,), tuple(R90[1]) + (0.0,),
             tuple(R90[2]) + (1.0,), (0.0, 0.0, 0.0, 1.0))
    dets = {}
    for i in range(n):
        d = _det(float(i), "milk", (0.0, 0.0, 1.0), i)
        if i > 0:
            d = Sam6DDetection(stamp=float(i), frame_id="camera",
                               detection_id=i, object_name="milk",
                               T_cam_obj=T_rot, score=0.9)
        dets[i] = [d]
    r = run_object_memory(poses, dets, ts, assoc_rot_gate_deg=-1.0)
    milk = r.store.landmarks_by_name("milk")
    assert len(milk) == 1                                 # rotation not gated
    assert milk[0].observation_count == n
