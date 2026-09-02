"""Runner tests using in-memory synthetic poses (no bag/dataset files)."""

from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus
from core.state_machine import ObjectStatus
from core.transforms import compute_T_map_obj, identity_transform, make_transform
from pipeline.object_memory_runner import run_object_memory

I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _pose(stamp, tx):
    return SlamCameraPose(
        stamp=stamp,
        frame_id="map",
        child_frame_id="camera",
        T_map_cam=make_transform(I3, (tx, 0.0, 0.0)),
        tracking_status=TrackingStatus.OK,
        source_slam_id="synthetic",
    )


def _det(stamp, name, t):
    return Sam6DDetection(
        stamp=stamp,
        frame_id="camera",
        detection_id=int(stamp * 10),
        object_name=name,
        T_cam_obj=make_transform(I3, t),
        score=0.9,
    )


def test_persistent_object_id_and_promotion():
    poses = [_pose(0.0, 0.0), _pose(1.0, 0.0), _pose(2.0, 0.0)]
    ts = [0.0, 1.0, 2.0]
    dets = {
        0: [_det(0.0, "mug", (0.5, 1.0, 2.0))],
        1: [_det(1.0, "mug", (0.5, 1.0, 2.0))],
    }
    result = run_object_memory(poses, dets, ts)
    # exactly one persistent object, promoted to active after 2 hits.
    assert len(result.store.all_landmarks()) == 1
    lm = result.store.all_landmarks()[0]
    assert lm.object_name == "mug"
    assert lm.object_id == 1
    assert lm.observation_count == 2
    assert lm.status is ObjectStatus.active


def test_T_map_obj_composition():
    poses = [_pose(0.0, 10.0)]
    ts = [0.0]
    dets = {0: [_det(0.0, "box", (0.5, 1.0, 2.0))]}
    result = run_object_memory(poses, dets, ts)
    lm = result.store.require(1)
    expected = compute_T_map_obj(poses[0].T_map_cam, dets[0][0].T_cam_obj)
    assert abs(lm.T_map_obj[0][3] - expected[0][3]) < 1e-9
    assert abs(lm.T_map_obj[0][3] - 10.5) < 1e-9  # 10 + 0.5


def test_no_slam_pose_does_not_create_landmark():
    poses = [_pose(0.0, 0.0)]
    # detection timestamp far from the only pose -> INV-008 skip.
    ts = [100.0]
    dets = {0: [_det(100.0, "ghost", (1.0, 1.0, 1.0))]}
    result = run_object_memory(poses, dets, ts, time_tolerance=0.05)
    assert result.frames_without_slam == 1
    assert len(result.store.all_landmarks()) == 0


def test_first_instance_decays_when_detections_move_to_a_new_instance():
    # multi-instance: instance #1 forms at (0,0,1); then every later detection
    # lands far away at a CONSISTENT new location -> those spawn/feed instance #2
    # while instance #1, still in view but unfed, decays via in-view misses to
    # lost. The far detections are NOT fused into #1 (its pose stays put).
    poses = [_pose(float(i), 0.0) for i in range(9)]
    ts = [float(i) for i in range(9)]
    far = (5.0, 0.0, 1.0)  # >0.15 m from #1 -> matches no live instance of #1
    dets = {0: [_det(0.0, "can", (0.0, 0.0, 1.0))],
            1: [_det(1.0, "can", (0.0, 0.0, 1.0))]}   # #1 promoted to active
    for i in range(2, 8):
        dets[i] = [_det(float(i), "can", far)]        # 6 frames feeding #2
    result = run_object_memory(poses, dets, ts)
    cans = result.store.landmarks_by_name("can")
    first = cans[0]
    assert first.observation_count == 2       # far frames not fused into #1
    assert first.missed_count == 6            # in-view misses (its detn went to #2)
    assert first.status is ObjectStatus.lost  # decayed, not frozen
    assert first.confidence < 0.3
    # a distinct second instance formed at the far, consistent location.
    assert len(cans) >= 2
    second = cans[1]
    assert abs(second.T_map_obj[0][3] - 5.0) < 1e-6
    # frame-2 snapshot: #1 is simply unseen (a miss), not a "reject".
    fr2 = next(fr for fr in result.frame_results if fr.frame_idx == 2)
    snap1 = next(s for s in fr2.objects if s.object_id == first.object_id)
    assert not snap1.visible
    assert not snap1.rejected_this_frame


def test_accept_after_rejections_resets_missed_count():
    poses = [_pose(float(i), 0.0) for i in range(4)]
    ts = [float(i) for i in range(4)]
    far = (5.0, 0.0, 1.0)
    dets = {
        0: [_det(0.0, "can", (0.0, 0.0, 1.0))],
        1: [_det(1.0, "can", far)],               # rejected -> miss 1
        2: [_det(2.0, "can", (0.0, 0.0, 1.0))],   # accepted -> reset
    }
    result = run_object_memory(poses, dets, ts)
    lm = result.store.require(result.object_ids_by_name["can"])
    assert lm.missed_count == 0
    assert lm.observation_count == 2


def test_mixed_accept_and_reject_same_frame_counts_as_seen():
    poses = [_pose(0.0, 0.0), _pose(1.0, 0.0)]
    ts = [0.0, 1.0]
    dets = {
        0: [_det(0.0, "can", (0.0, 0.0, 1.0))],
        1: [_det(1.0, "can", (0.0, 0.0, 1.0)),    # accepted
            _det(1.0, "can", (5.0, 0.0, 1.0))],   # rejected, same name
    }
    result = run_object_memory(poses, dets, ts)
    lm = result.store.require(result.object_ids_by_name["can"])
    assert lm.missed_count == 0
    fr1 = next(fr for fr in result.frame_results if fr.frame_idx == 1)
    snap = next(s for s in fr1.objects if s.object_name == "can")
    assert snap.visible
    assert not snap.rejected_this_frame


def _frames(names_by_frame):
    """Build poses/ts/dets for consecutive integer-stamped frames."""
    n = len(names_by_frame)
    poses = [_pose(float(i), 0.0) for i in range(n)]
    ts = [float(i) for i in range(n)]
    dets = {}
    for i, specs in enumerate(names_by_frame):
        if specs:
            dets[i] = [_det(float(i), nm, t) for nm, t in specs]
    return poses, ts, dets


def test_tentative_ghost_pruned_by_existence_floor():
    # one-shot FP whose claimed position stays IN VIEW: existence decays each
    # missed frame until the retire floor -> deleted + name freed.
    frames = [[("ghost", (0.0, 0.0, 1.0)), ("real", (1.0, 0.0, 1.0))]]
    frames += [[("real", (1.0, 0.0, 1.0))]] * 10
    poses, ts, dets = _frames(frames)
    result = run_object_memory(poses, dets, ts)
    assert "ghost" not in result.object_ids_by_name          # name released
    ghost = next(lm for lm in result.store.all_landmarks()
                 if lm.object_name == "ghost")
    assert ghost.status is ObjectStatus.deleted              # tombstone remains
    reasons = [tr.reason for oid, tr in result.store.transition_log()
               if oid == ghost.object_id]
    assert reasons == ["existence_floor"]


def test_out_of_view_ghost_pruned_by_age_cap():
    # ghost whose claimed position never re-enters the view: r stays frozen,
    # so the tentative age cap is the safety net.
    frames = [[("ghost", (0.0, 0.0, -5.0)), ("real", (1.0, 0.0, 1.0))]]
    frames += [[("real", (1.0, 0.0, 1.0))]] * 12
    poses, ts, dets = _frames(frames)
    result = run_object_memory(poses, dets, ts, tentative_max_age=10)
    ghost = next(lm for lm in result.store.all_landmarks()
                 if lm.object_name == "ghost")
    assert ghost.status is ObjectStatus.deleted
    reasons = [tr.reason for oid, tr in result.store.transition_log()
               if oid == ghost.object_id]
    assert reasons == ["tentative_timeout"]


def test_out_of_view_absence_preserves_identity():
    # THE fix-r scenario: a well-observed object, then the camera drives away
    # (object far outside the view) for 30 frames. P_D = 0 -> r frozen -> the
    # landmark must survive with the SAME object_id (fix3 deleted it).
    frames = [[("can", (0.0, 0.0, 1.0))]] * 6
    poses = [_pose(float(i), 0.0) for i in range(6)]
    for i in range(6, 36):   # camera far away; only "other" is visible there
        frames.append([("other", (0.0, 0.0, 1.0))])
        poses.append(_pose(float(i), 50.0))
    frames.append([("can", (0.0, 0.0, 1.0))])       # camera returns
    poses.append(_pose(36.0, 0.0))
    ts = [float(i) for i in range(len(frames))]
    dets = {i: [_det(ts[i], nm, t) for nm, t in specs]
            for i, specs in enumerate(frames) if specs}
    result = run_object_memory(poses, dets, ts)
    cans = [lm for lm in result.store.all_landmarks()
            if lm.object_name == "can"]
    assert len(cans) == 1                            # no delete + re-seed churn
    lm = cans[0]
    assert lm.object_id == result.object_ids_by_name["can"]
    assert lm.status is ObjectStatus.active
    assert lm.observation_count == 7
    assert lm.missed_count == 0                      # zero IN-VIEW misses
    reasons = [tr.reason for oid, tr in result.store.transition_log()
               if oid == lm.object_id]
    assert "missed_threshold" not in reasons         # never even went lost


def test_reseed_after_prune_gets_new_object_id():
    # poisoned first pose (in view) -> later detections rejected -> existence
    # floor prunes it -> the next detection re-seeds at the correct position.
    bad, good = (0.3, 0.0, 2.0), (0.0, 0.0, 1.0)
    frames = [[("can", bad)]] + [[("can", good)]] * 10   # all rejected (gate)
    frames += [[("can", good)]] * 2                      # after prune: re-seed
    poses, ts, dets = _frames(frames)
    result = run_object_memory(poses, dets, ts)
    cans = sorted((lm for lm in result.store.all_landmarks()
                   if lm.object_name == "can"), key=lambda x: x.object_id)
    assert len(cans) == 2
    assert cans[0].status is ObjectStatus.deleted            # poisoned tombstone
    assert cans[1].status is ObjectStatus.active             # re-seeded, promoted
    assert cans[1].object_id != cans[0].object_id            # INV-002: no reuse
    assert abs(cans[1].T_map_obj[2][3] - 1.0) < 1e-6         # corrected position
    assert result.object_ids_by_name["can"] == cans[1].object_id


def test_long_observed_lost_becomes_remembered_not_deleted():
    # 6 accepted obs (>= MIN_OBS_LONGTERM), then 30 misses -> remembered.
    frames = [[("can", (0.0, 0.0, 1.0)), ("other", (3.0, 0.0, 1.0))]] * 6
    frames += [[("other", (3.0, 0.0, 1.0))]] * 30
    poses, ts, dets = _frames(frames)
    result = run_object_memory(poses, dets, ts)
    lm = result.store.require(result.object_ids_by_name["can"])
    assert lm.status is ObjectStatus.remembered
    assert lm.missed_count < 30           # retired well before 30 in-view misses
    reasons = [tr.reason for oid, tr in result.store.transition_log()
               if oid == lm.object_id]
    assert reasons[-1] == "long_term_memory"


def test_remembered_redetected_reactivates_same_id():
    frames = [[("can", (0.0, 0.0, 1.0)), ("other", (3.0, 0.0, 1.0))]] * 6
    frames += [[("other", (3.0, 0.0, 1.0))]] * 30
    frames += [[("can", (0.0, 0.0, 1.0))]]                  # reappears
    poses, ts, dets = _frames(frames)
    result = run_object_memory(poses, dets, ts)
    lm = result.store.require(result.object_ids_by_name["can"])
    assert lm.object_id == 1                                 # same identity
    assert lm.status is ObjectStatus.active
    reasons = [tr.reason for oid, tr in result.store.transition_log()
               if oid == lm.object_id]
    assert reasons[-2:] == ["long_term_memory", "redetected"]


def test_miss_to_lost_and_redetect():
    # hit, hit (active), then 6 in-view misses -> lost, then redetect -> active.
    poses = [_pose(float(i), 0.0) for i in range(10)]
    ts = [float(i) for i in range(10)]
    dets = {
        0: [_det(0.0, "can", (0.0, 0.0, 1.0))],
        1: [_det(1.0, "can", (0.0, 0.0, 1.0))],
    }
    # frames 2..7 present but without "can" -> in-view misses
    for i in range(2, 8):
        dets[i] = [_det(float(i), "other", (0.0, 0.0, 2.0))]
    dets[9] = [_det(9.0, "can", (0.0, 0.0, 1.0))]
    result = run_object_memory(poses, dets, ts)
    can_id = result.object_ids_by_name["can"]
    lm = result.store.require(can_id)
    assert lm.status is ObjectStatus.active  # redetected at frame 6
    reasons = [tr.reason for _, tr in result.store.transition_log() if _ == can_id]
    assert "missed_threshold" in reasons
    assert "redetected" in reasons
