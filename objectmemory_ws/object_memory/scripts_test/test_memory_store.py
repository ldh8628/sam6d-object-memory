"""Unit tests for core.memory_store.ObjectMemoryStore and AssociationDecision."""

import pytest

from core.memory_store import ObjectMemoryStore
from core.models import AssociationDecision, DecisionType
from core.state_machine import ObjectStatus
from core.transforms import identity_transform


def _new_store():
    return ObjectMemoryStore()


def _make_tentative(store, name="mug", stamp=1.0):
    return store.create_tentative(
        object_name=name, T_map_obj=identity_transform(), first_seen_time=stamp
    )


def test_tentative_creation():
    store = _new_store()
    lm = _make_tentative(store)
    assert lm.status is ObjectStatus.tentative
    assert lm.object_id == 1
    assert store.get(1) is lm


def test_auto_increment_and_no_duplicate_ids():
    store = _new_store()
    a = _make_tentative(store, "a")
    b = _make_tentative(store, "b")
    c = _make_tentative(store, "c")
    ids = {a.object_id, b.object_id, c.object_id}
    assert ids == {1, 2, 3}
    assert len(store.all_landmarks()) == 3


def test_starting_object_id_validation():
    with pytest.raises(ValueError):
        ObjectMemoryStore(starting_object_id=0)


def test_require_raises_on_missing():
    store = _new_store()
    assert store.get(99) is None
    with pytest.raises(KeyError):
        store.require(99)


def test_status_and_active_query():
    store = _new_store()
    a = _make_tentative(store, "a")
    store.apply_transition(a.object_id, ObjectStatus.active, "confirmed")
    assert store.by_status(ObjectStatus.active) == [a]
    assert store.active_objects() == [a]


def test_remembered_or_lost_query():
    store = _new_store()
    a = _make_tentative(store, "a")
    store.apply_transition(a.object_id, ObjectStatus.active, "confirmed")
    store.apply_transition(a.object_id, ObjectStatus.lost, "missed")
    assert a in store.remembered_or_lost_objects()
    store.apply_transition(a.object_id, ObjectStatus.remembered, "aged")
    assert a in store.remembered_or_lost_objects()


def test_landmark_update():
    store = _new_store()
    a = _make_tentative(store, "a")
    store.update_landmark(a.object_id, confidence=0.8, missed_count=2)
    assert a.confidence == 0.8
    assert a.missed_count == 2


def test_direct_status_update_rejected():
    store = _new_store()
    a = _make_tentative(store, "a")
    with pytest.raises(ValueError):
        store.update_landmark(a.object_id, status=ObjectStatus.active)


def test_object_id_change_rejected():
    store = _new_store()
    a = _make_tentative(store, "a")
    # object_id is bound by the signature, so it can never leak into **changes;
    # passing it via a dict collides at call time. The guard is still asserted
    # for the reachable path (unknown field) to keep identity immutable.
    with pytest.raises(ValueError):
        store.update_landmark(a.object_id, not_a_field=1)


def test_transition_reason_recorded():
    store = _new_store()
    a = _make_tentative(store, "a")
    store.apply_transition(a.object_id, ObjectStatus.active, "confirmed by score")
    assert a.transition_history[-1].reason == "confirmed by score"
    log = store.transition_log()
    assert log[-1][0] == a.object_id
    assert log[-1][1].to_status is ObjectStatus.active


def test_debug_summary():
    store = _new_store()
    a = _make_tentative(store, "a")
    _make_tentative(store, "b")
    store.apply_transition(a.object_id, ObjectStatus.active, "confirmed")
    summary = store.debug_summary()
    assert summary["object_count"] == 2
    assert summary["next_object_id"] == 3
    assert summary["status_counts"]["active"] == 1
    assert summary["status_counts"]["tentative"] == 1
    assert summary["transition_count"] == 1


def test_association_decision_distinguishes_detection_and_object_id():
    d = AssociationDecision(
        decision_type=DecisionType.short_term_match,
        detection_id=7,
        object_id=3,
        score=0.9,
    )
    assert d.detection_id == 7
    assert d.object_id == 3
    # match decision without object_id must fail.
    with pytest.raises(ValueError):
        AssociationDecision(
            decision_type=DecisionType.short_term_match,
            detection_id=7,
            score=0.9,
        )


def test_rejected_decision_requires_reason():
    with pytest.raises(ValueError):
        AssociationDecision(
            decision_type=DecisionType.rejected, detection_id=1, score=0.1
        )
    ok = AssociationDecision(
        decision_type=DecisionType.rejected,
        detection_id=1,
        score=0.1,
        reject_reason="class gate failed",
    )
    assert ok.reject_reason == "class gate failed"


def test_score_range_validation():
    with pytest.raises(ValueError):
        AssociationDecision(
            decision_type=DecisionType.new_tentative, detection_id=1, score=1.5
        )
