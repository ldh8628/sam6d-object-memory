"""Unit tests for core.state_machine lifecycle transitions."""

import pytest

from core.state_machine import (
    ObjectStatus,
    build_transition,
    coerce_status,
    is_transition_allowed,
)


def test_tentative_to_active():
    t = build_transition(ObjectStatus.tentative, ObjectStatus.active, "confirmed")
    assert t.from_status is ObjectStatus.tentative
    assert t.to_status is ObjectStatus.active


def test_active_to_lost():
    assert is_transition_allowed(ObjectStatus.active, ObjectStatus.lost)
    build_transition(ObjectStatus.active, ObjectStatus.lost, "missed")


def test_lost_to_remembered():
    assert is_transition_allowed("lost", "remembered")
    build_transition("lost", "remembered", "aged out to memory")


def test_remembered_to_active():
    build_transition(ObjectStatus.remembered, ObjectStatus.active, "reacquired")


def test_tentative_to_deleted():
    build_transition(ObjectStatus.tentative, ObjectStatus.deleted, "spurious")


def test_any_to_deleted_allowed():
    assert is_transition_allowed(ObjectStatus.remembered, ObjectStatus.deleted)


def test_invalid_transition_rejected():
    assert not is_transition_allowed(ObjectStatus.tentative, ObjectStatus.lost)
    with pytest.raises(ValueError):
        build_transition(ObjectStatus.tentative, ObjectStatus.lost, "bad")


def test_empty_reason_rejected():
    with pytest.raises(ValueError):
        build_transition(ObjectStatus.tentative, ObjectStatus.active, "")
    with pytest.raises(ValueError):
        build_transition(ObjectStatus.tentative, ObjectStatus.active, "   ")


def test_coerce_status():
    assert coerce_status("active") is ObjectStatus.active
    with pytest.raises(ValueError):
        coerce_status("nonsense")
