"""Unit tests for Bernoulli existence updates and FOV-based P_D."""

from core.existence import R_MAX, update_existence_detected, update_existence_missed
from core.transforms import make_transform
from core.visibility import expected_p_d

I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def test_out_of_view_miss_keeps_existence():
    for r in (0.1, 0.5, 0.99):
        assert update_existence_missed(r, 0.0) == r


def test_in_view_miss_decreases_monotonically():
    r = 0.99
    prev = r
    for _ in range(10):
        r = update_existence_missed(r, 0.6)
        assert r < prev
        prev = r
    assert r < 0.05  # eventually crosses the retire floor


def test_detection_increases_and_caps():
    r = 0.7
    for _ in range(10):
        r2 = update_existence_detected(r, 0.6, 0.1)
        assert r2 > r or r2 == R_MAX
        r = r2
    assert r <= R_MAX
    # a miss from the cap still carries evidence (r=1 must not be absorbing).
    assert update_existence_missed(r, 0.6) < r


def test_expected_p_d_fov():
    cam = make_transform(I3, (0.0, 0.0, 0.0))
    ahead = make_transform(I3, (0.0, 0.0, 2.0))
    behind = make_transform(I3, (0.0, 0.0, -2.0))
    side = make_transform(I3, (5.0, 0.0, 1.0))
    # angular fallback (no intrinsics)
    assert expected_p_d(cam, ahead) > 0.0
    assert expected_p_d(cam, behind) == 0.0
    assert expected_p_d(cam, side) == 0.0
    # pinhole projection path
    K = ((400.0, 0.0, 320.0), (0.0, 400.0, 240.0), (0.0, 0.0, 1.0))
    assert expected_p_d(cam, ahead, K, (640, 480)) > 0.0
    assert expected_p_d(cam, side, K, (640, 480)) == 0.0
    assert expected_p_d(cam, behind, K, (640, 480)) == 0.0
