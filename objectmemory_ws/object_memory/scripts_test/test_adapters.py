"""Adapter tests.

Pure-logic tests run always. Real-data smoke tests run only when the known
SAM_occlusion inputs are present on this machine; otherwise they skip (no fake
files are created).
"""

import os

import pytest

from adapters.sam6d_pem import load_pem_detections
from adapters.slam_trajectory import load_slam_trajectory
from adapters.bag_frame_index import load_frame_timestamps

_HERE = os.path.dirname(__file__)
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))

SLAM_TRAJ = os.path.join(
    _REPO, "slam_comparison/output/SAM_occlusion/orbslam3_noimu/CameraTrajectory.txt"
)
# canonical SAM-6D pose output (flat layout: frame_<idx>_<Object>.json)
PEM_DIR = os.path.join(
    _REPO, "sam6d_ws/outputs/rgbd_imu_sdk_bag/SAM_occlusion/output/pem"
)
BAG = os.path.join(_REPO, "data_slam/rgbd_imu_sdk_bag/SAM_occlusion")

_HAVE_REAL = all(os.path.exists(p) for p in (SLAM_TRAJ, PEM_DIR, BAG))
_real = pytest.mark.skipif(not _HAVE_REAL, reason="SAM_occlusion inputs not present")


# ----- pure-logic tests ----------------------------------------------------
def test_slam_trajectory_parse(tmp_path):
    p = tmp_path / "traj.txt"
    p.write_text(
        "# comment\n"
        "0.0 1.0 2.0 3.0 0.0 0.0 0.0 1.0\n"
        "1.0 4.0 5.0 6.0 0.0 0.0 0.0 1.0\n"
    )
    poses = load_slam_trajectory(str(p), source_slam_id="unit")
    assert len(poses) == 2
    assert poses[0].stamp == 0.0
    assert poses[0].source_slam_id == "unit"
    # identity quaternion -> translation preserved.
    assert poses[0].T_map_cam[0][3] == 1.0
    assert poses[1].T_map_cam[2][3] == 6.0


def test_slam_trajectory_bad_columns(tmp_path):
    p = tmp_path / "bad.txt"
    p.write_text("0.0 1.0 2.0\n")
    with pytest.raises(ValueError):
        load_slam_trajectory(str(p))


def test_pem_missing_dir():
    with pytest.raises(FileNotFoundError):
        load_pem_detections(os.path.join(_HERE, "__no_such_pem__"))


# ----- real-data smoke tests ----------------------------------------------
@_real
def test_real_slam_trajectory():
    poses = load_slam_trajectory(SLAM_TRAJ)
    assert len(poses) > 100
    assert poses == sorted(poses, key=lambda p: p.stamp)


@_real
def test_real_frame_timestamps_align_with_traj():
    ts = load_frame_timestamps(BAG)
    poses = load_slam_trajectory(SLAM_TRAJ)
    assert len(ts) > 0
    # first bag color timestamp should match the first SLAM pose stamp closely.
    assert abs(ts[0] - poses[0].stamp) < 0.01


@_real
def test_real_pem_detections_have_pose_and_scale():
    ts = load_frame_timestamps(BAG)
    dets = load_pem_detections(PEM_DIR, timestamps=ts)
    assert len(dets) > 0
    any_det = next(iter(dets.values()))[0]
    assert any_det.object_name
    # translation converted mm -> m: within a few metres of the camera.
    z = any_det.T_cam_obj[2][3]
    assert 0.0 < abs(z) < 20.0
