import uuid
import subprocess
import sys

import numpy as np

from realtime.shm_channel import FrameReader, FrameWriter


def test_frame_channel_preserves_timestamped_slam_pose():
    name = f"sam6d_test_{uuid.uuid4().hex}"
    writer = FrameWriter(name=name, size=4096)
    reader = FrameReader(name=name)
    try:
        pose = np.eye(4)
        pose[:3, 3] = [1.0, 2.0, 3.0]
        writer.write(np.zeros((2, 3, 3), np.uint8), np.ones((2, 3), np.uint16),
                     np.eye(3), 1_000_000_000, 1.5,
                     {"T_map_camera": pose, "pose_stamp_ns": 999_000_000,
                      "tracking_state": "TRACKING_OK", "map_id": "map_한글_A"},
                     source_seq=42)
        assert reader.read_new() is not None
        assert reader.last_source_seq == 42
        context = reader.last_slam_context
        assert context["tracking_state"] == "TRACKING_OK"
        assert context["rgb_stamp_ns"] == 1_000_000_000
        assert context["pose_stamp_ns"] == 999_000_000
        assert context["map_id"] == "map_한글_A"
        assert np.allclose(context["T_map_camera"], pose)
    finally:
        reader.close()
        writer.close()


def test_reader_process_exit_does_not_unlink_writer_segment():
    name = f"sam6d_test_{uuid.uuid4().hex}"
    writer = FrameWriter(name=name, size=4096)
    try:
        child = subprocess.run(
            [sys.executable, "-c",
             ("from realtime.shm_channel import FrameReader; "
              f"r=FrameReader({name!r}); r.close()")],
            text=True, capture_output=True, check=True)
        assert "resource_tracker" not in child.stderr
        reader = FrameReader(name=name)
        reader.close()
    finally:
        writer.close()
