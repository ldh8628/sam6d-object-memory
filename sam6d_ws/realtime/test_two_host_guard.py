"""Run in the sam6d ROS environment: python realtime/test_two_host_guard.py."""
import json
import struct
import sys
import tempfile
import threading
import time
from collections import Counter, deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from sam6d_receiver_node import Receiver
from two_host_guard import guard_healthy
from two_host_quality import measure_quality

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "objectmemory_ws/object_memory/ros"))
from object_memory_node import ObjectMemoryNode
from core.pose_buffer import PoseBuffer


def main():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "guard.json"

        def heartbeat(healthy=True, map_id="map/1", age=0):
            path.write_text(json.dumps({"healthy": healthy, "map_id": map_id,
                                       "updated_unix_s": time.time() - age}))

        assert guard_healthy("")
        assert not guard_healthy(path, "map/1")
        for value in ("{", "[]", "null", '{"healthy":true}'):
            path.write_text(value)
            assert not guard_healthy(path, "map/1")
        for age in (3, -3, float("nan")):
            heartbeat(age=age)
            assert not guard_healthy(path, "map/1")
        heartbeat()
        assert guard_healthy(path, "map/1") and not guard_healthy(path, "map/2")

        node = Receiver.__new__(Receiver)
        node._guard_file = str(path)
        node._two_host_map = node.map_id = "map/1"
        node._map_changed = node._guard_was_healthy = False
        node._guard_generation = 0
        node._accepted_frames = {}
        node._slam_lock = threading.Lock()
        node._anchor_lock = threading.Lock()
        node._map_anchors = {}
        node._slam_seed = node._slam_candidate = None
        node._slam_tracking_ok = False
        node._slam_wait_s = 0
        node._pose_match_ns = 50_000_000
        node._match_counts = Counter()
        node._inference_times_ms = deque()
        node._quality_counts = Counter()
        node._quality_overlaps = deque()
        node._quality_depths = deque()
        identity = np.eye(4)
        contexts = []
        node._write_frame = lambda frame, context: contexts.append(context)

        def match(poses, target=1_050_000_000):
            node._slam_poses = deque(poses)
            node._pending_frames = deque([{"stamp_ns": target, "queued_at": 0}])
            node._flush_pending()
            return contexts[-1]

        poses = [(1_000_000_000, identity, False), (1_100_000_000, identity, False)]
        assert match(poses)["pose_match_mode"] == "interpolated"
        assert match(poses, 1_000_000_000)["pose_match_mode"] == "exact"
        assert match(poses[:1])["tracking_state"] == "TRACKING_LOST"  # no nearest fallback
        node._on_slam_state(SimpleNamespace(data="not_tracking"))
        assert node._slam_poses  # asynchronous status is display-only

        # Faults revoke an inference already in progress, even if health recovers.
        node._accepted_frames[123] = ("map/1", 0)
        heartbeat(False)
        assert match(poses)["tracking_state"] == "TRACKING_LOST"
        assert node._guard_generation == 1 and not node._accepted_frames
        heartbeat()
        result = {"stamp_ns": 123, "frame_seq": 5, "map_id": "map/1",
                  "slam_paired": True, "dets": []}
        published = []
        node.pub_det = SimpleNamespace(publish=published.append)
        node._object_memory_enabled = True
        node._ack = bytearray(8)
        node._publish_result(result)
        assert not published and struct.unpack_from("<q", node._ack)[0] == 5
        node._accepted_frames[123] = ("map/1", 1)
        node._publish_result(result)
        assert len(published) == 1

        # Fusion receives the matched SAM timestamp, not the preceding ORB timestamp.
        camera_poses = []
        node.pub_camera_pose = SimpleNamespace(publish=camera_poses.append)
        node._map_frame_id = "calibrated_map"
        node._write_lock = threading.Lock()
        node.n_written = 0
        node._f_stat = None
        node.fw = SimpleNamespace(write=lambda *args, **kwargs: None)
        node._publish_overlay = lambda *args: None
        frame = {"stamp_ns": 1_050_000_000, "rgb": None, "depth": None, "K": None,
                 "recv_wall": time.time(), "depth_stamp_ns": 1_050_000_000, "source_seq": 0}
        context = {"tracking_state": "TRACKING_OK", "pose_match_mode": "interpolated",
                   "T_map_camera": identity, "map_id": "map/1"}
        Receiver._write_frame(node, frame, context)
        assert len(camera_poses) == 1
        assert camera_poses[0].header.stamp.sec == 1
        assert camera_poses[0].header.stamp.nanosec == 50_000_000
        assert camera_poses[0].header.frame_id == "calibrated_map"
        heartbeat(False)
        Receiver._write_frame(node, frame, context)
        assert len(camera_poses) == 1  # controller fault revokes pose publication too

        node._on_map_id(SimpleNamespace(data="map/2"))
        heartbeat(map_id="map/2")
        assert not node._two_host_healthy()  # never silently switch coordinate systems

        # The final fusion boundary also checks the heartbeat and pins map identity.
        memory = ObjectMemoryNode.__new__(ObjectMemoryNode)
        memory._guard_file = str(path)
        memory._map_id = None
        memory._map_changed = False
        memory._guard_was_healthy = False
        memory.buf = PoseBuffer(tol=0, interpolate=False)
        memory._lock = threading.RLock()
        fused = []
        memory._consume_detections = fused.append
        memory._map_cb(SimpleNamespace(data="map/2"))
        memory._det_cb("valid")
        heartbeat(False, "map/2")
        memory._det_cb("fault")
        heartbeat(map_id="map/3")
        memory._map_cb(SimpleNamespace(data="map/3"))
        memory._det_cb("changed map")
        assert fused == ["valid"]

        # Actual CAD projection vs the processed mask/depth: no model/GPU needed.
        pose = np.eye(4)
        pose[2, 3] = 1
        points = np.array([[x, y, 0] for x in (-.02, 0, .02) for y in (-.02, 0, .02)])
        snapshot = {"map_id": "map/1", "anchors": {"test": pose.tolist()}}
        context = {"map_id": "map/1", "tracking_state": "TRACKING_OK", "T_map_camera": np.eye(4)}
        depth = np.full((40, 40), 1000, np.uint16)
        K = np.array([[100, 0, 20], [0, 100, 20], [0, 0, 1]])
        def quality(mask, depth=depth):
            return measure_quality(snapshot, context, {"test": points},
                                   {"test": mask} if mask is not None else {}, depth, K)
        good = quality(np.ones((40, 40), bool))
        assert good["passed"] and good["samples"][0]["mask_overlap"] == 1
        assert good["samples"][0]["depth_residual_mm"] == 0
        assert not quality(np.zeros((40, 40), bool))["passed"]
        assert not quality(np.ones((40, 40), bool), depth + 101)["passed"]
        missing = quality(None)
        assert missing["passed"] is False and missing["samples"][0]["mask_overlap"] is None
        assert measure_quality(None, context, {}, {}, depth, K)["passed"] is None
        incomplete = {**snapshot, "anchors": {**snapshot["anchors"], "missing": pose.tolist()}}
        assert measure_quality(incomplete, context, {"test": points},
            {"test": np.ones((40, 40), bool)}, depth, K)["passed"] is None

        # Exercise the actual viewer callbacks without opening a window or needing a camera.
        from sam6d_viewer import view_overlay
        def viewer_spin(viewer, **kwargs):
            heartbeat(map_id="map/1")
            viewer._on_map(SimpleNamespace(data="map/1"))
            viewer.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *args: np.ones((32, 48, 3), np.uint8))
            viewer._on_image(None)
            viewer._on_tracking(SimpleNamespace(data="tracking"))
            assert viewer.frame is not None and viewer.tracking
            heartbeat(False, "map/1")
            viewer._on_tracking(SimpleNamespace(data="tracking"))
            viewer._on_image(None)
            assert viewer.frame is None and not viewer.tracking
            assert viewer.slam is None and viewer.sam is None and not viewer.objects
            assert viewer.frame_shape == (32, 48, 3)
            heartbeat(map_id="map/2")
            viewer._on_map(SimpleNamespace(data="map/2"))
            viewer._on_image(None)
            assert viewer.frame is None  # coordinate change stays latched
            raise KeyboardInterrupt
        with patch("rclpy.spin_once", side_effect=viewer_spin):
            view_overlay("/test/overlay", 1, 15, "", "/orbslam3/pose", "/sam6d/camera_pose",
                         "/orbslam3/tracking_state", "/object_memory/landmarks",
                         headless=True, guard_file=str(path))
    print("two-host receiver and fusion guard: PASS")


if __name__ == "__main__":
    main()
