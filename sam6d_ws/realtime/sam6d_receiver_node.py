#!/usr/bin/env python3
"""Receive synchronized RGB-D into a bounded FIFO; consume in one worker.

Conversion, pose waiting, shared-memory writes and overlay projection run outside
image callbacks. ISM/PEM independently choose the latest shared-memory frame.
"""
from __future__ import annotations

import argparse
import json
import mmap
import os
import struct
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rclpy                                                     # noqa: E402
from cv_bridge import CvBridge                                   # noqa: E402
from geometry_msgs.msg import Pose, PoseStamped                  # noqa: E402
from message_filters import ApproximateTimeSynchronizer, Subscriber  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup         # noqa: E402
from rclpy.clock import Clock                                    # noqa: E402
from rclpy.clock_type import ClockType                            # noqa: E402
from rclpy.executors import MultiThreadedExecutor                # noqa: E402
from rclpy.node import Node                                      # noqa: E402
from rclpy.parameter import Parameter                            # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)                         # noqa: E402
try:                                                               # noqa: E402
    from realsense2_camera_msgs.msg import RGBD
except ImportError:                                                # bag/legacy installs need no RealSense msgs
    RGBD = None
from scipy.spatial.transform import Rotation                     # noqa: E402
from sensor_msgs.msg import CameraInfo, Image                    # noqa: E402
from std_srvs.srv import Trigger
from std_msgs.msg import String                                  # noqa: E402
from vision_msgs.msg import (Detection3D, Detection3DArray,      # noqa: E402
                             ObjectHypothesisWithPose)
from shm_channel import FrameWriter                              # noqa: E402
from slam_pose_memory import (canonical_object_points, canonical_object_rotation,
                              interpolate_se3, project_pose_axes, project_pose_box,
                              repair_isolated_pose, tracking_state_ok, valid_se3)  # noqa: E402


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "integration"))
from input_integrity import SequentialInput, stamp_ns
from two_host_guard import guard_healthy


BOX_EDGES = ((0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
             (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7))


class Receiver(Node):
    def __init__(self, cfg):
        super().__init__("sam6d_receiver",
                         parameter_overrides=[Parameter(
                             "use_sim_time", Parameter.Type.BOOL,
                             bool(cfg["runtime"].get("use_sim_time", True)))])
        self.bridge = CvBridge()
        self._anchor_lock = threading.Lock()
        self._map_anchors = {}
        self._object_boxes = {}
        for path in (Path(__file__).resolve().parents[1] / "assets" /
                     "model_points").glob("*.npy"):
            points = np.load(path, mmap_mode="r")
            points = canonical_object_points(path.stem, points)
            self._object_boxes[path.stem] = np.asarray(
                [points.min(0), points.max(0)], dtype=float) / 1000.0
        self.K = None
        self.camera_info = None
        self.n_in = self.n_written = 0
        self._last_input_monotonic = None
        self._slam_poses = deque(maxlen=100)
        self._slam_seed = None
        self._slam_candidate = None
        self._slam_repaired = 0
        self._slam_tracking_ok = False
        self._slam_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._pending_frames = deque()
        # 수신 쪽 실측: 촬영(stamp)에서 여기까지 걸린 시간과, 콜백 자체가 먹는 시간.
        # 추론이 빨라지면 병목이 이쪽으로 옮겨오므로 반드시 같이 재야 한다.
        self._stats = []
        odir = cfg.get("output", {}).get("dir", "output/rt_split")
        odir = odir if os.path.isabs(odir) else os.path.join(
            str(Path(__file__).resolve().parents[1]), odir)
        os.makedirs(odir, exist_ok=True)
        self._f_stat = open(os.path.join(odir, "receiver.jsonl"), "w", encoding="utf-8")
        self._result_fifo = os.path.join(odir, ".sam6d_results.fifo")
        try:
            os.unlink(self._result_fifo)
        except FileNotFoundError:
            pass
        os.mkfifo(self._result_fifo)
        self._result_fd = os.open(self._result_fifo, os.O_RDONLY | os.O_NONBLOCK)
        self._result_buffer = b""
        self._ack_path = os.path.join(odir, ".sam6d_result_ack")
        self._ack_fd = os.open(self._ack_path, os.O_CREAT | os.O_RDWR, 0o600)
        os.ftruncate(self._ack_fd, 8)
        self._ack = mmap.mmap(self._ack_fd, 8)
        struct.pack_into("<q", self._ack, 0, -1)
        self.fw = FrameWriter()
        t = cfg["topics"]
        # ⚠ 영상 토픽에 BEST_EFFORT 를 쓰면 안 된다. 900 KB 짜리 Image 는 UDP 로 잘게
        # 쪼개져 가는데, best-effort 독자는 조각 하나만 유실돼도 그 샘플을 통째로 버린다.
        # 실측(temp/rx_probe.py): 같은 30 Hz 입력에서 RELIABLE 30.0 Hz vs BEST_EFFORT 3.7 Hz,
        # 큐를 30 으로 키워도 그대로였다. '최신 것만 처리'는 QoS 가 아니라 공유메모리
        # 덮어쓰기로 하는 것이 옳다 — 버릴 것을 우리가 고르지, 전송이 무작위로 버리게 두지 않는다.
        # 카메라가 best_effort 로만 발행하면 그때만 이 값을 바꾼다(안 맞으면 데이터가 0 이다).
        # 검증 신뢰도 발행 여부(split 모드에서는 ROS 발행이 이쪽에서 일어난다)
        self.pub_verify = bool((cfg["runtime"].get("verify") or {}).get("publish", False))
        rel = str(cfg["runtime"].get("qos_reliability", "reliable")).lower()
        qd = int(cfg["runtime"].get("qos_depth", 120))
        self.qos_reliability, self.qos_depth = rel, qd
        qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT if rel.startswith("best")
                         else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST, depth=qd)
        self.get_logger().info(f"[recv] QoS {rel} depth={qd}")
        par = ReentrantCallbackGroup()
        self._intake_group = MutuallyExclusiveCallbackGroup()
        self._image_subscriptions = []
        self.pub_det = self.create_publisher(Detection3DArray, "/sam6d/detections", 100)
        self.pub_status = self.create_publisher(String, "/sam6d/status", 10)
        overlay_topic = str(cfg.get("output", {}).get(
            "overlay_topic", "/sam6d/overlay")).strip()
        overlay_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_overlay = (self.create_publisher(Image, overlay_topic, overlay_qos)
                            if overlay_topic else None)
        if t.get("rgbd"):
            if RGBD is None:
                raise RuntimeError(
                    "topics.rgbd requires realsense2_camera_msgs (install the Jazzy package)")
            self._image_subscriptions.append(self.create_subscription(
                RGBD, t["rgbd"], self._on_rgbd, qos, callback_group=self._intake_group))
            self.get_logger().info(f"[recv] combined RGBD {t['rgbd']}")
        else:
            self.create_subscription(CameraInfo, t["caminfo"], self._on_caminfo, qos,
                                     callback_group=par)
        slam_cfg = cfg.get("slam", {})
        self.map_id = str(slam_cfg.get("map_id", "default"))
        self._guard_file = str(slam_cfg.get("two_host_guard_file", ""))
        self._metrics_path = str(slam_cfg.get("two_host_metrics_path", ""))
        self._two_host_map = None
        self._map_changed = False
        self._guard_was_healthy = False
        self._guard_generation = 0
        self._accepted_frames = {}
        self._match_counts = Counter()
        # ponytail: retain the latest 10k delays; stream a histogram for longer distributions.
        self._pose_delays_ms = deque(maxlen=10_000)
        self._inference_times_ms = deque(maxlen=10_000)
        self._quality_counts = Counter()
        self._quality_overlaps = deque(maxlen=10_000)
        self._quality_depths = deque(maxlen=10_000)
        self._quality_last = None
        self._landmarks_path = Path(odir) / ".two_host_landmarks.json"
        self._T_slam_sam = np.asarray(
            slam_cfg.get("T_slam_camera_sam_camera", np.eye(4)), dtype=np.float64)
        if not valid_se3(self._T_slam_sam):
            raise ValueError("slam.T_slam_camera_sam_camera must be a valid 4x4 SE(3)")
        self._object_memory_enabled = bool(
            (cfg.get("object_memory") or {}).get("enabled", False))
        self._slam_wait_s = max(
            0.0, float(slam_cfg.get("timestamp_tolerance_ms", 100.0)) / 1000.0)
        # The wrapper stamps T_map_camera with the RGB timestamp, so matching is
        # intentionally tight. The 100 ms setting above is how long we wait for it.
        self._pose_match_ns = int(
            max(0.0, float(slam_cfg.get("pose_match_tolerance_ms", 1.0))) * 1e6)
        self.create_subscription(
            PoseStamped, str(slam_cfg.get("pose_topic", "/orbslam3/pose")),
            self._on_slam_pose, 30, callback_group=par)
        self.create_subscription(
            String, str(slam_cfg.get("tracking_topic", "/orbslam3/tracking_state")),
            self._on_slam_state, 30, callback_group=par)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            String, str(slam_cfg.get("map_id_topic", "/orbslam3/map_id")),
            self._on_map_id, map_qos, callback_group=par)
        self.pub_camera_pose = self.create_publisher(
            PoseStamped, str(slam_cfg.get("sam_pose_topic", "/sam6d/camera_pose")), 30)
        if self._object_memory_enabled:
            memory_topic = str((cfg.get("object_memory") or {}).get(
                "landmarks_topic", "/object_memory/landmarks"))
            self.create_subscription(
                Detection3DArray, memory_topic, self._on_memory_landmarks, 10,
                callback_group=par)
            self.get_logger().info(f"[recv] ObjectMemory landmarks {memory_topic}")
        self.sync = None
        if not t.get("rgbd"):
            self._legacy_subscribers = [Subscriber(self, Image, t[key], qos_profile=qos,
                callback_group=self._intake_group) for key in ("rgb", "depth")]
            self._image_subscriptions.extend(sub.sub for sub in self._legacy_subscribers)
            self.sync = ApproximateTimeSynchronizer(
                self._legacy_subscribers,
                queue_size=int(cfg["runtime"].get("sync_queue", 2)),
                slop=float(cfg["runtime"].get("sync_slop", 0.02)))
            self.sync.registerCallback(self._on_pair)
        # bag의 /clock이 멈춘 뒤 끝난 마지막 추론도 회수해야 한다.
        wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.002, self._pump, callback_group=par,
                          clock=wall_clock)                         # 결과 회수·발행
        self.create_timer(float(cfg["runtime"].get("status_period_s", 1.0)),
                          self._status, callback_group=par, clock=wall_clock)
        self.create_timer(10.0, self._warn_if_silent, callback_group=par,
                          clock=wall_clock)
        camera = (cfg.get("input", {}).get("sam_camera") or {})
        self._input = SequentialInput(
            Path(odir) / "sam_input_health.jsonl", self._consume_input, self._describe_input,
            stage="sam", camera=camera.get("name", "sam_camera"),
            serial=camera.get("serial", ""), capacity=int(cfg["runtime"].get("input_queue_size", 60)))
        self._native_info_pub = (self.create_publisher(CameraInfo, t["caminfo"], qos)
            if cfg["runtime"].get("publish_native_camera_info") and t.get("rgbd") else None)
        self.create_service(Trigger, "/sam6d/drain_input", self._drain_input,
                            callback_group=self._intake_group)
        self.get_logger().info("[recv] bounded sequential input worker READY")

    def _on_caminfo(self, msg):
        first = self.K is None
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        roi = msg.roi
        self.camera_info = {
            "frame_id": msg.header.frame_id, "width": int(msg.width),
            "height": int(msg.height), "distortion_model": msg.distortion_model,
            "d": list(msg.d), "k": list(msg.k), "r": list(msg.r), "p": list(msg.p),
            "binning_x": int(msg.binning_x), "binning_y": int(msg.binning_y),
            "roi": {"x_offset": int(roi.x_offset), "y_offset": int(roi.y_offset),
                    "height": int(roi.height), "width": int(roi.width),
                    "do_rectify": bool(roi.do_rectify)},
        }
        if first:
            self.get_logger().info(f"[recv] caminfo fx={self.K[0, 0]:.1f}")

    def _on_rgbd(self, msg):
        """RealSense's already synchronized/aligned RGBD message: no Python ATS."""
        self._input.put(msg)

    @staticmethod
    def _describe_input(payload, identity_only=False):
        rgb, depth = ((payload.rgb, payload.depth) if hasattr(payload, "rgb") else payload[:2])
        return {"kind": "rgbd", "stamp_ns": stamp_ns(rgb), "depth_stamp_ns": stamp_ns(depth)}

    def _consume_input(self, payload, row):
        self._intake_row = row
        if hasattr(payload, "rgb"):
            self._on_caminfo(payload.rgb_camera_info)
            if self._native_info_pub is not None:
                self._native_info_pub.publish(payload.rgb_camera_info)
            rgb, depth = payload.rgb, payload.depth
        else:
            rgb, depth = payload
        if self.K is None:
            raise RuntimeError("CameraInfo missing; frame cannot be consumed")
        self._on_images(rgb, depth)
        while self._pending_frames:
            self._flush_pending()
            if self._pending_frames:
                time.sleep(0.001)

    def _drain_input(self, request, response):
        # Same mutually exclusive group as intake: no executor can take a competing sample.
        while True:
            taken = False
            for sub in self._image_subscriptions:
                with sub.handle:
                    item = sub.handle.take_message(sub.msg_type, sub.raw)
                if item is not None:
                    sub.callback(item[0])
                    taken = True
            if not taken:
                break
        for sub in self._image_subscriptions:
            self.destroy_subscription(sub)
        self._image_subscriptions.clear()
        if self.sync is not None:
            # Legacy message_filters subscriptions are stopped by the intake gate.
            self.sync = None
        response.success = self._input.close()
        response.message = json.dumps({"received": self._input.received,
                                      "consumed": self._input.consumed,
                                      "failures": self._input.failures})
        return response

    def _on_slam_state(self, msg):
        with self._slam_lock:
            self._slam_tracking_ok = tracking_state_ok(msg.data)
            if self._guard_file:
                self._match_counts["orb_tracking" if self._slam_tracking_ok else "orb_not_tracking"] += 1
            if not self._guard_file and not self._slam_tracking_ok:
                self._slam_poses.clear()
                self._slam_seed = None
                self._slam_candidate = None

    def _on_map_id(self, msg):
        value = str(msg.data).strip()
        if value:
            with self._slam_lock:
                if self._guard_file:
                    if value == "unknown":
                        self._map_changed = self._two_host_map is not None
                    elif self._two_host_map is None:
                        self._two_host_map = value
                    elif value != self._two_host_map:
                        self._map_changed = True
                if value != self.map_id:
                    # Poses from different Atlas coordinates must never cross the
                    # map boundary. The next TRACKING_OK/pose pair re-enables anchors.
                    self._slam_poses.clear()
                    self._slam_seed = None
                    self._slam_candidate = None
                    self._slam_tracking_ok = False
                    with self._anchor_lock:
                        self._map_anchors.clear()
                self.map_id = value

    def _two_host_healthy(self):
        """Caller holds _slam_lock. A fault invalidates already-running inference."""
        healthy = (not self._map_changed and self._two_host_map is not None
                   and guard_healthy(self._guard_file, self._two_host_map))
        if not healthy and self._guard_was_healthy:
            self._guard_generation += 1
            self._accepted_frames.clear()
            self._slam_poses.clear()
            self._slam_seed = self._slam_candidate = None
        self._guard_was_healthy = healthy
        return healthy

    def _on_slam_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        matrix = np.eye(4, dtype=np.float64)
        try:
            matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        except ValueError:
            return
        matrix[:3, 3] = [p.x, p.y, p.z]
        matrix = matrix @ self._T_slam_sam
        if not valid_se3(matrix):
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        accepted = None
        with self._slam_lock:
            self._map_frame_id = msg.header.frame_id
            if self._guard_file:
                self._pose_delays_ms.append((time.time_ns() - stamp) / 1e6)
                if not self._two_host_healthy():
                    return
            if self._slam_seed is None:
                self._slam_seed = (stamp, matrix)
            elif self._slam_candidate is None:
                self._slam_candidate = (stamp, matrix, msg.header)
            else:
                middle_stamp, middle, middle_header = self._slam_candidate
                left_stamp, left = self._slam_seed
                filtered, repaired = repair_isolated_pose(
                    left_stamp, left, middle_stamp, middle, stamp, matrix)
                self._slam_poses.append((middle_stamp, filtered, repaired))
                self._slam_seed = (middle_stamp, filtered)
                self._slam_candidate = (stamp, matrix, msg.header)
                self._slam_repaired += int(repaired)
                accepted = middle_header, filtered
        if accepted is None or self._guard_file:
            return
        self._publish_camera_pose(accepted[0], accepted[1])

    def _publish_camera_pose(self, header, matrix):
        transformed = PoseStamped()
        transformed.header = header
        transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z = (
            float(v) for v in matrix[:3, 3])
        quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        (transformed.pose.orientation.x, transformed.pose.orientation.y,
         transformed.pose.orientation.z, transformed.pose.orientation.w) = map(float, quat)
        self.pub_camera_pose.publish(transformed)

    def _on_memory_landmarks(self, msg):
        anchors = {}
        for detection in msg.detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0]
            pose = hypothesis.pose.pose
            matrix = np.eye(4, dtype=np.float64)
            try:
                matrix[:3, :3] = Rotation.from_quat([
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w]).as_matrix()
            except ValueError:
                continue
            matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            if valid_se3(matrix):
                anchors[str(hypothesis.hypothesis.class_id)] = matrix
        with self._anchor_lock:
            self._map_anchors = anchors
        if self._guard_file:
            with self._slam_lock:
                if not self._two_host_healthy():
                    return
                snapshot = {"map_id": self.map_id, "updated_unix_s": time.time(),
                            "anchors": {name: pose.tolist() for name, pose in anchors.items()}}
            with self._write_lock:
                temporary = self._landmarks_path.with_suffix(".tmp")
                temporary.write_text(json.dumps(snapshot), encoding="utf-8")
                os.replace(temporary, self._landmarks_path)

    def _flush_pending(self):
        """Publish a frame only after its same-stamp SLAM pose arrives or times out.

        Looking up the latest pose immediately in the RGB callback systematically pairs
        frame N with frame N-1. The short time-bounded queue preserves input order while
        preventing that stale anchor transform; inference still reads only the latest
        shared-memory frame.
        """
        pending = context = None
        with self._slam_lock:
            if not self._pending_frames:
                return
            frame = self._pending_frames[0]
            chosen_pose = None
            pose_allowed = (self._two_host_healthy() if self._guard_file
                            else self._slam_tracking_ok)
            if pose_allowed and self._slam_poses:
                target = frame["stamp_ns"]
                exact = next((row for row in self._slam_poses if row[0] == target), None)
                left = max((row for row in self._slam_poses if row[0] < target),
                           default=None, key=lambda row: row[0])
                right = min((row for row in self._slam_poses if row[0] > target),
                            default=None, key=lambda row: row[0])
                if exact is not None:
                    chosen_pose = (target, exact[1].copy(), "exact", bool(exact[2]))
                elif left is not None and right is not None:
                    matrix = interpolate_se3(
                        target, left[0], left[1], right[0], right[1], max_span_s=0.2)
                    if matrix is not None:
                        chosen_pose = (
                            target, matrix, "interpolated", bool(left[2] or right[2]))
            timed_out = time.monotonic() - frame["queued_at"] >= self._slam_wait_s
            if (not self._guard_file and chosen_pose is None and self._slam_tracking_ok
                    and timed_out and self._slam_poses):
                pose_stamp, matrix, repaired = min(
                    self._slam_poses, key=lambda row: abs(row[0] - frame["stamp_ns"]))
                if abs(pose_stamp - frame["stamp_ns"]) <= self._pose_match_ns:
                    chosen_pose = (
                        pose_stamp, matrix.copy(), "nearest_timeout",
                        bool(repaired))
            if chosen_pose is not None:
                pending = self._pending_frames.popleft()
                context = {
                    "T_map_camera": chosen_pose[1],
                    "pose_stamp_ns": chosen_pose[0],
                    "pose_match_mode": chosen_pose[2],
                    "pose_delta_ns": chosen_pose[0] - frame["stamp_ns"],
                    "repaired_pose_contributed": chosen_pose[3],
                    "tracking_state": "TRACKING_OK",
                    "map_id": self.map_id,
                }
            elif (not pose_allowed or
                  timed_out):
                pending = self._pending_frames.popleft()
                # Preserve the current map identity, but explicitly prevent Anchor use.
                context = {"tracking_state": "TRACKING_LOST", "map_id": self.map_id}
        if pending is not None:
            self._write_frame(pending, context)

    def _write_frame(self, frame, slam_context):
        if self._guard_file:
            with self._slam_lock:
                if not self._two_host_healthy():
                    slam_context = {"tracking_state": "TRACKING_LOST", "map_id": self.map_id}
                mode = slam_context.get("pose_match_mode", "missed")
                self._match_counts[mode] += 1
                if self._slam_tracking_ok:
                    self._match_counts["tracking_" + mode] += 1
                if mode != "missed":
                    self._accepted_frames[frame["stamp_ns"]] = (self.map_id, self._guard_generation)
                    # Retain only the input window needed by delayed inference.
                    while len(self._accepted_frames) > 1000:
                        self._accepted_frames.pop(next(iter(self._accepted_frames)))
                    header = PoseStamped().header
                    header.frame_id = getattr(self, "_map_frame_id", "map")
                    header.stamp.sec, header.stamp.nanosec = divmod(frame["stamp_ns"], 1_000_000_000)
                    self._publish_camera_pose(header, slam_context["T_map_camera"])
        with self._write_lock:
            self.fw.write(frame["rgb"], frame["depth"], frame["K"],
                          frame["stamp_ns"], frame["recv_wall"], slam_context,
                          depth_stamp_ns=frame["depth_stamp_ns"],
                          source_seq=frame["source_seq"])
            self.n_written += 1
            if self._f_stat is not None:
                self._f_stat.write(json.dumps({
                    "source_seq": frame["source_seq"],
                    "stamp_ns": frame["stamp_ns"],
                    "depth_stamp_ns": frame["depth_stamp_ns"],
                    "rgb_frame_id": frame["rgb_frame_id"],
                    "depth_frame_id": frame["depth_frame_id"],
                    "deliver_ms": frame["deliver_ms"],
                    "cb_ms": frame["cb_ms"],
                    "slam_paired": bool(slam_context.get("T_map_camera") is not None),
                    "pose_stamp_ns": slam_context.get("pose_stamp_ns"),
                    "pose_match_mode": slam_context.get("pose_match_mode"),
                    "pose_delta_ns": slam_context.get("pose_delta_ns"),
                    "repaired_pose_contributed": bool(
                        slam_context.get("repaired_pose_contributed")),
                    "slam_pose_repaired_total": self._slam_repaired,
                    **({"camera_info": self.camera_info} if self.n_written == 1 else {}),
                }) + "\n")
                self._f_stat.flush()
        self._publish_overlay(frame, slam_context)

    def _publish_overlay(self, frame, slam_context):
        """Project the latest trusted map anchors on every incoming RGB frame."""
        if self.pub_overlay is None or self.pub_overlay.get_subscription_count() == 0:
            return
        overlay = frame["rgb"].copy()
        twc = np.asarray(slam_context.get("T_map_camera"), float)
        with self._slam_lock:
            allowed = not self._guard_file or self._two_host_healthy()
        if allowed and slam_context.get("tracking_state") == "TRACKING_OK" and valid_se3(twc):
            with self._anchor_lock:
                anchors = [(name, pose.copy()) for name, pose in self._map_anchors.items()]
            camera_to_map = np.linalg.inv(twc)
            for name, map_to_object in anchors:
                camera_to_object = camera_to_map @ map_to_object
                box = project_pose_box(
                    camera_to_object, frame["K"], self._object_boxes.get(name))
                if box is not None and np.max(np.abs(box)) <= 100_000:
                    box = np.rint(box).astype(int)
                    for start, end in BOX_EDGES:
                        cv2.line(overlay, tuple(box[start]), tuple(box[end]),
                                 (190, 130, 255), 1, cv2.LINE_AA)
                pixels = project_pose_axes(camera_to_object, frame["K"])
                if pixels is None or np.max(np.abs(pixels)) > 100_000:
                    continue
                points = np.rint(pixels).astype(int)
                origin = tuple(int(v) for v in points[0])
                for end, color in zip(points[1:], ((255, 60, 70), (70, 230, 120),
                                                    (80, 160, 255))):
                    cv2.line(overlay, origin, tuple(int(v) for v in end), color, 2,
                             cv2.LINE_AA)
                cv2.circle(overlay, origin, 4, (190, 130, 255), 1, cv2.LINE_AA)
                cv2.putText(overlay, f"MAP {name}", (origin[0] + 7, origin[1] - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (225, 205, 255), 1,
                            cv2.LINE_AA)
        msg = self.bridge.cv2_to_imgmsg(overlay, encoding="rgb8")
        msg.header.stamp.sec = frame["stamp_ns"] // 1_000_000_000
        msg.header.stamp.nanosec = frame["stamp_ns"] % 1_000_000_000
        msg.header.frame_id = frame["rgb_frame_id"]
        self.pub_overlay.publish(msg)

    def _on_pair(self, rgb_msg, depth_msg):
        self._input.put((rgb_msg, depth_msg))

    def _on_images(self, rgb_msg, depth_msg):
        self.n_in += 1
        self._last_input_monotonic = time.monotonic()
        if self.K is None:
            return
        t_cb = time.perf_counter()
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        stamp = rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec
        depth_stamp = (depth_msg.header.stamp.sec * 1_000_000_000
                       + depth_msg.header.stamp.nanosec)
        frame = {
            "rgb": np.ascontiguousarray(rgb),
            "depth": np.ascontiguousarray(depth),
            "K": self.K.copy(),
            "stamp_ns": stamp,
            "depth_stamp_ns": depth_stamp,
            "source_seq": self._intake_row["sequence"],
            "rgb_frame_id": rgb_msg.header.frame_id,
            "depth_frame_id": depth_msg.header.frame_id,
            "recv_wall": time.time(),
            "queued_at": time.monotonic(),
            "deliver_ms": None,  # Source/host clock domains are not subtracted.
            "cb_ms": self._intake_row["callback_ms"],
        }
        with self._slam_lock:
            self._pending_frames.append(frame)
        self._flush_pending()

    def _pump(self):
        try:
            chunk = os.read(self._result_fd, 1 << 20)
        except BlockingIOError:
            return
        if chunk:
            self._result_buffer += chunk
        while b"\n" in self._result_buffer:
            line, self._result_buffer = self._result_buffer.split(b"\n", 1)
            if line:
                self._publish_result(json.loads(line))

    def _publish_result(self, r):
        if self._guard_file:
            total_ms = (r.get("ms") or {}).get("total")
            if isinstance(total_ms, (int, float)) and np.isfinite(total_ms):
                self._inference_times_ms.append(total_ms)
            with self._slam_lock:
                healthy = self._two_host_healthy()
                accepted = self._accepted_frames.pop(r["stamp_ns"], None)
                allowed = (healthy and r.get("slam_paired") is True
                           and r.get("map_id") == self.map_id
                           and accepted == (self.map_id, self._guard_generation))
            if not allowed:
                self._match_counts["results_suppressed"] += 1
                struct.pack_into("<q", self._ack, 0, int(r["frame_seq"]))
                self._last = r
                return
            quality = r.get("object_memory_quality") or {}
            self._quality_last = quality
            samples = quality.get("samples", [])
            if samples:
                self._quality_counts["visible_processed_frames"] += 1
                self._quality_counts["passing_frames"] += int(quality.get("passed") is True)
            else:
                self._quality_counts["unmeasured_frames"] += 1
            self._quality_counts["object_samples"] += len(samples)
            for sample in samples:
                for key, values in (("mask_overlap", self._quality_overlaps),
                                    ("depth_residual_mm", self._quality_depths)):
                    value = sample.get(key)
                    if isinstance(value, (int, float)) and np.isfinite(value):
                        values.append(value)
        if not self._object_memory_enabled and str(r.get("map_id", "")) == self.map_id:
            anchors = {}
            for name, value in (r.get("map_anchors") or {}).items():
                pose = np.asarray(value, float)
                if valid_se3(pose):
                    pose = pose.copy()
                    pose[:3, :3] = canonical_object_rotation(name, pose[:3, :3])
                    anchors[str(name)] = pose
            with self._anchor_lock:
                self._map_anchors = anchors
        arr = Detection3DArray()
        arr.header.stamp.sec = r["stamp_ns"] // 1_000_000_000
        arr.header.stamp.nanosec = r["stamp_ns"] % 1_000_000_000
        for d in r["dets"]:
            det = Detection3D()
            det.header = arr.header
            det.id = d["object"]
            p = Pose()
            p.position.x, p.position.y, p.position.z = [float(v) / 1000.0 for v in d["t_mm"]]
            q = Rotation.from_matrix(canonical_object_rotation(
                d["object"], d["R"])).as_quat()
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d["object"]
            hyp.hypothesis.score = float(d["score"])
            hyp.pose.pose = p
            det.results = [hyp]
            # results[0] 은 그대로 두고 뒤에 덧붙인다 — 하류는 results[0] 만 읽는다.
            if self.pub_verify and d.get("verify"):
                v = ObjectHypothesisWithPose()
                v.hypothesis.class_id = "_verify"
                v.hypothesis.score = float(d["verify"].get("conf", 0.0))
                v.pose.pose = p
                det.results.append(v)
            det.bbox.center = p
            arr.detections.append(det)
        self.pub_det.publish(arr)
        struct.pack_into("<q", self._ack, 0, int(r["frame_seq"]))
        self._last = r

    def _warn_if_silent(self):
        """QoS 가 안 맞으면 데이터가 아예 안 온다 — 조용히 굶지 말고 알려 준다.
        (아직 재생/카메라를 켜지 않은 정상 상황도 있으므로 한 번만 찍는다.)"""
        if self.n_in == 0 and not getattr(self, "_warned", False):
            self._warned = True
            self.get_logger().warn(
                "[recv] 아직 영상이 한 장도 안 왔다. 재생/카메라를 아직 안 켰다면 정상이다. "
                "켰는데도 계속 0 이면 발행 측 QoS 불일치다 "
                "— runtime.qos_reliability 를 best_effort 로 바꿔 볼 것")

    def _status(self):
        last = getattr(self, "_last", {})
        age = (None if self._last_input_monotonic is None else
               round(time.monotonic() - self._last_input_monotonic, 3))
        self.pub_status.publish(String(data=json.dumps({
            "ready": True, "frames_in": self._input.received, "frames_written": self.n_written,
            "input_failures": self._input.failures,
            "inference_latest_skipped": max(0, int(last.get("source_seq", -1)) + 1 - int(last.get("n_proc", 0))),
            "qos": {"reliability": self.qos_reliability, "depth": self.qos_depth},
            "slam_pose_repaired": self._slam_repaired,
            "silent": self.n_in == 0 or (age is not None and age > 2.0),
            "last_input_age_s": age,
            "processed": last.get("n_proc", 0), "detections": last.get("n_det", 0),
            "last_ms": last.get("ms", {})}, ensure_ascii=False)))
        if self._guard_file:
            with self._slam_lock:
                healthy = self._two_host_healthy()
                delays = list(self._pose_delays_ms)
            if self._metrics_path:
                visible = self._quality_counts["visible_processed_frames"]
                fraction = self._quality_counts["passing_frames"] / visible if visible else None
                summary = {"healthy": healthy, "map_id": self.map_id,
                    "updated_unix_s": time.time(), "pose_matching": dict(self._match_counts),
                    "pose_capture_to_arrival_ms": {str(q): float(np.percentile(delays, q))
                                                   for q in (50, 95)} if delays else None,
                    "pose_lan_delay_ms": None,
                    "pose_lan_delay_note": "PoseStamped has capture time, not remote send time",
                    "sam_received": self._input.received, "sam_written": self.n_written,
                    "sam_processed": last.get("n_proc", 0),
                    "sam_skipped": max(0, int(last.get("source_seq", -1)) + 1 - int(last.get("n_proc", 0))),
                    "sam_inference_ms": {str(q): float(np.percentile(list(self._inference_times_ms), q))
                                         for q in (50, 95)} if self._inference_times_ms else None,
                    "sam_last_inference_ms": last.get("ms", {}),
                    "object_memory_quality": {**dict(self._quality_counts),
                        "passing_visible_frame_fraction": fraction,
                        "gate_passed": fraction >= .95 if fraction is not None else None,
                        "thresholds": {"mask_overlap_min": .5, "depth_residual_max_mm": 100,
                                       "visible_fraction_min": .2, "passing_frame_fraction_min": .95},
                        "mask_overlap": {str(q): float(np.percentile(list(self._quality_overlaps), q))
                                         for q in (50, 95)} if self._quality_overlaps else None,
                        "depth_residual_mm": {str(q): float(np.percentile(list(self._quality_depths), q))
                                              for q in (50, 95)} if self._quality_depths else None,
                        "samples_file": str(self._landmarks_path.parent / "frames.jsonl"),
                        "last_frame": self._quality_last}}
                path = Path(self._metrics_path)
                with self._write_lock:
                    temporary = path.with_suffix(".tmp")
                    temporary.write_text(json.dumps(summary), encoding="utf-8")
                    os.replace(temporary, path)

    def close(self):
        self._input.close()
        if self._guard_file and self._metrics_path:
            self._status()
        self.get_logger().info(f"[recv] 수신 {self.n_in} · 공유메모리 기록 {self.n_written}")
        self.fw.close()
        self._f_stat.close()
        os.close(self._result_fd)
        self._ack.close()
        os.close(self._ack_fd)
        try:
            os.unlink(self._result_fifo)
            os.unlink(self._ack_path)
        except FileNotFoundError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    a, ros_args = ap.parse_known_args()
    cfg = yaml.safe_load(open(a.config, encoding="utf-8")) or {}
    cfg.setdefault("runtime", {})
    rclpy.init(args=ros_args)
    node = Receiver(cfg)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
