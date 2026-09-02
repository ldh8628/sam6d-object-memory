#!/usr/bin/env python3
"""Phase B -- live ROS2 object_memory node.

This is the real-time deployment of the Phase A engine. It reuses PoseBuffer +
StreamingObjectMemory VERBATIM; only the event source changes -- from file replay
(scripts_test/run_realtime_object_memory.py) to live ROS topics:

  - SLAM pose  (geometry_msgs/PoseStamped, ~30 Hz, low latency)  -> PoseBuffer
  - SAM-6D dets(vision_msgs/Detection3DArray, few Hz, LATE)      -> mem.step,
      fused against the buffered camera pose at the detection header stamp
      (time alignment). header.stamp is the CAPTURE time of the frame SAM ran on.

Publishes the persistent long-term map -- surviving (active + remembered)
landmarks -- as geometry_msgs/PoseArray in the map frame (Unity/RViz consumable).

Wiring notes:
  * A MultiThreadedExecutor + ReentrantCallbackGroup is REQUIRED so the fast pose
    stream keeps filling the buffer WHILE a slow detection message is being
    processed (mirrors the ORB-SLAM3 IMU-wrapper lesson).
  * The stock sam6d_multiobject_node publishes geometry_msgs/PoseArray, which
    drops object identity. To drive this node it must publish
    vision_msgs/Detection3DArray instead: each Detection3D.results[0].hypothesis
    carries class_id (object_name) + score, and results[0].pose.pose carries
    T_cam_obj. That is a localized change in the SAM node's publish block (it
    already has name+score+pose in hand); no custom interface package needed.

Run (inside the ROS env, e.g. conda sam6d_ros_humble, after `source`-ing ROS):
    python object_memory_node.py --ros-args \
        -p pose_topic:=/orbslam3/pose \
        -p detections_topic:=/sam6d/detections \
        -p caminfo_topic:=/camera/camera/color/camera_info
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src")))

import rclpy
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)

from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus
from core.pose_buffer import PoseBuffer
from core.state_machine import ObjectStatus
from core.transforms import make_transform_from_quat
from pipeline.object_memory_runner import StreamingObjectMemory

PUBLISHED = (ObjectStatus.active, ObjectStatus.remembered)


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _pose_to_T(p: Pose):
    """geometry_msgs/Pose -> 4x4 transform tuple."""
    t = (p.position.x, p.position.y, p.position.z)
    q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    return make_transform_from_quat(t, q)


def _T_to_pose(T) -> Pose:
    from core.transforms import rotation_to_quat
    rot = ((T[0][0], T[0][1], T[0][2]),
           (T[1][0], T[1][1], T[1][2]),
           (T[2][0], T[2][1], T[2][2]))
    qx, qy, qz, qw = rotation_to_quat(rot)
    p = Pose()
    p.position.x, p.position.y, p.position.z = T[0][3], T[1][3], T[2][3]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
        qx, qy, qz, qw
    return p


class ObjectMemoryNode(Node):
    def __init__(self):
        super().__init__("object_memory_node")
        self.declare_parameter("pose_topic", "/orbslam3/pose")
        self.declare_parameter("detections_topic", "/sam6d/detections")
        self.declare_parameter("caminfo_topic",
                               "/camera/camera/color/camera_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("assoc_trans_gate_m", 0.20)
        self.declare_parameter("assoc_rot_gate_deg", -1.0)
        self.declare_parameter("buffer_window_s", 10.0)
        self.declare_parameter("quality_weighting", True)
        self.declare_parameter("landmarks_topic", "/object_memory/landmarks")
        self.declare_parameter("output_path", "")

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame = gp("map_frame")
        self.buf = PoseBuffer(window_s=gp("buffer_window_s"))
        self.mem = StreamingObjectMemory(
            assoc_trans_gate_m=gp("assoc_trans_gate_m"),
            assoc_rot_gate_deg=gp("assoc_rot_gate_deg"),
            quality_weighting=gp("quality_weighting"),
        )
        self._frame_i = 0
        self._cam_K = None
        self._img_size = None
        self._output_path = str(gp("output_path")).strip()
        if self._output_path:
            os.makedirs(os.path.dirname(os.path.abspath(self._output_path)), exist_ok=True)

        # ReentrantCallbackGroup: pose + detection callbacks run concurrently, so
        # the buffer keeps filling while a slow detection message is processed.
        pose_grp = ReentrantCallbackGroup()
        detection_grp = MutuallyExclusiveCallbackGroup()
        self.create_subscription(PoseStamped, gp("pose_topic"),
                                 self._pose_cb, 50, callback_group=pose_grp)
        self.create_subscription(Detection3DArray, gp("detections_topic"),
                                 self._det_cb, 100, callback_group=detection_grp)
        self.create_subscription(CameraInfo, gp("caminfo_topic"),
                                 self._caminfo_cb, 5, callback_group=pose_grp)
        self.pub = self.create_publisher(PoseArray, "~/landmarks", 10)
        self.pub_named = self.create_publisher(
            Detection3DArray, gp("landmarks_topic"), 10)
        self.get_logger().info("object_memory_node up (Phase B, streaming engine)")

    # -- inputs --------------------------------------------------------------
    def _caminfo_cb(self, msg: CameraInfo):
        if self._cam_K is None:
            k = msg.k
            self._cam_K = ((k[0], k[1], k[2]), (k[3], k[4], k[5]),
                           (k[6], k[7], k[8]))
            self._img_size = (msg.width, msg.height)
            self.mem.cam_K = self._cam_K
            self.mem.img_size = self._img_size

    def _pose_cb(self, msg: PoseStamped):
        self.buf.add(SlamCameraPose(
            stamp=_stamp_to_sec(msg.header.stamp),
            frame_id=msg.header.frame_id or self.map_frame,
            child_frame_id="camera",
            T_map_cam=_pose_to_T(msg.pose),
            tracking_status=TrackingStatus.OK,
            source_slam_id="orbslam3",
        ))

    def _det_cb(self, msg: Detection3DArray):
        ts = _stamp_to_sec(msg.header.stamp)   # CAPTURE stamp of the SAM frame
        dets = []
        for i, d in enumerate(msg.detections):
            if not d.results:
                continue
            hyp = d.results[0]
            dets.append(Sam6DDetection(
                stamp=ts, frame_id="camera", detection_id=self._frame_i * 1000 + i,
                object_name=hyp.hypothesis.class_id,
                T_cam_obj=_pose_to_T(hyp.pose.pose),
                score=float(hyp.hypothesis.score),
            ))
        # time alignment: fuse against the camera pose at the CAPTURE stamp.
        pose = self.buf.lookup(ts)
        if pose is None:
            self.mem.step_no_pose(dets, ts, self._frame_i)   # INV-008: dropped
            self.get_logger().warn(
                f"frame {self._frame_i}: no pose in buffer for stamp {ts:.3f} "
                f"(dropped {len(dets)} dets)")
        else:
            self.mem.step(dets, pose, ts, self._frame_i)
        self._frame_i += 1
        self._publish()

    # -- output --------------------------------------------------------------
    def _publish(self):
        pa = PoseArray()
        pa.header.frame_id = self.map_frame
        pa.header.stamp = self.get_clock().now().to_msg()
        named = Detection3DArray()
        named.header = pa.header
        n_act = n_rem = 0
        output_objects = []
        for lm in self.mem.store.all_landmarks():
            if lm.status in PUBLISHED:
                pose = _T_to_pose(lm.T_map_obj)
                pa.poses.append(pose)
                detection = Detection3D()
                detection.header = pa.header
                detection.id = str(lm.object_id)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = lm.object_name
                hypothesis.hypothesis.score = float(lm.confidence)
                hypothesis.pose.pose = pose
                detection.results = [hypothesis]
                named.detections.append(detection)
                output_objects.append({
                    "object_id": lm.object_id, "object_name": lm.object_name,
                    "status": lm.status.value, "confidence": lm.confidence,
                    "observations": lm.observation_count,
                    "T_map_obj": [[float(value) for value in row]
                                  for row in lm.T_map_obj],
                })
                if lm.status is ObjectStatus.active:
                    n_act += 1
                else:
                    n_rem += 1
        self.pub.publish(pa)
        self.pub_named.publish(named)
        if self._output_path:
            temporary = f"{self._output_path}.{os.getpid()}.tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump({"frame": self._frame_i, "map_frame": self.map_frame,
                           "objects": output_objects}, stream, indent=2,
                          ensure_ascii=False)
            os.replace(temporary, self._output_path)
        self.get_logger().info(
            f"frame {self._frame_i}: map has active={n_act} remembered={n_rem}")


def main(argv=None):
    rclpy.init(args=argv)
    node = ObjectMemoryNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
