#!/usr/bin/env python3
"""sam6d_receiver_node.py — 수신 프로세스. **여기서는 추론을 하지 않는다.**

하는 일은 셋뿐이다.
  1) RGB + aligned depth + camera_info 를 구독해 짝을 맞춘다
  2) 가장 최신 한 쌍을 공유메모리에 덮어쓴다 (밀리면 그냥 버린다)
  3) 추론 프로세스가 돌려준 결과를 `/sam6d/detections` 로 발행한다

무거운 일이 없으므로 ROS 콜백들이 GIL 을 오래 붙잡지 않는다. 추론은 ROS 가 전혀 없는
별도 프로세스(`sam6d_infer.py`)에서 돈다 — 이것이 초 단위 정체를 없애기 위한 분리다
(근거: NOTES 19~21절).

    conda activate sam6d
    python realtime/sam6d_receiver_node.py --config <run yaml>
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rclpy                                                     # noqa: E402
from cv_bridge import CvBridge                                   # noqa: E402
from geometry_msgs.msg import Pose, PoseStamped                  # noqa: E402
from message_filters import ApproximateTimeSynchronizer, Subscriber  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup         # noqa: E402
from rclpy.executors import MultiThreadedExecutor                # noqa: E402
from rclpy.node import Node                                      # noqa: E402
from rclpy.parameter import Parameter                            # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from scipy.spatial.transform import Rotation                     # noqa: E402
from sensor_msgs.msg import CameraInfo, Image                    # noqa: E402
from std_msgs.msg import String                                  # noqa: E402
from vision_msgs.msg import (Detection3D, Detection3DArray,      # noqa: E402
                             ObjectHypothesisWithPose)
from shm_channel import FrameWriter, JsonReader, PoseRingWriter  # noqa: E402


class Receiver(Node):
    def __init__(self, cfg):
        super().__init__("sam6d_receiver",
                         parameter_overrides=[Parameter(
                             "use_sim_time", Parameter.Type.BOOL,
                             bool(cfg["runtime"].get("use_sim_time", True)))])
        self.bridge = CvBridge()
        self.K = None
        self.n_in = self.n_written = 0
        # 수신 쪽 실측: 촬영(stamp)에서 여기까지 걸린 시간과, 콜백 자체가 먹는 시간.
        # 추론이 빨라지면 병목이 이쪽으로 옮겨오므로 반드시 같이 재야 한다.
        self._stats = []
        self._f_stat = None
        odir = cfg.get("output", {}).get("dir")
        if odir:
            import os as _o
            odir = odir if _o.path.isabs(odir) else _o.path.join(
                str(Path(__file__).resolve().parents[1]), odir)
            _o.makedirs(odir, exist_ok=True)
            self._f_stat = open(_o.path.join(odir, "receiver.jsonl"), "w", encoding="utf-8")
        self.fw = FrameWriter()
        self.jr = None
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
        qd = int(cfg["runtime"].get("qos_depth", 30))
        qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT if rel.startswith("best")
                         else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST, depth=qd)
        self.get_logger().info(f"[recv] QoS {rel} depth={qd}")
        par = ReentrantCallbackGroup()
        self.pub_det = self.create_publisher(Detection3DArray, "/sam6d/detections", 10)
        self.pub_status = self.create_publisher(String, "/sam6d/status", 10)
        self.create_subscription(CameraInfo, t["caminfo"], self._on_caminfo, qos,
                                 callback_group=par)
        self.sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, t["rgb"], qos_profile=qos),
             Subscriber(self, Image, t["depth"], qos_profile=qos)],
            queue_size=int(cfg["runtime"].get("sync_queue", 2)),
            slop=float(cfg["runtime"].get("sync_slop", 0.02)))
        self.sync.registerCallback(self._on_pair)
        # ── ch2: SLAM 자세 구독 → 포즈 링 ──────────────────────────────
        # ⚠ 자세는 **프레임 헤더에 실을 수 없다** — 프레임을 쓰는 시점에 그 프레임의
        #   자세는 아직 없다(ORB-SLAM3 가 30 ms 쯤 뒤에 낸다). 이력으로 흘려보내고
        #   추론 쪽이 capture stamp 로 조회한다.
        # ⚠ SAM-6D 는 ROS_DOMAIN_ID 72, ORB-SLAM3 는 71 로 갈라져 있다(README §5).
        #   자세를 받으려면 도메인을 맞추거나 브리지를 놓아야 한다.
        mpc = cfg["runtime"].get("map_prior") or {}
        self.pw_slam = self.pw_self = None
        if mpc.get("enabled", True) and str(mpc.get("pose_source", "none")) not in (
                "none", "trajectory_file"):
            src = str(mpc.get("pose_source", "both"))
            if src in ("slam_extrinsic", "both"):
                self.pw_slam = PoseRingWriter("sam6d_pose")
                self.create_subscription(
                    PoseStamped, mpc.get("pose_topic", "/orbslam3/pose"),
                    lambda m: self._on_pose(m, self.pw_slam), 50, callback_group=par)
            if src in ("self_localization", "both"):
                self.pw_self = PoseRingWriter("sam6d_pose_self")
                self.create_subscription(
                    PoseStamped, mpc.get("self_pose_topic", "/orbslam3_sam/pose"),
                    lambda m: self._on_pose(m, self.pw_self), 50, callback_group=par)
            self.get_logger().info(f"[recv] ch2 포즈 구독: {src}")

        self.create_timer(0.02, self._pump, callback_group=par)     # 결과 회수·발행
        self.create_timer(float(cfg["runtime"].get("status_period_s", 1.0)),
                          self._status, callback_group=par)
        self.create_timer(10.0, self._warn_if_silent, callback_group=par)
        self.get_logger().info("[recv] 수신 전용 노드 시작 (추론은 별도 프로세스)")

    def _on_pose(self, msg, w):
        """자세는 그냥 링에 흘린다. 시각 정합은 추론 쪽이 capture stamp 로 한다."""
        if w is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.position, msg.pose.orientation
        w.write(t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _on_caminfo(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f"[recv] caminfo fx={self.K[0, 0]:.1f}")

    def _on_pair(self, rgb_msg, depth_msg):
        self.n_in += 1
        if self.K is None:
            return
        t_cb = time.perf_counter()
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        stamp = rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        self.fw.write(np.ascontiguousarray(rgb), np.ascontiguousarray(depth),
                      self.K, stamp, time.time())
        self.n_written += 1
        if self._f_stat is not None:
            self._f_stat.write(json.dumps({
                "stamp_ns": stamp,
                "deliver_ms": round((now_ns - stamp) / 1e6, 1),   # 촬영 → 수신 콜백
                "cb_ms": round(1e3 * (time.perf_counter() - t_cb), 1),
            }) + "\n")

    def _pump(self):
        if self.jr is None:
            try:
                self.jr = JsonReader()
            except FileNotFoundError:
                return
        r = self.jr.read_new()
        if r is None:
            return
        arr = Detection3DArray()
        arr.header.stamp.sec = r["stamp_ns"] // 1_000_000_000
        arr.header.stamp.nanosec = r["stamp_ns"] % 1_000_000_000
        for d in r["dets"]:
            det = Detection3D()
            det.header = arr.header
            det.id = d["object"]
            p = Pose()
            p.position.x, p.position.y, p.position.z = [float(v) / 1000.0 for v in d["t_mm"]]
            q = Rotation.from_matrix(np.array(d["R"])).as_quat()
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
        self.pub_status.publish(String(data=json.dumps({
            "ready": True, "frames_in": self.n_in, "frames_written": self.n_written,
            "processed": last.get("n_proc", 0), "detections": last.get("n_det", 0),
            "last_ms": last.get("ms", {})}, ensure_ascii=False)))

    def close(self):
        self.get_logger().info(f"[recv] 수신 {self.n_in} · 공유메모리 기록 {self.n_written}")
        self.fw.close()
        for w in (self.pw_slam, self.pw_self):     # 죽은 실행이 세그먼트를 남기지 않게
            if w is not None:
                w.close()
        if self._f_stat:
            self._f_stat.close()
        if self.jr:
            self.jr.close()


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
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
