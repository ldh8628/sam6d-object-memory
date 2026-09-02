#!/usr/bin/env python3
"""rx_probe.py — 30 Hz 입력이 어디서 10 Hz 로 줄어드는지 단계별로 센다.

한 프로세스에서 네 가지를 동시에 센다.
  A) BEST_EFFORT depth=2   (지금 수신 노드가 쓰는 설정)
  B) BEST_EFFORT depth=30
  C) RELIABLE  depth=30
  D) ApproximateTimeSynchronizer(rgb+depth) 로 짝지어진 수
동시에 rgb·depth 스탬프 차이도 기록한다(슬롭 20 ms 로 충분한지).
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber

RGB = "/camera/camera/color/image_raw"
DEP = "/camera/camera/aligned_depth_to_color/image_raw"


class Probe(Node):
    def __init__(self):
        super().__init__("rx_probe")
        self.n = {"A": 0, "B": 0, "C": 0, "D": 0, "rgbC": 0, "depC": 0}
        self.dt = []
        self.t0 = None
        par = ReentrantCallbackGroup()
        be2 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        be30 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=30)
        rel30 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=30)
        self.create_subscription(Image, RGB, lambda m: self._c("A", m), be2, callback_group=par)
        self.create_subscription(Image, RGB, lambda m: self._c("B", m), be30, callback_group=par)
        self.create_subscription(Image, RGB, lambda m: self._c("C", m), rel30, callback_group=par)
        self.create_subscription(Image, DEP, lambda m: self._c("depC", m), rel30, callback_group=par)
        s = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, RGB, qos_profile=be2),
             Subscriber(self, Image, DEP, qos_profile=be2)], queue_size=2, slop=0.02)
        s.registerCallback(self._pair)
        self.sync = s
        self.create_timer(5.0, self._report, callback_group=par)

    def _c(self, k, msg):
        if self.t0 is None:
            self.t0 = time.time()
        self.n[k] += 1

    def _pair(self, a, b):
        self.n["D"] += 1
        ta = a.header.stamp.sec + a.header.stamp.nanosec * 1e-9
        tb = b.header.stamp.sec + b.header.stamp.nanosec * 1e-9
        self.dt.append(abs(ta - tb))

    def _report(self):
        if not self.t0:
            return
        el = max(1e-6, time.time() - self.t0)
        import numpy as np
        d = np.array(self.dt) if self.dt else np.zeros(1)
        print(f"[{el:5.1f}s] BE-2 {self.n['A']/el:5.1f} Hz · BE-30 {self.n['B']/el:5.1f} · "
              f"REL-30 {self.n['C']/el:5.1f} · depth REL-30 {self.n['depC']/el:5.1f} · "
              f"짝지음 {self.n['D']/el:5.1f} Hz | rgb-depth 스탬프차 중앙 {np.median(d)*1000:.1f} ms "
              f"최대 {d.max()*1000:.1f} ms", flush=True)


def main():
    rclpy.init()
    n = Probe()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


main()
