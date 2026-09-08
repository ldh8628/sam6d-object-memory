#!/usr/bin/env python3
"""sam6d_viewer.py — 시각화 프로세스.

수신 프로세스가 쓰는 프레임 공유메모리와 추론 프로세스가 쓰는 결과 공유메모리를
**그대로 읽어서** 그린다. 그래서

  · ROS 토픽을 하나도 늘리지 않는다 (30 Hz 영상을 또 구독하지 않는다)
  · 수신/추론과 별개 프로세스라 GIL 을 다투지 않는다 (NOTES 19~22절)
  · mp4 렌더와 **같은 화면**을 실시간으로 본다

화면은 세 칸이다.
  왼쪽   지금 들어오는 영상 (수신되는 대로)
  가운데 마지막으로 처리된 프레임 + CAD 점군 투영
  오른쪽 같은 프레임 + 3D 경계상자 (축 색 X 빨강 / Y 초록 / Z 파랑)
아래 띠는 같은 객체의 연속 추정 사이 회전 변화량(Δrot) 이력이다.

    conda activate sam6d          # (또는 numpy·opencv 가 있는 아무 env)
    python realtime/sam6d_viewer.py                 # 창으로 보기
    python realtime/sam6d_viewer.py --save live.mp4 # 보면서 녹화
    python realtime/sam6d_viewer.py --headless --save live.mp4   # 화면 없이 녹화만
    python realtime/sam6d_viewer.py --overlay-topic /sam6d/overlay # map overlay
    SAM6D_VIEWER_POSE_SYNC=0 python realtime/sam6d_viewer.py ...  # 이전 비동기 방식
"""
from __future__ import annotations

import argparse
import collections
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shm_channel import FrameReader, JsonReader          # noqa: E402
from two_host_guard import guard_healthy

REPO = Path(__file__).resolve().parents[1]
PALETTE = [(80, 220, 90), (60, 165, 255), (240, 170, 60), (200, 100, 240),
           (80, 225, 225), (150, 150, 255), (120, 200, 160), (255, 205, 120),
           (175, 120, 255), (110, 240, 185)]
AXCOL = [(70, 70, 255), (80, 220, 80), (255, 170, 60)]      # X 빨강 Y 초록 Z 파랑
EDGES = [(i, i ^ b, {4: 0, 2: 1, 1: 2}[b]) for i in range(8) for b in (4, 2, 1) if not (i & b)]


def geodesic_deg(A, B):
    c = np.clip((np.trace(np.asarray(A).T @ np.asarray(B)) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


class MapTopdown:
    """Static PCD raster plus a few live pose markers; no 3-D renderer needed."""

    def __init__(self, points, label, size=480):
        points = np.asarray(points, np.float32)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            raise ValueError("map PCD contains no finite points")
        if len(points) > 200_000:
            points = points[::int(np.ceil(len(points) / 200_000))]
        xz = points[:, (0, 2)]
        lo, hi = np.percentile(xz, (1, 99), axis=0)
        span = np.maximum(hi - lo, 0.5)
        lo, hi = lo - span * 0.05, hi + span * 0.05
        self.size, self.label, self.lo, self.hi = size, label, lo, hi
        self.scale = (size - 36) / max(hi - lo)
        used = (hi - lo) * self.scale
        self.offset = (np.array([size, size]) - used) / 2
        uv = self._pixels(xz)
        uv = uv[(uv[:, 0] >= 0) & (uv[:, 0] < size) &
                (uv[:, 1] >= 0) & (uv[:, 1] < size)]
        counts = np.bincount(uv[:, 1] * size + uv[:, 0], minlength=size * size)
        density = np.log1p(counts.reshape(size, size)).astype(np.float32)
        if density.max() > 0:
            density *= 180.0 / density.max()
        density = cv2.dilate(density.astype(np.uint8), np.ones((2, 2), np.uint8))
        self.background = np.full((size, size, 3), 18, np.uint8)
        self.background[..., 0] += density // 2
        self.background[..., 1] += density
        self.background[..., 2] += density

    @classmethod
    def from_pcd(cls, path, size=480):
        path = Path(path)
        skip = None
        with path.open(encoding="utf-8", errors="replace") as stream:
            for line_no, line in enumerate(stream):
                if line.strip().lower().startswith("data "):
                    if line.strip().lower() != "data ascii":
                        raise ValueError("only ASCII PCD is supported")
                    skip = line_no + 1
                    break
        if skip is None:
            raise ValueError("invalid PCD header")
        points = np.loadtxt(path, skiprows=skip, usecols=(0, 1, 2),
                            dtype=np.float32, ndmin=2)
        kind = "DENSE MAP" if "dense" in path.name else "SPARSE MAP (dense unavailable)"
        print(f"[view] {kind}: {len(points)} points from {path}")
        return cls(points, kind, size)

    def _pixels(self, xz):
        uv = self.offset + (np.asarray(xz) - self.lo) * self.scale
        uv[..., 1] = self.size - uv[..., 1]
        return np.rint(uv).astype(int)

    def _marker(self, image, marker, color, label):
        if marker is None:
            return
        position, forward = marker
        at = self._pixels(np.asarray(position)[[0, 2]])
        if not (0 <= at[0] < self.size and 0 <= at[1] < self.size):
            return
        direction = np.asarray(forward)[[0, 2]]
        norm = np.linalg.norm(direction)
        end = at if norm < 1e-6 else at + np.rint(
            direction / norm * 18 * np.array([1, -1])).astype(int)
        cv2.circle(image, tuple(at), 6, color, -1, cv2.LINE_AA)
        cv2.arrowedLine(image, tuple(at), tuple(end), color, 2, cv2.LINE_AA,
                        tipLength=0.35)
        cv2.putText(image, label, tuple(at + [8, -7]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, color, 1, cv2.LINE_AA)

    def draw(self, slam=None, sam=None, objects=(), tracking=False):
        image = self.background.copy()
        slam_color = (70, 220, 80) if tracking else (90, 90, 170)
        self._marker(image, slam, slam_color, "SLAM" if tracking else "SLAM LOST")
        self._marker(image, sam, (255, 210, 70) if tracking else (100, 100, 150), "SAM")
        for name, marker in objects:
            self._marker(image, marker, (220, 100, 230), name)
        cv2.rectangle(image, (0, 0), (self.size - 1, 27), (20, 20, 20), -1)
        cv2.putText(image, f"TOP-DOWN X/Z | {self.label}", (8, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (235, 235, 235), 1,
                    cv2.LINE_AA)
        cv2.putText(image, "SLAM", (8, self.size - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (70, 220, 80), 1, cv2.LINE_AA)
        cv2.putText(image, "SAM", (62, self.size - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (255, 210, 70), 1, cv2.LINE_AA)
        cv2.putText(image, "OBJECT", (108, self.size - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (220, 100, 230), 1, cv2.LINE_AA)
        return image


def load_models():
    MP, BOX, idx = {}, {}, {}
    d = REPO / "assets" / "model_points"
    for i, p in enumerate(sorted(d.glob("*.npy"))):
        full = np.load(p).astype(np.float32)
        MP[p.stem] = full[::8]
        idx[p.stem] = i
        lo, hi = full.min(0), full.max(0)
        BOX[p.stem] = np.array([[x, y, z] for x in (lo[0], hi[0])
                                for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], np.float32)
    return MP, BOX, idx


def view_overlay(topic, scale, fps, map_pcd, slam_pose_topic, sam_pose_topic,
                 tracking_topic, landmarks_topic, save="", headless=False, guard_file=""):
    """ObjectMemory가 투영한 ROS Image를 가장 최근 프레임만 보여준다."""
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped
    from message_filters import Subscriber, TimeSynchronizer
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from vision_msgs.msg import Detection3DArray

    renderer = None
    if map_pcd:
        try:
            renderer = MapTopdown.from_pcd(map_pcd)
        except (OSError, ValueError) as exc:
            print(f"[view] top-down map disabled: {exc}", file=sys.stderr)

    def marker(pose):
        p, q = pose.position, pose.orientation
        quat = np.asarray([q.x, q.y, q.z, q.w], float)
        norm = np.linalg.norm(quat)
        if not np.isfinite(quat).all() or norm < 1e-9:
            return None
        x, y, z, w = quat / norm
        forward = np.array([2 * (x * z + w * y),
                            2 * (y * z - w * x),
                            1 - 2 * (x * x + y * y)])
        return np.array([p.x, p.y, p.z]), forward

    class OverlayViewer(Node):
        def __init__(self):
            super().__init__("sam6d_overlay_viewer")
            self.bridge, self.frame = CvBridge(), None
            self.frame_shape = (480, 640, 3)
            self.slam = self.sam = None
            self.objects, self.tracking = [], False
            self.map_id, self.map_changed = None, False
            if guard_file:
                self.create_subscription(String, "/orbslam3/map_id", self._on_map,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(
                Image, topic, self._on_image, qos_profile_sensor_data)
            if os.environ.get("SAM6D_VIEWER_POSE_SYNC", "1") == "0":
                self.create_subscription(
                    PoseStamped, slam_pose_topic,
                    lambda msg: self._on_pose("slam", msg), 10)
                self.create_subscription(
                    PoseStamped, sam_pose_topic,
                    lambda msg: self._on_pose("sam", msg), 10)
                self.get_logger().warn("pose timestamp sync disabled")
            else:
                self.slam_pose_sub = Subscriber(
                    self, PoseStamped, slam_pose_topic, 10)
                self.sam_pose_sub = Subscriber(
                    self, PoseStamped, sam_pose_topic, 10)
                self.pose_sync = TimeSynchronizer(
                    [self.slam_pose_sub, self.sam_pose_sub], queue_size=10)
                self.pose_sync.registerCallback(self._on_pose_pair)
            self.create_subscription(String, tracking_topic, self._on_tracking, 10)
            self.create_subscription(Detection3DArray, landmarks_topic,
                                     self._on_landmarks, 10)

        def _on_image(self, msg):
            if self._allow_updates():
                self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.frame_shape = self.frame.shape

        def _on_map(self, msg):
            value = str(msg.data).strip()
            if value and value != "unknown" and self.map_id is None:
                self.map_id = value
            elif self.map_id is not None and value != self.map_id:
                self.map_changed = True

        def _allow_updates(self):
            if not guard_file:
                return True
            allowed = (self.map_id is not None and not self.map_changed
                       and guard_healthy(guard_file, self.map_id))
            if not allowed:
                self.frame = self.slam = self.sam = None
                self.objects, self.tracking = [], False
            return allowed

        def _on_pose(self, name, msg):
            if self._allow_updates():
                setattr(self, name, marker(msg.pose))

        def _on_pose_pair(self, slam_msg, sam_msg):
            if not self._allow_updates():
                return
            self.slam = marker(slam_msg.pose)
            self.sam = marker(sam_msg.pose)

        def _on_tracking(self, msg):
            self.tracking = self._allow_updates() and str(msg.data).strip().lower() == "tracking"

        def _on_landmarks(self, msg):
            if not self._allow_updates():
                return
            objects = []
            for detection in msg.detections:
                if detection.results:
                    result = detection.results[0]
                    objects.append((str(result.hypothesis.class_id),
                                    marker(result.pose.pose)))
            self.objects = objects

    rclpy.init(args=[])
    node = OverlayViewer()
    enc, n_written, t0 = None, 0, None
    print(f"[view] {topic} 대기 중 ... (창에서 q 또는 Ctrl-C 로 종료)")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0 / max(fps, 1.0))
            healthy = node._allow_updates()
            if healthy and node.frame is None:
                continue
            shown = node.frame if healthy else np.zeros(node.frame_shape, np.uint8)
            if not healthy:
                cv2.putText(shown, "TRACKING LOST: two-host guard", (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (70, 70, 255), 2)
            if renderer is not None:
                topdown = renderer.draw(node.slam, node.sam, node.objects, node.tracking)
                if topdown.shape[0] != shown.shape[0]:
                    width = round(topdown.shape[1] * shown.shape[0] / topdown.shape[0])
                    topdown = cv2.resize(topdown, (width, shown.shape[0]))
                shown = np.hstack((shown, topdown))
            if scale != 1.0:
                shown = cv2.resize(shown, None, fx=scale, fy=scale)
            if save:
                if enc is None:
                    h, w = shown.shape[:2]
                    enc = subprocess.Popen(
                        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                         "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}",
                         "-r", f"{fps}", "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
                         "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", save],
                        stdin=subprocess.PIPE)
                    t0 = time.time()
                want = int((time.time() - t0) * fps)
                while n_written < want:
                    enc.stdin.write(shown.tobytes())
                    n_written += 1
            if not headless:
                cv2.imshow("ObjectMemory camera overlay | map top-down", shown)
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if enc is not None:
            enc.stdin.close(); enc.wait()
            print(f"[view] 녹화 저장: {save}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not headless:
            cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default="", help="보이는 그대로 mp4 로 녹화")
    ap.add_argument("--fps", type=float, default=15.0, help="화면·녹화 fps")
    ap.add_argument("--headless", action="store_true", help="창 없이 녹화만")
    ap.add_argument("--scale", type=float, default=1.0, help="창 배율")
    ap.add_argument("--overlay-topic", default="", help="ObjectMemory ROS Image 토픽")
    ap.add_argument("--map-pcd", default="", help="top-down 배경용 dense/sparse PCD")
    ap.add_argument("--slam-pose-topic", default="/orbslam3/pose")
    ap.add_argument("--sam-pose-topic", default="/sam6d/camera_pose")
    ap.add_argument("--tracking-topic", default="/orbslam3/tracking_state")
    ap.add_argument("--landmarks-topic", default="/object_memory/landmarks")
    ap.add_argument("--two-host-guard-file", default="")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()

    if a.self_test:
        from geometry_msgs.msg import PoseStamped
        from message_filters import SimpleFilter, TimeSynchronizer

        points = np.array([[-1, 0, -1], [-1, 0, 1], [1, 0, -1], [1, 0, 1]])
        renderer = MapTopdown(points, "TEST", 160)
        marker_test = (np.zeros(3), np.array([0, 0, 1]))
        image = renderer.draw(marker_test, marker_test, [("object", marker_test)], True)
        assert image.shape == (160, 160, 3) and image.max() > image.min()
        left, right, pairs = SimpleFilter(), SimpleFilter(), []
        sync = TimeSynchronizer([left, right], queue_size=2)
        sync.registerCallback(lambda a, b: pairs.append((a, b)))
        for source, nanosec in ((left, 1), (right, 2), (right, 1)):
            msg = PoseStamped()
            msg.header.stamp.sec = 1
            msg.header.stamp.nanosec = nanosec
            source.signalMessage(msg)
        assert (len(pairs) == 1 and
                pairs[0][0].header.stamp.nanosec ==
                pairs[0][1].header.stamp.nanosec == 1)
        print("self-test: PASS")
        return

    if a.overlay_topic:
        view_overlay(a.overlay_topic, a.scale, a.fps, a.map_pcd,
                     a.slam_pose_topic, a.sam_pose_topic, a.tracking_topic,
                     a.landmarks_topic, a.save, a.headless, a.two_host_guard_file)
        return

    MP, BOX, cidx = load_models()
    print(f"[view] CAD 점군 {len(MP)} 객체")
    fr = jr = None
    for _ in range(600):                       # 수신·추론이 먼저 뜰 때까지 기다린다
        try:
            fr = fr or FrameReader()
            jr = jr or JsonReader()
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if fr is None or jr is None:
        raise SystemExit("[view] 공유메모리를 찾지 못했다. 수신/추론 프로세스를 먼저 띄울 것")

    cache = collections.OrderedDict()          # stamp -> bgr (처리된 프레임을 찾기 위한 최근 창)
    hist = collections.defaultdict(lambda: collections.deque(maxlen=300))
    prevR, last, live, K = {}, None, None, None
    enc, n_written, t0 = None, 0, None
    STRIP, BAR = 130, 34
    fps_in = collections.deque(maxlen=60)
    next_draw = 0.0

    def draw(res, base):
        """가운데(점군)·오른쪽(3D 상자) 두 칸을 만든다."""
        mid, rgt = base.copy(), base.copy()
        h, w = base.shape[:2]
        for j, d in enumerate(sorted(res["dets"], key=lambda x: x["object"])):
            nm = d["object"]
            col = PALETTE[cidx.get(nm, j) % len(PALETTE)]
            R, t = np.array(d["R"]), np.array(d["t_mm"])
            P = MP.get(nm)
            if P is not None:
                q = (R @ P.T).T + t
                q = q[q[:, 2] > 1]
                if len(q):
                    u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(np.int32)
                    v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(np.int32)
                    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                    mid[v[ok], u[ok]] = col
            if nm in BOX:
                q = (R @ BOX[nm].T).T + t
                if (q[:, 2] > 1).all():
                    u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(np.int32)
                    v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(np.int32)
                    for i0, i1, ax in EDGES:
                        cv2.line(rgt, (u[i0], v[i0]), (u[i1], v[i1]), AXCOL[ax], 2, cv2.LINE_AA)
            lb = nm.replace("_high", "") + f" {d['score']:.2f}"
            if d.get("drot") is not None:
                lb += f"  drot {d['drot']:.0f}"
            uu = int(np.clip(K[0, 0] * t[0] / max(t[2], 1) + K[0, 2], 4, w - 120))
            vv = int(np.clip(K[1, 1] * t[1] / max(t[2], 1) + K[1, 2], 14, h - 6))
            for im in (mid, rgt):
                cv2.putText(im, lb, (uu, vv), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
        return mid, rgt

    print("[view] 대기 중 ... (창에서 q 또는 Ctrl-C 로 종료)")
    try:
        while True:
            got = fr.read_new()
            if got is not None:
                rgb, dep, K, stamp, recv = got
                live = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cache[stamp] = live
                while len(cache) > 150:
                    cache.popitem(last=False)
                fps_in.append(time.time())
            r = jr.read_new()
            if r is not None:
                for d in sorted(r["dets"], key=lambda x: x["object"]):
                    nm = d["object"]; R = np.array(d["R"])
                    d["drot"] = geodesic_deg(prevR[nm], R) if nm in prevR else None
                    prevR[nm] = R
                    if d["drot"] is not None:
                        hist[nm].append((time.time(), d["drot"]))
                base = cache.get(r["stamp_ns"])
                if base is None and cache:
                    # 그 프레임을 못 받았으면(뷰어가 잠깐 밀렸으면) 가장 가까운 것으로 대신한다
                    k = min(cache, key=lambda x: abs(x - r["stamp_ns"]))
                    if abs(k - r["stamp_ns"]) < 3e8:      # 0.3 초 이내면 인정
                        base = cache[k]
                if base is not None and K is not None:
                    last = (r, *draw(r, base), time.time())
            if got is None and r is None:
                if not a.headless and (cv2.waitKey(5) & 0xFF) in (ord("q"), 27):
                    break
                if a.headless:
                    time.sleep(0.005)
                continue
            now_draw = time.monotonic()
            if now_draw < next_draw:
                continue
            next_draw = now_draw + 1.0 / max(a.fps, 1.0)
            if live is None:
                time.sleep(0.005)
                continue

            h, w = live.shape[:2]
            H, W = BAR + h + STRIP, w * 3
            canvas = np.empty((H, W, 3), np.uint8)
            hz = (len(fps_in) - 1) / max(fps_in[-1] - fps_in[0], 1e-6) if len(fps_in) > 2 else 0

            def bar(txt, sub=""):
                b = np.full((BAR, w, 3), 26, np.uint8)
                cv2.putText(b, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                            (255, 255, 255), 1, cv2.LINE_AA)
                if sub:
                    cv2.putText(b, sub, (max(4, w - 8 - 9 * len(sub)), 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (170, 210, 255), 1, cv2.LINE_AA)
                return b

            if last is None:
                mid = rgt = np.full_like(live, 18)
                cv2.putText(mid, "waiting for SAM-6D result ...", (30, h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1, cv2.LINE_AA)
                rgt = mid.copy()
                txt, sub = "SAM-6D  (no result yet)", ""
            else:
                r, mid, rgt, tshow = last
                ms = r.get("ms", {})
                txt = f"SAM-6D  obj {r['n']}  proc {r.get('n_proc', 0)}"
                sub = (f"{ms.get('total', 0)}ms [y{ms.get('yolo',0)}/i{ms.get('ism',0)}/"
                       f"p{ms.get('pem',0)}]  held {time.time()-tshow:4.1f}s")
            canvas[:BAR, :w] = bar("LIVE", f"{hz:4.1f} Hz in")
            canvas[:BAR, w:2*w] = bar(txt + "   [CAD points]", sub)
            canvas[:BAR, 2*w:] = bar("3D bbox   X=red Y=green Z=blue", "")
            canvas[BAR:BAR + h, :w] = live
            canvas[BAR:BAR + h, w:2*w] = mid
            canvas[BAR:BAR + h, 2*w:] = rgt
            strip = np.full((STRIP, W, 3), 20, np.uint8)
            cv2.putText(strip, "PEM rotation change between consecutive estimates (deg)",
                        (8, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (205, 205, 205), 1, cv2.LINE_AA)
            now = time.time()
            for gv in (0, 60, 120, 180):
                gy = int(STRIP - 10 - gv / 180.0 * (STRIP - 34))
                cv2.line(strip, (58, gy), (W - 8, gy), (52, 52, 52), 1)
                cv2.putText(strip, f"{gv:3d}", (20, gy + 4), cv2.FONT_HERSHEY_SIMPLEX,
                            0.34, (115, 115, 115), 1, cv2.LINE_AA)
            for nm, hs in hist.items():                 # 최근 60 초
                col = PALETTE[cidx.get(nm, 0) % len(PALETTE)]
                for t, dv in hs:
                    if now - t > 60:
                        continue
                    x = int(58 + (1 - (now - t) / 60.0) * (W - 66))
                    y = int(STRIP - 10 - min(dv, 180) / 180.0 * (STRIP - 34))
                    cv2.circle(strip, (x, y), 2, col, -1)
            canvas[BAR + h:] = strip

            if a.save:
                if enc is None:
                    enc = subprocess.Popen(
                        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                         "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
                         "-r", f"{a.fps}", "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
                         "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.save],
                        stdin=subprocess.PIPE)
                    t0 = time.time()
                    n_written = 0
                # 실시간 흐름을 fps 에 맞춰 적는다(빠뜨리거나 겹쳐 적지 않게)
                want = int((time.time() - t0) * a.fps)
                while n_written < want:
                    enc.stdin.write(canvas.tobytes())
                    n_written += 1
            if not a.headless:
                sh = canvas if a.scale == 1.0 else cv2.resize(
                    canvas, None, fx=a.scale, fy=a.scale)
                cv2.imshow("SAM-6D  live | CAD points | 3D bbox", sh)
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                    break
            else:
                time.sleep(max(0.0, 1.0 / a.fps - 0.005))
    except KeyboardInterrupt:
        pass
    finally:
        if enc is not None:
            enc.stdin.close(); enc.wait()
            print(f"[view] 녹화 저장: {a.save}")
        if not a.headless:
            cv2.destroyAllWindows()
        fr.close(); jr.close()


if __name__ == "__main__":
    main()
