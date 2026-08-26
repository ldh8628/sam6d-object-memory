#!/usr/bin/env python3
"""shm_channel.py — 두 프로세스가 공유메모리로 '최신 것 하나'만 주고받는 통로.

락을 쓰지 않는다. 쓰는 쪽이 순번(seq)을 **홀수로 올리고 → 쓰고 → 짝수로 올린다**.
읽는 쪽은 순번을 앞뒤로 두 번 읽어 같고 짝수일 때만 그 스냅샷을 신뢰한다(seqlock).
읽는 쪽이 느려도 쓰는 쪽은 절대 막히지 않고, 밀린 프레임은 그냥 덮어써진다 —
실시간에서 원하는 동작 그대로다.
"""
from __future__ import annotations

import json
from multiprocessing import shared_memory

import numpy as np

HDR = 128          # 헤더 바이트
MAXBUF = 12 << 20   # 프레임 한 장 최대 12 MB (1280x720 RGB+depth 여유)


def _hdr(buf):
    return (np.ndarray((4,), np.int64, buffer=buf, offset=0),      # seq, stamp_ns, h, w
            np.ndarray((2,), np.float64, buffer=buf, offset=32),   # recv_wall, spare
            np.ndarray((9,), np.float64, buffer=buf, offset=48))   # K


class FrameWriter:
    def __init__(self, name="sam6d_frame", size=MAXBUF):
        try:
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            shared_memory.SharedMemory(name=name).unlink()
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        self.i, self.f, self.K = _hdr(self.shm.buf)
        self.i[0] = 0

    def write(self, rgb: np.ndarray, depth: np.ndarray, K, stamp_ns: int, recv_wall: float):
        h, w = rgb.shape[:2]
        nr, nd = rgb.nbytes, depth.nbytes
        if HDR + nr + nd > self.shm.size:
            raise ValueError("공유메모리가 너무 작다")
        self.i[0] += 1                                   # 홀수 = 쓰는 중
        self.i[1], self.i[2], self.i[3] = stamp_ns, h, w
        self.f[0] = recv_wall
        self.K[:] = np.asarray(K, np.float64).ravel()
        self.shm.buf[HDR:HDR + nr] = rgb.tobytes()
        self.shm.buf[HDR + nr:HDR + nr + nd] = depth.tobytes()
        self.i[0] += 1                                   # 짝수 = 완료

    def close(self):
        self.shm.close()
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass


class FrameReader:
    def __init__(self, name="sam6d_frame"):
        self.shm = shared_memory.SharedMemory(name=name)
        self.i, self.f, self.K = _hdr(self.shm.buf)
        self.last = -1

    def read_new(self):
        """새 프레임이 있으면 (rgb, depth, K, stamp_ns, recv_wall), 없으면 None."""
        s0 = int(self.i[0])
        if s0 % 2 or s0 == self.last or s0 == 0:
            return None
        stamp, h, w = int(self.i[1]), int(self.i[2]), int(self.i[3])
        recv, K = float(self.f[0]), np.array(self.K).reshape(3, 3)
        nr, nd = h * w * 3, h * w * 2
        rgb = np.frombuffer(self.shm.buf[HDR:HDR + nr], np.uint8).reshape(h, w, 3).copy()
        dep = np.frombuffer(self.shm.buf[HDR + nr:HDR + nr + nd], np.uint16).reshape(h, w).copy()
        if int(self.i[0]) != s0:                          # 읽는 중에 덮어써졌다 → 버린다
            return None
        self.last = s0
        return rgb, dep, K, stamp, recv

    def close(self):
        self.shm.close()


class JsonWriter:
    def __init__(self, name="sam6d_result", size=4 << 20):
        try:
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            shared_memory.SharedMemory(name=name).unlink()
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        self.i = np.ndarray((2,), np.int64, buffer=self.shm.buf, offset=0)
        self.i[0] = 0

    def write(self, obj):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.i[0] += 1
        self.i[1] = len(b)
        self.shm.buf[16:16 + len(b)] = b
        self.i[0] += 1

    def close(self):
        self.shm.close()
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass


class JsonReader:
    def __init__(self, name="sam6d_result"):
        self.shm = shared_memory.SharedMemory(name=name)
        self.i = np.ndarray((2,), np.int64, buffer=self.shm.buf, offset=0)
        self.last = -1

    def read_new(self):
        s0 = int(self.i[0])
        if s0 % 2 or s0 == self.last or s0 == 0:
            return None
        n = int(self.i[1])
        b = bytes(self.shm.buf[16:16 + n])
        if int(self.i[0]) != s0:
            return None
        self.last = s0
        try:
            return json.loads(b.decode())
        except Exception:
            return None

    def close(self):
        self.shm.close()


# ===========================================================================
# 포즈 링 — SLAM 자세를 수신 프로세스 → 추론 프로세스로 넘긴다 (ch2)
# ===========================================================================
# ⚠ 프레임 헤더에 자세를 실으면 **안 된다.** 프레임을 쓰는 시점에 그 프레임의 SLAM
#   자세는 아직 없다(ORB-SLAM3 가 30 ms 쯤 뒤에 낸다). 그때 붙이면 *이전* 프레임의
#   자세가 붙는다. 그래서 자세는 이력으로 흘려보내고, 추론 쪽이 **capture stamp 로**
#   조회해 쓴다.
#
# 프레임 통로와 달리 최신 하나가 아니라 **링(과거 N 개)** 이다 — 늦게 도착한 결과를
# 그 프레임을 찍던 순간의 자세에 맞춰야 하기 때문이다.
POSE_SLOTS = 512
POSE_HDR = 16          # cursor(int64), spare(int64)
POSE_REC = 8           # stamp_s, tx,ty,tz, qx,qy,qz,qw (float64)
POSE_SIZE = POSE_HDR + POSE_SLOTS * POSE_REC * 8


class PoseRingWriter:
    """수신 프로세스가 자세를 흘려 넣는다. 절대 막히지 않고, 오래된 것부터 덮어쓴다."""

    def __init__(self, name="sam6d_pose"):
        try:
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=POSE_SIZE)
        except FileExistsError:                     # 죽은 실행이 남긴 세그먼트를 치운다
            shared_memory.SharedMemory(name=name).unlink()
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=POSE_SIZE)
        self.c = np.ndarray((2,), np.int64, buffer=self.shm.buf, offset=0)
        self.d = np.ndarray((POSE_SLOTS, POSE_REC), np.float64,
                            buffer=self.shm.buf, offset=POSE_HDR)
        self.c[0] = 0

    def write(self, stamp_s: float, xyz, quat_xyzw):
        n = int(self.c[0])
        self.d[n % POSE_SLOTS, 0] = float(stamp_s)
        self.d[n % POSE_SLOTS, 1:4] = np.asarray(xyz, np.float64)
        self.d[n % POSE_SLOTS, 4:8] = np.asarray(quat_xyzw, np.float64)
        self.c[0] = n + 1                            # 슬롯을 다 쓴 뒤에 커서를 올린다

    def close(self):
        self.shm.close()
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass


class PoseRingReader:
    """추론 프로세스가 새로 들어온 자세만 집어 간다. 없으면 빈 목록."""

    def __init__(self, name="sam6d_pose"):
        self.shm = shared_memory.SharedMemory(name=name)
        self.c = np.ndarray((2,), np.int64, buffer=self.shm.buf, offset=0)
        self.d = np.ndarray((POSE_SLOTS, POSE_REC), np.float64,
                            buffer=self.shm.buf, offset=POSE_HDR)
        self.seen = int(self.c[0])

    def drain(self):
        """되돌림: [(stamp_s, xyz(3,), quat_xyzw(4,)), ...] — 새로 들어온 것만."""
        n = int(self.c[0])
        if n <= self.seen:
            return []
        first = max(self.seen, n - POSE_SLOTS)       # 밀려서 덮인 것은 포기한다
        out = []
        for k in range(first, n):
            r = self.d[k % POSE_SLOTS].copy()
            out.append((float(r[0]), r[1:4], r[4:8]))
        self.seen = n
        return out

    def close(self):
        self.shm.close()
