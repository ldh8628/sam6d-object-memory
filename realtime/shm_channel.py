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

HDR = 384          # 헤더 바이트 (frame metadata + pose + K + UTF-8 map_id)
MAXBUF = 12 << 20   # 프레임 한 장 최대 12 MB (1280x720 RGB+depth 여유)


def _hdr(buf):
    return (np.ndarray((8,), np.int64, buffer=buf, offset=0),      # seq, frame, pose metadata
            np.ndarray((17,), np.float64, buffer=buf, offset=64),  # recv_wall + T_map_camera
            np.ndarray((9,), np.float64, buffer=buf, offset=200),  # K
            np.ndarray((96,), np.uint8, buffer=buf, offset=288))   # map_id


class FrameWriter:
    def __init__(self, name="sam6d_frame", size=MAXBUF):
        try:
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            shared_memory.SharedMemory(name=name).unlink()
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        self.i, self.f, self.K, self.map_id = _hdr(self.shm.buf)
        self.i[0] = 0

    def write(self, rgb: np.ndarray, depth: np.ndarray, K, stamp_ns: int, recv_wall: float,
              slam_context=None, depth_stamp_ns=None):
        h, w = rgb.shape[:2]
        nr, nd = rgb.nbytes, depth.nbytes
        if HDR + nr + nd > self.shm.size:
            raise ValueError("공유메모리가 너무 작다")
        self.i[0] += 1                                   # 홀수 = 쓰는 중
        self.i[1], self.i[2], self.i[3] = stamp_ns, h, w
        self.i[6] = stamp_ns if depth_stamp_ns is None else int(depth_stamp_ns)
        self.f[0] = recv_wall
        slam_context = slam_context or {}
        pose = slam_context.get("T_map_camera")
        valid = pose is not None
        self.i[4] = int(slam_context.get("pose_stamp_ns", -1))
        self.i[5] = 1 if (valid and slam_context.get("tracking_state") == "TRACKING_OK") else 0
        self.f[1:17] = (np.asarray(pose, np.float64).reshape(-1) if valid
                        else np.full(16, np.nan, np.float64))
        encoded_map = str(slam_context.get("map_id", "")).encode("utf-8")[:95]
        self.map_id[:] = 0
        self.map_id[:len(encoded_map)] = np.frombuffer(encoded_map, np.uint8)
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
        self.i, self.f, self.K, self.map_id = _hdr(self.shm.buf)
        self.last = -1
        self.last_slam_context = None
        self.last_depth_stamp_ns = None

    def read_new(self):
        """새 프레임이 있으면 (rgb, depth, K, stamp_ns, recv_wall), 없으면 None."""
        s0 = int(self.i[0])
        if s0 % 2 or s0 == self.last or s0 == 0:
            return None
        stamp, h, w = int(self.i[1]), int(self.i[2]), int(self.i[3])
        depth_stamp = int(self.i[6])
        recv, K = float(self.f[0]), np.array(self.K).reshape(3, 3)
        pose_stamp, tracking_ok = int(self.i[4]), bool(self.i[5])
        pose = np.array(self.f[1:17]).reshape(4, 4)
        map_bytes = bytes(self.map_id).split(b"\0", 1)[0]
        nr, nd = h * w * 3, h * w * 2
        rgb = np.frombuffer(self.shm.buf[HDR:HDR + nr], np.uint8).reshape(h, w, 3).copy()
        dep = np.frombuffer(self.shm.buf[HDR + nr:HDR + nr + nd], np.uint16).reshape(h, w).copy()
        if int(self.i[0]) != s0:                          # 읽는 중에 덮어써졌다 → 버린다
            return None
        self.last = s0
        self.last_depth_stamp_ns = depth_stamp
        self.last_slam_context = {
            "T_map_camera": pose,
            "pose_stamp_ns": pose_stamp,
            "rgb_stamp_ns": stamp,
            "tracking_state": "TRACKING_OK" if tracking_ok else "TRACKING_LOST",
            "map_id": map_bytes.decode("utf-8", errors="replace"),
        }
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
