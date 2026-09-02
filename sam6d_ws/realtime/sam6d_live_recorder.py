#!/usr/bin/env python3
"""Non-blocking research recorder for the split SAM-6D live pipeline."""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shm_channel import FrameReader, JsonReader  # noqa: E402

REPO = Path(__file__).resolve().parents[1]


def _atomic_json(path, value):
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, path)


class _Encoder:
    """One bounded queue and one ffmpeg process; producers never wait."""

    def __init__(self, output, index_path, fps, kind, cq=32, queue_size=8):
        self.output, self.index_path = Path(output), Path(index_path)
        self.fps, self.kind, self.cq = float(fps), kind, int(cq)
        self.queue = queue.Queue(maxsize=queue_size)
        self.closing = threading.Event()
        self.process = None
        self.enqueued = self.written = self.dropped = 0
        self.error = None
        self.thread = threading.Thread(target=self._run, name=f"record-{kind}", daemon=True)
        self.thread.start()

    def put(self, image, metadata):
        if self.error or self.closing.is_set():
            self.dropped += 1
            return False
        try:
            self.queue.put_nowait((image, metadata))
            self.enqueued += 1
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _command(self, width, height):
        raw = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
               "-f", "rawvideo", "-pix_fmt", "rgb24" if self.kind == "rgb" else "gray16le",
               "-s:v", f"{width}x{height}", "-framerate", f"{self.fps:g}", "-i", "-"]
        if self.kind == "rgb":
            return raw + ["-an", "-c:v", "h264_nvenc", "-preset", "p4", "-tune", "hq",
                          "-rc", "vbr", "-cq", str(self.cq), "-b:v", "0",
                          "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.output)]
        return raw + ["-an", "-c:v", "ffv1", "-level", "3", "-threads", "2",
                      str(self.output)]

    def _fail(self, exc):
        detail = str(exc)
        if self.process and self.process.poll() is not None and self.process.stderr:
            try:
                stderr = self.process.stderr.read().decode("utf-8", errors="replace").strip()
                if stderr:
                    detail = f"{detail}: {stderr[-1000:]}"
            except Exception:
                pass
        self.error = detail

    def _run(self):
        shape = None
        try:
            with self.index_path.open("w", encoding="utf-8") as index:
                while not self.closing.is_set() or not self.queue.empty():
                    try:
                        image, metadata = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    h, w = image.shape[:2]
                    if self.process is None:
                        shape = (h, w)
                        self.process = subprocess.Popen(
                            self._command(w, h), stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                            start_new_session=True)
                    if shape != (h, w):
                        raise ValueError(f"frame size changed from {shape} to {(h, w)}")
                    self.process.stdin.write(image.tobytes())
                    metadata = {"frame_index": self.written, **metadata}
                    index.write(json.dumps(metadata, ensure_ascii=False) + "\n")
                    index.flush()
                    self.written += 1
            if self.process is not None:
                self.process.stdin.close()
                rc = self.process.wait(timeout=15)
                if rc:
                    self._fail(RuntimeError(f"ffmpeg exited {rc}"))
        except Exception as exc:
            self._fail(exc)
        finally:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()

    def close(self):
        self.closing.set()
        self.thread.join(timeout=20)
        if self.thread.is_alive():
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                self.error = self.error or "encoder did not stop"


class LiveRecorder:
    def __init__(self, config_path):
        self.config_path = Path(config_path).resolve()
        self.cfg = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}
        out = self.cfg.get("output", {})
        self.output = Path(out.get("dir", "output/live"))
        if not self.output.is_absolute():
            self.output = REPO / self.output
        self.output.mkdir(parents=True, exist_ok=True)
        rec = out.get("live_recording", {}) or {}
        self.policy = str(rec.get("depth_policy", "processed"))
        if self.policy not in {"processed", "full"}:
            raise ValueError("depth_policy must be processed or full")
        self.fps = float(rec.get("rgb_fps", 30))
        if not math.isfinite(self.fps) or self.fps <= 0:
            raise ValueError("rgb_fps must be finite and positive")
        protected = ("live_manifest.json", "rgb.mp4", "rgb_frames.jsonl",
                     "depth.mkv", "depth_frames.jsonl")
        existing = [name for name in protected if (self.output / name).exists()]
        if existing:
            raise FileExistsError(
                f"refusing to replace live recording in {self.output}: {', '.join(existing)}")
        self.rgb = _Encoder(self.output / "rgb.mp4", self.output / "rgb_frames.jsonl",
                            self.fps, "rgb", rec.get("nvenc_cq", 32), queue_size=120)
        self.depth = _Encoder(self.output / "depth.mkv", self.output / "depth_frames.jsonl",
                              self.fps, "depth", queue_size=8)
        self.cache = collections.OrderedDict()
        self.stop = threading.Event()
        self.stop_reason = "normal"
        self.started_wall_ns = time.time_ns()
        self.first_stamp_ns = self.last_stamp_ns = None
        self.resolution = None
        self.rgb_observed = self.source_missed = self.processed_seen = 0
        self.processed_results_missed = 0
        self.processed_depth_cache_miss = 0
        self.last_source_seq = None
        self.last_result_seq = None
        self.manifest_path = self.output / "live_manifest.json"
        run_config = self.output / "run_config.yaml"
        if not run_config.exists():
            run_config.write_text(yaml.safe_dump(self.cfg, sort_keys=False), encoding="utf-8")
        self._write_manifest(finalized=False, completed=False)

    def _signal(self, signum, _frame):
        self.stop_reason = signal.Signals(signum).name
        self.stop.set()

    def _receiver_summary(self, finalized=False):
        summary = {"frames_shared": None, "last_source_seq": self.last_source_seq,
                   "camera_info": None,
                   "rgb_frame_id": None, "depth_frame_id": None}
        path = self.output / "receiver.jsonl"
        try:
            lines = path.open(encoding="utf-8")
            rows = lines if finalized else [lines.readline()]
            count = 0
            for line in rows:
                if not line:
                    continue
                row = json.loads(line)
                count += 1
                summary["last_source_seq"] = row.get("source_seq")
                summary["rgb_frame_id"] = row.get("rgb_frame_id")
                summary["depth_frame_id"] = row.get("depth_frame_id")
                summary["camera_info"] = row.get("camera_info", summary["camera_info"])
            if finalized:
                summary["frames_shared"] = count
            lines.close()
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        return summary

    def _manifest(self, finalized, completed):
        receiver = self._receiver_summary(finalized)
        errors = [f"rgb: {self.rgb.error}" if self.rgb.error else None,
                  f"depth: {self.depth.error}" if self.depth.error else None]
        errors = [e for e in errors if e]
        return {
            "format": "live_replay", "schema": "sam6d-live-replay", "schema_version": 1,
            "completed": bool(completed), "finalized": bool(finalized),
            "started_wall_ns": self.started_wall_ns,
            "ended_wall_ns": time.time_ns() if finalized else None,
            "first_stamp_ns": self.first_stamp_ns, "last_stamp_ns": self.last_stamp_ns,
            "resolution": self.resolution,
            "stop_reason": self.stop_reason if finalized else None,
            "topics": self.cfg.get("topics", {}),
            "camera_info": receiver.pop("camera_info"),
            "receiver": receiver,
            "recording": {
                "depth_policy": self.policy, "fps": self.fps,
                "rgb": {"file": "rgb.mp4", "index": "rgb_frames.jsonl",
                        "codec": "h264_nvenc", "cq": self.rgb.cq},
                "depth": {"file": "depth.mkv", "index": "depth_frames.jsonl",
                          "codec": "ffv1", "pixel_format": "gray16le", "threads": 2},
            },
            "counts": {
                "rgb_observed": self.rgb_observed, "rgb_recorded": self.rgb.written,
                "rgb_queue_dropped": self.rgb.dropped,
                "rgb_encoder_unwritten": self.rgb.enqueued - self.rgb.written,
                "source_frames_missed_before_reader": self.source_missed,
                "processed_results_seen": self.processed_seen,
                "processed_results_missed_before_reader": self.processed_results_missed,
                "processed_depth_cache_miss": self.processed_depth_cache_miss,
                "depth_recorded": self.depth.written,
                "depth_queue_dropped": self.depth.dropped,
                "depth_encoder_unwritten": self.depth.enqueued - self.depth.written,
            },
            "errors": errors,
        }

    def _write_manifest(self, finalized, completed):
        _atomic_json(self.manifest_path, self._manifest(finalized, completed))

    def _on_frame(self, got, reader):
        rgb, depth, _K, stamp_ns, _recv_wall = got
        source_seq = reader.last_source_seq
        if source_seq is None or source_seq < 0:
            source_seq = self.rgb_observed
        if self.last_source_seq is None:
            self.source_missed += max(0, source_seq)
        else:
            self.source_missed += max(0, source_seq - self.last_source_seq - 1)
        self.last_source_seq = source_seq
        self.rgb_observed += 1
        self.resolution = {"width": int(rgb.shape[1]), "height": int(rgb.shape[0])}
        self.first_stamp_ns = stamp_ns if self.first_stamp_ns is None else self.first_stamp_ns
        self.last_stamp_ns = stamp_ns
        meta = {"source_seq": source_seq, "stamp_ns": stamp_ns,
                "depth_stamp_ns": reader.last_depth_stamp_ns}
        self.rgb.put(rgb, meta)
        self.cache[(source_seq, stamp_ns)] = (depth, meta)
        while len(self.cache) > 120:
            self.cache.popitem(last=False)
        if self.policy == "full":
            self.depth.put(depth, {**meta, "policy": "full"})

    def _on_result(self, result):
        self.processed_seen += 1
        result_seq = int(result.get("frame_seq", -1))
        if result_seq >= 0:
            if self.last_result_seq is None:
                self.processed_results_missed += result_seq
            else:
                self.processed_results_missed += max(
                    0, result_seq - self.last_result_seq - 1)
            self.last_result_seq = result_seq
        if self.policy != "processed":
            return
        key = (int(result.get("source_seq", -1)), int(result["stamp_ns"]))
        cached = self.cache.get(key)
        if cached is None and key[0] < 0:
            cached = next((value for (source_seq, stamp), value in reversed(self.cache.items())
                           if stamp == key[1]), None)
        if cached is None:
            self.processed_depth_cache_miss += 1
            return
        depth, meta = cached
        self.depth.put(depth, {**meta, "policy": "processed",
                               "frame_seq": result.get("frame_seq")})

    def run(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self._signal)
        frame_reader = result_reader = None
        next_manifest = time.monotonic() + 1
        try:
            while not self.stop.is_set():
                if frame_reader is None:
                    try:
                        frame_reader = FrameReader()
                    except FileNotFoundError:
                        time.sleep(0.05)
                        continue
                if result_reader is None:
                    try:
                        result_reader = JsonReader()
                    except FileNotFoundError:
                        pass
                got = frame_reader.read_new()
                if got is not None:
                    self._on_frame(got, frame_reader)
                if result_reader is not None:
                    result = result_reader.read_new()
                    if result is not None:
                        self._on_result(result)
                if time.monotonic() >= next_manifest:
                    self._write_manifest(finalized=False, completed=False)
                    next_manifest = time.monotonic() + 1
                if self.rgb.error or self.depth.error:
                    self.stop_reason = "encoder_error"
                    self.stop.set()
                if got is None:
                    time.sleep(0.001)
        finally:
            if frame_reader is not None:
                frame_reader.close()
            if result_reader is not None:
                result_reader.close()
            self.rgb.closing.set()
            self.depth.closing.set()
            self.rgb.close()
            self.depth.close()
            if self.rgb_observed == 0:
                self.rgb.error = self.rgb.error or "no RGB frames observed"
            if self.rgb_observed and self.depth.written == 0:
                self.depth.error = self.depth.error or "no depth frames recorded"
            completed = not self.rgb.error and not self.depth.error
            self._write_manifest(finalized=True, completed=completed)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    recorder = LiveRecorder(args.config)
    print(f"[record] {recorder.policy} depth -> {recorder.output}", flush=True)
    recorder.run()


if __name__ == "__main__":
    main()
