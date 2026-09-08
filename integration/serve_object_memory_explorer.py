#!/usr/bin/env python3
"""Serve an ObjectMemory result directory."""
from __future__ import annotations

import argparse
import json
import mimetypes
import sys
import tempfile
import threading
from collections import defaultdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PurePosixPath
from urllib.parse import parse_qs, unquote, urlsplit

HERE = Path(__file__).resolve().parent
STATIC = HERE / "object_memory_explorer"
SAM_OUTPUT = (HERE.parent / "sam6d_ws" / "output").resolve()
OBJECT_MEMORY_OUTPUT = (HERE.parent / "output" / "object_memory").resolve()


def select_dataset(dataset=None, output_root=OBJECT_MEMORY_OUTPUT):
    root = Path(output_root).resolve()
    if dataset:
        if Path(dataset).name != dataset or dataset in {".", ".."}:
            raise ValueError("dataset must be one directory name")
        candidates = [root / dataset]
    else:
        candidates = [path for path in root.iterdir()] if root.is_dir() else []
        candidates = [path for path in candidates
                      if path.is_dir() and not path.is_symlink()
                      and (path / "fused/object_map.json").is_file()]
        candidates.sort(key=lambda path: (path / "fused/object_map.json").stat().st_mtime,
                        reverse=True)
    if not candidates or not (candidates[0] / "fused/object_map.json").is_file():
        suffix = f"/{dataset}" if dataset else ""
        raise ValueError(f"no ObjectMemory result under {root}{suffix}")
    return candidates[0]


class ExplorerData:
    def __init__(self, dataset_dir):
        raw = Path(dataset_dir)
        if raw.is_symlink() or not raw.is_dir():
            raise ValueError("dataset directory must be a real directory")
        self.dataset_dir = raw.resolve()
        self.fused = self.dataset_dir / "fused"
        map_path = self.fused / "object_map.json"
        if map_path.is_symlink() or not map_path.is_file():
            raise ValueError("fused/object_map.json is missing")
        self.object_map = json.loads(map_path.read_text(encoding="utf-8"))
        explorer = self.object_map.get("explorer") or {}
        if explorer.get("schema_version") != 1:
            raise ValueError("unsupported or missing explorer metadata")

        timeline_name = explorer.get("timeline")
        if not timeline_name or PurePosixPath(timeline_name).name != timeline_name:
            raise ValueError("timeline must be one file inside fused/")
        timeline = self.fused / timeline_name
        if timeline.is_symlink() or not timeline.is_file():
            raise ValueError("timeline file is missing")

        preview = Path((explorer.get("preview") or {}).get("path", ""))
        if preview.is_symlink() or not preview.is_file():
            raise ValueError("preview video is missing or unsafe")
        self.preview = preview.resolve()
        allowed_roots = (SAM_OUTPUT, self.dataset_dir.parent)
        if not any(root == self.preview.parent or root in self.preview.parents
                   for root in allowed_roots):
            raise ValueError("preview video must remain under the dataset or legacy SAM output")

        self.timeline = timeline
        self.frame_offsets = []
        self.object_samples = defaultdict(list)
        previous_status = {}
        latest_objects = {}
        with timeline.open("rb") as stream:
            line_no = 0
            while True:
                offset = stream.tell()
                line = stream.readline()
                if not line:
                    break
                line_no += 1
                row = json.loads(line)
                seq = int(row["frame_seq"])
                if seq != line_no - 1:
                    raise ValueError("timeline frame_seq must be contiguous")
                self.frame_offsets.append(offset)
                events = defaultdict(list)
                for event in row.get("transitions", ()):
                    events[int(event["object_id"])].append(event)
                handled = set()
                for item in row.get("objects", ()):
                    object_id = int(item["object_id"])
                    latest_objects[object_id] = item
                    changed = previous_status.get(object_id) != item["status"]
                    previous_status[object_id] = item["status"]
                    if not item.get("visible") and not changed and not events[object_id]:
                        continue
                    handled.add(object_id)
                    fused = item.get("fused_T_map_obj")
                    observed = item.get("observed_T_map_obj")
                    self.object_samples[object_id].append({
                        "frame_seq": seq, "timestamp_s": row["timestamp_s"],
                        "status": item["status"], "visible": item.get("visible", False),
                        "observation_count": item.get("observation_count"),
                        "fused_xyz": None if fused is None else [fused[i][3] for i in range(3)],
                        "observed_xyz": (None if observed is None
                                         else [observed[i][3] for i in range(3)]),
                        "residual_translation_mm": item.get("residual_translation_mm"),
                        "residual_rotation_deg": item.get("residual_rotation_deg"),
                        "events": events[object_id],
                    })
                for object_id, object_events in events.items():
                    if not object_events or object_id in handled or object_id not in latest_objects:
                        continue
                    item = latest_objects[object_id]
                    fused = item.get("fused_T_map_obj")
                    status = object_events[-1]["to"]
                    previous_status[object_id] = status
                    self.object_samples[object_id].append({
                        "frame_seq": seq, "timestamp_s": row["timestamp_s"],
                        "status": status, "visible": False,
                        "observation_count": item.get("observation_count"),
                        "fused_xyz": None if fused is None else [fused[i][3] for i in range(3)],
                        "observed_xyz": None, "residual_translation_mm": None,
                        "residual_rotation_deg": None, "events": object_events,
                    })

        if not self.frame_offsets:
            raise ValueError("timeline must contain at least one frame")
        if len(self.frame_offsets) != int(explorer.get("frame_count", -1)):
            raise ValueError("timeline frame count does not match object_map.json")
        self.objects = {
            int(item["object_id"]): item for item in self.object_map.get("objects", ())
        }

    def meta(self):
        objects = [{key: value for key, value in item.items() if key != "pose_history"}
                   for item in self.objects.values()]
        return {
            "dataset": self.object_map.get("dataset"),
            "map_id": self.object_map.get("map_id"),
            "formula": self.object_map.get("formula"),
            "corrected_pose": self.object_map.get("corrected_pose"),
            "sam_inference": self.object_map.get("sam_inference"),
            "persistent_stats": self.object_map.get("persistent_stats"),
            "explorer": self.object_map["explorer"],
            "objects": objects,
        }

    def frame(self, seq):
        if not 0 <= seq < len(self.frame_offsets):
            raise FileNotFoundError(f"frame {seq} does not exist")
        with self.timeline.open("rb") as stream:
            stream.seek(self.frame_offsets[seq])
            return json.loads(stream.readline())

    def object(self, object_id):
        if object_id not in self.objects:
            raise FileNotFoundError(f"object {object_id} does not exist")
        item = self.objects[object_id]
        return {"object_id": object_id, "object_name": item["object_name"],
                "final_status": item["status"], "samples": self.object_samples[object_id]}


class ExplorerServer(ThreadingHTTPServer):
    def __init__(self, address, data):
        self.data = data
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    server: ExplorerServer

    def do_GET(self):
        try:
            host = self.headers.get("Host", "")
            if urlsplit("//" + host).hostname not in {"127.0.0.1", "localhost", "::1"}:
                return self._json({"error": "localhost Host required"}, HTTPStatus.FORBIDDEN)
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).hostname not in {"127.0.0.1", "localhost", "::1"}:
                return self._json({"error": "cross-origin request refused"}, HTTPStatus.FORBIDDEN)
            self._get()
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except FileNotFoundError as exc:
            self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)

    def _get(self):
        parsed = urlsplit(self.path)
        path = unquote(parsed.path)
        query = parse_qs(parsed.query)
        if path == "/api/meta":
            return self._json(self.server.data.meta())
        if path == "/api/frame":
            return self._json(self.server.data.frame(int(query.get("seq", ["-1"])[0])))
        if path == "/api/object":
            return self._json(self.server.data.object(int(query.get("id", ["-1"])[0])))
        if path == "/preview":
            return self._preview()
        assets = {"/": "index.html", "/index.html": "index.html",
                  "/app.js": "app.js", "/app.css": "app.css"}
        if path not in assets:
            raise FileNotFoundError(path)
        return self._file(STATIC / assets[path])

    def _json(self, payload, status=HTTPStatus.OK):
        blob = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(blob)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(blob)

    def _file(self, path):
        path = Path(path)
        if path.is_symlink() or not path.is_file() or path.parent.resolve() != STATIC.resolve():
            raise FileNotFoundError(path)
        blob = path.read_bytes()
        mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(blob)))
        self.send_header("X-Content-Type-Options", "nosniff")
        if mime == "text/html":
            self.send_header("Content-Security-Policy",
                             "default-src 'self'; script-src 'self'; style-src 'self'; "
                             "connect-src 'self'; media-src 'self'")
        self.end_headers()
        self.wfile.write(blob)

    def _preview(self):
        path = self.server.data.preview
        size = path.stat().st_size
        start, end, status = 0, size - 1, HTTPStatus.OK
        requested = self.headers.get("Range")
        if requested:
            try:
                unit, requested_range = requested.split("=", 1)
                if unit.lower() != "bytes" or "," in requested_range:
                    raise ValueError
                first, last = requested_range.split("-", 1)
                if first:
                    start = int(first)
                    end = min(int(last), size - 1) if last else size - 1
                else:
                    length = int(last)
                    if length <= 0:
                        raise ValueError
                    start = max(0, size - length)
                if start < 0 or start > end or start >= size:
                    raise ValueError
            except (ValueError, TypeError):
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header("Content-Range", f"bytes */{size}")
                self.end_headers()
                return
            status = HTTPStatus.PARTIAL_CONTENT
        length = end - start + 1
        self.send_response(status)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(length))
        if status == HTTPStatus.PARTIAL_CONTENT:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        with path.open("rb") as stream:
            stream.seek(start)
            remaining = length
            while remaining:
                chunk = stream.read(min(1 << 20, remaining))
                if not chunk:
                    break
                try:
                    self.wfile.write(chunk)
                except (BrokenPipeError, ConnectionResetError):
                    break
                remaining -= len(chunk)

    def log_message(self, fmt, *args):
        sys.stderr.write("[object-memory-explorer] " + fmt % args + "\n")


def self_test():
    from urllib.error import HTTPError
    from urllib.request import Request, urlopen

    with tempfile.TemporaryDirectory(dir=SAM_OUTPUT) as directory:
        root = Path(directory) / "dataset"
        fused = root / "fused"
        fused.mkdir(parents=True)
        preview = Path(directory) / "preview.mp4"
        preview.write_bytes(b"0123456789")
        meta = {
            "dataset": "test", "map_id": "test", "objects": [
                {"object_id": 7, "object_name": "cup", "status": "active"}],
            "corrected_pose": {"formula": "inverse camera times fused object",
                               "minimum_status": "active"},
            "explorer": {"schema_version": 1, "timeline": "object_memory_timeline.jsonl",
                         "frame_count": 2, "preview": {"path": str(preview)}},
        }
        (fused / "object_map.json").write_text(json.dumps(meta), encoding="utf-8")
        rows = [
            {"frame_seq": 0, "stamp_ns": "1", "timestamp_s": 0, "camera": None,
             "objects": [], "transitions": [], "predictions": [{
                 "object_id": 7, "object_name": "cup", "status": "active",
                 "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t_mm": [1, 2, 3]}]},
            {"frame_seq": 1, "stamp_ns": "2", "timestamp_s": 0.1, "camera": None,
             "detections": [{"detection_id": 4, "corrected_pose": {
                 "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t_mm": [1, 2, 3]}}],
             "objects": [{"object_id": 7, "object_name": "cup", "status": "active",
                           "visible": True, "observation_count": 2,
                           "fused_T_map_obj": [[1, 0, 0, 1], [0, 1, 0, 2],
                                               [0, 0, 1, 3], [0, 0, 0, 1]],
                           "observed_T_map_obj": None,
                           "residual_translation_mm": 2,
                           "residual_rotation_deg": 3}], "transitions": []},
        ]
        (fused / "object_memory_timeline.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
        assert select_dataset("dataset", Path(directory)) == root
        assert select_dataset(output_root=Path(directory)) == root
        data = ExplorerData(root)
        assert data.frame(1)["stamp_ns"] == "2" and data.object(7)["samples"]
        server = ExplorerServer(("127.0.0.1", 0), data)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            assert json.load(urlopen(base + "/api/meta"))["dataset"] == "test"
            frame = json.load(urlopen(base + "/api/frame?seq=1"))
            assert frame["frame_seq"] == 1
            assert frame["detections"][0]["corrected_pose"]["t_mm"] == [1, 2, 3]
            assert json.load(urlopen(base + "/api/frame?seq=0"))["predictions"][0][
                "object_id"] == 7
            for url in ("/api/frame?seq=9", "/api/object?id=9", "/%2e%2e/secret"):
                try:
                    urlopen(base + url)
                    raise AssertionError(f"expected 404: {url}")
                except HTTPError as exc:
                    assert exc.code == 404
            response = urlopen(Request(base + "/preview", headers={"Range": "bytes=2-5"}))
            assert response.status == 206 and response.read() == b"2345"
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
    print("self-test: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", help="legacy result name under output/object_memory")
    parser.add_argument("--result-dir", type=Path,
                        help="direct object_memory result directory containing fused/")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        parser.error("ObjectMemory Explorer is localhost-only")
    try:
        if args.result_dir and args.dataset:
            parser.error("use either --result-dir or --dataset")
        dataset_dir = (args.result_dir.expanduser().resolve() if args.result_dir else
                       select_dataset(args.dataset))
        server = ExplorerServer((args.host, args.port), ExplorerData(dataset_dir))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(f"ObjectMemory Explorer [{dataset_dir.name}]: http://{args.host}:{args.port}",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
