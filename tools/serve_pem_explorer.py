#!/usr/bin/env python3
"""Localhost-only PEM Explorer v2 server with fail-closed path handling."""
from __future__ import annotations

import argparse
import json
import math
import mimetypes
import statistics
import sys
import threading
from collections import defaultdict, deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PurePosixPath
from urllib.parse import parse_qs, unquote, urlsplit

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "realtime"))
from pem_explorer_record import CANDIDATE_BYTES, SCHEMA, read_attempt  # noqa: E402

STATIC = Path(__file__).resolve().parent / "pem_explorer_live"


def json_safe(value):
    if hasattr(value, "tolist"):
        return json_safe(value.tolist())
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def safe_run(root, name):
    if not name or name in {".", ".."} or "/" in name or "\\" in name:
        raise ValueError("run must be one direct child of output root")
    root = Path(root).resolve()
    candidate = root / name
    if candidate.is_symlink():
        raise PermissionError("symlink runs are not served")
    resolved = candidate.resolve(strict=False)
    if resolved.parent != root:
        raise PermissionError("run escapes output root")
    return resolved


def classify_run(path):
    path = Path(path)
    manifest_path = path / "explorer_manifest.json"
    if manifest_path.is_file() and not manifest_path.is_symlink():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            return {"kind": "incomplete", "reason": f"invalid manifest: {exc}"}
        if (manifest.get("schema") != SCHEMA or manifest.get("schema_version") != 2 or
                manifest.get("candidate_record_bytes") != CANDIDATE_BYTES):
            return {"kind": "incomplete", "reason": "unsupported explorer schema"}
        required = ("explorer_index.jsonl", "candidates.bin", "replay.bin")
        if (not manifest.get("completed") or
                any(not (path / name).is_file() or (path / name).is_symlink()
                    for name in required)):
            return {"kind": "incomplete",
                    "reason": "run is not finalized; rerun the capture from the start",
                    "manifest": manifest}
        try:
            expected_candidates = int(manifest.get("candidate_count", -1)) * CANDIDATE_BYTES
        except (TypeError, ValueError):
            return {"kind": "incomplete", "reason": "invalid candidate count",
                    "manifest": manifest}
        if (expected_candidates < 0 or
                (path / "candidates.bin").stat().st_size != expected_candidates):
            return {"kind": "incomplete", "reason": "candidate binary size mismatch",
                    "manifest": manifest}
        return {"kind": "explorer_v2", "manifest": manifest}
    if (path / "detections.jsonl").is_file():
        return {"kind": "partial", "reason": "300개 후보 정보 미수집"}
    if (path / "index.html").is_file():
        return {"kind": "legacy", "url": f"/legacy/{path.name}/index.html"}
    return {"kind": "unknown", "reason": "recognized result files not found"}


def discover_runs(root):
    root = Path(root).resolve()
    if not root.is_dir():
        return []
    rows = []
    for path in sorted(root.iterdir(), key=lambda p: p.name.lower()):
        if path.is_symlink() or not path.is_dir():
            continue
        state = classify_run(path)
        if state["kind"] != "unknown":
            rows.append({"name": path.name, **state})
    return rows


def load_index(run_dir):
    rows = []
    with open(Path(run_dir) / "explorer_index.jsonl", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                row = json.loads(line)
                if int(row.get("attempt_id", -1)) != len(rows):
                    raise ValueError("explorer index attempt ids must be contiguous")
                rows.append(row)
    return rows


class ExplorerServer(ThreadingHTTPServer):
    def __init__(self, address, root, no_model=False):
        raw_root = Path(root)
        if raw_root.is_symlink() or not raw_root.is_dir():
            raise ValueError("--root must be a real output directory, not a symlink")
        self.root = raw_root.resolve()
        self.no_model = no_model
        self.index_cache = {}
        self.analyzers = {}
        self.analyzer_errors = {}
        self.analysis_lock = threading.Lock()
        self.timings = defaultdict(lambda: deque(maxlen=200))
        super().__init__(address, Handler)
        if not no_model:
            complete = next((row for row in discover_runs(self.root)
                             if row["kind"] == "explorer_v2"), None)
            if complete:
                config = complete["manifest"].get("provenance", {}).get("config", {}).get("path")
                if config:
                    try:
                        self.analyzers[complete["name"]] = self._build_analyzer(
                            complete["name"], complete)
                    except Exception as exc:
                        self.analyzer_errors[complete["name"]] = (
                            f"{type(exc).__name__}: {exc}")

    def rows(self, run):
        if run not in self.index_cache:
            self.index_cache[run] = load_index(safe_run(self.root, run))
        return self.index_cache[run]

    def _build_analyzer(self, run, state):
        from pem_explorer_live.analyzer import PemLiveAnalyzer
        config = state.get("manifest", {}).get("provenance", {}).get(
            "config", {}).get("path")
        analyzer = PemLiveAnalyzer(
            REPO, config, state.get("manifest", {}).get("provenance"))
        # Initialize the complete replay/geometry/texture/mask kernel path before the
        # server announces READY. Discard the feature so the first user click remains
        # an honest cold-object measurement rather than a hidden cache hit.
        row = next((item for item in self.rows(run)
                    if int(item.get("candidate_count", 0)) > 0), None)
        if row is not None:
            candidates, replay = read_attempt(safe_run(self.root, run), row)
            analyzer.analyze(safe_run(self.root, run), row, candidates[0], replay)
            analyzer.features.clear()
        return analyzer

    def analyzer_for(self, run, state):
        if self.no_model:
            return None
        if run in self.analyzer_errors:
            return None
        if run not in self.analyzers:
            config = state.get("manifest", {}).get("provenance", {}).get(
                "config", {}).get("path")
            if not config:
                return None
            # Keep at most one CUDA PEM resident. The request lock serializes this
            # replacement with all in-flight analysis.
            self.analyzers.clear()
            try:
                self.analyzers[run] = self._build_analyzer(run, state)
            except Exception as exc:
                self.analyzer_errors[run] = f"{type(exc).__name__}: {exc}"
                return None
        return self.analyzers[run]


class Handler(BaseHTTPRequestHandler):
    server: ExplorerServer

    def _json(self, payload, status=200):
        blob = json.dumps(json_safe(payload), ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status); self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(blob))); self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(blob)

    def _error(self, status, message):
        self._json({"error": message}, status)

    def do_GET(self):
        host = self.headers.get("Host", "")
        hostname = urlsplit("//" + host).hostname
        if hostname not in {"127.0.0.1", "localhost", "::1"}:
            return self._error(HTTPStatus.FORBIDDEN, "localhost Host required")
        origin = self.headers.get("Origin")
        if origin and urlsplit(origin).hostname not in {"127.0.0.1", "localhost", "::1"}:
            return self._error(HTTPStatus.FORBIDDEN, "cross-origin request refused")
        try:
            self._get()
        except ValueError as exc: self._error(HTTPStatus.BAD_REQUEST, str(exc))
        except PermissionError as exc: self._error(HTTPStatus.FORBIDDEN, str(exc))
        except FileNotFoundError as exc: self._error(HTTPStatus.NOT_FOUND, str(exc))
        except (IndexError, KeyError) as exc: self._error(HTTPStatus.BAD_REQUEST, str(exc))
        except Exception as exc: self._error(HTTPStatus.INTERNAL_SERVER_ERROR,
                                             f"{type(exc).__name__}: {exc}")

    def _get(self):
        parsed = urlsplit(self.path); path = unquote(parsed.path)
        if path == "/api/runs":
            runs = discover_runs(self.server.root)
            return self._json({"runs": runs,
                               "analysis_enabled": (not self.server.no_model and
                                                    any(
                                                        row["kind"] == "explorer_v2" and
                                                        row["name"] not in
                                                        self.server.analyzer_errors and
                                                        bool(row.get("manifest", {}).get(
                                                            "provenance", {}).get(
                                                                "config", {}).get("path"))
                                                        for row in runs))})
        if path.startswith("/api/run/"):
            parts = PurePosixPath(path).parts
            if len(parts) < 4: raise ValueError("missing run")
            run = parts[3]; run_dir = safe_run(self.server.root, run)
            state = classify_run(run_dir)
            action = parts[4] if len(parts) > 4 else "summary"
            if action == "summary":
                payload = {"name": run, **state}
                if state["kind"] == "explorer_v2": payload["attempts"] = self.server.rows(run)
                elif state["kind"] == "partial":
                    with open(run_dir / "detections.jsonl", encoding="utf-8") as stream:
                        payload["detections"] = [json.loads(line) for line in stream if line.strip()]
                return self._json(payload)
            if state["kind"] != "explorer_v2":
                return self._error(HTTPStatus.CONFLICT, state.get("reason", "analysis unavailable"))
            query = parse_qs(parsed.query); attempt_id = int(query.get("attempt", ["-1"])[0])
            row = self.server.rows(run)[attempt_id]
            if int(row["attempt_id"]) != attempt_id: raise ValueError("attempt id mismatch")
            candidates, replay = read_attempt(run_dir, row)
            if action == "candidates":
                return self._json({"attempt": row, "candidates": candidates})
            if action == "analysis":
                index = int(query.get("candidate", ["-1"])[0])
                candidate = next(c for c in candidates if int(c["index300"]) == index)
                with self.server.analysis_lock:
                    analyzer = self.server.analyzer_for(run, state)
                    if analyzer is None:
                        detail = self.server.analyzer_errors.get(
                            run, "dynamic analysis disabled (--no-model or invalid provenance)")
                        return self._error(HTTPStatus.SERVICE_UNAVAILABLE, detail)
                    result = analyzer.analyze(run_dir, row, candidate, replay)
                category = "warm" if result.get("feature_cache_hit") else "cold"
                self.server.timings[(row["object"], category)].append(result["timing_ms"])
                summaries = {}
                for kind in ("cold", "warm"):
                    ordered = sorted(self.server.timings[(row["object"], kind)])
                    if ordered:
                        summaries[kind] = {
                            "samples": len(ordered), "p50_ms": statistics.median(ordered),
                            "p90_ms": ordered[min(len(ordered) - 1,
                                                   int(0.9 * len(ordered)))],
                        }
                result["timing_summary"] = summaries
                return self._json(result)
            raise ValueError("unknown API action")
        if path.startswith("/legacy/"):
            parts = PurePosixPath(path).parts
            if len(parts) < 4: raise ValueError("legacy asset missing")
            run_dir = safe_run(self.server.root, parts[2])
            relative = Path(*parts[3:])
            target = run_dir / relative
            if target.is_symlink() or not target.is_file() or target.resolve().parent != target.parent.resolve():
                raise PermissionError("unsafe legacy asset")
            if run_dir.resolve() not in target.resolve().parents: raise PermissionError("legacy traversal")
            return self._file(target)
        if path in {"/", "/index.html"}: return self._file(STATIC / "index.html")
        if path in {"/app.js", "/app.css", "/worker.js"}:
            return self._file(STATIC / path[1:])
        raise FileNotFoundError(path)

    def _file(self, path):
        blob = Path(path).read_bytes(); mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
        self.send_response(200); self.send_header("Content-Type", mime)
        self.send_header("X-Content-Type-Options", "nosniff")
        if mime == "text/html":
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self'; "
                "img-src 'self' data:; connect-src 'self'; worker-src 'self'")
        self.send_header("Content-Length", str(len(blob))); self.end_headers(); self.wfile.write(blob)

    def log_message(self, fmt, *args):
        sys.stderr.write("[pem-explorer] " + fmt % args + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default="output")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-model", action="store_true")
    args = parser.parse_args()
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        parser.error("PEM Explorer is localhost-only")
    server = ExplorerServer((args.host, args.port), args.root, args.no_model)
    print(f"PEM Explorer: http://{args.host}:{args.port}", flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()


if __name__ == "__main__": main()
