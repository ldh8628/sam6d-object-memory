"""On-demand single-candidate replay. All caches are process RAM only."""
from __future__ import annotations

import base64
import hashlib
import importlib
import os
import sqlite3
import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml


class _PointAsset:
    def __init__(self, points):
        self._pts = points


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _asset_provenance_mismatches(provenance, bag_path=None):
    """Hash recorded model assets and the validated source bag before replay."""
    entries = []
    for key in ("config", "checkpoint"):
        if isinstance(provenance.get(key), dict):
            entries.append((key, provenance[key]))
    for group in ("templates", "cad"):
        for name, entry in (provenance.get(group) or {}).items():
            entries.append((f"{group}.{name}", entry))
    mismatches = []
    for label, entry in entries:
        path = Path(entry.get("path", ""))
        if (not path.is_file() or path.stat().st_size != int(entry.get("size", -1)) or
                (entry.get("sha256") and _sha256(path) != entry["sha256"])):
            mismatches.append(label)
    bag_entry = provenance.get("bag") or {}
    if bag_path is not None and isinstance(bag_entry, dict):
        bag_path = Path(bag_path).resolve()
        recorded = {item.get("name"): item for item in bag_entry.get("files", [])}
        for name, entry in recorded.items():
            path = bag_path / str(name)
            if (not path.is_file() or path.is_symlink() or
                    path.stat().st_size != int(entry.get("size", -1)) or
                    (entry.get("sha256") and _sha256(path) != entry["sha256"])):
                mismatches.append(f"bag.{name}")
    return mismatches


def _strided_texture_features(points, features, stride):
    stride = max(1, int(stride))
    return points[::stride], features[::stride]


class PemReplayCore:
    """Load PEM/checkpoint/assets only; never initialize YOLO, DINO or SAM."""

    def __init__(self, repo, cfg):
        repo = Path(repo)
        pem_dir = repo / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
        realtime = repo / "realtime"
        for path in (repo, realtime, pem_dir, pem_dir / "provider", pem_dir / "utils",
                     pem_dir / "model", pem_dir / "model" / "pointnet2"):
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))
        import gorilla
        import run_inference_custom as ric
        import verify_config as VC

        rt = cfg.get("runtime", {})
        self.device = (str(rt.get("device", "cuda:0"))
                       if torch.cuda.is_available() else "cpu")
        appe_cfg, self.verify = VC.build_appe_cfg(rt)
        self.texture_stride = max(1, int((appe_cfg or {}).get("stride", 1)))
        previous = Path.cwd()
        try:
            os.chdir(pem_dir)
            pcfg = gorilla.Config.fromfile(str(pem_dir / "config" / "base.yaml"))
            pcfg.model_name = "pose_estimation_model"
            if appe_cfg:
                pcfg.model.appe_rerank = appe_cfg
            model_module = importlib.import_module(pcfg.model_name)
            self.pem = model_module.Net(pcfg.model).to(self.device).eval()
            gorilla.solver.load_checkpoint(
                model=self.pem,
                filename=str(pem_dir / "checkpoints" / "sam-6d-pem-base.pth"))
        finally:
            os.chdir(previous)
        self.ric = ric

        ism_path = Path(cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml"))
        if not ism_path.is_absolute():
            ism_path = repo / ism_path
        if (ism_path.is_symlink() or not ism_path.is_file() or
                repo.resolve() not in ism_path.resolve().parents):
            raise PermissionError("ISM config must be a real file under the repository")
        ism_path = ism_path.resolve()
        ism = yaml.safe_load(ism_path.read_text(encoding="utf-8")) or {}
        requested = set(cfg.get("ism", {}).get("objects") or [])
        names = [item["name"] for item in ism.get("objects", [])
                 if item.get("enabled", True) and (not requested or item["name"] in requested)]
        self._pts, self._tem = {}, {}
        for name in names:
            points = np.load(repo / "assets" / "model_points" / f"{name}.npy").astype(
                np.float32, copy=False)
            blob = torch.load(repo / "assets" / "pem_templates" / f"{name}.pt",
                              map_location=self.device)
            self._pts[name] = _PointAsset(points)
            self._tem[name] = (blob["tp"].to(self.device), blob["tf"].to(self.device),
                               None if blob.get("tc") is None else
                               blob["tc"].to(self.device).float())
        # Pay CUDA kernel/module initialization when the lazy analyzer is created.
        # This tensor is discarded immediately and never persisted.
        if names:
            warm_name = names[0]
            warm = {
                "rgb": torch.zeros(1, 3, 224, 224, device=self.device),
                "rgb_choose": torch.zeros(1, 2048, dtype=torch.long,
                                          device=self.device),
                "pts": torch.zeros(1, 2048, 3, device=self.device),
                "dense_po": self._tem[warm_name][0],
                "dense_fo": self._tem[warm_name][1],
            }
            with torch.inference_mode():
                self.pem.feature_extraction(warm)
            if self.device.startswith("cuda"):
                torch.cuda.synchronize(torch.device(self.device))


class BagFrameCache:
    def __init__(self, bag_path, topics, capacity=8):
        self.bag_path = str(bag_path)
        self.topics = topics
        self.capacity = capacity
        self.cache = OrderedDict()
        bag_path = Path(bag_path)
        databases = sorted(bag_path.glob("*.db3")) if bag_path.is_dir() else [bag_path]
        if len(databases) != 1:
            raise ValueError(f"expected one sqlite bag database in {bag_path}")
        self.connection = sqlite3.connect(
            f"file:{databases[0].resolve().as_posix()}?mode=ro", uri=True,
            check_same_thread=False)
        self.lock = threading.Lock()
        # ROS type support imports are expensive on first use; make the server's
        # advertised READY state mean that replay decoding is ready as well.
        from cv_bridge import CvBridge
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        self.bridge = CvBridge()
        self.deserialize_message = deserialize_message
        topic_rows = self.connection.execute("SELECT id,name,type FROM topics").fetchall()
        self.topic_info = {name: (int(topic_id), msgtype)
                           for topic_id, name, msgtype in topic_rows}
        missing = sorted(set(topics.values()) - set(self.topic_info))
        if missing:
            raise ValueError(f"bag topics missing: {missing}")
        self.message_classes = {
            name: get_message(self.topic_info[name][1]) for name in topics.values()
        }

    def _decode(self, key, header_stamp):
        with self.lock:
            topic = self.topics[key]
            topic_id, _ = self.topic_info[topic]
            # Standard bags use the header stamp as the storage timestamp. Keep a
            # bounded fallback for valid bags whose recorder used arrival time.
            rows = self.connection.execute(
                "SELECT data FROM messages WHERE topic_id=? AND timestamp=? LIMIT 1",
                (topic_id, int(header_stamp))).fetchall()
            if not rows:
                rows = self.connection.execute(
                    "SELECT data FROM messages WHERE topic_id=? "
                    "AND timestamp BETWEEN ? AND ? ORDER BY ABS(timestamp-?) LIMIT 16",
                    (topic_id, int(header_stamp) - 1_000_000_000,
                     int(header_stamp) + 1_000_000_000, int(header_stamp))).fetchall()
            msg = None
            message_class = self.message_classes[topic]
            for (blob,) in rows:
                candidate = self.deserialize_message(blob, message_class)
                candidate_stamp = (int(candidate.header.stamp.sec) * 1_000_000_000
                                   + int(candidate.header.stamp.nanosec))
                if candidate_stamp == int(header_stamp):
                    msg = candidate
                    break
            if msg is None:
                raise LookupError(f"{topic} header stamp {header_stamp} not found")
            return np.asarray(self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="rgb8" if key == "rgb" else "passthrough"))

    def get_rgb(self, rgb_stamp_ns):
        rgb_stamp_ns = int(rgb_stamp_ns)
        for stamps, value in list(self.cache.items()):
            if stamps[0] == rgb_stamp_ns:
                self.cache.move_to_end(stamps)
                return value[0]
        return self._decode("rgb", rgb_stamp_ns)

    def get(self, rgb_stamp_ns, depth_stamp_ns):
        stamps = (int(rgb_stamp_ns), int(depth_stamp_ns))
        if stamps in self.cache:
            self.cache.move_to_end(stamps)
            return self.cache[stamps]
        found = {"rgb": self._decode("rgb", stamps[0]),
                 "depth": self._decode("depth", stamps[1])}
        value = (np.asarray(found["rgb"]), np.asarray(found["depth"]))
        self.cache[stamps] = value
        while len(self.cache) > self.capacity:
            self.cache.popitem(last=False)
        return value


class PemLiveAnalyzer:
    """Preload the shared PEM assets and recompute one selected candidate at a time."""

    def __init__(self, repo, config_path, provenance=None, bag_path=None):
        repo = Path(repo)
        self.repo = repo
        self.config_path = Path(config_path)
        cfg = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}
        rt = cfg.get("runtime", {}); topics = cfg["topics"]
        bag = Path(bag_path) if bag_path is not None else Path(
            (cfg.get("output", {}).get("pem_explorer", {}) or {}).get(
                "source_bag") or cfg.get("bag", {}).get("path", ""))
        if not bag.is_absolute(): bag = repo / bag
        self.provenance_mismatches = _asset_provenance_mismatches(
            provenance or {}, bag)
        self.core = PemReplayCore(repo, cfg)
        self.frames = BagFrameCache(bag, topics)
        self.features = OrderedDict()
        self.feature_capacity = 8

    @staticmethod
    def _bitmask(mask):
        packed = np.packbits(np.asarray(mask, dtype=np.uint8).reshape(-1), bitorder="little")
        return base64.b64encode(packed.tobytes()).decode("ascii")

    def analyze(self, run_dir, row, candidate, replay):
        started = time.perf_counter()
        rgb, depth = self.frames.get(
            row.get("rgb_stamp_ns", row["stamp_ns"]),
            row.get("depth_stamp_ns", row["stamp_ns"]))
        K = np.asarray(row["K"], np.float32).reshape(3, 3)
        y1, y2, x1, x2 = [int(round(v)) for v in row["crop_bbox_yxyx"]]
        source = np.asarray(replay["source_pixel_index"], np.int64)
        run_dir = Path(run_dir).resolve()
        mask_relative = Path(row["mask_asset"] or "")
        if mask_relative.is_absolute() or ".." in mask_relative.parts:
            raise PermissionError("unsafe mask asset path")
        mask_path = run_dir / mask_relative
        resolved_mask = mask_path.resolve(strict=False)
        if (mask_path.is_symlink() or not mask_path.is_file() or
                run_dir not in resolved_mask.parents):
            raise PermissionError("mask asset escapes the run")
        mask_image = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        if mask_image is None:
            raise FileNotFoundError(row["mask_asset"])
        if row.get("mask_bit") is not None:
            observed_full = (mask_image.astype(np.uint16) & int(row["mask_bit"])) != 0
        else:
            observed_full = mask_image == int(row["mask_label"])
        observed_crop = observed_full[y1:y2, x1:x2]
        frame_width = rgb.shape[1]
        source_y, source_x = source // frame_width, source % frame_width
        crop_source = (source_y - y1) * (x2 - x1) + (source_x - x1)
        if (np.any(source_y < y1) or np.any(source_y >= y2) or
                np.any(source_x < x1) or np.any(source_x >= x2)):
            raise ValueError("replay source pixel lies outside recorded crop")
        rgb_choose = self.core.ric.get_resize_rgb_choose(
            crop_source, [y1, y2, x1, x2], 224)
        name = row["object"]
        cad_indices = np.asarray(replay["cad_sample_index"], np.int64)
        model = self.core._pts[name]._pts[cad_indices].astype(np.float32) / 1000.0
        feature_key = (int(row["stamp_ns"]), name,
                       int(row.get("mask_bit", row["mask_label"])))
        cached = self.features.get(feature_key)
        feature_cache_hit = cached is not None
        if cached is None:
            # Reconstruct only the exact stored observations. Building the whole
            # 640x480 point cloud costs ~26 ms and provides no additional evidence.
            z = depth.reshape(-1)[source].astype(np.float32) / np.float32(1000.0)
            cloud = np.stack(((source_x.astype(np.float32) - K[0, 2]) * z / K[0, 0],
                              (source_y.astype(np.float32) - K[1, 2]) * z / K[1, 1],
                              z), axis=1).astype(np.float32, copy=False)
            crop_rgb = rgb[y1:y2, x1:x2, :][:, :, ::-1].copy()
            crop_rgb *= observed_crop[:, :, None].astype(np.uint8)
            crop_rgb = cv2.resize(crop_rgb, (224, 224), interpolation=cv2.INTER_LINEAR)
            tensor_rgb = self.core.ric.rgb_transform(crop_rgb)[None].to(self.core.device)
            inp = {
                "rgb": tensor_rgb,
                "rgb_choose": torch.as_tensor(
                    rgb_choose, device=self.core.device)[None].long(),
                "pts": torch.as_tensor(cloud, device=self.core.device)[None].float(),
                "dense_po": self.core._tem[name][0], "dense_fo": self.core._tem[name][1],
            }
            with torch.inference_mode():
                cached = self.core.pem.feature_extraction(inp)
            self.features[feature_key] = cached
            while len(self.features) > self.feature_capacity:
                self.features.popitem(last=False)
        else:
            self.features.move_to_end(feature_key)
        pm, fm, po, fo, radius = cached
        with torch.inference_mode():
            R = torch.as_tensor(candidate["R"], device=self.core.device).float()
            t = torch.as_tensor(candidate["t_m"], device=self.core.device).float() / radius[0]
            fps = torch.as_tensor(replay["coarse_fps_index"], device=self.core.device).long()
            valid = torch.as_tensor(replay["coarse_valid"], device=self.core.device).bool()
            transformed_geo = (pm[0, fps] - t) @ R
            geometry_model = torch.as_tensor(
                model, device=self.core.device).float() / radius[0]
            geo_distance = torch.cdist(
                transformed_geo[None], geometry_model[None]).squeeze(0).min(1).values
            geometry_score = valid.sum() / (geo_distance[valid].sum() + 1e-8)
            texture_pm, texture_fm = _strided_texture_features(
                pm[0], fm[0], self.core.texture_stride)
            transformed = (texture_pm - t) @ R
            nearest = torch.cdist(transformed, po[0]).argmin(1)
            similarity = torch.einsum(
                "nd,nd->n", torch.nn.functional.normalize(texture_fm, dim=1),
                torch.nn.functional.normalize(fo[0][nearest], dim=1))

        cad = self.core._pts[name]._pts.astype(np.float32) / 1000.0
        cam = cad @ np.asarray(candidate["R"], np.float32).T + np.asarray(
            candidate["t_m"], np.float32)
        z = cam[:, 2]; valid_z = np.isfinite(z) & (z > 1e-6)
        u = np.rint(K[0, 0] * cam[:, 0] / np.maximum(z, 1e-6) + K[0, 2]).astype(int)
        v = np.rint(K[1, 1] * cam[:, 1] / np.maximum(z, 1e-6) + K[1, 2]).astype(int)
        inside = valid_z & (u >= 0) & (u < rgb.shape[1]) & (v >= 0) & (v < rgb.shape[0])
        projected = np.zeros(observed_full.shape, np.uint8); projected[v[inside], u[inside]] = 1
        projected = cv2.dilate(projected, np.ones((5, 5), np.uint8)) > 0
        # Score mismatch uses the exact production 224-crop adaptive-splat routine;
        # the full-frame bitmask above is visualization-only.
        from model_utils import _candidate_shape_metrics
        shape_224 = cv2.resize(observed_crop.astype(np.uint8), (224, 224),
                               interpolation=cv2.INTER_NEAREST) > 0
        with torch.inference_mode():
            metrics = _candidate_shape_metrics(
                R.reshape(1, 1, 3, 3), t.reshape(1, 1, 3),
                torch.as_tensor(cad, device=self.core.device)[None].float() / radius[0],
                {"shape_mask": torch.as_tensor(shape_224, device=self.core.device)[None],
                 "K": torch.as_tensor(K, device=self.core.device)[None],
                 "crop_bbox_yxyx": torch.tensor([[y1, y2, x1, x2]],
                                                  device=self.core.device).float(),
                 "verify": self.core.verify})
        mask_iou = float(metrics["mask_iou"][0, 0].cpu())
        elapsed = (time.perf_counter() - started) * 1000.0
        recomputed = {"geometry": float(geometry_score.cpu()),
                      "texture": float(similarity.mean().cpu()), "mask_iou": mask_iou}
        stored = {key: float(candidate[key]) for key in recomputed}
        mismatch = {key: abs(recomputed[key] - stored[key]) for key in recomputed}
        return {
            "attempt_id": row["attempt_id"], "candidate_index300": candidate["index300"],
            "geometry_distance": geo_distance.detach().cpu().float().tolist(),
            "geometry_valid": np.asarray(replay["coarse_valid"], bool).tolist(),
            "texture_similarity": similarity.detach().cpu().float().tolist(),
            "texture_template_index": nearest.detach().cpu().int().tolist(),
            "mask": {"size": list(observed_full.shape),
                     "observed_bits": self._bitmask(observed_full),
                     "projected_bits": self._bitmask(projected)},
            "stored": stored, "recomputed": recomputed, "absolute_error": mismatch,
            "provenance_mismatch": (bool(self.provenance_mismatches) or
                                    any(value > 5e-3 for value in mismatch.values())),
            "provenance_mismatch_assets": self.provenance_mismatches,
            "feature_cache_hit": feature_cache_hit,
            "timing_ms": elapsed,
        }
