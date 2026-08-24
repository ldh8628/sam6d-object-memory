#!/usr/bin/env python3
"""sam6d_core.py — ROS 없는 SAM-6D 처리 본체 (ISM + PEM).

`sam6d_realtime_node.py` 의 적재·처리와 **같은 함수**를 같은 순서로 부른다.
차이는 ROS 가 없다는 것뿐이다. 그래서 판정 결과는 노드와 같아야 한다(파리티 확인 도구:
`temp/check_core_parity.py`).

이 파일이 따로 존재하는 이유: 고속 영상 구독과 무거운 추론을 **같은 파이썬 인터프리터**에
두면 GIL 때문에 추론이 초 단위로 굶는다(NOTES 19~21절). 그래서 추론을 ROS 가 전혀 없는
프로세스로 옮기기 위한 알맹이다.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

REPO = str(Path(__file__).resolve().parents[1])
if REPO not in sys.path:
    sys.path.insert(0, REPO)
PEM_DIR = os.path.join(REPO, "sam6d_master", "SAM-6D", "Pose_Estimation_Model")
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import yolo_ism as yi                                           # noqa: E402
import yolo_ism_object_n as o_n                                 # noqa: E402
from verify_config import UNSET, build_appe_cfg, describe        # noqa: E402
from slam_pose_memory import ObjectAnchorManager, pose_matrix    # noqa: E402


def _sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


class Sam6DCore:
    def __init__(self, ism_config, objects=None, device="cuda:0",
                 det_score_thresh=0.2, log=print, appe_rerank=None, verify=UNSET,
                 pem_diagnostic=None, coarse_npoint_override=None):
        self.log = log
        # coarse_npoint 스윕 실험용. base.yaml 값을 대체한다(None 이면 그대로).
        self.coarse_npoint_override = (int(coarse_npoint_override)
                                       if coarse_npoint_override else None)
        self.device = device if torch.cuda.is_available() else "cpu"
        self.det_thresh = float(det_score_thresh)
        # 외형 재정렬 옵션(기본 None = 예전 동작). split 모드도 단일 프로세스 모드와
        # 똑같이 이 옵션을 받게 한다 — 예전에는 sam6d_realtime_node.py 에만 있어서
        # split 실행에서는 config 에 써도 조용히 무시됐다.
        self.appe_rerank = appe_rerank
        # 텍스처 검증(Stage V). 노드와 같은 기본값·같은 병합 규칙을 쓴다.
        rt = {"appe_rerank": appe_rerank}
        if verify is not UNSET:                    # 안 주면 기본값(=검증 ON)
            rt["verify"] = verify
        self.appe_cfg, self.verify = build_appe_cfg(rt)
        self.verify = self.verify or {}
        self.pem_diagnostic = dict(pem_diagnostic or {})
        self.pem_explorer_v2 = bool(
            ((self.pem_diagnostic.get("explorer_v2") or {}).get("enabled")))
        if self.pem_diagnostic.get("enabled"):
            # 진단은 후보 재정렬의 중간값을 계측한다. opt-in일 때만 설정을 복사해
            # 모델에 전달하므로 기본 실행의 연산/선택 순서는 그대로다.
            self.appe_cfg = dict(self.appe_cfg or {"topk": 100, "stride": 1})
            self.appe_cfg["diagnostic"] = dict(self.pem_diagnostic)
        self.last_frame_diag = {}
        self.anchor_manager = None
        self._load_ism(ism_config, objects or [])
        self._load_pem()

    def configure_anchors(self, map_id, config=None):
        """Enable session-only Object Anchors; no state is loaded from disk."""
        self.anchor_manager = ObjectAnchorManager(map_id, config)

    # ------------------------------------------------------------------ 적재
    def _load_ism(self, cfg_path, want):
        cfg_path = cfg_path if os.path.isabs(cfg_path) else os.path.join(REPO, cfg_path)
        defaults, objs = o_n.load_config(cfg_path)
        if want:
            objs = [o for o in objs if o["name"] in want]
            missing = sorted(set(want) - {o["name"] for o in objs})
            if missing:
                raise SystemExit(f"[config] 요청한 객체가 config 에 없다: {missing}")
        self.log(f"[load] DINOv2 + 템플릿 특징 ({len(objs)} 객체) ...")
        self.model = yi.build_dinov2(
            defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, self.device)
        self.objs = o_n.prepare_objects(objs, self.model, self.device, rebuild=False)
        stale = [o["name"] for o in self.objs
                 if o.get("hsv_gate_enabled") and o.get("_hsv_proto") is None]
        if stale:
            raise SystemExit("[hsv] HSV 게이트가 켜져 있는데 기준값 캐시가 무효다: "
                             + ", ".join(stale) +
                             "\n      복구: python tools/build_hsv_template_cache.py --force")
        self.segmentor = yi.build_segmentor(
            o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), self.device)
        self.pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
        self.unique_prompts, self.groups = o_n.build_prompt_groups(self.objs)
        self.tsim = o_n.template_similarity(self.objs)
        self.min_score = min(float(o.get("score_threshold", 0.02)) for o in self.objs)
        self.imgsz = int(defaults.get("imgsz", 640))
        self.multi_label = o_n._multi_label_on(self.objs[0])
        import ultralytics.nn.text_model as _tm
        _tm.WEIGHTS_DIR = Path(os.path.join(REPO, "assets"))
        from ultralytics import YOLOWorld
        weights = o_n._abspath(defaults.get("weights", "yolov8m-worldv2.pt"))
        self.yolo = YOLOWorld(weights)
        self.yolo.set_classes(self.unique_prompts)
        self._warmup()
        self.log(f"[load] YOLO-World imgsz={self.imgsz} multi_label={self.multi_label} "
                 f"프롬프트 {len(self.unique_prompts)}개")
        self.log(f"[ism] relative_assignment={bool(self.objs[0].get('relative_assignment_enabled'))} "
                 f"cross_object_nms={bool(self.objs[0].get('cross_object_nms_enabled'))} "
                 f"hsv_gate={bool(self.objs[0].get('hsv_gate_enabled'))}")

    def _warmup(self, size=(480, 640)):
        try:
            bgr = np.full((size[0], size[1], 3), 127, np.uint8)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            with o_n.multi_label_nms(self.multi_label):
                self.yolo.predict(bgr, conf=self.min_score, imgsz=self.imgsz,
                                  verbose=False, device=self.device)
            pb = {i: [] for i in range(len(self.unique_prompts))}
            pb[0] = [([size[1] // 4, size[0] // 4, size[1] // 2, size[0] // 2], 0.5)]
            o_n.recognize_frame_auto(self.groups, pb, bgr, rgb, yi.normalize_rgb(rgb),
                                     self.model, self.device, self.segmentor, self.pool, self.tsim)
        except Exception as e:
            self.log(f"[load] 예열 실패({type(e).__name__}: {e})")

    def _load_pem(self):
        cwd = os.getcwd()
        try:
            return self._load_pem_impl()
        finally:
            os.chdir(cwd)

    def _load_pem_impl(self):
        cwd = os.getcwd()
        os.chdir(PEM_DIR)
        for sub in ("provider", "utils", "model", os.path.join("model", "pointnet2")):
            sys.path.append(os.path.join(PEM_DIR, sub))
        sys.path.insert(0, PEM_DIR)
        import gorilla, importlib
        import run_inference_custom as ric
        self.ric = ric
        self.pcfg = gorilla.Config.fromfile(os.path.join(PEM_DIR, "config", "base.yaml"))
        self.pcfg.model_name = "pose_estimation_model"
        if self.appe_cfg:
            self.pcfg.model.appe_rerank = self.appe_cfg
        if self.coarse_npoint_override is not None:
            # coarse_npoint 스윕 실험에서만 base.yaml 값을 덮는다.
            # coarse_point_matching 의 attention 은 N 에 독립이라 재빌드 없이 안전하다.
            self.pcfg.model.coarse_npoint = int(self.coarse_npoint_override)
            self.log(f"[pem] coarse_npoint override → {self.pcfg.model.coarse_npoint}")
        self.log(describe(self.appe_cfg, self.verify or None))
        MODEL = importlib.import_module(self.pcfg.model_name)
        self.pem = MODEL.Net(self.pcfg.model).to(self.device).eval()
        gorilla.solver.load_checkpoint(
            model=self.pem, filename=os.path.join(PEM_DIR, "checkpoints", "sam-6d-pem-base.pth"))

        class _Stub:
            def __init__(s, pts):
                s._pts = pts
                s.last_sample_indices = None
            def sample(s, n):
                if n == len(s._pts):
                    s.last_sample_indices = np.arange(len(s._pts), dtype=np.int64)
                    return s._pts
                # Preserve the historical RNG call and expose only its integer indices.
                s.last_sample_indices = np.random.choice(
                    len(s._pts), n, replace=(n > len(s._pts)))
                return s._pts[s.last_sample_indices]

        pts_dir = os.path.join(REPO, "assets", "model_points")
        tem_dir = os.path.join(REPO, "assets", "pem_templates")
        self._pts, self._tem, self._extent = {}, {}, {}
        for o in self.objs:
            nm = o["name"]
            p_pts, p_tem = os.path.join(pts_dir, f"{nm}.npy"), os.path.join(tem_dir, f"{nm}.pt")
            for p in (p_pts, p_tem):
                if not os.path.isfile(p):
                    raise SystemExit(f"[pem] {nm}: 자산이 없다 — {p}")
            pts = np.load(p_pts).astype(np.float32)
            self._pts[nm] = _Stub(pts)
            self._extent[nm] = (pts.max(0) - pts.min(0)) / 1000.0
            blob = torch.load(p_tem, map_location=self.device)
            tc = blob.get("tc")          # 점별 색 (tools/add_template_colors.py 로 추가)
            if tc is None and self.verify.get("enabled") and self.verify.get("w_col"):
                self.log(f"[pem] {nm}: 템플릿에 색(tc)이 없다 — 원색 채널 없이 검증한다. "
                         f"python tools/add_template_colors.py --objects {nm}")
            self._tem[nm] = (blob["tp"].to(self.device), blob["tf"].to(self.device),
                             None if tc is None else tc.to(self.device).float())
        self.ric.trimesh.load_mesh = lambda key, *a, **k: self._pts[key]
        os.chdir(cwd)
        self.log(f"[load] PEM 템플릿·모델점 {len(self._tem)} 객체")

    # ------------------------------------------------------------------ 처리
    def process(self, bgr, depth, K, want_mask=False, diagnostic_references=None,
                slam_context=None):
        """한 프레임 → (검출 목록, 단계별 ms, 마스크 라벨 이미지 or None)."""
        self.last_frame_diag = {"pem_candidates": [], "pem_error": None,
                                "rejections": []}
        _sync(); t0 = time.perf_counter()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = bgr.shape[:2]
        with o_n.multi_label_nms(self.multi_label):
            res = self.yolo.predict(bgr, conf=self.min_score, imgsz=self.imgsz,
                                    verbose=False, device=self.device)
        _sync(); t1 = time.perf_counter()
        pb = {i: [] for i in range(len(self.unique_prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                ci = int(b.cls[j]) if b.cls is not None else 0
                if ci in pb:
                    pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
        for k in pb:
            pb[k].sort(key=lambda t: t[1], reverse=True)
        n_boxes = sum(len(v) for v in pb.values())
        results = o_n.recognize_frame_auto(self.groups, pb, bgr, rgb, yi.normalize_rgb(rgb),
                                           self.model, self.device, self.segmentor,
                                           self.pool, self.tsim)
        _sync(); t2 = time.perf_counter()

        hits = [(n, r) for n, r in sorted(results.items())
                if r.get("accepted") and r.get("mask") is not None]
        rows, lab, shadows = [], None, []
        if hits:
            frame = self.ric.prepare_frame(rgb, depth, K)
            dets = [{"score": 1.0, "mask": r["mask"].astype(bool), "cad": n} for n, r in hits]
            # PEM이 조용히 검출을 버리는 조건을 프레임 산출물에 남긴다. 이전에는 모든
            # 후보가 걸러지면 torch.stack([]) 예외만 나와 어떤 객체/입력이 원인인지 몰랐다.
            for nm, r in hits:
                m = r["mask"].astype(bool)
                valid = m & (depth > 0)
                n_valid = int(valid.sum())
                n_radius = 0
                reason = None
                if n_valid <= 32:
                    reason = "valid_depth_px<=32"
                else:
                    cloud = frame["pts"][valid]
                    center = np.mean(cloud, axis=0)
                    radius_m = float(np.max(np.linalg.norm(self._pts[nm]._pts, axis=1))) / 1000.0
                    n_mean = int((np.linalg.norm(cloud - center[None, :], axis=1)
                                  < radius_m * 1.2).sum())
                    robust_center = np.median(cloud, axis=0)
                    n_median = int((np.linalg.norm(cloud - robust_center[None, :], axis=1)
                                    < radius_m * 1.2).sum())
                    n_radius = max(n_mean, n_median) if n_mean < 4 else n_mean
                    if n_radius < 4:
                        reason = "radius_inliers<4"
                self.last_frame_diag["pem_candidates"].append({
                    "object": nm,
                    "bbox": [int(v) for v in r["box"]],
                    "mask_px": int(m.sum()),
                    "valid_depth_px": n_valid,
                    "radius_inliers": n_radius,
                    "input_rejection": reason,
                })
            try:
                inp, mpts_list, used = self.ric.get_instance_data(
                    frame, None, dets, self.det_thresh, self.pcfg.test_dataset,
                    record_replay=bool(((self.pem_diagnostic.get("explorer_v2") or {})
                                        .get("enabled"))))
                names = [d["cad"] for d in used]
                if self.pem_diagnostic.get("enabled"):
                    refs = diagnostic_references or {}
                    valid, ref_r, ref_t = [], [], []
                    for name in names:
                        ref = refs.get(name)
                        valid.append(ref is not None)
                        ref_r.append(np.eye(3) if ref is None else np.asarray(ref["R"], float))
                        ref_t.append(np.zeros(3) if ref is None else
                                     np.asarray(ref["t_mm"], float) / 1000.0)
                    inp["diagnostic_ref_valid"] = torch.as_tensor(
                        valid, dtype=torch.bool, device=self.device)
                    inp["diagnostic_ref_R"] = torch.as_tensor(
                        np.asarray(ref_r), dtype=torch.float32, device=self.device)
                    inp["diagnostic_ref_t"] = torch.as_tensor(
                        np.asarray(ref_t), dtype=torch.float32, device=self.device)
                used_names = set(names)
                for d in self.last_frame_diag["pem_candidates"]:
                    d["used_by_pem"] = d["object"] in used_names
                with torch.inference_mode():
                    inp["dense_po"] = torch.cat([self._tem[n][0] for n in names], 0)
                    inp["dense_fo"] = torch.cat([self._tem[n][1] for n in names], 0)
                    if all(self._tem[n][2] is not None for n in names):
                        inp["dense_co"] = torch.cat([self._tem[n][2] for n in names], 0)
                    inp["obj_names"] = names              # 대칭 선언 조회용
                    out = self.pem(inp)
                coarse = out["score"].detach().cpu().numpy()
                pose_s = (out["pred_pose_score"].detach().cpu().numpy()
                          if "pred_pose_score" in out else None)
                ps = coarse * pose_s if pose_s is not None else coarse
                Rs = out["pred_R"].detach().cpu().numpy()
                ts = out["pred_t"].detach().cpu().numpy() * 1000.0
                vfs = out.get("verify") or [None] * len(names)
                explorer_payloads = out.get("pem_explorer") or [None] * len(names)
                pds = out.get("pem_diagnostic") or [None] * len(names)
                score_analysis = out.get("pem_score_analysis") or [None] * len(names)
                by = dict(hits)
                for j, nm in enumerate(names):
                    r = by[nm]
                    verification = dict(vfs[j] or {})
                    explorer_payload = explorer_payloads[j]
                    fine_r = np.asarray(Rs[j], dtype=np.float32)
                    fine_t = np.asarray(ts[j], dtype=np.float32)
                    fine_valid = bool(
                        np.isfinite(fine_r).all() and np.isfinite(fine_t).all()
                        and fine_r.shape == (3, 3) and fine_t.shape == (3,)
                        and np.linalg.norm(fine_r.T @ fine_r - np.eye(3)) <= 1e-2
                        and np.linalg.det(fine_r) > 0.99)
                    final_pose = {
                        "valid": fine_valid,
                        "R": (fine_r.tolist() if fine_valid else None),
                        "t_mm": (fine_t.tolist() if fine_valid else None),
                        "score": (float(ps[j]) if np.isfinite(ps[j]) else None),
                        "stage": "fine_refined",
                    }
                    geometry_top1_index = None
                    geometry_top1_proposal = None
                    if explorer_payload is not None:
                        ranks = np.asarray(explorer_payload.get("rank_geo", []))
                        top = np.flatnonzero(ranks == 0)
                        if top.size:
                            geometry_top1_index = int(top[0])
                            proposals = np.asarray(
                                explorer_payload.get("proposal6000", []))
                            if geometry_top1_index < proposals.size:
                                geometry_top1_proposal = int(
                                    proposals[geometry_top1_index])
                    if explorer_payload is not None:
                        for candidate_diag in self.last_frame_diag["pem_candidates"]:
                            if candidate_diag["object"] == nm:
                                candidate_diag.update({
                                    "crop_bbox_yxyx": explorer_payload.get(
                                        "crop_bbox_yxyx", []),
                                    "decision": explorer_payload.get(
                                        "decision", verification),
                                    "explorer_candidates": explorer_payload,
                                    "explorer_replay": explorer_payload.get("replay"),
                                    "geometry_top1_index300": geometry_top1_index,
                                    "geometry_top1_proposal6000": geometry_top1_proposal,
                                    "final_pose": final_pose,
                                    "stage_summary": dict(pds[j] or {}),
                                })
                                break
                    shadows.append({
                        "object": nm, "R": Rs[j], "t_mm": ts[j],
                        "score": float(ps[j]), "verify": verification, "ism": r,
                    })
                    if self.verify.get("enabled") and not verification.get("accepted", False):
                        rejection = {
                            "object": nm,
                            "rank_geo": verification.get("rank_geo"),
                            "mask_iou": ((verification.get("fine") or {}).get("mask_iou")),
                            "texture_score": ((verification.get("fine") or {}).get(
                                "texture_score")),
                            "cluster_occupancy": verification.get("cluster_occupancy"),
                            "pose_source": "sam6d_rejected",
                            "rejection_reason": verification.get(
                                "rejection_reason", "candidate_verification_failed"),
                        }
                        self.last_frame_diag["rejections"].append(rejection)
                        for candidate_diag in self.last_frame_diag["pem_candidates"]:
                            if candidate_diag["object"] == nm:
                                candidate_diag.update(rejection)
                        continue
                    diagnostic = dict(pds[j] or {})
                    # Pointwise evidence already lives in ``verify``. Keeping the same
                    # large array under ``diagnostic`` doubled top-100 dump size without
                    # adding information; the report builder reads either legacy location.
                    m = r["mask"].astype(bool)
                    dz = depth[m & (depth > 0)]
                    x1, y1, x2, y2 = [int(v) for v in r["box"]]
                    crop = bgr[y1:y2, x1:x2]
                    sharpness = (float(cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY),
                                                    cv2.CV_64F).var())
                                 if crop.size else 0.0)
                    rows.append({
                        "object": nm, "score": round(float(ps[j]), 5),
                        "R": [[round(float(v), 6) for v in row] for row in Rs[j]],
                        "t_mm": [round(float(v), 4) for v in ts[j]],
                        "bbox": [int(v) for v in r["box"]],
                        **({"verify": verification} if verification else {}),
                        **({"diagnostic": diagnostic} if diagnostic else {}),
                        **({"score_analysis": score_analysis[j]}
                           if score_analysis[j] else {}),
                        "ism": {k: (round(float(r[k]), 5) if isinstance(r.get(k), (int, float))
                                    else r.get(k))
                                for k in ("best_yolo", "best_sem", "masked_appe", "rank_appe",
                                          "hsv_score", "decision") if k in r},
                        "pem": {"mask_px": int(m.sum()),
                                "depth_valid_px": int((m & (depth > 0)).sum()),
                                "depth_valid_frac": round(float((m & (depth > 0)).sum()
                                                                / max(m.sum(), 1)), 4),
                                "crop_sharpness": round(sharpness, 2),
                                "z_med_mm": round(float(np.median(dz)), 1) if dz.size else None,
                                "n_pts_in": int(inp["pts"].shape[1]),
                                "coarse": round(float(coarse[j]), 5),
                                "pose_score": (round(float(pose_s[j]), 5)
                                               if pose_s is not None else None),
                                "batch": len(names)},
                        "rank_geo": verification.get("rank_geo"),
                        "mask_iou": ((verification.get("fine") or {}).get("mask_iou")),
                        "texture_score": ((verification.get("fine") or {}).get(
                            "texture_score")),
                        "cluster_occupancy": verification.get("cluster_occupancy"),
                        "pose_source": "sam6d",
                        "rejection_reason": None,
                    })
            except Exception as e:
                if ((((self.pem_diagnostic or {}).get("score_analysis") or {}).get("enabled"))):
                    raise
                self.last_frame_diag["pem_error"] = f"{type(e).__name__}: {e}"
                self.log(f"PEM 건너뜀: {type(e).__name__}: {e}")
            if want_mask:
                # Explorer uses one uint16 bitset PNG per frame so overlapping object
                # masks remain lossless. Legacy diagnostics keep their uint8 labels.
                if self.pem_explorer_v2 and len(hits) > 16:
                    raise ValueError("Explorer uint16 mask supports at most 16 objects")
                lab = np.zeros((h, w), np.uint16 if self.pem_explorer_v2 else np.uint8)
                for i, (nm, r) in enumerate(hits):
                    if self.pem_explorer_v2:
                        lab[r["mask"].astype(bool)] |= np.uint16(1 << i)
                    else:
                        lab[r["mask"].astype(bool)] = i + 1
        if self.anchor_manager is not None:
            rows = self._apply_anchors(
                rows, shadows, hits, depth, K, (h, w), slam_context)
        _sync(); t3 = time.perf_counter()
        ms = {"yolo": round(1e3 * (t1 - t0)), "ism": round(1e3 * (t2 - t1)),
              "pem": round(1e3 * (t3 - t2)), "total": round(1e3 * (t3 - t0))}
        return rows, ms, n_boxes, lab

    def _apply_anchors(self, rows, shadows, hits, depth, K, image_shape, slam_context):
        """Update anchors from shadow SAM-6D and apply the configured A/B output policy."""
        manager = self.anchor_manager
        if not slam_context:
            for row in rows:
                row.setdefault("map_id", manager.map_id)
                row.setdefault("anchor_state", "collecting")
            self.last_frame_diag["anchor"] = {"state": "unused", "reason": "slam_pose_missing"}
            return rows
        map_id = str(slam_context.get("map_id", manager.map_id))
        map_changed = manager.set_map(map_id)
        for row in rows:
            row["map_id"] = map_id
            row["anchor_state"] = ("registered" if row["object"] in manager.anchors
                                   else "collecting")
        twc = np.asarray(slam_context.get("T_map_camera"), dtype=float)
        state = slam_context.get("tracking_state", "")
        pose_stamp = int(slam_context.get("pose_stamp_ns", -1))
        rgb_stamp = int(slam_context.get("rgb_stamp_ns", -2))
        if not manager.slam_pose_valid(twc, state, pose_stamp, rgb_stamp):
            self.last_frame_diag["anchor"] = {
                "state": "unused", "map_id": map_id, "map_changed": map_changed,
                "reason": "slam_pose_invalid",
            }
            return rows

        updates = []
        for shadow in shadows:
            verify = shadow.get("verify") or {}
            fine = verify.get("fine") or {}
            tco = pose_matrix(shadow["R"], np.asarray(shadow["t_mm"], float) / 1000.0)
            object_id = shadow["object"]
            if verify.get("accepted"):
                updates.append({"object": object_id, **manager.observe(
                    object_id, twc, tco, state, pose_stamp, rgb_stamp)})
            elif object_id in manager.anchors and fine.get("anchor_shadow_valid"):
                mask_only = verify.get("rejection_reason") in {
                    "mask_filter_empty", "final_mask_below_threshold"}
                updates.append({"object": object_id, **manager.validate_shadow(
                    object_id, twc @ tco, mask_only_failure=mask_only)})

        # Registration/release may have happened above, so expose the state after
        # this frame's observations rather than the state at function entry.
        for row in rows:
            row["anchor_state"] = ("registered" if row["object"] in manager.anchors
                                   else "collecting")

        hit_by_name = dict(hits)
        row_by_name = {row["object"]: row for row in rows}
        eligible_objects = (list(manager.anchors) if manager.config["anchor_output_mode"] == "fov_always"
                            else [name for name in manager.anchors if name in hit_by_name])
        decisions = []
        for object_id in eligible_objects:
            ism = hit_by_name.get(object_id)
            pose, diag = manager.output_decision(
                object_id, twc, self._pts[object_id]._pts / 1000.0, K, image_shape,
                None if ism is None else ism["mask"], depth,
                manager.config["anchor_output_mode"])
            decisions.append({"object": object_id, **diag})
            if pose is None:
                continue
            previous = row_by_name.get(object_id, {})
            anchor_row = {
                **previous,
                "object": object_id,
                # fov_always may create a row without an ISM confidence. Do not
                # fabricate a perfect detector score for an anchor-only output.
                "score": previous.get("score", 0.0),
                "R": [[round(float(v), 6) for v in matrix_row]
                      for matrix_row in pose[:3, :3]],
                "t_mm": [round(float(v) * 1000.0, 4) for v in pose[:3, 3]],
                "bbox": (previous.get("bbox") if previous else
                         ([int(v) for v in ism["box"]] if ism is not None else None)),
                "pose_source": "slam_anchor", "map_id": map_id,
                "anchor_state": "registered", "anchor_diagnostic": diag,
                "rejection_reason": None,
            }
            row_by_name[object_id] = anchor_row
        self.last_frame_diag["anchor"] = {
            "state": "active", "map_id": map_id, "map_changed": map_changed,
            "registered_objects": sorted(manager.anchors),
            "updates": updates, "decisions": decisions,
        }
        # Preserve original order, then append fov-only anchor detections.
        output, seen = [], set()
        for row in rows:
            output.append(row_by_name[row["object"]]); seen.add(row["object"])
        for object_id, row in row_by_name.items():
            if object_id not in seen and row.get("pose_source") == "slam_anchor":
                output.append(row)
        return output
