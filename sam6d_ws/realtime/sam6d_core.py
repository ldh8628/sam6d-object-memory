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
from map_prior import MapPrior                                  # noqa: E402


def _sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


class Sam6DCore:
    def __init__(self, ism_config, objects=None, device="cuda:0",
                 det_score_thresh=0.2, log=print, appe_rerank=None, verify=UNSET,
                 mask_gate=UNSET, map_prior=UNSET):
        self.log = log
        self.device = device if torch.cuda.is_available() else "cpu"
        self.det_thresh = float(det_score_thresh)
        # 외형 재정렬 옵션(기본 None = 예전 동작). split 모드도 단일 프로세스 모드와
        # 똑같이 이 옵션을 받게 한다 — 예전에는 sam6d_realtime_node.py 에만 있어서
        # split 실행에서는 config 에 써도 조용히 무시됐다.
        self.appe_rerank = appe_rerank
        # 텍스처 검증(Stage V) · 마스크 게이트(ch1) · Map 사전(ch2).
        # 노드와 같은 기본값·같은 병합 규칙을 쓴다 — 기본값이 두 군데로 갈라지면
        # "노드는 되는데 split 은 안 된다"가 반드시 생긴다.
        rt = {"appe_rerank": appe_rerank}
        if verify is not UNSET:                    # 안 주면 기본값(=검증 ON)
            rt["verify"] = verify
        if mask_gate is not UNSET:
            rt["mask_gate"] = mask_gate
        if map_prior is not UNSET:
            rt["map_prior"] = map_prior
        self.appe_cfg, self.verify = build_appe_cfg(rt)
        self.verify = self.verify or {}
        # ch2 — Map 사전. 포즈가 안 들어오면 조용히 아무 일도 하지 않는다(fail-open).
        self.map_prior = MapPrior((self.appe_cfg or {}).get("map_prior"), log=self.log)
        self._map_dg = None
        # 후보 소실 케이스 보관(위험 1 후속 분석용). 검출 행을 부풀리지 않도록 여기 모았다가
        # 부르는 쪽이 gate_cases.jsonl 로 따로 쓴다. max_cases 를 넘으면 건수만 센다.
        _mgc = (self.appe_cfg or {}).get("mask_gate") or {}
        self.gate_cases = []
        self.gate_cases_dropped = 0
        self._max_cases = int(_mgc.get("max_cases", 5000)) if _mgc.get("dump_cases", True) else 0
        self._load_ism(ism_config, objects or [])
        self._load_pem()

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
        self.log(describe(self.appe_cfg, self.verify or None))
        MODEL = importlib.import_module(self.pcfg.model_name)
        self.pem = MODEL.Net(self.pcfg.model).to(self.device).eval()
        gorilla.solver.load_checkpoint(
            model=self.pem, filename=os.path.join(PEM_DIR, "checkpoints", "sam-6d-pem-base.pth"))

        class _Stub:
            def __init__(s, pts): s._pts = pts
            def sample(s, n):
                if n == len(s._pts):
                    return s._pts
                return s._pts[np.random.choice(len(s._pts), n, replace=(n > len(s._pts)))]

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



    def _stash_case(self, name, gt):
        """gate 행에서 후보 덤프를 떼어 따로 모은다(검출 행은 가볍게 유지)."""
        c = gt.pop("cases", None)
        if c is not None:
            if len(self.gate_cases) < self._max_cases:
                self.gate_cases.append({"object": name, **c})
            else:
                self.gate_cases_dropped += 1
        return gt

    # ------------------------------------------------------------ ch2 보조
    def _attach_map_prior(self, inp, names, by_name, T_map_cam, K):
        """등록된 Map 자세를 카메라계 참조로 바꿔 PEM 입력에 싣는다.

        PEM 을 부르기 **전**이라 t_cam_obj 가 없다 → 지도 위치로 군집할 수 없고,
        등록 인스턴스를 현재 카메라로 역투영해 검출 박스에 떨어지는지로 연관한다.
        """
        if not self.map_prior.enabled or T_map_cam is None:
            return
        Kn = np.asarray(K, dtype=float).reshape(3, 3)
        refs, oks, dgs = [], [], []
        for nm in names:
            box = by_name.get(nm, {}).get("box")
            R, dg = self.map_prior.reference(nm, T_map_cam, box=box, K=Kn)
            oks.append(R is not None)
            refs.append(np.eye(3) if R is None else R)
            dgs.append(dg)
        if not any(oks):
            return
        dev = inp["pts"].device
        inp["map_ref_R"] = torch.tensor(np.stack(refs), dtype=torch.float32, device=dev)
        inp["map_ref_ok"] = torch.tensor(oks, dtype=torch.bool, device=dev)
        self._map_dg = dgs

    def _vote_map_prior(self, nm, R, t_mm, r, vf, gt, T_map_cam):
        """PEM 결과 하나를 지도에 올린다. 자격 미달이면 조용히 버린다."""
        if not self.map_prior.enabled or T_map_cam is None:
            return None
        box = r.get("box")
        px = min(box[2] - box[0], box[3] - box[1]) if box else None
        return self.map_prior.vote(
            nm, R, [v / 1000.0 for v in t_mm], T_map_cam, bbox_px=px,
            verdict=(vf or {}).get("verdict"),
            admitted_by=(gt or {}).get("admit"))

    # ------------------------------------------------------------------ 처리
    def process(self, bgr, depth, K, want_mask=False, stamp_ns=None, T_map_cam=None):
        """한 프레임 → (검출 목록, 단계별 ms, 마스크 라벨 이미지 or None).

        T_map_cam 은 **그 프레임을 찍던 순간**의 카메라 자세다(도착 시각이 아니다).
        조회는 부르는 쪽이 한다 — 이 파일은 ROS 를 쓰지 않는 것이 존재 이유다.
        """
        _sync(); t0 = time.perf_counter()
        self.map_prior.note_camera(T_map_cam)     # 재위치추정 점프면 지도 좌표가 무의미해진다
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
        rows, lab = [], None
        if hits:
            frame = self.ric.prepare_frame(rgb, depth, K)
            dets = [{"score": 1.0, "mask": r["mask"].astype(bool), "cad": n} for n, r in hits]
            try:
                inp, mpts_list, used = self.ric.get_instance_data(
                    frame, None, dets, self.det_thresh, self.pcfg.test_dataset)
                names = [d["cad"] for d in used]
                with torch.inference_mode():
                    inp["dense_po"] = torch.cat([self._tem[n][0] for n in names], 0)
                    inp["dense_fo"] = torch.cat([self._tem[n][1] for n in names], 0)
                    if all(self._tem[n][2] is not None for n in names):
                        inp["dense_co"] = torch.cat([self._tem[n][2] for n in names], 0)
                    inp["obj_names"] = names              # 대칭 선언 조회용
                    self._attach_map_prior(inp, names, by_name=dict(hits),
                                           T_map_cam=T_map_cam, K=K)
                    out = self.pem(inp)
                coarse = out["score"].detach().cpu().numpy()
                pose_s = (out["pred_pose_score"].detach().cpu().numpy()
                          if "pred_pose_score" in out else None)
                ps = coarse * pose_s if pose_s is not None else coarse
                Rs = out["pred_R"].detach().cpu().numpy()
                ts = out["pred_t"].detach().cpu().numpy() * 1000.0
                vfs = out.get("verify") or [None] * len(names)
                gts = out.get("gate") or [None] * len(names)
                by = dict(hits)
                # ⚠ 투표는 **행을 만들기 전에** 한다 — gate.admit(어느 채널이 승자를
                #   들여보냈는가)이 있어야 고리 차단기(vote_need_mask)가 작동한다.
                mvs = [self._vote_map_prior(nm, Rs[j], ts[j], by[nm], vfs[j], gts[j],
                                            T_map_cam)
                       for j, nm in enumerate(names)]
                for j, nm in enumerate(names):
                    if mvs[j] is not None and getattr(self, "_map_dg", None):
                        mvs[j] = dict(self._map_dg[j] or {}, **mvs[j])
                self._map_dg = None
                for j, nm in enumerate(names):
                    r = by[nm]
                    m = r["mask"].astype(bool)
                    dz = depth[m & (depth > 0)]
                    rows.append({
                        "object": nm, "score": round(float(ps[j]), 5),
                        "R": [[round(float(v), 6) for v in row] for row in Rs[j]],
                        "t_mm": [round(float(v), 4) for v in ts[j]],
                        "bbox": [int(v) for v in r["box"]],
                        **({"verify": vfs[j]} if vfs[j] else {}),
                        **({"gate": self._stash_case(nm, gts[j])} if gts[j] else {}),
                        **({"map": mvs[j]} if mvs[j] else {}),
                        "ism": {k: (round(float(r[k]), 5) if isinstance(r.get(k), (int, float))
                                    else r.get(k))
                                for k in ("best_yolo", "best_sem", "masked_appe", "rank_appe",
                                          "hsv_score", "decision") if k in r},
                        "pem": {"mask_px": int(m.sum()),
                                "depth_valid_px": int((m & (depth > 0)).sum()),
                                "depth_valid_frac": round(float((m & (depth > 0)).sum()
                                                                / max(m.sum(), 1)), 4),
                                "z_med_mm": round(float(np.median(dz)), 1) if dz.size else None,
                                "n_pts_in": int(inp["pts"].shape[1]),
                                "coarse": round(float(coarse[j]), 5),
                                "pose_score": (round(float(pose_s[j]), 5)
                                               if pose_s is not None else None),
                                "batch": len(names)},
                    })
            except Exception as e:
                self.log(f"PEM 건너뜀: {type(e).__name__}: {e}")
            if want_mask:
                lab = np.zeros((h, w), np.uint8)
                for i, (nm, r) in enumerate(hits):
                    lab[r["mask"].astype(bool)] = i + 1
        _sync(); t3 = time.perf_counter()
        ms = {"yolo": round(1e3 * (t1 - t0)), "ism": round(1e3 * (t2 - t1)),
              "pem": round(1e3 * (t3 - t2)), "total": round(1e3 * (t3 - t0))}
        return rows, ms, n_boxes, lab
