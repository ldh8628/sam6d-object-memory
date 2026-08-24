#!/usr/bin/env python3
"""offline_stall_test.py — 몇 초짜리 정체가 ROS 입력 처리 탓인가, 파이프라인 자체 탓인가.

같은 프레임을 ROS 없이 N 번 반복 처리한다. 입력·구독·executor 가 전혀 없으므로,
여기서도 몇 초짜리 정체가 나오면 원인은 **파이프라인(커널 launch/동기화 패턴)** 이고,
안 나오면 원인은 **ROS 입력 처리(다른 스레드·GIL)** 다.

단계별로 벽시계·스레드 CPU 시간을 같이 남긴다.
"""
import argparse, json, os, resource, sys, time
from pathlib import Path

import cv2
import numpy as np
import torch

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import yolo_ism as yi
import yolo_ism_object_n as o_n
import pem_probe as PP


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", required=True, help="probe/<stamp> 폴더")
    ap.add_argument("--n", type=int, default=300)
    a = ap.parse_args()
    dev = "cuda:0"
    meta = json.load(open(Path(a.frame) / "meta.json"))
    bgr = cv2.imread(str(Path(a.frame) / "rgb.png"))
    dep = cv2.imread(str(Path(a.frame) / "depth.png"), cv2.IMREAD_UNCHANGED)
    K = np.array(meta["cam_K"], float).reshape(3, 3)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(objs, model, dev, rebuild=False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    prompts, groups = o_n.build_prompt_groups(objs)
    tsim = o_n.template_similarity(objs)
    imgsz = int(defaults.get("imgsz", 640))
    ml = o_n._multi_label_on(objs[0])
    minsc = min(float(o.get("score_threshold", 0.02)) for o in objs)
    import ultralytics.nn.text_model as _tm
    _tm.WEIGHTS_DIR = Path(os.path.join(REPO, "assets"))
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(o_n._abspath(defaults.get("weights", "yolov8m-worldv2.pt")))
    yolo.set_classes(prompts)
    ric, pcfg, pem, tem = PP.load_pem(dev, [o["name"] for o in objs])
    print(f"[load] 객체 {len(objs)} · 프롬프트 {len(prompts)}")

    norm = yi.normalize_rgb(rgb)
    rows = []
    for it in range(a.n):
        torch.cuda.synchronize()
        ru0 = resource.getrusage(resource.RUSAGE_SELF)
        t0, c0 = time.perf_counter(), time.thread_time()
        with o_n.multi_label_nms(ml):
            res = yolo.predict(bgr, conf=minsc, imgsz=imgsz, verbose=False, device=dev)
        torch.cuda.synchronize(); t1, c1 = time.perf_counter(), time.thread_time()
        pb = {i: [] for i in range(len(prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), bgr.shape[1] - 1)); y1 = max(0, min(int(xy[1]), bgr.shape[0] - 1))
                x2 = max(x1 + 1, min(int(xy[2]), bgr.shape[1])); y2 = max(y1 + 1, min(int(xy[3]), bgr.shape[0]))
                ci = int(b.cls[j]) if b.cls is not None else 0
                if ci in pb:
                    pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
        for k in pb:
            pb[k].sort(key=lambda t: t[1], reverse=True)
        nb = sum(len(v) for v in pb.values())
        out = o_n.recognize_frame_auto(groups, pb, bgr, rgb, norm, model, dev,
                                       segmentor, pool, tsim)
        torch.cuda.synchronize(); t2, c2 = time.perf_counter(), time.thread_time()
        hits = [(n, r) for n, r in sorted(out.items()) if r.get("accepted") and r.get("mask") is not None]
        npem = 0
        if hits:
            dets = [{"score": 1.0, "mask": r["mask"].astype(bool), "cad": n} for n, r in hits]
            frame = ric.prepare_frame(rgb, dep, K)
            try:
                inp, mpts, used = ric.get_instance_data(frame, None, dets, 0.2, pcfg.test_dataset)
                names = [d["cad"] for d in used]
                with torch.inference_mode():
                    inp["dense_po"] = torch.cat([tem[n][0] for n in names], 0)
                    inp["dense_fo"] = torch.cat([tem[n][1] for n in names], 0)
                    pem(inp)
                npem = len(names)
            except Exception as e:
                if it == 0:
                    print("  PEM 건너뜀:", type(e).__name__, e)
        torch.cuda.synchronize(); t3, c3 = time.perf_counter(), time.thread_time()
        ru1 = resource.getrusage(resource.RUSAGE_SELF)
        rows.append(dict(yolo=1e3*(t1-t0), ism=1e3*(t2-t1), pem=1e3*(t3-t2),
                         total=1e3*(t3-t0), cyolo=1e3*(c1-c0), cism=1e3*(c2-c1),
                         cpem=1e3*(c3-c2), nvcsw=ru1.ru_nvcsw-ru0.ru_nvcsw,
                         nb=nb, n=npem))
    A = {k: np.array([r[k] for r in rows], float) for k in rows[0]}
    print(f"\n반복 {a.n} 회 · 박스 {A['nb'][0]:.0f}개 · PEM 객체 {A['n'][0]:.0f}개 (ROS 없음)")
    print(f"{'단계':>6} {'중앙':>7} {'p90':>8} {'p99':>8} {'최대':>9} {'CPU/벽 중앙':>11}")
    for k, ck in (("yolo","cyolo"), ("ism","cism"), ("pem","cpem"), ("total",None)):
        w = A[k]
        r = (np.median(A[ck]/np.maximum(w,1e-6)) if ck else float('nan'))
        print(f"{k:>6} {np.median(w):>7.0f} {np.percentile(w,90):>8.0f} "
              f"{np.percentile(w,99):>8.0f} {w.max():>9.0f} {r:>11.2f}")
    s = A["total"] > 1000
    print(f"\n1 초 넘는 반복 {int(s.sum())} / {a.n}  ({s.mean()*100:.1f}%)")
    print(f"자발적 문맥교환: 중앙 {np.median(A['nvcsw']):.0f} · 최대 {A['nvcsw'].max():.0f}")
    if s.sum():
        print(f"   느린 반복의 문맥교환 중앙 {np.median(A['nvcsw'][s]):.0f}")


if __name__ == "__main__":
    main()
