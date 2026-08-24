#!/usr/bin/env python3
"""sam6d_infer.py — 추론 프로세스. **ROS 를 전혀 쓰지 않는다.**

공유메모리에서 '가장 최신 프레임 하나'만 집어 ISM+PEM 을 돌리고, 결과를 다시 공유메모리로
돌려준다. 산출물(jsonl·마스크)도 여기서 쓴다.

이 프로세스에 ROS 가 없다는 것이 요점이다. 고속 영상 구독과 같은 인터프리터에 있으면
GIL 때문에 추론이 초 단위로 굶는다(NOTES 19~21절). ROS 밖 실측 상한은 ISM 107 ms · PEM 156 ms.

    conda activate sam6d
    python realtime/sam6d_infer.py --config <run yaml>
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shm_channel import FrameReader, JsonWriter          # noqa: E402
import verify_config as VC                             # noqa: E402
from sam6d_core import Sam6DCore, REPO                   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config, encoding="utf-8")) or {}
    rt, out = cfg.get("runtime", {}), cfg.get("output", {})
    odir = out.get("dir", "output/rt_split")
    odir = odir if os.path.isabs(odir) else os.path.join(REPO, odir)
    os.makedirs(odir, exist_ok=True)
    diag = bool(out.get("diagnostics", False))
    if diag:
        os.makedirs(os.path.join(odir, "masks"), exist_ok=True)

    core = Sam6DCore(cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml"),
                     cfg.get("ism", {}).get("objects", []),
                     rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2),
                     appe_rerank=rt.get("appe_rerank"),
                     verify=rt.get("verify", VC.UNSET),
                     pem_diagnostic=rt.get("pem_diagnostic"))

    # 모델이 다 올라온 뒤에야 수신 쪽이 재생을 시작하도록 신호를 남긴다
    ready = os.path.join(odir, "READY")
    Path(ready).write_text(str(time.time()))
    print(f"[infer] 준비 완료 → {ready}", flush=True)

    fr, jw = None, JsonWriter()
    for _ in range(600):                     # 수신 프로세스가 먼저 뜰 때까지 기다린다
        try:
            fr = FrameReader(); break
        except FileNotFoundError:
            time.sleep(0.1)
    if fr is None:
        raise SystemExit("[infer] 프레임 공유메모리를 찾지 못했다")

    f_det = open(os.path.join(odir, "detections.jsonl"), "w", encoding="utf-8")
    f_frm = open(os.path.join(odir, "frames.jsonl"), "w", encoding="utf-8")
    meta = {"started_wall": time.time(), "mode": "split(receiver+infer)",
            "objects": [o["name"] for o in core.objs], "config": cfg}
    json.dump(meta, open(os.path.join(odir, "run_meta.json"), "w"), indent=1, ensure_ascii=False)

    n_proc = n_det = 0
    t_start = time.monotonic()
    idle_since = time.time()
    got_any = False          # 첫 프레임이 오기 전에는 종료 타이머를 걸지 않는다
    try:
        while True:
            got = fr.read_new()
            if got is None:
                # 터미널을 따로 띄우면 사람이 재생/카메라를 켜기까지 시간이 걸린다. 그동안
                # 종료해 버리면 안 되므로 **한 장이라도 받은 뒤에만** 유휴 종료를 건다.
                # idle_exit_s 가 0 이면 스스로 끝나지 않는다(카메라 운용).
                _ie = float(rt.get("idle_exit_s", 20))
                if got_any and _ie > 0 and time.time() - idle_since > _ie:
                    print("[infer] 입력이 끊겨 종료한다", flush=True)
                    break
                time.sleep(0.002)
                continue
            idle_since = time.time()
            got_any = True
            rgb, depth, K, stamp_ns, recv_wall = got
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            t_a = time.time()
            rows, ms, n_boxes, lab = core.process(bgr, depth, K, want_mask=diag)
            t_b = time.time()
            n_proc += 1; n_det += len(rows)
            # bag 시각 환산: 이 프레임이 수신된 벽시계 시각을 기준점으로 삼는다(rate 1.0)
            t_start_ns = int(stamp_ns + (t_a - recv_wall) * 1e9)
            t_done_ns = int(stamp_ns + (t_b - recv_wall) * 1e9)
            jw.write({"stamp_ns": stamp_ns, "t_done_ns": t_done_ns, "n": len(rows),
                      "dets": [{"object": r["object"], "score": r["score"],
                                "R": r["R"], "t_mm": r["t_mm"]} for r in rows],
                      "ms": ms, "n_proc": n_proc, "n_det": n_det})
            for r in rows:
                f_det.write(json.dumps({**r, "stamp_ns": stamp_ns,
                                        "frame_seq": n_proc - 1}, ensure_ascii=False) + "\n")
            f_frm.write(json.dumps({"stamp_ns": stamp_ns, "frame_seq": n_proc - 1,
                                    "n_accept": len(rows), "n_boxes": n_boxes, "ms": ms,
                                    "t_start_ns": t_start_ns, "t_done_ns": t_done_ns,
                                    "objects": [r["object"] for r in rows],
                                    "diagnostics": core.last_frame_diag},
                                   ensure_ascii=False) + "\n")
            f_det.flush(); f_frm.flush()
            if lab is not None and rows:
                cv2.imwrite(os.path.join(odir, "masks", f"{stamp_ns}.png"), lab)
            print(f"#{n_proc-1} {len(rows)}개 ({', '.join(r['object'] for r in rows) or '-'})  "
                  f"{ms['total']}ms [yolo {ms['yolo']} / ism {ms['ism']} / pem {ms['pem']}]",
                  flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        el = max(1e-6, time.monotonic() - t_start)
        summary = {"frames_processed": n_proc, "detections": n_det,
                   "elapsed_s": round(el, 1), "hz_processed": round(n_proc / el, 2)}
        print(f"[summary] {summary}", flush=True)
        meta["summary"] = summary
        json.dump(meta, open(os.path.join(odir, "run_meta.json"), "w"),
                  indent=1, ensure_ascii=False)
        f_det.close(); f_frm.close(); fr.close(); jw.close()
        try:
            os.remove(ready)
        except OSError:
            pass


if __name__ == "__main__":
    main()
