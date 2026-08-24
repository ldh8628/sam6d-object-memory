#!/usr/bin/env python3
"""bench_offline.py — 이 기계에서 SAM-6D 가 실제로 얼마나 빠른지·같은 답을 내는지 잰다.

**ROS 를 전혀 쓰지 않는다.** 미들웨어·구독·GIL 경쟁을 빼고 순수 처리 성능만 보므로,
다른 기계로 옮겼을 때 "느려진 게 하드웨어 탓인가 ROS 탓인가"를 가를 수 있다.

    conda activate sam6d
    python tools/bench_offline.py --frames <프레임폴더> --n 200 --out bench_<기계이름>.json

프레임 폴더는 `<stamp>/{rgb.png,depth.png,meta.json}` 구조다(meta.json 에 cam_K).
없으면 `--frames` 대신 `--synthetic` 으로 회색 프레임을 쓸 수 있다(속도만 비교 가능).

내는 것
  · 단계별(YOLO/ISM/PEM) 시간 분포와 객체 수별 프레임 시간
  · GPU 클럭·전력·온도의 시작/끝 (노트북 발열로 인한 성능 저하를 눈으로 확인)
  · **프레임별 ISM 판정(객체 집합·박스)** — 다른 기계 결과와 비교하면 정확도 동일성 확인
"""
from __future__ import annotations

import argparse, json, os, subprocess, sys, time
from pathlib import Path

import cv2
import numpy as np
import torch

REPO = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, os.path.join(REPO, "realtime"))
from sam6d_core import Sam6DCore                                   # noqa: E402


def gpu_state():
    try:
        q = ("clocks.sm,clocks.max.sm,power.draw,power.limit,temperature.gpu,"
             "utilization.gpu,name")
        o = subprocess.run(["nvidia-smi", f"--query-gpu={q}", "--format=csv,noheader"],
                           capture_output=True, text=True, timeout=5).stdout.splitlines()[0]
        k = [x.strip() for x in o.split(",")]
        return {"sm_mhz": k[0], "sm_max_mhz": k[1], "power_w": k[2], "power_limit_w": k[3],
                "temp_c": k[4], "util": k[5], "name": k[6]}
    except Exception:
        return {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default="", help="<stamp>/{rgb,depth,meta} 폴더들의 상위 경로")
    ap.add_argument("--synthetic", action="store_true", help="프레임이 없을 때 회색 영상으로")
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--config", default="configs/yolo_ism_objects.yaml")
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    core = Sam6DCore(a.config, [], "cuda:0")
    items = []
    if a.frames:
        for d in sorted(Path(a.frames).iterdir()):
            if (d / "meta.json").is_file():
                m = json.load(open(d / "meta.json"))
                items.append((cv2.imread(str(d / "rgb.png")),
                              cv2.imread(str(d / "depth.png"), cv2.IMREAD_UNCHANGED),
                              np.array(m["cam_K"], float).reshape(3, 3), d.name))
    if not items:
        if not a.synthetic:
            raise SystemExit("프레임이 없다. --frames 를 주거나 --synthetic 을 쓸 것")
        bgr = np.full((480, 640, 3), 127, np.uint8)
        dep = np.full((480, 640), 1000, np.uint16)
        K = np.array([[387.0, 0, 320.0], [0, 387.0, 240.0], [0, 0, 1]])
        items = [(bgr, dep, K, "synthetic")]
    print(f"[bench] 프레임 {len(items)} 종 · {a.n} 회 반복")

    g0 = gpu_state()
    rows, decisions = [], {}
    t_all = time.perf_counter()
    for i in range(a.n):
        bgr, dep, K, name = items[i % len(items)]
        det, ms, nb, _ = core.process(bgr, dep, K)
        rows.append({**ms, "n": len(det), "nb": nb, "i": i})
        if i < len(items):                     # 첫 바퀴만 판정 기록(파리티 비교용)
            decisions[name] = sorted(
                [{"object": d["object"], "bbox": d["bbox"]} for d in det],
                key=lambda x: x["object"])
        if i and i % 50 == 0:
            g = gpu_state()
            print(f"  {i}/{a.n}  중앙 {np.median([r['total'] for r in rows]):.0f} ms  "
                  f"GPU {g.get('sm_mhz','?')} / {g.get('power_w','?')} / {g.get('temp_c','?')}C",
                  flush=True)
    el = time.perf_counter() - t_all
    g1 = gpu_state()

    A = {k: np.array([r[k] for r in rows], float) for k in ("yolo", "ism", "pem", "total")}
    warm = slice(1, None)                       # 첫 회는 예열이라 뺀다
    out = {
        "machine": {"gpu": g0.get("name"), "cpu_count": os.cpu_count(),
                    "torch": torch.__version__, "cuda": torch.version.cuda},
        "gpu_start": g0, "gpu_end": g1,
        "n_iter": a.n, "elapsed_s": round(el, 1),
        "hz": round(a.n / el, 2),
        "stage_ms": {k: {"median": round(float(np.median(v[warm])), 1),
                         "p90": round(float(np.percentile(v[warm], 90)), 1),
                         "p99": round(float(np.percentile(v[warm], 99)), 1),
                         "max": round(float(v[warm].max()), 1)}
                     for k, v in A.items()},
        "by_nobj": {}, "decisions": decisions,
    }
    n = np.array([r["n"] for r in rows])
    for k in range(0, 8):
        m = (n == k)
        m[0] = False
        if m.sum() > 2:
            out["by_nobj"][str(k)] = {"n": int(m.sum()),
                                      "median": round(float(np.median(A["total"][m])), 1),
                                      "max": round(float(A["total"][m].max()), 1)}
    print(f"\n=== {g0.get('name','?')} ===")
    print(f"{'단계':>7} {'중앙':>7} {'p90':>7} {'p99':>7} {'최대':>8}  (ms)")
    for k in ("yolo", "ism", "pem", "total"):
        s = out["stage_ms"][k]
        print(f"{k:>7} {s['median']:>7.0f} {s['p90']:>7.0f} {s['p99']:>7.0f} {s['max']:>8.0f}")
    print(f"\n객체 수별 프레임 시간(중앙/최대 ms): " +
          " · ".join(f"{k}개 {v['median']:.0f}/{v['max']:.0f}"
                     for k, v in out["by_nobj"].items()))
    print(f"처리율 {out['hz']:.1f} Hz (쉬지 않고 돌렸을 때)")
    print(f"GPU 클럭 {g0.get('sm_mhz')} → {g1.get('sm_mhz')} (최대 {g0.get('sm_max_mhz')}) · "
          f"전력 {g0.get('power_w')} → {g1.get('power_w')} (한도 {g0.get('power_limit_w')}) · "
          f"온도 {g0.get('temp_c')} → {g1.get('temp_c')} C")
    print("  ↑ 클럭이 내려가고 온도가 올라갔다면 발열로 인한 성능 저하다")
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1, ensure_ascii=False)
        print(f"\n저장: {a.out}  (다른 기계 결과와 tools/bench_compare.py 로 비교)")


if __name__ == "__main__":
    main()
