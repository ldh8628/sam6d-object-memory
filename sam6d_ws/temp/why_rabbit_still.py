#!/usr/bin/env python3
"""why_rabbit_still.py — CAD 크기를 고친 뒤에도 Rabbit 이 흔들리는 이유를 항목별로 잰다.

  A. 크기가 아직 안 맞나          — 마스크로 잰 실물 vs (고친) CAD
  B. 형상이 각도를 정해주나        — 관측 점군에 회전 600 가지를 걸어 잔차 지형
  C. 깊이 품질                   — 국소 평면 잔차(표면 거칠기), 유효 depth 비율
  D. 외형(RGB)이 도움이 되나       — 마스크 안의 명암 대비·에지 밀도
  E. CAD 가 실제 형상과 맞나       — 추정 포즈에서의 관측↔CAD 잔차
"""
import argparse, json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
ROTS = [Rotation.random(random_state=i).as_matrix() for i in range(600)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    a = ap.parse_args()
    MP, TR, DIA = {}, {}, {}
    for f in (REPO / "assets" / "model_points").glob("*.npy"):
        v = np.load(f).astype(np.float64)
        MP[f.stem] = v
        TR[f.stem] = cKDTree(v)
        DIA[f.stem] = float(np.linalg.norm(v.max(0) - v.min(0)))
    S = defaultdict(lambda: defaultdict(list))
    for d in sorted(Path(a.probe).iterdir()):
        if not (d / "meta.json").is_file():
            continue
        m = json.load(open(d / "meta.json"))
        K = np.array(m["cam_K"], float).reshape(3, 3)
        lab = cv2.imread(str(d / "mask.png"), 0)
        dep = cv2.imread(str(d / "depth.png"), cv2.IMREAD_UNCHANGED).astype(float)
        bgr = cv2.imread(str(d / "rgb.png"))
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(float)
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, 3); gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, 3)
        gm = np.hypot(gx, gy)
        for oi, o in enumerate(m["objects"]):
            raw = (lab == oi + 1)
            msk = raw & (dep > 0)
            if msk.sum() < 400:
                continue
            ys, xs = np.nonzero(msk); z = dep[ys, xs]
            C = np.stack([(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], 1)
            det = [x for x in m["dets"] if x["object"] == o][0]
            R0, t0 = np.array(det["R"]), np.array(det["t_mm"])
            # A. 크기
            S[o]["w"].append((xs.max() - xs.min()) * np.median(z) / K[0, 0])
            S[o]["h"].append((ys.max() - ys.min()) * np.median(z) / K[1, 1])
            S[o]["z"].append(float(np.median(z)))
            S[o]["valid"].append(msk.sum() / max(raw.sum(), 1))
            # C. 표면 거칠기 — 이웃 24점 평면 맞춤 잔차 (포즈와 무관)
            sub = C[np.random.RandomState(0).choice(len(C), min(400, len(C)), replace=False)]
            tr = cKDTree(C)
            rough = []
            for p in sub:
                idx = tr.query(p, k=min(25, len(C)))[1]
                Q = C[idx] - C[idx].mean(0)
                w = np.linalg.eigvalsh(np.cov(Q.T))
                rough.append(np.sqrt(max(w[0], 0)))
            S[o]["rough"].append(float(np.median(rough)))
            # D. 외형
            S[o]["contrast"].append(float(gray[msk].std()))
            S[o]["edge"].append(float(np.median(gm[msk])))
            # E. 잔차
            Cc = C - t0
            def res(R):
                return float(np.median(TR[o].query(Cc @ R)[0]))
            r0 = res(R0)
            S[o]["res"].append(r0)
            # B. 회전 지형
            rr = np.array([res(R) for R in ROTS])
            ang = np.array([np.degrees(np.arccos(np.clip((np.trace(R0.T @ R) - 1) / 2, -1, 1)))
                            for R in ROTS])
            far = ang > 60
            S[o]["ambig"].append(float((far & (rr <= r0 * 1.15)).mean()))
            S[o]["rndres"].append(float(np.median(rr[far])))

    print(f"{'객체':>22} {'n':>3} {'실측 가로×세로':>15} {'CAD 가로×세로':>14} "
          f"{'거칠기mm':>9} {'대비':>6} {'에지':>6} {'잔차mm':>7} {'모호도':>7} {'거리mm':>7}")
    for o in sorted(S):
        g = S[o]
        if len(g["res"]) < 3:
            continue
        dim = np.sort(MP[o].max(0) - MP[o].min(0))[::-1]
        print(f"{o:>22} {len(g['res']):>3} "
              f"{np.percentile(g['w'],85):7.0f}×{np.percentile(g['h'],85):<7.0f} "
              f"{dim[0]:6.0f}×{dim[1]:<7.0f} "
              f"{np.median(g['rough']):>9.2f} {np.median(g['contrast']):>6.1f} "
              f"{np.median(g['edge']):>6.0f} {np.median(g['res']):>7.1f} "
              f"{np.median(g['ambig']):>7.3f} {np.median(g['z']):>7.0f}")
    print("\n거칠기 = 이웃 25점 평면 맞춤 잔차 중앙(mm). 털처럼 IR 이 파고드는 표면이면 커진다.")
    print("대비/에지 = 마스크 안 밝기 표준편차 / Sobel 크기 중앙. 무늬가 없으면 작다.")
    print("모호도 = PEM 답에서 60° 넘게 떨어졌는데 잔차가 1.15배 안에 드는 회전의 비율.")


if __name__ == "__main__":
    main()
