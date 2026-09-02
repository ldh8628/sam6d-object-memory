#!/usr/bin/env python3
"""rabbit_ambiguity_axis.py — '똑같이 잘 맞는' 다른 각도들이 어느 축을 도는 것인가.

관측 점군에 대해 잔차가 PEM 답의 1.15 배 안에 드는 회전들을 모아, 그 회전의 축을
**물체 좌표계**에서 본다. 한 축에 몰리면 그 축을 도는 것이 안 보인다는 뜻이다
(예: 회전체는 세로축, 납작판은 판 법선).
그림도 만든다: 실제 이미지 위에 그 각도들을 얹어 보여준다.
"""
import argparse, json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
ROTS = [Rotation.random(random_state=i).as_matrix() for i in range(900)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    MP, TR = {}, {}
    for f in (REPO / "assets" / "model_points").glob("*.npy"):
        v = np.load(f).astype(np.float64)
        MP[f.stem] = v; TR[f.stem] = cKDTree(v)
    acc = defaultdict(list)
    figs = {}
    for d in sorted(Path(a.probe).iterdir()):
        if not (d / "meta.json").is_file():
            continue
        m = json.load(open(d / "meta.json"))
        K = np.array(m["cam_K"], float).reshape(3, 3)
        lab = cv2.imread(str(d / "mask.png"), 0)
        dep = cv2.imread(str(d / "depth.png"), cv2.IMREAD_UNCHANGED).astype(float)
        bgr = cv2.imread(str(d / "rgb.png"))
        for oi, o in enumerate(m["objects"]):
            msk = (lab == oi + 1) & (dep > 0)
            if msk.sum() < 400:
                continue
            ys, xs = np.nonzero(msk); z = dep[ys, xs]
            C = np.stack([(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], 1)
            det = [x for x in m["dets"] if x["object"] == o][0]
            R0, t0 = np.array(det["R"]), np.array(det["t_mm"])
            Cc = C - t0
            r0 = float(np.median(TR[o].query(Cc @ R0)[0]))
            good = []
            for R in ROTS:
                ang = np.degrees(np.arccos(np.clip((np.trace(R0.T @ R) - 1) / 2, -1, 1)))
                if ang <= 60:
                    continue
                if float(np.median(TR[o].query(Cc @ R)[0])) <= r0 * 1.15:
                    good.append((R, ang))
            for R, ang in good:
                Rr = R0.T @ R
                w, v = np.linalg.eig(Rr)
                ax = np.real(v[:, np.argmin(np.abs(w - 1))])
                acc[o].append(np.abs(ax / (np.linalg.norm(ax) + 1e-12)))
            if o == "Rabbit" and o not in figs and len(good) >= 4 and msk.sum() > 900:
                figs[o] = (bgr, det, K, R0, t0, good[:4], r0, msk)
    print(f"{'객체':>22} {'동등 회전 수':>11} {'축 |x|':>8} {'|y|':>7} {'|z|':>7}")
    for o in sorted(acc):
        A = np.array(acc[o])
        if len(A) < 5:
            print(f"{o:>22} {len(A):>11}  (표본 부족)")
            continue
        m_ = np.median(A, 0)
        print(f"{o:>22} {len(A):>11} {m_[0]:>8.2f} {m_[1]:>7.2f} {m_[2]:>7.2f}")
    if a.out and figs:
        bgr, det, K, R0, t0, good, r0, msk = figs["Rabbit"]
        P = MP["Rabbit"]
        x1, y1, x2, y2 = det["bbox"]; pad = 28
        X1, Y1 = max(0, x1 - pad), max(0, y1 - pad)
        X2, Y2 = min(bgr.shape[1], x2 + pad), min(bgr.shape[0], y2 + pad)
        def panel(R, txt, col):
            im = bgr.copy()
            q = (R @ P.T).T + t0
            q = q[q[:, 2] > 1]
            u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(int)
            v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(int)
            ok = (u >= 0) & (u < im.shape[1]) & (v >= 0) & (v < im.shape[0])
            im[v[ok], u[ok]] = col
            c = cv2.resize(im[Y1:Y2, X1:X2], (200, 200), interpolation=cv2.INTER_NEAREST)
            cv2.rectangle(c, (0, 181), (200, 200), (0, 0, 0), -1)
            cv2.putText(c, txt, (4, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
            return c
        raw = cv2.resize(bgr[Y1:Y2, X1:X2], (200, 200), interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(raw, (0, 181), (200, 200), (0, 0, 0), -1)
        cv2.putText(raw, "observed (CAD size fixed)", (4, 195), cv2.FONT_HERSHEY_SIMPLEX,
                    0.36, (255, 255, 255), 1)
        tiles = [raw, panel(R0, f"PEM  fit {r0:.1f}mm", (60, 230, 60))]
        for R, ang in good:
            rr = float(np.median(TR["Rabbit"].query(
                (np.stack(np.nonzero(msk), 1)[:1] * 0 + 0).astype(float))[0])) if False else 0
            tiles.append(panel(R, f"{ang:.0f}deg away", (80, 160, 255)))
        cv2.imwrite(a.out, np.hstack(tiles))
        print(f"\n그림: {a.out}")


if __name__ == "__main__":
    main()
