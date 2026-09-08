#!/usr/bin/env python3
"""pose_ambiguity_landscape.py — "이 관측은 각도를 하나로 정해주는가"를 실제 데이터로 잰다.

한 검출에 대해, 그 프레임에서 실제로 찍힌 점군(마스크∧depth)을 놓고 **회전을 600 가지로
바꿔 가며** CAD 와의 잔차를 잰다. 추정된 포즈만 잘 맞고 나머지는 다 나쁘면 답이 하나로
정해지는 것이고, 엉뚱한 각도도 비슷하게 잘 맞으면 관측이 각도를 못 정해주는 것이다.

그림도 같이 만든다: 실제 이미지 위에 '거의 똑같이 잘 맞는' 다른 각도들을 얹어 보여준다.
"""
import json, sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
PROBE = Path(sys.argv[1])
OUT = Path(sys.argv[2]); OUT.mkdir(parents=True, exist_ok=True)
NROT = 600

MP, TR, DIA = {}, {}, {}
for f in (REPO / "assets" / "model_points").glob("*.npy"):
    v = np.load(f).astype(np.float64)
    MP[f.stem] = v
    TR[f.stem] = cKDTree(v)
    DIA[f.stem] = float(np.linalg.norm(v.max(0) - v.min(0)))
ROTS = [Rotation.random(random_state=i).as_matrix() for i in range(NROT)]

stat = defaultdict(list)
figs = {}
for d in sorted(PROBE.iterdir()):
    if not (d / "meta.json").is_file():
        continue
    m = json.load(open(d / "meta.json"))
    K = np.array(m["cam_K"], float).reshape(3, 3)
    dep = cv2.imread(str(d / "depth.png"), cv2.IMREAD_UNCHANGED).astype(np.float64)
    lab = cv2.imread(str(d / "mask.png"), 0)
    bgr = cv2.imread(str(d / "rgb.png"))
    for oi, o in enumerate(m["objects"]):
        msk = (lab == oi + 1) & (dep > 0)
        if msk.sum() < 200:
            continue
        ys, xs = np.nonzero(msk); z = dep[ys, xs]
        C = np.stack([(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], 1)
        det = [x for x in m["dets"] if x["object"] == o][0]
        R0, t0 = np.array(det["R"]), np.array(det["t_mm"])
        Cc = C - t0
        def res(R):
            dd, _ = TR[o].query(Cc @ R)
            return 100.0 * float(np.median(dd)) / DIA[o]
        r0 = res(R0)
        rr = np.array([res(R) for R in ROTS])
        ang = np.array([np.degrees(np.arccos(np.clip((np.trace(R0.T @ R) - 1) / 2, -1, 1)))
                        for R in ROTS])
        far = ang > 60                                   # 확실히 다른 각도만
        good = far & (rr <= r0 * 1.15)                   # 그런데도 비슷하게 잘 맞는 것
        stat[o].append((r0, float(np.median(rr[far])), float(good.mean()),
                        float(rr[far].min())))
        # 그림은 '마스크가 크고 추정 포즈가 정상적으로 붙은' 검출로 고른다
        if msk.sum() > 900 and r0 < 6 and msk.sum() > figs.get(o, (0,))[0]:
            order = np.argsort(np.where(far, rr, 1e9))[:4]
            figs[o] = (int(msk.sum()), bgr, det, o, K, R0, t0,
                       [(ROTS[i], rr[i], ang[i]) for i in order], r0)

print("실제 관측이 각도를 하나로 정해주는가 (잔차 = 관측점→CAD 표면, 지름 대비 %)\n")
print(f"{'객체':>22} {'n':>4} {'추정포즈':>8} {'무작위각도':>10} {'60°+ 떨어졌는데':>16} {'그 최소잔차':>10}")
print(f"{'':>22} {'':>4} {'잔차':>8} {'잔차중앙':>10} {'비슷하게 맞는 비율':>16} {'':>10}")
for o in sorted(stat):
    a = np.array(stat[o])
    print(f"{o:>22} {len(a):>4} {np.median(a[:,0]):>8.1f} {np.median(a[:,1]):>10.1f} "
          f"{np.median(a[:,2]):>16.3f} {np.median(a[:,3]):>10.1f}")

# ---- 그림 ----
for o, (_npx, bgr, det, nm, K, R0, t0, alts, r0) in figs.items():
    x1, y1, x2, y2 = det["bbox"]
    pad = 26
    X1, Y1 = max(0, x1 - pad), max(0, y1 - pad)
    X2, Y2 = min(bgr.shape[1], x2 + pad), min(bgr.shape[0], y2 + pad)
    P = MP[nm]
    def panel(R, txt, col):
        im = bgr.copy()
        q = (R @ P.T).T + t0
        q = q[q[:, 2] > 1]
        u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(int)
        v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(int)
        ok = (u >= 0) & (u < im.shape[1]) & (v >= 0) & (v < im.shape[0])
        im[v[ok], u[ok]] = col
        c = cv2.resize(im[Y1:Y2, X1:X2], (190, 190), interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(c, (0, 172), (190, 190), (0, 0, 0), -1)
        cv2.putText(c, txt, (4, 186), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
        return c
    raw = cv2.resize(bgr[Y1:Y2, X1:X2], (190, 190), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(raw, (0, 172), (190, 190), (0, 0, 0), -1)
    cv2.putText(raw, "observed", (4, 186), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
    tiles = [raw, panel(R0, f"PEM  fit {r0:.1f}%", (60, 230, 60))]
    for R, r, a in alts:
        tiles.append(panel(R, f"{a:.0f}deg away  fit {r:.1f}%", (80, 160, 255)))
    cv2.imwrite(str(OUT / f"ambig_{nm}.png"), np.hstack(tiles))
print(f"\n그림: {OUT}/ambig_<객체>.png  (초록=PEM 이 고른 포즈, 파랑=크게 다른 각도인데 비슷하게 맞는 포즈)")
