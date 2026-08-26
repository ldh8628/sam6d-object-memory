#!/usr/bin/env python3
"""view_confusability.py — "이 물체는 각도를 알아볼 수 있게 생겼나"를 그림과 숫자로 낸다.

깊이 카메라가 보는 것은 물체 전체가 아니라 **앞면 한 겹**이다. 그래서 물체를 여러 각도로
돌려 놓고 앞면만 렌더해서, 서로 얼마나 달라 보이는지를 직접 잰다.
많이 돌렸는데도 앞면이 똑같아 보이는 물체는 한 장으로 각도를 정할 수 없다.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/vc")
OUT.mkdir(parents=True, exist_ok=True)
S = 96                                    # 렌더 해상도


def render_depth(P, R, size=S):
    """점군을 R 로 돌린 뒤 정면에서 본 '앞면 깊이 지도'. 뒤에 가린 점은 버린다."""
    Q = P @ R.T
    d = np.linalg.norm(P.max(0) - P.min(0))
    sc = (size * 0.82) / d
    u = np.clip((Q[:, 0] * sc + size / 2).astype(int), 0, size - 1)
    v = np.clip((Q[:, 1] * sc + size / 2).astype(int), 0, size - 1)
    img = np.full((size, size), np.inf)
    np.minimum.at(img, (v, u), Q[:, 2])   # z 가 작은 것(카메라에 가까운 것)만 남긴다
    m = np.isfinite(img)
    if m.sum() < 20:
        return None, None
    img[m] -= img[m].mean()
    img[~m] = 0
    return img * sc, m                    # 픽셀 단위로 환산한 깊이


def diss(a, ma, b, mb):
    """두 앞면이 얼마나 다른가 — 겹치지 않는 실루엣 + 겹치는 곳의 깊이 차이 (픽셀)."""
    inter = ma & mb
    union = ma | mb
    sil = 1.0 - inter.sum() / max(union.sum(), 1)
    dep = float(np.abs(a[inter] - b[inter]).mean()) / S if inter.sum() else 1.0
    return sil + dep


names = sorted(p.stem for p in (REPO / "assets" / "model_points").glob("*.npy"))
Rs = [Rotation.random(random_state=i).as_matrix() for i in range(48)]
print(f"{'객체':>22} {'>90° 떨어진 쌍의 앞면 차이':>26} {'그중 거의 같은 쌍':>18}")
rows = {}
for nm in names:
    P = np.load(REPO / "assets" / "model_points" / f"{nm}.npy").astype(np.float64)
    P = P - P.mean(0)
    ims = [render_depth(P, R) for R in Rs]
    D, same = [], 0
    for i in range(len(Rs)):
        for j in range(i + 1, len(Rs)):
            ang = np.degrees(np.arccos(np.clip((np.trace(Rs[i].T @ Rs[j]) - 1) / 2, -1, 1)))
            if ang < 90 or ims[i][0] is None or ims[j][0] is None:
                continue
            v = diss(*ims[i], *ims[j])
            D.append(v)
            same += int(v < 0.25)
    D = np.array(D)
    rows[nm] = (float(np.median(D)), same / max(len(D), 1))
    print(f"{nm:>22} {np.median(D):>26.3f} {same/max(len(D),1):>18.2f}")
    # 눈으로 볼 수 있게 12 각도 렌더를 한 줄로
    tiles = []
    for k in range(12):
        a, m = ims[k]
        if a is None:
            continue
        t = np.zeros((S, S, 3), np.uint8)
        z = a.copy()
        z[m] = (z[m] - z[m].min()) / max(float(z[m].max()-z[m].min()), 1e-6)
        t[..., 0] = t[..., 1] = t[..., 2] = (m * (60 + 195 * (1 - z))).astype(np.uint8)
        cv2.rectangle(t, (0, 0), (S - 1, S - 1), (45, 45, 45), 1)
        tiles.append(t)
    cv2.imwrite(str(OUT / f"views_{nm}.png"), np.hstack(tiles))

print("\n앞면 차이 0 = 완전히 같아 보임, 1 이상 = 전혀 다름. '거의 같은 쌍'은 차이<0.25 인 비율.")
print("(48 개 무작위 각도 중 90° 넘게 떨어진 모든 쌍)")
order = sorted(rows, key=lambda k: rows[k][0])
print("\n각도를 알아보기 어려운 순서:", " < ".join(order))
