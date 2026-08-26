#!/usr/bin/env python3
"""analyze_partial_view_ambiguity.py — 비대칭 물체는 왜 흔들리나.

물체 전체가 대칭이 아니어도, 카메라는 **보이는 앞면 한 겹**만 본다. 둥근 덩어리는
앞면만 놓고 보면 여러 각도에서 거의 같은 껍질이 된다. 그러면 데이터 자체가 답을
하나로 못 정한다 — 이것은 코드 결함이 아니라 관측의 한계다.

이 스크립트는 그것을 직접 잰다. 같은 프레임에서 나온 여러 갈래 포즈에 대해,
**실제로 관측된 점군(마스크∧depth)** 이 그 포즈의 CAD 에 얼마나 잘 붙는지를 각각 재고
주모드와 뒤집힌 모드를 비교한다.

  두 잔차가 비슷하다              -> 데이터가 둘을 구분 못 한다 (관측 모호성)
  뒤집힌 쪽 잔차가 뚜렷이 크다     -> 구분 가능한데 PEM 이 나쁜 쪽을 골랐다 (추정 실패)
"""
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
PROBE = REPO / "outputs" / "test" / "probe"
RES = REPO / "outputs" / "test" / "probe_results.jsonl"


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def cluster(Rs, thr=30.0):
    lab, reps = [], []
    for R in Rs:
        for k, r in enumerate(reps):
            if geo(r, R) < thr:
                lab.append(k); break
        else:
            reps.append(R); lab.append(len(reps) - 1)
    return np.array(lab), reps


MP, TR, DIA = {}, {}, {}
for f in (REPO / "assets" / "model_points").glob("*.npy"):
    v = np.load(f).astype(np.float64)
    MP[f.stem] = v
    TR[f.stem] = cKDTree(v)
    DIA[f.stem] = float(np.linalg.norm(v.max(0) - v.min(0)))

P = [json.loads(l) for l in open(RES, encoding="utf-8") if '"R"' in l]
P = [p for p in P if p["variant"] == "same"]
by = defaultdict(list)
for p in P:
    by[(p["stamp_ns"], p["object"])].append(p)

clouds = {}
for d in sorted(PROBE.iterdir()):
    if not (d / "meta.json").is_file():
        continue
    m = json.load(open(d / "meta.json"))
    K = np.array(m["cam_K"], float).reshape(3, 3)
    dep = cv2.imread(str(d / "depth.png"), cv2.IMREAD_UNCHANGED).astype(np.float64)
    lab = cv2.imread(str(d / "mask.png"), 0)
    for oi, o in enumerate(m["objects"]):
        msk = (lab == oi + 1) & (dep > 0)
        if msk.sum() < 100:
            continue
        ys, xs = np.nonzero(msk); z = dep[ys, xs]
        clouds[(m["stamp_ns"], o)] = np.stack(
            [(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], 1)


def resid(o, cloud, R, t):
    """관측점 -> 그 포즈의 CAD 표면까지 거리 (지름 대비 %)."""
    d, _ = TR[o].query((cloud - t) @ R)
    return 100.0 * float(np.median(d)) / DIA[o]


rows = []
for (ns, o), g in by.items():
    if len(g) < 5 or (ns, o) not in clouds:
        continue
    C = clouds[(ns, o)]
    Rs = [np.array(x["R"]) for x in g]
    ts = [np.array(x["t_mm"]) for x in g]
    lab, reps = cluster(Rs)
    sizes = np.bincount(lab)
    big = int(np.argmax(sizes))
    r_main = np.median([resid(o, C, Rs[i], ts[i]) for i in range(len(Rs)) if lab[i] == big])
    if len(reps) < 2:
        rows.append((o, r_main, None, None, None)); continue
    sec = int(np.argsort(sizes)[::-1][1])
    r_sec = np.median([resid(o, C, Rs[i], ts[i]) for i in range(len(Rs)) if lab[i] == sec])
    ang = geo(reps[big], reps[sec])
    # 명백히 틀린 포즈의 잔차 = 무작위 회전 (기준선)
    r_rnd = np.median([resid(o, C, Rotation.random(random_state=k).as_matrix() @ Rs[big],
                             ts[big]) for k in range(8)])
    rows.append((o, r_main, r_sec, ang, r_rnd))

print("같은 프레임에서 갈라진 두 모드를 '실제 관측 점군' 이 구분하는가")
print("(잔차 = 관측점에서 그 포즈의 CAD 표면까지 거리, 물체 지름 대비 %)\n")
print(f"{'객체':>22} {'조합':>4} {'주모드':>7} {'딴모드':>7} {'차이':>7} {'무작위포즈':>10} "
      f"{'모드간각':>8}  {'판정':>12}")
tot = defaultdict(list)
for o in sorted({r[0] for r in rows}):
    g = [r for r in rows if r[0] == o and r[2] is not None]
    if not g:
        g0 = [r for r in rows if r[0] == o]
        print(f"{o:>22} {len(g0):>4} {np.median([r[1] for r in g0]):>7.1f} "
              f"{'-':>7} {'-':>7} {'-':>10} {'-':>8}  {'단일모드':>12}")
        continue
    m1 = np.median([r[1] for r in g]); m2 = np.median([r[2] for r in g])
    rr = np.median([r[4] for r in g]); an = np.median([r[3] for r in g])
    # 딴 모드가 무작위 포즈 쪽에 가까운가, 주모드 쪽에 가까운가
    frac = (m2 - m1) / max(rr - m1, 1e-9)
    verdict = "관측 모호" if frac < 0.35 else ("추정 실패" if frac > 0.65 else "중간")
    tot[verdict].append(o)
    print(f"{o:>22} {len(g):>4} {m1:>7.1f} {m2:>7.1f} {m2-m1:>+7.1f} {rr:>10.1f} "
          f"{an:>8.0f}  {verdict:>12}")
print("\n판정 기준: (딴모드 잔차 - 주모드 잔차) / (무작위포즈 잔차 - 주모드 잔차)")
print("  0 에 가까우면 = 딴 모드도 관측을 주모드만큼 잘 설명한다 -> 데이터로 구분 불가")
print("  1 에 가까우면 = 딴 모드는 명백히 안 맞는다 -> PEM 이 잘못 고른 것")
