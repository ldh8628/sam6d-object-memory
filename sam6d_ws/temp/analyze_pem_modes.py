#!/usr/bin/env python3
"""analyze_pem_modes.py — "같은 입력인데 왜 두 갈래로 갈리나"를 끝까지 판다.

pem_probe.py 의 반복 실행 결과를 모드(군집)로 나눈 뒤 두 가지를 묻는다.

  Q1. 두 모드를 잇는 회전이 그 물체의 **진짜 대칭**인가?
      -> 모델 점군에 그 회전을 걸어 보고 자기 자신과 얼마나 겹치는지(Chamfer)로 판정.
         겹치면 '동치인 해'(형상만으론 구분 불가), 안 겹치면 '틀린 해'.
  Q2. PEM 이 스스로 내는 점수(pose_score)가 두 모드를 가려내는가?
      -> 가려낸다면 최고점 채택만으로 안정화되고, 못 가려내면 점수로는 못 고친다.
"""
import argparse, json
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[1]


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def cluster(Rs, thr=30.0):
    lab = [-1] * len(Rs)
    reps = []
    for i, R in enumerate(Rs):
        for k, r in enumerate(reps):
            if geo(r, R) < thr:
                lab[i] = k
                break
        else:
            reps.append(R)
            lab[i] = len(reps) - 1
    return np.array(lab), reps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe-results", required=True)
    a = ap.parse_args()
    P = [json.loads(l) for l in open(a.probe_results, encoding="utf-8") if '"R"' in l]
    P = [p for p in P if p["variant"] == "same"]
    by = defaultdict(list)
    for p in P:
        by[(p["stamp_ns"], p["object"])].append(p)

    pts, tree, diam = {}, {}, {}
    for f in sorted((REPO / "assets" / "model_points").glob("*.npy")):
        v = np.load(f).astype(np.float64)
        v = v[np.random.RandomState(0).choice(len(v), min(3000, len(v)), replace=False)]
        v = v - v.mean(0)
        pts[f.stem] = v
        tree[f.stem] = cKDTree(v)
        diam[f.stem] = float(np.linalg.norm(v.max(0) - v.min(0)))

    def sym_err(name, Rr):
        """R 을 걸었을 때 모델 점군이 자기 자신에 얼마나 겹치나 (지름 대비 %)."""
        v = pts[name]
        d, _ = tree[name].query(v @ Rr.T)
        return 100.0 * float(np.mean(d)) / diam[name]

    print("=" * 86)
    print("같은 입력 10회 반복 — 모드 분해")
    print("=" * 86)
    print(f"{'객체':>22} {'조합':>5} {'2모드이상':>9} {'주모드비율':>10} {'모드간각도':>11} "
          f"{'대칭오차%':>10} {'점수차':>8} {'최고점=주모드':>13}")
    agg = defaultdict(lambda: dict(n=0, multi=0, frac=[], ang=[], sym=[], dsc=[], pick=0, pickn=0))
    for (ns, nm), g in by.items():
        if len(g) < 5:
            continue
        Rs = [np.array(x["R"]) for x in g]
        lab, reps = cluster(Rs)
        sizes = np.bincount(lab)
        A = agg[nm]
        A["n"] += 1
        A["frac"].append(sizes.max() / len(Rs))
        if len(reps) > 1:
            A["multi"] += 1
            big, sec = np.argsort(sizes)[::-1][:2]
            Rrel = reps[big].T @ reps[sec]
            A["ang"].append(geo(reps[big], reps[sec]))
            A["sym"].append(sym_err(nm, Rrel))
            s1 = np.mean([g[i]["pose_score"] for i in range(len(g)) if lab[i] == big])
            s2 = np.mean([g[i]["pose_score"] for i in range(len(g)) if lab[i] == sec])
            A["dsc"].append(s1 - s2)
            best = int(np.argmax([x["pose_score"] for x in g]))
            A["pick"] += int(lab[best] == big)
            A["pickn"] += 1
    tot = dict(n=0, multi=0, sym=[], ang=[], dsc=[], pick=0, pickn=0, frac=[])
    for nm in sorted(agg):
        A = agg[nm]
        tot["n"] += A["n"]; tot["multi"] += A["multi"]; tot["pick"] += A["pick"]
        tot["pickn"] += A["pickn"]
        for k in ("sym", "ang", "dsc", "frac"):
            tot[k] += A[k]
        print(f"{nm:>22} {A['n']:>5} {A['multi']:>9} {np.mean(A['frac']):>10.2f} "
              f"{(np.median(A['ang']) if A['ang'] else float('nan')):>11.1f} "
              f"{(np.median(A['sym']) if A['sym'] else float('nan')):>10.1f} "
              f"{(np.median(A['dsc']) if A['dsc'] else float('nan')):>+8.3f} "
              f"{(A['pick']/A['pickn'] if A['pickn'] else float('nan')):>13.2f}")
    print("-" * 86)
    print(f"{'합계':>22} {tot['n']:>5} {tot['multi']:>9} {np.mean(tot['frac']):>10.2f} "
          f"{np.median(tot['ang']):>11.1f} {np.median(tot['sym']):>10.1f} "
          f"{np.median(tot['dsc']):>+8.3f} {tot['pick']/max(tot['pickn'],1):>13.2f}")

    print("\n[대칭 판정 기준] 회전을 걸어도 점군이 자기 자신에 겹치면(지름 대비 오차 ~1% 이하)")
    print("  두 해는 형상만으로는 구분 불가능한 '동치'다. 그보다 크면 진짜로 틀린 해다.")
    print("\n참고 — 각 물체가 실제로 가진 대칭 (모델 점군에 직접 회전을 걸어 측정, 지름 대비 %)")
    print(f"{'객체':>22} {'z180':>7} {'x180':>7} {'y180':>7} {'z90':>7} {'z45':>7} {'z10':>7}")
    def Rz(d):
        c, s = np.cos(np.radians(d)), np.sin(np.radians(d))
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
    Rx180 = np.diag([1., -1, -1]); Ry180 = np.diag([-1., 1, -1])
    for nm in sorted(pts):
        print(f"{nm:>22} {sym_err(nm, Rz(180)):>7.1f} {sym_err(nm, Rx180):>7.1f} "
              f"{sym_err(nm, Ry180):>7.1f} {sym_err(nm, Rz(90)):>7.1f} "
              f"{sym_err(nm, Rz(45)):>7.1f} {sym_err(nm, Rz(10)):>7.1f}")

    # 점수 기반 안정화가 실제로 얼마나 먹히나
    print("\n[점수로 고를 때] 반복 10회 중 pose_score 최고를 채택했을 때와 무작위 1회의 차이")
    rnd, best = [], []
    for (ns, nm), g in by.items():
        if len(g) < 5:
            continue
        Rs = [np.array(x["R"]) for x in g]
        lab, reps = cluster(Rs)
        sizes = np.bincount(lab)
        big = int(np.argmax(sizes))
        rnd.append(np.mean(lab != big))                       # 무작위 1회가 주모드를 벗어날 확률
        bi = int(np.argmax([x["pose_score"] for x in g]))
        best.append(float(lab[bi] != big))
    print(f"   무작위 1회가 주모드를 벗어날 확률  {np.mean(rnd):.2f}")
    print(f"   최고점 1회가 주모드를 벗어날 확률  {np.mean(best):.2f}   (n={len(best)})")


if __name__ == "__main__":
    main()
