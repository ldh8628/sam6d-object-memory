#!/usr/bin/env python3
"""analyze_pem_symmetry_split.py — 실시간에서 관측된 흔들림을 두 갈래로 나눈다.

  (a) 형상 대칭으로 인한 '동치 해' 사이의 뛰기  — 기하만으로는 원리적으로 구분 불가
  (b) 그냥 틀린 해                              — 구분 가능한데 틀린 것

판정: 두 추정의 상대회전 R_rel 을 그 물체의 모델 점군에 걸어 자기 자신과 겹쳐 본다.
겹침 오차를 **무작위 회전을 걸었을 때의 오차**로 나눠 정규화한다(물체마다 점군이
뭉툭한 정도가 달라 절대값 비교는 부당하다). 0 에 가까우면 대칭, 1 에 가까우면 남남.
"""
import argparse, json
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    a = ap.parse_args()
    pts, tree, base = {}, {}, {}
    rs = np.random.RandomState(0)
    for f in sorted((REPO / "assets" / "model_points").glob("*.npy")):
        v = np.load(f).astype(np.float64)
        v = v[rs.choice(len(v), min(3000, len(v)), replace=False)]
        v -= v.mean(0)
        pts[f.stem], tree[f.stem] = v, cKDTree(v)
    def err(nm, R):
        d, _ = tree[nm].query(pts[nm] @ R.T)
        return float(np.mean(d))
    for nm in pts:                       # 무작위 회전 기준선
        base[nm] = np.median([err(nm, Rotation.random(random_state=i).as_matrix())
                              for i in range(40)])

    D = [json.loads(l) for l in open(Path(a.run) / "detections.jsonl", encoding="utf-8")]
    seq = defaultdict(list)
    for d in D:
        seq[d["object"]].append(d)
    print("실시간 연속 추정 쌍(간격 1.5s 이내) — 큰 회전(>90°)이 대칭인가 오답인가")
    print(f"{'객체':>22} {'>90°쌍':>7} {'정규화겹침':>11} {'대칭(≤0.35)':>12} {'오답(>0.35)':>12}")
    tot = [0, 0]
    for nm in sorted(seq):
        if nm not in pts:
            continue
        lst = sorted(seq[nm], key=lambda d: d["stamp_ns"])
        rr = []
        for x, y in zip(lst, lst[1:]):
            if (y["stamp_ns"] - x["stamp_ns"]) * 1e-9 > 1.5:
                continue
            R1, R2 = np.array(x["R"]), np.array(y["R"])
            if geo(R1, R2) <= 90:
                continue
            rr.append(err(nm, R1.T @ R2) / base[nm])
        if not rr:
            continue
        rr = np.array(rr)
        tot[0] += int((rr <= .35).sum()); tot[1] += int((rr > .35).sum())
        print(f"{nm:>22} {len(rr):>7} {np.median(rr):>11.2f} "
              f"{(rr<=.35).sum():>12} {(rr>.35).sum():>12}")
    print("-" * 70)
    print(f"{'합계':>22} {tot[0]+tot[1]:>7} {'':>11} {tot[0]:>12} {tot[1]:>12}")
    print("\n물체별 '이 회전은 대칭인가' 참고표 (정규화 겹침오차, 0=완전대칭 1=남남)")
    def Rz(d):
        c, s = np.cos(np.radians(d)), np.sin(np.radians(d))
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
    print(f"{'객체':>22} {'z180':>7} {'z90':>7} {'x180':>7} {'y180':>7}")
    for nm in sorted(pts):
        print(f"{nm:>22} {err(nm,Rz(180))/base[nm]:>7.2f} {err(nm,Rz(90))/base[nm]:>7.2f} "
              f"{err(nm,np.diag([1.,-1,-1]))/base[nm]:>7.2f} "
              f"{err(nm,np.diag([-1.,1,-1]))/base[nm]:>7.2f}")


if __name__ == "__main__":
    main()
