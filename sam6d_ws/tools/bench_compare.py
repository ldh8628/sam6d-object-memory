#!/usr/bin/env python3
"""bench_compare.py — 두 기계의 bench_offline 결과를 비교한다 (속도 + 판정 동일성)."""
import argparse, json

ap = argparse.ArgumentParser()
ap.add_argument("base"); ap.add_argument("other")
a = ap.parse_args()
A, B = json.load(open(a.base)), json.load(open(a.other))
print(f"기준 : {A['machine']['gpu']}  (torch {A['machine']['torch']}, cuda {A['machine']['cuda']})")
print(f"비교 : {B['machine']['gpu']}  (torch {B['machine']['torch']}, cuda {B['machine']['cuda']})")
print(f"\n{'단계':>7} {'기준 중앙':>9} {'비교 중앙':>9} {'배율':>7} | {'기준 최대':>9} {'비교 최대':>9}")
for k in ("yolo", "ism", "pem", "total"):
    x, y = A["stage_ms"][k], B["stage_ms"][k]
    r = y["median"] / max(x["median"], 1e-9)
    print(f"{k:>7} {x['median']:>9.0f} {y['median']:>9.0f} {r:>6.2f}x | "
          f"{x['max']:>9.0f} {y['max']:>9.0f}")
print(f"\n처리율 {A['hz']:.1f} Hz → {B['hz']:.1f} Hz  ({B['hz']/max(A['hz'],1e-9):.2f}x)")
for tag, S in (("기준", A), ("비교", B)):
    g0, g1 = S["gpu_start"], S["gpu_end"]
    print(f"{tag}: GPU 클럭 {g0.get('sm_mhz')} → {g1.get('sm_mhz')} (최대 {g0.get('sm_max_mhz')}), "
          f"전력 {g0.get('power_w')} → {g1.get('power_w')} / {g0.get('power_limit_w')}, "
          f"온도 {g0.get('temp_c')} → {g1.get('temp_c')}")
da, db = A.get("decisions", {}), B.get("decisions", {})
common = sorted(set(da) & set(db))
if common:
    same = sum(1 for k in common
               if [d["object"] for d in da[k]] == [d["object"] for d in db[k]])
    print(f"\n판정 동일성: 공통 프레임 {len(common)}장 중 객체 집합 일치 {same}장 "
          f"({same*100//max(len(common),1)}%)")
    for k in common:
        oa = [d["object"] for d in da[k]]; ob = [d["object"] for d in db[k]]
        if oa != ob:
            print(f"   차이 {k}: {oa}  vs  {ob}")
else:
    print("\n판정 비교: 공통 프레임이 없다(같은 --frames 로 돌려야 비교된다)")
