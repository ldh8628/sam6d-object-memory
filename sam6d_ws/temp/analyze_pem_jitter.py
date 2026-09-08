#!/usr/bin/env python3
"""analyze_pem_jitter.py — 실시간 실행 로그에서 (1) 시간 성능 (2) PEM 흔들림의 원인을 가른다.

원인 후보를 **서로 배타적인 증거로** 나눈다.
  H1 내부 무작위성   : get_test_data 가 관측점 2048·모델점 1024 를 매번 무작위 표본추출한다
  H2 마스크 경계     : ISM 마스크가 조금만 달라져도 포즈가 바뀐다
  H3 대칭 모호성     : 물체가 대칭이라 180° 등 동치 해가 존재한다 (평균으로 못 없앤다)
  H4 관측 빈약       : 유효 depth 점이 적거나 멀어서 기하 제약이 약하다
  H5 시점 변화       : 두 추정 사이에 카메라가 움직여 '진짜로' 다른 문제를 푼 것

H1·H2 는 pem_probe.py 가 만든 통제 실험으로 직접 측정하고,
H3~H5 는 실시간 로그 안에서 조건을 나눠 본다.
"""
import argparse, json, math
from collections import defaultdict
from pathlib import Path

import numpy as np


def geo(A, B):
    c = np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


def axis_angle(Rr):
    ang = np.degrees(np.arccos(np.clip((np.trace(Rr) - 1) / 2, -1, 1)))
    w, v = np.linalg.eig(Rr)
    ax = np.real(v[:, np.argmin(np.abs(w - 1))])
    return ang, ax / (np.linalg.norm(ax) + 1e-12)


def q(a, p):
    return float(np.percentile(a, p)) if len(a) else float("nan")


def sec(t):
    print("\n" + "=" * 78 + f"\n{t}\n" + "=" * 78)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--probe-results", default="")
    a = ap.parse_args()
    run = Path(a.run)
    F = [json.loads(l) for l in open(run / "frames.jsonl", encoding="utf-8")]
    D = [json.loads(l) for l in open(run / "detections.jsonl", encoding="utf-8")]
    meta = json.load(open(run / "run_meta.json", encoding="utf-8"))

    # ---------------------------------------------------------------- 1. 시간
    sec("1. 시간 성능")
    s = meta.get("summary", {})
    print(f"들어온 프레임 {s.get('frames_in')} · 처리 {s.get('frames_processed')} · "
          f"건너뜀 {s.get('frames_skipped')} · 검출 {s.get('detections')} · "
          f"{s.get('elapsed_s')}s ({s.get('hz_processed')} Hz)")
    tot = np.array([f["ms"]["total"] for f in F], float)
    print(f"프레임당 총 시간  중앙 {np.median(tot):.0f} ms · p90 {q(tot,90):.0f} · 최대 {tot.max():.0f}")
    print(f"{'객체수':>5} {'n':>5} {'total':>8} {'yolo':>7} {'ism':>7} {'pem':>8}   (중앙 ms)")
    for k in range(0, 8):
        g = [f for f in F if f["n_accept"] == k]
        if not g:
            continue
        print(f"{k:>5} {len(g):>5} "
              f"{np.median([x['ms']['total'] for x in g]):>8.0f} "
              f"{np.median([x['ms']['yolo'] for x in g]):>7.0f} "
              f"{np.median([x['ms']['ism'] for x in g]):>7.0f} "
              f"{np.median([x['ms']['pem'] for x in g]):>8.0f}")
    pem = np.array([f["ms"]["pem"] for f in F if f["n_accept"] > 0], float)
    npc = np.array([f["n_accept"] for f in F if f["n_accept"] > 0], float)
    if len(pem) > 2:
        A = np.vstack([npc, np.ones_like(npc)]).T
        k, b = np.linalg.lstsq(A, pem, rcond=None)[0]
        print(f"PEM 시간 ≈ {b:.0f} ms + {k:.0f} ms × 객체수  (객체마다 따로 도는 구조)")
    lat = np.array([(f["t_done_ns"] - f["stamp_ns"]) * 1e-9 for f in F if f.get("t_done_ns")])
    print(f"촬영→결과 지연  중앙 {np.median(lat):.2f}s · p90 {q(lat,90):.2f}s · 최대 {lat.max():.2f}s")
    gap = np.diff(sorted(f["t_done_ns"] * 1e-9 for f in F if f.get("t_done_ns")))
    print(f"결과 갱신 간격   중앙 {np.median(gap):.2f}s · p90 {q(gap,90):.2f}s · 최대 {gap.max():.2f}s")
    nb = np.array([f["n_boxes"] for f in F], float)
    ism = np.array([f["ms"]["ism"] for f in F], float)
    print(f"ISM 시간 vs YOLO 박스 수 상관 {np.corrcoef(nb, ism)[0,1]:.2f} "
          f"(박스 중앙 {np.median(nb):.0f}, 최대 {nb.max():.0f})")

    # ---------------------------------------------------------------- 2. 흔들림
    sec("2. 흔들림의 크기 — 같은 객체의 연속 추정 사이")
    seq = defaultdict(list)
    for d in D:
        seq[d["object"]].append(d)
    pairs = []
    for nm, lst in seq.items():
        lst.sort(key=lambda d: d["stamp_ns"])
        for x, y in zip(lst, lst[1:]):
            dt = (y["stamp_ns"] - x["stamp_ns"]) * 1e-9
            R1, R2 = np.array(x["R"]), np.array(y["R"])
            pairs.append({"obj": nm, "dt": dt, "drot": geo(R1, R2),
                          "dpos": float(np.linalg.norm(np.array(y["t_mm"]) - np.array(x["t_mm"]))),
                          "R1": R1, "R2": R2, "a": x, "b": y})
    print(f"쌍 {len(pairs)} 개")
    print(f"{'간격':>12} {'n':>5} {'Δrot중앙':>9} {'p90':>7} {'>30°':>6} {'>150°':>6} {'Δpos중앙mm':>11}")
    for lo, hi, lab in [(0, .5, "≤0.5s"), (.5, 1.5, "0.5~1.5s"), (1.5, 5, "1.5~5s"), (5, 1e9, ">5s")]:
        g = [p for p in pairs if lo <= p["dt"] < hi]
        if not g:
            continue
        dr = np.array([p["drot"] for p in g])
        print(f"{lab:>12} {len(g):>5} {np.median(dr):>9.1f} {q(dr,90):>7.1f} "
              f"{(dr>30).mean():>6.2f} {(dr>150).mean():>6.2f} "
              f"{np.median([p['dpos'] for p in g]):>11.0f}")
    print(f"\n{'객체':>22} {'n':>4} {'Δrot중앙':>9} {'p90':>7} {'>150°':>6}  (간격 1.5s 이내만)")
    near = [p for p in pairs if p["dt"] < 1.5]
    for nm in sorted({p["obj"] for p in near}):
        g = [p for p in near if p["obj"] == nm]
        dr = np.array([p["drot"] for p in g])
        print(f"{nm:>22} {len(g):>4} {np.median(dr):>9.1f} {q(dr,90):>7.1f} {(dr>150).mean():>6.2f}")

    # ---------------------------------------------------------------- 3. H3 대칭
    sec("3. H3 대칭 모호성 — 흔들림이 특정 각도에 뭉치나")
    dr = np.array([p["drot"] for p in near])
    hist, edges = np.histogram(dr, bins=[0, 10, 20, 30, 60, 90, 120, 150, 165, 175, 180.01])
    for i in range(len(hist)):
        bar = "#" * int(40 * hist[i] / max(hist.max(), 1))
        print(f"{edges[i]:>5.0f}~{edges[i+1]:<5.0f} {hist[i]:>4}  {bar}")
    print("\n큰 회전(>120°)일 때 상대회전의 축이 물체 좌표계에서 어디를 향하나:")
    print(f"{'객체':>22} {'n':>4} {'|축·x|':>7} {'|축·y|':>7} {'|축·z|':>7} {'각도중앙':>9}")
    for nm in sorted({p["obj"] for p in near}):
        g = [p for p in near if p["obj"] == nm and p["drot"] > 120]
        if len(g) < 3:
            continue
        AX, AN = [], []
        for p in g:
            ang, ax = axis_angle(p["R1"].T @ p["R2"])     # 물체 좌표계 기준
            AX.append(np.abs(ax)); AN.append(ang)
        AX = np.array(AX)
        print(f"{nm:>22} {len(g):>4} {np.median(AX[:,0]):>7.2f} {np.median(AX[:,1]):>7.2f} "
              f"{np.median(AX[:,2]):>7.2f} {np.median(AN):>9.1f}")

    # ---------------------------------------------------------------- 4. H4/H5
    sec("4. H4 관측 빈약 / H5 시점 변화 — 무엇과 같이 움직이나")
    def getp(d, k, dflt=np.nan):
        return (d.get("pem") or {}).get(k, dflt)
    rows = [(p["drot"], min(getp(p["a"], "pose_score", np.nan), getp(p["b"], "pose_score", np.nan)),
             min(getp(p["a"], "mask_px", np.nan), getp(p["b"], "mask_px", np.nan)),
             min(getp(p["a"], "depth_valid_frac", np.nan), getp(p["b"], "depth_valid_frac", np.nan)),
             np.nanmean([getp(p["a"], "z_med_mm", np.nan), getp(p["b"], "z_med_mm", np.nan)]),
             p["dpos"], p["dt"])
            for p in near]
    M = np.array([r for r in rows if not np.isnan(r[1])], float)
    if len(M) > 10:
        cols = ["pose_score", "mask_px", "depth_valid_frac", "z_med_mm", "Δpos", "dt"]
        print("Δrot 과의 상관계수 (간격 1.5s 이내):")
        for j, c in enumerate(cols, start=1):
            v = M[:, j]
            ok = ~np.isnan(v)
            print(f"   {c:>18} {np.corrcoef(M[ok,0], v[ok])[0,1]:+.2f}")
        print("\npose_score 구간별:")
        print(f"{'pose_score':>14} {'n':>5} {'Δrot중앙':>9} {'>150°':>6}")
        for lo, hi in [(0, .3), (.3, .5), (.5, .7), (.7, 1.01)]:
            g = M[(M[:, 1] >= lo) & (M[:, 1] < hi)]
            if len(g):
                print(f"   {lo:.1f}~{hi:.1f}    {len(g):>5} {np.median(g[:,0]):>9.1f} "
                      f"{(g[:,0]>150).mean():>6.2f}")
        print("\n거리 구간별 (z 중앙, mm):")
        print(f"{'z':>14} {'n':>5} {'Δrot중앙':>9} {'>150°':>6}")
        for lo, hi in [(0, 800), (800, 1500), (1500, 2500), (2500, 1e9)]:
            g = M[(M[:, 4] >= lo) & (M[:, 4] < hi)]
            if len(g):
                print(f"   {lo:.0f}~{hi:.0f}   {len(g):>5} {np.median(g[:,0]):>9.1f} "
                      f"{(g[:,0]>150).mean():>6.2f}")

    # ---------------------------------------------------------------- 5. probe
    if a.probe_results and Path(a.probe_results).is_file():
        sec("5. H1 내부 무작위성 / H2 마스크 경계 — 통제 실험")
        P = [json.loads(l) for l in open(a.probe_results, encoding="utf-8")]
        P = [p for p in P if "R" in p]
        byk = defaultdict(list)
        for p in P:
            byk[(p["stamp_ns"], p["object"], p["variant"])].append(p)
        # H1: 같은 입력 반복
        spreads, flips, tspreads = [], [], []
        for (ns, nm, var), g in byk.items():
            if var != "same" or len(g) < 3:
                continue
            Rs = [np.array(x["R"]) for x in g]
            dd = [geo(Rs[i], Rs[j]) for i in range(len(Rs)) for j in range(i + 1, len(Rs))]
            spreads.append((nm, np.median(dd), max(dd), np.mean(np.array(dd) > 150)))
            ts = np.array([x["t_mm"] for x in g])
            tspreads.append(float(np.linalg.norm(ts.std(0))))
        if spreads:
            md = np.array([s[1] for s in spreads]); mx = np.array([s[2] for s in spreads])
            fl = np.array([s[3] for s in spreads])
            print(f"같은 프레임·같은 마스크를 반복 입력 ({len(spreads)} 개 (프레임,객체) 조합)")
            print(f"   반복 간 Δrot 중앙 {np.median(md):.1f}° · 상위10% {q(md,90):.1f}° · 최대 {mx.max():.1f}°")
            print(f"   >150° 로 뒤집히는 조합 비율 {(fl>0).mean():.2f}")
            print(f"   위치 표준편차 중앙 {np.median(tspreads):.1f} mm")
            print(f"\n{'객체':>22} {'조합':>5} {'반복Δrot중앙':>13} {'최대':>7} {'뒤집힘조합':>10}")
            for nm in sorted({s[0] for s in spreads}):
                g = [s for s in spreads if s[0] == nm]
                print(f"{nm:>22} {len(g):>5} {np.median([x[1] for x in g]):>13.1f} "
                      f"{max(x[2] for x in g):>7.1f} {np.mean([x[3]>0 for x in g]):>10.2f}")
        # H2: 마스크 섭동
        print("\n마스크를 깎거나 불렸을 때 (기준 = 같은 입력 반복의 중앙 포즈)")
        print(f"{'변형':>10} {'n':>5} {'Δrot중앙':>9} {'p90':>7} {'>150°':>6} {'Δpos중앙mm':>11}")
        base = {}
        for (ns, nm, var), g in byk.items():
            if var == "same":
                Rs = [np.array(x["R"]) for x in g]
                d = [np.median([geo(Rs[i], Rs[j]) for j in range(len(Rs)) if j != i])
                     for i in range(len(Rs))]
                base[(ns, nm)] = (Rs[int(np.argmin(d))], np.array(g[int(np.argmin(d))]["t_mm"]))
        for var in ["erode1", "erode2", "erode3", "dilate1", "dilate2", "dilate3"]:
            dr, dp = [], []
            for (ns, nm, v), g in byk.items():
                if v != var or (ns, nm) not in base:
                    continue
                Rb, tb = base[(ns, nm)]
                dr.append(geo(Rb, np.array(g[0]["R"])))
                dp.append(float(np.linalg.norm(np.array(g[0]["t_mm"]) - tb)))
            if dr:
                dr = np.array(dr)
                print(f"{var:>10} {len(dr):>5} {np.median(dr):>9.1f} {q(dr,90):>7.1f} "
                      f"{(dr>150).mean():>6.2f} {np.median(dp):>11.0f}")


if __name__ == "__main__":
    main()
