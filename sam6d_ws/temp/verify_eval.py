#!/usr/bin/env python3
"""verify_eval.py — Stage V(텍스처 검증)를 통제된 조건에서 재고 판정한다.

**왜 bag 재생으로는 안 되나**: 실시간 재생은 프레임 유실이 매번 달라서 PEM 의 가설
추출 난수(torch.rand) 흐름이 달라진다. 그래서 같은 설정을 두 번 돌려도 포즈가 다르다
(실측: 공통 스탬프 260건 전부 불일치). 설정을 비교하려면 **같은 프레임을 같은 씨앗으로**
돌려야 한다. 이 스크립트는 bag 에서 프레임을 뽑아 메모리에 올린 뒤, 모델 하나로
설정만 바꿔 가며 같은 순서·같은 씨앗으로 돌린다.

기준 자세: GT 가 없으므로 **객체쌍 상대회전 합의**를 쓴다. 물체들은 선반에 고정돼
있으므로 R_i^T R_j 가 상수다. 이것을 전 프레임에서 군집해 (최빈 군집) 교정한 뒤,
한 프레임에서 **자기 자신을 뺀 나머지 객체**로 그 물체의 자세를 예측한다. 파트너 2개
이상이 30° 안에서 일치할 때만 기준으로 채택한다(A등급).

    conda activate sam6d
    python temp/verify_eval.py --bag <sam bag> --out temp/verify_eval --stride 7
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import collections
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, os.path.join(REPO, "realtime"))

COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
CAMINFO = "/camera/camera/color/camera_info"
SEED = 12345

_APPE = {"topk": 100, "w_geo": 2.0, "stride": 1}
# ⚠ 옛 세 설정에는 mask_gate / map_prior 를 **명시적으로 null** 로 둔다. 이것들은
#   verify_config 에서 기본 ON 이라, 키를 빼면 옛 설정이 조용히 게이트를 달고 돌아
#   "off" 가 더는 예전 동작이 아니게 된다.
_OLD = {"mask_gate": None, "map_prior": None}

SETTINGS = {
    # 이름 -> runtime 조각. appe_rerank 는 세 설정 모두 final3 운영값으로 고정한다.
    "off": {"appe_rerank": _APPE, "verify": None, **_OLD},
    "w0": {"appe_rerank": _APPE,
           "verify": {"enabled": True, "w_col": 0.0, "geo_guard": 0.0}, **_OLD},
    "on": {"appe_rerank": _APPE,
           "verify": {"enabled": True, "w_col": 1.0, "geo_guard": 0.9}, **_OLD},

    # ── 마스크 게이트(ch1) 파리티/운영 ────────────────────────────────────
    # mg_null   : 구조 중립. gate=None 이라 원래 텐서 연산이 그대로 돈다.
    # mg_parity : 배선 중립. 렌더·비교를 전부 태우고도 아무도 거르지 않는다.
    #             → mg_null 과 R/t 가 완전히 같아야 한다(게이트가 무해함의 증명).
    # mg_on     : 운영 기본값.
    "mg_null": {"appe_rerank": _APPE,
                "verify": {"enabled": True, "w_col": 1.0, "geo_guard": 0.9}, **_OLD},
    "mg_parity": {"appe_rerank": _APPE,
                  "verify": {"enabled": True, "w_col": 1.0, "geo_guard": 0.9},
                  "mask_gate": {"enabled": True, "gate_guard": 0.0, "min_keep": 300,
                                "topk": 300, "ism_reject": False},
                  "map_prior": None},
    "mg_on": {"appe_rerank": _APPE,
              "verify": {"enabled": True, "w_col": 1.0, "geo_guard": 0.9},
              "mask_gate": {"enabled": True},
              "map_prior": None},
}

# 대칭 선언(objectmemory core/fusion.py 의 보수적 표와 같다)
SYM_AXES = {"Sikhye_high": (0.0, 0.0, 1.0), "Sauce_high": (0.0, 0.0, 1.0),
            "Mugcup_high": (0.1057, 0.0337, 0.9938)}
SYM_STEP = 10


# ----------------------------------------------------------------- 회전 유틸
def _axis_rot(axis, deg):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    x, y, z = (c / n for c in axis)
    a = math.radians(deg)
    c, s, k = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return np.array([[c + x * x * k, x * y * k - z * s, x * z * k + y * s],
                     [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
                     [z * x * k - y * s, z * y * k + x * s, c + z * z * k]])


_SYM = {}


def sym_group(name):
    if name not in _SYM:
        ax = SYM_AXES.get(name)
        _SYM[name] = ([np.eye(3)] if ax is None else
                      [_axis_rot(ax, d) for d in range(0, 360, SYM_STEP)])
    return _SYM[name]


def ang_deg(A, B, name=None):
    """A 와 B 사이 각. 대칭이 선언되면 동치 자세 중 최소각(자세는 물체→카메라)."""
    best = 180.0
    for S in sym_group(name):
        tr = float(np.trace(A.T @ (B @ S)))
        best = min(best, math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))))
    return best


def rot_mean(mats):
    """회전 평균(자세 행렬 합의 극분해). 같은 봉우리 안에서만 쓸 것."""
    M = sum(mats)
    u, _, vt = np.linalg.svd(M)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1
        R = u @ vt
    return R


def mode_cluster(mats, tol, name=None, cap=400):
    """최빈 군집의 (평균, 점유율). 상대회전 교정에 쓴다.

    **메도이드** 방식이다. 예전의 탐욕 군집은 대표가 원소를 받을 때마다 움직여서,
    자세가 넓게 퍼진 경우 같은 봉우리가 여러 조각으로 쪼개졌다(260804 에서 점유율이
    11~49% 로 나온 주범). 여기서는 '자기 tol 안에 가장 많은 이웃을 가진 원소'를 찾아
    그 이웃들만 평균한다 — 순서에 무관하고 조각나지 않는다.
    """
    idx = (np.linspace(0, len(mats) - 1, cap).astype(int) if len(mats) > cap
           else np.arange(len(mats)))
    sub = [mats[i] for i in idx]
    best_i, best_n = 0, -1
    for i, A in enumerate(sub):
        n = sum(1 for B in sub if ang_deg(A, B, name) < tol)
        if n > best_n:
            best_i, best_n = i, n
    ctr = sub[best_i]
    grp = [R for R in mats if ang_deg(ctr, R, name) < tol]
    return rot_mean(grp), len(grp) / max(len(mats), 1)


# ------------------------------------------------- SLAM 궤적으로 기준 세우기
# 객체쌍 합의는 파트너가 2개 이상 동시에 보여야 하고, 대칭 물체는 파트너로 못 쓴다.
# 그래서 표본이 절반 이하로 줄고 세션에 따라 아예 성립하지 않는다(260804).
# 카메라 궤적이 있으면 훨씬 낫다: 물체가 정지해 있으므로 **지도좌표계 자세가 상수**다.
#   R_map_obj(t) = R_map_cam(t) · R_cam_obj(t)
# 전 프레임에서 이 값의 최빈 자세를 그 물체의 기준으로 삼고, 프레임마다 카메라계로 되돌린다.
# 파트너가 필요 없으므로 **모든 검출**에 기준이 생긴다.

def load_traj(path, extrinsic=None):
    """TUM (t tx ty tz qx qy qz qw) -> (times[N], R_map_cam[N,3,3], p_map_cam[N,3])."""
    from scipy.spatial.transform import Rotation
    ts, qs, ps = [], [], []
    for line in open(path, encoding="utf-8"):
        p = line.split()
        if len(p) < 8 or p[0].startswith("#"):
            continue
        ts.append(float(p[0]))
        ps.append([float(x) for x in p[1:4]])
        qs.append([float(x) for x in p[4:8]])
    R = Rotation.from_quat(np.asarray(qs)).as_matrix()
    P = np.asarray(ps)
    if extrinsic is not None:                 # SLAM 카메라 궤적 -> SAM 카메라
        X = np.asarray(extrinsic)
        P = P + R @ X[:3, 3]
        R = R @ X[:3, :3]
    return np.asarray(ts), R, P


def traj_at(ts, Rs, Ps, t, max_gap=0.05):
    """가장 가까운 자세·위치(간격이 max_gap 초를 넘으면 None). 30 Hz 라 보간 없이 충분하다."""
    i = int(np.searchsorted(ts, t))
    cand = [j for j in (i - 1, i) if 0 <= j < len(ts)]
    if not cand:
        return None, None
    j = min(cand, key=lambda j: abs(ts[j] - t))
    return (None, None) if abs(ts[j] - t) > max_gap else (Rs[j], Ps[j])


MULTI_INSTANCE = {"choco_hazelnut_high": 2}   # 실물 개수 (사용자 확인: choco 만 2개)


def map_mode_reference(cases, ts, Rs, Ps, tol=30.0, min_px=40, pos_tol=0.30, min_inst=15,
                       multi=None):
    """지도좌표계에서 **인스턴스별** 최빈 자세를 구하고, 사례마다 카메라계 기준을 채운다.

    ⚠ 물체 이름 하나에 실물이 하나라고 가정하면 안 된다 — 0807/185223 의 choco 는
    1.31 m 떨어진 **두 개**이고 방향이 서로 90° 다르다. 이름 단위로 최빈을 잡으면
    한쪽 인스턴스 전체가 '뒤집힘' 으로 잘못 집계된다(실측 gross 42%가 그렇게 나왔다).
    그래서 먼저 **지도좌표계 위치로 군집**해 인스턴스를 나눈 뒤 자세 최빈을 구한다.
    ⚠ 단, **실물이 두 개인 것은 choco 뿐**이다(사용자 확인). 나머지를 위치로 쪼개면
    위치 추정 산포가 가짜 인스턴스를 만든다(260804 Febreze 가 0.44 m 떨어진 두 덩어리로
    갈렸는데 시간대가 겹쳐 있어 드리프트가 아니라 산포다). 그래서 MULTI_INSTANCE 밖의
    물체는 강제로 하나로 묶는다.

    최빈 자세는 가까이서 크게 찍힌 검출만으로 정한다(bbox 짧은 변 >= min_px).
    한 방법에 치우치지 않도록 cases[i]['ref_src'] 의 자세를 모두 쓴다.
    """
    for c in cases:
        Rc, Pc = traj_at(ts, Rs, Ps, c["stamp"] / 1e9)
        c["Rmc"], c["Pmc"] = Rc, Pc
        c["Pmap"] = (None if Rc is None else
                     Rc @ (np.asarray(c["t_mm"], float) / 1000.0) + Pc)

    # 1) 이름별 위치 군집 = 인스턴스
    inst = defaultdict(list)
    for c in cases:
        if c["Pmap"] is not None:
            inst[c["object"]].append(c)
    modes, share = {}, {}
    for o, group in inst.items():
        P = np.array([c["Pmap"] for c in group])
        lab = -np.ones(len(P), int)
        k = 0
        for i in range(len(P)):
            if lab[i] >= 0:
                continue
            m = (np.linalg.norm(P - P[i], axis=1) < pos_tol) & (lab < 0)
            lab[m] = k
            k += 1
        tab = MULTI_INSTANCE if multi is None else multi
        want = int(tab.get(o, 1))
        if want <= 1:
            lab[:] = 0                        # 실물이 하나인 물체는 쪼개지 않는다
        else:
            # 실물 개수만큼만 남기고 나머지 조각은 가장 가까운 군집에 흡수시킨다.
            cnt = collections.Counter(lab.tolist())
            keep = [c for c, _ in cnt.most_common(want)]
            ctr = np.array([P[lab == c].mean(0) for c in keep])
            for i in range(len(P)):
                if lab[i] not in keep:
                    lab[i] = keep[int(np.linalg.norm(ctr - P[i], axis=1).argmin())]
            remap = {c: j for j, c in enumerate(keep)}
            lab = np.array([remap[x] for x in lab])
        for c, l in zip(group, lab):
            c["inst"] = int(l)
        # 2) 인스턴스별 자세 최빈
        for l in set(lab.tolist()):
            g = [c for c, x in zip(group, lab) if x == l]
            if len(g) < min_inst:
                continue
            src = []
            for c in g:
                x1, y1, x2, y2 = c["bbox"]
                if min(x2 - x1, y2 - y1) < min_px:
                    continue
                src += [c["Rmc"] @ np.asarray(R) for R in c.get("ref_src", [c["R"]])]
            if len(src) < min_inst:
                src = [c["Rmc"] @ np.asarray(R) for c in g for R in c.get("ref_src", [c["R"]])]
            M, sh = mode_cluster(src, tol, o)
            modes[(o, l)] = M
            share[(o, l)] = (sh, len(g))

    for c in cases:
        key = (c["object"], c.get("inst"))
        c["ref"] = (None if c["Rmc"] is None or key not in modes
                    else c["Rmc"].T @ modes[key])
    return modes, share


# ----------------------------------------------------------------- bag 읽기
def read_frames(bag, stride, limit, slop_ns=20_000_000):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image

    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
            rosbag2_py.ConverterOptions("", ""))
    K, frames, pend, deps = None, [], [], []
    n_color = 0

    def img(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        return a

    while rd.has_next():
        topic, data, _ = rd.read_next()
        if topic == CAMINFO and K is None:
            K = np.array(deserialize_message(data, CameraInfo).k, float).reshape(3, 3)
        elif topic == DEPTH:
            m = deserialize_message(data, Image)
            t = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            deps.append((t, np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)))
            del deps[:-40]
        elif topic == COLOR:
            n_color += 1
            if (n_color - 1) % stride:
                continue
            m = deserialize_message(data, Image)
            t = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            pend.append((t, img(m), m.encoding))
        # 대기 중인 컬러를 깊이와 짝지어 확정
        keep = []
        for t, a, enc in pend:
            hit = min(deps, key=lambda d: abs(d[0] - t)) if deps else None
            if hit and abs(hit[0] - t) <= slop_ns:
                bgr = a[:, :, ::-1] if enc == "rgb8" else a
                frames.append((t, np.ascontiguousarray(bgr), hit[1].copy()))
            elif deps and deps[-1][0] < t + slop_ns:
                keep.append((t, a, enc))          # 아직 깊이가 안 왔다
        pend = keep
        if limit and len(frames) >= limit:
            break
    print(f"[bag] 컬러 {n_color} 중 {len(frames)} 프레임 사용 (stride={stride})")
    return K, frames


# ----------------------------------------------------------------- 실행
def run(args):
    import torch
    from sam6d_core import Sam6DCore
    from verify_config import build_appe_cfg

    K, frames = read_frames(args.bag, args.stride, args.limit)
    if K is None or not frames:
        raise SystemExit("bag 에서 프레임/카메라 정보를 못 읽었다")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    core = Sam6DCore(args.config, [], args.device)
    for name in args.settings.split(","):
        rt = SETTINGS[name]
        ar, _ = build_appe_cfg(rt)
        core.pem.cfg.appe_rerank = ar
        print(f"[run] {name}: {ar}", flush=True)
        rows = []
        for i, (t, bgr, dep) in enumerate(frames):
            torch.manual_seed(SEED + i)
            np.random.seed(SEED + i)
            det, _ms, _nb, _ = core.process(bgr, dep, K)
            for d in det:
                rows.append({"stamp_ns": t, "i": i, **{k: d[k] for k in
                             ("object", "score", "R", "t_mm") if k in d},
                             **({"verify": d["verify"]} if "verify" in d else {}),
                             **({"gate": d["gate"]} if "gate" in d else {})})
            if i % 50 == 0:
                print(f"  {i}/{len(frames)}  누적검출 {len(rows)}", flush=True)
        (out / f"{name}.jsonl").write_text(
            "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
        print(f"[run] {name}: 검출 {len(rows)} → {out / (name + '.jsonl')}", flush=True)


# ----------------------------------------------------------------- 채점
def load(p):
    by = defaultdict(dict)
    for l in open(p, encoding="utf-8"):
        r = json.loads(l)
        by[r["i"]][r["object"]] = r
    return by


def calibrate(runs, tol=30.0, min_n=20):
    """상대회전 R_(p->t) = R_p^T R_t 를 최빈 군집으로 교정한다.

    ⚠ **대칭이 선언된 물체는 파트너로 쓰지 않는다.** 그 물체의 추정 자세는 동치 자세 중
    아무거나일 수 있어서, 파트너로 쓰면 예측에 그 애매함이 그대로 실린다(대상 쪽 애매함은
    마지막 각도 비교에서 접히므로 괜찮다). 그래서 키는 (비대칭 파트너, 대상) 순서쌍이다.
    """
    acc = defaultdict(list)
    for by in runs:
        for _i, objs in by.items():
            for p in objs:
                if p in SYM_AXES:                 # 대칭 물체는 파트너 자격 없음
                    continue
                Rp = np.array(objs[p]["R"])
                for t in objs:
                    if t == p:
                        continue
                    acc[(p, t)].append(Rp.T @ np.array(objs[t]["R"]))
    rel, share = {}, {}
    for (p, t), v in acc.items():
        if len(v) < min_n:
            continue
        M, sh = mode_cluster(v, tol, t)           # 대상 쪽 대칭만 접는다
        rel[(p, t)] = M
        share[(p, t)] = (sh, len(v))
    return rel, share


def reference(objs, target, rel, tol=30.0):
    """자기 자신을 뺀 나머지 객체로 target 의 자세를 예측. (기준, 파트너수)."""
    preds = []
    for p, r in objs.items():
        if p == target or p in SYM_AXES:
            continue
        if (p, target) in rel:
            preds.append(np.array(r["R"]) @ rel[(p, target)])
    if len(preds) < 2:
        return None, len(preds)
    # 서로 30° 안에서 뭉치는 최대 집합만 채택
    best = []
    for i, A in enumerate(preds):
        grp = [B for B in preds if ang_deg(A, B, target) < tol]
        if len(grp) > len(best):
            best = grp
    if len(best) < 2:
        return None, len(preds)
    return rot_mean(best), len(best)


def report(args):
    names = args.settings.split(",")
    runs = {n: load(Path(args.out) / f"{n}.jsonl") for n in names}
    rel, share = calibrate(list(runs.values()))
    print(f"[교정] 객체쌍 {len(rel)}쌍")
    for k in sorted(share, key=lambda k: -share[k][1])[:8]:
        print(f"    {k[0]:24s} {k[1]:24s} 최빈군집 {share[k][0]*100:5.1f}%  n={share[k][1]}")

    err = {n: {} for n in names}
    for n in names:
        for i, objs in runs[n].items():
            for o in objs:
                # 기준은 **OFF 설정의 파트너 자세**로 세운다 — 설정마다 기준이 흔들리면
                # 비교가 안 된다. 대상 물체 자신은 그 설정의 값을 쓴다.
                base = runs[names[0]].get(i, {})
                part = {k: v for k, v in base.items() if k != o}
                if len(part) < 2:
                    continue
                ref, npart = reference({**part, o: objs[o]}, o, rel)
                if ref is None:
                    continue
                err[n][(i, o)] = (ang_deg(ref, np.array(objs[o]["R"]), o), npart,
                                  objs[o].get("verify"))
    print()
    print(f"{'설정':6s} {'평가건수':>7s} {'중앙°':>7s} {'<15°':>7s} {'>45°(gross)':>12s}")
    for n in names:
        e = [v[0] for v in err[n].values()]
        if not e:
            print(f"{n:6s}  (평가 가능한 건이 없다)")
            continue
        e = np.array(e)
        print(f"{n:6s} {len(e):7d} {np.median(e):7.1f} {100*(e<15).mean():6.1f}% "
              f"{100*(e>45).mean():11.1f}%")

    base = names[0]
    for n in names[1:]:
        com = set(err[base]) & set(err[n])
        fix = sum(1 for k in com if err[base][k][0] > 45 and err[n][k][0] <= 45)
        brk = sum(1 for k in com if err[base][k][0] <= 45 and err[n][k][0] > 45)
        chg = sum(1 for k in com if abs(err[base][k][0] - err[n][k][0]) > 1e-6)
        gross0 = sum(1 for k in com if err[base][k][0] > 45)
        print(f"\n[{base} -> {n}] 공통 {len(com)}건 · 답이 바뀐 비율 {100*chg/max(len(com),1):.1f}%"
              f" · 뒤집힘 교정 {fix} : 파손 {brk}"
              f" (기준 뒤집힘 {gross0}건 중 {100*fix/max(gross0,1):.0f}% 회복)")

    for n in names[1:]:
        vd = defaultdict(list)
        for k, v in err[n].items():
            if v[2]:
                vd[v[2]["verdict"]].append(v[0])
        if not vd:
            continue
        print(f"\n[{n}] 판정별 실제 오차")
        for k in ("PASS", "CORRECTED", "AMBIGUOUS"):
            if k not in vd:
                continue
            a = np.array(vd[k])
            print(f"    {k:10s} n={len(a):4d}  중앙 {np.median(a):5.1f}°  "
                  f"<15° {100*(a<15).mean():5.1f}%  >45° {100*(a>45).mean():5.1f}%")
        conf = [(err[n][k][2]["conf"], err[n][k][0]) for k in err[n] if err[n][k][2]]
        if conf:
            conf.sort()
            print(f"    신뢰도 구간별 gross:", end=" ")
            for lo, hi in ((0.0, .55), (.55, .7), (.7, .85), (.85, 1.01)):
                s = [e for c, e in conf if lo <= c < hi]
                if s:
                    print(f"[{lo:.2f},{hi:.2f}) {100*np.mean(np.array(s) > 45):.0f}%"
                          f"(n={len(s)})", end="  ")
            print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default="")
    ap.add_argument("--out", default="temp/verify_eval")
    ap.add_argument("--config", default="configs/yolo_ism_objects.yaml")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--stride", type=int, default=7)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--settings", default="off,w0,on")
    ap.add_argument("--report", action="store_true", help="이미 만든 jsonl 로 채점만")
    a = ap.parse_args()
    if not a.report:
        if not a.bag:
            raise SystemExit("--bag 이 필요하다")
        run(a)
    report(a)


if __name__ == "__main__":
    main()
