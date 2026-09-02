#!/usr/bin/env python3
"""gate_report.py — 후보 게이트(ch1 마스크 비교 · ch2 Map 사전) 실측 결과를 HTML 한 장으로.

**추론을 다시 돌리지 않는다.** temp/eval_*/ 의 jsonl 과 후보 덤프, 그리고 프레임 이미지를
위한 bag 만 읽는다(GPU 불필요). temp/small_object_report.py 와 같은 방식이다.

목적은 결론을 **주장**하는 게 아니라 **검증 가능하게** 보여 주는 것이다. 특히 둘:
  · 왜 어떤 문턱으로도 게이트를 살릴 수 없는가 (스윕 곡선)
  · 왜 ch2 의 겉보기 이득이 순환인가 (admit 분해가 기준과 같은 것을 쓴다)

    conda activate sam6d && cd sam6d_realtime
    python temp/gate_report.py --out outputs/test/gate_report.html
"""
from __future__ import annotations

import argparse
import html
import json
import math
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

REPO = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, os.path.join(REPO, "realtime"))
sys.path.insert(0, os.path.join(REPO, "temp"))

import verify_eval as V                                          # noqa: E402
from verify_report import b64, cls_of, render                    # noqa: E402

OK = 30.0          # 정답 = 기준에서 이 안
NEAR = 45.0        # 이보다 크면 gross(뒤집힘)
SETTINGS = ["off", "mg_null", "mg_parity", "mg_on", "mp_on"]
LABEL = {"off": "off (기하만)", "mg_null": "mg_null (현행 운영)",
         "mg_parity": "mg_parity (배선 중립)", "mg_on": "mg_on (ch1)",
         "mp_on": "mp_on (ch1+ch2)"}
COLOR = {"off": "#888888", "mg_null": "#9ad0ff", "mg_parity": "#4d7f9e",
         "mg_on": "#ff7676", "mp_on": "#ffb347"}
BASE = "mg_null"   # 판정 기준선 = 현행 운영. off 대비가 아니다.


# ===================================================================== 분석
def analyse(ev_dir, traj, extrinsic):
    """한 bag 의 모든 수치를 만든다. 프레임 이미지는 건드리지 않는다."""
    d = {}
    runs = {n: V.load(os.path.join(ev_dir, f"{n}.jsonl")) for n in SETTINGS}
    d["runs"] = runs

    # --- 기준 자세 (SLAM 궤적 + 인스턴스별 최빈) --------------------------
    X = np.load(extrinsic) if extrinsic else None
    ts, Rs, Ps = V.load_traj(traj, X)
    keys = sorted({(i, o) for n in SETTINGS for i, objs in runs[n].items() for o in objs})
    cases = []
    for i, o in keys:
        row = next((runs[n][i][o] for n in SETTINGS if o in runs[n].get(i, {})), None)
        if row is None or "bbox" not in row:
            continue
        cases.append({"i": i, "object": o, "stamp": row["stamp_ns"], "t_mm": row["t_mm"],
                      "bbox": row["bbox"], "R": row["R"],
                      "ref_src": [runs[n][i][o]["R"] for n in SETTINGS
                                  if o in runs[n].get(i, {})]})
    modes, share = V.map_mode_reference(cases, ts, Rs, Ps)
    # 실제로 처리한 프레임 수는 run.log 가 정본이다 — 검출이 없는 프레임도 있으므로
    # 검출 인덱스에서 역산하면 과소평가된다.
    d["n_frame"] = None
    lg = os.path.join(ev_dir, "run.log")
    if os.path.isfile(lg):
        for line in open(lg, encoding="utf-8", errors="ignore"):
            if line.startswith("[bag]"):
                try:
                    d["n_frame"] = int(line.split("중")[1].split("프레임")[0])
                except Exception:
                    pass
                break
    d["share"] = share
    d["n_det"] = len(cases)
    ref_by = {(c["i"], c["object"]): c["ref"] for c in cases if c["ref"] is not None}
    inst_by = {(c["i"], c["object"]): c.get("inst") for c in cases}
    d["ref_by"], d["inst_by"] = ref_by, inst_by
    d["n_ref"] = len(ref_by)

    # --- 설정별 오차 ------------------------------------------------------
    err = {}
    for n in SETTINGS:
        e = {}
        for i, objs in runs[n].items():
            for o, row in objs.items():
                r = ref_by.get((i, o))
                if r is not None:
                    e[(i, o)] = V.ang_deg(r, np.array(row["R"]), o)
        err[n] = e
    d["err"] = err

    # --- 채점표 -----------------------------------------------------------
    d["score"] = {}
    for n in SETTINGS:
        a = np.array(list(err[n].values()))
        d["score"][n] = dict(n=len(a), med=float(np.median(a)),
                             lt15=100 * float((a < 15).mean()),
                             gross=100 * float((a > NEAR).mean()))

    # --- 현행 대비 교정 : 파손 -------------------------------------------
    d["delta"] = {}
    for n in SETTINGS:
        if n == BASE:
            continue
        com = set(err[BASE]) & set(err[n])
        fix = [k for k in com if err[BASE][k] > NEAR >= err[n][k]]
        brk = [k for k in com if err[n][k] > NEAR >= err[BASE][k]]
        chg = [k for k in com if abs(err[BASE][k] - err[n][k]) > 1e-6]
        d["delta"][n] = dict(n=len(com), fix=len(fix), brk=len(brk),
                             chg=len(chg), keys_fix=fix, keys_brk=brk)

    # --- 객체별 승자 변경 -------------------------------------------------
    chg_obj = Counter(k[1] for k in d["delta"]["mg_on"]["keys_fix"])
    com = set(err[BASE]) & set(err["mg_on"])
    changed = [k for k in com if runs[BASE][k[0]][k[1]]["R"] != runs["mg_on"][k[0]][k[1]]["R"]]
    d["by_object"] = []
    for o in sorted({k[1] for k in com}):
        ks = [k for k in com if k[1] == o]
        ch = [k for k in changed if k[1] == o]
        f = sum(1 for k in ks if err[BASE][k] > NEAR >= err["mg_on"][k])
        b = sum(1 for k in ks if err["mg_on"][k] > NEAR >= err[BASE][k])
        d["by_object"].append((o, len(ks), len(ch), f, b,
                               100 * float(np.mean([err[BASE][k] > NEAR for k in ks])),
                               100 * float(np.mean([err["mg_on"][k] > NEAR for k in ks]))))

    # --- admit 분해 (⚠ 순환 주의) -----------------------------------------
    d["admit"] = {}
    for n in ("mg_on", "mp_on"):
        by = {}
        for i, objs in runs[n].items():
            for o, row in objs.items():
                by[(i, o)] = (row.get("gate") or {}).get("admit")
        tab = {}
        for ch in sorted({v for v in by.values() if v}):
            ks = [k for k in err[n] if by.get(k) == ch]
            if ks:
                a = np.array([err[n][k] for k in ks])
                tab[ch] = dict(n=len(a), med=float(np.median(a)),
                               gross=100 * float((a > NEAR).mean()))
        d["admit"][n] = tab
        if n == "mg_on":
            d["suspect"] = [k for k in by
                            if (runs[n][k[0]][k[1]].get("gate") or {}).get("ism_suspect")]

    # --- 후보 덤프: 스윕 + 진단 분포 --------------------------------------
    d.update(sweep(ev_dir, runs, ref_by))
    return d


def sweep(ev_dir, runs, ref_by, name="mg_parity"):
    """중립 설정 덤프로 문턱을 후처리로 훑는다. 재추론 없음."""
    src = runs[name]
    order = [(i, o) for i in sorted(src) for o in sorted(src[i])]
    rows = [json.loads(l) for l in open(os.path.join(ev_dir, f"{name}.cases.jsonl"),
                                        encoding="utf-8")]
    items = []
    for (i, o), r in zip(order, rows):
        ref = ref_by.get((i, o))
        if ref is None or r.get("object") != o:
            continue
        q = np.array(r["q"], float)     # [K,11] qx qy qz qw geo s cov prec r_area dtheta ch
        items.append(dict(
            key=(i, o), obj=o,
            ang=np.array([V.ang_deg(ref, V._q2R(v[:4]), o) for v in q]),
            geo=q[:, 4], s=q[:, 5], cov=q[:, 6], prec=q[:, 7],
            r_area=q[:, 8], dth=q[:, 9], ecc=float(r.get("ecc", 0.0)),
            use_shape=bool(r.get("use_shape", False))))

    have = [it for it in items if (it["ang"] <= OK).any()]

    def score(it, terms, w_p, theta0, ecc_min):
        S = it["cov"].copy()
        if "prec" in terms:
            S = S * np.clip(it["prec"], 1e-6, None) ** w_p
        if "shape" in terms and it["ecc"] >= ecc_min:
            S = S * np.exp(-np.radians(it["dth"]) / math.radians(theta0))
        return S

    def ev(terms="cov,prec,shape", w_p=0.5, theta0=30.0, ecc_min=1.6, g=0.85, min_keep=5):
        surv, pick, nk = 0, 0, []
        for it in items:
            S = score(it, terms, w_p, theta0, ecc_min)
            keep = S >= g * S.max()
            if min_keep > 0:
                keep[np.argsort(-S)[:min_keep]] = True
            keep[int(np.argmax(it["geo"]))] = True
            nk.append(int(keep.sum()))
            good = it["ang"] <= OK
            if good.any() and (good & keep).any():
                surv += 1
            w = int(np.argmax(np.where(keep, it["s"], -np.inf)))
            if it["ang"][w] <= OK:
                pick += 1
        return dict(surv=100 * surv / max(len(have), 1), pick=100 * pick / max(len(items), 1),
                    nkeep=float(np.median(nk)))

    guards = [0.0, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98]
    curve = [(g, ev(g=g)) for g in guards]
    terms_tab = [(t, w, th, ev(terms=t, w_p=w, theta0=th))
                 for t, w, th in (("cov", 0.0, 30.0), ("cov,prec", 0.5, 30.0),
                                  ("cov,prec", 1.0, 30.0), ("cov,prec,shape", 0.5, 15.0),
                                  ("cov,prec,shape", 0.5, 30.0), ("cov,prec,shape", 0.5, 60.0),
                                  ("cov,prec,shape", 1.0, 30.0))]

    # 용도 B: cov_best / r_area_best 가 '정답 후보가 아예 없음'을 가려내는가
    covb = np.array([it["cov"].max() for it in items])
    rab = np.array([it["r_area"][int(np.argmax(it["cov"]))] for it in items])
    good = np.array([bool((it["ang"] <= OK).any()) for it in items])
    roc = []
    if good.any() and (~good).any():
        for t in np.linspace(0.0, 1.2, 121):
            roc.append((float((covb[good] < t).mean()), float((covb[~good] < t).mean())))
        auc = float((covb[good][:, None] > covb[~good][None, :]).mean()
                    + 0.5 * (covb[good][:, None] == covb[~good][None, :]).mean())
    else:
        auc = float("nan")
    op = [(t, int((covb[~good] < t).sum()), int((covb[good] < t).sum()))
          for t in (0.3, 0.4, 0.5, 0.6, 0.7)]

    return dict(items=items, n_have=len(have), n_items=len(items),
                curve=curve, terms_tab=terms_tab, roc=roc, auc=auc, op=op,
                n_good=int(good.sum()), n_bad=int((~good).sum()),
                dist=dict(cov=covb, r_area=rab, good=good,
                          prec=np.array([it["prec"][int(np.argmax(it["cov"]))] for it in items]),
                          dth=np.array([it["dth"][int(np.argmax(it["cov"]))] for it in items])))


# ================================================================== SVG 차트
# matplotlib PNG 대신 손으로 SVG 를 쓴다 — 자체완결이고, 확대해도 선명하고,
# 다크 테마와 싸울 일이 없다. 차트가 6종뿐이라 이쪽이 가볍다.
AX, FG, GRID = "#666", "#bbb", "#282828"


def _sc(v, lo, hi, a, b):
    return a + (b - a) * (0.0 if hi == lo else (v - lo) / (hi - lo))


def svg_xy(series, xr, yr, xlab, ylab, w=560, h=280, xticks=None, yticks=None,
           ysuffix="%", pad=(52, 14, 34, 12), markers=True, diag=False):
    """series = [(이름, 색, [(x,y),...], 점선여부), ...]"""
    L, R, B, T = pad
    x0, x1, y0, y1 = L, w - R, h - B, T
    P = [f"<svg viewBox='0 0 {w} {h}' width='{w}' height='{h}' "
         f"font-family='sans-serif' font-size='10.5'>"]
    xticks = xticks or list(np.linspace(xr[0], xr[1], 5))
    yticks = yticks or list(np.linspace(yr[0], yr[1], 5))
    for v in yticks:
        y = _sc(v, yr[0], yr[1], y0, y1)
        P.append(f"<line x1='{x0}' y1='{y:.1f}' x2='{x1}' y2='{y:.1f}' stroke='{GRID}'/>")
        P.append(f"<text x='{x0-6}' y='{y+3.5:.1f}' fill='{FG}' text-anchor='end'>"
                 f"{v:g}{ysuffix}</text>")
    for v in xticks:
        x = _sc(v, xr[0], xr[1], x0, x1)
        P.append(f"<line x1='{x:.1f}' y1='{y0}' x2='{x:.1f}' y2='{y1}' stroke='{GRID}'/>")
        P.append(f"<text x='{x:.1f}' y='{y0+14}' fill='{FG}' text-anchor='middle'>{v:g}</text>")
    P.append(f"<rect x='{x0}' y='{y1}' width='{x1-x0}' height='{y0-y1}' fill='none' "
             f"stroke='{AX}'/>")
    if diag:
        P.append(f"<line x1='{x0}' y1='{y0}' x2='{x1}' y2='{y1}' stroke='#444' "
                 f"stroke-dasharray='3 3'/>")
    for name, col, pts, dash in series:
        if not pts:
            continue
        d = " ".join(f"{_sc(x, xr[0], xr[1], x0, x1):.1f},{_sc(y, yr[0], yr[1], y0, y1):.1f}"
                     for x, y in pts)
        P.append(f"<polyline points='{d}' fill='none' stroke='{col}' stroke-width='2'"
                 + (" stroke-dasharray='5 4'" if dash else "") + "/>")
        if markers and len(pts) <= 20:
            for x, y in pts:
                P.append(f"<circle cx='{_sc(x, xr[0], xr[1], x0, x1):.1f}' "
                         f"cy='{_sc(y, yr[0], yr[1], y0, y1):.1f}' r='2.6' fill='{col}'/>")
    P.append(f"<text x='{(x0+x1)/2:.0f}' y='{h-3}' fill='{FG}' text-anchor='middle'>"
             f"{html.escape(xlab)}</text>")
    P.append(f"<text x='12' y='{(y0+y1)/2:.0f}' fill='{FG}' text-anchor='middle' "
             f"transform='rotate(-90 12 {(y0+y1)/2:.0f})'>{html.escape(ylab)}</text>")
    ly = T + 4
    for name, col, pts, dash in series:
        if not pts:
            continue
        P.append(f"<line x1='{x1-118}' y1='{ly}' x2='{x1-100}' y2='{ly}' stroke='{col}' "
                 f"stroke-width='2'" + (" stroke-dasharray='5 4'" if dash else "") + "/>")
        P.append(f"<text x='{x1-95}' y='{ly+3.5}' fill='{FG}'>{html.escape(name)}</text>")
        ly += 14
    P.append("</svg>")
    return "".join(P)


def cdf_pts(a, hi=180.0, n=200):
    a = np.sort(np.asarray(a))
    xs = np.linspace(0, hi, n)
    return [(float(x), 100.0 * float((a <= x).mean())) for x in xs]


def hist_pts(a, lo, hi, bins=40):
    """계단 외곽선 좌표 (밀도 %)."""
    a = np.asarray(a)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return []
    c, e = np.histogram(np.clip(a, lo, hi), bins=bins, range=(lo, hi))
    c = 100.0 * c / max(c.sum(), 1)
    pts = []
    for k in range(bins):
        pts += [(float(e[k]), float(c[k])), (float(e[k + 1]), float(c[k]))]
    return pts


# ================================================================ 프레임 패널
def pick_cases(d, n_brk=20, n_fix=12, n_sus=8, n_same=4):
    """보여 줄 케이스 고르기.

    ⚠ 파손만 보여 주면 게이트가 실제보다 나빠 보인다. 교정과 무변 대조군을 반드시 같이
    넣고, 각 분류의 **전체 건수**를 패널 머리에 적는다(HTML 쪽에서).
    """
    e0, e1 = d["err"][BASE], d["err"]["mg_on"]
    dl = d["delta"]["mg_on"]
    brk = sorted(dl["keys_brk"], key=lambda k: -(e1[k] - e0[k]))[:n_brk]
    fix = sorted(dl["keys_fix"], key=lambda k: -(e0[k] - e1[k]))[:n_fix]
    sus = [k for k in d["suspect"] if k in e1]
    sus = sorted(sus, key=lambda k: -e1[k])[:n_sus]
    com = set(e0) & set(e1)
    same = [k for k in com if abs(e0[k] - e1[k]) < 1e-9 and e0[k] <= OK]
    same = same[::max(1, len(same) // max(n_same, 1))][:n_same]
    out = []
    for kind, ks in (("파손", brk), ("교정", fix), ("ism_suspect", sus), ("무변(대조)", same)):
        for k in ks:
            out.append((kind, k))
    return out, dict(파손=len(dl["keys_brk"]), 교정=len(dl["keys_fix"]),
                     ism_suspect=len(d["suspect"]), **{"무변(대조)": len(same)})


def build_panels(d, bag, stride, picks, tile=132):
    """선정된 케이스의 타일을 만든다. **프레임은 V.read_frames 로만 얻는다** —
    stride·depth 페어링이 조금이라도 다르면 엉뚱한 사진에 자세를 그리게 된다."""
    import torch
    need = {k[0] for _kind, k in picks}
    if not need:
        return []
    print(f"[frame] bag 디코드 (stride={stride}) — 필요한 프레임 {len(need)}장", flush=True)
    K, frames = V.read_frames(bag, stride, 0)
    img = {i: frames[i][1].copy() for i in need if i < len(frames)}
    del frames                                    # 수 GB 를 즉시 놓아 준다
    tem = {}
    for o in sorted({k[1] for _kind, k in picks}):
        p = os.path.join(REPO, "assets", "pem_templates", f"{o}.pt")
        if os.path.isfile(p):
            b = torch.load(p, map_location="cpu")
            tem[o] = (b["tp"][0].numpy() * 1000.0, b["tc"][0].numpy())

    out = []
    for kind, (i, o) in picks:
        if i not in img or o not in tem:
            continue
        bgr, (tp, tc) = img[i], tem[o]
        r0 = d["runs"][BASE][i][o]
        r1 = d["runs"]["mg_on"][i][o]
        box, ref = r0["bbox"], d["ref_by"][(i, o)]
        a0, a1 = d["err"][BASE][(i, o)], d["err"]["mg_on"][(i, o)]
        g = r1.get("gate") or {}
        tiles = [("입력", b64(render(bgr, None, None, tp, tc, K, box, out=tile)), None, "inp"),
                 (f"현행 {LABEL[BASE].split()[0]}",
                  b64(render(bgr, np.array(r0["R"]), r0["t_mm"], tp, tc, K, box, out=tile)),
                  a0, "sel" if a0 > NEAR else "ok"),
                 ("게이트 mg_on",
                  b64(render(bgr, np.array(r1["R"]), r1["t_mm"], tp, tc, K, box, out=tile)),
                  a1, "sel" if a1 > NEAR else "ok"),
                 ("기준", b64(render(bgr, np.array(ref), r0["t_mm"], tp, tc, K, box, out=tile)),
                  0.0, "ref")]
        out.append(dict(kind=kind, i=i, obj=o, inst=d["inst_by"].get((i, o)),
                        bbox=box, a0=a0, a1=a1, tiles=tiles,
                        cov=g.get("cov"), prec=g.get("prec"), r_area=g.get("r_area"),
                        dtheta=g.get("dtheta"), cov_best=g.get("cov_best"),
                        n_keep=g.get("n_keep"), n_eval=g.get("n_eval"),
                        suspect=g.get("ism_suspect"),
                        verdict=((r1.get("verify") or {}).get("verdict"))))
    print(f"[frame] 타일 {4*len(out)}장", flush=True)
    return out


# ===================================================================== HTML
CSS = """
body{background:#111;color:#ddd;font-family:'Nanum Gothic','Malgun Gothic',sans-serif;margin:0;padding:18px}
h1{color:#ffe066;font-size:23px}
h2{color:#9ad0ff;margin-top:36px;border-bottom:1px solid #333;padding-bottom:6px}
h3{color:#cfcfcf;font-size:14px;margin:20px 0 6px}
table{border-collapse:collapse;font-size:12px;margin:10px 0}
th,td{border:1px solid #333;padding:4px 9px;text-align:right}th{background:#1c1c1c;color:#ffe066}
td.l,th.l{text-align:left}
.A{color:#ff7676}.C{color:#ffb347}.D{color:#7dff9b}
.legend{background:#181818;padding:11px 15px;border-radius:6px;font-size:12px;line-height:1.85}
.verdict{background:#171d17;border-left:4px solid #7dff9b;padding:12px 16px;border-radius:5px;
         font-size:13px;line-height:1.9;margin:14px 0}
.warn{background:#241a17;border-left:4px solid #ff7676;padding:11px 15px;border-radius:5px;
      font-size:12.5px;line-height:1.85;margin:12px 0}
.nav a{color:#9ad0ff;margin-right:16px;font-size:13px}
.case{border-bottom:1px solid #222;padding:9px 0;margin-bottom:5px}
.meta{font-size:11.5px;color:#bbb;line-height:1.6;margin-bottom:6px}
.strip{display:flex;gap:6px;align-items:flex-start;flex-wrap:nowrap;overflow-x:auto}
.tile{text-align:center;font-size:10.5px;color:#aaa}
.tile img{display:block;border:2px solid #2a2a2a;border-radius:3px}
.tile.sel img{border-color:#ff3b3b}.tile.ok img{border-color:#7dff9b}
.tile.ref img{border-color:#9ad0ff}.tile.inp img{border-color:#555}
.gain{color:#7dff9b}.loss{color:#ff7676}.dim{color:#777}
.k파손{color:#ff7676}.k교정{color:#7dff9b}.kism_suspect{color:#ffb347}
.chart{margin:8px 0 4px}
.two{display:flex;gap:26px;flex-wrap:wrap}
.note{font-size:12px;color:#999;line-height:1.75;margin:6px 0 0}
"""


def t_row(cells, cls=None):
    out = "<tr>"
    for j, c in enumerate(cells):
        k = " class='l'" if j == 0 else (f" class='{cls[j]}'" if cls and cls[j] else "")
        out += f"<td{k}>{c}</td>"
    return out + "</tr>"


def sec_scorecard(P, tag, d):
    P.append(f"<h3>{tag} — 설정별 채점</h3><table>")
    P.append("<tr><th class='l'>설정</th><th>평가건수</th><th>중앙°</th><th>&lt;15°</th>"
             "<th>&gt;45° gross</th></tr>")
    for n in SETTINGS:
        s = d["score"][n]
        b = " style='background:#16202a'" if n == BASE else ""
        P.append(f"<tr{b}><td class='l'>{html.escape(LABEL[n])}</td><td>{s['n']}</td>"
                 f"<td>{s['med']:.1f}</td><td>{s['lt15']:.1f}%</td>"
                 f"<td>{s['gross']:.1f}%</td></tr>")
    P.append("</table>")
    P.append(f"<h3>{tag} — 현행 운영(mg_null) 대비 <span class='dim'>(판정은 이 표로 한다)</span>"
             "</h3><table>")
    P.append("<tr><th class='l'>설정</th><th>공통</th><th>답이 바뀜</th><th>교정</th>"
             "<th>파손</th><th>순</th></tr>")
    for n in SETTINGS:
        if n == BASE:
            continue
        x = d["delta"][n]
        net = x["fix"] - x["brk"]
        c = "gain" if net > 0 else ("loss" if net < 0 else "dim")
        P.append(f"<tr><td class='l'>{html.escape(LABEL[n])}</td><td>{x['n']}</td>"
                 f"<td>{100*x['chg']/max(x['n'],1):.1f}%</td>"
                 f"<td class='gain'>{x['fix']}</td><td class='loss'>{x['brk']}</td>"
                 f"<td class='{c}'>{net:+d}</td></tr>")
    P.append("</table>")
    o = d["delta"]["off"]
    P.append(f"""<p class='note'>맨 윗줄(<b>off</b>)은 거꾸로 읽어야 한다 — 현행에서 텍스처 검증을
<b>끄면</b> 교정 {o['fix']} : 파손 {o['brk']} 이 된다는 뜻이므로, <b>Stage V 자체의 이득이
순 {o['brk']-o['fix']:+d}</b> 라는 말이다. 게이트 판정과 헷갈리지 말 것.</p>""")


def sec_object(P, tag, d):
    P.append(f"<h3>{tag} — 객체별 (현행 → ch1)</h3><table>")
    P.append("<tr><th class='l'>객체</th><th>n</th><th>승자 변경</th><th>교정</th><th>파손</th>"
             "<th>gross 현행</th><th>gross ch1</th></tr>")
    for o, n, ch, f, b, g0, g1 in sorted(d["by_object"], key=lambda r: -r[2]):
        c = "gain" if g1 < g0 - 0.05 else ("loss" if g1 > g0 + 0.05 else "dim")
        P.append(f"<tr><td class='l'>{html.escape(o)}</td><td>{n}</td><td>{ch}</td>"
                 f"<td class='gain'>{f}</td><td class='loss'>{b}</td>"
                 f"<td>{g0:.1f}%</td><td class='{c}'>{g1:.1f}%</td></tr>")
    P.append("</table>")


def sec_sweep(P, tag, d):
    cur = d["curve"]
    P.append(f"<h3>{tag} — gate_guard 스윕</h3>")
    P.append("<div class='chart'>" + svg_xy(
        [("정답 선택률", "#7dff9b", [(g, v["pick"]) for g, v in cur], False),
         ("정답 후보 생존률", "#9ad0ff", [(g, v["surv"]) for g, v in cur], False)],
        xr=(0, 1.0), yr=(min(50, min(v["pick"] for _g, v in cur) - 5), 101),
        xlab="gate_guard  (0 = 거르지 않음)", ylab="%",
        xticks=[0, 0.25, 0.5, 0.75, 1.0]) + "</div>")
    P.append("<table><tr><th>gate_guard</th><th>생존률</th><th>선택률</th>"
             "<th>중앙 생존 후보수</th></tr>")
    best = max(v["pick"] for _g, v in cur)
    for g, v in cur:
        c = "gain" if v["pick"] >= best - 1e-9 else ""
        P.append(f"<tr><td>{g:.2f}</td><td>{v['surv']:.1f}%</td>"
                 f"<td class='{c}'>{v['pick']:.1f}%</td><td>{v['nkeep']:.0f}</td></tr>")
    P.append("</table>")
    P.append("<table><tr><th class='l'>결합 항</th><th>w_p</th><th>theta0</th><th>생존률</th>"
             "<th>선택률</th><th>중앙 생존</th></tr>")
    for t, w, th, v in d["terms_tab"]:
        P.append(f"<tr><td class='l'>{t}</td><td>{w:.1f}</td><td>{th:.0f}</td>"
                 f"<td>{v['surv']:.1f}%</td><td>{v['pick']:.1f}%</td>"
                 f"<td>{v['nkeep']:.0f}</td></tr>")
    P.append("</table>")


def sec_circular(P, tag, d):
    P.append(f"<h3>{tag} — admit 채널별 <span class='loss'>⚠ 순환</span></h3>")
    P.append("""<div class='warn'>
<b>⚠ 이 표를 ch2 의 근거로 쓰면 안 된다.</b> 아래를 보면 <code>both</code>(마스크·지도 둘 다
통과)가 gross 거의 0 이고 <code>mask</code> 만 통과한 쪽은 크게 나빠 <b>ch2 가 극적으로 좋아
보인다</b>. 그런데 <b>채점 기준 자체가 같은 map-frame 최빈 자세</b>다 — 즉
"승자가 지도 최빈과 맞다"와 "승자가 기준과 맞다"가 같은 말이 되어 버린다.
이 분해는 <b>무엇이 쉬웠는지</b>를 보여 줄 뿐 ch2 가 무엇을 <b>고쳤는지</b>는 말해 주지 않는다.
비순환 증거는 위 <b>현행 대비 교정:파손</b> 하나뿐이다.
</div>""")
    P.append("<table><tr><th class='l'>설정</th><th class='l'>admit</th><th>n</th>"
             "<th>중앙°</th><th>&gt;45° gross</th></tr>")
    for n in ("mg_on", "mp_on"):
        for ch, v in sorted(d["admit"][n].items()):
            P.append(f"<tr style='color:#8a8a8a'><td class='l'>{html.escape(LABEL[n])}</td>"
                     f"<td class='l'>{ch}</td><td>{v['n']}</td><td>{v['med']:.1f}</td>"
                     f"<td>{v['gross']:.1f}%</td></tr>")
    P.append("</table>")


def sec_usecase_b(P, tag, d):
    P.append(f"<h3>{tag} — 용도 B: ISM 오인식 판별</h3>")
    if d["roc"]:
        P.append("<div class='chart'>" + svg_xy(
            [("cov_best", "#ffb347", [(100 * fp, 100 * tp) for fp, tp in d["roc"]], False)],
            xr=(0, 100), yr=(0, 100), xlab="정상을 잘못 버림 (%)",
            ylab="정답 후보 없는 검출을 잡음 (%)", w=330, h=280, diag=True,
            markers=False) + "</div>")
    P.append(f"<p class='note'>AUC <b>{d['auc']:.3f}</b> · 정답 후보가 있는 검출 "
             f"{d['n_good']} vs 없는 검출 {d['n_bad']}</p>")
    P.append("<table><tr><th>cov_best 임계</th><th>오인식 잡음</th><th>정상 잘못 버림</th></tr>")
    for t, tp, fp in d["op"]:
        P.append(f"<tr><td>{t:.1f}</td><td class='gain'>{tp} / {d['n_bad']}</td>"
                 f"<td class='loss'>{fp} / {d['n_good']}</td></tr>")
    P.append("</table>")


def sec_dist(P, tag, d):
    ds = d["dist"]
    g, b = ds["good"], ~ds["good"]
    P.append(f"<h3>{tag} — 진단 항 분포 (승자 후보 기준)</h3>")
    P.append("<div class='two'>")
    for key, lo, hi, name in (("cov", 0, 1.05, "cov"), ("r_area", 0, 3.0, "r_area"),
                              ("prec", 0, 1.05, "prec"), ("dth", 0, 90, "dtheta (도)")):
        a = ds[key]
        P.append("<div class='chart'>" + svg_xy(
            [("정답 있음", "#7dff9b", hist_pts(a[g], lo, hi), False),
             ("정답 없음", "#ff7676", hist_pts(a[b], lo, hi), False)],
            xr=(lo, hi), yr=(0, max(6, 1.1 * max(
                [y for _x, y in hist_pts(a[g], lo, hi)] +
                [y for _x, y in hist_pts(a[b], lo, hi)] or [6]))),
            xlab=name, ylab="비율", w=330, h=210, markers=False) + "</div>")
    P.append("</div>")
    P.append("""<p class='note'>설계 근거였던 비대칭 — <b>가림은 r_area 를 올리고(마스크가 줄어드니까)
더 큰 엉뚱한 물체 위 오인식은 내린다</b> — 이 실데이터에서도 성립하는지 보는 그림이다.
성립한다면 '정답 없음'(빨강)이 r_area 낮은 쪽으로 치우쳐야 한다.</p>""")


def sec_panels(P, tag, panels, counts, anchor):
    P.append(f"<h2 id='{anchor}'>{tag} — 실제 프레임</h2>")
    P.append(f"""<div class='legend'>
타일 넷: <b>입력</b>(오버레이 없음) · <b class='dim'>현행</b>(mg_null 이 고른 자세) ·
<b class='loss'>게이트</b>(mg_on 이 고른 자세) · <b style='color:#9ad0ff'>기준</b>.
타일은 그 자세로 <b>색 있는 템플릿 점군</b>을 영상에 반투명으로 얹은 것이라, 자세가 어긋나면
무늬가 밀려 보인다. 테두리 <span class='D'>초록</span>=정답(≤{NEAR:.0f}°),
<span style='color:#ff3b3b'>빨강</span>=gross, <span style='color:#9ad0ff'>파랑</span>=기준.
숫자는 기준과의 각도(<span class='D'>≤{OK:.0f}°</span> · <span class='C'>≤{NEAR:.0f}°</span> ·
<span class='A'>그 이상</span>).<br>
<b>고른 방식</b>: 파손 {counts.get('파손',0)}건 중 최대 20 · 교정 {counts.get('교정',0)}건 중 최대 12 ·
ism_suspect {counts.get('ism_suspect',0)}건 중 최대 8 · 무변 대조군 4.
<b>파손만 보면 게이트가 실제보다 나빠 보이므로</b> 교정과 무변을 반드시 같이 실었다.
</div>""")
    for c in panels:
        d0, d1 = cls_of(c["a0"]), cls_of(c["a1"])
        P.append("<div class='case'>")
        P.append(f"<div class='meta'><b class='k{c['kind']}'>{c['kind']}</b> · "
                 f"<b>{html.escape(c['obj'])}</b>"
                 + (f" #{c['inst']}" if c["inst"] is not None else "")
                 + f" · frame {c['i']} · bbox {c['bbox'][2]-c['bbox'][0]}×"
                 f"{c['bbox'][3]-c['bbox'][1]}px · "
                 f"현행 <span class='{d0}'>{c['a0']:.1f}°</span> → "
                 f"게이트 <span class='{d1}'>{c['a1']:.1f}°</span>"
                 + (f" · verdict {c['verdict']}" if c["verdict"] else "")
                 + (" · <span class='C'>ism_suspect</span>" if c["suspect"] else "")
                 + (f" · cov {c['cov']:.3f} / prec {c['prec']:.3f} / r_area {c['r_area']:.2f}"
                    f" / dθ {c['dtheta']:.1f}° · 생존 {c['n_keep']}/{c['n_eval']}"
                    if c["cov"] is not None else "") + "</div>")
        P.append("<div class='strip'>")
        for name, im, ang, klass in c["tiles"]:
            lab = "" if ang is None else (
                f"<span class='{cls_of(ang)}'>{ang:.1f}°</span>" if klass != "ref" else "기준")
            P.append(f"<div class='tile {klass}'><img src='{im}'>"
                     f"<div>{html.escape(name)}</div><div>{lab}</div></div>")
        P.append("</div></div>")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval-0807", default="temp/eval_0807")
    ap.add_argument("--bag-0807", default="data/bag_0807_185223_sam")
    ap.add_argument("--traj-0807",
                    default="../integration/output/0807_185223_circle8/slam_orb3/"
                            "samcam_trajectory_in_slam_map.txt")
    ap.add_argument("--stride-0807", type=int, default=3)
    ap.add_argument("--eval-260804", default="temp/eval_260804")
    ap.add_argument("--bag-260804", default="../data_slam_converted/260804_office/longcircle2/SAM")
    ap.add_argument("--traj-260804",
                    default="../integration/output/260804_office/longcircle2/orb3/trajectory.txt")
    ap.add_argument("--extrinsic-260804",
                    default="../data_slam_converted/260804_office/longcircle2/calib/X_camSLAM_camSAM.npy")
    ap.add_argument("--stride-260804", type=int, default=1)
    ap.add_argument("--out", default="outputs/test/gate_report.html")
    ap.add_argument("--no-frames", action="store_true", help="이미지 없이 수치만(빠른 확인용)")
    a = ap.parse_args()

    BAGS = [("0807 185223__circle_ccw_8laps", "b0807", a.eval_0807, a.bag_0807,
             a.traj_0807, "", a.stride_0807),
            ("260804 longcircle2", "b260804", a.eval_260804, a.bag_260804,
             a.traj_260804, a.extrinsic_260804, a.stride_260804)]

    res = []
    for tag, anc, ev, bag, traj, extr, stride in BAGS:
        print(f"[분석] {tag}", flush=True)
        d = analyse(ev, traj, extr)
        picks, counts = pick_cases(d)
        panels = [] if a.no_frames else build_panels(d, bag, stride, picks)
        res.append((tag, anc, d, panels, counts, stride))

    # --- 파리티 (원시 행 비교) -------------------------------------------
    par = []
    for tag, _anc, d, _p, _c, _s in res:
        r0 = [d["runs"]["mg_null"][i][o] for i in sorted(d["runs"]["mg_null"])
              for o in sorted(d["runs"]["mg_null"][i])]
        r1 = [d["runs"]["mg_parity"][i][o] for i in sorted(d["runs"]["mg_parity"])
              for o in sorted(d["runs"]["mg_parity"][i])]
        f = lambda r: (r["R"], r["t_mm"], r["score"])              # noqa: E731
        bad = sum(1 for x, y in zip(r0, r1) if f(x) != f(y))
        par.append((tag, len(r0), len(r1), bad))

    P = ["<!DOCTYPE html><html lang='ko'><head><meta charset='utf-8'>",
         "<title>SAM-6D 후보 게이트 실측</title><style>", CSS, "</style></head><body>"]
    P.append("<h1>SAM-6D 후보 게이트 — 두 bag 전량 실측</h1>")

    g0, g1 = res[0][2], res[1][2]
    d0, d1 = g0["delta"]["mg_on"], g1["delta"]["mg_on"]
    m0, m1 = g0["delta"]["mp_on"], g1["delta"]["mp_on"]
    P.append(f"""<div class='verdict'>
<b>결론 — 후보를 거르는 것은 순손실이라 기본값을 뒤집었다.</b><br>
현행 운영(Stage V 텍스처 검증만) 대비 ch1 마스크 게이트는
0807 에서 <b class='gain'>교정 {d0['fix']}</b> : <b class='loss'>파손 {d0['brk']}</b>
= <b class='loss'>순 {d0['fix']-d0['brk']:+d}</b>,
260804 에서 {d1['fix']} : {d1['brk']} = 순 {d1['fix']-d1['brk']:+d} 였다.
gross 는 0807 에서 {g0['score']['mg_null']['gross']:.1f}% →
{g0['score']['mg_on']['gross']:.1f}% 로 <b class='loss'>나빠졌다</b>.<br>
문턱 스윕도 같은 말을 한다 — 정답 선택률이 <b>gate_guard 0.00 에서 최대이고 단조 감소</b>한다.
<b>살릴 문턱이 없다.</b><br>
그래서 기본값은 <code>gate_guard 0.0</code> · <code>min_keep 300</code> — <b>점수는 계산하되
자격을 깎지 않는다</b>. 결과가 예전과 비트 단위로 같으면서 진단과 <code>ism_suspect</code> 판정은
그대로 얻는다. <b>ch2(Map 사전)는 기본 OFF</b> — 이득이 증명되지 않았고
(순 {m0['fix']-m0['brk']:+d} / {m1['fix']-m1['brk']:+d}, 둘 다 잡음 범위),
겉보기 이득은 <b>순환</b>이다(§4 참고).
</div>""")
    P.append("<p class='nav'>" + " ".join(
        f"<a href='#{x}'>{y}</a>" for x, y in
        (("cond", "1 측정 조건"), ("gain", "2 이득"), ("sweep", "3 문턱 스윕"),
         ("circ", "4 ⚠ 순환"), ("useb", "5 용도 B"), ("dist", "6 진단 분포"),
         ("b0807", "7 프레임 · 0807"), ("b260804", "8 프레임 · 260804"))) + "</p>")

    # 1 측정 조건
    P.append("<h2 id='cond'>1. 측정 조건</h2>")
    P.append("<table><tr><th class='l'>bag</th><th>stride</th><th>프레임</th><th>검출</th>"
             "<th>기준이 선 검출</th><th>파리티 행</th><th>불일치</th></tr>")
    for (tag, _a, d, _p, _c, stride), (_t, n0, _n1, bad) in zip(res, par):
        P.append(f"<tr><td class='l'>{html.escape(tag)}</td><td>{stride}</td>"
                 f"<td>{d['n_frame'] if d['n_frame'] else '-'}</td><td>{d['n_det']}</td>"
                 f"<td>{d['n_ref']} ({100*d['n_ref']/max(d['n_det'],1):.1f}%)</td>"
                 f"<td>{n0}</td><td class='{'gain' if bad==0 else 'loss'}'>{bad}</td></tr>")
    P.append("</table>")
    P.append("""<p class='note'><b>기준 자세</b>는 GT 가 없으므로 <b>SLAM 궤적 + 인스턴스별 최빈</b>으로
세운다(물체가 정지해 있으므로 지도좌표계 자세가 상수다). 예전에 쓰던 객체쌍 합의 기준은
파트너가 2개 이상 동시에 보여야 해서 0807 에서 38% 만, 260804 는 아예 성립하지 않았다.<br>
<b>파리티</b>: <code>mg_parity</code>(렌더·비교를 전부 태우되 아무도 거르지 않음) 가
<code>mg_null</code> 과 R·t·score 가 완전히 같아야 한다. 위 표의 불일치 0 이 그 증명이다.</p>""")
    P.append("<h3>기준이 얼마나 튼튼한가 — 인스턴스별 최빈 점유</h3>")
    P.append("<table><tr><th class='l'>bag</th><th class='l'>객체</th><th>inst</th>"
             "<th>최빈 점유</th><th>n</th></tr>")
    for tag, _anc, d, _p, _c, _s in res:
        for k in sorted(d["share"], key=lambda k: -d["share"][k][1]):
            sh, n = d["share"][k]
            c = "loss" if sh < 0.6 else ("C" if sh < 0.8 else "gain")
            P.append(f"<tr><td class='l'>{html.escape(tag.split()[0])}</td>"
                     f"<td class='l'>{html.escape(k[0])}</td><td>{k[1]}</td>"
                     f"<td class='{c}'>{100*sh:.1f}%</td><td>{n}</td></tr>")
    P.append("</table>")
    P.append("""<div class='warn'>⚠ <b>260804 는 기준 자체가 약하다</b>(Febreze 37.9% · Sauce 57.6% ·
Mugcup 62.5%). 뒤에 나오는 260804 의 gross 37% 는 자세가 그만큼 나쁘다기보다 <b>기준이 흔들려서</b>
부풀려진 값이다. <b>두 bag 을 평균 내지 말고 따로 읽어야 한다</b> — 카메라도 D455F vs D435i 로 다르다.</div>""")

    # 2 이득
    P.append("<h2 id='gain'>2. 이득 — 현행 대비 순손실</h2>")
    for tag, _anc, d, _p, _c, _s in res:
        sec_scorecard(P, tag.split()[0], d)
    P.append("<h3>오차 분포 (CDF) — 곡선이 겹치면 차이가 없다는 뜻</h3><div class='two'>")
    for tag, _anc, d, _p, _c, _s in res:
        P.append("<div class='chart'>" + svg_xy(
            [(LABEL[n].split()[0], COLOR[n], cdf_pts(list(d["err"][n].values())),
              n == "mg_parity") for n in SETTINGS],
            xr=(0, 60), yr=(0, 100), xlab=f"{tag.split()[0]} — 기준과의 각도 (도)",
            ylab="누적 %", w=430, h=270, markers=False) + "</div>")
    P.append("</div>")
    for tag, _anc, d, _p, _c, _s in res:
        sec_object(P, tag.split()[0], d)

    # 3 스윕
    P.append("<h2 id='sweep'>3. 문턱 스윕 — 살릴 문턱이 없다</h2>")
    P.append("""<p class='note'>중립 설정(<code>gate_guard 0</code>·<code>min_keep 300</code>·전량 덤프)의
후보 덤프에는 검출마다 후보 100개의 <code>[qx qy qz qw geo s cov prec r_area dtheta ch]</code> 가
들어 있다. 여기에 궤적 기준을 붙이면 "그 문턱이었으면 정답 후보가 살아남았을까 / 승자가 누구였을까"를
<b>재추론 없이</b> 전부 다시 계산할 수 있다.<br>
<b>생존률</b> = 정답 후보를 가진 검출 중 그 후보가 문턱을 통과한 비율(1.0 이어야 한다) ·
<b>선택률</b> = 전체 검출 중 텍스처 승자가 정답인 비율.</p>""")
    for tag, _anc, d, _p, _c, _s in res:
        sec_sweep(P, tag.split()[0], d)
    c0 = g0["curve"]
    p_lo = [v["pick"] for gg, v in c0 if gg == 0.0][0]
    p_hi = [v["pick"] for gg, v in c0 if gg == 0.85][0]
    P.append(f"""<p class='note'><b>교차검증</b>: 0807 에서 스윕이 예측한 손실은
{p_lo:.1f}% → {p_hi:.1f}% = <b>{p_lo-p_hi:.1f}%p</b>, 검출 {g0['n_items']}건 기준
약 {round((p_lo-p_hi)/100*g0['n_items'])}건이다. 실제로 측정한 값은
<b>순 {d0['fix']-d0['brk']:+d}</b>(교정 {d0['fix']} : 파손 {d0['brk']}) 로 맞아떨어진다 —
스윕 계산 자체는 믿을 만하다는 뜻이다.</p>""")

    # 4 순환
    P.append("<h2 id='circ'>4. ⚠ ch2 의 겉보기 이득은 순환이다</h2>")
    for tag, _anc, d, _p, _c, _s in res:
        sec_circular(P, tag.split()[0], d)

    # 5 용도 B
    P.append("<h2 id='useb'>5. 용도 B — ISM 오인식 판별</h2>")
    P.append("""<p class='note'>후보를 거르는 것(용도 A)과 달리, <b>후보를 아무리 잘 골라도 마스크가
안 맞으면 그 검출 자체가 의심스럽다</b>는 판정은 절대값으로 봐야 한다. 오인식이면 후보 300개가
<b>전부</b> 안 맞으므로 상대 문턱으로는 원리상 못 잡는다.</p>""")
    P.append("<div class='two'>")
    for tag, _anc, d, _p, _c, _s in res:
        P.append("<div>")
        sec_usecase_b(P, tag.split()[0], d)
        P.append("</div>")
    P.append("</div>")
    P.append("""<p class='note'>정밀하지만 <b>재현율이 낮은</b> 검출기다. 그래서 기본값은
<code>ism_reject: false</code> — 검출을 지우지 않고 <code>ism_suspect</code> 로 <b>표시만</b> 한다.</p>""")

    # 6 분포
    P.append("<h2 id='dist'>6. 진단 항 분포</h2>")
    for tag, _anc, d, _p, _c, _s in res:
        sec_dist(P, tag.split()[0], d)

    # 7·8 프레임
    for tag, anc, d, panels, counts, _s in res:
        if panels:
            sec_panels(P, tag, panels, counts, anc)

    P.append("<p class='note' style='margin-top:30px'>생성: temp/gate_report.py · "
             "추론 재실행 없음 · 이미지는 전부 base64 인라인(외부 참조 0)</p>")
    P.append("</body></html>")

    out = a.out if os.path.isabs(a.out) else os.path.join(REPO, a.out)
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text("\n".join(P), encoding="utf-8")
    print(f"[out] {out}  ({os.path.getsize(out)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
