#!/usr/bin/env python3
"""verify_report.py — 텍스처 검증이 무엇을 바꾸는지 눈으로 확인하는 HTML 보고서.

한 프레임·한 객체마다 두 줄을 그린다.
  (A) 기하 점수 상위 N 개  — 선택(1등)에 빨간 테두리   = 예전 방식
  (B) 텍스처까지 넣어 재정렬한 상위 N 개 — 선택에 빨간 테두리 = Stage V
같은 후보 300개에서 같은 이미지를 놓고 그린 것이라 둘의 차이는 **채점 함수뿐**이다.

기준 자세는 GT 가 없으므로 **객체쌍 상대회전 합의**로 세운다(verify_eval 과 같은 방법).
정답 판정은 **기준에서 30° 이내**로 본다 — fine 정제의 포획범위가 45~60° 이므로
30° 안이면 정제가 확실히 수렴한다(하나의 정답이 아니라 '수렴하는 자세 그룹').

    conda activate sam6d
    python temp/verify_report.py --bag <sam bag> --out outputs/test/verify_report.html
"""
from __future__ import annotations

import argparse
import base64
import html
import io
import os
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

REPO = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, os.path.join(REPO, "realtime"))
sys.path.insert(0, os.path.join(REPO, "temp"))

import verify_eval as V                                            # noqa: E402

OK_DEG = 30.0        # 이 안이면 fine 이 수렴한다 = 정답
NEAR_DEG = 45.0      # 포획범위 경계


# ------------------------------------------------------------------ 렌더
def render(bgr, R, t_mm, tp_mm, tc_bgr, K, box, out=132, pad=1.7, radius=2):
    """후보 자세로 색 있는 모델 점군을 얹은 뒤 객체 주변만 잘라 낸다."""
    img = bgr.copy()
    if R is not None:
        P = tp_mm @ np.asarray(R).T + np.asarray(t_mm)
        keep = P[:, 2] > 1.0
        P, C = P[keep], tc_bgr[keep]
        o = np.argsort(-P[:, 2])                       # 먼 점부터 = 화가 알고리즘
        P, C = P[o], np.clip(C[o] * 255.0, 0, 255)
        uv = (K @ (P / P[:, 2:3]).T).T[:, :2]
        h, w = img.shape[:2]
        layer = img.copy()
        mask = np.zeros((h, w), np.uint8)
        for (u, v), c in zip(uv, C):
            if 0 <= u < w and 0 <= v < h:
                p_ = (int(u), int(v))
                cv2.circle(layer, p_, radius, (float(c[0]), float(c[1]), float(c[2])), -1)
                cv2.circle(mask, p_, radius, 255, -1)
        # 반투명으로 얹는다 — 렌더된 무늬와 그 아래 실제 물체가 같이 보여야
        # '어긋났다'를 눈으로 판단할 수 있다.
        m = (mask > 0)[:, :, None]
        img = np.where(m, (0.7 * layer + 0.3 * img).astype(np.uint8), img)
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    s = max(x2 - x1, y2 - y1) * pad / 2
    h, w = img.shape[:2]
    a, b = int(max(0, cx - s)), int(max(0, cy - s))
    c, d = int(min(w, cx + s)), int(min(h, cy + s))
    crop = img[b:d, a:c]
    if crop.size == 0:
        crop = np.zeros((out, out, 3), np.uint8)
    return cv2.resize(crop, (out, out), interpolation=cv2.INTER_AREA)


def b64(img, q=78):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return "data:image/jpeg;base64," + base64.b64encode(buf).decode() if ok else ""


def cls_of(a):
    return "D" if a <= OK_DEG else ("C" if a <= NEAR_DEG else "A")


# ------------------------------------------------------------------ 수집
def collect(args):
    import torch
    from sam6d_core import Sam6DCore
    from verify_config import build_appe_cfg

    K, frames = V.read_frames(args.bag, args.stride, args.limit)
    if K is None or not frames:
        raise SystemExit("bag 에서 프레임/카메라 정보를 못 읽었다")

    core = Sam6DCore(args.config, [], args.device)
    ar, _ = build_appe_cfg({"verify": {"dump_topk": args.topn}})
    core.pem.cfg.appe_rerank = ar
    print(f"[run] {ar['verify']['w_col']=} {ar['verify']['geo_guard']=} dump={args.topn}", flush=True)

    tem = {}                                    # 렌더용 색 점군 (mm, BGR 0~1)
    for nm, (tp, tf, tc) in core._tem.items():
        if tc is not None:
            tem[nm] = (tp[0].cpu().numpy() * 1000.0, tc[0].cpu().numpy())

    cases = []
    for i, (t, bgr, dep) in enumerate(frames):
        torch.manual_seed(V.SEED + i)
        np.random.seed(V.SEED + i)
        det, _ms, _nb, _ = core.process(bgr, dep, K)
        for d in det:
            v = d.get("verify")
            if not v or "cands" not in v:
                continue
            cases.append({"i": i, "stamp": t, "object": d["object"], "bbox": d["bbox"],
                          "R": d["R"], "t_mm": d["t_mm"], "verify": v, "frame": i})
        if i % 50 == 0:
            print(f"  {i}/{len(frames)} · 사례 {len(cases)}", flush=True)
    return K, frames, tem, cases


# ------------------------------------------------------------------ 기준
def build_reference(cases):
    runs = defaultdict(dict)
    for c in cases:
        runs[c["i"]][c["object"]] = {"R": c["R"]}
    rel, share = V.calibrate([runs])
    for c in cases:
        objs = runs[c["i"]]
        ref, npart = V.reference(objs, c["object"], rel)
        c["ref"] = None if ref is None else ref
        c["npart"] = npart
    return rel, share


# ------------------------------------------------------------------ 본문
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", default="outputs/test/verify_report.html")
    ap.add_argument("--config", default="configs/yolo_ism_objects.yaml")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--topn", type=int, default=8)
    ap.add_argument("--per-object", type=int, default=8, help="객체당 그림으로 보여줄 사례 수")
    ap.add_argument("--title", default="")
    a = ap.parse_args()

    K, frames, tem, cases = collect(a)
    rel, share = build_reference(cases)
    ev = [c for c in cases if c["ref"] is not None]
    print(f"[기준] 객체쌍 {len(rel)} · 사례 {len(cases)} 중 기준 있는 것 {len(ev)}")

    # --- 통계 -----------------------------------------------------------
    for c in ev:
        cd = {x["rank_geo"]: x for x in c["verify"]["cands"]}
        cs = {x["rank_s"]: x for x in c["verify"]["cands"]}
        c["pick_geo"], c["pick_ver"] = cd.get(0), cs.get(0)
        c["a_geo"] = V.ang_deg(c["ref"], np.array(c["pick_geo"]["R"]), c["object"])
        c["a_ver"] = V.ang_deg(c["ref"], np.array(c["pick_ver"]["R"]), c["object"])
        c["best"] = min(V.ang_deg(c["ref"], np.array(x["R"]), c["object"])
                        for x in c["verify"]["cands"])

    def stat(rows, key, thr):
        return 100.0 * sum(1 for r in rows if r[key] <= thr) / max(len(rows), 1)

    objs = sorted({c["object"] for c in ev})
    tbl = []
    for o in objs + ["__ALL__"]:
        rows = ev if o == "__ALL__" else [c for c in ev if c["object"] == o]
        if not rows:
            continue
        fix = sum(1 for r in rows if r["a_geo"] > OK_DEG >= r["a_ver"])
        brk = sum(1 for r in rows if r["a_ver"] > OK_DEG >= r["a_geo"])
        tbl.append((o, len(rows), stat(rows, "a_geo", OK_DEG), stat(rows, "a_ver", OK_DEG),
                    stat(rows, "a_geo", NEAR_DEG), stat(rows, "a_ver", NEAR_DEG),
                    float(np.median([r["a_geo"] for r in rows])),
                    float(np.median([r["a_ver"] for r in rows])), fix, brk))

    # --- 그림으로 보여줄 사례 고르기 (바뀐 것 우선) -----------------------
    show = []
    for o in objs:
        rows = [c for c in ev if c["object"] == o]
        chg = [c for c in rows if c["pick_geo"]["rank_s"] != 0]
        same = [c for c in rows if c["pick_geo"]["rank_s"] == 0]
        chg.sort(key=lambda c: -(c["a_geo"] - c["a_ver"]))     # 많이 고친 것부터
        pick = chg[:max(1, a.per_object - 2)] + same[:2]
        show += pick[:a.per_object]
    show.sort(key=lambda c: (c["object"], c["i"]))
    print(f"[그림] {len(show)} 사례 렌더")

    # --- HTML -----------------------------------------------------------
    P = []
    P.append("<!DOCTYPE html><html lang='ko'><head><meta charset='utf-8'>")
    P.append("<title>SAM-6D 포즈 후보: 기하 점수 vs 텍스처 검증</title><style>")
    P.append("""
body{background:#111;color:#ddd;font-family:'Nanum Gothic','Malgun Gothic',sans-serif;margin:0;padding:18px}
h1{color:#ffe066;font-size:22px}h2{color:#9ad0ff;margin-top:34px;border-bottom:1px solid #333;padding-bottom:6px}
h3{color:#cfcfcf;font-size:14px;margin:18px 0 6px}
table{border-collapse:collapse;font-size:12px;margin:10px 0}
th,td{border:1px solid #333;padding:4px 9px;text-align:right}th{background:#1c1c1c;color:#ffe066}
td.l,th.l{text-align:left}
.A{color:#ff7676}.C{color:#ffb347}.D{color:#7dff9b}
.legend{background:#181818;padding:10px 14px;border-radius:6px;font-size:12px;line-height:1.8}
.nav a{color:#9ad0ff;margin-right:14px;font-size:13px}
.case{border-bottom:1px solid #222;padding:10px 0;margin-bottom:6px}
.meta{font-size:11.5px;color:#bbb;line-height:1.6;margin-bottom:6px}
.strip{display:flex;gap:6px;align-items:flex-start;flex-wrap:nowrap;overflow-x:auto}
.tile{text-align:center;font-size:10.5px;color:#aaa}
.tile img{display:block;border:2px solid #2a2a2a;border-radius:3px}
.tile.sel img{border-color:#ff3b3b}
.tile.ref img{border-color:#9ad0ff}
.tile.inp img{border-color:#555}
.lab{width:64px;font-size:11px;color:#8ab;flex:0 0 64px;padding-top:44px}
.gain{color:#7dff9b}.loss{color:#ff7676}
""")
    P.append("</style></head><body>")
    P.append(f"<h1>SAM-6D 초기 포즈 선택: 기하 점수만 vs 텍스처 검증 추가{html.escape(a.title)}</h1>")
    P.append(f"""<div class='legend'>
<b>무엇을 보나</b> 같은 프레임·같은 후보 300개에 채점 함수만 바꿔 그렸다.
<b>(A)</b> 기하 점수 상위 {a.topn}개, <b>(B)</b> 텍스처(PEM 특징 + 원색)까지 넣어 재정렬한 상위 {a.topn}개.
각 줄에서 <span style='color:#ff3b3b'><b>선택된 후보에 빨간 테두리</b></span>, <span style='color:#9ad0ff'>기준 자세는 파란 테두리</span>.<br>
<b>타일</b>은 그 자세로 <b>색 있는 템플릿 점군</b>을 입력 영상에 얹은 것이다 — 자세가 뒤집히면 무늬가 어긋나 보인다.
타일 아래 숫자는 기준과의 각도(<span class='D'>≤{OK_DEG:.0f}°</span> · <span class='C'>≤{NEAR_DEG:.0f}°</span> · <span class='A'>그 이상</span>).<br>
<b>기준 자세</b>: GT 가 없으므로 <b>객체쌍 상대회전 합의</b>로 세운다(물체가 선반에 고정돼 R_i<sup>T</sup>R_j 가 상수).
전 프레임 최빈 군집으로 교정한 뒤, 한 프레임에서 <b>자기 자신을 뺀 나머지 객체</b>로 예측하고 파트너 2개 이상이 30° 안에서 일치할 때만 채택.
대칭 선언 객체(Sikhye·Sauce·Mugcup)는 동치 회전을 모두 고려한다.<br>
<b>정답</b> = 기준에서 <b>{OK_DEG:.0f}° 이내</b>. 하나의 자세가 아니라 <b>fine 정제가 수렴하는 자세 그룹</b>이다
(포획범위 45~60°, 20° 틀어져도 95% 복귀).
</div>""")

    # 요약 표
    P.append("<h2>정답 선택률 — 기하 점수만 vs 텍스처 검증</h2>")
    P.append("<table><tr><th class='l'>객체</th><th>사례</th>"
             f"<th>기하만 ≤{OK_DEG:.0f}°</th><th>검증 ≤{OK_DEG:.0f}°</th>"
             f"<th>기하만 ≤{NEAR_DEG:.0f}°</th><th>검증 ≤{NEAR_DEG:.0f}°</th>"
             "<th>기하만 중앙</th><th>검증 중앙</th><th>교정</th><th>파손</th></tr>")
    for (o, n, g30, v30, g45, v45, mg, mv, fix, brk) in tbl:
        nm = "<b>전체</b>" if o == "__ALL__" else html.escape(o)
        d = v30 - g30
        col = "gain" if d > 0.05 else ("loss" if d < -0.05 else "")
        P.append(f"<tr><td class='l'>{nm}</td><td>{n}</td>"
                 f"<td>{g30:.1f}%</td><td class='{col}'>{v30:.1f}%</td>"
                 f"<td>{g45:.1f}%</td><td>{v45:.1f}%</td>"
                 f"<td>{mg:.1f}°</td><td>{mv:.1f}°</td>"
                 f"<td class='gain'>{fix}</td><td class='loss'>{brk}</td></tr>")
    P.append("</table>")
    P.append("<p style='font-size:12px;color:#999'>‘교정’은 기하만으로는 정답 밖이던 것이 검증으로 정답이 된 건수, "
             "‘파손’은 그 반대다. 중앙값은 기준 자세 잡음(파트너 편차 12~13°)보다 작은 차이라 판정에 쓰지 말 것.</p>")

    P.append("<p class='nav'><b>바로가기:</b> " +
             " ".join(f"<a href='#{html.escape(o)}'>{html.escape(o)}</a>" for o in objs) + "</p>")

    # 사례
    cur = None
    for c in show:
        if c["object"] != cur:
            cur = c["object"]
            P.append(f"<h2 id='{html.escape(cur)}'>{html.escape(cur)}</h2>")
        bgr = frames[c["i"]][1]
        tp, tc = tem.get(c["object"], (None, None))
        if tp is None:
            continue
        box = c["bbox"]
        t_sel = c["pick_ver"]["t_mm"]
        inp = b64(render(bgr, None, None, tp, tc, K, box))
        ref = b64(render(bgr, c["ref"], t_sel, tp, tc, K, box))
        v = c["verify"]
        d30 = ("<span class='gain'>교정</span>" if c["a_geo"] > OK_DEG >= c["a_ver"] else
               "<span class='loss'>파손</span>" if c["a_ver"] > OK_DEG >= c["a_geo"] else "")
        P.append("<div class='case'>")
        P.append(f"<div class='meta'>frame {c['i']:04d} · bbox {box[2]-box[0]}x{box[3]-box[1]}px · "
                 f"판정 <b>{v['verdict']}</b> · 신뢰도 {v['conf']} · 마진 {v['margin']} · "
                 f"모드 {v['n_modes']} · 파트너 {c['npart']} · "
                 f"기하선택 <b class='{cls_of(c['a_geo'])}'>{c['a_geo']:.0f}°</b> → "
                 f"검증선택 <b class='{cls_of(c['a_ver'])}'>{c['a_ver']:.0f}°</b> "
                 f"(후보 중 최선 {c['best']:.0f}°) {d30}</div>")
        for lab, key in (("A 기하만", "rank_geo"), ("B 검증", "rank_s")):
            P.append("<div class='strip'>")
            P.append(f"<div class='lab'>{lab}</div>")
            if key == "rank_geo":
                P.append(f"<div class='tile inp'><img src='{inp}'><br>입력</div>")
                P.append(f"<div class='tile ref'><img src='{ref}'><br>기준</div>")
            else:
                P.append("<div class='tile'><div style='width:132px'></div></div>" * 2)
            ranked = sorted([x for x in v["cands"] if x[key] < a.topn], key=lambda x: x[key])
            for x in ranked:
                ang = V.ang_deg(c["ref"], np.array(x["R"]), c["object"])
                sel = " sel" if x[key] == 0 else ""
                im = b64(render(bgr, x["R"], x["t_mm"], tp, tc, K, box))
                sc = (f"geo {x['geo']:.2f}" if key == "rank_geo" else f"s {x['s']:.2f}")
                P.append(f"<div class='tile{sel}'><img src='{im}'><br>"
                         f"<span class='{cls_of(ang)}'>{ang:.0f}°</span> · {sc}</div>")
            P.append("</div>")
        P.append("</div>")

    P.append("</body></html>")
    out = a.out if os.path.isabs(a.out) else os.path.join(REPO, a.out)
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text("\n".join(P), encoding="utf-8")
    print(f"[출력] {out}  ({os.path.getsize(out)/1e6:.1f} MB)")

    for (o, n, g30, v30, *_r) in tbl:
        print(f"  {o:24s} n={n:5d}  기하 {g30:5.1f}%  검증 {v30:5.1f}%")


if __name__ == "__main__":
    main()
