#!/usr/bin/env python3
"""small_object_report.py — bbox 가 작은 검출에서 포즈 후보가 어떻게 구성되는지 전부 본다.

가설: '후보에 정답이 없다 / 정답이 안 뽑힌다' 는 경우는 대부분 물체가 **작게 찍힌** 것이다.
여기서는 bbox 짧은 변이 임계(기본 40px) 미만인 검출을 **전부** 찾아, 한 건마다
  (A) 기하 점수 상위 N 후보   (B) 텍스처까지 넣어 재정렬한 상위 N 후보
를 그려 준다. 선택된 후보는 빨간 테두리, 기준 자세는 파란 테두리다.

추론을 다시 하지 않는다 — `verify_report.py` 가 남긴 `*.cases.json`(후보 사원수 + 기하/결합 점수)과
bag 프레임만 읽는다. ⚠ 후보마다의 병진은 덤프에 없으므로 **모든 후보를 그 검출의 위치에 놓고
자세만** 그린다(후보 간 차이는 사실상 회전이고, 병진은 깊이가 잡아 준다).

    conda activate sam6d
    python temp/small_object_report.py --cases outputs/test/verify_report_185223.html.cases.json \
        --bag <sam bag> --stride 5 --traj <tum> [--extrinsic X.npy] --out outputs/test/small_0807.html
"""
from __future__ import annotations

import argparse
import html
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, os.path.join(REPO, "realtime"))
sys.path.insert(0, os.path.join(REPO, "temp"))

import verify_eval as V                                            # noqa: E402
from verify_report import angles_to, b64, cls_of, quat_to_R, render  # noqa: E402

OK_DEG = 30.0
GUARD = 0.9
W_GEO_USED = 2.0
BINS = [(0, 20), (20, 30), (30, 40), (40, 60), (60, 10 ** 6)]


def outcome(a, elig, s):
    """성공 / 채점실패(후보엔 있는데 못 고름) / 후보없음."""
    good = np.where((a <= OK_DEG) & elig)[0]
    pick = int(np.argmax(np.where(elig, s, -np.inf)))
    if len(good) == 0:
        return "후보없음", pick
    return ("성공" if a[pick] <= OK_DEG else "채점실패"), pick


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--stride", type=int, required=True)
    ap.add_argument("--traj", required=True)
    ap.add_argument("--extrinsic", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-px", type=int, default=40)
    ap.add_argument("--topn", type=int, default=5)
    ap.add_argument("--tile", type=int, default=104)
    ap.add_argument("--title", default="")
    a = ap.parse_args()

    raw = [r for r in json.load(open(a.cases, encoding="utf-8")) if r.get("quats")]
    # 기준 다시 세우기 (choco 만 2개, 나머지는 단일 인스턴스)
    X = np.load(a.extrinsic) if a.extrinsic else None
    ts, Rs, Ps = V.load_traj(a.traj, X)
    cases = [{"i": r["i"], "stamp": r["stamp"], "object": r["object"], "bbox": r["bbox"],
              "t_mm": r["t_mm"], "R": r["R"], "ref_src": [r["R_geo"], r["R_ver"]]} for r in raw]
    modes, share = V.map_mode_reference(cases, ts, Rs, Ps)
    for r, c in zip(raw, cases):
        r["ref"], r["inst"] = c.get("ref"), c.get("inst")
    raw = [r for r in raw if r["ref"] is not None]
    print(f"[기준] 인스턴스 {len(modes)} · 기준 있는 검출 {len(raw)}")
    for k in sorted(share, key=lambda k: -share[k][1]):
        print(f"    {k[0]:24s} #{k[1]}  최빈군집 {share[k][0]*100:5.1f}%  n={share[k][1]}")

    # 후보 해석
    for r in raw:
        q = np.asarray(r["quats"], float)
        r["_geo"], r["_s"] = q[:, 4], q[:, 5]
        r["_zg"] = (r["_geo"] - r["_geo"].mean()) / (r["_geo"].std() + 1e-9)
        r["_tex"] = r["_s"] - W_GEO_USED * r["_zg"]          # 텍스처 항 복원
        r["_R"] = quat_to_R(q[:, :4])
        r["_a"] = angles_to(r["ref"], r["_R"], r["object"])
        r["_elig"] = (r["_geo"] >= GUARD * r["_geo"].max())
        r["_elig"][0] = True
        r["_kind"], r["_pick"] = outcome(r["_a"], r["_elig"], r["_s"])
        r["_px"] = min(r["bbox"][2] - r["bbox"][0], r["bbox"][3] - r["bbox"][1])
        good = np.where((r["_a"] <= OK_DEG) & r["_elig"])[0]
        r["_best"] = float(r["_a"].min())
        r["_rank_geo"] = int(good.min()) if len(good) else None
        r["_rank_tex"] = (int(np.argsort(np.argsort(-r["_tex"]))[good].min())
                          if len(good) else None)

    small = sorted([r for r in raw if r["_px"] < a.max_px],
                   key=lambda r: (r["object"], r["_px"], r["i"]))
    print(f"[대상] {a.max_px}px 미만 {len(small)} / 전체 {len(raw)}")

    # ---- 프레임 읽기 (필요한 것만) --------------------------------------
    need = {r["i"] for r in small}
    K, frames = V.read_frames(a.bag, a.stride, 0)
    img = {i: frames[i][1] for i in need if i < len(frames)}
    print(f"[프레임] {len(img)} / 필요 {len(need)}")

    # ---- 템플릿 색 점군 --------------------------------------------------
    import torch
    tem = {}
    for o in {r["object"] for r in small}:
        b = torch.load(os.path.join(REPO, "assets", "pem_templates", f"{o}.pt"),
                       map_location="cpu")
        if "tc" in b:
            tem[o] = (b["tp"][0].numpy() * 1000.0, b["tc"][0].numpy())

    # ---- 요약표 ---------------------------------------------------------
    def bin_of(px):
        for lo, hi in BINS:
            if lo <= px < hi:
                return f"{lo}~{hi if hi < 10**6 else ''}px"
        return "?"

    tbl = defaultdict(lambda: defaultdict(int))
    for r in raw:
        tbl[bin_of(r["_px"])][r["_kind"]] += 1
        tbl[bin_of(r["_px"])]["n"] += 1
    per_obj = defaultdict(lambda: defaultdict(lambda: defaultdict(int)))
    for r in raw:
        per_obj[r["object"]][bin_of(r["_px"])][r["_kind"]] += 1
        per_obj[r["object"]][bin_of(r["_px"])]["n"] += 1

    P = ["<!DOCTYPE html><html lang='ko'><head><meta charset='utf-8'>",
         "<title>작은 물체의 포즈 후보 구성</title><style>", """
body{background:#111;color:#ddd;font-family:'Nanum Gothic','Malgun Gothic',sans-serif;margin:0;padding:18px}
h1{color:#ffe066;font-size:22px}h2{color:#9ad0ff;margin-top:30px;border-bottom:1px solid #333;padding-bottom:6px}
table{border-collapse:collapse;font-size:12px;margin:10px 0}
th,td{border:1px solid #333;padding:4px 9px;text-align:right}th{background:#1c1c1c;color:#ffe066}
td.l,th.l{text-align:left}
.A{color:#ff7676}.C{color:#ffb347}.D{color:#7dff9b}
.legend{background:#181818;padding:10px 14px;border-radius:6px;font-size:12px;line-height:1.8}
.nav a{color:#9ad0ff;margin-right:12px;font-size:13px}
.case{border-bottom:1px solid #222;padding:7px 0}
.meta{font-size:11px;color:#bbb;margin-bottom:4px}
.strip{display:flex;gap:5px;align-items:flex-start;overflow-x:auto}
.tile{text-align:center;font-size:10px;color:#aaa}
.tile img{display:block;border:2px solid #2a2a2a;border-radius:3px}
.tile.sel img{border-color:#ff3b3b}.tile.ref img{border-color:#9ad0ff}.tile.inp img{border-color:#555}
.tile.hit img{border-color:#7dff9b}
.lab{width:56px;flex:0 0 56px;font-size:10.5px;color:#8ab;padding-top:36px}
.k성공{color:#7dff9b}.k채점실패{color:#ffb347}.k후보없음{color:#ff7676}
""", "</style></head><body>",
         f"<h1>작은 물체의 포즈 후보 구성{html.escape(a.title)}</h1>",
         f"""<div class='legend'>
<b>대상</b> bbox 짧은 변 <b>{a.max_px}px 미만</b> 검출 <b>{len(small)}</b>건(기준 있는 전체 {len(raw)}건 중)을 <b>전부</b> 싣는다.<br>
<b>(A)</b> 기하 점수 상위 {a.topn} · <b>(B)</b> 텍스처까지 넣어 재정렬한 상위 {a.topn}.
<span style='color:#ff3b3b'>선택=빨강</span> · <span style='color:#9ad0ff'>기준=파랑</span> ·
<span style='color:#7dff9b'>정답(기준 30° 이내)=초록</span>.<br>
<b>타일</b>은 그 자세로 색 있는 템플릿 점군을 얹은 것이다. ⚠ 후보별 병진은 덤프에 없어
<b>모든 후보를 그 검출의 위치에 놓고 자세만</b> 그렸다(후보 간 차이는 사실상 회전이다).<br>
<b>분류</b> <span class='k성공'>성공</span>=선택이 30° 이내 ·
<span class='k채점실패'>채점실패</span>=후보엔 정답이 있는데 못 고름 ·
<span class='k후보없음'>후보없음</span>=기하 상위 100 안에 정답 자세가 아예 없음.<br>
<b>기준</b>: SLAM 궤적으로 지도좌표계 최빈 자세(정지 물체). 실물이 두 개인 choco 만 위치로 인스턴스를 나눈다.
</div>"""]

    P.append("<h2>bbox 크기와 실패의 관계 (기준 있는 전체 검출)</h2>")
    P.append("<table><tr><th class='l'>bbox 짧은 변</th><th>n</th><th>성공</th>"
             "<th>채점실패</th><th>후보없음</th></tr>")
    for lo, hi in BINS:
        k = f"{lo}~{hi if hi < 10**6 else ''}px"
        d = tbl.get(k)
        if not d:
            continue
        n = d["n"]
        P.append(f"<tr><td class='l'>{k}</td><td>{n}</td>"
                 f"<td class='D'>{100*d['성공']/n:.1f}%</td>"
                 f"<td class='C'>{100*d['채점실패']/n:.1f}%</td>"
                 f"<td class='A'>{100*d['후보없음']/n:.1f}%</td></tr>")
    P.append("</table>")

    P.append("<h2>객체 × bbox 크기 — 후보없음 비율</h2>")
    keys = [f"{lo}~{hi if hi < 10**6 else ''}px" for lo, hi in BINS]
    P.append("<table><tr><th class='l'>객체</th>" +
             "".join(f"<th>{k}</th>" for k in keys) + "</tr>")
    for o in sorted(per_obj):
        row = f"<tr><td class='l'>{html.escape(o)}</td>"
        for k in keys:
            d = per_obj[o].get(k)
            row += ("<td>-</td>" if not d else
                    f"<td class='A'>{100*d['후보없음']/d['n']:.0f}%<span style='color:#666'> /{d['n']}</span></td>")
        P.append(row + "</tr>")
    P.append("</table>")

    objs = sorted({r["object"] for r in small})
    P.append("<p class='nav'><b>바로가기:</b> " +
             " ".join(f"<a href='#{html.escape(o)}'>{html.escape(o)}</a>" for o in objs) + "</p>")

    cur = None
    for r in small:
        if r["object"] != cur:
            cur = r["object"]
            n = sum(1 for x in small if x["object"] == cur)
            P.append(f"<h2 id='{html.escape(cur)}'>{html.escape(cur)} — {n}건</h2>")
        if r["i"] not in img or r["object"] not in tem:
            continue
        bgr = img[r["i"]]
        tp, tc = tem[r["object"]]
        box, t = r["bbox"], r["t_mm"]
        inp = b64(render(bgr, None, None, tp, tc, K, box, out=a.tile), q=62)
        ref = b64(render(bgr, r["ref"], t, tp, tc, K, box, out=a.tile), q=62)
        P.append("<div class='case'>")
        P.append(f"<div class='meta'>frame {r['i']:04d} · <b>{r['_px']}px</b> · "
                 f"{t[2]:.0f}mm · <span class='k{r['_kind']}'>{r['_kind']}</span> · "
                 f"판정 {r['verdict']}({r['conf']}) · "
                 f"선택 {r['_a'][r['_pick']]:.0f}° · 후보최선 {r['_best']:.0f}° · "
                 f"정답등수 기하 {r['_rank_geo']} / 텍스처 {r['_rank_tex']}</div>")
        for lab, key in (("A 기하", "geo"), ("B 검증", "tex")):
            order = (np.arange(len(r["_geo"])) if key == "geo"
                     else np.argsort(-np.where(r["_elig"], r["_s"], -np.inf)))
            P.append(f"<div class='strip'><div class='lab'>{lab}</div>")
            if key == "geo":
                P.append(f"<div class='tile inp'><img src='{inp}'><br>입력</div>")
                P.append(f"<div class='tile ref'><img src='{ref}'><br>기준</div>")
            else:
                P.append(f"<div style='width:{2*(a.tile+9)}px;flex:0 0 auto'></div>")
            for k in order[:a.topn]:
                k = int(k)
                ang = float(r["_a"][k])
                sel = " sel" if (k == r["_pick"] and key == "tex") or (k == 0 and key == "geo") else ""
                if not sel and ang <= OK_DEG:
                    sel = " hit"
                im = b64(render(bgr, r["_R"][k], t, tp, tc, K, box, out=a.tile), q=62)
                sc = f"geo {r['_geo'][k]:.2f}" if key == "geo" else f"s {r['_s'][k]:.2f}"
                P.append(f"<div class='tile{sel}'><img src='{im}'><br>"
                         f"<span class='{cls_of(ang)}'>{ang:.0f}°</span> · {sc}</div>")
            P.append("</div>")
        P.append("</div>")

    P.append("</body></html>")
    out = a.out if os.path.isabs(a.out) else os.path.join(REPO, a.out)
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text("\n".join(P), encoding="utf-8")
    print(f"[출력] {out}  ({os.path.getsize(out)/1e6:.1f} MB)")
    for lo, hi in BINS:
        k = f"{lo}~{hi if hi < 10**6 else ''}px"
        d = tbl.get(k)
        if d:
            print(f"  {k:10s} n={d['n']:5d} 성공 {100*d['성공']/d['n']:5.1f}% "
                  f"채점실패 {100*d['채점실패']/d['n']:5.1f}% 후보없음 {100*d['후보없음']/d['n']:5.1f}%")


if __name__ == "__main__":
    main()
