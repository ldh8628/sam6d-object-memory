#!/usr/bin/env python3
"""build_260714_report.py — one HTML page for all 260714 pairs.

Collects, per pair, the synchronised video (both cameras + one 2D map per SLAM
algorithm) and the numbers from rig_offset.json, and lays them out with the
cross-session comparison on top.

    python3 integration/build_260714_report.py
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT_ROOT = HERE / "output" / "260714_pairs"
TITLE = {"orb3": "ORB-SLAM3", "rtabmap": "RTAB-Map", "hdl_graph_slam": "hdl_graph_slam"}


def fmt_t(v):
    return ", ".join(f"{x:+.3f}" for x in v) if v else "—"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fit", default=str(OUT_ROOT / "rig_offset.json"))
    ap.add_argument("--out", default=str(OUT_ROOT / "index.html"))
    a = ap.parse_args()

    data = json.loads(Path(a.fit).read_text())
    pairs, cross = data["pairs"], data.get("cross_session", {})
    algos = list(TITLE)

    # ---- per-pair rows ---------------------------------------------------
    rows = []
    for pair, byalgo in pairs.items():
        for algo in algos:
            r = byalgo.get(algo, {})
            if not r.get("ok"):
                rows.append(f"<tr><td>{pair}</td><td>{TITLE[algo]}</td>"
                            f"<td colspan=6 class=skip>{r.get('reason', '—')}</td></tr>")
                continue
            d, dr = r["dev_trans_cm"], r["dev_rot_deg"]
            cls = "good" if d["median"] < 10 else ("warn" if d["median"] < 25 else "bad")
            rows.append(
                f"<tr><td>{pair}</td><td>{TITLE[algo]}</td>"
                f"<td>{r['n_pairs']}</td><td>{r['tau_s']:+.2f}</td>"
                f"<td>{r['X_rot_deg']:.2f}°</td>"
                f"<td>{r['X_t_norm_m']:.3f} <span class=dim>({fmt_t(r['X_t_m'])})</span></td>"
                f"<td class={cls}>{d['median']:.1f} / {d['p90']:.1f} / {d['max']:.1f}</td>"
                f"<td>{dr['median']:.2f} / {dr['p90']:.2f} / {dr['max']:.2f}</td></tr>")

    # ---- cross-session ---------------------------------------------------
    crows = []
    for algo in algos:
        c = cross.get(algo)
        if not c:
            crows.append(f"<tr><td>{TITLE[algo]}</td><td colspan=3 class=skip>—</td></tr>")
            continue
        crows.append(f"<tr><td>{TITLE[algo]}</td><td>{c['sessions']}</td>"
                     f"<td>{c['rot_max_diff_deg']:.2f}°</td>"
                     f"<td>{c['t_range_cm']:.1f} cm "
                     f"<span class=dim>(축별 σ {', '.join(f'{v:.1f}' for v in c['t_spread_cm'])})</span>"
                     f"</td></tr>")

    # ---- per-pair video blocks ------------------------------------------
    blocks = []
    for pair in pairs:
        meta_p = OUT_ROOT / pair / "meta.json"
        vid = OUT_ROOT / pair / "sync_view.mp4"
        if not vid.is_file():
            continue
        meta = json.loads(meta_p.read_text()) if meta_p.is_file() else {}
        chips = "".join(
            f"<span class='chip {'ok' if x['ok'] else 'off'}'>{x['title']}: "
            + (f"{x['dev_median_cm']:.1f} cm" if x["ok"] else x["note"]) + "</span>"
            for x in meta.get("algos", []))
        blocks.append(f"""
<section>
  <h3>{pair} <span class=dim>· 시계 오차 {meta.get('tau_s', 0):+.2f}s ·
      {meta.get('frames', 0)}프레임 / {meta.get('duration_s', 0)}초</span></h3>
  <div class=chips>{chips}</div>
  <video src="{pair}/sync_view.mp4" poster="{pair}/poster.jpg" controls preload="none"></video>
</section>""")

    html = f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<title>260714 pairs — SLAM 알고리즘별 두 카메라 위치</title>
<style>
 body {{ font-family: system-ui, -apple-system, "Noto Sans KR", sans-serif; margin:0;
        padding:22px 26px 70px; background:#fbfbfc; color:#181818; }}
 h1 {{ font-size:21px; margin:0 0 6px; }} h2 {{ font-size:17px; margin:30px 0 8px; }}
 h3 {{ font-size:15px; margin:0 0 6px; }}
 .sub {{ color:#666; font-size:13px; margin-bottom:14px; max-width:900px; line-height:1.6 }}
 table {{ border-collapse:collapse; font-size:13px; background:#fff; margin-bottom:6px }}
 th,td {{ border:1px solid #e4e4e8; padding:5px 10px; text-align:left; }}
 th {{ background:#f4f4f6; }} .dim {{ color:#888; font-size:12px }}
 .good {{ background:#e8f7ec }} .warn {{ background:#fff6e0 }} .bad {{ background:#fdeaea }}
 .skip {{ color:#999; font-style:italic }}
 section {{ margin:26px 0 34px }}
 video {{ max-width:100%; max-height:82vh; display:block; background:#000;
          border-radius:8px; box-shadow:0 2px 12px rgba(0,0,0,.12) }}
 .chips {{ margin:6px 0 10px }}
 .chip {{ display:inline-block; font-size:12px; padding:3px 9px; border-radius:12px;
          margin-right:6px; background:#eef3fb; color:#2a4a7a }}
 .chip.off {{ background:#f0f0f0; color:#888 }}
 code {{ background:#f0f0f2; padding:1px 5px; border-radius:4px }}
 .note {{ background:#fff; border:1px solid #e4e4e8; border-left:4px solid #d0a000;
          padding:10px 14px; font-size:13px; max-width:900px; line-height:1.6 }}
</style></head><body>

<h1>260714 6쌍 — SLAM 알고리즘별 2D 지도와 두 카메라 위치</h1>
<div class="sub">SLAM 카메라와 SAM 카메라는 한 대차에 고정돼 있으므로, 둘의 상대 위치는 주행 내내
같아야 합니다. 각 알고리즘을 두 bag에 <b>따로</b> 돌린 뒤 AX = ZB로 맵 프레임 정합 M과 리그 오프셋 X를
동시에 풀고, <b>X(t)가 시간에 따라 얼마나 흔들리는지</b>를 측정했습니다. 이 흔들림은 GT 없이 계산되는
자기일관성 지표입니다 — 드리프트·스케일 오차·트래킹 노이즈가 모두 여기에 잡힙니다.</div>

<div class="note"><b>hdl_graph_slam은 이 데이터셋에서 실행 불가</b>입니다. 260714 bag에는
LiDAR가 없습니다(<code>color/image_raw</code>, <code>aligned_depth_to_color/image_raw</code>,
camera_info 2개 — 총 4토픽뿐). hdl_graph_slam은 3D LiDAR 포인트클라우드를 입력으로 요구하므로
데이터가 없으면 돌릴 대상이 없습니다.</div>

<h2>쌍별 결과</h2>
<table>
<tr><th>쌍</th><th>알고리즘</th><th>표본</th><th>시계오차 τ [s]</th><th>X 회전</th>
    <th>X 병진 |t| [m]</th><th>오프셋 편차 med/p90/max [cm]</th><th>회전 편차 med/p90/max [°]</th></tr>
{''.join(rows)}
</table>
<div class="dim">편차 = X(t)와 그 세션 중앙값의 차이. 작을수록 알고리즘이 자기일관적입니다.</div>

<h2>세션 간 재현성 (리그는 그대로였으므로 X가 같아야 함)</h2>
<table>
<tr><th>알고리즘</th><th>세션 수</th><th>회전 최대 차이</th><th>병진 범위</th></tr>
{''.join(crows)}
</table>

<h2>쌍별 영상</h2>
<div class="sub">위 두 패널 = 같은 순간의 두 카메라 RGB. 아래 = 알고리즘별 2D 지도(같은 순간의
두 카메라 위치·시선). 지도 아래 스파크라인이 그 세션 동안의 오프셋 편차이고, 세로선이 현재 시점입니다.
평평할수록 좋습니다.</div>
{''.join(blocks)}

</body></html>"""
    Path(a.out).write_text(html, encoding="utf-8")
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
