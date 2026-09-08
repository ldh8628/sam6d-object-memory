#!/usr/bin/env bash
# tools/finalize_pem_coarse_sweep.sh
#
# 스윕이 끝난(또는 부분 완료된) 후 실행하면:
#   1. analyzer 를 돌려 point-count-sweep.{json,csv,md} 갱신
#   2. 추천 npoint 에 대해 handle-flip 시각 증거 최대 20 장 렌더링
#   3. output/pem_coarse_sweep_v1/README.md 를 갱신
# 스윕이 아직 실행 중이어도 안전하다(진행 중인 n<x> 는 자동 skip).

set -o pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SWEEP="${REPO}/output/pem_coarse_sweep_v1"
ANALYSIS="${SWEEP}/analysis"

if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
else
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
fi
conda activate sam6d
cd "${REPO}"

echo "[finalize] analysis"
python tools/analyze_pem_coarse_sweep.py --sweep-dir "${SWEEP}" --baseline-npoint 196 || {
    echo "[finalize] analyzer failed"; exit 1;
}

# extract recommended npoint from json
REC=$(python3 -c "
import json, sys
data = json.load(open('${ANALYSIS}/point-count-sweep.json'))
runs = [e for e in data['runs'] if e.get('effectively_complete')]
prev_rate=None; prev_np=None; saturated=None
for e in runs:
    s = e['summary']
    denom = s.get('iou_ge_050_count_well_obs') or 0
    rate = (s['success_count_target']/denom) if denom else 0.0
    if prev_rate is not None and (rate - prev_rate)*100 < 1.0:
        saturated = prev_np; break
    prev_rate=rate; prev_np=e['npoint']
if saturated is None and runs:
    saturated = runs[-1]['npoint']
cand = [e for e in runs if e['npoint'] <= saturated]
if not cand: cand = runs
best = min(cand, key=lambda e: (e['elapsed_sec'] or 1e12, e['npoint']))
print(best['npoint'])
")
echo "[finalize] recommended npoint = ${REC}"

# render handle-flip evidence for recommended npoint (best-effort)
if [[ -n "${REC}" ]]; then
    EVIDENCE="${ANALYSIS}/mugcup_handle_flip_evidence_n${REC}"
    mkdir -p "${EVIDENCE}"
    echo "[finalize] rendering handle-flip evidence for n=${REC}"
    python tools/render_mugcup_handle_evidence.py \
        --detail "${ANALYSIS}/detail_mugcup_handle_fail.jsonl" \
        --npoint "${REC}" \
        --out "${EVIDENCE}" \
        --max 20 2>&1 | tail -20 || echo "[finalize] evidence render failed (non-fatal)"
fi

# top-level README
python3 - "${SWEEP}" "${REC}" <<'PY'
import json, os, sys
sweep, rec = sys.argv[1], sys.argv[2]
data = json.load(open(f"{sweep}/analysis/point-count-sweep.json"))
runs = data['runs']
lines = [
    "# PEM coarse_npoint sweep on longcircle2_sam",
    "",
    f"- baseline (기준): n={data.get('baseline_npoint')}",
    f"- 추천 운영값: **n={rec}**",
    f"- 전체 detection: {runs[0]['summary']['total_detections'] if runs else '?'}",
    "",
    "## 재현 방법",
    "```",
    "cd sam6d_ws",
    "conda activate sam6d",
    "# 스윕 실행",
    "bash tools/run_pem_coarse_sweep.sh",
    "# 분석 (도중에도 재실행 안전)",
    "bash tools/finalize_pem_coarse_sweep.sh",
    "```",
    "",
    "## 산출물",
    "- 각 point count 별 원본 결과: `n<value>/off.jsonl`, `n<value>/off.provenance.json`",
    "- `analysis/point-count-sweep.json` — 모든 지표",
    "- `analysis/point-count-sweep.csv` — 표 요약",
    "- `analysis/point-count-sweep.md` — 사람이 읽는 보고서",
    "- `analysis/per-object-success.csv` — 객체별 성공률",
    "- `analysis/mugcup-60-61-detail.csv` — Mugcup 60/61 상세",
    "- `analysis/detail_pose_changed.jsonl` — 196 baseline 대비 pose 변경 detection",
    "- `analysis/detail_iou_pass_but_fail.jsonl` — IoU>=0.5 인데 실패한 detection",
    "- `analysis/detail_mugcup_handle_fail.jsonl` — Mugcup handle-visible 실패",
    f"- `analysis/mugcup_handle_flip_evidence_n{rec}/` — 시각 증거 (최대 20 장)",
    "",
    "## 실행 이력",
    "",
    "| n | rc | elapsed s | per-det ms | VRAM MiB | IoU>=0.5 (well_obs) | success_target | rate |",
    "|---|----|----------:|-----------:|---------:|-------------------:|---------------:|-----:|",
]
for e in runs:
    s = e['summary']
    denom = s.get('iou_ge_050_count_well_obs') or 0
    rate = (s['success_count_target']/denom*100) if denom else None
    rate_s = f"{rate:.2f}%" if rate is not None else "-"
    pdms = e['per_detection_sec']*1000 if e['per_detection_sec'] else 0.0
    lines.append(f"| {e['npoint']} | {e['return_code']} | {e['elapsed_sec']:.1f} |"
                 f" {pdms:.1f} | {e['vram_max_mib']} |"
                 f" {s['iou_ge_050_count_well_obs']} | {s['success_count_target']} | {rate_s} |")
lines.append("")
lines.append("전체 지표는 `analysis/point-count-sweep.md` 참고.")
with open(f"{sweep}/README.md", "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"[finalize] wrote {sweep}/README.md")
PY

echo "[finalize] done"
