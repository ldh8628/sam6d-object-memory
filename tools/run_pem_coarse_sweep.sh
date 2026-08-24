#!/usr/bin/env bash
# tools/run_pem_coarse_sweep.sh
#
# coarse_npoint 를 여러 값으로 바꿔가며 longcircle2_sam 전체(2130 프레임)에 대해
# verify_eval.py 를 반복 실행한다. 각 실행은 동일 씨앗·동일 detection·동일 카메라
# intrinsics·동일 template·동일 checkpoint 를 쓰며, 결과는 실행값별 디렉터리에 저장한다.
# 기존 196/392 baseline 을 절대 건드리지 않는다.

set -o pipefail
# conda activate hooks reference unset env vars; keep -u off intentionally.

REPO="/home/etri/sam6d_realtime/sam6d_realtime"
OUT_ROOT="${REPO}/output/pem_coarse_sweep_v1"
LOG_ROOT="${OUT_ROOT}/logs"
BAG="data/longcircle2_sam"
SETTINGS="off"
TOPN=10
STRIDE_DIAG=8

# 사용자가 요구한 최소 sweep + 추가 확장값
VALUES=(${VALUES:-196 256 392 512 768 1024 1536 2048 3072 4096})

mkdir -p "${OUT_ROOT}" "${LOG_ROOT}"

# 스윕 전체 상위 provenance
python3 - "${OUT_ROOT}" <<'PY' >/dev/null
import json, os, sys
outdir = sys.argv[1]
p = os.path.join(outdir, "sweep-metadata.json")
if not os.path.exists(p):
    with open(p, "w") as f:
        json.dump({"schema_version": 1,
                   "purpose": "coarse_npoint sweep over longcircle2_sam",
                   "runs": []}, f, indent=2)
PY

source /home/etri/miniconda3/etc/profile.d/conda.sh
conda activate sam6d
cd "${REPO}"

for N in "${VALUES[@]}"; do
    OUT="${OUT_ROOT}/n${N}"
    LOG="${LOG_ROOT}/n${N}.log"
    VRAM="${LOG_ROOT}/n${N}.vram.jsonl"
    if [[ -s "${OUT}/off.provenance.json" ]]; then
        echo "[sweep] n=${N} already complete → skip" | tee -a "${LOG_ROOT}/summary.log"
        continue
    fi
    rm -rf "${OUT}"
    mkdir -p "${OUT}"
    echo "[sweep] n=${N} start $(date -Iseconds)" | tee -a "${LOG_ROOT}/summary.log"

    # 이 실행 동안만 VRAM 을 1 초 간격으로 기록
    (
        while true; do
            ts=$(date +%s)
            line=$(nvidia-smi --query-gpu=memory.used,memory.total,utilization.gpu \
                              --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
            printf '{"t":%s,"line":"%s"}\n' "$ts" "$line" >> "${VRAM}"
            sleep 1
        done
    ) &
    VMONPID=$!
    trap "kill ${VMONPID} 2>/dev/null" EXIT

    T0=$(date +%s.%N)
    set +e
    python temp/verify_eval.py \
        --bag "${BAG}" \
        --out "${OUT}" \
        --device cuda:0 \
        --stride 1 \
        --settings "${SETTINGS}" \
        --pem-diagnostic-topn "${TOPN}" \
        --pem-diagnostic-stride "${STRIDE_DIAG}" \
        --pem-coarse-npoint "${N}" \
        --skip-summary-report \
        >>"${LOG}" 2>&1
    RC=$?
    set -e
    T1=$(date +%s.%N)
    kill ${VMONPID} 2>/dev/null || true
    trap - EXIT

    ELAPSED=$(python3 -c "print(f'{${T1}-${T0}:.2f}')")
    echo "[sweep] n=${N} rc=${RC} elapsed=${ELAPSED}s" | tee -a "${LOG_ROOT}/summary.log"

    # per-run manifest
    python3 - "${OUT}" "${N}" "${RC}" "${ELAPSED}" "${VRAM}" <<'PY'
import json, os, sys
outdir, n, rc, elapsed, vram = sys.argv[1:]
prov = os.path.join(outdir, "off.provenance.json")
prov_ok = os.path.exists(prov)
manifest = {"schema_version": 1,
            "coarse_npoint": int(n),
            "return_code": int(rc),
            "elapsed_sec": float(elapsed),
            "provenance_written": prov_ok,
            "vram_log": vram}
with open(os.path.join(outdir, "sweep-run.json"), "w") as f:
    json.dump(manifest, f, indent=2)
PY
done

echo "[sweep] ALL DONE $(date -Iseconds)" | tee -a "${LOG_ROOT}/summary.log"
