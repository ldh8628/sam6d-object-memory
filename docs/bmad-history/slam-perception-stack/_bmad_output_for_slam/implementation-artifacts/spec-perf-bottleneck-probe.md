---
title: 'SAM-6D ISM 병목 계측 격리 환경 (_perf_bottleneck_probe_2026_07_06)'
type: 'infra/profiling'
created: '2026-07-06'
status: 'done'
route: 'one-shot'
---

# SAM-6D ISM 병목 계측 격리 환경

## Intent

**Problem:** technical research 보고서(2026-07-06)가 도출한 병목 후보 Top 5(DINOv2 batch=1 forward, MobileSAM 객체당 인코딩, YOLO-World, 출력 Disk IO, per-object 전처리 중복)를 실측해야 하지만, 원본 pipeline 파일은 절대 수정할 수 없다.

**Approach:** `sam6d_ws/_perf_bottleneck_probe_2026_07_06/` 격리 폴더에 `yolo_ism.py`/`yolo_ism_object_n.py` 복사본을 두고 복사본에만 PERF 훅(`perf/stage_timer.py`, 기본 OFF)을 삽입. `--perf-out` 지정 시에만 run_meta.json + per_frame/per_object/stage_summary CSV 생성. CUDA stage 경계 `torch.cuda.synchronize`, warm-up frame 플래그·통계 제외, atexit로 크래시 시 partial summary 보존. 원본 6개 보호 파일은 전후 md5 checksum으로 비수정 입증. 폴더 삭제만으로 원상 복구.

## Suggested Review Order

1. [reports/quick-dev-result.md](../../sam6d_ws/_perf_bottleneck_probe_2026_07_06/reports/quick-dev-result.md) — 결과 요약·검증 증거(여기만 봐도 충분)
2. [perf/stage_timer.py](../../sam6d_ws/_perf_bottleneck_probe_2026_07_06/perf/stage_timer.py) — 계측 코어: sync/warm-up/summary/rank 로직
3. [copied/yolo_ism_object_n.py](../../sam6d_ws/_perf_bottleneck_probe_2026_07_06/copied/yolo_ism_object_n.py) — 원본 대비 diff 관점: import 가드(§상단), 캐시 격리(main 초입), recognize() PERF 훅, frame loop 래핑
4. [copied/yolo_ism.py](../../sam6d_ws/_perf_bottleneck_probe_2026_07_06/copied/yolo_ism.py) — 변경 1곳(REPO_ROOT)만 확인
5. [README.md](../../sam6d_ws/_perf_bottleneck_probe_2026_07_06/README.md) — 실행법·해석 주의

리뷰 포인트: 알고리즘 판정(threshold/게이트/정렬)은 원본과 동일해야 하며, smoke 결과 accepted=23 obj-frames가 계측 ON/OFF에서 일치함이 그 증거.
