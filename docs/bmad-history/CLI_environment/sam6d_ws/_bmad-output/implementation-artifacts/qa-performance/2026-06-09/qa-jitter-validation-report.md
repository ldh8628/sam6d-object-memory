# QA Jitter Validation Report

Date: 2026-06-09

## Phase 1. Pose Output Logging Audit

- Previous CSV columns: `frame, ism_ok, ism_candidates, best_ism_score, pose_output, skip_reason, score, tx, ty, tz`
- Current CSV columns: `frame, ism_ok, ism_candidates, best_ism_score, pose_output, skip_reason, confidence_score, score, tx, ty, tz, qx, qy, qz, qw`
- Translation: `tx`, `ty`, `tz` 기록됨
- Rotation: 기존 CSV에는 누락, 패치 후 `qx`, `qy`, `qz`, `qw` 기록됨
- Confidence: 기존 `score`, 패치 후 `confidence_score` alias 추가
- Rejection reason: `skip_reason` 기록됨
- Frame id: `frame` 기록됨
- Timestamp: 추출 CSV/pose CSV 모두 절대 timestamp 없음. 현재 frame id 기반 분석만 가능

## Phase 2. Rotation Logging Patch

- 수정 파일: `tools/run_sam6d_reliability_eval.py`
- 추가 필드: `confidence_score`, `qx`, `qy`, `qz`, `qw`
- 추가 옵션: `--disable-temporal`, `--temporal-translation-alpha`, `--temporal-rotation-alpha`
- 검증: `py_compile` 통과

## Phase 3-4. only_Milk Jitter Metrics

Metric definition:

- `T Delta`: consecutive valid pose translation distance in mm.
- `R Delta`: consecutive valid pose quaternion angular distance in degrees, using `2*acos(abs(dot(q0,q1)))`.
- `R Std`: standard deviation of angular distance from mean quaternion.
- `T Std Norm`: global position spread and includes camera motion; for jitter assessment, frame-to-frame delta is the stronger signal.
- `baseline` below is a fresh rerun with the patched evaluation CSV logging, not the historical QA CSV from the previous report.

| Condition | Final Pose Recall | Output Rate | Missing | Longest Valid | Longest Missing | T Std Norm mm | T Delta Mean mm | T Delta Max mm | R Std deg | R Delta Mean deg | R Delta Max deg | Conf Mean | Conf Std |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 100.00% | 100.00% | 0 | 173 | 0 | 1279.331 | 1053.316 | 3615.249 | 48.145 | 102.727 | 179.810 | 0.0764 | 0.0371 |
| recall_recovery_no_temporal | 99.42% | 99.42% | 1 | 102 | 1 | 118.692 | 44.743 | 206.355 | 72.374 | 69.321 | 179.853 | 0.6064 | 0.0272 |
| recall_recovery_smoothing | 99.42% | 99.42% | 1 | 102 | 1 | 106.782 | 20.275 | 68.428 | 17.103 | 8.591 | 56.556 | 0.6049 | 0.0262 |

### Smoothing Effect

- Translation frame-to-frame mean: 44.743 -> 20.275 mm
- Translation frame-to-frame max: 206.355 -> 68.428 mm
- Rotation frame-to-frame mean: 69.321 -> 8.591 deg
- Rotation frame-to-frame max: 179.853 -> 56.556 deg

## Phase 5. SLAM Hidden FPR Failure Analysis

- Visible Recall: 80.75%
- Hidden FPR(non-hand): 1.38% (3 frames)
- Hand FPR: 0.00%

| Frame | Candidate Score | Pose Conf | BBox Area | Mask Area | Depth Valid | Score Margin | Why Passed |
|---|---:|---:|---:|---:|---:|---:|---|
| 001245 | 0.5192 | 0.5192 | 9724.0 | 8502 | 0.9672 | 0.0493 | candidate score >= 0.50; bbox area >= 3000; mask area >= 3000; depth valid ratio >= 0.80; pose confidence >= 0.50 |
| 001720 | 0.5628 | 0.5628 | 10218.0 | 7259 | 0.9329 | 0.0230 | candidate score >= 0.50; bbox area >= 3000; mask area >= 3000; depth valid ratio >= 0.80; pose confidence >= 0.50 |
| 001808 | 0.5827 | 0.5776 | 4182.0 | 3156 | 0.9883 | 0.0162 | candidate score >= 0.50; bbox area >= 3000; mask area >= 3000; depth valid ratio >= 0.80; pose confidence >= 0.50 |

### Threshold Impact

For `min_bbox_area=3000`, `min_mask_area=3000`:

| Score Threshold | SLAM Visible Recall | SLAM Hidden FPR | SLAM Hand FPR | only_Milk Visible Recall |
|---:|---:|---:|---:|---:|
| 0.50 | 80.75% | 1.38% | 0.00% | 99.42% |
| 0.55 | 67.91% | 0.92% | 0.00% | 98.27% |
| 0.60 | 14.97% | 0.00% | 0.00% | 69.94% |

단순 score threshold 상향은 FP를 줄이지만 visible recall 목표를 깨뜨린다. `0.55`는 SLAM visible recall 67.91%, `0.60`은 14.97%로 하락한다.

## Final QA Answers

1. Recall 복구 수정은 유지되는가? Yes. only_Milk final pose recall 99.42%, SLAM visible recall 80.75%.
2. only_Milk pose output continuity는 충분한가? Yes. output rate 99.42%, missing 1/173, longest valid segment 102.
3. translation jitter는 허용 가능한가? Conditional. smoothing 후 F2F mean 20.275 mm, max 68.428 mm. 정량 acceptance threshold가 없어 PASS 대신 CONCERNS.
4. rotation jitter는 허용 가능한가? Conditional. smoothing 후 F2F mean 8.591 deg, max 56.556 deg. acceptance threshold 미정으로 CONCERNS.
5. hidden FPR 1.38%는 production 적용 가능한가? CONCERNS. 성공 기준 FPR <= 5%는 만족하지만, 실제 hidden pose output 3건은 production에서는 잘못된 pose 발행이므로 모니터링/추가 검증 gate가 필요.
6. 추가 튜닝이 필요한가? Yes, 단순 hard threshold가 아니라 geometric/depth consistency 또는 score-margin 기반 영향 분석이 필요하다. 현재 threshold 상향은 recall을 다시 붕괴시킨다.

## Artifacts

- Metrics JSON: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/qa_jitter_validation/only_Milk_jitter_metrics.json`
- Column audit JSON: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/qa_jitter_validation/pose_outputs_column_audit.json`
- Rotation logging diff: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/qa_jitter_validation/rotation_logging_patch.diff`
- Jitter comparison CSV: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/qa_jitter_validation/only_Milk_jitter_comparison.csv`
- Jitter plot: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/qa_jitter_validation/only_Milk_jitter_comparison.png`
- Hidden FP CSV: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/qa_jitter_validation/SLAM_hidden_false_positive_frames.csv`
- Hidden FP montage: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/qa_jitter_validation/SLAM_hidden_false_positive_montage.jpg`
