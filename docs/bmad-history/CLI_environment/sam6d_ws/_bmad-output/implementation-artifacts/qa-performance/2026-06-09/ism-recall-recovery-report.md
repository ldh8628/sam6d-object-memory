# ISM Recall Recovery QA Report

Date: 2026-06-09

## 변경 요약

- `min_bbox_area`: 9000 -> 3000
- `min_mask_area`: 9000 -> 3000
- `min_final_score`: 0.55 -> 0.50
- `object_existence.min_score`: 0.55 -> 0.50
- `pose_confidence.min_score`: 0.55 -> 0.50
- temporal jump hard reject: 비활성화, smoothing-only 유지 (`max_translation_jump_mm=0`, `max_rotation_jump_deg=0`, `hold_last_good_frames=0`)
- no candidate / low object score 이후 temporal state reset 추가

## Root Cause Evidence

- only_Milk `no_ism_candidate` 86건은 raw candidate 부재가 아니라 후처리 gate 제거였다.
- 원인 분류: bbox+mask area 부족 78건, mask area 부족 7건, depth 문제 1건.
- visible Milk 후보 median area가 기존 9000 threshold보다 작았다: SLAM bbox 6678 / mask 4736, only_Milk bbox 7436 / mask 6873.
- only_Milk temporal reject 78건의 translation jump median은 161.12 mm, rotation median은 13.89 deg로, 기존 25 mm hard reject가 정상 카메라 이동까지 제거했다.

## Before / After 비교

### SLAM_with_milk_nomilk

| Variant | Visible Recall | Hidden FPR(non-hand) | Hand FPR | Final Pose Outputs | no_ism_candidate | temporal_jump_reject |
|---|---:|---:|---:|---:|---:|---:|
| improved | 1.60% | 0.00% | 0.00% | 3 | 506 | 3 |
| improved_recall_recovery | 80.75% | 1.38% | 0.00% | 154 | 358 | 0 |

Re-visible recovery: 3/3 segments, success=100.00%, max latency=0 frames

### only_Milk

| Variant | Candidate Recall | Final Pose Recall | no_ism_candidate | temporal_jump_reject | Pose Outputs | Longest Gap |
|---|---:|---:|---:|---:|---:|---:|
| improved | 50.29% | 1.73% | 86 | 78 | 3 | 153 |
| improved_recall_recovery | 99.42% | 99.42% | 1 | 0 | 172 | 1 |

## Temporal / Jitter 산출 가능성

- SLAM_with_milk_nomilk: valid poses=154/512, position std norm=212.9217 mm, frame delta mean=23.1651 mm, jitter score=236.0868 mm. Rotation column은 `pose_outputs.csv`에 없어 회전 jitter는 이 평가 출력만으로 산출 불가.
- only_Milk: valid poses=172/173, position std norm=104.1939 mm, frame delta mean=18.9317 mm, jitter score=123.1256 mm. Rotation column은 `pose_outputs.csv`에 없어 회전 jitter는 이 평가 출력만으로 산출 불가.

## Failure Case 분석

### SLAM_with_milk_nomilk

- False negatives: 36 / reasons={'no_ism_candidate': 36}
- False positives: 3
- False negative montage: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/ism_recall_recovery/failure_montages/SLAM_with_milk_nomilk_after_false_negative_montage.jpg`
- False positive montage: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/ism_recall_recovery/failure_montages/SLAM_with_milk_nomilk_after_false_positive_montage.jpg`

### only_Milk

- False negatives: 1 / reasons={'no_ism_candidate': 1}
- False positives: 0
- False negative montage: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/ism_recall_recovery/failure_montages/only_Milk_after_false_negative_montage.jpg`

## Threshold Sweep 결론

- 최종 적용값: `min_bbox_area=3000`, `min_mask_area=3000`, `min_final_score=0.50`.
- 이 조합은 sweep에서 SLAM visible recall >= 0.80, hidden FPR <= 0.05, only_Milk candidate recall >= 0.95를 만족했다.
- 동일 key metric에서 bbox 1000/mask 3000/score 0.50도 Pareto 후보였지만, bbox 3000은 더 보수적인 최소 box 크기를 유지하면서 같은 recall/FPR을 보였다.

## 남은 이슈

- SLAM after visible recall이 목표 0.80은 만족하지만, false negative 36건이 남아 있다. after CSV 기준 남은 원인은 모두 `no_ism_candidate`이다.
- only_Milk after final pose recall은 99.42%로 목표 0.90을 만족한다. 남은 실패 1건도 `no_ism_candidate`이며, temporal hard reject는 0건이다.
- 현재 CSV에는 rotation pose가 없어 Rotation Std/Delta는 산출하지 못한다. jitter QA를 위해 rotation quaternion 또는 rotation matrix columns를 eval output에 추가해야 한다.

## Artifacts

- Metrics JSON: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/ism_recall_recovery/ism_recall_recovery_metrics.json`
- Threshold sweep CSV: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/ism_recall_recovery/threshold_sweep_bbox_mask_score.csv`
- Pareto CSV: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/ism_recall_recovery/threshold_sweep_pareto_candidates.csv`
- Recall/FPR plot: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/ism_recall_recovery/before_after_recall_fpr.png`
- Rejection count plot: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/ism_recall_recovery/before_after_rejection_counts.png`
