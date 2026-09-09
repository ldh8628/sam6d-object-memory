# Consistency Gate Report

Date: 2026-06-09

## Root Cause Summary

- Hidden FP 3프레임은 candidate score, bbox/mask area, depth valid ratio, pose confidence를 모두 통과했다.
- PEM/ISM 산출물에는 correspondence count, inlier count, reprojection error가 저장되지 않는다. 현재 코드 기반으로 직접 검증 가능한 신호는 mask depth 통계, projected model bbox/mask approximation, projected point depth residual, score margin이다.
- 전수 rule impact 분석 결과, hidden FP를 1개라도 제거하는 consistency hard reject는 SLAM visible TP를 최소 5개 잃어 visible recall 80% 조건을 깨뜨린다.
- Rotation spike는 quaternion sign flip 계산 버그가 아니다. delta 계산은 `abs(dot(q0,q1))`로 sign flip을 보정했고, 실제 smoothed orientation 변화가 50도 이상 발생했다.

## Hidden FP Feature Evidence

| Frame | Score | Pose Conf | Depth Err mm | Render IoU approx | Proj Depth Residual mm | Score Margin |
|---|---:|---:|---:|---:|---:|---:|
| 1245 | 0.5192 | 0.5192 | 41.000 | 0.8036 | 11.504 | 0.0493 |
| 1720 | 0.5628 | 0.5628 | 42.000 | 0.7868 | 11.089 | 0.0230 |
| 1808 | 0.5827 | 0.5776 | 16.466 | 0.0719 | 30.052 | 0.0162 |

## Consistency Rule Impact

Rules that remove FP also damage recall:

| Rule | FP Removed | SLAM TP Lost | SLAM Recall After | only_Milk Output After |
|---|---:|---:|---:|---:|
| render_iou>0.8 | 1 | 5 | 78.07% | 98.27% |
| render_iou>0.75 | 2 | 9 | 75.94% | 83.24% |
| render_iou>0.7 | 2 | 14 | 73.26% | 72.83% |
| score_margin>0.02 AND depth_error>35 | 2 | 16 | 72.19% | 92.49% |
| (depth_error>40 AND render_iou>0.75) OR (render_iou<0.10 AND projected_depth_residual>25) | 3 | 16 | 72.19% | 93.64% |
| render_iou<0.1 | 1 | 20 | 70.05% | 93.06% |
| score_margin>0.04 | 1 | 21 | 69.52% | 77.46% |
| (depth_error>40 AND render_iou>0.75) OR render_iou<0.10 | 3 | 23 | 68.45% | 93.06% |

결론: hidden FP 제거용 hard reject/penalty는 이번 반복에서 적용하지 않았다. 현재 데이터에서는 threshold 상향보다 나은 consistency gate가 발견되지 않았다.

## Implemented Minimal Change

- `temporal_filter.max_rotation_step_deg=30.0` 추가
- `_stabilize_pose`: smoothed rotation delta가 max step을 넘으면 previous pose에서 max step만큼만 추가 보간
- 평가 CSV: `temporal_rotation_limited`, `temporal_rotation_prelimit_delta_deg` 추가
- Hidden FP consistency gate는 적용하지 않음. 전수 영향 분석에서 FP 제거 rule이 visible recall 또는 only_Milk output target을 위반했다.
- Updated config summary: bbox/mask/score threshold는 recall recovery 값 유지, temporal hard reject 비활성 유지, rotation step cap만 추가.

## Before / After Metrics

### SLAM_with_milk_nomilk

| Variant | Visible Recall | Hidden FPR | Hand FPR | Output Count | Hidden FP Frames | Re-visible Recovery | Rotation Max deg |
|---|---:|---:|---:|---:|---|---:|---:|
| before | 80.75% | 1.38% | 0.00% | 154 | 001245, 001720, 001808 | 100.00% | N/A |
| after | 80.21% | 1.38% | 0.00% | 153 | 001245, 001720, 001808 | 100.00% | 38.161 |

### only_Milk

| Variant | Output Rate | Missing | T F2F Mean mm | T F2F Max mm | R F2F Mean deg | R F2F Max deg | Rotation Limited Count |
|---|---:|---:|---:|---:|---:|---:|---:|
| before | 99.42% | 1 | 20.275 | 68.428 | 8.591 | 56.556 | 0 |
| after | 98.84% | 2 | 20.517 | 62.778 | 5.414 | 30.080 | 13 |

## Final Verdict

1. hidden FP 3프레임이 제거되었는가? No. Before 3, after 3. 제거 가능한 consistency rule은 recall 목표를 위반했다.
2. visible recall 80% 이상이 유지되었는가? Yes. After SLAM visible recall 80.21%.
3. only_Milk output rate 95% 이상이 유지되었는가? Yes. After output rate 98.84%.
4. rotation max jitter가 감소했는가? Yes. only_Milk rotation max 56.556 -> 30.080 deg.
5. 추가된 consistency score가 threshold 상향보다 나은가? No actionable consistency score was safe. Threshold 상향도 recall을 깨고, consistency hard reject도 TP 손실이 커서 적용하지 않았다.
6. production 적용 가능한가? CONCERNS. Recall/continuity/rotation jitter는 개선됐지만 hidden FP 3건이 남아 object absence 구간에서 잘못된 pose output 가능성이 있다.

Residual target gap:

- only_Milk missing frame target `<=1`은 after run에서 `2/173`으로 한 프레임 초과했다. 추가 missing frame `000190`은 `pose_confidence_score_below_min`이며 rotation cap 적용 이후 reject가 아니라 PEM confidence 재실행 변동이다.

## Artifacts

- feature_samples: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/consistency_feature_samples.csv`
- feature_all_pose_outputs: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/consistency_features_all_pose_outputs.csv`
- feature_separation_scores: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/feature_separation_scores.csv`
- rule_impact_sweep: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/consistency_rule_impact_sweep.csv`
- before_after_csv: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/before_after_comparison.csv`
- before_after_plot: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/before_after_consistency_comparison.png`
- hidden_fp_before_montage: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/hidden_fp_before_montage.jpg`
- hidden_fp_after_montage: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/hidden_fp_after_montage.jpg`
- rotation_spike_before_montage: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/rotation_spike_before_montage.jpg`
- rotation_spike_after_montage: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/rotation_spike_after_montage.jpg`
- metrics_json: `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/consistency_gate_analysis/consistency_metrics.json`
