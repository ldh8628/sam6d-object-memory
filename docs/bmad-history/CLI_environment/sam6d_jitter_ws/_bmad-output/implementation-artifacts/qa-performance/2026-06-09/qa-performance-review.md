# SAM6D Milk QA and Performance Review

Date: 2026-06-09

## Evaluation Setup

Input Bags:

| Bag | Purpose | Source Frames | Evaluated Frames | Label Source |
|---|---:|---:|---:|---|
| `data/ros2_bag/SLAM_with_milk_nomilk` | False Positive Pose Output | 2486 RGB / 2339 depth | 512 synchronized RGB-D frames | RGB contact sheet manual interval labels |
| `data/ros2_bag/only_Milk` | Pose Jitter | 865 RGB / 837 depth | 173 synchronized RGB-D frames, stride 5 | all frames Milk visible |

Executed modes:

| Mode | Candidate Filter | Object Gate | Pose Confidence Gate | Temporal Filter |
|---|---:|---:|---:|---:|
| Baseline | off | off | off | off |
| Improved | on | `best_ism >= 0.55` | `pose >= 0.55` | on |

Improved active thresholds:

| Parameter | Value |
|---|---:|
| `candidate_filter.min_final_score` | `0.55` |
| `candidate_filter.min_mask_area` | `9000` |
| `candidate_filter.min_bbox_area` | `9000` |
| `candidate_filter.min_depth_valid_ratio` | `0.80` |
| `temporal_filter.translation_alpha` | `0.35` |
| `temporal_filter.rotation_alpha` | `0.35` |
| `temporal_filter.max_translation_jump_mm` | `25.0` |
| `temporal_filter.max_rotation_jump_deg` | `120.0` |
| `temporal_filter.hold_last_good_frames` | `2` |

## 1. Baseline 측정

### SLAM_with_milk_nomilk

| Metric | Baseline |
|---|---:|
| Frames | 512 |
| TP | 187 |
| FP | 324 |
| TN | 1 |
| FN | 0 |
| Precision | 0.3659 |
| Recall | 1.0000 |
| False Positive Rate | 0.9969 |
| False Negative Rate | 0.0000 |
| Hidden FP | 218 |
| Hand-interaction FP | 106 |

Baseline은 Milk가 보이지 않는 구간과 손으로 이동하는 구간에서도 거의 계속 pose를 출력했다.

### only_Milk

| Metric | Baseline |
|---|---:|
| Frames | 173 |
| TP | 164 |
| FN | 9 |
| Precision | 1.0000 |
| Recall | 0.9480 |
| False Negative Rate | 0.0520 |
| ISM failures | 9 |

Temporal baseline:

| Metric | Baseline |
|---|---:|
| Pose count | 164 |
| Position Std X/Y/Z mm | `[696.25, 567.77, 833.86]` |
| Rotation Std deg | 45.0015 |
| F2F Translation Delta Mean mm | 1156.3742 |
| F2F Translation Delta Max mm | 3565.6131 |
| F2F Rotation Delta Mean deg | 104.8594 |
| F2F Rotation Delta Max deg | 179.7850 |
| Jitter Score | 1330.6016 |

## 2. 개선 버전 측정

### SLAM_with_milk_nomilk

| Metric | Improved |
|---|---:|
| Frames | 512 |
| TP | 3 |
| FP | 0 |
| TN | 325 |
| FN | 184 |
| Precision | 1.0000 |
| Recall | 0.0160 |
| False Positive Rate | 0.0000 |
| False Negative Rate | 0.9840 |
| Hidden FP | 0 |
| Hand-interaction FP | 0 |

Skip reason counts:

| Reason | Count |
|---|---:|
| `no_ism_candidate` | 506 |
| `pose_output` | 3 |
| `temporal_jump_reject` | 3 |

### only_Milk

| Metric | Improved |
|---|---:|
| Frames | 173 |
| TP | 3 |
| FN | 170 |
| Precision | 1.0000 |
| Recall | 0.0173 |
| False Negative Rate | 0.9827 |

Skip reason counts:

| Reason | Count |
|---|---:|
| `no_ism_candidate` | 86 |
| `temporal_jump_reject` | 78 |
| `pose_confidence_score_below_min` | 6 |
| `pose_output` | 3 |

Temporal improved:

| Metric | Improved |
|---|---:|
| Pose count | 3 |
| Position Std X/Y/Z mm | `[0.00, 0.00, 0.00]` |
| Rotation Std deg | 0.0000 |
| F2F Translation Delta Mean mm | 0.0000 |
| F2F Rotation Delta Mean deg | 0.0000 |
| Jitter Score | 0.0000 |

This temporal result is not a valid jitter improvement result because only 3/173 visible frames produced a pose.

## 3. 수치 비교

### False Positive Target Bag

| Metric | Baseline | Improved | Delta |
|---|---:|---:|---:|
| FP | 324 | 0 | -324 |
| FPR | 0.9969 | 0.0000 | -0.9969 |
| Hidden FP | 218 | 0 | -218 |
| Hand-interaction FP | 106 | 0 | -106 |
| Recall | 1.0000 | 0.0160 | -0.9840 |
| FN | 0 | 184 | +184 |

Result:
- Milk hidden/hand 구간의 pose 출력 금지는 달성했다.
- Milk 재등장 시 정상 복구는 실패했다. Visible positive frame 187개 중 3개만 pose를 출력했다.

### Jitter Target Bag

| Metric | Baseline | Improved | Delta |
|---|---:|---:|---:|
| Pose output count | 164 | 3 | -161 |
| Recall | 0.9480 | 0.0173 | -0.9306 |
| Position Std norm mm | 1225.7423 | 0.0000 | not comparable |
| F2F Translation Delta Mean mm | 1156.3742 | 0.0000 | not comparable |
| F2F Rotation Delta Mean deg | 104.8594 | 0.0000 | not comparable |
| Jitter Score | 1330.6016 | 0.0000 | not comparable |

Result:
- Translation/rotation jitter 감소 성공으로 판정할 수 없다.
- Improved mode가 pose를 대부분 억제했기 때문에 temporal metric이 작아진 것이다.

## 4. Failure Case 분석

### Failure 1: Baseline False Positive

`SLAM_with_milk_nomilk` baseline은 hidden/hand 구간에서 324개 FP를 출력했다.

Representative frames:
- Hidden FP: `000348`, `000352`, `000357`, `000362`
- Baseline top ISM score range in these examples: about `0.289-0.320`

Root cause from measured outputs:
- Baseline has no object existence gate.
- Baseline has no publish-time pose confidence gate.
- Baseline publishes pose for weak ISM candidates.

### Failure 2: Improved Recall Collapse

Improved mode suppresses false positives but also suppresses real Milk detections.

Measured evidence:

| Bag | Visible Frames | Improved Pose Outputs | Recall |
|---|---:|---:|---:|
| `SLAM_with_milk_nomilk` | 187 | 3 | 0.0160 |
| `only_Milk` | 173 | 3 | 0.0173 |

Root cause from measured outputs:
- `no_ism_candidate` dominates: 506/512 on SLAM and 86/173 on only_Milk.
- `temporal_jump_reject` blocks 78/173 only_Milk frames.
- Current thresholds are too strict for these Bag conditions.

### Failure 3: Baseline Runtime Instability

`only_Milk` baseline produced 9 `ism_failed` frames.

Observed error:

```text
TensorAdvancedIndexing.cpp:3045 INTERNAL ASSERT FAILED
```

Affected sampled frames include:
- `000370`
- `000375`
- `000380`
- `000385`
- `000500`
- `000505`
- `000510`
- `000515`
- `000520`

This is a runtime stability issue in the ISM scoring path and must be handled separately from pose filtering.

## 5. 시각화

Generated graphs:

| Visualization | File |
|---|---|
| SLAM pose output timeline | `SLAM_with_milk_nomilk_pose_timeline.png` |
| SLAM detection metric bar chart | `SLAM_with_milk_nomilk_detection_metrics.png` |
| SLAM translation/score graph | `SLAM_with_milk_nomilk_translation_score.png` |
| only_Milk pose output timeline | `only_Milk_pose_timeline.png` |
| only_Milk detection metric bar chart | `only_Milk_detection_metrics.png` |
| only_Milk translation/score graph | `only_Milk_translation_score.png` |
| Representative failure montage | `failure_cases_montage.jpg` |

Output directory:

```text
_bmad-output/implementation-artifacts/qa-performance/2026-06-09
```

## 6. QA Verdict

| Requirement | Result | Evidence |
|---|---|---|
| Milk hidden 중 pose 출력 금지 | PASS | FPR `0.9969 -> 0.0000` |
| Hand 이동 중 pose 출력 금지 | PASS | hand FP `106 -> 0` |
| Milk 재등장 시 pose 정상 복구 | FAIL | recall `1.0000 -> 0.0160` on SLAM |
| only_Milk translation std 감소 | FAIL / not comparable | pose output count `164 -> 3` |
| only_Milk rotation std 감소 | FAIL / not comparable | pose output count `164 -> 3` |
| frame-to-frame delta 감소 | FAIL / not comparable | pose output count `164 -> 3` |
| jitter score 감소 | FAIL / not comparable | suppression, not stabilization |

## 7. Required Fixes Before Production

1. Retune candidate/object/pose thresholds with positive recall as a hard gate.
   - Current `min_final_score=0.55` is too strict for these Bags.
   - Current config reduces false positives by over-suppressing all detections.

2. Change temporal stabilization to smooth valid poses without rejecting sustained motion.
   - `temporal_jump_reject=78` on only_Milk indicates the jump gate is acting as a hard recall killer.
   - For camera motion, temporal filter should not drop most frames.

3. Add calibration sweep report.
   - Sweep `min_final_score`, `min_bbox_area`, `min_depth_valid_ratio`, `pose_confidence.min_score`.
   - Select thresholds only if both FP suppression and visible recall pass.

4. Handle ISM internal assert.
   - Add a defensive fallback around empty/invalid proposal tensors before DINO/score indexing.
   - Record failed frame and continue without crashing the node.

## 8. Reproduction Commands

```bash
/home/etri/miniconda3/envs/sam6d_test/bin/python tools/extract_rosbag_rgbd.py --bag data/ros2_bag/SLAM_with_milk_nomilk --output-dir output/qa_performance_20260609/SLAM_with_milk_nomilk/extracted --frame-list output/qa_performance_20260609/SLAM_with_milk_nomilk/frame_list.txt --max-sync-slop-ms 80

/home/etri/miniconda3/envs/sam6d_test/bin/python tools/extract_rosbag_rgbd.py --bag data/ros2_bag/only_Milk --output-dir output/qa_performance_20260609/only_Milk/extracted --stride 5 --max-sync-slop-ms 80

/home/etri/miniconda3/envs/sam6d_test/bin/python tools/run_sam6d_reliability_eval.py --workspace . --rgb-dir output/qa_performance_20260609/SLAM_with_milk_nomilk/extracted/rgb --depth-dir output/qa_performance_20260609/SLAM_with_milk_nomilk/extracted/depth --output-dir output/qa_performance_20260609/SLAM_with_milk_nomilk/baseline --mode baseline --no-vis

/home/etri/miniconda3/envs/sam6d_test/bin/python tools/run_sam6d_reliability_eval.py --workspace . --rgb-dir output/qa_performance_20260609/SLAM_with_milk_nomilk/extracted/rgb --depth-dir output/qa_performance_20260609/SLAM_with_milk_nomilk/extracted/depth --output-dir output/qa_performance_20260609/SLAM_with_milk_nomilk/improved --mode improved --no-vis

/home/etri/miniconda3/envs/sam6d_test/bin/python tools/run_sam6d_reliability_eval.py --workspace . --rgb-dir output/qa_performance_20260609/only_Milk/extracted/rgb --depth-dir output/qa_performance_20260609/only_Milk/extracted/depth --output-dir output/qa_performance_20260609/only_Milk/baseline --mode baseline --no-vis

/home/etri/miniconda3/envs/sam6d_test/bin/python tools/run_sam6d_reliability_eval.py --workspace . --rgb-dir output/qa_performance_20260609/only_Milk/extracted/rgb --depth-dir output/qa_performance_20260609/only_Milk/extracted/depth --output-dir output/qa_performance_20260609/only_Milk/improved --mode improved --no-vis
```
