# SAM6D Milk Recall Kill Analysis

Date: 2026-06-09

Scope:
- No runtime/pipeline implementation changes.
- Analysis uses existing QA outputs under `output/qa_performance_20260609`.
- Primary artifacts are under `_bmad-output/implementation-artifacts/qa-performance/2026-06-09/recall_kill_analysis`.

## Executive Finding

The recall collapse is not primarily caused by `min_final_score`.

The dominant recall killer is the candidate area gate:
- `candidate_filter.min_mask_area=9000`
- `candidate_filter.min_bbox_area=9000`

Measured evidence:

| Bag | Visible Frames | Rank-0 Area Rejects | Rank-0 Score Rejects | Current Recall |
|---|---:|---:|---:|---:|
| `SLAM_with_milk_nomilk` | 187 | 165 | 53 | 0.0160 |
| `only_Milk` | 173 | 139 | 2 | 0.0173 |

For visible Milk frames, rank-0 ISM scores are usually high enough:

| Bag | Rank-0 ISM Score Min | Median | Max | Rank-0 BBox Area Median | Rank-0 Mask Area Median |
|---|---:|---:|---:|---:|---:|
| `SLAM_with_milk_nomilk` | 0.4140 | 0.5770 | 0.6315 | 6678 | 4736 |
| `only_Milk` | 0.5354 | 0.6203 | 0.6616 | 7436 | 6873 |

The configured 9000 area threshold is above the median real Milk candidate size in both Bags.

## Phase 1: Recall Kill Analysis

### Stage Recall Drop Table

| Bag | Visible Frames | Stage | Removed | Stage Drop | Cumulative Drop | Remaining |
|---|---:|---|---:|---:|---:|---:|
| `SLAM_with_milk_nomilk` | 187 | `no_ism_candidate` | 181 | 96.79% | 96.79% | 6 |
| `SLAM_with_milk_nomilk` | 187 | `pose_confidence_score_below_min` | 0 | 0.00% | 96.79% | 6 |
| `SLAM_with_milk_nomilk` | 187 | `temporal_jump_reject` | 3 | 1.60% | 98.40% | 3 |
| `SLAM_with_milk_nomilk` | 187 | survived pose output | 3 | - | - | 3 |
| `only_Milk` | 173 | `no_ism_candidate` | 86 | 49.71% | 49.71% | 87 |
| `only_Milk` | 173 | `pose_confidence_score_below_min` | 6 | 3.47% | 53.18% | 81 |
| `only_Milk` | 173 | `temporal_jump_reject` | 78 | 45.09% | 98.27% | 3 |
| `only_Milk` | 173 | survived pose output | 3 | - | - | 3 |

### Top Recall Killer Ranking

| Rank | Killer | Evidence | Impact |
|---:|---|---|---|
| 1 | Candidate area gate | SLAM rank-0 area rejects 165/187; only_Milk 139/173 | Primary recall collapse source |
| 2 | Temporal jump gate | only_Milk visible frames: 78 rejected after PEM | Secondary collapse source on moving camera Bag |
| 3 | Pose confidence gate | only_Milk: 6 rejected; SLAM: 0 rejected | Minor |
| 4 | `min_final_score=0.55` | SLAM rank-0 score rejects 53/187; only_Milk 2/173 | Secondary on SLAM, negligible on only_Milk |

Important terminology correction:
- `no_ism_candidate` does not mean FastSAM/ISM had no raw candidates.
- It means the candidate filter accepted zero candidates after applying score/area/depth filters.
- Example `SLAM_with_milk_nomilk/000000`: raw candidate count 48, accepted 0. Rank-0 score 0.5997, rejected by `mask_area_below_min;bbox_area_below_min`.

## Phase 2: Threshold Sweep

### Current Area Gate Fixed

When keeping `min_mask_area=9000` and `min_bbox_area=9000`, reducing `min_final_score` does not recover recall.

| Bag | Sweep | Best Recall with Current Area Gate | FPR |
|---|---|---:|---:|
| `SLAM_with_milk_nomilk` | `min_final_score` 0.0-0.55 | 0.0321 | 0.0000 |
| `only_Milk` | `min_final_score` 0.0-0.55 | 0.4682 | N/A |

Interpretation:
- Score threshold alone cannot recover recall.
- Candidate area threshold blocks the visible object before pose estimation.

### Pose Confidence Sweep

With current area gate fixed:

| Bag | Pose Threshold Range | Best Recall | Comment |
|---|---:|---:|---|
| `SLAM_with_milk_nomilk` | 0.0-0.55 | 0.0321 | Candidate filter already killed most visible frames |
| `only_Milk` | 0.0-0.3 | 0.5029 | Limited by candidate area gate |
| `only_Milk` | 0.55 | 0.4682 | 6 additional visible frames lost |

Interpretation:
- Pose confidence threshold is not the main SLAM reappearance failure.
- It should still be lowered or made advisory until candidate gating is calibrated.

### Temporal Jump Sweep

Using raw PEM outputs from improved run:

| Bag | Pose Threshold | Translation Jump Threshold | Recall | FPR |
|---|---:|---:|---:|---:|
| `SLAM_with_milk_nomilk` | 0.55 | 25 | 0.0160 | 0.0000 |
| `SLAM_with_milk_nomilk` | 0.55 | 120 | 0.0160 | 0.0000 |
| `SLAM_with_milk_nomilk` | 0.55 | 1000+ | 0.0321 | 0.0000 |
| `only_Milk` | 0.55 | 25 | 0.0173 | N/A |
| `only_Milk` | 0.55 | 60 | 0.0983 | N/A |
| `only_Milk` | 0.55 | 120 | 0.1387 | N/A |
| `only_Milk` | 0.55 | 250 | 0.4682 | N/A |
| `only_Milk` | 0.55 | 1000+ | 0.4682 | N/A |

Interpretation:
- `max_translation_jump_mm=25` is too strict for camera motion.
- On `only_Milk`, increasing translation jump to 250 mm recovers all frames that survived candidate+pose confidence gates.
- On `SLAM_with_milk_nomilk`, temporal gate is not the primary problem because candidate filtering already removes almost all visible frames.

### Candidate Filter Pareto Analysis

Candidate-level ablation over area and score shows the practical tradeoff:

| Bag | Min Area | Min Final Score | Recall | FPR | Precision | TP | FP | FN |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `SLAM_with_milk_nomilk` | 3000 | 0.50 | 0.8075 | 0.0092 | 0.9805 | 151 | 3 | 36 |
| `SLAM_with_milk_nomilk` | 0 | 0.50 | 0.8075 | 0.0246 | 0.9497 | 151 | 8 | 36 |
| `SLAM_with_milk_nomilk` | 5000 | 0.40 | 0.9251 | 0.6123 | 0.4651 | 173 | 199 | 14 |
| `SLAM_with_milk_nomilk` | 3000 | 0.40 | 0.9840 | 0.7754 | 0.4220 | 184 | 252 | 3 |
| `only_Milk` | 3000 | 0.50 | 0.9942 | N/A | 1.0000 | 172 | 0 | 1 |
| `only_Milk` | 5000 | 0.50 | 0.9884 | N/A | 1.0000 | 171 | 0 | 2 |
| `only_Milk` | 9000 | 0.55 | 0.5029 candidate-level before temporal | N/A | 1.0000 | 87 | 0 | 86 |

Best current candidate-level Pareto point:
- `min_area=3000`
- `min_final_score=0.50`

Why:
- SLAM FPR stays below 1%.
- SLAM recall recovers from 1.6% to 80.7% at candidate level.
- only_Milk recall stays above 99% at candidate level.

## Phase 3: Reappearance Failure Analysis

### Hidden -> Visible Transition 1: frame `000000`

First visible sequence after start:

| Frame | Rank-0 Score | BBox Area | Mask Area | Accepted | Reject Reason |
|---|---:|---:|---:|---:|---|
| `000000` | 0.5997 | 6050 | 4423 | 0 | area below min |
| `000013` | 0.5774 | 6625 | 4539 | 0 | area below min |
| `000017` | 0.5948 | 6678 | 4557 | 0 | area below min |
| `000022` | 0.6019 | 6858 | 4768 | 0 | area below min |

The Milk candidate score is valid, but the object is too small for the fixed 9000 area threshold.

### Hidden -> Visible Transition 2: frame `001251`

| Frame | Rank-0 Score | BBox Area | Mask Area | Accepted | Reject Reason |
|---|---:|---:|---:|---:|---|
| `001251` | 0.5351 | 4095 | 3342 | 0 | score + area below min |
| `001260` | 0.5851 | 5808 | 4710 | 0 | area below min |
| `001276` | 0.6304 | 5734 | 4286 | 0 | area below min |

Again, score is often sufficient after reappearance; area gate prevents PEM from running.

### Hidden -> Visible Transition 3: frame `001724`

| Frame | Rank-0 Score | BBox Area | Mask Area | Accepted | Reject Reason |
|---|---:|---:|---:|---:|---|
| `001724` | 0.5530 | 9880 | 7378 | 0 | mask area below min |
| `001734` | 0.5735 | 7503 | 4667 | 0 | area below min |
| `001767` | 0.6136 | 4284 | 3214 | 0 | area below min |

This transition proves bbox area alone is insufficient; `mask_area=9000` is also too strict.

### Hidden -> Visible Transition 4: frame `002185`

Later reappearance is harder:
- Rank-0 scores often around 0.50-0.56.
- Area is sometimes valid for bbox but mask remains below 9000.
- Both score and mask threshold contribute.

## Root Cause

### Root Cause 1: Absolute Area Threshold Is Not Scale-Aware

The current filter uses fixed pixel area thresholds:

```yaml
min_mask_area: 9000
min_bbox_area: 9000
```

But real visible Milk candidates in these Bags commonly have:
- SLAM median bbox area: 6678
- SLAM median mask area: 4736
- only_Milk median bbox area: 7436
- only_Milk median mask area: 6873

Therefore the filter rejects normal true positives.

### Root Cause 2: Temporal Jump Gate Acts As Hard Rejection

Current temporal setting:

```yaml
max_translation_jump_mm: 25.0
max_rotation_jump_deg: 120.0
hold_last_good_frames: 2
```

On `only_Milk`, camera motion causes pose jumps above 25 mm. The temporal filter rejects 78 visible frames.

### Root Cause 3: Pose Confidence Gate Is Secondary

Only 6/173 `only_Milk` visible frames are killed by `pose_confidence_score_below_min`; SLAM has 0 at this stage. This is not the primary recall issue.

## Recommended Minimal Fix Plan

No implementation has been applied yet.

### Fix A: Replace Current Area Thresholds With Lower First-Pass Values

Recommended first calibration:

```yaml
candidate_filter:
  min_final_score: 0.50
  min_mask_area: 3000
  min_bbox_area: 3000
  min_depth_valid_ratio: 0.80
```

Expected from candidate-level sweep:
- SLAM recall: 0.8075
- SLAM FPR: 0.0092
- only_Milk candidate recall: 0.9942

This is the smallest change that recovers recall while keeping FPR low in the mixed Bag.

### Fix B: Relax Temporal Jump Rejection Or Convert It To Smoothing-Only

Recommended first calibration:

```yaml
temporal_filter:
  max_translation_jump_mm: 250.0
  max_rotation_jump_deg: 180.0
  hold_last_good_frames: 0
```

Or safer:
- Do not suppress pose output in temporal filter.
- Emit smoothed pose and diagnostic `temporal_jump=true`.
- Use temporal rejection only after object existence has been false for consecutive frames.

Expected from current raw PEM temporal sweep on only_Milk:
- recall at 25 mm: 0.0173
- recall at 250 mm: 0.4682, bounded by current candidate+pose gates

After Fix A, this needs a full rerun.

### Fix C: Keep Pose Confidence Conservative But Not Primary

Recommended:

```yaml
pose_confidence:
  min_score: 0.50
```

Rationale:
- Pose confidence is not the top killer.
- Lowering it alone cannot recover SLAM recall because candidate filter blocks PEM first.

## Do Not Implement Yet: Validation Gate For Next Step

Before changing code/config, run a full rerun plan with these candidate configs:

| Config | Min Area | Min Final | Temporal Jump | Purpose |
|---|---:|---:|---:|---|
| C1 | 3000 | 0.50 | 250 mm | primary recommendation |
| C2 | 3000 | 0.55 | 250 mm | stricter score |
| C3 | 5000 | 0.50 | 250 mm | stricter area |
| C4 | 3000 | 0.50 | smoothing-only | test temporal no-drop |

Acceptance target for the next run:
- `SLAM_with_milk_nomilk` FPR <= 0.05
- `SLAM_with_milk_nomilk` recall after reappearance >= 0.80
- `only_Milk` recall >= 0.90
- Jitter metrics computed only if pose output count remains >= 90% of visible frames

## Artifacts

CSV:
- `recall_kill_analysis/stage_recall_drop_table.csv`
- `recall_kill_analysis/threshold_sweep.csv`
- `recall_kill_analysis/pose_temporal_sweep_from_raw_pem.csv`
- `recall_kill_analysis/candidate_filter_ablation_area_score.csv`
- `recall_kill_analysis/reappearance_transition_analysis.csv`

Plots:
- `recall_kill_analysis/SLAM_with_milk_nomilk_stage_recall_kill.png`
- `recall_kill_analysis/only_Milk_stage_recall_kill.png`
- `recall_kill_analysis/SLAM_with_milk_nomilk_candidate_pareto.png`
- `recall_kill_analysis/only_Milk_candidate_pareto.png`
- `recall_kill_analysis/SLAM_with_milk_nomilk_temporal_jump_sweep.png`
- `recall_kill_analysis/only_Milk_temporal_jump_sweep.png`
