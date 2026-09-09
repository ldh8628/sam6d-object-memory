---
title: SAM6D Milk Pose Reliability
status: draft
created: 2026-06-09
updated: 2026-06-09
---

# PRD: SAM6D Milk Pose Reliability

## 1. Current State

The current project is a ROS2 RGB-D pipeline that detects object candidates from bag input and estimates the 6D pose of a Milk object using SAM-6D. The pipeline loads a Milk PLY/CAD model, uses rendered templates to extract features, generates object candidates through the Instance Segmentation Model (ISM), treats those candidates as possible Milk instances, and runs the Pose Estimation Model (PEM) to produce pose outputs.

Current behavior is detection-first and permissive. ISM generates proposals from FastSAM/SAM, scores them with semantic, appearance, and geometric matching, then forwards the top candidates to PEM. The Milk runtime configuration currently uses `det_score_thresh=0.2`, `top_k_for_pem=5`, and `max_proposals=50`. In observed outputs, ISM scores were approximately `0.411-0.639`, so the current PEM threshold does not meaningfully block candidates.

The current system has no explicit object-existence decision, no calibrated negative-image rejection threshold, no pose publish gate, no robust geometric verification after pose estimation, and no temporal pose stabilization. PEM also samples observed mask points with random sampling, so identical or near-identical frames can produce different model inputs.

## 2. Problem Statement

The pipeline currently produces two reliability failures:

1. When the Milk object is not present in the input image, ISM can still generate visually plausible object candidates. Those candidates may pass the low score threshold and trigger PEM, causing false positive pose outputs.
2. When the camera and object are stationary, frame-to-frame pose outputs can jitter because segmentation masks, candidate boxes, sampled point clouds, feature matching, depth noise, and pose estimation outputs vary between frames.

The product requirement is to convert the current research-style SAM-6D pipeline into a reliability-gated pose pipeline: pose estimation and publication must occur only when Milk existence is sufficiently verified, and accepted poses must remain stable in static scenes.

## 3. Functional Requirements

### Feature A: Object Existence Verification

**목적:** Milk 객체가 입력 이미지에 실제로 존재하는지 PEM 실행 전에 판단한다. Milk가 없으면 pose estimation을 수행하지 않거나 결과 발행을 억제한다.

**입력:**
- RGB image
- Depth image
- Camera intrinsics
- ISM proposal list
- Rendered Milk template descriptors
- Optional prior frame state

**출력:**
- `object_present: bool`
- `existence_score: float`
- `rejection_reason: string | null`
- Candidate list allowed for downstream filtering

**구현 전략:**
- FR-1: System shall compute a calibrated Milk existence decision before PEM runs.
- FR-2: System shall reject frames where no candidate exceeds `min_existence_score`.
- FR-3: System shall support a score margin rule: top candidate must exceed second-best candidate by `min_score_margin` when multiple proposals are available.
- FR-4: System shall record existence decision metadata for every processed frame.
- FR-5: System shall make thresholds configurable through ROS parameters.

**성공 기준:**
- Milk 미존재 validation set에서 False Positive Detection Rate <= 5%.
- Milk 미존재 validation set에서 Pose 출력 억제 성공률 >= 95%.
- Milk 존재 validation set에서 valid pose recall degradation <= 10% relative to current baseline. [ASSUMPTION: acceptable recall loss should be confirmed by project owner.]

### Feature B: Candidate Confidence Filtering

**목적:** ISM 후보 중 Milk로 보기 어려운 후보를 PEM 전에 제거한다.

**입력:**
- Raw FastSAM/SAM proposals
- ISM final score
- ISM component scores: semantic, appearance, geometric, visible ratio
- Candidate bbox, mask area, depth-valid pixels
- Runtime thresholds

**출력:**
- Filtered candidate list
- Candidate-level confidence report
- Candidate rejection log

**구현 전략:**
- FR-6: System shall preserve ISM component scores in memory and optionally in `detection_ism_debug.json`.
- FR-7: System shall apply minimum thresholds for final score, semantic score, appearance score, geometric score, and visible ratio.
- FR-8: System shall filter candidates by mask area, bbox aspect ratio, bbox size, valid depth ratio, and expected Milk physical size range.
- FR-9: System shall allow `top_k_for_pem` to be set independently from diagnostic candidate count.
- FR-10: System shall support threshold sweep runs for offline evaluation.

**성공 기준:**
- Candidate filtering removes >= 90% of negative-frame candidates that would previously reach PEM. [ASSUMPTION: measured on the validation dataset defined in Section 9.]
- Candidate filtering does not remove the best true Milk candidate in >= 90% of positive validation frames.
- Filtering outputs enough debug metadata to explain every rejection.

### Feature C: Geometric Verification

**목적:** Candidate와 estimated pose가 RGB-D geometry와 일치하는지 검증해 false positive pose를 제거한다.

**입력:**
- Candidate mask and bbox
- Depth image
- Camera intrinsics
- CAD/model points
- Estimated pose candidates
- Rendered/projected model silhouette or point projection

**출력:**
- `geometry_verified: bool`
- Silhouette IoU
- Projected bbox IoU
- Depth residual statistics
- Reprojection residual statistics
- Geometric rejection reason

**구현 전략:**
- FR-11: System shall project the estimated Milk model into the image and compare the projected silhouette/bbox against the observed mask/bbox.
- FR-12: System shall compute depth consistency between visible model surface and observed depth inside the candidate region.
- FR-13: System shall reject pose candidates whose projected bbox IoU, mask IoU, or depth residual falls outside configured limits.
- FR-14: System shall support a future PnP-RANSAC verification path using 2D-3D correspondences and inlier count when correspondence data is available.
- FR-15: System shall expose geometric verification metrics in debug outputs.

**성공 기준:**
- At least 95% of published poses pass all geometric verification gates.
- Negative frames with visually similar non-Milk objects are rejected when geometric consistency is poor.
- Geometric rejection can be reproduced offline from saved frame artifacts.

### Feature D: Pose Confidence Scoring

**목적:** PEM 결과를 그대로 발행하지 않고, multi-factor confidence score로 pose publish 여부를 결정한다.

**입력:**
- ISM score and component scores
- PEM pose score
- Geometric verification metrics
- Temporal consistency metrics
- Candidate rank and score margin

**출력:**
- `pose_confidence: float`
- `publish_pose: bool`
- Final selected pose or no-pose decision
- Confidence breakdown

**구현 전략:**
- FR-16: System shall compute a final pose confidence score from ISM confidence, PEM confidence, geometric verification, and temporal consistency.
- FR-17: System shall publish poses only when `pose_confidence >= min_pose_confidence`.
- FR-18: System shall reject sudden jumps from the last accepted pose unless confidence and geometric consistency are high enough.
- FR-19: System shall distinguish between `NO_OBJECT`, `LOW_CONFIDENCE`, `GEOMETRY_FAILED`, `TEMPORAL_REJECTED`, and `POSE_ACCEPTED` states.
- FR-20: System shall publish diagnostic state without publishing an invalid pose.

**성공 기준:**
- Wrong pose outputs in Milk-absent frames are eliminated or reduced below target.
- Accepted poses include confidence metadata suitable for logging and visualization.
- Pose output suppression does not crash downstream ROS consumers.

### Feature E: Pose Temporal Stabilization

**목적:** 정지 장면에서 pose output jitter를 줄이고, 움직임이 있는 경우에는 과도한 lag 없이 추적한다.

**입력:**
- Accepted pose stream
- Pose confidence
- Frame timestamp
- Previous accepted pose state
- Optional candidate bbox/mask history

**출력:**
- Stabilized pose
- Temporal state
- Jitter metrics
- Hold/reject reason when output is suppressed

**구현 전략:**
- FR-21: System shall apply temporal filtering only after pose confidence and geometric gates pass.
- FR-22: System shall support translation smoothing using EMA or One Euro filtering.
- FR-23: System shall support rotation smoothing using quaternion slerp or equivalent SO(3)-safe filtering.
- FR-24: System shall implement hold-last-good-pose for short low-confidence gaps.
- FR-25: System shall implement jump rejection based on configurable translation and rotation deltas.
- FR-26: System shall use deterministic point sampling or deterministic seeding for PEM input generation.

**성공 기준:**
- Static-scene translation variance decreases relative to baseline.
- Static-scene rotation variance decreases relative to baseline.
- Filtered pose remains responsive enough for the configured ROS bag frame rate. [ASSUMPTION: target latency budget is <= 1 frame unless otherwise specified.]

### Feature F: Debug Visualization

**목적:** FP 제거와 pose 안정화 판단 근거를 사람이 프레임 단위로 확인할 수 있게 한다.

**입력:**
- RGB image
- Candidate masks/bboxes
- Candidate score breakdown
- Pose projection
- Verification metrics
- Temporal state

**출력:**
- Debug overlay image
- Per-frame JSON debug report
- Optional ROS debug image topic
- Optional summary CSV

**구현 전략:**
- FR-27: System shall visualize raw proposals, filtered candidates, selected candidate, rejected candidates, and final projected pose.
- FR-28: System shall overlay score breakdown and rejection reason per candidate.
- FR-29: System shall save debug artifacts for negative frames, rejected frames, and accepted pose frames.
- FR-30: System shall support `debug_visualization.enabled` and output path ROS parameters.
- FR-31: System shall not require debug visualization to be enabled for production inference.

**성공 기준:**
- Engineers can determine why a frame did or did not publish a pose using saved artifacts alone.
- Debug mode does not alter pose decision results.
- Debug mode performance overhead is measurable and can be disabled.

### Feature G: Evaluation Pipeline

**목적:** FP rate, pose suppression, jitter, and regression impact를 반복 측정할 수 있는 offline evaluation pipeline을 제공한다.

**입력:**
- Labeled ROS2 bag or extracted RGB-D frames
- Frame-level labels: Milk present/absent
- Optional pose ground truth or static-scene reference pose
- Pipeline output JSON/debug artifacts
- Threshold sweep configuration

**출력:**
- Evaluation report
- Metrics JSON/CSV
- Threshold sweep summary
- Failure-case artifact index

**구현 전략:**
- FR-32: System shall run inference over validation datasets and collect detection, suppression, confidence, and pose metrics.
- FR-33: System shall compute FP rate on Milk-absent frames.
- FR-34: System shall compute pose output suppression success rate on Milk-absent frames.
- FR-35: System shall compute translation and rotation jitter on static positive scenes.
- FR-36: System shall compare current run against baseline outputs.
- FR-37: System shall produce a sorted failure list for manual inspection.

**성공 기준:**
- Evaluation can be run from a single command or launch target.
- Report includes all metrics in Section 8.
- Threshold changes can be compared reproducibly across runs.

## 4. Non Functional Requirements

- NFR-1 Reliability: The system must prefer no-pose output over low-confidence wrong pose output.
- NFR-2 Determinism: Offline evaluation with the same inputs and configuration must produce repeatable decisions and comparable metrics.
- NFR-3 Observability: Every pose suppression must have a machine-readable reason.
- NFR-4 Configurability: All gating thresholds must be ROS parameters or evaluation config values.
- NFR-5 Runtime Safety: If verification fails internally, the system must not publish a pose for that frame.
- NFR-6 Compatibility: Existing ROS pose topics should remain compatible unless explicitly versioned.
- NFR-7 Performance: Verification and smoothing should not increase per-frame latency beyond the configured processing budget. [ASSUMPTION: exact latency budget to be measured against current bag playback rate.]
- NFR-8 Debuggability: Debug artifacts must be optional and disabled by default for runtime performance.

## 5. Acceptance Criteria

### Goal 1: No Pose Estimation When Milk Is Absent

- AC-1: On the Milk-absent validation set, False Positive Detection Rate is <= 5%.
- AC-2: On the Milk-absent validation set, pose output suppression success rate is >= 95%.
- AC-3: No frame classified as `NO_OBJECT`, `LOW_CONFIDENCE`, or `GEOMETRY_FAILED` publishes a pose.
- AC-4: Each suppressed frame has a rejection reason in debug JSON.
- AC-5: At least one threshold sweep report demonstrates selected thresholds outperform current baseline.

### Goal 2: Stable Pose in Static Scenes

- AC-6: Static positive-scene translation jitter decreases from baseline.
- AC-7: Static positive-scene rotation jitter decreases from baseline.
- AC-8: Deterministic sampling produces repeatable pose outputs for identical saved RGB-D input.
- AC-9: Temporal smoothing does not publish stale pose after object absence exceeds configured hold duration.
- AC-10: Debug report includes raw pose, stabilized pose, confidence, and temporal state.

## 6. Technical Approach

Implementation should be staged so early risk reduction is measurable:

1. Add a `PoseGate` layer between ISM and PEM/publish. It owns object existence, candidate filtering, pose confidence, and rejection states.
2. Extend ISM fast path to preserve component scores and candidate diagnostics.
3. Add deterministic PEM point sampling before temporal smoothing, so jitter reduction is not masking randomness.
4. Add geometric verification after PEM and before pose publication.
5. Add `PoseSmoother` only for accepted poses.
6. Add offline evaluation and threshold sweep tooling.

Recommended initial configurable parameters:

- `min_existence_score`
- `min_ism_score`
- `min_semantic_score`
- `min_appearance_score`
- `min_geometric_score`
- `min_visible_ratio`
- `min_score_margin`
- `min_depth_valid_ratio`
- `min_pose_confidence`
- `min_mask_iou`
- `min_projected_bbox_iou`
- `max_depth_residual_mm`
- `max_translation_jump_mm`
- `max_rotation_jump_deg`
- `hold_last_good_frames`
- `temporal_filter.enabled`
- `debug_visualization.enabled`

## 7. Risk Analysis

- Risk 1: Thresholds overfit to current bag and fail on new lighting/backgrounds. Mitigation: evaluate on multiple Milk-present and Milk-absent scenes, and report precision/recall tradeoffs.
- Risk 2: Aggressive filtering suppresses true Milk poses. Mitigation: track positive-frame recall and use component-wise diagnostics to tune thresholds.
- Risk 3: Temporal smoothing hides real motion. Mitigation: use confidence-aware One Euro/EMA settings and disable smoothing for high-velocity motion if needed.
- Risk 4: 180-degree rotation ambiguity remains for symmetric or low-texture views. Mitigation: add symmetry-aware pose comparison and previous-pose-consistent solution selection.
- Risk 5: Geometric verification adds latency. Mitigation: start with lightweight bbox/mask/depth checks, then add heavier render/PnP checks only where needed.
- Risk 6: Debug output changes timing or behavior. Mitigation: keep debug read-only and make decisions independent from visualization code.

## 8. Metrics

Primary metrics:

- M-1 False Positive Detection Rate: Milk-absent frames with `object_present=true` / total Milk-absent frames. Target <= 5%.
- M-2 Pose Output Suppression Success Rate: Milk-absent frames without pose publication / total Milk-absent frames. Target >= 95%.
- M-3 Wrong Pose Output Count: number of published poses in Milk-absent frames. Target: zero or below accepted threshold.
- M-4 Static Translation Jitter: standard deviation and max delta of x/y/z translation over static positive frames. Target: lower than baseline.
- M-5 Static Rotation Jitter: angular distance standard deviation and max delta over static positive frames. Target: lower than baseline.

Secondary metrics:

- M-6 True Positive Pose Recall: Milk-present frames with accepted pose / total Milk-present frames.
- M-7 Candidate Rejection Rate by Reason.
- M-8 Mean Pose Confidence for accepted vs rejected frames.
- M-9 Geometric Verification Pass Rate.
- M-10 Runtime Latency per stage: ISM, filtering, PEM, geometric verification, smoothing, visualization.

Counter-metrics:

- CM-1 Do not optimize FP reduction by suppressing all poses. Positive-frame recall must be reported with FP metrics.
- CM-2 Do not optimize jitter by adding unacceptable lag. Temporal latency must be reported with jitter metrics.

## 9. Validation Dataset

The validation dataset must include:

- VD-1 Milk-absent negative frames: scenes with no Milk object, including visually similar boxes, cartons, posters, monitors, and cluttered desk backgrounds.
- VD-2 Milk-present static frames: camera and Milk object stationary, enough frames to measure jitter.
- VD-3 Milk-present varied pose frames: Milk at different positions, rotations, distances, and partial occlusions.
- VD-4 Hard negatives: objects with similar rectangular/carton shape or similar color blocks.
- VD-5 Depth edge cases: missing depth, noisy depth, reflective surfaces, and occlusion boundaries.

Minimum dataset recommendation:

- [ASSUMPTION] At least 200 Milk-absent frames.
- [ASSUMPTION] At least 200 static Milk-present frames.
- [ASSUMPTION] At least 100 varied Milk-present frames.
- [ASSUMPTION] At least 50 hard-negative frames.

Each frame or frame range must have labels:

- `milk_present: true|false`
- `static_scene: true|false`
- `hard_negative: true|false`
- Optional `expected_pose` or static reference pose when available
- Notes for occlusion, depth quality, and lighting

## 10. Test Plan

### Unit Tests

- T-1 Candidate score gate accepts/rejects candidates according to configured thresholds.
- T-2 Score margin logic handles one candidate, two candidates, ties, and empty candidate lists.
- T-3 Geometry verifier returns failure when projected bbox/mask/depth residual is invalid.
- T-4 Pose confidence scorer produces deterministic scores for fixed inputs.
- T-5 Temporal smoother handles accepted pose, rejected pose, hold-last-good-pose, and object disappearance.
- T-6 Deterministic sampling returns the same sampled point indices for the same mask/depth input.

### Integration Tests

- T-7 Milk-absent bag segment produces no pose publications above target threshold.
- T-8 Milk-present static bag segment produces stable pose publications.
- T-9 Debug visualization enabled/disabled does not change accept/reject decisions.
- T-10 Existing ROS pose consumers continue to receive valid messages when poses are accepted.
- T-11 Rejection states are published/logged without publishing invalid pose transforms.

### Evaluation Tests

- T-12 Run baseline pipeline on validation dataset and store baseline metrics.
- T-13 Run gated pipeline on validation dataset and compare against baseline.
- T-14 Run threshold sweep and select operating point satisfying FP and recall constraints.
- T-15 Run repeated inference on identical saved RGB-D frame to confirm deterministic behavior.
- T-16 Run static-sequence jitter report before and after temporal stabilization.

## Open Questions

1. What is the minimum acceptable true positive recall after adding FP suppression?
2. What is the maximum allowed end-to-end latency per frame in the ROS2 runtime?
3. Should pose estimation be skipped before PEM when object existence is low, or should PEM run but publication be suppressed for diagnostics?
4. Is an external Milk detector acceptable in MVP, or must MVP remain SAM-6D-only?
5. Does the Milk CAD/model have known symmetries that should be encoded for rotation stability?

## Assumptions Index

- Section 3 Feature A: acceptable positive recall degradation is assumed to be <= 10%.
- Section 3 Feature E: target temporal latency budget is assumed to be <= 1 frame.
- Section 4: exact runtime latency budget is not yet specified.
- Section 9: minimum validation dataset sizes are recommended assumptions pending dataset availability.
