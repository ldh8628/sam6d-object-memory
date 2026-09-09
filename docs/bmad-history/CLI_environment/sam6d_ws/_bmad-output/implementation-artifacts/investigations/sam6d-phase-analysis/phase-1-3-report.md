# SAM6D Milk Reliability Phase 1-3 Report

Date: 2026-06-09

## Scope

This report covers code-grounded analysis before any implementation change. No pipeline code was modified during this phase.

Inputs:
- PRD: `_bmad-output/planning-artifacts/prds/prd-sam6d_ws-2026-06-09/prd.md`
- Technical investigation: `_bmad-output/implementation-artifacts/investigations/sam6d-fp-jitter-investigation.md`
- Runtime config: `src/sam6d_ros/config/no_cli_milk.yaml`
- Existing output: `output/sam6d_milk_verify/*/detection_ism.json`, `detection_pem.json`
- Bag: `data/rgbd_bag/bag_0.db3`

Generated analysis artifacts:
- `sam6d-phase-analysis/ism_candidates.csv`
- `sam6d-phase-analysis/pem_best_pose.csv`
- `sam6d-phase-analysis/summary.json`
- `output/bag_frame_samples/color_000.png`, `color_026.png`, `color_053.png`, `color_080.png`, `color_106.png`

## Phase 1: Pipeline Trace

### Actual Runtime Path

The active ROS path is not the subprocess path in `run_batch_inference.py`. The ROS node imports and calls `run_batch_inference_fast.py` in-process.

Actual stack for online ROS inference:

1. ROS image sync callback receives RGB/depth.
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:420`
2. `_run_inference_from_images()` writes `live_input/rgb.png` and `live_input/depth.png`.
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:452`
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:480`
3. `_run_inference()` creates per-frame output dir and calls ISM.
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:763`
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:789`
4. `run_ism_single()` generates FastSAM proposals and computes ISM scores.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:358`
5. `_run_inference()` calls PEM with the generated `detection_ism.json`.
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:809`
6. `run_pem_single()` loads candidates, samples RGB-D points, runs PEM, and writes `detection_pem.json`.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:637`
7. `_run_inference()` converts PEM results into `PoseDetection`.
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:842`
8. `_publish_poses()` publishes `PoseArray` and `TransformStamped`.
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:906`

Model/template load stack:

1. `_load_models()` imports fast-path loader functions.
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:320`
2. `load_ism_model()` composes Hydra config and instantiates FastSAM/SAM + DINOv2 descriptor.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:169`
3. `load_ism_templates()` loads rendered `rgb_*.png`, `mask_*.png`, extracts DINOv2 template descriptors, stores template poses and CAD point cloud in `model.ref_data`.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:227`
4. `load_pem_model_and_templates()` loads PEM checkpoint and extracts PEM template features.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:451`
5. `preload_pem_mesh()` samples CAD points once for PEM preprocessing.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:480`

### PLY -> Render

Rendering is done before runtime by `Render/render_custom_templates.py`.

Inputs:
- `--cad_path`
- `--output_dir`
- `--normalize`
- `--colorize`
- `cam_poses_level0.npy`

Outputs:
- `{output_dir}/templates/rgb_{idx}.png`
- `{output_dir}/templates/mask_{idx}.png`
- `{output_dir}/templates/xyz_{idx}.npy`

Code evidence:
- CAD load: `sam6d_master/SAM-6D/Render/render_custom_templates.py:54`
- Scale/normalization: `sam6d_master/SAM-6D/Render/render_custom_templates.py:49`
- Camera poses: `sam6d_master/SAM-6D/Render/render_custom_templates.py:46`
- RGB/mask/xyz save: `sam6d_master/SAM-6D/Render/render_custom_templates.py:124`

### Render Feature

ISM render features:
- Loads rendered template RGB/mask.
- Crops/resizes templates to 224.
- Computes DINOv2 class descriptors and masked patch descriptors.

Code evidence:
- Template RGB/mask load: `sam6d_master/SAM-6D/run_batch_inference_fast.py:232`
- Crop/resize: `sam6d_master/SAM-6D/run_batch_inference_fast.py:248`
- DINOv2 descriptors: `sam6d_master/SAM-6D/run_batch_inference_fast.py:252`
- Template pose and CAD point cloud: `sam6d_master/SAM-6D/run_batch_inference_fast.py:260`

PEM render features:
- Loads `rgb`, `mask`, `xyz` template triplets.
- Randomly samples template mask points.
- Extracts PEM object features once at model load.

Code evidence:
- Template RGB/mask/xyz load: `sam6d_master/SAM-6D/run_batch_inference_fast.py:488`
- Template random point sampling: `sam6d_master/SAM-6D/run_batch_inference_fast.py:509`
- PEM object feature extraction: `sam6d_master/SAM-6D/run_batch_inference_fast.py:466`

### ISM Candidate

Inputs:
- RGB image
- Depth image
- Camera intrinsics
- ISM rendered template descriptors
- CAD point cloud

Outputs:
- `detection_ism.json`
- `scores` list in memory

Flow:
1. FastSAM creates masks and boxes.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:382`
2. Optional proposal cap keeps largest-area proposals before DINOv2 matching.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:386`
3. DINOv2 descriptors are extracted for proposals.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:394`
4. Semantic score filters proposals using `matching_config.confidence_thresh`.
   - `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:260`
   - `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:286`
5. Appearance score is computed against best template.
   - `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:298`
6. Geometric score is box IoU between projected template bbox and proposal box, weighted later by visible ratio.
   - `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:310`
7. Final score is computed.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:416`

Scores used:
- `semantic_score`
- `appe_scores`
- `geometric_score`
- `visible_ratio`
- `final_score = (semantic + appearance + geometric * visible_ratio) / (2 + visible_ratio)`

Thresholds:
- FastSAM `conf_threshold=0.05`, `iou_threshold=0.9`, `max_det=200`.
  - `sam6d_master/SAM-6D/Instance_Segmentation_Model/configs/model/segmentor_model/fast_sam.yaml:5`
- ISM semantic `confidence_thresh=0.2`.
  - `sam6d_master/SAM-6D/Instance_Segmentation_Model/configs/model/ISM_fastsam.yaml:25`
- Runtime `max_proposals=50`, `top_k_for_pem=5`.
  - `src/sam6d_ros/config/no_cli_milk.yaml:19`

### Candidate Filtering

Current fast-path filtering stages:

1. FastSAM internal model threshold: `conf_threshold=0.05`.
2. Optional largest-area cap: `max_proposals`.
3. Semantic threshold: `confidence_thresh=0.2`.
4. Top-K final score save: `top_k_for_pem`.
5. PEM input threshold: `det_score_thresh=0.2`.
6. PEM validity: mask depth > 32 pixels and radius-based cloud pruning.

Missing in current fast path:
- No final object existence gate.
- No final score threshold before saving ISM JSON.
- No component-wise threshold for appearance/geometric/visible ratio.
- No score margin check.
- No small detection removal in `run_ism_single()`.
- No per-object NMS in `run_ism_single()`.
- No post-PEM geometric verification.
- No publish-time confidence gate.

Code evidence:
- `run_ism_single()` does not call `remove_very_small_detections()` or `apply_nms_per_object_id()`.
  - `sam6d_master/SAM-6D/run_batch_inference_fast.py:382`
  - `sam6d_master/SAM-6D/run_batch_inference_fast.py:425`
- Original detector has small detection removal and NMS, but this method is not used by the fast path.
  - `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:352`
  - `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:388`
- PEM filters only `det["score"] > det_score_thresh`.
  - `sam6d_master/SAM-6D/run_batch_inference_fast.py:546`

### Pose Estimation

Inputs to PEM:
- Filtered ISM JSON candidate list
- RGB crop masked by candidate mask
- Observed point cloud sampled from depth inside candidate mask
- Template object points/features
- CAD sampled model points
- Camera intrinsics

Outputs:
- `detection_pem.json`
- In-memory `poses` list with `ism_score`, `pem_score`, `final_score`, `t_mm`, `R`

Flow:
1. `_get_pem_test_data()` loads candidates above `det_score_thresh`.
2. It converts depth to point cloud and crops by candidate mask bbox.
3. It filters points by CAD radius around observed cloud center.
4. It randomly samples `n_sample_observed_point=2048` points.
5. PEM extracts dense image/object features.
6. Coarse matching computes an initial pose.
7. Fine matching computes final pose and `pred_pose_score`.
8. `pose_scores = pred_pose_score * ISM_score`.

Code evidence:
- Candidate threshold: `sam6d_master/SAM-6D/run_batch_inference_fast.py:546`
- Observed point random sampling: `sam6d_master/SAM-6D/run_batch_inference_fast.py:587`
- PEM forward: `sam6d_master/SAM-6D/run_batch_inference_fast.py:670`
- Pose score product: `sam6d_master/SAM-6D/run_batch_inference_fast.py:676`
- PEM network structure: `sam6d_master/SAM-6D/Pose_Estimation_Model/model/pose_estimation_model.py:23`
- Coarse pose: `sam6d_master/SAM-6D/Pose_Estimation_Model/model/coarse_point_matching.py:70`
- Fine pose and `pred_pose_score`: `sam6d_master/SAM-6D/Pose_Estimation_Model/model/fine_point_matching.py:75`

Important correction:
- This code path does not use OpenCV PnP.
- It uses attention-based correspondence, random pose hypothesis sampling in coarse matching, and Weighted Procrustes/SVD pose estimation.
- There is no explicit ICP/render refinement stage in the active code path.

### Pose Refinement

Observed active implementation:
- No separate pose refinement module after PEM.
- "Refinement" equivalent is the internal coarse-to-fine PEM process:
  - CoarsePointMatching computes `init_R`, `init_t`.
  - FinePointMatching computes `pred_R`, `pred_t`, `pred_pose_score`.

No external render-and-compare refinement, ICP, or PnP-RANSAC refinement was found in the active code path.

## Phase 2: False Positive Analysis

### Data Availability

Current bag metadata:
- Duration: 4.68 s
- Color frames: 107
- Aligned depth frames: 106
- Topics include `/camera/camera/color/image_raw` and `/camera/camera/aligned_depth_to_color/image_raw`.

The current Python environment does not have `rosbag2_py`, `rclpy`, `sensor_msgs`, or `cv_bridge`, but the SQLite CDR payload could be parsed directly for `sensor_msgs/Image`. Sample extracted frames all show the Milk object present:
- `output/bag_frame_samples/color_000.png`
- `output/bag_frame_samples/color_053.png`
- `output/bag_frame_samples/color_106.png`

Conclusion:
- The available bag is a Milk-present static positive bag, not a Milk-absent negative bag.
- Phase 2 cannot be completed as empirical negative-bag analysis until a Milk-absent bag or labeled negative frame set is provided.

### Code-Grounded Root Cause for FP Passage

Confirmed root cause area:

1. FastSAM is a generic proposal generator, not a Milk detector.
2. FastSAM objectness threshold is low (`0.05`).
3. ISM semantic threshold is low (`0.2`).
4. Fast path saves top-K candidates by final score even without final object existence threshold.
5. PEM accepts any candidate with `score > det_score_thresh`, and current Milk config uses `0.2`.
6. Existing output has ISM score range `0.411-0.639`, so all saved ISM candidates pass the current PEM threshold.
7. Publish path converts every PEM pose with `R`/`t` into a ROS pose without a final confidence gate.

### Reproduction Procedure for Negative Bag

Required input:
- A ROS2 bag or extracted RGB-D sequence where Milk is absent.

Steps:
1. Run current pipeline using `src/sam6d_ros/config/no_cli_milk.yaml`.
2. Save `detection_ism.json` and `detection_pem.json` for every frame.
3. Extract candidate score/bbox/mask stats:
   - candidate count
   - ISM final score
   - PEM final score
   - bbox area/aspect ratio
   - valid depth ratio
4. Count frames where:
   - ISM candidate exists
   - PEM runs
   - pose is published
5. Compute:
   - false positive detection rate
   - pose suppression failure rate
   - per-threshold sweep results

### Modification Points

Recommended first modification points:

1. `run_batch_inference_fast.py::run_ism_single`
   - preserve component scores
   - apply small detection removal/NMS consistently
   - apply configurable final/component score filters
   - write debug candidate JSON
2. `sam6d_inference_node.py::_run_inference`
   - skip PEM if no candidate remains
   - treat no candidate as `NO_OBJECT`, not fatal error
3. `run_batch_inference_fast.py::run_pem_single`
   - expose enough pose confidence data for gate
4. `sam6d_inference_node.py::_publish_poses`
   - publish only accepted pose states after gate

## Phase 3: Pose Jitter Analysis

### Available Static Positive Data

Existing output frames:
- ISM JSON: 26 frames
- PEM JSON: 25 frames
- Bag sample images show stationary Milk-present scene.

### Numeric Results

From `summary.json`:

ISM top-1 candidate:
- Score mean/std: `0.62795 / 0.00495`
- Bbox x std: `0.27 px`
- Bbox y std: `4.70 px`
- Bbox w std: `1.33 px`
- Bbox h std: `4.64 px`
- Bbox area std: `535.89 px^2`

PEM best pose:
- Score mean/std: `0.61787 / 0.01206`
- Translation mean: `[-8.80, 237.97, 476.21] mm`
- Translation std: `[0.76, 4.47, 1.04] mm`
- Adjacent translation delta mean/max: `5.26 / 16.18 mm`
- Adjacent rotation delta mean/std/max: `30.96 / 61.28 / 179.37 deg`
- Rotation from first frame mean/std/max: `161.38 / 50.66 / 179.94 deg`

### Jitter 발생 단계

Confirmed from code and outputs:

1. ISM candidate bbox jitter exists, especially y/h dimensions.
2. PEM input point cloud changes because observed points are randomly sampled every inference.
   - `sam6d_master/SAM-6D/run_batch_inference_fast.py:587`
3. PEM coarse pose hypothesis generation uses `torch.rand` sampling.
   - `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py:221`
4. PEM final pose is Weighted Procrustes from soft correspondences, not PnP.
   - `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py:263`
5. No temporal smoothing or previous-pose selection exists before publish.
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:842`
   - `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:906`

Not currently observable without instrumentation:
- Feature correspondence changes per frame.
- Coarse `init_R`/`init_t` changes.
- Fine matching assignment matrix changes.
- Internal `pred_pose_score` distribution before multiplication with ISM score.

### Root Cause

Primary jitter root causes:

1. Candidate bbox/mask jitter from ISM.
2. Random observed-point sampling in PEM preprocessing.
3. Random coarse pose hypothesis sampling inside PEM.
4. No symmetry-aware selection or temporal consistency check.
5. No temporal filter before ROS publish.

The 180-degree rotation jumps suggest either:
- object/model symmetry ambiguity,
- multiple equivalent correspondence solutions,
- or unstable selection between pose hypotheses.

This needs instrumentation before being treated as a solved root cause.

## Phase 4 Design Proposal Before Code Changes

Implementation must be staged and verified after each step.

### Step 1: Candidate Filtering

Design:
- Add a pure filtering helper in `run_batch_inference_fast.py` that accepts scored candidate rows and returns accepted/rejected candidates with reasons.
- Preserve component scores in `detection_ism_debug.json`.
- Add thresholds with defaults that preserve current behavior unless explicitly enabled.

Initial thresholds:
- `min_ism_score`
- `min_semantic_score`
- `min_appearance_score`
- `min_geometric_score`
- `min_visible_ratio`
- `min_score_margin`
- `min_mask_area`
- `max_mask_area`
- `min_depth_valid_ratio`

Verification:
- Run current positive bag and confirm true candidate still passes.
- Run threshold sweep once negative bag is available.

### Step 2: Object Existence Verification

Design:
- Add frame-level state: `OBJECT_PRESENT` or `NO_OBJECT`.
- Skip PEM when no candidate passes filtering.
- Return `ok=True`, `poses=[]`, and explicit state instead of treating no detection as a runtime failure.

Verification:
- Synthetic no-candidate JSON test.
- Negative bag once available.

### Step 3: Geometric Verification

Design:
- After PEM, project CAD/model points using predicted pose and camera intrinsics.
- Compare projected bbox against candidate bbox.
- Compute depth residual for projected visible points where observed depth exists.
- Gate pose before publish.

Verification:
- Positive static bag should pass most frames.
- Negative/hard-negative bag should reject inconsistent poses.

### Step 4: Pose Confidence Score

Design:
- Combine ISM score, PEM score, geometric pass metrics, score margin, and temporal delta into a single `pose_confidence`.
- Publish pose only when confidence exceeds threshold.

Verification:
- Accepted poses must include confidence breakdown.
- Rejected poses must include reason.

### Step 5: Temporal Stabilization

Design:
- First remove deterministic causes:
  - deterministic PEM input sampling
  - deterministic torch generator or seed for coarse hypothesis sampling in evaluation mode
- Then add `PoseSmoother`:
  - translation EMA or One Euro
  - quaternion slerp
  - jump reject
  - hold-last-good-pose with max hold frames

Verification:
- Compare static jitter metrics against baseline CSV.
- Ensure object disappearance does not continue publishing stale pose beyond configured hold window.

## Current Blockers

- Phase 2 empirical FP analysis requires a Milk-absent bag or labeled negative RGB-D frames.
- Feature correspondence/coarse/fine internal jitter analysis requires instrumentation because current outputs do not expose correspondence matrices or coarse pose.

