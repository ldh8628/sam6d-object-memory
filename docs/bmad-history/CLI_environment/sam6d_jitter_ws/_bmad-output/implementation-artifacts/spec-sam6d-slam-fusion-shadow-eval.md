---
title: 'SAM-6D SLAM Fusion Shadow Evaluation MVP'
type: 'feature'
created: '2026-06-10'
status: 'done'
baseline_commit: '503e871dc4ff6dd4b4d4b03b21dd3f939ea4bbf3'
context:
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-slam-based-camera-pose-stabilization-for-sam-6d-pose-jitter-reduction-research-2026-06-10.md'
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-static-object-6d-pose-jitter-reduction-for-sam-6d-research-2026-06-10.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** SAM-6D camera-frame object pose jitter may be reducible by composing it with SLAM camera/base localization, but this must be validated quickly as a product decision experiment rather than a new pose-estimation algorithm.

**Approach:** Add a separate ROS2 shadow-evaluation node that subscribes to SAM-6D object poses, ORB-SLAM3 / hdl_graph_slam / RTAB-Map pose outputs, and TF, then publishes fused/debug topics and writes CSV plus a markdown verdict report.

## Boundaries & Constraints

**Always:** Keep production SAM-6D outputs intact. Publish fused/debug outputs under separate topics. Continue evaluating other SLAM branches when one branch is missing or not evaluable. Mark base/LiDAR SLAM branches without `T_BC` as `not_evaluable` rather than crashing. Use configurable topics and frames discovered from local code where available.

**Ask First:** Any change to SAM-6D core model/ISM/PEM, existing recall/FP logic, existing production topic names, or files outside `/home/etri/CLI_environment/sam6d_jitter_ws`.

**Never:** Implement a new factor graph or modify ORB-SLAM3, hdl_graph_slam, RTAB-Map, or `../sam6d_ws`. Do not replace production PoseArray/TF output. Do not make a failed SLAM branch fail the whole MVP.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| RGB-D SLAM fusion | SAM-6D `T_CO` and SLAM camera pose `T_WC` are available | Publish `T_WO = T_WC * T_CO`, append CSV row, update metrics | If stale/missing pose, row has `fusion_valid=false` and reason |
| Base/LiDAR SLAM fusion | SAM-6D `T_CO`, SLAM base pose `T_WB`, and TF `T_BC` are available | Publish `T_WO = T_WB * T_BC * T_CO` | If `T_BC` is missing, mark branch `not_evaluable` only |
| Missing SAM pose | No current SAM-6D pose | Still emit branch status in metrics/report as invalid | Do not publish fused pose |
| Simple hold shadow | Fused pose jump exceeds configured thresholds | Keep/report held world pose on hold debug topic/metrics only | Do not replace production SAM-6D output |

</frozen-after-approval>

## Code Map

- `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py` -- local SAM-6D node publishes `/sam6d_inference/poses` PoseArray and configurable `pose_6dof_topic` TransformStamped with `camera_color_optical_frame` parent.
- `/home/etri/CLI_environment/orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp` -- ORB-SLAM3 publishes `/orbslam3/pose`, `/orbslam3/odom`, `/orbslam3/tracking_state`, and TF `world_frame -> camera_frame`.
- `/home/etri/CLI_environment/hdlgraphslam_ws/src/hdl_graph_slam/src/hdl_graph_slam_ros2/scan_matching_odometry_node.cpp` -- hdl scan matcher publishes `/odom`, `/scan_matching_odometry/transform`, `/scan_matching_odometry/status`, and odom TF to input cloud/base frame.
- `src/sam6d_slam_fusion_eval/` -- new isolated shadow-evaluation ROS2 package.

## Tasks & Acceptance

**Execution:**
- [x] `src/sam6d_slam_fusion_eval/package.xml`, `setup.py`, package files -- create a ROS2 Python package with a `sam6d_slam_fusion_shadow_eval` console script.
- [x] `src/sam6d_slam_fusion_eval/config/sam6d_slam_fusion_shadow_eval.yaml` -- add default topics/frames for SAM-6D, ORB-SLAM3, hdl_graph_slam, and RTAB-Map, including discovered topic notes.
- [x] `src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py` -- implement subscribers, transform composition, TF lookup for `T_BC`, branch validity, publishers, CSV writer, rolling metrics, simple hold shadow, and markdown report writer.
- [x] `src/sam6d_slam_fusion_eval/README.md` -- document topic discovery, run commands, CSV/report outputs, verdict rules, and final decision questions.

**Acceptance Criteria:**
- Given SAM-6D TransformStamped and ORB-SLAM3 PoseStamped are received, when the node spins, then `/sam6d/slam_fused/orbslam3/object_pose_world` publishes composed world poses and CSV rows include raw/world deltas.
- Given hdl_graph_slam odometry is received but camera extrinsic TF is missing, when SAM poses arrive, then the hdl branch is marked `not_evaluable` without stopping ORB/RTAB evaluation.
- Given no RTAB-Map source is running, when SAM/other SLAM inputs arrive, then report valid frame count for RTAB is zero and other branches continue.
- Given enough samples are logged, when the node shuts down or report timer fires, then the markdown report contains the required condition table and GO/WEAK GO/NO-GO verdict rule outputs.

## Spec Change Log

## Design Notes

Use pure Python quaternion/transform math in the evaluator to avoid pulling SAM-6D or SLAM libraries into the shadow path. Treat SAM raw jitter in camera frame as its own condition; fused world jitter is calculated per branch. `sam6d_score` remains blank/NaN unless a future SAM debug topic provides score, because current local SAM-6D production topics do not publish score.

## Verification

**Commands:**
- `colcon build --packages-select sam6d_slam_fusion_eval --symlink-install` -- expected: package builds without touching `../sam6d_ws`.
- `python3 -m py_compile src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py` -- expected: syntax succeeds.

## Suggested Review Order

**Shadow Node Flow**

- Start here to understand the isolated evaluator design.
  [`shadow_eval_node.py:299`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L299)

- Branch setup wires independent fused and hold-debug publishers.
  [`shadow_eval_node.py:422`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L422)

- SAM callback path logs raw pose and evaluates every branch.
  [`shadow_eval_node.py:628`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L628)

**Fusion Safety**

- Missing SAM poses still create invalid CSV/debug branch rows.
  [`shadow_eval_node.py:586`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L586)

- Fusion rejects stale timestamps, frame mismatch, and missing extrinsics.
  [`shadow_eval_node.py:726`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L726)

- Simple hold remains a shadow-only debug stream.
  [`shadow_eval_node.py:777`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L777)

**Metrics And Verdict**

- Markdown report generates the required condition table.
  [`shadow_eval_node.py:911`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L911)

- Verdict logic prevents GO when FPS impact is unknown.
  [`shadow_eval_node.py:982`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L982)

- Final decision questions are emitted in the report.
  [`shadow_eval_node.py:1012`](../../src/sam6d_slam_fusion_eval/sam6d_slam_fusion_eval/shadow_eval_node.py#L1012)

**Config And Docs**

- Default topics capture discovered SAM/ORB/hdl/RTAB candidates.
  [`sam6d_slam_fusion_shadow_eval.yaml:1`](../../src/sam6d_slam_fusion_eval/config/sam6d_slam_fusion_shadow_eval.yaml#L1)

- README documents outputs, run commands, and review questions.
  [`README.md:53`](../../src/sam6d_slam_fusion_eval/README.md#L53)
