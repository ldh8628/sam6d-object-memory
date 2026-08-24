---
title: longcircle2 SAM-6D evaluation and static-pose stabilization
status: implemented
date: 2026-08-22
---

# Intent

Make `longcircle2` independently usable from the local workspace, execute
controlled ISM/PEM comparisons, construct an explicitly qualified SLAM-based
pseudo ground truth, preserve failure cases, and test a static-scene pose-memory
path toward the requested 90% pose-success target.

# Acceptance trace

- Local data is byte-verified, read-only, and is the only entry under `data/`.
- A working `sam6d` Conda environment loads ROS 2, CUDA, ISM, and PEM.
- Pure geometry, feature rerank, and geometry+texture use the same 2,165 paired
  RGB/depth frames and deterministic per-frame seeds.
- Reference poses are stable clusters after transforming camera poses into the
  ORB-SLAM3 map frame; weak clusters are excluded.
- Failures and PEM input rejections are saved as machine-readable records and
  visual contact sheets.
- Static pose memory is reported separately from standalone PEM accuracy and is
  evaluated only after causal per-object initialization.

# Suggested review order

1. `output/longcircle2_experiments/20260822_SUMMARY.md`
2. `output/longcircle2_experiments/20260822_slam_eval_final/REPORT.md`
3. `output/longcircle2_experiments/20260822_failure_analysis_final/FAILURE_ANALYSIS.md`
4. `realtime/slam_pose_memory.py`
5. `sam6d_master/SAM-6D/Pose_Estimation_Model/run_inference_custom.py`

