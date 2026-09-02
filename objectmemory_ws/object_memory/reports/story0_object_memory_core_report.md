# Story 0 Object Memory Core Report

Date: 2026-07-01

## Implemented Scope

Standalone `object_memory` baseline, independent of SAM-6D internals, SLAM
backends, Unity, ROS2 custom messages, launch files, and runtime nodes.

- `core.transforms` — rigid transform utilities (standard library only).
- `core.state_machine` — `ObjectStatus` lifecycle enum, `StateTransition`,
  transition validation.
- `core.models` — `TrackingStatus`, `DecisionType`, `SlamCameraPose`,
  `Sam6DDetection`, `ObjectLandmark`, `AssociationDecision`.
- `core.memory_store` — `ObjectMemoryStore`, sole owner of persistent
  `object_id`.
- `core.association_contracts` — `AssociationInput` / `AssociationOutput`
  (contracts only, no matching logic).
- `object_memory` — compatibility re-export package.
- `scripts_test/check_sam_circle_bag.py` — read-only bag usability checker.

## Transform Formula Coverage

- `T_map_obj = T_map_cam * T_cam_obj` (`compute_T_map_obj`) — INV-004.
- `T_cam_curr_obj_pred = inverse(T_map_cam_curr) * T_map_obj_prev`
  (`predict_current_camera_object_pose`) — Story 4 precondition.
- rigid inverse (`R^T`, `-R^T t`), translation distance, rotation distance
  in degrees.

## Validation Commands

```bash
cd objectmemory_ws/object_memory
python3 -m compileall src scripts_test
python3 -m pytest scripts_test -q
python3 scripts_test/check_sam_circle_bag.py \
  --bag-path ~/temp_ws/CLI_environment/data_slam/rgbd_imu_sdk_bag/SAM_circle \
  --output-report reports/story0_sam_circle_bag_check.md \
  --max-topic-samples 5
```

## Validation Results

- `compileall`: PASS.
- `pytest`: 31 passed (spec target was 21; superset).
- bag checker exit code: 0.

## SAM_circle Verdict

`부분 사용 가능` — RGB (`/camera/camera/color/image_raw`), aligned depth,
both `camera_info` topics, and IMU are present with valid timestamps
(1322 image frames, 8988 IMU samples). Missing `/tf`, `/tf_static`, SLAM pose,
and SAM-6D object output — as expected for a raw RGB-D/IMU capture.

## Created Files

```text
object_memory/
  pyproject.toml
  README.md
  src/core/{__init__,transforms,state_machine,models,memory_store,association_contracts}.py
  src/object_memory/__init__.py
  scripts_test/{check_sam_circle_bag,test_transforms,test_state_machine,test_memory_store,test_sam_circle_bag_contract}.py
  reports/{story0_sam_circle_bag_check,story0_object_memory_core_report,story0-dev-story-result}.md
```

No fake bag / dataset / SAM-output folders were created.

## Acceptance Criteria Status

- [x] `src/core` imports work.
- [x] transform tests pass.
- [x] state machine tests pass.
- [x] memory store tests pass.
- [x] SAM_circle verdict recorded (`부분 사용 가능`).
- [x] no fake data folders created.

## Next Story Order

Story 1 (Common SLAM Pose Adapter) → 2 → 3 → 4 → 5 → 6 → 7.
Recommended Story 1 backend: ORB-SLAM3 (mature outputs already in this
workspace). Do not implement two SLAM backends in the first story.
