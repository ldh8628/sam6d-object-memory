# Object Memory Baseline Reconstruction Guide

Date: 2026-07-01

## 1. Purpose

이 문서는 `object_memory` baseline 코드가 없는 다른 PC에서, markdown만 보고 최소 baseline을 다시 생성하기 위한 재구성 지시서이다.

대상 baseline은 완성된 SLAM/SAM-6D/Unity 통합 시스템이 아니다. 다음 기능만 갖는 독립 Python baseline이다.

- core data model
- transform utility
- object lifecycle state machine
- in-memory object memory store
- association input/output contract
- SAM_circle ros2 bag usability checker
- pytest 기반 최소 테스트

이 baseline의 목적은 이후 Story 1 이후 작업을 시작하기 전, `object_memory`의 핵심 수식과 자료구조가 독립적으로 동작한다는 것을 보장하는 것이다.

## 2. Required Final Directory Structure

빈 PC에서 다음 구조를 만든다.

```text
~/git_ws/object_memory/
  README.md
  pyproject.toml
  src/
    core/
      __init__.py
      models.py
      transforms.py
      state_machine.py
      memory_store.py
      association_contracts.py
    object_memory/
      __init__.py
  scripts_test/
    check_sam_circle_bag.py
    test_transforms.py
    test_state_machine.py
    test_memory_store.py
    test_sam_circle_bag_contract.py
  reports/
    story0_sam_circle_bag_check.md
    story0_object_memory_core_report.md
    story0-dev-story-result.md
```

`src/core/`에는 runtime에 필요한 core 코드만 둔다.

`scripts_test/`에는 테스트와 확인용 스크립트만 둔다.

`reports/`에는 실행 결과 문서만 둔다.

## 3. Do Not Create

다음은 baseline 재구성 중 만들지 않는다.

- fake ros2 bag folder
- fake RGB/depth dataset folder
- fake SAM output folder
- ROS2 package.xml
- ROS2 launch file
- ROS2 custom msg file
- Unity client code
- SAM-6D 내부 수정 파일
- SLAM backend 내부 수정 파일

필요한 synthetic pose는 pytest 코드 내부의 Python object로만 만든다.

## 4. pyproject.toml

`~/git_ws/object_memory/pyproject.toml`:

```toml
[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[project]
name = "object-memory"
version = "0.1.0"
description = "Standalone object memory core for SLAM-assisted SAM-6D object ID association"
requires-python = ">=3.10"

[tool.pytest.ini_options]
testpaths = ["scripts_test"]
pythonpath = ["src", "scripts_test"]
addopts = "-q"

[tool.setuptools.packages.find]
where = ["src"]
```

## 5. Runtime Core Files

### 5.1 `src/core/transforms.py`

Required functions:

- `make_transform(rotation, translation)`
- `identity_transform()`
- `compose_transform(a, b)`
- `invert_transform(transform)`
- `transform_distance_translation(a, b)`
- `transform_distance_rotation_deg(a, b)`
- `compute_T_map_obj(T_map_cam, T_cam_obj)`
- `predict_current_camera_object_pose(T_map_cam_curr, T_map_obj_prev)`

Required formulas:

```text
T_map_obj = T_map_cam * T_cam_obj
T_cam_curr_obj_pred = inverse(T_map_cam_curr) * T_map_obj_prev
```

Implementation requirements:

- Use only Python standard library.
- Represent transforms as immutable 4x4 nested tuples.
- Validate input dimensions.
- Use rigid transform inverse: `R^T`, `-R^T t`.
- Rotation distance must return degrees.

### 5.2 `src/core/state_machine.py`

Required enum:

```text
tentative
active
lost
remembered
merged
deleted
```

Required dataclass:

```text
StateTransition
  from_status
  to_status
  reason
  stamp optional
  metadata dict
```

Required functions:

- `coerce_status(status)`
- `is_transition_allowed(from_status, to_status)`
- `build_transition(from_status, to_status, reason, stamp=None, metadata=None)`

Allowed transitions:

```text
tentative -> active
tentative -> deleted
active -> lost
lost -> active
lost -> remembered
remembered -> active
active -> merged
remembered -> merged
any -> deleted
```

Rules:

- Empty transition reason must raise `ValueError`.
- Invalid transition must raise `ValueError`.

### 5.3 `src/core/models.py`

Required enums:

```text
TrackingStatus:
  OK
  DEGRADED
  LOST
  RELOCALIZING
  LOOP_CLOSING
  RESET

DecisionType:
  short_term_match
  long_term_match
  new_tentative
  rejected
  ambiguous
```

Required dataclasses:

```text
SlamCameraPose
  stamp: float
  frame_id: str
  child_frame_id: str
  T_map_cam: Matrix4
  tracking_status: TrackingStatus
  source_slam_id: str
  confidence: optional float
  map_version: optional int

Sam6DDetection
  stamp: float
  frame_id: str
  detection_id: int
  object_name: str
  T_cam_obj: Matrix4
  score: float
  bbox optional
  depth_quality optional
  model_id optional
  size_extent optional
  class_id optional

ObjectLandmark
  object_id: int
  object_name: str
  status: ObjectStatus
  T_map_obj: Matrix4
  confidence: float
  first_seen_time: float
  last_seen_time: float
  observation_count: int = 1
  missed_count: int = 0
  T_map_obj_smoothed optional
  last_detection_id optional
  last_sam6d_score optional
  source_slam_status optional
  class_id optional
  transition_history list

AssociationDecision
  decision_type: DecisionType
  detection_id: int
  score: float
  object_id optional
  reject_reason optional
  debug_info dict
```

Validation rules for `AssociationDecision`:

- `short_term_match` and `long_term_match` require `object_id`.
- `rejected` requires `reject_reason`.
- `score` must be in `[0, 1]`.
- `detection_id` and `object_id` must be separate fields.

### 5.4 `src/core/memory_store.py`

Required class:

```text
ObjectMemoryStore
```

Required behavior:

- Constructor accepts `starting_object_id=1`.
- `starting_object_id < 1` raises `ValueError`.
- `create_tentative(...)` creates a new `ObjectLandmark` with status `tentative`.
- `object_id` auto-increments.
- `get(object_id)` returns object or `None`.
- `require(object_id)` returns object or raises `KeyError`.
- `all_landmarks()` returns all landmarks.
- `by_status(status)` returns landmarks with matching status.
- `active_objects()` returns active landmarks.
- `remembered_or_lost_objects()` returns remembered/lost landmarks.
- `update_landmark(object_id, **changes)` updates dataclass fields.
- `update_landmark` must reject object_id changes.
- `update_landmark` must reject direct status changes.
- `apply_transition(object_id, to_status, reason, stamp=None, metadata=None)` applies state transition and records reason.
- `transition_log()` returns transition history.
- `debug_summary()` returns object count, next object id, status counts, transition count.

### 5.5 `src/core/association_contracts.py`

Required dataclasses:

```text
AssociationInput
  detections
  slam_pose
  active_tracks
  memory_candidates

AssociationOutput
  decisions
  updated_landmarks
  debug_summary
```

This file must not implement matching logic yet. It only defines contracts for later stories.

### 5.6 `src/object_memory/__init__.py`

This is an optional compatibility re-export package.

It should import and expose:

- `AssociationDecision`
- `DecisionType`
- `ObjectLandmark`
- `ObjectStatus`
- `Sam6DDetection`
- `SlamCameraPose`
- `TrackingStatus`

The actual implementation must remain in `src/core/`.

## 6. Bag Checker Script

Create:

```text
scripts_test/check_sam_circle_bag.py
```

Purpose:

- Check real SAM_circle bag usability.
- Do not copy bag contents.
- Do not create fake bag/dataset/SAM output folders.

Default bag path:

```text
~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle
```

Required CLI options:

```text
--bag-path
--output-report
--max-topic-samples
```

Required behavior:

1. Expand `~`.
2. Check whether bag path exists.
3. Find `metadata.yaml` in either:

```text
<bag_path>/metadata.yaml
<bag_path>/bag/metadata.yaml
<bag_path>/*/metadata.yaml
```

4. Try `ros2 bag info <resolved_bag_dir>` if `ros2` is available.
5. If `ros2 bag info` is unavailable or fails, fallback to parsing `metadata.yaml`.
6. If sqlite `.db3` exists, read:

```sql
select id, name, type from topics;
select count(*), min(timestamp), max(timestamp) from messages where topic_id = ?;
select timestamp from messages where topic_id = ? order by timestamp limit ?;
```

7. Classify topics:

- RGB image topic
- depth image topic
- camera_info topic
- IMU topic
- `/tf`
- `/tf_static`

8. Produce verdict:

```text
사용 가능
부분 사용 가능
사용 불가
```

Verdict rules:

- `사용 가능`: RGB/depth/camera_info timestamp data exists and association-related topics such as SLAM/TF and SAM/object output are present.
- `부분 사용 가능`: RGB/depth/camera_info timestamp data exists, but SLAM pose or SAM-6D object output is missing.
- `사용 불가`: bag folder missing, metadata missing, or timestamp/source topics cannot be read.

9. Write markdown report to `reports/story0_sam_circle_bag_check.md`.

Report must include:

- bag path
- resolved bag dir
- folder exists
- metadata exists
- ros2 bag info success/failure
- verdict
- reasons
- RGB/depth/camera_info/IMU/TF topic summary
- topic table with count and timestamp samples
- policy check showing fake folders were not created

## 7. Required Tests

### 7.1 `scripts_test/test_transforms.py`

Must test:

- identity compose
- inverse round trip
- `T_map_obj = T_map_cam * T_cam_obj`
- `T_cam_curr_obj_pred = inverse(T_map_cam_curr) * T_map_obj_prev`
- translation distance
- rotation distance in degrees

### 7.2 `scripts_test/test_state_machine.py`

Must test:

- `tentative -> active`
- `active -> lost`
- `lost -> remembered`
- `remembered -> active`
- `tentative -> deleted`
- invalid transition rejected
- empty reason rejected

### 7.3 `scripts_test/test_memory_store.py`

Must test:

- tentative creation
- auto-increment object_id
- no duplicate object_id
- status query
- active query
- remembered/lost query
- landmark update
- direct status update rejected
- transition reason recorded
- debug summary
- `AssociationDecision` distinguishes `detection_id` and `object_id`
- rejected decision requires reject reason

### 7.4 `scripts_test/test_sam_circle_bag_contract.py`

Must test:

- If default SAM_circle path exists, call bag checker and verify returned schema.
- Verdict must be one of:

```text
사용 가능
부분 사용 가능
사용 불가
```

- If path does not exist, skip or return unavailable without creating folder.
- Must explicitly verify missing test path is not created.

## 8. README.md Content

Create `README.md` with:

```markdown
# object_memory

Standalone Python skeleton for SLAM pose assisted SAM-6D object ID association.

This module is intentionally independent from SAM-6D, SLAM backends, Unity, ROS2
custom messages, launch files, and runtime nodes. Story 0 only establishes core
data contracts, transform utilities, lifecycle transitions, memory storage, and
SAM_circle ros2 bag usability checking.

## Quick Checks

```bash
python3 -m compileall src scripts_test
pytest scripts_test -q
python3 scripts_test/check_sam_circle_bag.py \
  --bag-path ~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle \
  --output-report reports/story0_sam_circle_bag_check.md
```

If the SAM_circle bag is unavailable, tests use only in-memory synthetic data.
No fake bag, dataset, image, depth, or SAM output folders are created.
```

## 9. Reports to Create

### 9.1 `reports/story0_sam_circle_bag_check.md`

This is generated by `scripts_test/check_sam_circle_bag.py`.

### 9.2 `reports/story0_object_memory_core_report.md`

Must summarize:

- implemented scope
- transform formula coverage
- validation commands
- validation results
- SAM_circle verdict
- created files
- acceptance criteria status
- next story order

### 9.3 `reports/story0-dev-story-result.md`

Must summarize:

- story title
- implementation location
- result summary
- SAM_circle verdict
- validation commands
- file list
- dev notes
- next recommended story order

## 10. Validation Commands

After reconstructing files, run:

```bash
cd ~/git_ws/object_memory
python3 -m compileall src scripts_test
pytest scripts_test -q
python3 scripts_test/check_sam_circle_bag.py \
  --bag-path ~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle \
  --output-report reports/story0_sam_circle_bag_check.md \
  --max-topic-samples 5
```

Expected:

- compileall passes.
- pytest passes.
- current baseline target: 21 tests pass.
- bag checker exits 0 when verdict is `사용 가능` or `부분 사용 가능`.
- bag checker may exit non-zero for `사용 불가`; that is acceptable only if the real bag is missing or broken and the reason is recorded.

## 11. Expected SAM_circle Result

When SAM_circle is present in the known current format, expected verdict is:

```text
부분 사용 가능
```

Expected topics:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/color/camera_info
/camera/camera/imu
```

Expected missing topics:

```text
/tf
/tf_static
SLAM pose topic
SAM-6D object output topic
```

Interpretation:

- Use SAM_circle for real RGB-D/IMU timestamp/source-data validation.
- Use in-memory synthetic `T_map_cam` and `T_cam_obj` only inside tests until Story 1 and Story 2 provide real pose contracts.

## 12. Meaning of In-Memory Synthetic Pose

This does not mean fake bag or fake dataset.

It means tests create temporary Python transform objects such as:

```python
T_map_cam = make_transform(
    ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    (10.0, 0.0, 0.0),
)
T_cam_obj = make_transform(
    ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    (0.5, 1.0, 2.0),
)
T_map_obj = compute_T_map_obj(T_map_cam, T_cam_obj)
```

These objects exist only in Python memory during unit tests.

## 13. Completion Criteria

The reconstructed baseline is complete when:

- `src/core` exists.
- `scripts_test` exists.
- `reports` exists.
- no `tools` folder is required.
- no `tests` folder is required.
- runtime code is not under `src/object_memory/core`.
- `python3 -m compileall src scripts_test` passes.
- `pytest scripts_test -q` passes.
- SAM_circle bag report is generated or the missing real bag is explicitly recorded.
- no fake data folders are created.

## 14. Next BMAD Story After Reconstruction

After this baseline is reconstructed, continue with:

1. Story 1: Common SLAM Pose Adapter
2. Story 2: SAM-6D Detection Output Contract
3. Story 3: Object Memory Schema and Lifecycle
4. Story 4: Short-term Association MVP
5. Story 5: Long-term Object Memory Association MVP
6. Story 6: Unity Object Update Contract
7. Story 7: Diagnostics and Debug Report

Do not start Story 1 until Story 0 baseline tests pass on the new PC.
