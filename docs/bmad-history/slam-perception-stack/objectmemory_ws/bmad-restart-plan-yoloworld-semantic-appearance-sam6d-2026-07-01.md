# BMAD Restart Plan: YOLO-World + Semantic + Appearance Improved SAM-6D to Object Memory MVP

Date: 2026-07-01

## 1. Purpose

이 문서는 다른 PC에서 처음부터 개발을 다시 시작할 때 따라야 하는 BMAD 기준 재개 보고서이다.

현재 가정은 다음이다.

- SAM-6D는 이미 YOLO-World + semantic + appearance 기반으로 ISM proposal/selection만 개선되어 있다.
- SAM-6D 내부 inference algorithm을 더 수정하지 않는다.
- 개선된 SAM-6D도 여전히 frame-local object pose hypothesis generator이다.
- persistent `object_id`, short-term tracking, long-term object memory, Unity duplicate prevention은 별도 `object_memory` 모듈에서 구현해야 한다.

핵심 목표는 다음 수식을 중심으로 object identity layer를 구현하는 것이다.

```text
T_map_obj(t) = T_map_cam(t) * T_cam_obj(t)
```

## 2. Current Baseline to Reproduce

현재 PC에서 만든 최소 baseline은 `~/git_ws/object_memory/`이다.

현재 구조:

```text
object_memory/
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

현재 구현된 기능:

- transform utility
- core data model
- object lifecycle enum
- state transition skeleton
- in-memory `ObjectMemoryStore`
- association input/output contract
- SAM_circle bag usability checker
- pytest unit tests

현재 검증 결과:

```bash
cd ~/git_ws/object_memory
python3 -m compileall src scripts_test
pytest scripts_test -q
python3 scripts_test/check_sam_circle_bag.py \
  --bag-path ~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle \
  --output-report reports/story0_sam_circle_bag_check.md \
  --max-topic-samples 5
```

기대 결과:

- compileall 통과
- pytest 21 tests 통과
- SAM_circle 판정: `부분 사용 가능`

## 3. Important Interpretation of the SAM-6D Assumption

YOLO-World + semantic + appearance로 ISM이 개선된 SAM-6D라고 해도, 이 개선은 object proposal/selection 또는 pose hypothesis 품질을 높이는 것이다.

이 개선만으로 해결되지 않는 것:

- frame 간 persistent `object_id`
- detection_id와 object_id 분리
- short-term ego-motion association
- long-term map-memory association
- missed detection 처리
- Unity duplicate object creation 방지
- SLAM tracking lost 상태에서 memory update 차단
- loop closure/map jump 이후 동일 object_id 유지

따라서 새 PC에서도 SAM-6D를 persistent memory owner로 만들지 않는다. SAM-6D는 다음을 출력하는 producer로만 둔다.

```text
Sam6DDetection
  stamp
  frame_id
  detection_id
  object_name or class_id
  T_cam_obj
  score
  bbox optional
  depth_quality optional
  model_id optional
  size_extent optional
```

`object_memory`가 persistent `object_id`의 유일한 owner가 되어야 한다.

## 4. Data to Prepare on the New PC

새 PC에는 최소 다음 데이터를 준비한다.

```text
~/ros2_bag_recording/output/rgbd_imu_sdk/
  SAM_circle/
  SAM_loop1/
  SAM_loop2/
  SAM_occlusion/
  SLAM_short_loop/
  SLAM_long_loop/
  SLAM_wide_loop/
```

현재 확인된 bag 형태:

```text
<bag_name>/bag/metadata.yaml
<bag_name>/bag/bag_0.db3
```

SAM_circle 현재 의미:

- RGB/depth/camera_info/IMU timestamp 검증용으로 사용 가능.
- SLAM pose topic 없음.
- SAM-6D object pose output topic 없음.
- 따라서 Story 0 core test에서는 `T_map_cam`, `T_cam_obj`를 파일로 만들지 않고 Python test 내부 in-memory transform으로 검증한다.

주의:

- fake bag folder 생성 금지.
- fake image/depth dataset 생성 금지.
- fake SAM output folder 생성 금지.
- 필요한 synthetic pose는 pytest 코드 내부 object로만 만든다.

## 5. New PC Setup Checklist

### 5.1 System Prerequisites

필수:

- Ubuntu 또는 ROS2 개발 가능한 Linux 환경
- Python 3.10+
- git
- pytest

권장:

- ROS2 Humble 또는 현재 bag을 읽을 수 있는 ROS2 버전
- `ros2 bag info` 사용 가능 환경
- sqlite3

최소 Python 검증:

```bash
python3 --version
python3 -m pip --version
python3 -m pip install --user -U pytest
pytest --version
```

### 5.2 Workspace Layout

새 PC의 권장 layout:

```text
~/git_ws/
  object_memory/
  sam6d-object-memory/        # improved SAM-6D repo, if needed
  orbslam_ws/                 # later Story 1 candidate
  rtabmap_ws/                 # later Story 1 candidate
~/ros2_bag_recording/output/rgbd_imu_sdk/
  SAM_circle/
  SAM_loop1/
  SAM_loop2/
  SAM_occlusion/
  SLAM_short_loop/
  SLAM_long_loop/
  SLAM_wide_loop/
```

## 6. First Commands on the New PC

```bash
cd ~/git_ws/object_memory
python3 -m compileall src scripts_test
pytest scripts_test -q
python3 scripts_test/check_sam_circle_bag.py \
  --bag-path ~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle \
  --output-report reports/story0_sam_circle_bag_check.md \
  --max-topic-samples 5
```

If `ros2 bag info` fails but metadata/sqlite fallback works, continue. This is acceptable for Story 0.

If SAM_circle is missing, do not create a fake replacement. Restore/copy the real bag, or let the bag contract test skip.

## 7. Architecture Invariants to Preserve

INV-001. SAM-6D remains a frame-local pose hypothesis generator.

INV-002. Object Memory owns persistent `object_id`.

INV-003. `detection_id` is never used as persistent identity.

INV-004. Object pose in map frame is always computed as:

```text
T_map_obj = T_map_cam * T_cam_obj
```

INV-005. Short-term association and long-term association are separate policies.

INV-006. SLAM backend differences are hidden behind a common timestamped camera pose interface.

INV-007. Unity must consume `object_id`, not SAM-6D `detection_id`.

INV-008. Low-confidence detection and SLAM tracking lost must not corrupt authoritative object memory.

## 8. Recommended BMAD Story Sequence from Scratch

### Story 0: Independent object_memory module and bag usability check

Status on current PC: complete.

On new PC, first reproduce this story by running tests and bag checker. Do not start Story 1 until Story 0 passes or the missing data reason is explicit.

Acceptance checkpoint:

- `src/core` imports work.
- transform tests pass.
- state machine tests pass.
- memory store tests pass.
- SAM_circle bag checker returns `사용 가능`, `부분 사용 가능`, or a clearly justified `사용 불가`.

### Story 1: Common SLAM Pose Adapter

Goal:

- Normalize ORB-SLAM3, ORB-SLAM3+IMU, or RTAB-Map output into a common `SlamCameraPose` contract.

Inputs:

- SLAM pose or TF.
- timestamp.
- frame id.
- tracking status.
- optional confidence/map_version.

Output:

```text
SlamCameraPose
  stamp
  frame_id
  child_frame_id
  T_map_cam
  tracking_status
  source_slam_id
  confidence optional
  map_version optional
```

First implementation recommendation:

- Pick exactly one SLAM backend first.
- If bag has no SLAM pose, build adapter tests with in-memory pose samples first.
- Add real bag/ROS2 integration only after contract tests pass.

### Story 2: SAM-6D Detection Output Contract

Goal:

- Define the detection contract that improved SAM-6D must provide to object_memory.

Important assumption:

- YOLO-World + semantic + appearance improved ISM may improve `score`, `object_name`, proposal quality, and pose quality.
- It still must expose `T_cam_obj`, `detection_id`, timestamp, frame_id, object metadata.

Required output:

```text
Sam6DDetection
  stamp
  frame_id
  detection_id
  object_name or class_id
  T_cam_obj
  score
  bbox optional
  depth_quality optional
  model_id optional
  size_extent optional
```

Do not create ROS2 custom messages until this contract is stable in Python tests.

### Story 3: Object Memory Schema and Lifecycle

Goal:

- Extend Story 0 skeleton into a stable MVP memory record.

Implement:

- object confidence update policy.
- status transition reason.
- missed count policy.
- last seen / first seen handling.
- source SLAM status tracking.

Do not implement matching algorithms yet except minimal update helpers.

### Story 4: Short-term Association MVP

Goal:

- Keep the same `object_id` across adjacent SAM-6D pose estimation frames.

Required formula:

```text
T_cam_curr_obj_pred = inverse(T_map_cam_curr) * T_map_obj_prev
```

Signals:

- object_name/class gate.
- translation residual.
- rotation residual.
- SAM-6D score.
- optional bbox/depth consistency.

Expected output:

- `short_term_match`
- `new_tentative`
- `rejected`

### Story 5: Long-term Object Memory Association MVP

Goal:

- Reuse remembered/lost `object_id` after an object disappears and reappears.

Signals:

- class/object_name coarse filtering.
- map position radius search.
- score margin for ambiguity.
- memory confidence/history.

Policy:

- unique confident match only.
- ambiguous match must not create permanent association.

### Story 6: Unity Object Update Contract

Goal:

- Define event contract so Unity does not create duplicate objects.

Events:

```text
create
update
hide
delete
merge
state_change
```

Rule:

- Unity dictionary key must be `object_id`.
- SAM-6D `detection_id` must never be used as Unity persistent identity.

### Story 7: Diagnostics and Debug Report

Goal:

- Make association decisions auditable.

Required debug fields:

- detection_count
- matched_pairs
- rejected_pairs
- reject_reason
- score_components
- selected object_id
- decision_source
- TF lookup status
- timestamp delta
- SLAM status
- state transition
- Unity event type

## 9. What Not to Do on the New PC

Do not:

- Put persistent memory inside SAM-6D.
- Treat YOLO-World/semantic/appearance output as persistent identity.
- Use SAM-6D `detection_id` as `object_id`.
- Start with Unity implementation.
- Start with ROS2 custom msg files before Python contracts stabilize.
- Create fake bag folders.
- Create fake RGB/depth datasets.
- Create fake SAM output folders.
- Implement DINOv2/CLIP re-ID as MVP.
- Implement full object pose graph optimization as MVP.
- Implement moving object full support as MVP.

## 10. Development Rules for the New PC

Rule 1: Every story starts with tests.

Rule 2: Use real bag metadata when available; use in-memory synthetic transforms only for missing pose contracts.

Rule 3: Keep `src/core` free of ROS2 runtime dependencies until the core logic is stable.

Rule 4: Put scripts/checkers/tests under `scripts_test`, not runtime package folders.

Rule 5: Keep reports under `reports`.

Rule 6: Keep SAM-6D, SLAM, Unity integration as adapters around `object_memory`, not inside core logic.

## 11. Minimum Handoff Package

To move to another PC, copy or commit:

```text
~/git_ws/object_memory/
```

and real bag data:

```text
~/ros2_bag_recording/output/rgbd_imu_sdk/
```

If available, also bring:

```text
~/git_ws/sam6d-object-memory/
~/git_ws/orbslam_ws/
~/git_ws/rtabmap_ws/
```

But the first reproducibility gate is only `object_memory` plus bag metadata.

## 12. First Success Definition on the New PC

The restart is successful when the new PC can run:

```bash
cd ~/git_ws/object_memory
python3 -m compileall src scripts_test
pytest scripts_test -q
python3 scripts_test/check_sam_circle_bag.py \
  --bag-path ~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle \
  --output-report reports/story0_sam_circle_bag_check.md \
  --max-topic-samples 5
```

and produce:

- all tests passing.
- no fake data folders.
- SAM_circle verdict recorded.
- clear next story selection.

## 13. Immediate Next Recommendation

Start with Story 1 only after reproducing Story 0 on the new PC.

Recommended Story 1 target:

- If ORB-SLAM3 RGB-D pose/status is already easier to run, start with ORB-SLAM3.
- If RTAB-Map gives cleaner ROS TF/status on the new PC, start with RTAB-Map.
- Do not implement both in the same first story.

The output of Story 1 should be a Python-level `SlamCameraPose` adapter contract and tests first, then ROS2 integration second.
