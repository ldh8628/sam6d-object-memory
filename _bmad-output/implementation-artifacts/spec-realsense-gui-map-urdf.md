---
title: 'RealSense GUI Publisher와 맵·URDF 생성'
type: 'feature'
created: '2026-08-31'
status: 'in-review'
baseline_commit: 'NO_VCS'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 두 D455f의 물리 역할을 사용자가 화면으로 확인할 수 없고, 실시간 실행기가 serial과 카메라 ROS 노드를 직접 관리해 캘리브레이션 당시 역할을 재사용한다는 보장이 없다. 신규 촬영에서 동기 녹화부터 Atlas·외부 파라미터 생성까지 한 번에 이어지는 진입점도 없다.

**Approach:** 시스템 Python의 V4L2/OpenCV/Tkinter preview에서 SLAM 카메라를 선택한 뒤에만 Jazzy RealSense publisher 두 개를 시작하는 공용 스크립트를 만든다. 최초 설정 스크립트는 그 publisher를 통해 한 bag을 녹화하고 기존 외부 파라미터 파이프라인을 호출하며, 실시간 스크립트도 같은 publisher와 저장된 역할 계약을 사용한다.

## Boundaries & Constraints

**Always:** 확인 전에는 ROS 카메라 프로세스를 시작하지 않는다. 서로 다른 serial 두 개만 허용한다. 녹화는 두 역할의 RGB, aligned depth, color/depth CameraInfo 토픽만 포함하고 원본 DB3를 보존한다. 기존 `camera_extrinsic_localization.py`의 Atlas·trajectory·SE(3) 품질 판정을 재사용한다. 모든 자식 프로세스는 취소·오류·Ctrl-C 때 종료한다.

**Ask First:** 기존 capture DB3 또는 검증된 Atlas/URDF를 삭제하거나 덮어써야 하는 변경.

**Never:** `pyrealsense2`, `sync_record_gui_v3`, 신규 의존성, 확인 전 ROS 노드 실행, 캘리브레이션 역할 불일치 상태의 실시간 실행.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| GUI 선택 | V4L2 RGB 장치 두 개 | SLAM 선택과 반대 장치를 SAM으로 저장 후 두 publisher 시작 | 취소·장치 부족·중복 serial이면 publisher 미실행 |
| 자동 선택 | 두 serial CLI 인자 | GUI 없이 동일한 역할 계약으로 publisher 시작 | 한 인자만 있거나 serial이 같으면 거부 |
| 최초 생성 | 새 이름과 정상 camera topics | 단일 capture, 역할 symlink, info.json, Atlas, JSON, URDF 생성 | 30초 미만 녹화 및 불완전 metadata 거부 |
| 실시간 실행 | 기존 map과 같은 역할 | publisher 뒤 ORB3, SAM-6D, ObjectMemory 실행 | 저장 역할과 다르면 즉시 종료, 역할 파일이 없으면 경고 후 실행 |

</frozen-after-approval>

## Code Map

- `integration/run_object_memory_realtime.py` -- 기존 카메라 launch와 실시간 ORB3/SAM-6D/ObjectMemory 수명주기.
- `integration/camera_extrinsic_localization.py` -- 검증된 bag replay, Atlas 생성, trajectory 정합, 품질 판정과 URDF 출력.
- `integration/run_object_memory_rosbag.py` -- 공용 conda 명령 및 프로세스 종료 도우미.
- `converting_launch/downgrade_bag_metadata.py` -- Jazzy v9 metadata를 Humble ORB3가 읽는 v5 view로 변환하는 기존 코드.

## Tasks & Acceptance

**Execution:**
- [x] `integration/camera_publish.py` -- V4L/udev 탐색, 두 화면 GUI, 역할 파일, 확인 후 Jazzy publisher 수명주기와 자체검사를 구현한다.
- [x] `integration/create_map_urdf.py` -- publisher 실행, topic 준비 확인, 단일 rosbag 녹화, info.json/역할 symlink 생성, 기존 외부 파라미터 파이프라인 호출과 자체검사를 구현한다.
- [x] `integration/run_object_memory_realtime.py` -- 직접 camera launch를 공용 publisher 자식 프로세스로 교체하고 저장된 역할을 검증한다.

**Acceptance Criteria:**
- Given GUI가 취소되거나 장치/serial 계약이 잘못됐을 때, when camera publisher를 실행하면, then RealSense ROS 프로세스는 하나도 시작되지 않는다.
- Given 확인된 두 카메라와 새 dataset 이름일 때, when 30초 이상 촬영을 마치면, then 동일 시각의 bag에서 Atlas와 외부 파라미터 JSON/URDF가 생성된다.
- Given 저장된 역할과 다른 물리 역할을 선택했을 때, when 실시간 실행을 시작하면, then ORB3/SAM-6D/ObjectMemory 시작 전에 거부된다.

## Spec Change Log

## Verification

**Commands:**
- `python3 integration/camera_publish.py --self-test` -- V4L/udev parsing, 역할 계약, topic/launch 명령 검사 통과.
- `python3 integration/create_map_urdf.py --self-test` -- 임시 Jazzy metadata와 CameraInfo에서 info.json 계약 검사 통과.
- `python3 integration/camera_extrinsic_localization.py --self-test` -- 기존 외부 파라미터 회귀검사 통과.
- `python3 integration/run_object_memory_realtime.py --self-test` -- 역할 검사와 실시간 설정 회귀검사 통과.
- `python3 -m py_compile integration/camera_publish.py integration/create_map_urdf.py integration/run_object_memory_realtime.py` -- 문법 검사 통과.
