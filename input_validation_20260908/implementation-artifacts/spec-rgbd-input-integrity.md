---
title: 카메라 RGB-D 공통 입력과 무결성 검증
status: done
type: feature
created: 2026-09-07
baseline_commit: NO_VCS
context: []
---

<frozen-after-approval reason="사용자가 제공한 구현 계획에 의해 승인됨">

## Intent

별도 live/record 입력 경로와 무거운 수신 콜백을 native RealSense RGBD 공통 입력 및 순차 소비자로 변경한다. 두 카메라의 30 FPS 수신·녹화 완전성을 계측하고, 증거 부족을 성공으로 보고하지 않는다.

## Boundaries & Constraints

**Always:** /tmp 후보 소스에서 먼저 구현·검증. Fast DDS 명시, reliable depth 120, bounded FIFO 60. MCAP 무압축 4 MiB chunk, index/CRC, cache 512 MiB. RGB/depth frame number 및 timestamp 독립 식별. 준비 10초도 로그 보존. 종료 전 측정 창 닫기, 수신·소비·recorder drain 후 판정. 원본 bag 유지. 입력과 pose 정확도 별개 판정.

**Ask First:** 승인 범위를 벗어나는 production 배포 및 mask-tracker 변경.

**Never:** 큐 덮어쓰기, timestamp 수정, 누락 복제, 해상도/FPS 감소를 합격으로 취급, 하드웨어 미실행을 PASS 처리.

## I/O & Edge-Case Matrix

| 상황 | 입력 | 동작 | 판정 |
|---|---|---|---|
| 정상 | 연속 metadata/RGBD | 전 프레임 순서 보존 | 수치 및 소비·bag 일치 시 PASS |
| 누락/중복/역순 | 센서 번호 또는 stamp 이상 | 실제 번호/경계 기록 | FAIL |
| 센서 재시작 | 번호 감소/시각 초기화 | restart 구간 구분 | FAIL |
| metadata 미비 | 번호/stamp 연결 불가 | 추정하지 않음 | INCOMPLETE |
| 느린 소비자 | 지속 queue 지연 | 지연 통계·이벤트 기록 | FAIL |
| 큐 초과/작업 오류 | 입력 enqueue/consume 실패 | 실패 기록, 기존 큐 drain | FAIL |
| recorder 지연/종료 | tail 메시지 대기 | 창을 먼저 닫고 writer 종료 확인 | 미완료 INCOMPLETE |
| 기존 SQLite | 개별 영상/보정 | 기존 replay 호환 | 별도 호환 검사 |

</frozen-after-approval>

## Code Map

- `integration/camera_publish.py`: 역할/시리얼, profile, RMW, 토픽.
- `integration/run_object_memory_realtime.py`, `create_map_urdf.py`: 공통 녹화 및 진단 orchestration.
- `integration/run_object_memory_rosbag.py`: 새 combined/기존 split replay.
- `sam6d_ws/realtime/sam6d_receiver_node.py`: SAM 수신, pose matching, overlay, shared-memory 최신 추론 입력.
- `orbslam_ws/src/orbslam3_ros2/{src/rgbd_node.cpp,launch/orb_slam.launch.py,CMakeLists.txt,package.xml}`: ORB native 구독·순차 worker 및 진단.

## Tasks & Acceptance

**Execution:**
- [x] ORB wrapper: native RGBD opt-in, legacy 유지, single worker FIFO/drain, 이벤트 JSONL, 큐 실패 가시화.
- [x] SAM receiver: 콜백 enqueue, single consumer가 변환/pose 대기/overlay/기록 처리, 최신 추론 선택 집계.
- [x] 공통 `input_integrity.py`: bounded queue, identity/metadata 연결, 오류/간격/지연/window 판정 및 JSON 산출.
- [x] 공통 `rgbd_capture.py`: recorder 설정/준비/QoS/용량, MCAP→SQLite 무손실 변환, replay 내용 비교.
- [x] `input_check.py`: 두 카메라 진단, 준비/drain, 실행 설정/최종 요약. 두 실행기 연결.
- [x] 테스트: 연속·누락·중복·역순·재시작·metadata 누락·큐 초과·consumer 오류·recorder 지연·tail, 실제 ROS 타입/MCAP/SQLite 왕복.
- [x] 검증 보고서·재현 명령·네 실행 JSON·기준 측정 JSON. 실제 장치 부재 시 INCOMPLETE 근거 남김.

**Acceptance Criteria:**
- Given 준비된 두 카메라, when N초 검사, then 10초 준비 이후 창에서 sensor FPS 29.5–30.5, 입력/소비 100%, 순서/번호 오류 0을 검사한다.
- Given 녹화, when 종료 후 비교, then RGB/depth/CameraInfo 내용 및 순서를 보존하고 원본·bag·replay를 대조한다.
- Given 정상 실행, when 통계 확정, then callback p95≤2ms, queue p95≤10ms/max≤100ms/지속증가 없음, host 간격 p95≤50ms/max≤100ms, baseline 대비 FPS 감소≤1%를 각각 평가한다.
- Given 계측/하드웨어/기준 미비, when 최종 판정, then 근거와 INCOMPLETE를 기록하고 production 적용하지 않는다.

## Verification

단위 검사 및 설치된 Jazzy ROS 타입을 사용하는 합성 MCAP/SQLite 왕복. 격리된 ORB wrapper 빌드. 하드웨어 탐색 결과를 원문 보존. 카메라 전용 → 두 대 → record → ORB → SAM/PEM → viewer 부하 순서 및 120초 네 조건의 명령을 보고서에 기록한다.

## 카메라 한 대 시점의 과거 Acceptance status

후보 구현과 자동 검사는 완료했다. 하드웨어 전체 합격은 미완료: SAM 카메라 미인식, 단일 카메라 호스트 간격 p95 및 녹화 metadata 실패. 네 실행은 INCOMPLETE이며 production/pose 검증을 완료했다고 보고하지 않는다. 상세 결과는 [검증 보고서](../verification/REPORT.md)에 보존한다.

## Suggested Review Order

- 실행 진입점과 종료 순서
  [run_object_memory_realtime.py:168](../integration/run_object_memory_realtime.py#L168)

- 공통 준비·창 닫기·DDS drain·최종 판정
  [input_check.py:19](../integration/input_check.py#L19)

- 독립 센서 번호·stamp와 실패 판정
  [input_integrity.py:172](../integration/input_integrity.py#L172)

- 단일 소비자와 명시적 큐 실패
  [input_integrity.py:60](../integration/input_integrity.py#L60)

- ORB native 구독과 순차 처리
  [rgbd_node.cpp:358](../orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp#L358)

- SAM 종료 직전 DDS 메시지 보존
  [sam6d_receiver_node.py:257](../sam6d_ws/realtime/sam6d_receiver_node.py#L257)

- MCAP 원본 녹화와 SQLite 내용 검증
  [rgbd_capture.py:37](../integration/rgbd_capture.py#L37)

- Fast DDS publisher history
  [fastdds_input.xml:1](../integration/fastdds_input.xml#L1)

- 실패·경계 조건 회귀 검사
  [test_input_integrity.py:1](../integration/test_input_integrity.py#L1)

- 실제 ROS 타입의 native·SQLite 왕복
  [test_rgbd_capture_ros.py:1](../integration/test_rgbd_capture_ros.py#L1)


## 두 카메라 연결 후 최우선 입력 완료

네 120초 부하 조건과 실제 프로젝트 기본 촬영/300초 실시간 검증은 PASS. 이전 실패 자료는 보존했다. 적용 소스·재현 명령·확인한 원인·선택 map viewer 미확정 예외는 ../README.md 및 ../verification/REPORT_TWO_CAMERAS.md 에 정리했다.
