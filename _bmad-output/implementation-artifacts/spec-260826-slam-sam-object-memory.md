---
title: '260826 SLAM 기반 SAM-6D 객체 메모리와 전 프레임 오버레이'
type: 'feature'
created: '2026-08-29'
status: 'done'
baseline_commit: 'NO_VCS'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 260826 SLAM/SAM 데이터의 지도 생성, 카메라 외부파라미터, SAM-6D 포즈의 지도 등록과 안정화 산출물은 존재하지만, SAM-6D가 처리하지 않은 프레임에서도 등록된 객체가 현재 SAM 카메라에 어떻게 보여야 하는지 확인할 수 있는 연속 오버레이가 없다.

**Approach:** 기존 ORB3 지도·URDF·SAM-6D 결과와 ObjectMemory 융합을 그대로 재사용한다. 각 입력 프레임의 SLAM 카메라 포즈에서 활성/장기기억 객체의 융합 지도 포즈를 카메라 좌표로 역투영하고, 기존 Explorer에서 검출 오버레이와 구분해 연속 표시한다.

## Boundaries & Constraints

**Always:** `T_map_obj = T_map_camSLAM(t) * X_camSLAM_camSAM * T_camSAM_obj` 좌표 규약을 유지한다. 유효한 동시간대 SLAM 포즈가 없으면 예측 포즈를 만들지 않는다. 기존 20개 관측 중 16개 수렴 등록과 5개 중 4개 이상치 해제, 대칭축 처리, 다중 인스턴스 ObjectMemory를 유지한다. 260826 네 조건의 기존 산출물을 훼손하지 않고 재생성 가능해야 한다.

**Ask First:** 원본 rosbag, ORB Atlas, SAM-6D 원본 추론 결과를 삭제하거나 덮어써야 하는 변경.

**Never:** 새 의존성·새 서비스·별도 상태 저장소를 추가하지 않는다. 유효하지 않은 SLAM 추적 구간에서 마지막 카메라 포즈를 임의 재사용하지 않는다. 원시 SAM-6D 포즈를 지도 융합 포즈로 가장하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 정상 전 프레임 | SLAM 포즈와 active/remembered 지도 객체 존재 | 각 객체의 `T_cam_obj` 예측을 타임라인에 기록하고 검출 없는 프레임에도 축 오버레이 | N/A |
| 추적 공백 | 허용 오차 내 SLAM 포즈 없음 | 해당 프레임 예측 배열은 비우고 기존 지도 메모리는 유지 | 잘못된 포즈를 합성하지 않음 |
| 미확정 객체 | tentative/lost/deleted 상태 | 연속 예측 대상에서 제외 | 검출/수렴 데이터는 기존 진단에 유지 |
| 카메라 뒤 객체 | 예측 원점의 z가 0 이하 | 화면 오버레이 생략 | Canvas 투영 단계에서 안전하게 제외 |

</frozen-after-approval>

## Code Map

- `integration/run_260826_object_memory.py` -- 4개 데이터셋 검증, ORB3 only-localization, URDF 적용, ObjectMemory 융합과 Explorer 타임라인 생성.
- `integration/object_memory_explorer/app.js` -- 원본/융합 포즈, 지도, 수렴 그래프를 재생하는 기존 UI.
- `sam6d_ws/realtime/slam_pose_memory.py` -- 20/16 등록, 5/4 해제, 대칭 포즈 거리와 지도 앵커 투영.
- `objectmemory_ws/object_memory/src/pipeline/object_memory_runner.py` -- 지도 좌표 다중 인스턴스 연관·융합·상태 전이의 공통 구현.
- `integration/serve_object_memory_explorer.py` -- 타임라인 프레임 API와 비디오 Range 응답.
- `sam6d_ws/realtime/sam6d_infer.py`, `sam6d_ws/realtime/sam6d_receiver_node.py` -- 추론 프로세스의 지도 앵커 전달과 전 입력 프레임 ROS 오버레이 발행.

## Tasks & Acceptance

**Execution:**
- [x] `integration/run_260826_object_memory.py` -- 모든 유효 프레임에 active/remembered 융합 객체의 카메라 상대 예측 포즈를 직렬화하고 자체검사 추가.
- [x] `integration/object_memory_explorer/app.js` -- 검출이 없는 프레임에도 예측 포즈 축과 객체 ID/상태를 별도 색상으로 표시.
- [x] `sam6d_ws/realtime/slam_pose_memory.py`, `sam6d_ws/realtime/run_*split.yaml` -- 실데이터 재생으로 회전 flip 유지와 실제 위치 이동 해제 임계값 보정.
- [x] `integration/serve_object_memory_explorer.py` -- 전체 프레임 객체 보관 대신 JSONL 오프셋 인덱스로 실시간 조회 메모리 절감.
- [x] `sam6d_ws/realtime/sam6d_infer.py`, `sam6d_ws/realtime/sam6d_receiver_node.py` -- 최신 지도 앵커를 공유메모리로 전달해 `/sam6d/overlay`를 모든 입력 프레임에 발행.
- [x] `integration/output/**` -- 4개 260826 조건을 재처리해 일관된 요약과 타임라인 생성.

**Acceptance Criteria:**
- Given 등록 객체와 동기화된 SLAM 포즈가 있을 때, when 연속 두 입력 프레임 중 하나에 SAM-6D 검출이 없으면, then 두 프레임 모두 지도 객체의 카메라 상대 예측 포즈를 가진다.
- Given SLAM 포즈가 없는 프레임일 때, when 타임라인을 생성하면, then 예측 포즈는 비어 있고 지도 객체 상태는 손상되지 않는다.
- Given 4개 데이터셋의 기존 SAM-6D와 ORB3 산출물일 때, when 통합 스크립트를 실행하면, then 입력 무결성 검사·자체검사·밝음/어두움 위치 일관성 결과를 생성한다.

## Spec Change Log

## Verification

**Commands:**
- `python3 integration/run_260826_object_memory.py --self-test` -- 좌표변환·수렴·예측 직렬화 자체검사 통과.
- `python3 -m pytest -q objectmemory_ws/object_memory/scripts_test` -- 기존 ObjectMemory 회귀검사 통과.
- `python3 integration/run_260826_object_memory.py` -- 4개 데이터셋 재처리 및 요약 생성.
- `python3 -m py_compile integration/run_260826_object_memory.py` -- 변경 스크립트 문법 검사 통과.
- `PYTHONPATH=sam6d_ws python3 -m pytest -q sam6d_ws/tests/test_slam_pose_memory.py` -- 회전 flip/이동 해제 회귀검사 통과.
- `python3 integration/serve_object_memory_explorer.py --self-test` -- 오프셋 조회·예측 필드·Range/경로 제한 검사 통과.

## Suggested Review Order

**실시간 데이터 흐름**

- 추론 사이 모든 입력 프레임을 순서대로 SLAM 포즈와 결합합니다.
  [`sam6d_receiver_node.py:213`](../../sam6d_ws/realtime/sam6d_receiver_node.py#L213)

- 최신 지도 앵커를 구독 시에만 0.65 ms 축 오버레이로 발행합니다.
  [`sam6d_receiver_node.py:270`](../../sam6d_ws/realtime/sam6d_receiver_node.py#L270)

- 결과에 map_id와 지도 앵커를 실어 지도 전환 오염을 막습니다.
  [`sam6d_infer.py:198`](../../sam6d_ws/realtime/sam6d_infer.py#L198)

- 실제 ORB 상태 문자열과 내부 상태 문자열을 한곳에서 정규화합니다.
  [`slam_pose_memory.py:17`](../../sam6d_ws/realtime/slam_pose_memory.py#L17)

**오프라인 융합과 탐색기**

- 확정 지도 객체를 매 SAM 프레임의 현재 카메라 좌표로 예측합니다.
  [`run_260826_object_memory.py:200`](../../integration/run_260826_object_memory.py#L200)

- 전체 타임라인에 예측과 명시적 추적 공백 정책을 직렬화합니다.
  [`run_260826_object_memory.py:253`](../../integration/run_260826_object_memory.py#L253)

- 검출 없는 프레임은 MAP 라벨 축으로 기존 융합 포즈와 구분합니다.
  [`app.js:33`](../../integration/object_memory_explorer/app.js#L33)

- JSONL 바이트 오프셋으로 서버 메모리를 498 MB에서 50 MB로 줄입니다.
  [`serve_object_memory_explorer.py:52`](../../integration/serve_object_memory_explorer.py#L52)

**안정화와 검증**

- 회전 flip은 유지하고 150 mm 지속 이동만 앵커를 재등록합니다.
  [`slam_pose_memory.py:106`](../../sam6d_ws/realtime/slam_pose_memory.py#L106)

- 회전 flip·투영·ORB 상태 호환을 단일 회귀 파일로 확인합니다.
  [`test_slam_pose_memory.py:30`](../../sam6d_ws/tests/test_slam_pose_memory.py#L30)
