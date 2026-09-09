---
title: 'RealSense 2카메라 실시간 실행의 보기·녹화 옵션 분리'
type: 'feature'
created: '2026-09-04'
status: 'draft'
context:
  - 'implementation-artifacts/spec-live-rgbd-research-recording.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `integration/run_object_memory_realtime.py`는 `--view`만 제공하지만 생성하는 SAM-6D 설정에서는 `live_recording.enabled`를 항상 켠다. 그 결과 사용자는 화면 표시와 RGB/Depth 녹화를 독립적으로 선택할 수 없고, 보기만 원하거나 최소 부하 결과만 저장할 때도 영상 인코더가 실행된다.

**Approach:** 통합 실행기에 독립적인 `--view`와 `--record` 플래그를 제공한다. 두 플래그의 기본값은 모두 꺼짐이며, 아무 옵션이 없어도 기존의 경량 수치 결과와 최종 ObjectMemory는 저장한다.

## Boundaries & Constraints

**Always:** `--view`는 뷰어 프로세스만, `--record`는 기존 비차단 RGB/processed-Depth recorder만 제어한다. 두 옵션은 단독 또는 함께 사용할 수 있다. `object_memory.json`, `frames.jsonl`, `detections.jsonl`, `receiver.jsonl`, 실행 설정과 ORB 궤적은 녹화 여부와 무관하게 유지한다. 녹화는 기존 bounded queue와 실패 격리 계약을 그대로 사용한다.

**Ask First:** `--record-full-depth` 같은 추가 모드, 인코더/코덱 변경, 기존 결과 형식 또는 저장 경로 변경이 필요한 경우.

**Never:** `--view`가 암묵적으로 녹화를 켜거나 `--record`가 뷰어를 켜게 하지 않는다. ROS bag을 추가하지 않는다. 기존 SAM-6D 하위 실행기의 `--view`/`--record` 동작을 변경하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 경량 실행 | 옵션 없음 | 뷰어·영상 recorder 없이 JSON/ObjectMemory 결과 저장 | 기존 자식 프로세스 오류 처리 유지 |
| 보기 전용 | `--view` | 뷰어만 실행, RGB/Depth 영상 미생성 | 뷰어 수명주기 격리 유지 |
| 녹화 전용 | `--record` | 뷰어 없이 RGB와 processed Depth 저장 | recorder 실패는 추론과 격리 |
| 보기와 녹화 | `--view --record` | 두 독립 프로세스 모두 실행 | 한 reader 종료가 다른 경로를 중단하지 않음 |

</frozen-after-approval>

## Code Map

- `integration/run_object_memory_realtime.py` -- CLI 파싱, 실시간 SAM 설정 생성 및 네 프로세스 수명주기.
- `integration/README.md` -- RealSense 2카메라 실행 명령과 저장 결과 설명.
- `sam6d_ws/realtime/launch/sam6d_split.launch.py` -- 생성 설정의 `view`와 `live_recording.enabled`를 실제 독립 프로세스로 변환하는 기존 소비자.

## Tasks & Acceptance

**Execution:**
- [ ] `integration/run_object_memory_realtime.py` -- `--record`를 추가하고 생성 설정의 녹화 활성화를 해당 값으로 덮어써 보기와 녹화를 독립 제어한다.
- [ ] `integration/README.md` -- 네 가지 옵션 조합과 영상이 `--record`에서만 생성됨을 문서화한다.
- [ ] 최소 자체검증 -- 옵션 조합이 생성 설정에서 정확히 분리되는지 고정한다.

**Acceptance Criteria:**
- Given 옵션이 없는 실행, when 설정을 생성하면, then `output.view=false`이고 `output.live_recording.enabled=false`이다.
- Given 각 단독 또는 조합 플래그, when 설정을 생성하면, then 두 설정값이 전달된 불리언과 각각 일치한다.
- Given `--record`가 꺼진 실행, when 정상 종료하면, then 경량 JSON/ObjectMemory 결과 계약은 유지된다.

## Spec Change Log

## Verification

**Commands:**
- `python3 integration/run_object_memory_realtime.py --self-test` -- 기존 역할/URDF 검사와 새 옵션 조합 검사 통과.
- `python3 integration/run_object_memory_realtime.py --help` -- `--view`와 `--record`를 독립 옵션으로 표시.
- `python3 -m py_compile integration/run_object_memory_realtime.py` -- 문법 검사 통과.

