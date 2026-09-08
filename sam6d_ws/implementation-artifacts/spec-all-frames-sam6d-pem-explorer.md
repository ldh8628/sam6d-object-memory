---
title: '전체 프레임 SAM-6D PEM Explorer 결과 생성'
type: 'feature'
created: '2026-08-26'
status: 'in-review'
baseline_commit: '1c1c8b29cce82a7219f0032d145ebf63725b871d'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 실시간 split 경로는 최신 프레임만 유지해 두 260826 RGB-D bag의 모든 프레임을 처리할 수 없다. 프레임 누락 없는 SAM-6D Explorer 결과가 각 데이터셋에 필요하다.

**Approach:** 기존 no-drop 순차 처리기 `tools/capture_pem_explorer.py`를 단일 자식으로 실행하는 최소 ROS launch 래퍼를 추가하고, 독립 복사·검증된 두 bag을 exhaustive profile로 GPU 충돌 없이 순차 처리한다.

## Boundaries & Constraints

**Always:** launch는 필수 `config` 하나만 받아 순차 캡처 프로세스 하나만 실행하고 자식 실패를 nonzero로 전파한다. 입력은 외부 `SAM/`을 reflink 가능 독립 복사하며 각 복사본에 검증된 `ground_truth_disabled` manifest를 둔다. 기존 출력이나 중단 결과는 덮어쓰지 않는다. 두 작업은 순차 실행한다.

**Ask First:** 원본 bag 검증값이 계획의 3081/3101과 다르거나, 출력 경로가 이미 존재하거나, 기존 미커밋 파일과 충돌하는 수정이 필요할 때.

**Never:** 실시간 split, receiver/shared memory, `ros2 bag play`, 재생 속도 변경, symlink/hardlink, pseudo-GT, YAML 스키마나 공개 타입 변경, 기존 미커밋 변경 덮어쓰기.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 정상 실행 | 유효한 full config와 manifest bag | 모든 bag 프레임을 정확히 한 번 처리하고 완료 manifest/preview 생성 | N/A |
| 자식 실패 | 잘못된 config 또는 캡처 오류 | launch도 nonzero 종료 | 실패 출력 보존, 덮어쓰기 금지 |
| 기존 출력 | 대상 디렉터리 존재 | 새 실행 시작 안 함 | 명시적 오류로 nonzero 종료 |

</frozen-after-approval>

## Code Map

- `tools/capture_pem_explorer.py` -- 재사용할 기존 no-drop SQLite 순차 처리기.
- `realtime/launch/sam6d_realtime.launch.py` -- ExecuteProcess/launch 인자 관례.
- `realtime/run_longcircle2_sam_pem_explorer_full.yaml` -- 복제할 exhaustive full 설정.
- `tools/validate_rgbd_dataset.py` -- manifest 스키마와 정확한 네 토픽 검증기.
- `tests/test_pem_explorer_v2.py` -- launch 회귀 테스트 위치.

## Tasks & Acceptance

**Execution:**
- [x] `realtime/launch/all_frames_sam6d.launch.py` -- config를 순차 캡처 한 프로세스에 전달하고 실패를 전파하는 launch 추가.
- [x] `realtime/run_260826_*_pem_explorer_full.yaml` -- 두 입력/출력용 exhaustive 설정 추가.
- [x] `data/260826_{differentObject,etri_similarObject}` -- 외부 bag 독립 복사 및 정확한 manifest 추가.
- [x] `tests/test_pem_explorer_v2.py` -- 단일 캡처 명령과 nonzero 전파 계약 회귀 테스트 추가.
- [x] 두 launch를 순차 실행하고 결과/preview/Explorer 검색을 검증.

**Acceptance Criteria:**
- Given 검증된 두 입력, when `--require-manifest` 검증을 실행하면, then 네 표준 토픽의 count와 timestamp 집합이 각각 완전히 일치한다.
- Given 각 full config, when 공통 launch를 실행하면, then differentObject는 3081, similarObject는 3101 프레임을 누락 없이 처리한다.
- Given 완료 결과, when manifest와 preview를 검사하면, then `completed:true`, `capture_profile:exhaustive_visualization`, 정확한 첫/마지막 timestamp 및 동일 preview 프레임 수를 갖는다.
- Given `serve_pem_explorer.py --no-model`, when output root를 스모크 검사하면, then 두 결과가 `explorer_v2`로 검색된다.

## Spec Change Log

## Verification

**Commands:**
- `python3 tools/validate_rgbd_dataset.py data/<dataset> --require-manifest` -- 각 bag 3081/3101 프레임과 정확한 경계 timestamp.
- `pytest -q tests/test_pem_explorer_v2.py tests/test_capture_pem_explorer.py` -- launch/capture 회귀 통과.
- `ros2 launch realtime/launch/all_frames_sam6d.launch.py config:=<config>` -- 두 작업을 순차 완주.
- `ffprobe` 및 manifest JSON 검사 -- preview/frame count와 완료 계약 일치.
- `python3 tools/serve_pem_explorer.py --root output --host 127.0.0.1 --port 8765 --no-model` -- 두 run 검색 스모크 통과.
