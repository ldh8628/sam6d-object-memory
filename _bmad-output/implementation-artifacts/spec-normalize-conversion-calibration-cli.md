---
title: '변환·카메라 외부 파라미터 CLI 통일과 진행 표시'
type: 'feature'
created: '2026-08-31'
status: 'done'
baseline_commit: 'NO_VCS'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 변환 기본 위치와 외부 파라미터 인자 형식이 후속 파이프라인과 다르다. ORB-SLAM3 stdout이 로그 파일로만 들어가 장시간 멈춘 것처럼 보인다.

**Approach:** 변환 기본 경로를 `output/<데이터명>/converted`로 바꾸고 외부 파라미터 CLI를 `--inputs:=...`, 선택적 `--outputs:=...`로 통일한다. ORB 실행은 예상 재생 시간 기반 진행 막대를 표시한다.

## Boundaries & Constraints

**Always:** 명시한 `--outputs`가 기본값보다 우선한다. 외부 파라미터 기본 출력은 `output/<데이터명>/camera_extrinsic`이다. 진행률은 추정치임을 표시하고 성공 시 100%, 실패 시 로그 tail을 보존한다. 기존 산출물은 이동·삭제하지 않는다.

**Ask First:** 기존 변환본이나 Atlas를 이동·삭제·강제 재생성해야 하는 경우.

**Never:** 새 의존성, ORB 로그 전체 복제, 품질 제한·timeout 약화.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 변환 기본값 | `--inputs:=data/<name>` | `output/<name>/converted` | 입력=출력 및 다른 원본 충돌은 기존처럼 거부 |
| 외부 파라미터 기본값 | `--inputs:=output/<name>/converted` | `output/<name>/camera_extrinsic` | `info.json`, `SLAM/`, `SAM/` 누락 시 실행 전 실패 |
| 출력 명시 | `--outputs:=/path/to/result` | 지정 경로 사용 | 빈 값 거부 |
| ORB 실행 | 유효 bag | map/localize 진행 막대와 최종 100% | 비정상 종료 시 로그 tail 포함 오류 |

</frozen-after-approval>

## Code Map

- `converting_launch/convert_all.py` -- 변환 결과 기본 경로와 충돌 방지의 정본.
- `integration/camera_extrinsic_localization.py` -- 외부 파라미터 CLI, 기본 출력, ORB 실행·timeout·진행 표시.
- `integration/run_object_memory.py` -- 공용 `run_launch()` 기존 호출자.
- `integration/README.md` -- 변환부터 Object Memory까지 복사·붙여넣기 명령의 정본.

## Tasks & Acceptance

**Execution:**
- [x] `converting_launch/convert_all.py` -- 기본 출력을 `ROOT/output/<inputs.name>/converted`로 변경하고 명시 출력·충돌 검사를 유지한다.
- [x] `integration/camera_extrinsic_localization.py` -- `:=`, `--inputs`, `--outputs`, 데이터명 기반 기본 출력과 표준 라이브러리 진행 막대를 구현한다.
- [x] `integration/README.md` -- 새 정식 명령과 기본 폴더 구조로 예제를 갱신한다.
- [x] 자체검사로 경로·빈 인자·진행 문자열·기존 호출 호환을 검증한다.

**Acceptance Criteria:**
- Given `data/260826_etri_eightcircle_SLAM`, when `convert_all.py`를 `--outputs` 없이 dry-run하면, then 목적지는 `output/260826_etri_eightcircle_SLAM/converted`다.
- Given `output/260826_etri_eightcircle_SLAM/converted`, when 외부 파라미터 실행기에 `--inputs:=`만 주면, then 결과 루트는 `output/260826_etri_eightcircle_SLAM/camera_extrinsic`다.
- Given 명시적 `--outputs:=X`, when 실행 경로를 계산하면, then X를 사용한다.
- Given ORB 단계가 실행 중이면, when terminal을 보면, then 단계명·추정 백분율·경과 시간과 종료 시 100%가 보인다.
- Given 기존 공용 호출, when 진행 인자를 생략하면, then 기존 의미가 유지된다.

## Spec Change Log

## Design Notes

`start_delay + duration/rate`를 예상 시간으로 쓰고 99%에서 대기한다. Atlas 저장 중에는 `마무리 중`, 성공 시 100%로 닫는다.

## Verification

**Commands:**
- `python3 -m py_compile converting_launch/convert_all.py integration/camera_extrinsic_localization.py integration/run_object_memory.py` -- 문법과 공용 함수 호출 호환.
- `python3 converting_launch/convert_all.py --inputs:=data/260826_etri_eightcircle_SLAM --stages:=clock,convert --dry-run` -- 새 기본 경로.
- `python3 integration/camera_extrinsic_localization.py --self-test` -- 인자 정규화·기본 출력·진행 문자열과 기존 수학 회귀검사.
- `python3 integration/camera_extrinsic_localization.py --help` -- 정식 `--inputs`, `--outputs` 인터페이스 노출.

## Suggested Review Order

**CLI와 기본 경로**

- 정식 입력·출력 인터페이스와 단일 데이터셋 실행 흐름이다.
  [`camera_extrinsic_localization.py:481`](../../integration/camera_extrinsic_localization.py#L481)

- 변환 결과를 데이터셋 폴더 아래에 일관되게 배치한다.
  [`convert_all.py:550`](../../converting_launch/convert_all.py#L550)

**경로 안전성**

- 기본 출력 계산과 입력 영역 겹침을 실행 전에 차단한다.
  [`camera_extrinsic_localization.py:82`](../../integration/camera_extrinsic_localization.py#L82)

- 심볼릭 링크까지 해석해 변환 원본 삭제 가능성을 막는다.
  [`convert_all.py:573`](../../converting_launch/convert_all.py#L573)

**진행 표시**

- 기존 timeout과 로그 tail을 유지하며 추정 진행 막대를 갱신한다.
  [`camera_extrinsic_localization.py:157`](../../integration/camera_extrinsic_localization.py#L157)

**사용법과 검사**

- 자체검사가 CLI 정규화·기본 경로·진행 문자열을 고정한다.
  [`camera_extrinsic_localization.py:438`](../../integration/camera_extrinsic_localization.py#L438)

- 전체 파이프라인 복사·붙여넣기 명령을 새 형식으로 맞춘다.
  [`README.md:168`](../../integration/README.md#L168)
