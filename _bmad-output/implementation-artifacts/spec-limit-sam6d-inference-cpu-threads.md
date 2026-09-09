---
title: 'Limit SAM-6D inference CPU threads'
type: 'chore'
created: '2026-09-04'
status: 'done'
baseline_commit: 'eed879d3196cf1f94c91d9595ca6a929c4d657a7'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** CUDA 기반 SAM-6D 추론 프로세스가 CPU 연산 라이브러리의 다중 스레드를 사용해 ORB-SLAM3 localization과 CPU를 경쟁한다. CPU 스레드를 줄이되 기존 약 0.5초 수준의 SAM-6D 전체 추론 시간과 검출·자세 결과를 악화시키면 안 된다.

**Approach:** SAM-6D 추론 자식 프로세스를 시작할 때만 CPU 연산 라이브러리 스레드 환경변수를 1로 설정한다. receiver, viewer, recorder, ObjectMemory, ORB-SLAM3 프로세스에는 이 제한을 전파하지 않는다.

## Boundaries & Constraints

**Always:** 동일한 녹화 RGB-D 입력에서 검출 객체 목록과 자세 결과가 유지되어야 한다. 대표적인 0개·1개·복수 객체/PEM 부하 프레임에서 평균 및 중앙값 추론 시간이 기준보다 늘어나면 적용하지 않는다. 기존 작업 트리 수정분을 보존한다.

**Ask First:** 대표 입력에서 출력 불일치나 추론 시간 회귀가 발생해 모델 또는 추론 구현을 바꿔야 하는 경우.

**Never:** ORB-SLAM3의 처리 프레임률을 낮추거나, SAM-6D 모델을 CPU로 이동하거나, 다른 프로세스의 스레드 수를 제한하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 일반 실시간 실행 | `sam6d_split.launch.py`가 추론 프로세스 시작 | 추론 프로세스만 CPU 스레드 1개 제한 | 기존 launch 실패 전파 유지 |
| 실제 객체 부하 | 검출 기록이 있는 24개 RGB-D 프레임 | 26개 검출·자세 동일, 평균/중앙값 비회귀 | 회귀 시 변경 적용 금지 |

</frozen-after-approval>

## Code Map

- `sam6d_ws/realtime/launch/sam6d_split.launch.py` -- receiver와 분리된 SAM-6D 추론 프로세스를 생성하는 공통 실시간 실행 경로.
- `output/260904_test_01/diagnostics/sam_thread_benchmark.py` -- 실제 녹화 데이터에서 스레드 제한 전후 출력 및 추론 시간을 비교하는 일회성 검증 도구.

## Tasks & Acceptance

**Execution:**
- [x] `sam6d_ws/realtime/launch/sam6d_split.launch.py` -- 추론 `ExecuteProcess`에만 CPU 연산 라이브러리 스레드 환경변수 1을 전달한다.
- [x] 실제 검출 녹화 데이터 A/B 결과와 launch 구문 검사를 실행한다.

**Acceptance Criteria:**
- Given 기본 실시간 명령, when SAM-6D 추론 프로세스가 시작되면, then OMP/MKL/OpenBLAS/NumExpr/OpenCV CPU 스레드 제한은 추론 프로세스에만 적용된다.
- Given 동일한 24개 대표 RGB-D 프레임, when 제한 전후 결과를 비교하면, then 검출 수·객체·자세가 동일하고 평균 및 중앙값 추론 시간이 증가하지 않는다.

## Spec Change Log

## Verification

**Commands:**
- `python3 -m py_compile sam6d_ws/realtime/launch/sam6d_split.launch.py` -- expected: 구문 오류 없음.
- `python3 output/260904_test_01/diagnostics/sam_thread_benchmark.py` (기본/제한 환경 A/B) -- expected: 출력 동일, 평균·중앙값 추론 시간 비회귀.

**Results:**
- 대표 24프레임: 검출 26개와 자세값 동일, 평균 299.5→247.4 ms, 중앙값 260.5→212.0 ms.
- 저장 depth가 있는 전체 584프레임: 최종 검출 및 객체 목록 386개 동일, 평균 215.1→177.7 ms, 중앙값 245.5→208.5 ms. 386개 자세 중 최대 수치 차이는 2.37 mm/2.67도였고 95백분위 이동 차이는 0 mm였다.
- launch AST 범위 검사 및 구문 검사 통과. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ... pytest -q tests/test_live_recording.py`: 4 passed.
- 45초 full-view 통합 재생: ORB 평균 29.6→24.0 ms, 중앙값 28.2→22.9 ms, p95 45.1→33.9 ms. 1,240회 추적 호출 중 pose 1,239개 출력. SAM 평균 72.8→57.2 ms, 처리 프레임 541→689.

## Suggested Review Order

- 추론 자식에만 네이티브 환경 제한을 적용해 다른 프로세스 CPU를 보존한다.
  [`sam6d_split.launch.py:74`](../../sam6d_ws/realtime/launch/sam6d_split.launch.py#L74)

- 실제 녹화 RGB-D로 출력과 단계별 시간을 동일 입력에서 비교한다.
  [`sam_thread_benchmark.py:1`](../../output/260904_test_01/diagnostics/sam_thread_benchmark.py#L1)
