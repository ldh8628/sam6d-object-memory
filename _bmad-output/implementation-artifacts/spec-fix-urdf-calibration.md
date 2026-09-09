---
title: 'URDF trajectory calibration false rejection 수정'
type: 'bugfix'
created: '2026-09-01'
status: 'done'
baseline_commit: 'NO_VCS'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `camera_extrinsic_localization.py`는 최종 외부 파라미터 품질을 병진 p90 10 cm·회전 p90 5°까지 허용하면서, 앞단 stable-transform cluster는 병진 5 cm·회전 3°로 더 엄격하게 잘라 정상 후보를 조기에 폐기한다. 실제 `260901_cbnu_eightcircle_SLAM`과 `260901_cbnu_bigeightcircle_SLAM`은 이 불일치 때문에 URDF가 새 측정값 대신 rig reference fallback으로 생성된다.

**Approach:** stable cluster의 기본 허용값을 이미 존재하는 최종 품질 기준과 일치시키고, 최종 p90 검증은 그대로 유지한다. 저장된 bigeight trajectory에서 새 기본값으로 재추정하여 `stable transform cluster` 기반 JSON/URDF가 생성되는지 검증한다.

## Boundaries & Constraints

**Always:** 최종 baseline·병진 p90·회전 p90 검증과 시간 분산 검증을 유지한다. 실제 추정 실패 시 rig reference fallback을 유지한다. CLI 사용자가 더 엄격한 stable gate를 지정할 수 있어야 한다. 변경 전후 수치를 기록한다.

**Ask First:** 기존 bag, Atlas, trajectory 또는 사용자 산출물을 삭제·덮어써야 하는 작업. 카메라 하드웨어를 다시 녹화해야 하는 작업.

**Never:** 품질 검증 제거, 결과 무조건 승인, reference 행렬을 새 추정값처럼 표기, 신규 의존성이나 별도 calibration 프레임워크 추가.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| bigeight 정상 후보 | 10 cm/5° 안에서 시간 전체에 분포한 transform cluster | 새 추정값으로 JSON/URDF 생성, source=`stable transform cluster` | 최종 p90와 baseline을 다시 검증 |
| 실제 불량 후보 | 최종 p90 또는 baseline 초과 | 새 추정값 거부 | rejection reason을 남기고 rig reference 사용 |
| 사용자 엄격 설정 | `--stable-translation`, `--stable-rotation` 명시 | 명시값을 기본값보다 우선 적용 | 양수가 아니면 기존 CLI 검증으로 거부 |

</frozen-after-approval>

## Code Map

- `integration/camera_extrinsic_localization.py` -- stable cluster 추정, 최종 품질 게이트, fallback 및 URDF 출력을 모두 담당하는 공통 진입점.
- `integration/create_map_urdf.py` -- 위 공통 진입점을 기본 인자로 호출하는 실시간 녹화 진입점; 변경 없이 새 기본값의 수혜를 받는다.
- `integration/camera_extrinsic_reference.json` -- 실제 추정이 실패할 때만 쓰는 검증된 fallback; 수정하지 않는다.

## Tasks & Acceptance

**Execution:**
- [x] `integration/camera_extrinsic_localization.py` -- stable cluster 기본값을 최종 허용 기준과 맞추고, 5–10 cm 분산 후보가 최종 품질 게이트를 통과하는 최소 회귀검사를 추가한다.
- [x] `output/260901_cbnu_bigeightcircle_SLAM/camera_extrinsic` -- 기존 Atlas와 trajectory를 보존한 채 재추정하여 JSON/URDF source와 품질 수치를 확인한다.

**Acceptance Criteria:**
- Given 저장된 bigeight SLAM/SAM trajectory, when 기본 인자로 외부 파라미터를 재추정하면, then 8개 이상의 시간 bin과 최종 p90 기준을 통과하고 `stable transform cluster` source의 URDF를 생성한다.
- Given 최종 허용 기준을 넘는 합성 trajectory, when 동일 추정·검증을 수행하면, then 기존 품질 게이트가 결과를 거부한다.
- Given 사용자가 stable gate CLI 값을 명시했을 때, when 파이프라인을 실행하면, then 명시한 값이 그대로 적용된다.

## Spec Change Log

## Verification

**Commands:**
- `python3 integration/camera_extrinsic_localization.py --self-test` -- 합성 정상·오염·최종 거부 회귀검사 통과.
- `python3 -m py_compile integration/camera_extrinsic_localization.py` -- 문법 검사 통과.
- `python3 integration/camera_extrinsic_localization.py --inputs:=output/260901_cbnu_bigeightcircle_SLAM/converted --outputs:=output/260901_cbnu_bigeightcircle_SLAM/camera_extrinsic` -- 기존 Atlas/trajectory 재사용 후 fallback 없이 측정 URDF 생성.

## Suggested Review Order

**판정 기준 정합**

- stable cluster 기본값을 기존 최종 품질 기준과 일치시킨 핵심 변경이다.
  [`camera_extrinsic_localization.py:450`](../../integration/camera_extrinsic_localization.py#L450)

- CLI 기본값도 Python API와 동일하게 유지한다.
  [`camera_extrinsic_localization.py:781`](../../integration/camera_extrinsic_localization.py#L781)

**재사용 안전성**

- 저장 결과가 현재 요청한 gate로 생성됐을 때만 재사용한다.
  [`camera_extrinsic_localization.py:252`](../../integration/camera_extrinsic_localization.py#L252)

- 엄격한 CLI 재실행이 완화 결과를 우회하지 못하게 한다.
  [`camera_extrinsic_localization.py:302`](../../integration/camera_extrinsic_localization.py#L302)

**회귀검사**

- 병진 5–10 cm 경계의 거부·승인과 추정 정확도를 검증한다.
  [`camera_extrinsic_localization.py:663`](../../integration/camera_extrinsic_localization.py#L663)

- 회전 3–5° 경계도 독립적으로 검증한다.
  [`camera_extrinsic_localization.py:682`](../../integration/camera_extrinsic_localization.py#L682)
