---
title: 'Bear 후보 증거 복구 및 Shadow 대체 후보 검증'
type: 'feature'
created: '2026-08-23'
status: 'done'
baseline_commit: 'NO_VCS'
context:
  - 'implementation-artifacts/spec-reusable-pem-pose-explorer.md'
  - 'implementation-artifacts/spec-pem-size-iou-candidate-filter.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** PEM Explorer가 detection당 상위 100개 후보를 보존하면서도 기본 `render-topn=10` 때문에 Bear `rank_geo=42`를 포함한 후순위 후보의 시각 근거를 제공하지 못한다. 기존 Shadow 보류 정책도 Top-1을 보류할 뿐, 중복 pose 군집을 제거한 독립 대체 후보의 가능성을 평가하지 않는다.

**Approach:** 재사용 builder가 상위 100개 후보의 pose·geometry·texture·mask 증거를 안전하게 게시하도록 확장하고, Tune에서만 선택한 pose 군집 임계값으로 원래 기하 순서를 보존하는 Shadow 대체 후보를 Holdout에 평가한다.

## Boundaries & Constraints

**Always:** `rank_geo`가 후보의 정체성과 UI 이동 기준이다. Top-1이 기존 texture/Mask IoU AND 조건으로 보류될 때만 대체를 탐색한다. Top-1과 symmetry-aware 회전 거리 및 이동 거리가 모두 임계값 이내인 후보는 같은 군집으로 제외한다. 남은 후보 중 texture `≥0.449562`, Mask IoU `≥0.420998`를 모두 만족하는 가장 높은 원래 기하 순위를 선택한다. Tune은 회전 `[10,15,20,30]°`, 이동 `[25,50,100]mm`만 평가하고 정확도 90% 이상, 정답 Top-1의 오답 대체 0건을 만족하는 최대 coverage 조합을 고른 뒤 Holdout에 고정한다. 실제 PEM pose는 절대 변경하지 않는다. 게시 전 50GiB 여유 공간을 확인하고 임시 디렉터리 완성 후 원자 교체한다.

**Ask First:** Shadow 후보를 실제 추론 pose로 승격하거나 임계값·분할·정답 정의를 변경하는 경우.

**Never:** 결합점수·재가중치로 후보를 선택하거나 Holdout 결과로 Tune 임계값을 변경하지 않는다. projection 불가를 빈 이미지 또는 정상 증거로 표시하지 않는다. detection별 입력 mask 이미지를 후보마다 중복 저장하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|---------------------------|----------------|
| 전체 증거 | 상위 100 후보와 pointwise/shape 입력 | 모든 `rank_geo`의 링크와 존재하는 전체 증거 표시 | 50GiB 미만이면 게시 전 중단 |
| projection 불가 | 비정상 R/t, z≤0, 빈 투영 | 후보와 사유를 명시하고 나머지 증거 유지 | 빈 화면 금지 |
| Shadow 대체 | Top-1 보류, 유사 군집 밖 통과 후보 존재 | 원래 기하 최상위 후보와 제외 군집 크기 기록 | 실제 pose 불변 |
| 대체 없음 | 전부 유사하거나 채널 미통과 | 구체적 사유 표시 | 후보를 임의 승격하지 않음 |

</frozen-after-approval>

## Code Map

- `_bmad/scripts/resolve_customization.py`, `_bmad/bmm/config.yaml` -- 프로젝트 BMAD 실행 복구와 Python 3.10 호환 설정.
- `tools/build_pem_verification_policy.py` -- Tune 임계값 선택, pose 군집 제거, Holdout Shadow 결과 생성.
- `tools/build_pem_explorer.py` -- 100개 증거, 공유 mask, 용량 preflight 및 정책/report 결합.
- `tools/pem_explorer/app.js`, `app.css` -- 보류 Top-1·제외 군집·Shadow 후보 및 rank 이동 UI.
- `tests/test_pem_verification_policy.py`, `tests/test_pem_explorer.py` -- 경계, 대칭, 순서, 불변성, asset 및 원자 게시 회귀.

## Tasks & Acceptance

**Execution:**
- [x] `_bmad/*` -- resolver와 프로젝트 config를 복구하고 Python 3.10 smoke test를 고정한다.
- [x] `tools/build_pem_verification_policy.py` -- Tune-only 군집 임계값 선택과 Shadow 대체 결과를 구현한다.
- [x] `tools/build_pem_explorer.py` -- rank 기반 100개 증거, 공유 mask, projection 사유와 50GiB preflight를 구현한다.
- [x] `tools/pem_explorer/*` -- Shadow 비교 패널과 대체 `rank_geo` 이동을 구현한다.
- [x] `tests/*` -- 계획의 단위·통합·불변성·게시 실패 경계를 검증한다.
- [x] `output/pem_explorer/longcircle2_sam` -- `render-topn=100` 전체 보고서를 원자 교체하고 Bear #42를 확인한다.

**Acceptance Criteria:**
- Given Bear 후보가 100개인 detection, when `rank_geo=42`를 선택하면, then R/t와 pose·geometry·texture·rendered mask·mask overlap 또는 명시적 불가 사유가 표시된다.
- Given 기존 Top-1이 보류될 때, when Shadow를 계산하면, then 실제 PEM R/t는 동일하고 대체 후보는 결합점수 없이 원래 기하 순서에서만 선택된다.
- Given Tune에서 임계값을 선택했을 때, when Holdout을 평가하면, then Holdout 결과는 선택 임계값을 변경하지 않는다.

## Spec Change Log

- 2026-08-23: 독립 리뷰에서 full-stage300 Shadow 추가행 7개의 proxy 증거 누락, GT 라벨 기반 후보 건너뛰기, 두 rename 사이의 게시 공백을 발견했다. 원래 기하 순서·Shadow-only·완전 증거를 KEEP하고, 추가행 강제 렌더/검증, GT 평가 불가 fail-closed, `RENAME_EXCHANGE` 단일 원자 교환으로 보강했다.
- 2026-08-23: Tune-only 경계를 엄격히 적용해 12-way sweep에서 Holdout 계산/필드를 제거하고, Tune 승자를 고정한 뒤 선택된 한 조합만 Holdout에 적용했다. 소스 정밀도와 로컬 split 검증을 KEEP해 반올림·분할 불일치를 fail-closed 처리한다.

## Design Notes

정책 sidecar는 baseline 보류 판정과 `fallback_shadow`를 함께 보존한다. 대체 결과에는 선택 rank/proposal, R/t 거리, texture/IoU, GT 판정, 제외 군집 크기와 실패 사유가 들어간다. 보고서 data에는 실제 PEM pose와 후보 데이터만 유지하고 UI는 sidecar 결과를 참조해 이동한다.

## Verification

**Commands:**
- `python3 _bmad/scripts/resolve_customization.py --skill /home/etri/.agents/skills/bmad-quick-dev --key workflow` -- Python 3.10 성공.
- `/home/etri/miniconda3/envs/sam6d/bin/python -m pytest -q` -- 165개 전체 회귀 통과.
- `conda run --no-capture-output -n sam6d python tools/build_pem_explorer.py ... --render-topn 100 --replace-existing` -- asset 검증 후 원자 게시.
- Chrome deep link `?frame=62&object=Bear&rank_geo=42` -- DOM과 screenshot에서 Bear #42 R/t 및 5종 후보별 evidence + 공유 input mask 확인.

## Suggested Review Order

**Shadow 정책과 Tune/Holdout 경계**

- Tune-only 선택과 고정 Holdout 적용의 전체 정책 진입점이다.
  [`build_pem_verification_policy.py:322`](../tools/build_pem_verification_policy.py#L322)

- 대칭 pose 군집과 원래 기하 순서 선택을 한곳에서 강제한다.
  [`build_pem_verification_policy.py:189`](../tools/build_pem_verification_policy.py#L189)

- SO(3) 대칭 거리가 군집 경계를 결정한다.
  [`build_pem_verification_policy.py:107`](../tools/build_pem_verification_policy.py#L107)

**증거 생성과 안전 게시**

- pointwise spooling과 Shadow 추가행 렌더를 report schema로 결합한다.
  [`build_pem_explorer.py:595`](../tools/build_pem_explorer.py#L595)

- 상위 100과 추가 Shadow 자산을 게시 전에 전수 검증한다.
  [`build_pem_explorer.py:1046`](../tools/build_pem_explorer.py#L1046)

- 완성 디렉터리를 단일 원자 교환하고 부모 디렉터리를 fsync한다.
  [`build_pem_explorer.py:1000`](../tools/build_pem_explorer.py#L1000)

- 대용량 빌드의 byte와 inode 여유를 staging 전에 확인한다.
  [`build_pem_explorer.py:1111`](../tools/build_pem_explorer.py#L1111)

**UI 정책 결합**

- 정책 지문·split·임계값·Shadow 필드를 fail-closed 검증한다.
  [`app.js:67`](../tools/pem_explorer/app.js#L67)

- Bear #42와 추가 Shadow 행을 같은 rank 기반 상세 UI로 표시한다.
  [`app.js:181`](../tools/pem_explorer/app.js#L181)

**진단 계측과 복구**

- 안정 tie-break로 geometry-only 선택과 명시적 rank를 보존한다.
  [`model_utils.py:606`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L606)

- Python 3.10에서 tomli fallback으로 BMAD resolver를 복구한다.
  [`resolve_customization.py:22`](../_bmad/scripts/resolve_customization.py#L22)

**회귀 경계**

- Tune 선택이 Holdout 결과로 바뀌지 않음을 고정한다.
  [`test_pem_verification_policy.py:220`](../tests/test_pem_verification_policy.py#L220)

- Top-100 밖 기존 Shadow 행도 proxy 증거를 강제한다.
  [`test_pem_explorer.py:367`](../tests/test_pem_explorer.py#L367)

- 실제 디렉터리 교환의 원자성을 검증한다.
  [`test_pem_explorer.py:587`](../tests/test_pem_explorer.py#L587)
