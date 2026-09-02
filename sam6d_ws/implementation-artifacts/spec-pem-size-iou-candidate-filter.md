---
title: 'PEM 기하 선택과 독립 텍스처·크기·IoU 검증'
type: 'feature'
created: '2026-08-23'
status: 'done'
baseline_commit: 'NO_VCS'
context:
  - 'implementation-artifacts/spec-reusable-pem-pose-explorer.md'
  - 'implementation-artifacts/spec-longcircle2-sam-camera-explorer.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 현재 확장 PEM은 6,000개 포즈를 300개로 줄인 뒤 기하·특징·색상 점수를 z-score와 가중치로 합쳐 초기 포즈를 재선택한다. 서로 구분 능력과 분포가 다른 점수를 하나로 합치면 가중치에 따라 기하 순위가 뒤집힐 수 있고, 투영 크기나 2D mask 겹침도 독립적으로 설명할 수 없다.

**Approach:** 원본 기하 점수만으로 300개를 정렬하고 기하 1위를 유일한 초기 포즈로 선택하도록 실행 구조를 변경한다. 텍스처, 투영 크기, 2D IoU는 이 후보를 재정렬하거나 합산하지 않는 별도 검증 필터로 계산하고, 현재 사용 중인 PEM Explorer `index.html`에서 채널별 PASS/FAIL과 근거를 바로 확인하게 한다.

## Boundaries & Constraints

**Always:** 300→1 선택은 기존 기하 점수와 결정적 tie-break만 사용한다. 텍스처·색상·크기·IoU는 기하 점수에 더하거나 곱하지 않고 후보 순위를 바꾸지 않는다. 기하 1위에 대해 `geometry_selected`, `texture_verified`, `size_verified`, `iou_verified`를 독립 기록하고 최종 상태는 채널별 결과를 보존한다. 검증 실패 시 기하 2위 이하를 자동 선택하지 않는다. 상위 N에는 각 채널의 원점수·채널 내부 순위·임계값·통과 사유와 원래 6,000후보 index를 저장한다. `verification disabled`는 원본 기하 1위 pose와 수치적으로 동일해야 한다. 입력 depth-valid ISM mask, K/crop과 `object @ R.T + t` 투영 규약을 provenance로 보존한다. 재사용 UI 소스와 builder를 먼저 변경한 뒤 기존 `output/pem_explorer/longcircle2_sam/index.html`을 완성본으로 안전 교체하며, 임시 생성 실패 시 기존 HTML을 보존한다.

**Ask First:** 실험 sweep으로 보정하지 않은 텍스처 또는 IoU 임계값을 실시간 pose 거부 조건으로 활성화하는 경우, 검증 실패 후보 대신 차순위 후보를 승격하는 경우, 외부 mesh rasterizer를 추가하는 경우.

**Never:** 합성 점수, z-score 합산, 가중 평균으로 최종 후보를 선택하지 않는다. 텍스처 1위를 기하 1위 대신 초기 포즈로 사용하지 않는다. 현재 3D `geometry.overlap_ratio`를 2D IoU로 부르거나 sparse CAD point mask를 정확한 mesh silhouette로 표현하지 않는다. GT 없이 낮은 검증 점수를 곧바로 오답 또는 정확도 향상으로 주장하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 모든 검증 통과 | 기하 1위가 texture/size/IoU 통과 | 기하 1위를 초기 pose로 유지하고 `verified` | 채널별 근거 기록 |
| 텍스처 불일치 | 기하 1위의 texture rank/score가 기준 미달 | pose와 기하 선택은 보존, `texture_failed` | 차순위 자동 승격 금지 |
| 렌더가 작음 | 입력 면적 > 후보 투영 면적 | `size_failed:render_too_small` | strict 1.0과 tolerance sweep 기록 |
| IoU 불일치 | 크기는 통과하나 mask 겹침 부족 | `iou_failed` 또는 threshold 미보정 시 `diagnostic` | coverage를 함께 표시 |
| 검증 비활성 | verification enabled=false | 원본 기하 1위·fine 입력과 수치 동일 | 검증값이 있어도 선택에 미사용 |
| 투영/mask 불가 | z≤0, 빈 투영, mask 부재 | 기하 선택은 보존하되 검증 불능 | `verification_unavailable`과 원인 기록 |

</frozen-after-approval>

## Code Map

- `sam6d_master/SAM-6D/Pose_Estimation_Model/run_inference_custom.py` -- depth-valid 입력 mask, crop bbox, K를 instance data에 보존.
- `sam6d_master/SAM-6D/Pose_Estimation_Model/model/pose_estimation_model.py` -- shape metadata를 coarse 후보 평가로 전달.
- `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py` -- 기하-only 300→1 선택, 별도 texture/size/IoU 채널과 chunked projection 계측.
- `realtime/verify_config.py`, `realtime/sam6d_core.py` -- 설정 검증, selection state와 후보 provenance 출력.
- `temp/verify_eval.py` -- 동일 데이터·seed의 합성-rerank 대비 기하-only 선택과 독립 필터 threshold sweep.
- `tools/build_pem_explorer.py`, `tools/pem_explorer/` -- 크기·IoU 표와 입력/투영/intersection 증거 이미지.
- `tools/build_pem_verification_policy.py` -- time-blocked stage300 분석에서 shadow 보류 판정과 재현 가능한 policy sidecar 생성.
- `output/pem_explorer/longcircle2_sam/index.html` -- 별도 실험 페이지가 아닌 기존 SAM 결과 진입점에 새 구조를 게시.
- `output/pem_explorer/index.html` -- 기존 landing과 SAM 기본 진입 동작을 유지.
- `tests/` -- 투영 규약, 독립 채널 경계값, 기하 선택 불변성, verification-off parity와 report schema 회귀.

## Tasks & Acceptance

**Execution:**
- [ ] 입력 mask/K/crop 전달과 결정적 후보 projection helper를 추가한다.
- [ ] 결합점수 재정렬을 제거하고 원본 기하 점수 300→1 선택 및 filter-off parity를 고정한다.
- [ ] 상위 N 후보의 텍스처 채널 순위를 별도 계산하고 기하 1위에 대한 검증 결과만 만든다.
- [ ] strict size와 IoU·coverage를 독립 채널로 메모리 제한 chunk에서 계산한다.
- [ ] config/core/dump에 채널별 상태, 임계값, 원순위와 실패 사유를 보존한다.
- [ ] 기존 Explorer의 프레임/객체 상세 화면에 기하 선택과 독립 texture/size/IoU 상태, 면적비·coverage 및 겹침 이미지를 추가한다.
- [x] 보류·수용·오답 올바른 보류·정답 오거부를 기존 Explorer에서 필터 탐색하고 각 임계값 근거를 표시한다.
- [ ] longcircle2 SAM 전체에서 기존 합성-rerank/기하-only와 strict/1.05/1.10 및 texture/IoU sweep을 실행하고 근거 산출물은 별도 보존한다.
- [x] 완성·asset·브라우저 검증을 통과한 보고서만 기존 `output/pem_explorer/longcircle2_sam/index.html`에 안전 게시한다.
- [x] 단위·통합·UI 회귀와 filter-off parity를 검증한다.

**Acceptance Criteria:**
- Given 동일한 300개 후보, when 검증 모듈을 켜거나 끄면, then 선택된 초기 pose는 항상 기하 점수 1위이며 텍스처·크기·IoU 값이 그 순위를 바꾸지 않는다.
- Given 대표 초코하임 오검출 `1785831526041608960`, when 기하 1위 상세를 열면, then 입력/렌더 면적비와 `size_failed` 근거를 확인할 수 있고 차순위 pose로 자동 교체되지 않는다.
- Given saffron 저겹침 사례, when 객체 상세를 열면, then 크기 통과 여부와 별개로 bbox IoU, point-mask IoU, coverage 및 교집합 이미지를 비교할 수 있다.
- Given 현재 `output/pem_explorer/index.html`에서 SAM 결과를 열었을 때, when 프레임과 객체를 선택하면, then 별도 URL로 이동하지 않고 변경된 기하 선택·독립 검증 구조가 표시된다.
- Given depth≥0.8 holdout 평가 결과, when `Holdout 보류` 보기를 선택하면, then 152개 보류 detection이 포함된 프레임만 순회하고 texture·Mask IoU 임계값과 true/false reject를 구분한다.
- Given GT가 없는 SAM 카메라 결과, when A/B 요약을 확인하면, then 기하 1위 불변 여부·채널별 검증 실패율·수동 검토 사례만 보고되고 정확도 증가는 표시되지 않는다.

## Spec Change Log

- 2026-08-23: 사용자 수정 요청에 따라 기하·텍스처·색상 z-score 합산과 가중치 기반 재정렬을 목표에서 제거했다. 원본 기하 300→1 선택을 KEEP하고 텍스처·크기·IoU를 독립 검증 채널로 분리해, 서로 다른 특징의 점수 분포가 하나의 가중치로 상쇄되거나 후보 순위를 뒤집는 상태를 방지한다.
- 2026-08-23: 별도 A/B 전용 HTML만 생성하던 경계를 수정해, 검증된 새 구조를 기존 SAM Explorer `index.html`에 게시하도록 했다. 재사용 UI/builder와 안전 교체를 KEEP해 생성 중 실패로 현재 보고서가 깨지는 상태를 방지한다.
- 2026-08-23: 사용자 검토 요청에 따라 보류 결과를 기존 Explorer에서 직접 순회하도록 했다. 원본 기하 Top-1과 report-data를 KEEP하고, full-stage300 분석에서 생성한 shadow policy sidecar로 Holdout 수용/보류 및 정답·오답 거부 근거를 표시해 서로 다른 진단 덤프의 분모가 섞이는 상태를 방지한다.

## Design Notes

독립 검증은 `기하 선택 → texture 검증 → size 검증 → IoU/coverage 검증`으로 표현하지만 뒤 채널은 앞 채널의 후보를 재정렬하지 않는다. texture는 기하 1위의 texture 채널 내 순위·절대 유사도·margin을 보여주며, 임계값 미보정 단계에서는 PASS/FAIL 대신 diagnostic으로 남긴다. 대표 초코하임 오검출은 입력/렌더 면적비 `2.374`, bbox IoU `0.362`이고, saffron 오포즈는 면적비 `0.700~0.768`이지만 bbox IoU `0.337~0.379`였다. strict 1.0은 전체 2,979건 중 약 452건에 영향을 줄 수 있어 1.05/1.10 sweep을 함께 제시한다.

## Verification

**Commands:**
- `conda run --no-capture-output -n sam6d python -m pytest -q tests` -- 기존 회귀와 신규 edge case 통과.
- `conda run --no-capture-output -n sam6d python temp/verify_eval.py ...` -- 동일 longcircle2 SAM 입력·seed의 verification off/on 및 채널별 sweep 생성.
- `conda run --no-capture-output -n sam6d python tools/build_pem_explorer.py ...` -- 독립 HTML 생성, schema/asset/browser smoke 검증.

## Suggested Review Order

**Shadow 정책과 데이터 결합**

- 시간 분할·독립 AND 거부·결과 지문을 한 정책 생성기에 고정한다.
  [`build_pem_verification_policy.py:36`](../tools/build_pem_verification_policy.py#L36)

- 리포트와 정책의 pose 지문·판정 키 불일치를 게시 전에 차단한다.
  [`build_pem_explorer.py:522`](../tools/build_pem_explorer.py#L522)

- 완성된 임시 번들만 기존 리포트와 원자적으로 교체한다.
  [`build_pem_explorer.py:900`](../tools/build_pem_explorer.py#L900)

**보류 결과 탐색 UI**

- 정책 지문과 판정 필드를 검증하고 오류 시 fail-closed 처리한다.
  [`app.js:12`](../tools/pem_explorer/app.js#L12)

- Holdout 보류·수용·true/false reject를 프레임 단위로 탐색한다.
  [`app.js:91`](../tools/pem_explorer/app.js#L91)

- 기존 Explorer 도구막대에 재사용 가능한 검증 보기 모드를 제공한다.
  [`index.html:23`](../tools/pem_explorer/index.html#L23)

- 실제 longcircle2 결과 진입점에 검증된 정적 번들을 게시했다.
  [`index.html:1`](../output/pem_explorer/longcircle2_sam/index.html#L1)

**회귀와 경계 검증**

- 원자 게시·지문 불일치·정확한 보류 정책 발행을 회귀 테스트한다.
  [`test_pem_explorer.py:206`](../tests/test_pem_explorer.py#L206)

- 중복·boolean·비정상 임계값과 pose 민감 지문을 검증한다.
  [`test_pem_verification_policy.py:31`](../tests/test_pem_verification_policy.py#L31)
