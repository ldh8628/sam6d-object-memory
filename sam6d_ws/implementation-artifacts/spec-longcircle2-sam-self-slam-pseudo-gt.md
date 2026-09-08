---
title: 'longcircle2 SAM 카메라 자체 SLAM 기반 pseudo-GT'
type: 'feature'
created: '2026-08-23'
status: 'done'
baseline_commit: 'NO_VCS'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 전용 SAM RGB-D 데이터는 별도 SLAM 카메라와의 외부파라미터가 없어 현재 HTML의 GT와 후보 포함 여부가 모두 미확인으로 표시된다. 큰 bbox와 선명한 프레임에서 반복 관측된 고정 객체 포즈를 같은 카메라의 ORB-SLAM3 궤적으로 지도 좌표계에 모아 신뢰 가능한 기준 포즈가 필요하다.

**Approach:** `longcircle2_sam` RGB-D 자체로 ORB-SLAM3 `T_map_camera` 궤적을 만들고, 품질 필터를 통과한 객체 포즈 중 회전 30°·이동 80 mm 이내의 직접 이웃이 가장 많은 포즈를 중심으로 강건 평균해 객체별 pseudo-GT를 만든다. 이 참조로 PEM 진단과 재사용 가능한 HTML을 다시 생성하되 외부 계측 GT가 아닌 동일 결과 기반의 탐색적 self-consistency 참조임을 명시한다.

## Boundaries & Constraints

**Always:** 궤적과 객체 포즈는 동일 `longcircle2_sam` bag의 같은 광학 프레임·타임스탬프를 사용한다. CameraInfo에서 ORB 설정을 생성하고 depth 단위 1000을 적용한다. 유효 pose/SLAM 시간 매칭 후 객체별 bbox 면적과 crop 선명도 임계치를 높은 분위수부터 낮추어 최소 10개 품질 표본을 확보하며 선택된 분위수와 각 탈락 이유를 저장한다. 기준 포즈는 DBSCAN의 연쇄 연결이 아니라 한 중심에 대한 직접 30°/80 mm 이웃 최대화로 정하고, 최소 5개와 품질 표본의 50% 이상을 만족할 때만 `trusted`로 둔다. 참조 구성 프레임과 비참조 평가 프레임을 구분하고 provenance·궤적 통계·설정/hash를 남긴다.

**Ask First:** 30°/80 mm, 최소 표본·cluster/share 기준을 변경하거나, 신뢰 기준을 통과하지 못한 객체를 강제로 GT 처리하거나, 현재 기본 HTML 결과를 삭제해야 할 때는 먼저 확인한다.

**Never:** 기존의 별도 SLAM 카메라 궤적을 외부파라미터 없이 적용하지 않는다. 전용 SAM 카메라를 기존 `slam_reference_camera`로 허위 재분류하지 않는다. texture/IoU/GT 오차를 참조 프레임 품질 필터에 사용하지 않으며, 참조 구성 프레임의 결과를 독립 정확도로 보고하지 않는다. 추적 실패·NaN·중복/비단조 timestamp·출처 불일치가 있는 궤적을 조용히 허용하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 정상 | 동일 bag RGB-D, 충분한 추적과 품질 포즈 | 객체별 trusted pseudo-GT, 6000/300/후보 O/X, provenance가 있는 HTML | N/A |
| 희소 객체 | 높은 품질 분위수에서 10개 미만 | 양쪽 품질 조건을 유지하며 임계치를 단계적으로 완화하고 실제 기준 기록 | 최저 기준에서도 부족하면 untrusted와 사유 표시 |
| 다봉 분포 | 서로 다른 pose 군집 존재 | 직접 30°/80 mm 이웃 수 최대 중심과 그 이웃만 평균 | 동률 규칙과 cluster share 기록 |
| SLAM 결손 | 추적 gap 또는 시간 매칭 실패 | 해당 관측 제외, coverage/gap/match 통계 표시 | 품질/cluster 기준 미달 객체는 GT 없음 유지 |
| 잘못된 provenance | 다른 bag/광학 프레임 궤적 | 결과 생성 차단 | fail-closed 오류 |

</frozen-after-approval>

## Code Map

- `configs/orbslam3_longcircle2_sam_rgbd.yaml` -- bag CameraInfo와 depth 단위를 고정한 ORB-SLAM3 설정.
- `tools/run_orbslam3_rgbd_bag.py` -- 같은 bag 재생, 궤적 수집, provenance/coverage 검증을 재현 가능하게 실행.
- `tools/evaluate_slam_pose.py` -- 품질 프레임 선별, 직접 이웃 medoid, 강건 pseudo-GT 및 구성원 기록.
- `tools/validate_rgbd_dataset.py` / `data/longcircle2_sam/dataset_manifest.json` -- 전용 카메라의 self-SLAM 정책과 동일-source provenance를 fail-closed 검증.
- `temp/verify_eval.py` / `tools/build_pem_explorer.py` / `tools/pem_explorer/*` -- GT O/X 재계측, 참조 구성 여부·출처·신뢰 사유 UI 표시.
- `tests/test_evaluate_slam_pose.py` / `tests/test_rgbd_dataset.py` / `tests/test_pem_explorer.py` -- 수치 경계와 오류 경로 회귀 테스트.

## Tasks & Acceptance

**Execution:**
- [x] ORB 설정/실행기를 추가하고 `longcircle2_sam` trajectory와 provenance를 생성·검증한다.
- [x] 평가기를 품질 적응형 필터와 직접 이웃 기반 pseudo-GT로 변경하고 모든 결정 근거를 직렬화한다.
- [x] manifest에 별도 self-SLAM 정책을 추가하고 궤적·pseudo-GT 입력의 동일-source 검증을 구현한다.
- [x] 기존 geometry-only PEM 진단을 새 참조로 재계측하고 기본 `index.html` 보고서를 안전 교체하되 이전 no-GT 보고서를 보존한다.
- [x] UI와 테스트를 갱신하고 전체 회귀 테스트 및 브라우저 smoke check를 수행한다.

**Acceptance Criteria:**
- Given 2,130쌍의 `longcircle2_sam`, when ORB-SLAM3를 실행하면, then 유한·단조 TUM 궤적과 추적 coverage/gap, CameraInfo, bag/설정 hash가 저장된다.
- Given 객체별 품질 포즈, when pseudo-GT를 만들면, then 선택 중심의 모든 구성원은 중심 기준 30°/80 mm 이내이며 bbox/선명도 임계치·표본수·cluster share·대표 프레임이 기록된다.
- Given trusted pseudo-GT와 동일-source trajectory, when 진단/HTML을 재생성하면, then 6000개·300개·상위 후보 O/X와 참조 구성 프레임 여부가 표시되고 GT 출처는 `same-camera ORB-SLAM3 pseudo-GT`로 보인다.
- Given 추적/품질/군집/provenance 실패, when 파이프라인을 실행하면, then 허위 O/X 없이 명시적 unavailable 사유로 종료 또는 표시된다.

## Spec Change Log

## Design Notes

품질 게이트는 객체별 `(bbox area, crop Laplacian variance)` 공동 분위수를 70%부터 단계적으로 낮춘다. 두 조건을 동시에 만족하는 최소 10개를 처음 확보한 단계만 사용하므로 객체 크기 차이를 흡수하면서도 한 품질 축이 낮은 프레임은 포함하지 않는다. pseudo-GT는 외부 정답이 아니므로 성능 수치는 참조 구성 프레임 제외 결과와 전체 self-consistency를 분리한다.

## Verification

**Commands:**
- `python -m pytest -q` -- 전체 테스트 통과.
- `python tools/validate_rgbd_dataset.py data/longcircle2_sam --require-manifest` -- self-SLAM provenance가 유효.
- `python tools/evaluate_slam_pose.py ...` -- trusted/untrusted 근거와 대표 프레임 출력.
- `python tools/build_pem_explorer.py ... --replace-existing` -- 완료 마커가 있는 보고서만 안전 교체.

## Suggested Review Order

**기준 생성 흐름**

- 전체 attestation·품질 선별·GT 생성·평가 분리를 한곳에서 조율한다.
  [`evaluate_slam_pose.py:961`](../tools/evaluate_slam_pose.py#L961)

- bag·trajectory·diagnostic-reference 해시 체인을 pseudo-GT 전에 검증한다.
  [`evaluate_slam_pose.py:121`](../tools/evaluate_slam_pose.py#L121)

- JPEG 캐시 대신 attested bag 원본 RGB로 crop 선명도를 계산한다.
  [`evaluate_slam_pose.py:294`](../tools/evaluate_slam_pose.py#L294)

- 직접 이웃 중심과 고정 임계치로 객체별 trusted 기준을 만든다.
  [`evaluate_slam_pose.py:459`](../tools/evaluate_slam_pose.py#L459)

- 참조 구성 프레임을 제외한 평가 전용 통계를 별도로 산출한다.
  [`evaluate_slam_pose.py:715`](../tools/evaluate_slam_pose.py#L715)

**출처 및 데이터 정책**

- 전용 SAM 카메라의 same-camera self-SLAM 정책을 fail-closed로 고정한다.
  [`validate_rgbd_dataset.py:330`](../tools/validate_rgbd_dataset.py#L330)

- 원본 진단과 compact reference를 동일 dataset snapshot에 묶는다.
  [`extract_pose_reference.py:25`](../tools/extract_pose_reference.py#L25)

- 실제 지정 ROS workspace를 실행하고 재현 입력을 provenance에 기록한다.
  [`run_orbslam3_rgbd_bag.py:124`](../tools/run_orbslam3_rgbd_bag.py#L124)

**진단 및 UI 안전성**

- RGB-D timestamp·중복·누락·메모리 경계를 진단 실행 전에 검증한다.
  [`verify_eval.py:170`](../temp/verify_eval.py#L170)

- 최종 GT unavailable 상태이면 모든 stage O/X를 제거한다.
  [`build_pem_explorer.py:237`](../tools/build_pem_explorer.py#L237)

- 프레임 역할·신뢰 사유·품질/군집 근거를 상세 패널에 표시한다.
  [`app.js:98`](../tools/pem_explorer/app.js#L98)

**회귀 검증**

- 참조 구성과 평가 전용 cohort 분리를 단위 테스트한다.
  [`test_evaluate_slam_pose.py:126`](../tests/test_evaluate_slam_pose.py#L126)

- 잘못된 최종 pose에서 stage O/X가 남지 않음을 검증한다.
  [`test_pem_explorer.py:494`](../tests/test_pem_explorer.py#L494)
