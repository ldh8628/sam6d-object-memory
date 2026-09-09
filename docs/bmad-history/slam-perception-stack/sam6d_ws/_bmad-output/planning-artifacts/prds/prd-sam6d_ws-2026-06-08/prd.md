---
title: "SAM-6D Pose Estimation 신뢰성 개선 PRD — False Positive 제거 및 Pose 안정화"
status: final
created: 2026-06-08
updated: 2026-06-08
project: sam6d_ws
basis: docs/sam6d-pose-rca-analysis-2026-06-08.md
---

# SAM-6D Pose Estimation 신뢰성 개선 PRD

> 본 PRD는 `docs/sam6d-pose-rca-analysis-2026-06-08.md`(근본 원인 분석)를 근거로 작성되었으며, 실제 라이브 실행 경로의 코드 구조를 반영한다.

## Background

SAM-6D 기반 객체 포즈 추정 시스템은 ROS 노드(`src/sam6d_ros/sam6d_ros/sam6d_inference_node.py`)를 진입점으로, RGB+Depth 동기화 프레임마다 ISM(분할/검출) → PEM(포즈 추정)을 백엔드(`sam6d_master/SAM-6D/run_batch_inference_fast.py`)에서 호출해 pose를 발행한다. 현재 라이브 운용에서 두 가지 신뢰성 문제가 재현되고 있다.

**현재 발생 중인 문제**

- **대상 객체가 없는 장면에서도** 검출과 pose가 출력된다. 시스템에 "객체 없음(no-object)" 판정 개념이 없고, 최종 신뢰도 기준의 발행 게이트가 없기 때문이다.
- **카메라/장면이 정지한 연속 프레임에서도** pose 결과가 프레임마다 흔들린다. 라이브 경로가 원본 SAM-6D의 시드 설정을 누락했고, 추론 내부에 프레임마다 실행되는 무시드(unseeded) 랜덤 연산이 존재하며, 결과를 완충할 temporal filter가 없다.

이 두 문제는 다운스트림(파지·조립 등 로봇 동작)의 의사결정 신뢰도를 직접 훼손하므로 개선이 필요하다.

## Problem Statement

### A. False Positive Pose Detection

대상 객체가 입력 이미지에 존재하지 않아도 pose가 발행된다.

- 검출 단계의 유일한 게이트는 semantic(DINOv2 CLS 코사인) **단일 모달 0.2** 임계뿐이다. (`Instance_Segmentation_Model/model/detector.py:286-288`, `confidence_thresh=0.2`)
- CAD 객체가 1종이라 `best_template`/`torch.max`가 항상 "가장 비슷한 템플릿"을 선택 → 배경 영역도 약한 유사도(>0.2)면 통과.
- PEM 입구 필터도 동일한 ISM score `> 0.2`(`run_batch_inference_fast.py:544-548`)이며, 검출이 0개일 때만 무출력.
- PEM의 `pred_pose_score`(`Pose_Estimation_Model/utils/model_utils.py:277-285`)는 계산되지만 **어떤 임계와도 비교되지 않고**, `pose_scores = pred_pose_score * ISM_score`로 모든 검출에 무조건 기록(`run_batch_inference_fast.py:680-689`) 후 노드가 조건 없이 발행(`sam6d_inference_node.py:898, 921`)한다.

→ **요약: 파이프라인 어디에도 "객체 없음" 판정과 최종 confidence floor가 없다.**

### B. Pose Instability

정지 장면의 연속 프레임에서 pose(R, t)가 프레임마다 변동한다.

- **무시드 관측 포인트클라우드 서브샘플링**: 매 프레임 다른 2048점 부분집합 선택. (`run_batch_inference_fast.py:587-594`, `np.random.choice`)
- **무시드 RANSAC 포즈 가설 샘플링**: coarse matching이 `torch.rand`로 ~6000개 무작위 가설을 뽑아 best 1개 선택. (`Pose_Estimation_Model/utils/model_utils.py:221`)
- **시드/결정성 설정 전면 부재**: 라이브 경로에 `manual_seed`/`np.random.seed`/`use_deterministic_algorithms`/`cudnn.deterministic` 0건. 원본 진입점(`Pose_Estimation_Model/test_bop.py:200-201`)에는 존재하나 라이브가 미사용.
- **Temporal filtering 전무**: 노드에 EMA/Kalman/평균화/프레임 간 트래킹 없음 → 위 변동이 출력에 그대로 노출.

→ **요약: 무시드 랜덤 + 시드 부재 + 후처리 안정화 부재가 결합되어 jitter가 발생한다.**

## Functional Requirements

> FR는 능력(capability) 단위로 기술하며, 각 FR에 실제 코드 앵커를 명시한다. 기본 임계값/계수는 `[ASSUMPTION]`으로 표기하며 Acceptance Test 단계에서 데이터 기반으로 확정한다.

### Feature 1 — No-Object Rejection (문제 A 대응)

**FR-1. 객체 없음 판정 기능**
시스템은 "대상 객체가 장면에 존재하지 않음"을 명시적으로 판정하고, 그 경우 어떤 pose도 발행하지 않아야 한다. 검출이 0개인 경우뿐 아니라, 검출이 존재하나 신뢰도가 기준 미달인 경우도 "객체 없음"으로 처리한다.
- 앵커: 발행 직전 분기 추가 지점 — `run_batch_inference_fast.py:680-717`(poses 구성), `sam6d_inference_node.py:898/921`(발행).

**FR-2. 최소 confidence threshold 적용**
검출/매칭 신뢰도에 대해 운용 가능한 **절대 최소 임계값**을 적용한다. 기존의 상대적 0.2 게이트(semantic 단일 모달)를 보완하여, ISM 결합 점수(`final_score`)와 PEM `pred_pose_score`에 각각 최소 floor를 둔다.
- 임계값은 설정 파일(`src/sam6d_ros/config/params.yaml`)에서 조정 가능해야 한다.
- `[ASSUMPTION]` 초기 후보: `pem_pose_score_min ≈ 0.3`, `ism_final_score_min ≈ 0.3` — Acceptance Test의 객체 없음 데이터셋 분포로 확정.
- 앵커: `detector.py:286-288`(semantic gate), `run_batch_inference_fast.py:544-548`(det gate), `model_utils.py:281-283`(pose_score).

**FR-3. Pose score filtering**
PEM이 산출한 `pred_pose_score`(`model_utils.py:277-285`)를 발행 전 필터 기준으로 **실제 사용**한다. 현재는 점수를 기록만 하고 사용하지 않으므로, score < floor인 detection은 발행 후보에서 제외한다.
- 앵커: `run_batch_inference_fast.py:676-689`.

**FR-4. Low confidence 결과 제거 (배선/통합 요구)**
FR-1(판정)·FR-3(점수 기준)을 실제 발행 경로에 **연결**하는 통합 요구다(FR-1/3와 중복이 아니라 end-to-end 배선을 명시): FR-2/FR-3 기준 미달 detection은 최종 `poses` 리스트에서 제거되어 노드로 전달되지 않아야 한다(상위 top-k 통과 후에도 floor 미달이면 drop). 모든 detection이 제거되면 노드는 "no detection" 상태로 처리하고 pose 토픽을 발행하지 않는다.
- 앵커: `run_batch_inference_fast.py:707-717`, `sam6d_inference_node.py:826-827`.
- **[구현 주의]** 현재 "no detection"은 전용 상태가 아니라 PEM 실패 핸들러(`sam6d_inference_node.py:826-827`의 `if not pem_ret["ok"]: raise`)로 처리되며, 백엔드가 `{"ok": False, "error": "no valid detection"}`(`run_batch_inference_fast.py:664-666`)로 신호한다. 즉 "객체 없음"이 실제 PEM 오류와 한 경로로 합쳐져 있으므로, FR-1 구현 시 둘을 구분하는 전용 상태/로깅이 필요하다.

### Feature 2 — Temporal Stabilization (문제 B 대응)

**FR-5. 연속 프레임 pose 안정화**
동일/정지 장면에 대해 시스템 출력 pose의 프레임 간 변동(translation/rotation variance)이 유의미하게 감소해야 한다. 이는 (a) 추론 결정성 확보와 (b) 후처리 안정화의 조합으로 달성한다.
- **범위 주의**: 원인측 결정성(FR-8)은 Must라 FR-5의 변동 감소가 **부분적으로 Must 범위에서 달성**되며, 후처리 평활화(FR-6/7)에 의존하는 추가 안정화는 Should 범위다. 따라서 AT-3/4의 1차 합격은 FR-8만으로 평가하고, 잔여 변동은 FR-6/7로 추가 개선한다.

**FR-6. Temporal smoothing**
프레임 간 pose를 평활화하는 후처리 단계를 노드에 도입한다(예: 동일 객체 인스턴스에 대한 EMA 또는 저차 필터).
- 평활화 계수와 on/off는 설정 가능해야 한다.
- `[ASSUMPTION]` 동일 인스턴스 연관(association)은 단일 객체·단일 인스턴스 가정 하에 단순 매칭으로 시작. 다중 인스턴스 트래킹은 Nice-to-Have.
- 앵커: 신규 후처리 모듈 + `sam6d_inference_node.py`의 `_publish_poses`(node:898) 직전 삽입.

**FR-7. Outlier rejection**
평활화 이전에 명백한 이상치 프레임(직전 안정 추정 대비 과도한 점프, 또는 FR-3 score 급락)을 거부/보류한다.
- `[ASSUMPTION]` 점프 임계(예: translation Δ, rotation Δ angle)는 설정 가능. 정지 데이터셋의 정상 변동 분포로 확정.

**FR-10. Stabilization ON/OFF 정량 비교 (Change Request 2026-06-09)**
안정화 효과를 정량 입증하기 위해, 시스템은 **단일 run에서 raw pose와 stabilized pose를 함께 저장**하고, 두 신호의 **variance(spread)** 와 **reduction rate**(=1−stabilized/raw)를 산출하며, raw vs stabilized **비교 시각화**를 생성한다.
- 노드 `_write_debug_log`가 raw(`t_*_mm`, `q*_raw`)와 stabilized(`t_*_stab_mm`, `q*_stab`)를 같은 CSV/JSON에 기록한다(추가 OFF run 불요). 안정화 직전 raw pose를 캡쳐.
- 분석 스크립트가 연속 TP 구간에서 translation/rotation spread와 reduction rate를 계산.
- 앵커: `sam6d_inference_node.py` `_run_inference`(raw 캡쳐)·`_write_debug_log`(컬럼 추가), `tools/validation/build_validation_report.py`(비교)·`make_visualizations.py`(플롯).

**FR-8. 추론 결정성 확보 (시드/결정성)**
라이브 경로 진입 시점에 난수 시드를 고정하고(`random.seed` / `np.random.seed` / `torch.manual_seed`), 필요 시 `torch.backends.cudnn.deterministic`/`use_deterministic_algorithms`를 적용한다. 원본 진입점이 수행하던 시드 설정을 라이브 경로에 복원하는 것을 포함한다.
- 앵커: `run_batch_inference_fast.py`(시드 0건) ← 복원 대상; 참고 `test_bop.py:200-201`, `config/base.yaml:102`(`rd_seed: 1`).
- 주의: `np.random.choice`(`:587-594`)와 `torch.rand`(`model_utils.py:221`)는 프레임마다 RNG 상태가 진행되므로, "프로세스 시작 시 1회 시드"만으로는 프레임 간 동일성이 보장되지 않을 수 있다. 프레임별 결정성이 필요하면 프레임 단위 재시드 또는 샘플링 결정화가 추가로 요구된다. → Acceptance Test에서 검증.

### Feature 3 — Observability

**FR-9 (요청의 FR-8). Debug 정보 저장**
원인 격리·임계값 튜닝·회귀 검증을 위해 프레임 단위 디버그 정보를 구조화하여 저장한다.
- CSV: 프레임별 점수(semantic/appearance/geometric/visible_ratio/ism_final/pred_pose_score/final)와 출력 pose(t_xyz, R_quat), 통과/탈락 카운트.
- JSON: 동일 입력 반복 추론 시 사용된 `np.random.choice` 인덱스·coarse 가설 수 등 랜덤 소스 추적.
- 저장 on/off 및 경로는 설정 가능. 기존 `output/sam6d_milk_verify/`, `dump_ism_mask_candidates.py`, `check_semantic_score.py` 자산과 정합.
- 앵커: `run_batch_inference_fast.py` 점수 산출부 + 노드 발행부.

## Non Functional Requirements

- **NFR-1 (정확도 유지)**: 객체가 실제로 존재하는 정상 장면에서의 검출/포즈 정확도는 기존 대비 저하되지 않아야 한다(true positive recall 유지). 신뢰도 floor는 false positive를 줄이되 true positive를 떨어뜨리지 않는 지점으로 설정한다.
- **NFR-2 (추론 속도)**: 추가되는 floor 검사·outlier·smoothing·로깅의 프레임당 오버헤드는 무시 가능한 수준이어야 한다. `[ASSUMPTION]` 프레임당 추가 지연 ≤ 5%(또는 ≤ 5 ms) — 측정으로 확정. 결정성 모드(cudnn.deterministic)는 처리량 영향이 있을 수 있어 옵션화하고 영향도를 측정한다.
- **NFR-3 (API 호환성)**: 기존 ROS 토픽/메시지 타입, `detection_ism.json` 스키마, `run_ism_single`/`run_pem_single` 시그니처를 유지한다. 신규 동작은 기본값 off 또는 기존과 동등한 기본값으로 추가하여 회귀를 방지한다.
- **NFR-4 (설정 가능성/재현성)**: 모든 신규 임계값·계수·시드·디버그 토글은 `params.yaml`/config로 외부화하며, 동일 입력·동일 설정에서 재현 가능해야 한다.

## Success Criteria

> **[2026-06-09 갱신] 검증 데이터셋이 실주행 ros2 bag 2종으로 교체되었다. 아래 절차/판정의 단일 출처는 `_bmad-output/implementation-artifacts/validation-sam6d-reliability-fix.md` 이며, 기존 `output/sam6d_milk_verify/` 단일 baseline 기준은 폐기한다.**
>
> 데이터셋: (1) **`SLAM_with_milk_nomilk`**(실제 `data/milk_nomilk_bag/`) — milk 등장↔사라짐 반복·미존재 구간 포함, 문제 A(False Positive) 검증. (2) **`only_Milk`**(실제 `data/only_milk/`) — milk 상시 가시·카메라 소폭 이동, 문제 B(stability) 검증.
>
> **측정 방법론(self-confirming 방지)**: (a) 개선 착수 **전에 baseline 지표를 먼저 측정**해 고정한다(FP 건수, σ_t, σ_R, recall). (b) confidence floor(FR-2/3)는 **튜닝 split에서 결정**하고 **분리된 hold-out split에서 검증**한다. (c) 두 bag 모두 GT pose 라벨이 없으므로 milk **가시성**은 수동 라벨(`manually_labeled_milk_visible`)로 대체하고, pose **정확도(recall)** 는 가시성 기반 TP/FN 분류로만 평가한다.

**`SLAM_with_milk_nomilk` (문제 A)**
- milk 미존재 프레임의 pose 발행 건수(`false_positive_pose_frames`) **목표 = 0**(하드 타깃) → `true_negative_rate = 1.0`. 조건부 합격: `false_positive_rate ≤ 0.05` + baseline 대비 FP ≥ 90% 감소.
- milk 재등장 후 추정 재개 `recovery_frames` 평균 ≤ 5.
- counter-metric: 가시 프레임 recall(`TP/(TP+FN)`) 저하 ≤ 2%p.

**`only_Milk` (문제 B)**
- 연속 프레임 Δt / ΔR(geodesic): stabilize **off 대비 p95 유의 감소** + "충분히 안정"(Resolved Decision #4). 절대 임계는 off-run 측정 후 확정.
- 결정성: `deterministic.enabled=true` + 동일 입력 반복 시 출력 분산 ↓.

> 정량 지표 정의·산출 위치·CSV/JSON Schema·시각화 산출물은 `validation-sam6d-reliability-fix.md` 참조.

## Acceptance Test

> 상세 절차/명령어/판정은 `validation-sam6d-reliability-fix.md`(AC-1~6, Test Procedure). 아래는 PRD-레벨 매핑.

| 항목 | 데이터셋 | 테스트 방법 | 성공 조건 |
|---|---|---|---|
| AT-0 Baseline 측정(선행) | both | 개선 전 코드로 두 bag 추론 | baseline FP건수·Δt/ΔR·recall 기록 완료 (FR-9 CSV) |
| AT-1 객체 없음 거부 (문제 A) | `SLAM_with_milk_nomilk` | nomilk 구간 재생 + 수동 가시성 라벨 조인 | `false_positive_pose_frames = 0`(하드), `true_negative_rate = 1.0` (조건부 FPR ≤ 0.05) |
| AT-2 정상 검출 유지 | `SLAM_with_milk_nomilk` | 가시 구간 TP/FN 분류 | recall(`TP/(TP+FN)`) 저하 ≤ 2%p (GT pose 없음 → 가시성 기반) |
| AT-2b 재등장 재개 | `SLAM_with_milk_nomilk` | milk 재등장 프레임 추적 | `recovery_frames` 평균 ≤ 5 |
| AT-3/4 Pose 안정성 (문제 B) | `only_Milk` | 연속 프레임 Δt/ΔR 측정, stabilize off vs on | p95 유의 감소 + "충분히 안정" |
| AT-5 결정성 격리 | `only_Milk` | 동일 입력 M회 반복 | 시드 적용 시 출력 분산↓("충분히 안정"이면 합격) |
| AT-6 성능 영향 | both | 전/후 프레임당 처리시간 | 추가 지연 ≤ NFR-2 |
| AT-7 API 호환 | both | 기존 토픽 소비자 회귀 | 토픽·스키마·시그니처 불변 |

## Implementation Scope

**Must Have**
- FR-1 객체 없음 판정, FR-2 최소 confidence threshold, FR-3 pose score filtering, FR-4 low-confidence 제거
- FR-8 추론 결정성(시드 복원) — 문제 B의 1차 원인 직접 해소
- FR-9 Debug 정보 저장(임계값 튜닝·검증의 전제)

**Should Have**
- FR-5/FR-6 연속 프레임 pose 안정화(temporal smoothing)
- FR-7 outlier rejection
- FR-10 Stabilization ON/OFF 정량 비교(raw+stabilized 동시 저장·variance·reduction rate·비교 시각화) — Change Request 2026-06-09, **구현 완료**

**Nice To Have**
- 다중 인스턴스 pose 트래킹/연관(association) 기반 평활화
- 프레임 단위 완전 결정성(프레임별 재시드 또는 샘플링 결정화) 옵션
- 적응형 임계(scene/조명에 따른 동적 floor)

## Risk

**예상 부작용**
- **R-1 (Threshold 과다 → True Positive 손실)**: confidence floor를 높게 잡으면 실제 객체를 놓칠 수 있다. → 완화: 튜닝 split에서 floor 결정 후 **분리된 hold-out split**에서 검증(self-confirming 방지), 객체 없음/존재 score 분포 분리도 활용(FR-9), AT-2 recall 허용오차로 회귀 감시.
- **R-7 (GT 라벨 부재 → AT-2 미실행)**: 정상 데이터셋에 GT pose 라벨이 없으면 recall 회귀(NFR-1/AT-2)를 정량 검증할 수 없다. → 완화: AT-2 선행 조건으로 라벨 확보 명시, 미확보 시 NFR-1은 정성 검토로 대체하고 리스크로 명시 추적.
- **R-2 (Smoothing 지연/관성)**: temporal smoothing이 실제 객체 이동에 대한 반응 지연(lag)을 유발할 수 있다. → 완화: outlier/동작 감지 시 평활화 우회, 계수 설정화.
- **R-3 (결정성 모드 성능 저하)**: `cudnn.deterministic`/`use_deterministic_algorithms`가 처리량을 떨어뜨릴 수 있다. → 완화: 옵션화, NFR-2 측정.
- **R-4 (프레임 단위 결정성 미달)**: 1회 시드만으로는 프레임 간 동일성이 보장되지 않을 수 있다(RNG 상태 진행). → AT-5로 사전 검증, 필요 시 Nice-to-Have 승격.
- **R-5 (API/회귀)**: 발행 경로·JSON 스키마 변경 시 다운스트림 파손. → 기본값 보존(NFR-3), AT-7.
- **R-6 (Smoothing × 다중 인스턴스 혼선)**: 인스턴스 연관 오류 시 서로 다른 객체의 pose를 섞을 수 있다. → 단일 인스턴스 가정으로 시작, 다중은 Nice-to-Have.

## Deliverables

**수정 파일 목록 (예상 — 실제 코드 구조 기준)**
- `sam6d_master/SAM-6D/run_batch_inference_fast.py` — confidence floor/pose score filtering(FR-2~4), 시드 복원(FR-8), 디버그 산출(FR-9). 주요 지점: `:544-548`, `:587-594`, `:676-717`.
- `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py` — 발행 게이트(FR-1/4), temporal smoothing·outlier rejection 후처리(FR-5~7), 디버그 저장 토글(FR-9). 주요 지점: `:826-827`, `:898`, `:921`.
- `src/sam6d_ros/config/params.yaml`, `src/sam6d_ros/config/no_cli_milk.yaml` — 신규 임계값·계수·시드·디버그 파라미터 추가(NFR-4).
- (참조/비수정) `Pose_Estimation_Model/utils/model_utils.py:221, 277-285` — 랜덤성·pose_score 위치(결정성/필터 설계 근거).

**추가 로그 파일**
- 프레임별 점수·pose CSV (FR-9).
- 랜덤 소스 추적 JSON (FR-9 / AT-5).
- (옵션) 정지 프레임 pose 축 누적 시각화 이미지.

**검증 리포트**
- 두 데이터셋에 대한 Acceptance Test 결과표(AT-1~7): FP 감소율, σ_t/σ_R, 성능 영향, API 회귀.

---

## Resolved Decisions (PM 확정, 2026-06-08)

1. ~~**Baseline 데이터셋 = `output/sam6d_milk_verify/`**~~ → **[2026-06-09 폐기·교체]** 검증 데이터셋은 실주행 ros2 bag 2종(`SLAM_with_milk_nomilk`=`data/milk_nomilk_bag/`, `only_Milk`=`data/only_milk/`)으로 교체. 임계값/목표 수치 확정·합격 판정은 `validation-sam6d-reliability-fix.md` 기준.
2. **문제 B 1차 해소 = 시드 복원(FR-8)만 Must** — temporal smoothing/outlier(FR-5~7)는 Should 유지. (Scope 그대로)
3. **운용 가정 = 단일 객체/단일 인스턴스** — 다중 인스턴스 트래킹/연관은 Nice-to-Have 유지.
4. **안정성 합격 기준 = "충분히 안정"** — 정지 장면 variance가 목표 임계 이하면 합격. 프레임 단위 비트 동일성(완전 결정성)은 Nice-to-Have 유지(AT-5 반영).

> 남은 `[ASSUMPTION]`은 임계값/목표 **수치값**뿐이며, 구현 착수 시 milk_verify baseline 측정(FR-9)으로 확정한다. 범위에 영향 없음.
