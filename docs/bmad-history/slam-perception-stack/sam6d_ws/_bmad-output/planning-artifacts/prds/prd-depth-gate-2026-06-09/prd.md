---
title: "SAM-6D Depth/Pose Consistency Secondary Gate — PRD"
status: draft
created: 2026-06-09
project: sam6d_ws
basis: temp.md(Technical Research FP/FN 분석), outputs/validation/SLAM_with_milk_nomilk
---

# Background

전체 bag GT 검증(SLAM_with_milk_nomilk, 316프레임) 결과 Precision 0.714 / Recall 0.829 / F1 0.768.
False Positive 70개는 주로 **사무실/원경 장면에서 milk가 아닌 흰색 물체(공기청정기·박스)를 milk로 오인**해
pose를 발행한 것이고, False Negative 36개는 ISM 후보는 있으나 PEM pose_score가 낮아 NO_OBJECT가 된 것.

Technical Research(temp.md 직전 버전) 실측 결론: **단일 score threshold 상향은 안전하지 않다**
(pose_score_min 0.30→0.35만으로 Recall 0.829→0.763). 그러나 FP는 **물리적으로 비현실적인 pose**를 가진다
(raw z median 1595mm vs TP 709mm, FP의 74%가 z<100/>1500mm 또는 |x|/|y|>800mm). 따라서 score floor는
유지하고 **발행 직전 depth/pose 타당성 secondary gate**로 기하학적으로 틀린 pose만 제거하는 것이 안전하다.

**실코드 사실(조사 완료)**
- `pose_eval_node`는 `config/params.yaml`에 설정만 있고 **구현 파일이 패키지에 없음**(`find` 결과 0건)
  → gate는 **`src/sam6d_ros/sam6d_ros/sam6d_inference_node.py` 내부**에 구현한다.
- 노드는 발행 시점에 다음을 모두 보유: depth 이미지(`live_input/depth.png`, 16UC1 mm,
  `aligned_depth_to_color`에서 저장, node:535), 카메라 내부행렬(`d455f_camera.json` cam_K
  fx=386.8 fy=386.2 cx=322.8 cy=250.3), CAD mesh 포인트(`self.model_points`,`self.radius` =
  `preload_pem_mesh`, node:410), pose `R,t`(t는 mm, node:1245에서 /1000으로 m 변환).
- 발행 경로: `_publish_poses`(node:1238) / `_publish_pose_transforms`(node:1261).
  결정 산출: `_filter_and_decide`→`_stabilize_poses`(node:_run_inference 내부, ~1170-1192).
- 단위 일관성: t(mm) + cam_K(px) + depth.png(mm) → 투영 깊이와 센서 깊이 모두 mm 비교 가능.
  단 `d455f_camera.json`의 `depth_scale=1.0`이라 16UC1(mm) 가정이 맞는지 **구현 시 1회 검증 필요**.

# Problem Statement

대상 milk가 없는 장면에서 기하학적으로 비현실적인 pose가 발행된다(FP=70). score floor만으로는
FP와 marginal TP가 점수상 겹쳐 분리 불가하며, floor 상향은 Recall을 0.80 미만으로 떨어뜨린다.
**score를 건드리지 않고** depth/pose 타당성으로 FP를 제거하되 Recall 손실을 최소화하는 발행 게이트가 없다.

# Goals

- **G1** 기존 score threshold(confidence.pose_score_min/ism_score_min)는 **변경하지 않는다**.
- **G2** 발행 직전 depth/pose 타당성 secondary gate를 추가해 비현실 pose FP를 제거한다.
- **G3** Recall 손실 최소화(하드 게이트 적용 시 Recall ≥ 0.80 유지가 합격 조건).
- **G4** **Shadow mode 우선**: 초기엔 발행 결과를 바꾸지 않고 gate 통과여부·사유만 로깅.
- **G5** gate 효과를 전체 bag으로 정량 측정(baseline vs shadow vs hard).

# Non Goals

이번 PRD에서 **제외**: 단순 pose_score/final_score/mask·bbox area threshold 상향, GT 라벨링 도구
추가 개발(visibility_labels.csv는 정식 GT로 간주), 모델 재학습, SAM/PEM 구조 변경, hard negative 학습,
전체 알고리즘 교체. temporal persistence requirement(FP가 연속 run이라 비효과적·재등장 지연 위험).

# Functional Requirements

### Feature 1 — Depth/Pose Consistency Gate

**FR-1. 발행 직전 secondary gate 적용**
`_stabilize_poses` 직후·`_publish_poses` 직전에, 각 pose에 대해 depth/pose 타당성을 평가하는
게이트를 적용한다. 기존 score 기반 `decision`(PUBLISH/NO_OBJECT/NO_DETECTION)은 그대로 산출하고,
게이트는 그 **이후 단계**로만 동작한다(score floor 불변).
- 앵커: `sam6d_inference_node.py:_run_inference`(~1170-1196), `_publish_poses`(:1238).

**FR-2. Workspace plausibility 검사**
pose translation이 작업공간 범위를 벗어나면 부적합으로 판정한다.
- 조건(설정 가능): `z_min < t_z < z_max`(예 100~1500mm), `|t_x| < xy_max`, `|t_y| < xy_max`(예 800mm).
- 근거: FP z_median 1595mm vs TP 709mm; FP 74%가 범위 밖.

**FR-3. Depth consistency 검사**
추정 pose로 CAD mesh(`model_points`)를 cam_K로 투영해 **projected depth**를 만들고, 동일 픽셀의
**sensor depth(depth.png)**와 비교한다. 아래 §Depth/Pose Gate Design의 metric으로 일치도를 산출한다.

**FR-4. Gate 모드 (shadow / hard / off)**
gate는 `off`(비활성), `shadow`(평가·로깅만, 발행 불변), `hard`(부적합 pose 발행 차단) 3모드를
config로 선택한다. **기본값 = `shadow`**(또는 off)로 두어 기존 동작을 보존(API 호환).

**FR-5. Per-pose 판정 + 사유**
각 pose에 대해 `depth_gate_pass`(bool), `depth_gate_reason`(str), `workspace_gate_pass`,
`workspace_gate_reason`을 산출한다. hard 모드에서 어느 한 gate라도 실패하면 해당 pose를 발행 목록에서
제외한다(모든 pose 제외 시 빈 PoseArray = no-object 경로 재사용).

**FR-6. 결정 추적**
프레임 단위로 `final_publish_decision_before_gate`(게이트 전, 기존 decision)와
`final_publish_decision_after_gate`(게이트 후)를 모두 기록한다. shadow 모드에서는 두 값이 달라도
**after = before로 실제 발행**(불변), hard 모드에서는 after로 발행.

# Depth/Pose Gate Design

> 입력: `R`(3x3), `t`(mm), `model_points`(CAD mesh, mm), `cam_K`, `depth.png`(mm), `mask`(detection_pem
> segmentation). 모든 길이 단위 mm로 통일.

**투영 절차**
1. `P_cam = R · model_points^T + t` (mm, 카메라 좌표). 각 점의 `z = P_cam_z`.
2. `u = fx·X/Z + cx`, `v = fy·Y/Z + cy` (cam_K). 이미지 경계 내 점만 사용.
3. 각 (u,v)에서 projected z(mm)와 sensor depth(depth.png[v,u], mm, depth_scale 보정) 비교.

**Depth Consistency Metric (정의, 설정 가능 가중)**
- `depth_valid_ratio` = (projected 점 중 sensor depth가 유효(>0, NaN 아님)인 비율). 낮으면 DEPTH_UNRELIABLE.
- `depth_error_median` = median(|projected_z − sensor_z|) over valid 점 (mm).
- `depth_error_p90` = 90퍼센타일 |Δz| (mm).
- `depth_agreement_ratio` = (|projected_z − sensor_z| < tol_mm 인 점 비율), tol 설정(예 30~50mm).
- (보조) `projected_mask` ∩ `observed_mask` IoU(mask overlap) — detection mask와 투영 mask 겹침.

**판정 규칙(설정 가능 임계)**
- `workspace_gate_pass` = workspace 조건 만족(FR-2).
- `depth_gate_pass` = `depth_valid_ratio ≥ vr_min` AND `depth_agreement_ratio ≥ agree_min` AND
  `depth_error_median ≤ err_max`. 셋 중 임계는 config.
- `depth_valid_ratio < vr_min`(센서 depth 부족) → `DEPTH_UNRELIABLE` 사유로 **gate를 통과(보류)** 처리
  (= depth 불충분 시 FP 의심만으로 TP를 죽이지 않음; 안전 우선). 사유 로깅.
- reason 문자열 예: `OK`, `WORKSPACE_Z_OUT(1620>1500)`, `DEPTH_DISAGREE(agree=0.21)`,
  `DEPTH_ERR_HIGH(med=210mm)`, `DEPTH_UNRELIABLE(valid=0.08)`.

**임계 산정**: §Validation의 shadow 데이터에서 TP는 통과·FP는 탈락하도록 임계를 정한다(아래 측정 후 확정,
초기 `[ASSUMPTION]`: z_max=1500mm, xy_max=800mm, tol_mm=40, agree_min=0.5, vr_min=0.3, err_max=100mm).

# Logging Requirements

debug CSV(`_write_debug_log`, node:~945) 및 per-frame JSON에 아래 컬럼을 **additive**로 추가한다
(기존 컬럼·스키마 보존). 일부(area/delta)는 이미 detection_pem.json/frame_results에서 산출 가능하나
디버그 CSV에는 없으므로 추가한다.

| 컬럼 | 출처 |
|---|---|
| `bbox_area` | detection_pem bbox w×h |
| `mask_area` | detection_pem segmentation RLE area |
| `pose_x`,`pose_y`,`pose_z` | t (mm) |
| `pose_distance_mm` | ‖t‖ |
| `raw_delta_translation`,`raw_delta_rotation_deg` | 직전 프레임 raw pose 대비 |
| `stabilized_delta_translation`,`stabilized_delta_rotation_deg` | 직전 stabilized 대비 |
| `depth_valid_ratio`,`depth_error_median`,`depth_error_p90`,`depth_agreement_ratio` | gate metric |
| `depth_gate_pass`,`depth_gate_reason` | gate 판정 |
| `workspace_gate_pass`,`workspace_gate_reason` | gate 판정 |
| `final_publish_decision_before_gate`,`final_publish_decision_after_gate` | FR-6 |

# Validation Requirements

전체 bag(SLAM_with_milk_nomilk 316프레임, 정식 GT) 기준 재계산:
- TP / TN / FP / FN / Precision / Recall / F1 / Accuracy
- FP reduction(Δ vs baseline), Recall drop, Precision gain
- **Depth gate pass/fail confusion matrix**: GT(milk有/무) × gate(pass/reject) 2×2.

**필수 3-way 비교**
- **Baseline**: 현재 결과(TP175/FP70/FN36, P0.714/R0.829/F1 0.768).
- **Shadow gate**: 발행 불변, gate 시뮬레이션상 "만약 hard였다면" 결과(=`*_after_gate`로 재계산).
- **Hard gate**: 실제 발행 차단 후 재추론 결과.

`tools/validation/build_validation_report.py`가 `*_before_gate`/`*_after_gate` 컬럼을 읽어 두 confusion
matrix를 산출하도록 확장한다. shadow 결과는 재추론 없이 debug CSV만으로 계산 가능해야 한다.

# Acceptance Criteria

| ID | 기준 |
|---|---|
| AC-1 | Depth/Pose secondary gate가 **shadow mode로 동작**할 수 있어야 한다(config). |
| AC-2 | **Shadow mode에서 기존 pose 발행 결과 불변**(after=before로 발행, 토픽/스키마 동일). |
| AC-3 | debug CSV에 §Logging의 depth/workspace gate 컬럼이 저장되어야 한다. |
| AC-4 | Validation report가 **gate 적용 전/후 confusion matrix**를 계산해야 한다(shadow 시뮬 포함). |
| AC-5 | **Hard gate 적용 시 Recall < 0.80이면 실패**로 간주한다. |
| AC-6 | **Hard gate 적용 시 Precision > baseline 0.714** 로 개선되어야 한다. |
| AC-7 | FP 예시 이미지에 `depth_gate_reason` 또는 `workspace_gate_reason`을 표시해야 한다(`make_visualizations.py`). |
| AC-8 | **FN 증가 여부를 별도 보고**해야 한다(gate가 TP를 죽여 FN을 늘렸는지). |

# Risks

- **R-1 Depth noise → TP 오탈락**: 센서 depth 결손/노이즈로 정상 pose가 DISAGREE 판정.
  → 완화: `depth_valid_ratio<vr_min`이면 gate 통과(보류), tol/agree 임계를 shadow에서 보수적으로 산정.
- **R-2 원경 milk가 workspace gate에서 제거**: 먼 milk(z>1500)가 정상인데 차단 → FN 증가.
  → 완화: z_max를 shadow에서 TP z분포로 산정(TP z_max 4018mm 존재 → 고정 1500은 위험), depth 일치
  검사를 1차로 두고 workspace는 보조. AC-8로 감시.
- **R-3 depth/RGB sync mismatch**: `aligned_depth_to_color`라 정렬되어 있으나 타임스탬프 슬롭(0.08s)으로
  미세 불일치 가능 → Δz 증가. → 완화: 정렬 토픽 사용 확인, tol 여유.
- **R-4 object model scale mismatch**: CAD가 195mm scaled. mesh 단위(mm)와 t(mm) 일치 가정 →
  구현 시 1프레임으로 투영 mask가 detection mask와 겹치는지 검증(scale 1회 확인).
- **R-5 camera intrinsic mismatch**: cam_K가 color 광학계 기준. depth가 color에 align되어 동일 K 사용
  가정 → 검증 필요.
- **R-6 shadow→hard 전환 시 성능 저하**: 투영·depth 비교가 프레임당 비용 추가(model_points 수에 비례).
  → 완화: 포인트 서브샘플(n_sample), gate는 best/PASS pose에만 적용, 처리시간 로깅.
- **R-7 GT 불확실성 잔존**: 정식 GT로 간주하나 scene 라벨 기원 → 임계 과적합 위험. → hold-out 구간 분리
  권장(전체를 튜닝·검증에 동시 사용 금지).

# Implementation Plan

> 실제 파일 기준. **아직 구현하지 않음.**

| 파일 | 변경 | 내용 |
|---|---|---|
| `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py` | 수정(additive) | ① 파라미터 선언/로드(`depth_gate.*`,`workspace.*`, node:~98-110 패턴) ② 신규 `_depth_pose_gate(poses)` 메서드(투영·metric·판정, model_points/cam_K/depth 사용) ③ `_run_inference`에서 `_stabilize_poses` 후 호출, shadow/hard 분기(node:~1190) ④ `_write_debug_log`에 신규 컬럼(node:~945) ⑤ depth np 접근(`live_depth_path` 로드 또는 `_run_inference_from_images`의 depth 전달) |
| `src/sam6d_ros/config/no_cli_milk_nomilk.yaml`, `params.yaml`, `no_cli_only_milk.yaml` | 수정 | `depth_gate:{mode:shadow, tol_mm, agree_min, vr_min, err_max}`, `workspace:{z_min,z_max,xy_max}` 추가(기본 shadow/off) |
| `tools/validation/build_validation_report.py` | 수정 | `*_before_gate`/`*_after_gate` 컬럼 읽어 baseline/shadow/hard confusion matrix + depth-gate confusion matrix 산출 |
| `tools/validation/make_visualizations.py` | 수정 | FP 이미지에 `depth_gate_reason`/`workspace_gate_reason` overlay(AC-7) |
| (신규, 선택) `tools/validation/sweep_depth_gate.py` | 신규 | shadow 로그로 임계(tol/agree/z_max) sweep → AC-5/6 만족 임계 탐색 |

- **cam_K/mesh 접근**: 노드에 이미 존재(`camera_for_inference_path` JSON, `self.model_points`).
- **depth 접근**: `cv2.imread(self.live_depth_path, -1)` 또는 `_run_inference_from_images`의 `depth` np를
  멤버로 보관해 gate에서 사용(현재 depth np는 콜백 지역변수 → 보관 1줄 추가).

# Test Plan

1. **단위**: `_depth_pose_gate` 투영 정확성 — 합성 R,t + 평면 depth로 projected z 검증, scale/단위 확인.
2. **Shadow 회귀(AC-2)**: 동일 입력에서 발행 토픽/PoseArray가 gate off와 **비트 동일**(after=before).
3. **로깅(AC-3)**: debug CSV에 신규 컬럼 존재·값 정상.
4. **Shadow 시뮬(AC-4)**: 전체 bag shadow 실행 → `build_validation_report`가 baseline vs shadow-as-hard
   confusion matrix 출력. depth-gate pass/fail × GT 2×2.
5. **임계 탐색**: `sweep_depth_gate.py`로 Recall≥0.80 유지하며 Precision 최대 임계 탐색.
6. **Hard 적용(AC-5/6/8)**: 선정 임계로 hard 실행·재검증 → Recall≥0.80, Precision>0.714, FN 증가량 보고.
7. **시각화(AC-7)**: FP 이미지에 사유 표시 확인.
8. **성능(R-6)**: gate on/off 프레임당 처리시간 비교.

# Rollout Plan

1. **Phase 0 — 로깅+shadow 구현**: gate 계산·로깅만, 발행 불변(기본 mode=shadow). 회귀 0 확인(AC-2).
2. **Phase 1 — 임계 산정**: 전체 bag shadow 데이터로 임계 sweep, hold-out 검증(R-7).
3. **Phase 2 — hard 게이트 검증**: hard 모드 재추론, AC-5/6/8 충족 시에만 승격.
4. **Phase 3 — 운영 기본값 전환**: 검증 통과 시 config 기본 mode=hard. 미충족 시 shadow 유지·리스크 기록.

# 다음 /bmad-quick-dev 권장 프롬프트

```
/bmad-quick-dev
PRD(temp.md / _bmad-output/.../prd-depth-gate) 승인. Phase 0만 구현한다(최소 패치, shadow 우선).

구현 범위(Phase 0 한정):
1. sam6d_inference_node.py:
   - 파라미터 추가: depth_gate.{mode:off|shadow|hard, tol_mm, agree_min, vr_min, err_max},
     workspace.{z_min,z_max,xy_max}. 기본 mode=shadow.
   - 신규 _depth_pose_gate(poses): model_points를 R,t로 투영(cam_K) → depth.png(mm) 비교하여
     depth_valid_ratio/error_median/error_p90/agreement_ratio + workspace 판정 + reason 산출.
   - _run_inference: _stabilize_poses 후 gate 호출. shadow=발행 불변(after=before), hard=reject pose 제외.
   - depth np 보관(콜백 depth → 멤버), 단위(mm) 1프레임 검증.
   - _write_debug_log: 신규 컬럼 전부 추가(additive).
2. config yaml: depth_gate/workspace 섹션 추가(shadow 기본).
3. build_validation_report.py: *_before/after_gate 로 baseline/shadow confusion + depth-gate 2x2.
4. make_visualizations.py: FP 이미지에 gate reason overlay.

제약: score floor(confidence.*) 변경 금지. shadow에서 발행 비트 동일(AC-2). 전체 함수 단위 diff.
코드 수정 전 백업(git) 확인. 구현 후 전체 bag shadow 실행해 baseline vs shadow-as-hard 비교 보고.
git add/commit/push 금지.
```

---
> 코드 미수정. 실제 파일 구조 반영(pose_eval_node 미존재 → 노드 내부 구현). git 작업 없음.
