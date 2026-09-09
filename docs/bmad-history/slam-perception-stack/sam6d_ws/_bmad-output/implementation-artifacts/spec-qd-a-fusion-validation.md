---
title: 'QD-A — SAM-6D Eq.(4) Fusion Offline Validation (GO/NO-GO)'
type: 'chore'
created: '2026-06-10'
status: 'in-review'
baseline_commit: '503e871dc4ff6dd4b4d4b03b21dd3f939ea4bbf3'
context:
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-sam6d-ism-root-cause-data-research-2026-06-09.md'
  - '{project-root}/_bmad-output/implementation-artifacts/report-template-selection-instrumentation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Root-cause가 Appearance Ambiguity(흰 물체)로 좁혀졌다. 시스템은 이미 SAM-6D 논문 Eq.(4) `(s_sem+s_appe+r·s_geo)/(2+r)`로 sem/appe/geo를 결합한다. FP=120 과발행이 (1)Eq.(4) 결합의 한계인지 (2)calibration 부족인지 (3)feature 정보량 자체의 ceiling인지 미판정.

**Approach:** **오프라인 전용**으로(프로덕션 코드 무수정) milk 데이터에서 Eq.(4) baseline 대비 대안 결합(가중합·로지스틱·+margin)이 present-vs-absent 분리를 **hold-out에서 유의미하게 능가하는지** 정량 검증. 능가 + 설명가능 + 비과적합 → **GO**(ROS 적용 Story), 아니면 **NO-GO**(구조 변경 연구). 효과 미입증 시 코드 미수정.

## Boundaries & Constraints

**Always:**
- **오프라인 분석만.** 산출물 = 신규 분석 스크립트 + 보고서. 기존 코드 0줄 수정.
- 데이터: Model A/B/C = `baseline_bak/frame_results.csv`(315 풀-GT, sem=similarity/appe=mask/geo=bbox/pose). Eq.(4) baseline score = `baseline_bak/_run/<fid>/detection_ism.json`의 frame별 max score(없으면 fallback 명시). Model D(+margin) = 56프레임 계측 run(top5_template_scores)만 → **저파워 명시**.
- **hold-out 필수**: 5-fold CV + 시간분할(temporal split) 둘 다. train-vs-holdout 성능차로 과적합 정량.
- sklearn 부재 → numpy/scipy로 logreg(경사하강/`scipy.optimize`)·ROC·PR-AUC·k-fold 직접 구현. 난수 seed 고정.
- 개선 시 **원인 귀속**(어느 feature가 driver인지: 계수/ablation) + milk 과적합 위험 평가 필수.

**Ask First:**
- Eq.(4) baseline score 재현이 detection_ism.json 부재로 불가하면, proxy 정의(예 stored ISM score 또는 r 가정)를 어떤 걸로 할지 보고 후 진행.

**Never:**
- ROS 노드/score 계산(`detector.py:384`)/threshold/workspace·depth gate/DINOv2 수정. 새 모델 도입. git add/commit/push.
- hold-out 없이 train 성능만으로 GO 판정.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output | Error Handling |
|---|---|---|---|
| 정상 벤치마크 | 315 frame_results + GT | A/B/C ROC·PR-AUC + 5fold/temporal hold-out P/R | N/A |
| Eq.(4) 재현 | detection_ism.json max score | Eq.(4) score ≈ 코드 결합(감사) | 부재 시 fallback proxy + 경고 |
| Model D(margin) | 56프레임 계측 run | +margin AUC, 단 n=56 저파워 경고 | 데이터부족 시 D 생략·명시 |
| 과적합 | train≫holdout | overfit 플래그, NO-GO 근거 | N/A |
| 개선 미미 | holdout P 개선<임계 | NO-GO → 구조변경 권고 | N/A |

</frozen-after-approval>

## Code Map

- `outputs/validation/SLAM_with_milk_nomilk_baseline_bak/frame_results.csv` -- 315 풀-GT, sem/appe/geo/pose/decision_type.
- `outputs/validation/SLAM_with_milk_nomilk_baseline_bak/_run/<fid>/detection_ism.json` -- frame별 candidate score(Eq.4 출력) 재현용.
- `outputs/validation/SLAM_with_milk_nomilk/_run/debug/sam6d_debug.csv` -- 56프레임 계측(top5 margin) — Model D.
- `detector.py:384` / `run_batch_inference_fast.py:416` -- Eq.(4) 코드(감사 참조, **수정금지**).
- `/tmp/ism_analysis2.py` -- 기존 stdlib AUC/PR 패턴 참고.

## Tasks & Acceptance

**Execution:**
- [x] `tools/validation/qd_a_fusion_benchmark.py` (신규, numpy/scipy) -- Eq.4 재현(detection_ism.json 315/315) + 단일/결합 ROC·PR-AUC + 5fold CV·temporal hold-out + Platt/temp calibration(ECE) + ablation + Model D(56). seed=1. ✓
- [x] `_bmad-output/implementation-artifacts/report-qd-a-fusion-validation.md` (신규) -- 요청형식 보고서, **GO/NO-GO = NO-GO**. ✓

**Acceptance Criteria:**
- Given 315 baseline, when 벤치마크 실행, then Eq.(4) 및 B/C의 hold-out PR-AUC·(동일 recall에서) precision이 **수치로 비교**됨.
- Given hold-out 결과, then train-vs-holdout 성능차가 보고되고 과적합 여부 판정됨.
- Given 핵심질문, then FP 원인이 (1)Eq4 한계/(2)calibration/(3)ceiling 중 무엇인지 데이터로 답함.
- Given 결과, then **GO 또는 NO-GO**가 기준(동일 R에서 명확한 P개선 & 설명가능 & 비과적합/비-milk특화)에 따라 명시되고 Recommended Next Story 제시.

## Verification

**Commands:**
- `conda run -n sam6d_ros_humble python tools/validation/qd_a_fusion_benchmark.py` -- A/B/C/D AUC·hold-out·calibration·verdict 출력.

**Manual checks:**
- Eq.(4) 재현 score가 stored detection score와 ±0.01 내 일치(감사 통과) 또는 fallback 사유 명시.
- holdout AUC가 train AUC 대비 급락 없음(과적합 점검).
