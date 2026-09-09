---
stepsCompleted: [1]
workflowType: 'research'
research_type: 'technical'
research_topic: 'SAM-6D Template Coverage & Template Selection Failure Investigation'
research_goals: 'FP/FN이 (1)Template Coverage 부족 (2)Template Selection 오류 (3)PEM 정보손실 중 무엇인지 코드·로그·실측으로 판정'
user_name: 'Ldh9501'
date: '2026-06-10'
source_verification: true
---

# Technical Research — SAM-6D Template Coverage & Template Selection Failure (Data-Driven)

> **데이터 소스**:
> - 코드: `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py`, `model/loss.py`, `Render/render_custom_templates.py`, `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py`, `run_batch_inference_fast.py`
> - 자산: `Milk_scaled_195mm/templates/`(rgb_0..41, 42뷰), `utils/poses/predefined_poses/cam_poses_level0.npy` (★ stdlib struct로 직접 파싱)
> - 로그: `outputs/validation/SLAM_with_milk_nomilk/{frame_results.csv, _run/debug/sam6d_debug.csv}`
> - 분석 스크립트: `/tmp/pose_coverage.py` (레포 미수정)
> **모든 수치는 실측. 추측은 "[추정]"으로 명시.** workspace/depth/pose-threshold 튜닝 논의는 본 보고서 범위에서 제외.
> **이번 연구의 결정적 데이터 반전**: 코드(`create_template_poses.py`)는 상반구 필터를 포함하지만, **실제 배포된 `cam_poses_level0.npy`는 필터되지 않은 풀-스피어 42뷰**다(파싱으로 확인). 코드 읽기만으로 "상반구 구멍" 결론을 냈다면 오판이었다 — 실측 우선 원칙으로 교정.

---

## Executive Summary

### 가장 중요한 결론 5개

1. **Template Coverage Failure(시점 미커버)는 데이터로 기각된다.** `cam_poses_level0.npy`(42×4×4, float64)를 직접 파싱한 결과 **풀-스피어 커버**다: 상반구 26 + 하반구 16, elevation −90°~+90°, azimuth 0~330° 전 섹터. icosphere subdivision level-1(12→**42**→162)의 전구 꼭짓점이며, 코드의 상반구 필터는 **이 배포 파일에 적용되지 않았다**. 따라서 "milk를 관측한 out-of-plane 시점에 해당하는 template가 아예 없다"는 가설(가설 B)은 시점 차원에서 성립하지 않는다.

2. **유일한 구조적 coverage 공백은 in-plane(roll) 회전이다 — 단, 이는 설계상 의도된 것이며 분석적으로 처리된다.** 42 template는 azimuth·elevation 2-DoF만 렌더하고 roll은 렌더하지 않는다(`render_custom_templates.py`는 `cam_poses_level0.npy`만 사용). 이는 SAM-6D/CNOS/GigaPose 공통 설계(roll까지 렌더하면 수천 장 필요 — GigaPose Table 4: 162 vs 5832)이고, in-plane은 `compute_inplane`으로 사후 복원된다. milk(직육면체, 부분대칭)에서 roll 미커버가 치명적이라는 직접 근거는 없다.

3. **Template Selection Failure는 현 로그로 검증 불가능(Unknown) — best_template가 계산되지만 단 한 곳도 기록되지 않기 때문.** `detector.py:199`에서 `torch.max(scores,dim=-1)`로 argmax 선택 → `:205` gather → `best_template` 산출. 이 변수는 appearance·projection에 **적극 사용**되지만, ROS 노드 `sam6d_inference_node.py:689`에서 in-scope임에도 `_write_debug_log()`(:1163)로 전달되지 않고 폐기된다. debug CSV 51컬럼·frame_results 30컬럼 어디에도 template id 없음. → "정답 template가 뽑히는가"를 데이터로 확인할 수단이 현재 0.

4. **ISM의 template 선택은 PEM pose로 직접 전파되지 않는다 — PEM은 독립적으로 pose를 추정한다.** ISM→PEM 핸드오프(`run_batch_inference_fast.py:434`, `detection_ism.json`)는 **mask+bbox+score만** 전달하고 template id/pose는 전달하지 않는다(`:278-301`). PEM은 mask+depth로 point cloud를 만들어 자체 object template로 pose를 푼다(SAM-6D 논문 Sec.3.2; repo `Pose_Estimation_Model/run_inference_custom.py`). → **가설 C("template은 맞지만 PEM에서 정보손실")는 부정확**: template 정보는 애초에 PEM에 전달되지 않으므로 "손실"이 아니라 "비전달(설계)". ISM template 품질은 **오직 mask 품질을 통해서만** PEM에 간접 영향.

5. **결과적으로 FP/FN의 template 관련 책임은 "selection"으로 좁혀지지만 미계측이라 미확정.** Coverage(시점)는 충분(기각), PEM은 독립(가설 C 재정의), proposal은 이전 연구에서 기각. 남는 것은 (a) textureless DINOv2 feature ambiguity(이전 연구 Confirmed)로 인한 **best_template argmax의 불안정**과 (b) 그 불안정이 실제로 오선택인지 — 후자는 **best_template_id 계측 없이는 판정 불가**. 즉 다음 단계는 새 연구가 아니라 **최소 계측 추가(Quick-Dev)**.

---

## Template Selection Call Graph

코드 기준(모든 anchor 실측). proposal 1개당 흐름:

```
run_batch_inference_fast.py / sam6d_inference_node.py
└─ model.compute_semantic_score(query_desc)               detector.py:255
   ├─ descriptors = ref_data["descriptors"]               detector.py:98   [N_obj,N_tmpl,C]  ← template descriptor 생성(init)
   │     생성: detector.py:88-99  compute_features(cls-token)  dinov2.py:147
   ├─ scores = metric(proposal_desc, descriptors)         detector.py:262  [N_prop,N_obj,N_tmpl]
   │     metric = PairwiseSimilarity(cosine)              loss.py:21-44
   ├─ semantic 집계 = avg_5 (top-5 mean over templates)    detector.py:273-277  → [N_prop,N_obj]
   ├─ confidence_thresh=0.2 게이트 (semantic)              detector.py:286
   └─ ★ best_template = best_template_pose(scores, pred_idx_objects)   detector.py:294
        ├─ _, best_template_idxes = torch.max(scores,-1)   detector.py:199  ← **여기서 best template 결정(argmax)**
        └─ best_template_idx = gather(...)[:,0]            detector.py:205  → [N_query]
   return (idx_selected, pred_idx_objects, semantic_score, best_template)   detector.py:296

→ best_template 소비처(모두 사용, 미기록):
   ├─ compute_appearance_score(best_template,…)            detector.py:374 / 302  (con_idx로 단일 template patch desc 인덱싱) loss.py:52-62
   ├─ project_template_to_image(best_template,…)           detector.py:377 / 209  (poses[best_pose] 투영 → geometric/bbox IoU)
   └─ geometric_score = compute_geometric_score(…)         detector.py:310-322  (visible_ratio loss.py:64-76)

final_score = (semantic + appearance + geometric*visible_ratio)/(2+visible_ratio)   detector.py:384

→ ISM→PEM 핸드오프:
   detections_to_json_direct(...)                          run_batch_inference_fast.py:278-301
   save detection_ism.json = {category_id, bbox, score, segmentation(RLE)}   ← template id/pose 없음
   PEM은 mask+depth로 독립 pose 추정                         run_batch_inference_fast.py:563-647
```

**"어디서 best template가 결정되는가" → `detector.py:199` `torch.max(scores, dim=-1)` (avg_5 집계 전의 raw per-template score에 대한 argmax), `:205`에서 할당 object 기준으로 gather.** 변수명 `best_template_idxes`→`best_template_idx`→리턴명 `best_template`.

---

## Template Coverage Analysis

`cam_poses_level0.npy` 직접 파싱(`/tmp/pose_coverage.py`):

| 항목 | 실측값 |
|---|---|
| pose 개수 | **42** (= icosphere level-1 꼭짓점) |
| 반지름 | 1000mm (전 뷰 동일, 정규 구면) |
| 상/하반구 | **상반구 26, 하반구 16 → 풀-스피어** |
| elevation 범위 | **−90° ~ +90°** |
| elevation 히스토그램(15°빈) | −90:1, −60:5, −45:5, −30:5, 0:10, +15:5, +30:5, +45:5, +90:1 |
| distinct elevations | −90, −58.3, −31.7, −26.6, 0, +26.6, +31.7, +58.3, +90 |
| azimuth 히스토그램(30°섹터) | 0:6,30:4,60:3,90:4,120:4,150:1,180:4,210:4,240:3,270:4,300:4,330:1 (전 섹터 ≥1) |
| in-plane(roll) | **렌더 안 함** (2-DoF만; roll은 `compute_inplane` 사후복원) |
| xyz_*.npy | NOCS 맵(per-pixel 정규화 3D), camera pose 아님 — `render_custom_templates.py:122-135`(float16) |

**해석:**
- **시점 커버리지는 균일·전구(full SO(2)×elevation)**. 인접 꼭짓점 각간격 ≈ 37°(icosphere L1 평균). milk을 어느 방향에서 봐도 ~18° 이내에 template 존재 → "해당 시점 template 부재" 형태의 coverage 공백 없음.
- 유일 공백 = **in-plane roll**. 설계상 의도(GigaPose arXiv:2311.14155 Sec.3.1 + Table4: roll 포함 시 162→5832). CNOS는 42뷰가 segmentation에 "충분"하다고 ablation(arXiv:2307.11067 Table3: 42 vs 162 AP 동일). 단 이 ablation은 *segmentation* 기준이고 *pose 정밀도* 충분성은 별개임을 명시.

---

## Coverage Failure 가능성

**판정: 시점(out-of-plane) coverage failure = 기각(REJECTED, 실측 근거). in-plane coverage = 구조적 공백이나 milk에서 치명성 미입증(Unknown-low).**

- 풀-스피어 42뷰 실측 → 가설 B(유사 시점 template 자체 부재)는 azimuth/elevation 차원에서 성립 안 함.
- 남는 위험: (1) 인접 뷰 ~37° 간격 사이의 "중간 시점"에서 cosine max가 다소 하락(메커니즘적으로 실재하나 CNOS/SAM-6D 논문에 정량 측정 없음 — 웹조사 honest gap), (2) roll 미커버. 둘 다 milk(부분대칭 박스)에서 FP=120 과발행을 설명하는 주원인일 가능성은 낮다 — FP는 "object 없는데 0.4~0.5대 점수"이므로 coverage가 아니라 feature/calibration 문제.

---

## Selection Failure 가능성

**판정: 검증 불가(UNKNOWN) — 계측 부재가 유일한 차단 요인.**

- best_template는 매 proposal `detector.py:199`에서 확정되고 appearance/projection에 쓰이지만 **로그에 한 번도 안 남는다**(audit 확인: debug 51컬럼 + frame_results 30컬럼 전수 검사, template id 없음).
- 따라서 "milk의 정답 뷰 template가 존재하나 다른(유사) template가 argmax로 뽑힌다"(가설 A)를 **현 데이터로 확인/반증 불가**.
- 정황(이전 연구 + 웹): textureless 흰 물체에서 DINOv2 **마지막 블록 cls/patch** 디스크립터는 모호(FoundPose arXiv:2311.18809 Sec.3.3: 깊은 층이 "solid color로 blend"되어 위치정보 손실, 대칭/저텍스처에서 모호). → argmax가 근소차로 흔들릴 개연성은 높으나 **실제 오선택 여부는 미계측**. Co-op(arXiv:2503.17731)은 best-scoring template가 ~180° 뒤집힌 sub-optimal일 수 있음을 보고(3rd-party, moderate).
- **결론**: selection failure는 "있다/없다"를 말할 수 없는 상태. 이를 판정하려면 §"필요한 계측" 적용이 선행 필수.

---

## 가설별 검증 가능성 (연구과제 4)

| 가설 | 내용 | 현 로그 판정 | 근거 |
|---|---|---|---|
| **A** | 정답 template 존재하나 ranking에서 다른 것 선택 | **검증 불가** | best_template 미기록(detector.py:199 계산→node:689 폐기). template id 컬럼 0 |
| **B** | 실제 관측방향과 유사 template 자체 부재 | **부분검증 → 시점차원 기각** | cam_poses 파싱: 풀-스피어 42뷰(elev −90~+90, az 전섹터). roll만 미커버(Unknown-low) |
| **C** | 선택은 맞지만 PEM에서 pose score 하락 | **부분검증 → 전제 수정 필요** | PEM은 template id/pose 미수신(detection_ism.json=mask+bbox만). PEM 독립추정 → "정보손실" 아니라 "비전달(설계)". ISM template→PEM 영향은 mask 품질 경유만 |

---

## Template Coverage Research 사례 조사 (연구과제 5, 웹 근거)

| 주제 | 사례·근거 | 강도 |
|---|---|---|
| 상반구 필터 | CNOS `src/poses/utils.py` `pose_distribution=="upper"`→`z>=0` 필터(레벨0=42). **단 본 프로젝트 배포 파일은 미필터 풀-스피어** — 우리 케이스엔 비적용 | 강(코드) |
| 42뷰 충분성 | CNOS arXiv:2307.11067 Table3: 42 vs 162뷰 AP 동일(~39.6) "42가 충분 coverage" — **단 segmentation AP 기준, pose 정밀도엔 미적용** | 강 |
| view 수 tradeoff | FoundPose arXiv:2311.18809: 400/800/1600 template 정확도 차 미미(단 **풀 SO(3)** 커버) | 강 |
| in-plane 미렌더 | GigaPose arXiv:2311.14155 Sec.3.1/Table4: roll 미렌더(2-DoF만), 사후 예측. 162 vs 5832(36×) | 강 |
| refiner는 in-plane 명시 | FoundationPose 42뷰×12 in-plane=504 가설, MegaPose 26×4=104 — 정적 매칭이 roll 복원 못해서 | 강 |
| textureless 모호 | FoundPose: 마지막층 DINOv2가 대칭/저텍스처에서 모호 → 중간층 patch 사용 권고 | 강 |
| 오선택(180°) | Co-op arXiv:2503.17731: best-score template가 GT서 ~180° 뒤집힌 sub-optimal일 수 있음 | 중(3rd-party) |
| PEM 독립성 | SAM-6D arXiv:2311.15707 Sec.3.2 + repo: PEM은 mask+depth point matching, ISM template 미사용 | 강 |
| honest gap | 풀-스피어 set의 "중간시점/하반구" score 하락을 정량측정한 논문 **없음**; CNOS cls-token의 in-plane 민감도 격리 ablation **없음** | — |

---

## 필요한 계측 항목 (연구과제 6, 코드 미수정·기록대상만 정의)

**계측 위치**: `sam6d_inference_node.py:689`(best_template in-scope) → `_write_debug_log()`(:1163) → debug CSV writer(:1227-1292)로 전달. **best_template는 이미 계산된 1D 텐서라 비용 최소(~5줄).**

**최소 계측 세트(MUST):**
| 항목 | 설명 | 활성화되는 분석 |
|---|---|---|
| `best_template_id` | argmax 선택 template index(0–41) — `detector.py:205` | template 사용 빈도/편향 히스토그램, 오선택 판정 |
| `best_template_sim` | 선택 template의 raw per-template max score(avg_5 집계 전, `:199` max값) | top1 강도 |
| `top5_template_ids` | per-proposal 상위 5 template index | argmax 근소차 흔들림(selection 불안정) 정량화 |
| `top5_template_sims` | 그 5개 score | margin 분리력 |

**권장(SHOULD):**
| 항목 | 설명 |
|---|---|
| `best_template_elev`,`best_template_az` | cam_poses_level0에서 매핑한 선택 뷰 각도 → coverage 사용편향 시각화 |
| `inplane_residual_deg` | `compute_inplane` 산출 잔차(있으면) → roll 미커버 영향 정량 |

이 세트만 있으면: (1) present-frame에서 milk 정답뷰 근방 template가 뽑히는지, (2) FP-frame에서 특정 소수 template로 argmax가 collapse되는지, (3) top1-top5 margin이 흰물체에서 무너지는지 — **가설 A를 데이터로 확정** 가능.

---

## Root Cause Evidence Matrix (연구과제 7)

| 항목 | 상태 | 근거(실측/코드/논문) |
|---|---|---|
| Proposal Failure | **Rejected** | 이전 연구: 모든 FN 후보 보유, FastSAM 후보 충분. 본 연구 변화 없음 |
| Template Coverage Failure | **Rejected** (시점) / Unknown-low (in-plane) | cam_poses 파싱: 풀-스피어 42뷰(elev −90~+90, az 전섹터). roll만 미렌더이나 설계상·치명성 미입증 |
| Template Selection Failure | **Unknown** | best_template `detector.py:199` 계산되나 미기록(node:689 폐기). 판정수단 0 → 계측 필요 |
| Appearance Ambiguity | **Confirmed** | 이전 데이터 AUC + FoundPose: 흰/저텍스처서 마지막층 DINOv2 모호. ISM은 마지막층 cls/patch 사용 |
| PEM Ranking Failure | **Partial** | FN 6프레임 pose<0.30(이전 연구). PEM은 ISM template 독립(mask+depth) → FN의 낮은 pose는 PEM 자체 산출. "template→PEM 전파"는 아님 |

> 핵심: 이번 연구로 **Coverage를 기각**하고 **가설 C(PEM 정보손실)를 "비전달(설계)"로 재정의**했다. 책임은 **Selection(미계측, Unknown) + Appearance Ambiguity(Confirmed)** 로 좁혀졌다.

---

## Recommended Next Story

### 추천: **Quick-Dev** (Technical Research 아님, PRD 아님)

**근거:**
1. **새 연구 불필요** — coverage는 실측으로 닫혔고(기각), PEM 독립성도 코드로 닫혔다. 남은 단 하나의 미지수(Selection Failure)는 *조사*가 아니라 **계측 한 가지**로 즉시 판정 가능. Technical Research를 또 돌리면 같은 결론에 막힌다.
2. **변경이 극소·국소** — `best_template`는 이미 계산된 텐서. node:689→`_write_debug_log` 경로에 컬럼 4개(MUST 세트) 추가 ~5줄. 모델/스코어링 로직 무변경, gate 무관.
3. **PRD 과함** — 요구발굴·아키텍처 결정 없음. 단일 계측 + 재분석.

**Quick-Dev story 후보(우선순위):**
- **QD-T1 (P0, 계측)**: debug CSV에 `best_template_id, best_template_sim, top5_template_ids, top5_template_sims` 추가(node:689→:1163→:1260 writer). 기존 run 1개 재실행으로 충분.
- **QD-T2 (P0, 코드0 분석)**: QD-T1 로그로 — (a) present vs absent template-id 히스토그램, (b) FP-frame argmax collapse 여부, (c) top1-top5 margin 분리력 → **가설 A(selection failure) 확정/기각**.
- **QD-T3 (P1, 조건부)**: QD-T2에서 오선택 확인 시에만 → 마지막층 대신 중간층 patch feature 매칭(FoundPose식) 또는 avg_5 집계 조정 실험.

**QA 기준:** template id 컬럼이 0–41 범위, present-frame에서 milk 정답뷰 근방(±1 인접 꼭짓점) 선택률 측정. selection failure가 기각되면 책임은 Appearance Ambiguity 단독으로 귀결 → 이전 보고서의 결합·보정(QD-A/B)으로 회귀.

**PRD 승격 조건:** QD-T2에서 "정답 template가 존재함에도 구조적으로 오선택"이 확인되고, 그 교정이 매칭 아키텍처 변경(중간층 feature, in-plane 가설 확장, detect-then-segment)으로만 가능하면 PRD로 승격.

---

## 부록 — 재현 & 한계

**재현:** `python3 /tmp/pose_coverage.py` (소스 `cam_poses_level0.npy`, numpy 불요·stdlib struct). 코드 call graph anchor는 detector.py/loss.py/run_batch_inference_fast.py 실측.

**데이터 한계(추측 금지 명시):**
- best_template id 미기록 → Selection Failure는 본 연구로도 **판정 불가**(계측 추가가 전제). 이것이 유일한 미해결 축.
- `render_custom_templates.py`의 상반구 필터 코드 존재 ≠ 배포 파일 상태. **배포 `cam_poses_level0.npy`는 풀-스피어**(파싱 확정)이므로 코드-읽기 결론보다 파싱 결과를 채택.
- 인접 뷰 ~37° 사이 "중간 시점" score 하락은 메커니즘상 실재하나 본 데이터/논문에 정량 측정 없음.
- xyz_*.npy=NOCS이지 pose 아님(혼동 주의).

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 research .md 표준 — 모든 보고서는 이 절로 끝낸다.

**추천: QD-T1(계측) → QD-T2(재분석)를 먼저.** 이번 연구가 도달한 단 하나의 미지수(Selection Failure)는 best_template_id 4컬럼 계측 없이는 영원히 Unknown이다. 이것부터 닫아야 한다.

선택지:
- **(A) QD-T1+T2를 `bmad-quick-dev` story로 생성** — best_template 4컬럼 계측 추가 + 재분석으로 가설 A 확정/기각. (추천)
- **(B) 직전 보고서의 QD-A 먼저 실행** — selection 미확정인 채로 [sem,appe,geo] 결합·보정으로 FP 과발행부터 줄이고, 계측은 후순위.
- **(C) 두 계측을 한 story로 통합** — best_template id + 직전 보고서 결합-게이트 분석을 하나의 Quick-Dev로 묶어 1회 재실행으로 동시 검증.

→ **어느 쪽으로 진행할까요?** (추천: A 또는 C — 계측이 들어가면 두 미해결(selection / 결합게이트)을 한 run에서 같이 검증 가능)
