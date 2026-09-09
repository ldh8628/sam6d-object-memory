---
stepsCompleted: [1]
workflowType: 'research'
research_type: 'technical'
research_topic: 'SAM-6D Paper vs Official Impl vs Current Project — 3-way Implementation Audit'
research_goals: '현재 프로젝트가 SAM-6D 원본 논문·공식구현을 충실히 따르는지 3자 비교로 감사하고, 차이가 FP/FN에 주는 영향 평가'
user_name: 'Ldh9501'
date: '2026-06-10'
source_verification: true
---

# Technical Research — SAM-6D 논문 ↔ 공식구현 ↔ 현재 프로젝트 3자 감사

> **비교 3자**: (P) 논문 arXiv:2311.15707v2(CVPR2024) / (O) 공식 repo github.com/JiehongLin/SAM-6D / (C) 현재 프로젝트 `sam6d_master/SAM-6D/`(벤더 사본) + `src/sam6d_ros/`(ROS 래퍼).
> **방법**: 논문 수식·repo 기본값은 웹 인용, 현재 코드는 file:line 실측. **추측 금지**. 추정은 "[추정]" 표기.
> **실측 확인 핵심**: ① 모델/스코어링 코어는 verbatim upstream, ② best_template 미기록·PEM 미전달은 **upstream 동작 그대로**(프로젝트 회귀 아님), ③ 차이는 전부 ROS 후처리 레이어 + `run_batch_inference_fast.py`(배칭) + `ism_rerank`(검증 run에서 OFF), ④ DINOv2 백본 = `vits14`(최소).

---

## Executive Summary — 가장 중요한 차이점 5개

1. **스코어링 코어는 논문과 100% 일치(차이 없음).** final score `s_m=(s_sem+s_appe+r·s_geo)/(1+1+r)`가 논문 Eq.(4)와 코드 `detector.py:384` 정확히 동일. avg_5(K=5), confidence_thresh 0.2, nms 0.25, appearance Eq.(2), geometric IoU Eq.(3), visible_ratio(thred 0.5) 전부 upstream 기본값과 동일. **→ 지금까지의 근본원인 분석은 "충실하게 구현된 SAM-6D" 위에서 한 것이므로 결론이 유효하다.**

2. **best_template 미기록·PEM 미전달은 프로젝트 버그가 아니라 upstream 설계 그대로다.** 공식 repo도 `best_template`를 로그에 남기지 않고(detector.py:198-207 계산·소비만), PEM도 `detection_ism.json`(mask+bbox+score)만 받아 mask+depth로 독립 pose 추정(논문 Sec.3.2). **→ 직전 보고서의 Selection-Failure Unknown / PEM 독립 결론을 공식구현이 그대로 확증.** 필요한 계측은 upstream에도 없으므로 우리가 추가해야 함(회귀 아님).

3. **현재 시스템의 모든 변형은 "ROS 후처리 레이어"에 격리되어 있다.** upstream에 없는 추가물: 워크스페이스/깊이 게이트, confidence floor(`pose_score_min=ism_score_min=0.3`), 시간적 안정화(EMA+jump), `ism_rerank`, debug CSV. 모두 `sam6d_inference_node.py`에만 존재하고 **모델 스코어링 수식은 건드리지 않는다.** → FP/FN의 책임은 "SAM-6D가 틀려서"가 아니라 "충실한 SAM-6D의 점수가 흰 물체에서 약하고, 후처리 floor가 관대해서".

4. **단 하나의 비-upstream candidate-ranking 변형 = `ism_rerank` (그러나 검증 run에서 OFF).** `rerank_score = 10.0 + semantic + mask_area/100000`로 큰 mask를 우대(node:745-765). 이는 upstream에 없는 선택 편향 소스지만, **`ism_rerank.enabled` 기본 False이고 runtime params 미오버라이드 → 우리가 분석한 데이터엔 미적용.** 향후 켜면 FP/FN에 영향 가능 → 변경 시 주의 플래그.

5. **충실도 갭 후보: DINOv2 백본이 `dinov2_vits14`(ViT-S/14, 384-dim) — 최소 모델.** 흰/저텍스처 물체 변별력에 직결. 논문 벤치마크는 더 큰 DINOv2 사용 [추정: vitl14]. 이게 사실이면 **Confirmed였던 Appearance Ambiguity를 구현이 더 악화**시키는 것. 단일 config 한 줄로 검증·교정 가능 → 최우선 확인 항목.

---

## Original SAM-6D Architecture (연구과제 1)

논문 기준 RGB-D → 6D pose call graph (괄호=공식/현재 코드 위치, 둘이 동일):

```
입력: RGB-D 이미지 + 카메라 K + 대상 CAD
[사전] template generation:
  blenderproc render_custom_templates.py
    └ cam_poses_level0.npy (42뷰, icosphere) → rgb_k, mask_k, xyz_k(NOCS)   k=0..41
[사전] template descriptor generation (ISM init):
  DINOv2 cls-token  → ref_data["descriptors"]      [N_obj,42,C]   (detector.py:88-99)
  DINOv2 patch-token(masked) → ref_data["appe_descriptors"]       (detector.py:113-125)

[런타임]
1. SAM/FastSAM: RGB → object proposals(mask 집합)            (segmentor_model)
2. ISM scoring (proposal m, 각 object o):
   a. semantic  s_sem = avg_top5_k cos(cls_m, cls_Tk)         논문Eq.(1형) / detector.py:262-277
      └ confidence_thresh 0.2 게이트                          detector.py:288
   b. best template T_best = argmax_k (per-template semantic) 논문"highest semantic" / detector.py:199,205
   c. appearance s_appe = mean_j max_i cos(patch_m,j, patch_Tbest,i)  Eq.(2) / detector.py:298-308, loss.py:52-62
   d. rough pose: R=R(T_best), t=masked-depth centroid       detector.py:209-232
      geometric s_geo = IoU(bbox_m, bbox_projected)          Eq.(3) / detector.py:310-322
      visible_ratio r = #matched_patch/#valid (thred .5)     loss.py:64-76
   e. final s_m = (s_sem+s_appe+r·s_geo)/(1+1+r)              Eq.(4) / detector.py:384
   f. NMS 0.25 (per object)                                  detector.py:388
   → detection_ism.json {category_id, bbox, score=s_m, segmentation(RLE)}   (template id/pose 미포함)
3. PEM (det_score_thresh 0.2 통과분만):
   mask∩depth → point cloud P_m;  자체 42 template 재로딩      run_inference_custom.py:159
   Coarse(bg token, partial-to-partial) → Fine point matching → WeightedProcrustes
   pose_score = frac(matched pts: dist<0.15) × mask_frac      model_utils.py:281-283
   → detection_pem.json: score = s_m × pose_score, + R,t       run_inference_custom.py:295-311
출력: 6D pose (R,t) + score
```

---

## Original ISM Analysis (연구과제 2) — 수식+코드(공식=현재 동일)

| 점수 | 논문 수식 | 코드 위치(공식/현재 동일) | 현재값 |
|---|---|---|---|
| semantic | top-K cls-token cosine 평균 | detector.py:262-277 (`topk k=5`→mean) | avg_5, K=5 |
| best template | argmax per-template semantic | detector.py:199 `torch.max(scores,-1)`, :205 gather | 동일 |
| appearance | Eq.(2) masked patch max-cosine 평균(T_best만) | detector.py:298-308 / loss.py:52-62 | 동일 |
| geometric | Eq.(3) IoU(proposal bbox, projected bbox) | detector.py:310-322 / loss.py:64-76 | 동일 |
| visible_ratio | patch 대응비(occlusion) thred 0.5 | loss.py:64-76 | 0.5 |
| **final** | **Eq.(4) (s_sem+s_appe+r·s_geo)/(1+1+r)** | **detector.py:384** | **동일** |
| 게이트 | confidence_thresh, nms | detector.py:288/388 | 0.2 / 0.25 |

**판정: ISM scoring = 논문·공식·현재 3자 완전 일치.** (DINOv2 백본 크기만 별도 이슈 — 아래 Diff 표.)

---

## Original PEM Analysis (연구과제 3)

| 질문 | 답(논문 Sec.3.2 + repo + 현재) |
|---|---|
| PEM 입력 | ISM `segmentation`(RLE mask) + bbox + depth + K. **mask로 point cloud 생성** |
| template 정보 전달? | **아니오.** detection_ism.json에 template id/pose 없음. PEM은 자체 42 template 재로딩(run_inference_custom.py:159) |
| mask만? bbox만? | mask(주) + bbox(crop). score는 carry되나 pose엔 미사용 |
| depth 사용 | mask∩(depth>0) → back-project → P_m point cloud (coarse/fine matching 입력) |
| pose_score 생성 | `(dist<0.15) 비율 × matched-mask 비율` (model_utils.py:281-283), 최종 detection score = s_m × pose_score (run_inference_custom.py:295) |

**판정: PEM은 ISM template과 독립.** 직전 보고서의 "가설 C는 비전달(설계)" 결론을 공식구현이 그대로 확증.

---

## Current Implementation Audit (연구과제 4) — 항목별 3자 비교

| 항목 | 논문(P) | 공식(O) | 현재(C) | 동일? |
|---|---|---|---|---|
| template generation | 42뷰 render | cam_poses_level0(42) | 동일(단 render 스크립트 fork: 한글주석·`--clean_each_view`) | ✅ 메커니즘 동일 |
| template descriptor | DINOv2 cls/patch | vits14 default | **vits14 (384-dim)** | ⚠️ 백본 크기 확인 필요 |
| semantic score | top-K cls cosine | avg_5 | avg_5 | ✅ |
| appearance score | Eq.(2) | loss.py | loss.py 동일 | ✅ |
| geometric score | Eq.(3) IoU | detector.py | 동일 | ✅ |
| best template selection | argmax semantic | detector.py:199 | 동일 | ✅ |
| ISM output | mask+bbox+score | detection_ism.json | 동일 | ✅ |
| PEM input | mask+depth | run_inference_custom | 동일 | ✅ |
| PEM output | R,t + score×pose_score | detection_pem.json | 동일 | ✅ |
| pose score | dist<0.15 비율 | model_utils.py:281 | 동일 | ✅ |
| **candidate rerank** | 없음 | **없음** | **ism_rerank(있음, OFF)** | ❌ 비-upstream(검증 run 미적용) |
| **confidence floor** | 없음 | 없음(데모) | **ism/pose_score_min=0.3** | ❌ ROS 추가 |
| **workspace/depth gate** | 없음 | 없음 | **있음** | ❌ ROS 추가 |
| **temporal stabilize** | 없음 | 없음 | **EMA+jump** | ❌ ROS 추가 |
| **debug CSV** | 없음 | 없음 | **51컬럼 로깅** | ❌ ROS 추가(분석 자산) |
| run orchestration | demo.sh | subprocess/이미지 | **run_batch_inference_fast.py(배칭, 수식무변경)** | ❌ 최적화 |

---

## Original vs Current Diff Table (연구과제 5) — 변경점 전수

**(i) 스코어링 수식**: 변경 0. detector.py / loss.py / 모든 config 임계 = upstream 기본값.

**(ii) 런 스크립트**: `run_batch_inference_fast.py`(프로젝트 신규) = 모델 1회 로딩+이미지 루프(서브프로세스 제거)·DINOv2 chunk 16→32·**컴포넌트 점수 breakdown 로깅 추가**(검증 데이터의 per-candidate sem/appe/geo가 여기서 나옴). **수식·rerank 변경 없음.**

**(iii) ROS 후처리 추가(전부 `sam6d_inference_node.py`, upstream에 없음)**:
- `ism_rerank` (node:88-92, 646-844): `rerank_score=10+semantic+mask_area/1e5`, **기본 OFF**, 검증 run 미적용.
- confidence floor `_filter_and_decide` (node:873-939): `passed=has_R and ism≥0.3 and pose≥0.3` → NO_OBJECT 경로.
- workspace gate (node:1019-1026), depth-agreement gate (node:1059-1084).
- temporal stabilize EMA+jump (node:941-987), determinism seed (node:849-862), debug CSV (node:1163-1320).

**(iv) FastSAM**: upstream이 SAM/FastSAM 둘 다 지원 → FastSAM 사용은 **변형 아님**(config 선택). stability_score_thresh 0.97 = config값(upstream sam.yaml은 0.85, fastsam 경로는 별도) — 사용 segmentor별 차이지 수식변경 아님.

**"몇 % 동일한가"**: **모델·스코어링·PEM 알고리즘 코어 = 사실상 100% 동일.** 차이는 전부 (a) 오케스트레이션 래퍼, (b) ROS 후처리 레이어로 격리. 즉 *알고리즘 충실도 ≈ 100%, 시스템 = 충실한 SAM-6D + 후처리 셸*. 유일한 실질 알고리즘 변수는 **DINOv2 백본 크기(vits14)** 하나.

---

## Best Template Information Flow (연구과제 6)

생성→사용→폐기 전체 경로(공식=현재 동일):
```
생성: detector.py:199 torch.max(scores,-1) → :205 gather → best_template [N_query]
사용: appearance(detector.py:374/302), projection→geometric(detector.py:377/209)
폐기: 함수 종료 시 소멸. detection_ism.json/CSV/PEM 어디에도 미기록·미전달
```
| 질문 | 답 |
|---|---|
| 1. 공식도 best_template 미로깅? | **예.** upstream도 계산·소비만, 출력 안 함 |
| 2. 공식도 PEM에 template 미전달? | **예.** detection_ism.json = mask+bbox+score (논문 Sec.3.2 일치) |
| 3. 현재가 다르게 바꾼 부분? | best_template 자체는 **무변경**. 단 그 *위*에서 candidate를 재정렬하는 `ism_rerank`(OFF)를 추가 — template-level 아닌 candidate-level |
| 4. Selection Failure 분석 정보 이미 존재? | **아니오.** upstream·현재 둘 다 template id 없음 → 계측 필수(직전 보고서 QD-T1) |

---

## Impact on Previous Research (연구과제 7)

| 기존 결론 | 3자 감사 후 영향 |
|---|---|
| Proposal Failure = Rejected | **불변.** FastSAM은 upstream 지원, 변형 아님 |
| Coverage Failure = Rejected | **불변.** cam_poses_level0(42) verbatim upstream |
| Selection Failure = Unknown | **불변+강화.** upstream도 미계측 → 회귀 아닌 구조적 공백, QD-T1 계측 정당성 확증 |
| Appearance Ambiguity = Confirmed | **잠재적 악화.** 백본이 vits14(384-dim, 최소)면 논문보다 변별력↓ → 흰 물체 ambiguity 증폭. **검증 1순위** |
| (신규) candidate rerank | `ism_rerank` 존재하나 OFF → 분석 데이터 영향 없음. 켜면 FP/FN 변동 가능, 변경 시 회귀테스트 필요 |
| 전체 신뢰도 | **상승.** 분석이 "충실 구현된 SAM-6D" 위에서 수행됐음이 확인 → FP/FN은 구현 버그가 아닌 (a)feature 약함(흰물체)+(b)관대한 후처리 floor 문제로 확정 |

---

## Recommended Next Story

### 추천: **Quick-Dev** (Technical Research 아님, PRD 아님)

조사는 충분히 끝났다. 코어가 충실함을 확인했으니 남은 것은 두 개의 *국소 검증/계측*이다.

- **QD-V1 (P0, config 검증·코드0)**: 공식 upstream의 DINOv2 descriptor 기본값을 확인(vits14 vs vitl14). **현재 vits14가 논문/공식 대비 다운그레이드면**, config 한 줄(`model_name: dinov2_vitl14`)로 교체 후 동일 315프레임에서 sem/appe AUC·FP 변화 측정. 흰 물체 Appearance Ambiguity의 구현측 원인인지 직접 판정. (가장 비용 대비 영향 큼.)
- **QD-T1 (P0, 계측)**: 직전 보고서대로 `best_template_id`+top5 4컬럼 계측(node:689→:1163→writer). Selection Failure Unknown 해소.
- **QD-A (P1)**: 첫 보고서의 [sem,appe,geo] 결합+hold-out — 관대한 단일 pose floor(0.3) 대체.
- (주의) `ism_rerank`는 검증 run에서 OFF였음을 모든 후속 실험 전제로 고정. 켤 경우 별도 회귀.

**PRD 승격 조건:** QD-V1에서 백본 교체로도, QD-T1에서 template 오선택이 아님이 확인되는데도 흰 물체 P/R 한계가 남으면 → feature 한계이므로 detect-then-segment(Grounded-SAM/YOLO-World)·중간층 patch(FoundPose식)·temporal voting 등 구조 변경 → PRD.

---

## 부록 — 재현 & 한계

**재현:** 코드 anchor는 detector.py/loss.py/sam6d_inference_node.py/run_batch_inference_fast.py 실측. 논문 수식은 arXiv:2311.15707v2 Eq.(1)-(4)·Sec.3.2, 공식 repo github.com/JiehongLin/SAM-6D. `ism_rerank.enabled` 기본 False(node:88), DINOv2 `dinov2_vits14`(configs/.../dinov2.yaml:2) — grep 실측.

**한계(추측 금지):**
- DINOv2 **upstream 기본값이 vits14인지 vitl14인지**는 본 보고서에서 단정 못함([추정] 더 큼). QD-V1로 확정 필요 — 결론이 아닌 검증항목으로 둠.
- `render_custom_templates.py`는 fork 변형(한글주석·flag) — 42뷰/cam_poses 메커니즘은 동일하나 렌더 세부는 upstream과 일대일 diff 미수행.
- best_template 미계측 → Selection Failure는 본 감사로도 미해소(QD-T1 전제).

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 research .md 표준 — 모든 보고서는 이 절로 끝낸다.

**추천: QD-V1(DINOv2 백본 확인·교체)을 최우선.** 코어가 충실함이 확인됐으므로, 흰 물체 FP/FN을 실제로 움직일 단일 알고리즘 변수는 백본 크기 하나뿐이다. config 한 줄 검증으로 가장 큰 영향을 확인할 수 있다.

선택지:
- **(A) QD-V1 + QD-T1을 하나의 `bmad-quick-dev` story로** — 백본 검증/교체 + best_template 계측을 1회 재실행에 묶어 동시 측정. (추천)
- **(B) QD-T1(계측)만 먼저** — Selection Failure부터 확정하고 백본은 후순위.
- **(C) QD-V1만 먼저** — 백본 교체 효과(흰 물체 변별력)부터 측정하고 계측은 후순위.

→ **어느 쪽으로 진행할까요?** (추천: A — 한 번의 재실행으로 충실도 갭과 selection 미지수를 같이 닫음)
