---
stepsCompleted: [1]
workflowType: 'research'
research_type: 'technical'
research_topic: 'SAM-6D ISM Root Cause Investigation (data-driven)'
research_goals: 'FP/FN 근본원인을 추측 없이 코드·로그·CSV·score 분포로 특정'
user_name: 'Ldh9501'
date: '2026-06-09'
source_verification: true
---

# Technical Research — SAM-6D ISM Root Cause Investigation (Data-Driven)

> **데이터 소스**: `outputs/validation/SLAM_with_milk_nomilk/frame_results.csv` (315프레임, GT 시간매핑, 점수 결측 0) — 권위 소스. 보조: `_run/debug/sam6d_debug.csv`(per-candidate, 1522행). 분석 스크립트 `/tmp/ism_analysis*.py` (레포 미수정).
> **점수 매핑**(노드 `sam6d_inference_node.py:894-896` 근거): `similarity_score`=semantic, `mask_score`=appearance, `bbox_score`=geometric, `pose_score`=PEM.
> **모든 수치는 실측.** 추측은 "[추정]"으로 명시. workspace/depth/pose-gate 개선안·임계 튜닝-only 결론은 제외.
> **데이터 품질 경고**: `visibility_labels.csv`의 frame_id는 debug CSV stem과 58/315만 겹친다(매핑 불일치, temp.md 경고와 일치). 따라서 GT 라벨은 시간매핑이 된 `frame_results.csv`(decision_type)를 권위로 사용했고, 두 소스의 교집합 58프레임에서 분류가 58/58 일치함을 확인했다.

---

## Executive Summary

**전체 confusion (게이트 미적용, 315프레임)**: TP=112, FP=120, FN=6, TN=77 → **P=0.483, R=0.949**. (temp.md baseline과 정확히 일치, 재현 확인.)

**문제의 비대칭이 결정적이다: FP=120 ≫ FN=6.** 즉 "객체 없는데 있다고 함"(FP)이 압도적이며, "있는데 없다고 함"(FN)은 6프레임뿐이다. 시스템은 **과발행(over-publish)** 한다.

### 가장 유력한 원인 3개 (영향도순, 모두 실측 근거)

1. **미보정 단일-점수 floor에 의한 과발행 (FP 지배)** — ISM 모델 자체는 결합점수 임계가 없는 forced-output(`detector.py:384-390`)이고, no-object 판정은 노드의 floor가 담당한다(`sam6d_inference_node.py:898-936`). **이번 run의 실효 floor는 `pose_score≈0.30`**(데이터: FILTERED pose_score max=0.300 / PASS min=0.301로 칼같이 분리). 그런데 FP 프레임의 pose_score 평균=0.566(중앙 0.485)로 이 floor를 쉽게 통과한다. floor가 너무 낮고 단일 점수에 의존 → FP 120 발생.
2. **ISM 단계 점수의 약한 분리력 + 미보정** — 순수 ISM 점수의 present-vs-absent AUC: geometric 0.886 > appearance 0.873 > semantic 0.858. **semantic이 가장 약하고**, 어느 단일 점수도 깨끗이 분리하지 못한다(최고 0.886). 분포가 1~2σ 겹쳐 단일 고정임계로는 P/R 동시 확보 불가.
3. **Appearance ambiguity (흰색 저텍스처) — 단, "무력"은 아님** — appearance는 TP-vs-FP AUC 0.896으로 실제 신호가 있으나(가설 일부 기각), TP 평균 0.643 vs FP 0.501 평균차(0.14)가 클래스내 std(0.075)의 ~2배에 그쳐 겹침이 크다. 보조 신호일 뿐 주역 아님.

### 즉시 검증 가능한 실험 3개

1. **[코드0] 다중-점수 결합 게이트 시뮬레이션** — `frame_results.csv` 상에서 [semantic, appearance, geometric] 로지스틱/가중합 + hold-out 분리로 present-vs-absent 분리. 단일 pose_score floor(현 0.30) 대비 P/R 개선폭 정량화. (이미 §"Effect" 표에서 단일점수 상한은 P≤0.85 확인 → 결합으로 상회 가능한지 검증.)
2. **[코드0] Per-candidate ranking 검증** — debug CSV의 frame별 5후보 중 best가 항상 최고 ism인지, top1-top2 margin이 present/absent를 가르는지 (현 데이터: margin은 분리력 거의 없음 → margin 단독 기각, §Forced Output). best_template id 미기록이므로 template coverage는 별도 계측 필요.
3. **[경량 계측] FN 6프레임 candidate dump** — FN 프레임의 모든 후보 mask/score를 GT milk 위치와 대조해 "milk 후보가 존재하나 점수 미달(ranking failure)"인지 "milk 후보 자체 부재(proposal failure)"인지 확정. 현 로그상 후보는 존재(=ranking failure 정황)하나 GT bbox 부재로 candidate-level 확증 불가.

### 다음 단계 추천: **Quick-Dev** (PRD 아님). 근거는 §Recommended Next Story.

---

## Score Separation Analysis (연구과제 1)

### 클래스별 점수 분포 (315프레임 실측)

| score | TP (n=112) | FP (n=120) | TN (n=77) | FN (n=6) |
|---|---|---|---|---|
| semantic | **0.575**±0.056 | 0.448±0.079 | 0.459±0.072 | 0.425±0.034 |
| appearance | **0.643**±0.075 | 0.501±0.074 | 0.495±0.069 | 0.478±0.072 |
| geometric | **0.388**±0.113 | 0.187±0.116 | 0.111±0.092 | 0.126±0.099 |
| pose | **0.939**±0.142 | 0.566±0.226 | 0.222±0.049 | 0.198±0.064 |

(값=mean±std. 중앙값은 부록 로그 참조.)

### 분리력 (AUC, Mann-Whitney)

| score | TP vs FP (published) | Present vs Absent (전체) | best-threshold (present-vs-absent) |
|---|---:|---:|---|
| semantic | 0.882 | 0.858 | thr 0.515 → TPR 0.84 / FPR 0.21 |
| appearance | **0.896** | 0.873 | thr 0.562 → TPR 0.84 / FPR 0.17 |
| geometric | 0.888 | **0.886** | thr 0.294 → TPR 0.83 / FPR 0.13 |
| pose(PEM) | 0.892 | **0.894** | thr 0.900 → TPR 0.86 / FPR 0.09 |

### 단일 점수를 publish 게이트로 썼을 때 (present-vs-absent, best-thr)

| gate score | thr | TP | FP | TN | FN | Precision | Recall |
|---|---|---|---|---|---|---|---|
| semantic | 0.515 | 99 | 41 | 156 | 19 | 0.707 | 0.839 |
| appearance | 0.562 | 99 | 33 | 164 | 19 | 0.750 | 0.839 |
| geometric | 0.294 | 98 | 26 | 171 | 20 | 0.790 | 0.831 |
| **pose** | 0.900 | 101 | 18 | 179 | 17 | **0.849** | 0.856 |

**답 — TP와 FP를 가장 잘 구분하는 score는?**
- **published 프레임 한정: appearance(0.896)가 근소 최고**, 그러나 semantic/geometric/pose 모두 0.88~0.90으로 사실상 동급. 결정적 단일 승자 없음.
- **전체 present-vs-absent: pose_score(0.894)가 최고이나 이는 PEM 출력(ISM 하류)**. **순수 ISM 점수 중에서는 geometric(bbox IoU, 0.886)이 최강, semantic이 최약(0.858)**.
- 실무적 결론: **어떤 단일 ISM 점수도 P/R을 동시에 확보하지 못한다(단일 게이트 상한 P≈0.79@geometric, R 유지 시).** 다중 점수 결합/보정이 필요하다 — 이것이 단일 임계 튜닝으로 결론낼 수 없는 이유.

---

## Forced Output Analysis (연구과제 2)

**구조 판정 (코드 근거):**
- ISM 모델(`detector.py`)은 결합 final_score에 **절대임계가 없다** → 모델 레벨에서는 forced-output(항상 후보 출력). 유일 게이트는 semantic `confidence_thresh=0.2` + NMS 0.25.
- **그러나 ROS2 노드가 no-object 경로를 추가한다**(`sam6d_inference_node.py:898-936`): `passed = has_R and (ism_score≥ism_score_min) and (pose_score≥pose_score_min)`; 통과 후보 0개 → `decision="NO_OBJECT"`. → **시스템 레벨에서는 순수 forced-output 아님.**

**실측:**
- NO_OBJECT 프레임 = **83개** (= TN 77 + FN 6). PUBLISH = 232.
- **83개 NO_OBJECT 프레임 전부 후보 행을 ≥1개 보유** → no-object 판정은 **빈 proposal이 아니라 score floor**가 만든다.
- 실효 floor 식별: **FILTERED pose_score [0.000, 0.300], PASS pose_score [0.301, 1.000]** — `pose_score_min≈0.30`이 binding. ism_score는 PASS[0.371,0.635]/FILTERED[0.361,0.577]로 겹쳐 binding 아님.
- top1_ism / top1-top2 margin 분리력:
  - top1_ism: PRESENT 0.589±0.017 vs ABSENT 0.463±0.049 (분리 있음, AUC 0.97 on 58-subset).
  - **top1-top2 margin: PRESENT 0.022 vs ABSENT 0.016 — 사실상 분리 없음** → margin 기반 게이트는 데이터상 기각.
  - |semantic−appearance| gap: PRESENT 0.074 vs ABSENT 0.072 — 분리 없음 → 기각.

**답:** 시스템은 forced-output이 **아니다**(83프레임 no-object 출력). 하지만 **no-object 결정이 단일 pose_score(≈0.30)에만 의존하고 그 임계가 너무 낮아** FP 프레임(pose 평균 0.566)이 floor를 통과한다. top1-top2 margin·sem-appe gap은 분리력이 없어 보조 신호로 못 쓴다. → **"무조건 선택"은 아니지만 "너무 관대한 단일 floor"가 실질 문제.**

---

## Proposal vs Ranking Failure (연구과제 3)

**FN = 6프레임 전부 후보를 보유** (present-frame 후보수 mean 4.96, min 4). FN 프레임 best-candidate 점수:

| frame | semantic | appearance | geometric | pose |
|---|---|---|---|---|
| 000318 | 0.458 | 0.437 | 0.102 | 0.169 |
| 001176 | 0.387 | 0.443 | 0.084 | 0.262 |
| 001197 | 0.439 | 0.475 | 0.340 | 0.298 |
| 001579 | 0.471 | 0.634 | 0.037 | 0.153 |
| 001586 | 0.379 | 0.460 | 0.078 | 0.112 |
| 001593 | 0.418 | 0.418 | 0.113 | 0.193 |

- FN best-cand: semantic 0.425±0.034 (FP/TN 대역과 겹침), **pose 0.198±0.064 → 전부 floor(0.30) 미달**.
- 비교: TP pose 0.939, FP pose 0.566. FN의 pose가 압도적으로 낮다.

**답 — Ranking Failure vs Proposal Failure:**
- **Proposal Failure 아님**: 모든 FN 프레임에 후보가 존재한다.
- **Score/Ranking Failure이며, 구체적으로 pose_score 미달**: milk의 best 후보가 semantic도 낮고(0.43, 흰 물체 변별 약함) PEM pose_score가 floor 미달(0.20)이라 NO_OBJECT 처리됨.
- **한계(추측 금지 원칙)**: frame_results/debug 로그는 frame당 best 후보만 점수를 남기고 **GT bbox/template id가 없어** "milk 후보가 5개 중 존재하나 best로 안 뽑힌 것(rank 실패)"인지 "best=milk인데 점수만 낮은 것"인지 candidate-level로 확증 불가. → Executive 실험3(FN candidate dump)로만 확정 가능.

---

## FastSAM Analysis (연구과제 4)

**현재 설정 (실측 config):** `segmentor_model=fastsam`, `stability_score_thresh=0.97`(params.yaml:45), `conf=0.05`/`iou=0.9`/`max_det=200`(fast_sam.yaml), `max_proposals=50`(params.yaml:50), `top_k_for_pem=3`.

**실측 근거:**
- frame당 후보수: present 4.96±0.20, absent 4.66±0.63 (min 3) → 후보 고갈 없음.
- FN 6프레임 전부 후보 보유. R=0.949(FN 6뿐).

**답:** **FN의 주원인은 FastSAM Proposal Failure가 아니라 Score Failure다.** 근거: (1) 모든 FN 프레임에 후보 존재, (2) FN best-cand의 semantic/pose가 낮음, (3) FN이 6개로 극소수. `stability_score_thresh=0.97`이 높지만 후보를 고갈시키지 않았다([추정] 더 어려운 시점/조명에서는 영향 가능하나 본 데이터셋에선 비지배). **지배 문제는 FP(120)이며 이는 proposal이 아니라 점수/보정 문제.**

---

## Template Coverage Analysis (연구과제 5)

**결론: 현재 로그만으로는 판정 불가 (계측 부재).**
- `frame_results.csv`·`sam6d_debug.csv` 어디에도 **선택된 template id가 기록되지 않는다**(컬럼 부재 — 부록 헤더 확인). 따라서 "특정 template 반복/미사용", "특정 시점 score 급락이 template 탓인지" 데이터로 검증 불가.
- 자산 사실(★ 직접확인, 이전 보고서): 42뷰 RGB 템플릿(rgb_0..41.png, 각 113–137KB) 존재, vertex-color PLY로 정상 렌더. 즉 **자산 자체는 비어있지 않다**.
- 정황: FN/FP의 semantic이 낮은 것이 template 커버리지 부족 때문인지, 흰 물체 변별력 때문인지 **분리 불가** → 추측 금지.
- **필요 조치(계측)**: detector의 `best_template` 인덱스를 디버그 로그에 추가하면 (a) template 사용 히스토그램, (b) present-frame에서 미사용 뷰, (c) 특정 뷰에 몰린 best 선택을 즉시 분석 가능. 이전 보고서 실험2/5와 동일.

---

## Root Cause Ranking (연구과제 6·7, 영향도순·실측 근거)

| 순위 | 원인 | 영향도 근거(실측) | 분류 |
|---|---|---|---|
| **1** | **미보정·과관대 단일 floor에 의한 과발행** | FP=120(≫FN 6). 실효 floor=pose_score 0.30, FP pose 평균 0.566로 통과. floor를 0.90으로 올리면 P 0.48→0.85(단, 단일점수 한계) | **Calibration / Absence-threshold** |
| **2** | **ISM 점수 약분리 + 미보정** | 순수 ISM 최고 AUC 0.886(geometric), semantic 최약 0.858. 단일 임계 P 상한 ≈0.79@R0.83 | **Score Calibration** |
| **3** | **Appearance ambiguity (흰 물체)** | appearance AUC 0.873, TP-FP 평균차 0.14 ≈ 2σ 겹침. 신호는 있으나 보조 | **Appearance Ambiguity** |
| **4** | **FN = pose/score 미달 (ranking failure)** | FN 6 전부 후보 보유, pose 0.20<floor. proposal 아님 | **Ranking/Score (PEM 결합)** |
| 5 | **Template Coverage** | 판정 불가(계측 부재) | **Undetermined** |
| — | **Proposal Failure** | **유의한 원인 아님**(모든 FN 후보 보유, 후보수 충분) | 기각 |
| — | **Geometry contribution 부족** | 오히려 geometric이 순수 ISM 최강 분리자(0.886) → 문제 아니라 **자산** | 기각 |

> 핵심 통찰: 흔한 직관과 반대로 **semantic/appearance(외관 매칭)가 약자, geometric(bbox IoU)가 강자**다. 그러나 geometric은 depth/template-pose 투영 기반이라 부분적으로 깊이 의존(단, post-pose workspace gate와는 다른 ISM-stage 점수). FP 지배 문제의 근본은 "ISM이 흰 배경에 0.4~0.5대 점수를 주고, 단일 pose floor가 이를 거르지 못함"이다.

---

## Recommended Next Story

### 추천: **Quick-Dev** (PRD 아님)

**근거:**
1. **데이터·진단 이미 완비** — 315프레임 GT매핑 점수 존재, 근본원인 정량 확정. PRD의 요구발굴 단계가 불필요.
2. **변경이 국소적** — candidate-stage rejection을 "이미 계산되는 [semantic, appearance, geometric] 점수의 보정된 결합 임계"로 추가. pose/workspace/depth gate와 분리된 seam(`run_batch_inference_fast.py:570` 직전 또는 노드 floor 계산부 `sam6d_inference_node.py:898`).
3. **위험 통제 가능** — temp.md의 과적합 경고를 반영해 **train/hold-out 분리 필수**.

**Quick-Dev story 후보 (우선순위):**
- **QD-A (P0, 코드0 선행 분석)**: `frame_results.csv`에서 [sem,appe,geo] 로지스틱회귀/가중합 + 5-fold(또는 시간분할) hold-out으로 present-vs-absent 분리. 목표: 단일 pose-floor 대비 동일 R(≈0.85)에서 P 개선 입증. **이게 안 되면 결합 무의미** → 먼저 확인.
- **QD-B (P1)**: QD-A가 유효하면, 그 결합 점수 + 보정(temperature scaling)을 candidate-stage rejection으로 구현. 단일 인스턴스 가정 유지, no-object를 명시 출력. pose-gate와 코드상 분리.
- **QD-C (P1, 계측)**: detector debug 로그에 `best_template` id 추가 → Template Coverage 분석 활성화(현재 판정불가 해소).

**QA 검증 기준:**
- 315-frame 셋에서 **R≥0.90 유지하며 FP 감소**를 ISM/candidate 단계(게이트 off)로 측정. workspace/depth gate와 분리 입증.
- hold-out에서 train 대비 P/R 저하 < (사전합의 폭). 단일 임계 sweep만으로 합격 처리 금지.

**PRD가 필요해지는 조건:** QD-A에서 결합점수로도 P/R 동시개선이 한계(예 P<0.7 @R0.9)면, 이는 **proposal/feature 품질 한계**(흰 물체)이므로 detect-then-segment 프론트엔드(Grounded-SAM/YOLO-World 등) 또는 temporal voting 같은 구조 변경이 필요 → 그때 PRD로 승격.

---

## 부록 — 재현 & 한계

**재현:** `python3 /tmp/ism_analysis2.py` (소스: `frame_results.csv`). frame-id 매핑 검증: 58프레임 교집합에서 derived-cls == frame_results decision_type **58/58 일치**.

**데이터 한계 (추측 금지 명시):**
- 선택 template id 미기록 → Template Coverage 미판정.
- frame당 best 후보만 GT매핑 점수 보유, GT bbox 부재 → candidate-level TP/FP 라벨 불가(frame-level만). proposal-vs-ranking은 정황(후보 존재)까지만 확증.
- `visibility_labels.csv` ↔ debug CSV frame_id 매핑 불일치(58/315) → margin/per-candidate 분석은 58-subset, frame-level은 frame_results(315) 사용.
- geometric_score는 depth/template-pose 투영 의존 → ISM-stage 점수이나 깊이와 부분 결합(post-pose gate와는 구분).
- `pose_score`는 PEM(ISM 하류) 출력 — 가장 강한 분리자이나 ISM 단독 신호 아님.

---

## 다음 액션 제안 (사용자 결정 필요)

> 본 절은 보고서를 읽은 직후 무엇을 할지 바로 고를 수 있도록 명시하는 클로징이다. (정식 research .md 표준 — 모든 보고서는 이 "다음 액션 제안" 절로 끝낸다.)

**추천: QD-A를 먼저 실행한다.** [sem, appe, geo] 결합이 단일 pose-floor를 실제로 능가하는지 코드0으로 먼저 검증하지 않으면 QD-B/C는 무의미하기 때문이다(§Recommended Next Story 근거).

선택지:
- **(A) QD-A 분석 지금 실행** — `frame_results.csv`로 [sem,appe,geo] 결합 + hold-out 분리, 단일 pose-floor 대비 P/R 개선폭 정량화. (코드0, 레포 미수정)
- **(B) 이 보고서를 `bmad-quick-dev` story로 승격** — QD-A/B/C를 스토리 백로그로 전환해 실행 관리.

→ **어느 쪽을 진행할까요?** (기본 추천: A 먼저 → 결과에 따라 B)
