---
stepsCompleted: [1, 2]
inputDocuments: []
workflowType: 'research'
lastStep: 3
research_type: 'technical'
research_topic: 'SAM-6D ISM 렌더 템플릿↔실물 도메인 갭 감소 및 TP/FP 판별 스코어 방법론'
research_goals: 'TP/FP를 확실히 분리하는 스코어; 도메인 갭 축소용 신규 특징 기반 스코어; depth 기반 스코어 및 patch→pixel 전환 효과 검증'
user_name: 'ldh'
date: '2026-07-20'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-07-20
**Author:** ldh
**Research Type:** technical

---

## Research Overview

본 리서치는 `sam6d_ws`의 ISM(Instance Segmentation Model) 단계에서 **TP 객체의 스코어가 0.6~0.7에 뭉치고 FP와 분리되지 않는** 문제를 해소하기 위한 방법론을 조사한다. 사용자의 3대 질문 — (1) TP/FP를 확실히 분리하는 스코어, (2) 렌더↔실물 도메인 갭 축소용 신규 특징 기반 스코어, (3) depth 기반 스코어 및 patch→pixel 전환 효과 — 를 축으로 삼는다.

**중요 전제(재작성 사유):** 본 문서 초안은 정식 SAM-6D 논문/`detector.py`를 baseline으로 서술했으나, `sam6d_ws`는 **정식 구현이 아니라 YOLO-World + DINOv2 ViT-S/14 기반 2단 게이트의 자체 구현**(`yolo_ism_object_n.py`, `yolo_ism.py`)이다. 이 재작성본은 **실제 실행 경로를 권위 baseline으로 고정**하고, 외부 문헌은 그 위의 선택지로 재정렬한다.

**방법론:** 최신 공개 출처(arXiv 논문·공식 코드·BOP)로 핵심 주장을 교차검증했고, 프로젝트 baseline은 실제 소스/config를 직접 읽어 확정했다.

---

## Technical Research Scope Confirmation

**Research Topic:** SAM-6D ISM 렌더 템플릿↔실물 도메인 갭 감소 및 TP/FP 판별 스코어 방법론
**Research Goals:** (1) TP/FP 절대분리 스코어, (2) 도메인 갭 축소용 신규 특징 기반 스코어, (3) depth 기반 스코어 및 patch→pixel 전환 효과 검증

**Technical Research Scope:**

- Score/Calibration 아키텍처 - 실제 baseline(2단 게이트+appe_gate)에서 TP/FP 분리
- 도메인 갭 축소 표현 - sim-to-real 갭 완화 피처 (ViT-S/14 제약 하)
- Depth/geometry 기반 스코어 - 현재 off된 축을 켜는 문제
- Patch vs Pixel(dense) 특징 - masked_appe의 블록 선택/해상도
- 통합/성능 고려 - 실제 파이프라인 내 배치, 실시간성

**Scope Confirmed:** 2026-07-20

---

## Method Landscape Analysis (기술/방법론 지형)

> 본 리서치는 CV 학술 영역이므로 일반 템플릿(프로그래밍 언어/클라우드)이 아니라 **`sam6d_ws` ISM 스코어링의 실제 구조 + 관련 방법론 지형**으로 적응해 조사한다.

### A. 프로젝트 실제 baseline — `sam6d_ws` ISM은 이렇게 돈다 (소스/config 직접 확인)

정식 SAM-6D와의 차이를 먼저 고정한다. **아래가 권위 기준이며, 이후 모든 선택지는 이 baseline에 상대적으로 해석한다.**

| 축 | 정식 SAM-6D (`detector.py`) | **본 프로젝트 (`yolo_ism_object_n.py`)** |
|---|---|---|
| Proposal | SAM ViT-H point-grid, 클래스-무관 무차별 | **YOLO-World(`yolov8m-worldv2.pt`)** 객체별 텍스트 프롬프트, imgsz=960, conf≥0.02, top_k=3 |
| 백본 | DINOv2 **ViT-L/14** (dim 1024, 24블록) | **DINOv2 ViT-S/14** (`dinov2_vits14`, **dim 384, 12블록**), crop 224 |
| 스코어 결합 | 가중평균 `s_m=(s_sem+s_appe+r_vis·s_geo)/(2+r_vis)` | **2단 순차 게이트 (합·곱 없음)** |
| geometric/depth | `s_geo`=2D bbox IoU 항 존재 | **OUT OF SCOPE (호출 안 함)**, 단 `--use-geometric` 스캐폴드는 존재 |
| 절대임계 | 없음(forced-output+NMS) | **`appe_gate=0.55` 절대임계 존재** |

**2단 게이트 상세 (실제 실행 순서):**

```
[YOLO-World proposals: 프롬프트별 top_k=3, conf≥0.02]
      │
Gate 1  SEMANTIC  ── semantic_score(cls) = cosine(crop CLS, template CLS) top-5 평균
      │            객체당 argmax-semantic proposal 1개 슬롯 선택
      │            통과조건: best_sem ≥ similarity_threshold (0.35)
      ▼
[MobileSAM 마스크 생성 → masked query fg patches]
      │
Gate 2  APPEARANCE ── masked_appe_score = (query fg patch마다 best template patch max cosine) 평균
      │            통과조건: masked_appe ≥ appe_gate (0.55)   ← 최종 수락 + 출력 json score
      ▼
[within-object NMS] → [cross-object NMS: nms_rank_blocks=[2,9]로 승자 결정]
```

- **사용자가 보는 "TP 0.6~0.7" = Gate 2의 masked_appe(patch 코사인)** 이다. 결합점수가 아니다.
- masked_appe의 operating point는 **객체마다 다르다**(실측: saffron median 0.750 … choco 0.611; choco MAX 0.712 < saffron 경계 0.72). → **단일 `appe_gate`로 모든 객체를 못 맞추는 것**이 사용자 문제의 실제 기전.
- `appe_blocks=[11]`(최종블록)이 gate+score 구동, `nms_rank_blocks=[2,9]`가 cross-object 승자만 구동.
- 캐시된 이력: per-object appe 임계 시도(milk 0.63, choco 0.59, saffron 0.72)는 모두 **revert** — "distance/blur로 실물 appe가 떨어져 FP와 밴드가 겹친다", "label-free valley-hunting 불가(Hartigan dip p=0.91 단봉)". 이것이 **정식 SAM-6D 문헌의 '코사인은 확률 아님·절대임계 불가' 결론을 프로젝트 실측이 재확인**한 지점.

_Source: `sam6d_ws/yolo_ism_object_n.py`, `sam6d_ws/yolo_ism.py`(semantic_score L238, masked_appe_score L324, build_dinov2 L62), `sam6d_ws/configs/yolo_ism_objects.yaml`(직접 확인). 대조군 정식 SAM-6D: arXiv:2311.15707 https://arxiv.org/html/2311.15707v2 · detector.py https://github.com/JiehongLin/SAM-6D/blob/main/SAM-6D/Instance_Segmentation_Model/model/detector.py_

### B. 스코어 분포/캘리브레이션 취약성 (문제의 원인, baseline 기준 재해석)

- 점수는 **확률이 아닌 raw 코사인**. 실물 crop↔렌더 템플릿은 real↔synthetic 갭 때문에 정답도 코사인 상한이 눌려 **높고 좁은 밴드(~0.5–0.8)에 압축** → TP가 0.6~0.7에 뭉치는 관측과 일치. (기존 감사: 실효 0.55~0.80, "우유 0.64는 천장 0.661 대비 정상".)
- **이 프로젝트에서의 차이:** 정식은 "임계 없음"이 문제지만, 여기선 **appe_gate라는 임계가 이미 있는데 per-object scale 때문에 안 통하는 것**이 문제다. 즉 필요한 것은 "임계 도입"이 아니라 **점수의 객체 간 정규화(스케일 제거) 또는 임계 없는 판별 재료**다.
- 학계도 동일 약점 명시: NOCTIS(arXiv:2507.01463, 2025)가 "CNOS/SAM-6D식 고정 confidence 임계는 객체별 캘리브레이션이 나쁘다"를 동기로 per-object *cyclic threshold* 도입.

_Source: NOCTIS arXiv:2507.01463 https://arxiv.org/html/2507.01463v1 · 프로젝트 config 실측 이력_

### C. 질문 ① — TP/FP 분리 스코어 방법 (baseline: appe_gate가 이미 존재)

문제 재정의: "임계가 없다"가 아니라 **"masked_appe의 스케일이 객체마다 달라 단일 게이트가 안 통하고, distance/blur가 실물 점수를 FP 밴드로 끌어내린다"**. 두 갈래 해법:

**(A) 점수를 상대화(스케일 제거) — 절대값 대신 cohort/순위**
- **AS-norm(adaptive cohort normalization, 화자인식 유래):** 매 프레임 distractor(다른 객체 슬롯/오답 후보)로 cohort를 만들어 masked_appe를 cohort 기준 z-score로 변환 → 객체별 밴드가 정규화돼 **단일 게이트가 의미를 갖게** 됨. per-object 임계 trap의 정면 대안.
- **RRF(Reciprocal Rank Fusion, k=60):** 절대값 버리고 순위만 사용. semantic·masked_appe·YOLO conf·(신규)depth·(신규)texture 채널을 순위로 융합. scale 불변 → 프로젝트 config가 반복 실패한 "여러 채널을 절대값으로 섞기"의 안전한 대체. (이미 `nms_rank_blocks`가 cross-object에서 순위-기반의 안전성을 실증.)
- **EVT/OpenMax tail 피팅:** 정답 점수 꼬리에 Weibull 피팅 → "none-of-the-above" reject 확률화. 무라벨 reject 결정.

**(B) 점수 재료를 바꾼다 — 코사인 대신 기하 검증 inlier**
- **FoundPose(ECCV 2024) 교훈:** 코사인 대신 **PnP-RANSAC inlier 개수**로 랭크 → 0에 가까우면 명백한 오답, 코사인보다 깨끗한 임계. (단 백본 ViT-L layer18 전제는 ViT-S/14에 직접 전이 불가.)
- **MUSE(BOP'24 winner, arXiv:2510.17866):** DINOv2 유사도를 **모델 불확실성으로 변조**해 "자신있는 오답" down-weight. **"CNOS/SAM-6D/NIDS-Net 대비 FP/TP 분리 최고" 명시 주장** — 질문 ①에 가장 직접 대응하는 최신 방법.

_Source: AS-norm Matejka et al. Interspeech 2017 https://www.isca-archive.org/interspeech_2017/matejka17_interspeech.html · RRF Cormack et al. SIGIR 2009 https://cormack.uwaterloo.ca/cormacksigir09-rrf.pdf · OpenMax arXiv:1511.06233 · FoundPose arXiv:2311.18809 · MUSE arXiv:2510.17866_

### D. 질문 ② — 도메인 갭 축소용 신규 특징 기반 스코어 (baseline: ViT-S/14 384dim)

- **SD>DINOv2 증거 없음:** "SD+DINO"(NeurIPS 2023, arXiv:2305.15347)·DIFT(arXiv:2306.03881)의 +19pt는 전부 **구 DINO(2021) 대비**(confound). SD-6D 논문(arXiv:2411.16668) 저자도 "DINOv2가 우수할 수 있다" 인정, FoundPose(DINOv2)가 능가. SD가 이득인 곳은 **textureless·occluded**(사용자 문제와 반대).
- **fine-tuning은 상대 robustness를 오히려 해침**(arXiv:2508.00272) → 현재 **frozen DINOv2 유지가 정당**. 도메인 갭은 "더 좋은 백본"이 아니라 **스코어링/특징 조합**으로 푸는 게 수렴점.
- **ViT는 shape-biased**(Naseer et al. NeurIPS 2021, arXiv:2105.10497): CNN(texture-biased)과 반대로 표면 무늬를 무시하고 형상에 집중하도록 학습됨 → **DINOv2가 같은-형상·다른-인쇄 상자를 못 가르는 게 정상 동작이자 버그**. CLS/semantic으로는 원리상 해결 불가. **해법 = 무시된 appearance/texture/색/텍스트 신호를 의도적으로 재주입.** (프로젝트 config도 동일 결론을 실측으로 도달: "DINOv2 CLS/pooled는 shape-biased, mid-block patch가 texture 유지".)
- **백본 제약 주의:** FoundPose layer 18 등은 ViT-L(24블록) 결과. 본 프로젝트 ViT-S/14(12블록)엔 미전이 → **프로젝트 자체 layer sweep(아래)이 유일한 근거**.

_Source: arXiv:2305.15347 · arXiv:2306.03881 · arXiv:2411.16668 · arXiv:2508.00272 · Naseer arXiv:2105.10497 · Geirhos(CNN texture-bias) arXiv:1811.12231_

### E. 질문 ③ — Depth 스코어 & patch→pixel 전환 (baseline: geometric off, appe=patch)

**E-1. Depth/geometry 스코어 — "꺼진 축을 켜는 문제"**
- 정식 `s_geo`는 2D bbox IoU일 뿐이나, 본 프로젝트는 **geometric을 아예 호출 안 함**(`--use-geometric` 스캐폴드만 존재). 즉 depth 추가는 "없던 것 신설"이 아니라 **기존 스캐폴드 활성화**.
- 표준 depth 검증 스코어: **VSD식 rendered-vs-observed depth 일치도**(BOP 표준, arXiv:2009.07378), **ICP fitness/inlier**(VeREFINE arXiv:1909.05730, PoseRBPF arXiv:1905.09304). coarse pose를 재활용해 PEM 이전 배치 가능.
- **결정적 한계:** 갈색상자↔초코하임은 **형상(육면체)이 같다** → **depth-only는 둘을 동일 취급**. depth는 "다른-형상 오탐"(둥근 물체→상자 오인) 제거엔 강력하나 **"같은-형상·다른-인쇄" 상자 판별엔 무력**. 상자 판별 스코어는 RGB/appearance 분기에서만 온다.

**E-2. patch→pixel(dense) 전환 — "블록 선택이 해상도보다 먼저"**
- masked_appe는 **이미 patch 기반**. patch(14px)는 로고·글자 획을 14×14 창에 평균해 sub-patch 인쇄 소실(FeatUp arXiv:2403.10516 명시).
- **그러나 FeatUp/LoftUp(arXiv:2504.14032) 등 dense 업샘플러는 "위치 해상도"만 복원, "인쇄 내용"은 생성 못 함** → dense 전환만으로 상자 판별 큰 개선 기대 어려움.
- **프로젝트 실측이 더 강한 레버를 이미 발견:** 어느 **patch 블록**을 읽느냐가 texture 판별을 좌우. choco 실물 vs 택배박스 AUC **block 9 = 0.996 vs block 11(현재) = 0.934** (84건 hand-label, 6데이터셋 균등표본; 0.59 게이트 시 실물 35/36 유지·택배박스 43/45 제거). 501건 6객체 확장 실측: 실물 100% 유지 임계에서 FP 제거율 block11 20/117(17%) → blocks[2,9] 44/117(38%), 어느 객체도 악화 없음, 비용 +0.1%.
  - **미해결/주의:** block 9 우위는 **choco 등 same-color 상자에 특화**일 수 있음(인형 Bear는 block2 0.92 vs block9 0.49로 역전). 그래서 gate 블록 변경은 revert됨(appe_gate 재보정 없이 [2,9] 배포 시 분포 상향→오탐 +16%). → **블록별 특성이 객체군마다 달라 "단일 블록" 자체가 함정**.
- **인쇄/텍스트 실측 판별자(shape-bias 보완):** 로컬 키포인트 매치수(SIFT/SuperPoint arXiv:1712.07629/DISK arXiv:2006.13566 — 민무늬 박스는 반복 키포인트 ~0), OCR **텍스트 존재/밀도**(정확 문자열 아님, MVA 2024 DOI:10.1007/s00138-024-01549-9), color 히스토그램(kraft 저채도 단봉 vs 인쇄 다봉 고채도). 이들을 RRF로 융합.

_Source: BOP VSD arXiv:2009.07378 · VeREFINE arXiv:1909.05730 · PoseRBPF arXiv:1905.09304 · FeatUp arXiv:2403.10516 · LoftUp arXiv:2504.14032 · SuperPoint arXiv:1712.07629 · DISK arXiv:2006.13566 · 프로젝트 config layer-sweep 실측_

### F. 최신 CAD-기반 novel-object 검출기 지형 (참고)

| 방법 | 연도 | 스코어링 | 본 프로젝트 관련성 |
|---|---|---|---|
| CNOS arXiv:2307.11067 | ICCVW'23 | DINOv2 CLS 코사인 top-k, δ=0.5 | 프로젝트 semantic gate의 조상 |
| SAM-6D arXiv:2311.15707 | CVPR'24 | sem+appe+r_vis·geo 가중평균 | 프로젝트가 fork·개조한 원본 |
| GigaPose arXiv:2311.14155 | CVPR'24 | 대조 ViT + affine + RANSAC | photometric aug 아이디어 |
| FoundPose arXiv:2311.18809 | ECCV'24 | patch TF-IDF BoW + PnP inlier | inlier-count 임계(질문①), 단 ViT-L 전제 |
| NIDS-Net arXiv:2405.17859 | IROS'25 | 학습형 Weight Adapter + 코사인 | appe refine 대안 |
| **MUSE** arXiv:2510.17866 | BOP'24 win | 불확실성 변조 유사도 | **FP/TP 분리 최고 주장, 질문① 직결** |
| NOCTIS arXiv:2507.01463 | 2025 | Cyclic Thresholding | per-object 임계 trap 대안 |

_핵심: MUSE(불확실성 가중)·NOCTIS(cyclic threshold)가 "단일 전역 코사인 임계" 대체의 최신 두 시도. CNOS ablation은 top-k 평균이 Max 능가 확인._

### G. 사용자 채택 방향 — 1순위 축 (2026-07-20 확정)

Step 2 지형 조사 결과를 바탕으로, 사용자가 **주력 방향 2개 + 신규 방식 1개**를 1순위로 채택했다. 이하는 그 축을 실제 baseline 위에 구체화한 것이다.

#### G-1. Color score를 독립 1순위 채널로 추가

**왜 이 프로젝트에 최적인가 (근거):**
- **shape-bias DINOv2의 사각지대를 정확히 보완.** ViT는 표면 무늬/색을 무시하도록 학습(Naseer 2021) → DINOv2가 못 보는 색 신호를 별도 채널이 담당하면 상보적.
- **오탐 대부분이 "같은 형상·다른 색"이다.** choco(붉은 maroon) vs 갈색 kraft 택배박스, saffron(크림) vs 갈색박스, milk(흰색) vs 갈색박스, Sauce(주황) — 색이 결정적 판별자.
- **YOLO 프롬프트가 이미 색 기반**("maroon box", "yellow can", "orange bottle", "white jug") → 색 축이 파이프라인 의도와 정합.
- **계산 거의 공짜** (히스토그램). masked_appe용 MobileSAM 마스크를 재활용해 전경만 계산.

**방법(권고 설계):**
- 색공간: **HSV의 H·S** 또는 **CIELab의 a·b**(밝기 L 제외 → 조명 강건). MobileSAM 마스크 내부 픽셀만.
- 프로토타입: 객체별 **템플릿 렌더의 색 히스토그램**(이미 42뷰 렌더 존재)을 참조.
- 거리: 히스토그램 **교차(intersection)/EMD(Earth Mover)/Bhattacharyya** → **물리적으로 0~1에 넓게 퍼짐**(갈색 vs 붉은색 = 낮음). classic Swain & Ballard "Color Indexing"(IJCV 1991), 조명불변판 Funt & Finlayson(TPAMI 1995).
- **한계/상보성:** 흰색/무채색 물체(milk, 인형)와 화이트밸런스 변화엔 약함 → **색만으론 부족, appe와 반드시 병용**. 같은-색·다른-물체는 못 가름.

_Source: Swain & Ballard IJCV 1991 · Funt & Finlayson TPAMI 1995 · 프로젝트 config(색기반 프롬프트·마스크 재활용)_

#### G-2. FP/TP를 확실히 가르는 "분리도 높은 새 score 분포" (목표 재정의 2026-07-20)

**목표 정정(중요):** 사용자 목표는 **단순히 0~1 범위를 만드는 것이 아니다.** **범위가 무엇이든, TP와 FP의 두 분포가 겹치지 않고 확실히 갈리는(분리도/separability 높은) 새 분포**를 얻는 것이다. 따라서 성공 지표는 "0~1 캘리브레이션"이 아니라 **두 난제 혼동쌍에서의 분리도(AUC / 분포 간 margin)**이다:

- **혼동쌍 A — 같은 형상·다른 인쇄/색 상자:** 갈색 kraft 택배박스 ↔ 초코하임(maroon). (현 masked_appe로 미분리)
- **혼동쌍 B — 인형 간 혼동:** Bear(갈색) ↔ Rabbit(흰색) ↔ Dinosaur(초록). 모두 둥근 무광 blob → semantic/appe가 못 가름.

**전제:** 코사인 자체는 넓힐 수 없다(도메인 갭이 상한을 누름). 그리고 **넓다고 잘 갈리는 것도 아니다** — 관건은 range가 아니라 **두 클래스 분포의 분리도**. 방법별로 "분리도"와 "범위"를 구분 평가:

| 후보 | 분리 원리 | 두 혼동쌍 분리력 | 범위 | 비고 |
|---|---|---|---|---|
| z-score 정규화(AS-norm) | 객체간 스케일 통일 | ✗ 원분포 분리도 그대로 | 무경계 | 겹침은 안 풀림 |
| RRF 순위융합 | 채널 순위 결합 | △ 채널이 좋으면 좋아짐 | 순위 | 결합 안전패턴 |
| **color 유사도(EMD/교차)** | **색 물리 차이** | **A:✅(maroon≠kraft) B:✅(갈/흰/초록)** | 0~1 | shape-bias 사각지대 정통 |
| **mid-block patch appe(block 선택)** | **texture 신호** | **A:✅ block9 AUC .996 / B:✅ block2 AUC .92(Bear)** | 코사인 | 객체군마다 블록 다름 |
| **inlier ratio(키포인트/기하)** | 인쇄 대응점 수 | A:✅(민박스 ~0) B:△(무광 blob 키포인트 적음) | 0~1 | 상자엔 강, 인형엔 약 |
| **MUSE식 불확실성 변조** | 자신있는 오답 억제 | A/B 공통 개선 | 0~1 근접 | 갈색박스 0.99 오답 down-weight |
| **학습형 verification head** | **라벨로 분리 최대화 직접학습** | **A·B 모두 직접 최적화** | 0~1 확률 | 보유 라벨 활용, 최강수 |

_핵심: 혼동쌍 A(상자)는 color+block9 texture로, 혼동쌍 B(인형)는 color(갈/흰/초록)+block2 texture로 갈린다 → **단일 채널·단일 블록으로는 둘 다 못 잡고, 다채널 융합/학습이 필수**. 이것이 config가 이미 실측한 "block9는 상자엔 최고지만 인형 Bear는 block2가 최고(block9은 0.49 실패)"와 정합._

**MUSE 방식(BOP'24 winner, arXiv:2510.17866) — 채택 후보:** DINOv2 유사도를 **모델 불확실성으로 변조**해 "자신있는 오답(갈색박스 0.99 등)"을 눌러버림. 프로젝트 적용안: **템플릿 42뷰 간 점수 분산 / MC-dropout / feature-norm**으로 불확실성 추정 → `score × (1 − uncertainty)`. "CNOS/SAM-6D/NIDS-Net 대비 FP/TP 분리 최고" 명시 주장.

**학습형 verification head — 프로젝트 최강수(라벨 이미 보유):** 이 프로젝트는 **choco 84건·6객체 501건 hand-label을 이미 축적**했다. (masked_appe[b2/b9/b11], semantic, color_sim, inlier, depth일치도)를 입력으로 **logistic/작은 MLP → P(real)** 학습 → **진짜 0~1 확률 + 객체 간 통일 + per-object 임계 trap 근본 해소**. MUSE 불확실성도 이 head의 feature로 흡수 가능.

**권고 로드맵:**
- **단기:** color 유사도 + (선택)inlier 채널을 추가해 **자연 0~1 채널 확보** → masked_appe와 **RRF 융합** (임계 slip 위험 0, 이미 nms_rank로 검증된 안전패턴).
- **중기:** 보유 라벨로 **logistic verification head** 학습 → 캘리브레이션된 0~1 P(real)로 게이트 교체.
- **MUSE 불확실성**은 중기 head의 입력 feature 또는 단독 변조항으로 편입.

#### G-3. 세 질문 재정합 (채택 방향 반영)

1. **질문①(TP/FP 분리):** appe_gate의 per-object scale 문제 → **color·inlier로 자연 0~1 채널 + RRF**(단기), **logistic verification head로 캘리브레이션 P(real)**(중기), **MUSE 불확실성 변조**로 자신있는 오답 억제.
2. **질문②(도메인 갭 특징):** 백본 교체 무효, frozen ViT-S/14 유지 → **color(G-1)·texture(block 선택)·OCR로 shape-bias 사각지대 재주입**.
3. **질문③(depth+patch):** depth는 "다른-형상" 오탐 전용(같은-형상 상자엔 무력) → inlier 검증으로 0~1 채널화; patch→pixel보다 **블록 선택(block9 실측) + color/texture 채널**이 레버.

→ **"단일 masked_appe 절대게이트"에서 → "color·appe(멀티블록)·inlier·depth 다채널을 RRF/verification-head로 융합해 두 혼동쌍(상자·인형)의 TP/FP 분포가 확실히 갈리는 새 분리도 높은 score"**를 얻는 것이 목표. 성공 판정 = **혼동쌍 A·B의 AUC/margin**(0~1 캘리브레이션은 부차적, verification head가 자연 부수).

**Step 3 진입 목표:** 위 다채널을 실제 `yolo_ism_object_n.py` 2단 게이트에 배치하고, 두 혼동쌍 분리도를 최대화하는 융합/학습 설계 확정.

---

## Integration Patterns Analysis (실제 파이프라인 통합 설계)

> 일반 템플릿(API/마이크로서비스) 대신, **채택 채널을 `yolo_ism_object_n.py`의 2단 게이트에 어떻게 배선·융합·평가하는가**로 적응. 각 채널의 삽입 지점·데이터 스키마·계산비용·평가를 확정한다.

### I-1. 통합 지점 지도 (기존 코드에 무엇을 어디에 꽂나)

현 `recognize()`/`recognize_multi()`는 proposal마다 `res` dict(best_sem, masked_appe, rank_appe, box, mask_area…)를 만들고, Gate1(sem≥0.35)·Gate2(masked_appe≥appe_gate)로 수락한다. **핵심 이점: 새 채널이 필요로 하는 재료가 이미 다 계산돼 있다.**

| 신규 채널 | 재사용하는 기존 자원 | 삽입 지점 | 추가비용 |
|---|---|---|---|
| **color_sim** | Gate2의 **MobileSAM 마스크** | mask 생성 직후, res에 필드 추가 | 무시(히스토그램) |
| **appe 멀티블록** | `dinov2_blocks_forward` 1-pass의 block 2/9/11 | 이미 추출됨 → res에 벡터로 노출 | +0.1%(norm만) |
| **relative-softmax / margin / entropy / view-variance** | 42템플릿 sem·appe 유사도 배열 | semantic_score 계산부에서 배열 보존 | 무시(통계) |
| **inlier_ratio**(선택, 상자용) | best 템플릿 crop | Gate2 통과 후보만 | 중(키포인트 매칭) |
| **depth 일치도**(선택, 다른-형상용) | `--use-geometric` 스캐폴드·CAD·depth | coarse pose 재활용 | 중(렌더/ICP) |
| **verification head** | 위 스칼라 전부 | Gate2 임계를 head P(real)로 교체 | 무시(logistic) |

### I-2. Color 채널 설계 (혼동쌍 B 인형에 최강, A 상자에 보조)

- **색공간: CIELab의 `a*b*`만 사용(`L*` 드롭)** 또는 HSV의 H–S(V 드롭) → 음영/노출 변화(주 nuisance)에 강건. van de Sande 조도 taxonomy상 **어떤 색공간도 illuminant-color(화이트밸런스) 변화엔 불변 아님** → real·render **둘 다 gray-world 정규화** 선처리 필수.
- **저채도 floor:** 채도 매우 낮은 픽셀(회색/흰색) 제거 후 히스토그램 → 흰 토끼의 hue 불안정 억제.
- **히스토그램:** 2D `a*b*`(3D 아님, 희소·밝기혼입 회피), **32×32 bins**, 마스크 내부만(`calcHist` mask 인자), L1 정규화. 객체별 **템플릿 42뷰 색 히스토그램 프로토타입**을 캐시(cls_cache/appe_cache와 동일 패턴, 신규 `color_cache`)하고 **best-of-42 max 유사도**로 비교(특징 매칭과 동형).
- **유사도 지표: Hellinger/Bhattacharyya(범위 [0,1]·대칭·저비용)** 기본 → 다른 [0,1] 채널과 깔끔 융합. render↔real hue offset이 관측되면 **hue 1D EMD**(shift-tolerant) 채널 추가.
- **한계:** 같은-색·다른-물체는 못 가름(색만으론 불충분) → appe와 반드시 병용.

_Source: Swain & Ballard IJCV 1991 http://www.liralab.it/teaching/SINA/slides-current/swain.ballard.1991.pdf · OpenCV compareHist/EMD https://docs.opencv.org/4.x/d6/dc7/group__imgproc__hist.html · van de Sande CVPR'08 https://ivi.fnwi.uva.nl/isis/publications/2008/vandeSandeCVPR2008/vandeSandeCVPR2008.pdf_

### I-3. MUSE식 억제 — 정정된 실제 메커니즘 채용

**중요 정정:** MUSE(arXiv:2510.17866)는 **DINOv2 내부 불확실성을 추정하지 않는다.** 실제는 (Eq.1–10):
- **relative(softmax) 유사도** `S_rel = softmax(S_abs/τ)` (τ=0.02) — 여러 클래스에 고루 매칭되는 proposal을 벌점(=confusability 억제). "자신있는 오답" 억제의 실체.
- **joint** `S_joint = β·S_abs + (1−β)·S_rel` (β=0.8).
- **objectness prior** `S_final = P(O|p)^γ · S_joint` (γ=0.1) — detector objectness를 Bayesian prior로 곱.

→ **본 프로젝트 채용안:** (a) 객체 슬롯들 간 **relative-softmax 정규화**를 채널로 추가(현재 객체별 독립 게이트 → 객체 간 경쟁 도입, per-object scale 문제 완화), (b) **YOLO conf를 objectness prior로 곱**(`conf^γ`). 둘 다 계산 무시 수준이고 즉시 이식 가능.

**공짜 불확실성 프록시(head feature로 추가):** ①42템플릿 유사도의 **top1−top2 margin**(가장 강한 "헷갈림" 신호), ②템플릿 **softmax 엔트로피**, ③**뷰 간 유사도 분산**(진짜 객체는 인접뷰 밴드를 일관 매칭). MC-dropout·deep ensemble은 N× 비용이라 제외.

_Source: MUSE arXiv:2510.17866 (Eq.1–10 직접 확인) · Energy-OOD Liu 2020 arXiv:2010.03759 · MC-dropout Gal 2016 arXiv:1506.02142(비채용 근거)_

### I-4. Verification head — per-object 임계 trap 근본 해소 (중기 최강수)

- **모델: L2 정규화 logistic regression**(6~10 스칼라, 라벨 100~500건엔 표준·과적합 안전·채널별 단조 가중·자체가 확률). MLP/부스팅은 이 데이터 규모에서 과적합 위험 → logistic 우선, 비선형 의심 시 monotonic-constrained GBM을 held-out으로만 검증.
- **입력 feature:** {sem cos, appe cos @ block2/9/11, color-Hellinger, inlier_ratio, YOLO conf, top1−top2 margin, template entropy, view-variance}.
- **출력:** P(real) — **범위 무관·객체 통일·두 혼동쌍 분리 직접 최적화**. Gate2의 `appe_gate` 단일임계를 이 head로 교체 → config가 반복 revert한 per-object 임계 문제를 학습으로 흡수.
- **캘리브레이션:** 소량 라벨이므로 **Platt/logistic**(isotonic은 ≥1000건 필요, 회피), softmax 캘리브레이션엔 temperature scaling. held-out split에서 ECE/reliability로 검증.
- **라벨 자산:** 이 프로젝트는 **choco 84건·6객체 501건 hand-label 보유** → 즉시 학습 가능(신규 라벨링 최소).

_Source: Platt 1999 https://www.microsoft.com/en-us/research/publication/probabilistic-outputs-for-support-vector-machines-and-comparisons-to-regularized-likelihood-methods/ · Guo 2017 arXiv:1706.04599 · RRF(무라벨 fallback) Cormack SIGIR 2009_

### I-5. 선택 채널 — inlier / depth

- **inlier_ratio(혼동쌍 A 상자용):** best 템플릿 crop과 SIFT/ORB(또는 SuperPoint) 매칭 + 호모그래피 inlier 비율. 인쇄면은 대응점 다수·민무늬 kraft는 ~0 → 상자 분리에 강. **단 무광 인형(B)엔 키포인트 희소 → 약함** → head가 알아서 가중.
- **depth 일치도(다른-형상 오탐용):** `--use-geometric` 스캐폴드 활성화, coarse pose로 VSD식 rendered-vs-observed depth 또는 ICP inlier. **주의: 갈색상자↔초코하임은 형상 동일 → depth 무력.** 둥근 물체를 상자로 오인하는 류의 오탐 제거 전용.

### I-6. 데이터 스키마·config 통합

- `res` dict / `CSV_FIELDS`에 `color_sim, appe_b2, appe_b9, appe_b11, margin, entropy, view_var, inlier_ratio, p_real` 추가(디버그·라벨링·평가 일원화; 기존 `sam6d_debug.csv` per-candidate 로깅과 정합).
- config: 신규 키 `color_cache`(객체별), `fusion_mode`(gate|rrf|head), `verifier_weights`(head 가중 경로), `objectness_gamma`, `rel_softmax_tau`. **기존 `appe_gate`는 fusion_mode=gate 하위호환으로 보존**.
- **단계적 배포(하위호환):** ①color+margin/entropy를 res에 기록만(무영향) → ②nms_rank처럼 **순위/보조에만** 반영(임계 slip 0) → ③head를 shadow로 병행 로깅 → ④검증 후 게이트 교체.

### I-7. 계산비용·실시간성

- color·멀티블록·relative/margin/entropy/view-var·head = **사실상 공짜**(마스크·블록 이미 존재, 통계·logistic 뿐). stride10 실시간 예산 영향 무시.
- inlier·depth만 유의미 비용 → **Gate 통과 후보에만** 적용(전수 아님)해 상수화.

### I-8. Step 3 종합 — 통합 청사진

```
YOLO-World(conf^γ objectness prior)
   → Gate1 semantic(슬롯선택, 유지)
   → MobileSAM mask ─┬─ masked_appe(b2/b9/b11)
                     ├─ color_sim(Lab a*b*, Hellinger, best-of-42)
                     └─ [선택] inlier_ratio / depth일치도
   → relative-softmax(객체간) + margin/entropy/view-var
   → Verification head: logistic → P(real)   ← 임계 교체(중기)
   → cross-object NMS(rank = P(real))
```

**단기(라벨 없이 즉시):** color 채널 + relative-softmax/objectness prior + RRF 융합(순위, 임계 slip 0).
**중기(보유 라벨):** logistic verification head로 P(real) 게이트 교체 → 두 혼동쌍 분리도 직접 최적화.
**평가:** 혼동쌍 A(kraft↔choco)·B(Bear/Rabbit/Dino) **AUC/margin**을 1차 지표로, 보유 라벨+`sam6d_debug.csv`로 A/B 측정(ablation: 채널별 기여).
