---
stepsCompleted: [1, 2, 3]
inputDocuments: []
workflowType: 'research'
lastStep: 1
research_type: 'technical'
research_topic: 'SAM-6D DINOv2 기반 ISM의 객체 인식 실패(오인식/FP/FN, 도메인 갭, 형상 편향, 스코어 캘리브레이션)를 다룬 선행 논문 조사'
research_goals: '자체 실측으로 확인한 ISM 실패 모드가 학계에 보고된 바 있는지 확인하고, 인용 가능한 선행 논문·해법 근거를 확보'
user_name: 'ldh'
date: '2026-07-21'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-07-21
**Author:** ldh
**Research Type:** technical

---

## Research Overview

본 리서치는 자체 실측(9-bag audit, 1521건 전수 라벨링, DINOv2 층 스윕, 6-데이터셋 2596프레임 관찰)에서 확인한 SAM-6D ISM 실패 모드가 **학계에 이미 보고된 것인지**를 확인하고, 인용 가능한 선행 논문·해법 근거를 확보하기 위해 수행되었다.

**검증 대상 자체 발견 5종:**
1. 유사 객체 혼동(갈색 택배박스 ↔ choco, saffron → Febreze 45% FP)
2. 렌더-실사 도메인 갭 및 prototype 선택 문제
3. shape 편향(색·질감이 판별에 기여하지 못함)
4. 코사인 점수의 절대 임계 부재/비교불가 (실효 0.55~0.80 압축)
5. proposal miss 기반 FN(후처리 99.8%)

---

## Technical Research Scope Confirmation

**Research Topic:** SAM-6D DINOv2 기반 ISM의 객체 인식 실패를 다룬 선행 논문 조사
**Research Goals:** 자체 실측 실패 모드의 학술적 선례 확인 + 인용 가능 해법 근거 확보

**Scope Confirmed:** 2026-07-21

---

## Literature Landscape Analysis (기술 스택 = 논문 계보)

### 계보 개관: SAM-6D ISM은 CNOS 계열의 파생이다

SAM-6D의 ISM(Instance Segmentation Model)은 독립 발명이 아니라 **CNOS(ICCV 2023 R6D Workshop) 계열의 직계 후손**이다. 계보를 확정하는 것이 중요한 이유는, 우리가 겪는 실패 모드 대부분이 **CNOS가 물려준 설계 결정**에서 유래하기 때문이다.

_원형(CNOS, 2023)_: CAD 모델을 Blender icosphere 기준 **42개 시점**으로 렌더링해 템플릿을 만들고, 입력 RGB에서 SAM/FastSAM으로 proposal을 뽑은 뒤 **DINOv2 cls token 코사인 유사도**로 매칭. BOP 7개 코어 데이터셋 SOTA, 이전 방법 대비 +19.8% AP.
_계승(SAM-6D, CVPR 2024)_: 동일한 렌더-템플릿 + DINOv2 구조에, cls token만 쓰던 CNOS를 넘어 **semantic(cls) + appearance(patch) + geometric** 3항 결합 점수로 확장.
_후속(NIDS-Net IROS 2025 / NOCTIS 2025 / MUSE 2025)_: 동일 파이프라인을 유지하되 proposal 생성기와 **점수 산정 방식**을 각각 손봄.
_Source: https://openaccess.thecvf.com/content/ICCV2023W/R6D/papers/Nguyen_CNOS_A_Strong_Baseline_for_CAD-Based_Novel_Object_Segmentation_ICCVW_2023_paper.pdf , https://arxiv.org/abs/2307.11067_

> **핵심 함의:** 우리 파이프라인의 "42템플릿", "cls token semantic score", "patch appearance score", "절대 임계 없는 코사인" 은 전부 CNOS/SAM-6D가 정의한 것이다. 따라서 개선 논의는 CNOS 계열 후속 논문들이 이미 어디를 건드렸는지와 정확히 겹친다.

### 핵심 선행 논문 5편 (직접 관련)

**1. CNOS (Nguyen et al., ICCV 2023 Workshop)** — 원형 baseline
- 기여: SAM + DINOv2 cls token 템플릿 매칭의 최초 강력 baseline
- 우리와의 관련: 42템플릿·cls token 설계의 출처
- 한계 보고: 논문 자체는 FP 분석이 상세하지 않음 (신뢰도: 높음 — 원문 확인)
_Source: https://arxiv.org/pdf/2307.11067_

**2. SAM-6D (CVPR 2024)** — 우리가 쓰는 그 방법
- 보고된 한계: **"similar-looking but different-sized objects"(예: YCB-V의 clamp류)와 untextured 객체에서 성능이 최적이 아니며, 이론상 depth로 풀릴 법한데도 SAM-6D는 해결하지 못한다"**
- 우리 발견과의 대응: **갈색 택배박스 ↔ choco 흡착 = 정확히 이 실패 모드의 인스턴스**
_Source: https://arxiv.org/pdf/2507.01463 (NOCTIS의 비교 분석 인용), https://arxiv.org/abs/2311.15707_

**3. NIDS-Net (Lu et al., arXiv 2405.17859, IROS 2025)** — proposal 품질 개선 갈래
- 기여: Grounding DINO + SAM으로 proposal 정확도 향상, DINOv2 patch embedding의 **foreground feature average** + weight adapter로 few-shot 과적합 억제
- 우리와의 관련: 우리 FN의 **99.8%가 후처리 단계 proposal miss**라는 발견 → NIDS-Net은 정확히 이 축을 공략한 선례
- 남은 한계(NOCTIS 보고): 더 나은 proposal에도 **"scene object를 잘못 라벨링하거나 과대 bounding box를 생성하는 오분류가 여전히 발생"**
_Source: https://arxiv.org/abs/2405.17859 , https://irvlutd.github.io/NIDSNet/_

**4. NOCTIS (2025, arXiv 2507.01463)** — 점수 산정 개선 갈래, 우리 문제와 가장 근접
- 진단 인용:
  - **"cls token은 매칭 시점(viewpoint)에 대한 정보가 불충분하여 낮은 appearance 값으로 이어질 수 있다"**
  - **"DINOv2 descriptor는 반복 텍스처/유사 외형 부위(예: 동일한 모서리나 면)에 유사한 patch token을 할당해 many-to-one 매칭을 유발한다"**
  - SAM/FastSAM은 **"객체와 그 부분(part)을 구분하는 데 문제가 있다"**
- 해법: (a) **cyclic thresholding(CT)** — patch와 그 왕복(round-trip) patch 간 거리로 사전 필터링, (b) appearance를 **최고 semantic score 템플릿 1장이 아니라 전 템플릿에 대해 평가·집계**, (c) proposal의 bbox/mask confidence를 가중치로 결합
- **우리 발견과의 정면 대응:** 우리가 독립적으로 도출한 **`sem_top5 → sem_mean`(42템플릿 평균)이 choco vs carton AUC .92 → .99** 개선은 NOCTIS의 (b)와 **동일한 처방**이다 (신뢰도: 높음 — 원문 문장 확인)
- 남은 한계: **"객체가 유사하게 생겼으나 크기가 다를 때 최고 성능을 내지 못하며", untextured 산업용 모델에서 여전히 고전**
_Source: https://arxiv.org/abs/2507.01463 , https://arxiv.org/html/2507.01463v4 , https://github.com/code-iai/noctis_

**5. MUSE (Cho, Park, Oh — arXiv 2510.17866, 2025)** — BOP 2025 1위, 점수 캘리브레이션 갈래
- 초록 인용: **"매칭 단계에서 MUSE는 절대(absolute) 유사도와 상대(relative) 유사도 점수를 결합한 joint similarity metric을 사용해 어려운 시나리오에서의 매칭 강건성을 높인다. 최종적으로 유사도 점수는 proposal 신뢰도를 반영하는 uncertainty-aware object prior로 보정된다."**
- patch embedding에 **GeM(generalized mean pooling)** 정규화 적용
- 성과: BOP Challenge 2025 Classic Core / H3 / Industrial **3개 트랙 전부 1위**; BOP-Classic-Core AP 52.0, 0.56 s/image
- **우리 발견과의 정면 대응:** 우리가 도출한 **"절대 임계 대신 cohort/상대 순위(RRF)"** 와 **"class margin(Top1−Top2 gap)"** 방향은 MUSE의 relative similarity + uncertainty prior와 **같은 문제의식**이다
_Source: https://arxiv.org/abs/2510.17866_

### BOP Challenge 공식 보고서 — "검출 단계가 병목" 이라는 공인된 사실

_BOP 2024 공식 보고서(arXiv 2504.02812)_:
- **"unseen object에 대한 2D 검출 정확도는 seen object 대비 현저히 뒤처지며(GDet2023 79.8 AP), 2D 검출 단계가 unseen object의 6D detection/localization 최신 파이프라인의 주된 병목(primary bottleneck)이다."**
- 2024 최고(MUSE) vs 2023 최고(CNOS): **52.0 vs 42.8 AP** (상대 +21%). 그럼에도 seen 대비 여전히 **-35%**
- BOP-H3에서 model-based 6D detection 최고(GigaPose) **31.2 AP** vs 6D localization 최고 **82.1 AR** → 검출이 얼마나 깎아먹는지 정량화
_Source: https://arxiv.org/abs/2504.02812 , https://arxiv.org/html/2504.02812v3_

> **함의:** "ISM 단계가 전체 파이프라인의 병목" 이라는 우리 프로젝트의 전제는 **BOP 공식 보고서가 명시적으로 지지**한다. 인용 가능한 최상급 근거.

### DINOv2 표현 자체의 알려진 편향

- **텍스처 비민감성:** "텍스처의 존재가 DINOv1에는 유익하지만 **DINOv2에는 미미한 개선만 제공**" → 색·질감 정보가 DINOv2 embedding에서 의도적으로 억제됨. 우리 실측의 **"class margin 도입 시 color 기여 0(판정 E)"** 과 정합
- **shape 편향:** DINO 계열은 형상이 커질수록 shape bias 증가 — shape 편향이 진짜 기전이라는 우리 가설의 표현학습 측 근거
- **불변성 설계:** DINOv2는 multi-crop + 강한 photometric augmentation으로 **"viewpoint, illumination, color에 불변인 표현"** 을 학습 → 색으로 두 갈색 박스를 가르는 것은 **설계상 불가능**
- **위치 편향:** DINOv2 feature에 위치 정보가 포함되어 classifier가 이에 과적합될 수 있음 (신뢰도: 중간 — 2차 출처)
_Source: https://arxiv.org/html/2304.07193v2 , https://arxiv.org/pdf/2402.04878 , https://www.emergentmind.com/topics/dinov2-features_

관련 직접 연구: **"Shape-biased Texture Agnostic Representations for Improved Textureless and Metallic Object Detection and 6D Pose Estimation"** (arXiv 2402.04878) — textureless/metallic 객체 6D pose를 위해 shape 편향 표현을 명시적으로 설계한 선행 연구.

### 도메인 갭(렌더 vs 실사) 갈래

- **FoundPose (ECCV 2024)**: DINOv2 patch descriptor를 **이미지 ↔ 사전 렌더 템플릿** 간 매칭해 2D-3D 대응을 세우는 방식. "foundation model을 활용해 sim-to-real 도메인 갭을 별도 대규모 랜덤화 학습 없이 다룬다"는 계열의 대표
- **GFreeDet (BOP 2024)**: **Gaussian Splatting**으로 템플릿을 생성 — CAD 렌더 대신 실사 기반 재구성으로 도메인 갭 자체를 우회하는 접근. 우리 실측의 **"실사 prototype이면 HSV 0.93~1.00 vs 렌더 0.50~1.00"** 발견과 방향이 정확히 일치
_Source: https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/03742.pdf , https://arxiv.org/html/2412.01552_

### 점수 캘리브레이션 / open-set 거부 이론

우리 문제("코사인 점수에 절대 임계가 없다")는 일반 ML에서 **Open-Set Recognition(OSR)** 으로 정식화되어 있다.
- 표준 결정규칙: 클래스 prototype에 대한 코사인 최대값이 임계 τ 초과면 할당, 아니면 unknown 거부
- 캘리브레이션: **EVT(극단값 이론)** 기반 점수 보정으로 신뢰수준에 따라 임계 결정
- **Threshold-consistent margin loss (arXiv 2307.04047)**: 서로 다른 테스트 분포에 **동일한 보편 임계**를 적용해도 유사한 FAR/FRR을 얻도록 임베딩을 학습 → 우리의 "객체마다 실효 범위가 다르다" 문제의 정식 해법
- 평가 지표: AUROC, FPR@95%TPR, **OSCR** — 우리가 쓴 AUC 기반 평가와 호환
_Source: https://arxiv.org/pdf/2307.04047 , https://www.emergentmind.com/topics/open-set-recognition-osr_

### 논문 지형 요약표

| 우리 발견 | 선행 논문 존재? | 대표 논문 | 정합도 |
|---|---|---|---|
| 유사 객체(갈색 박스) 흡착 FP | ✅ 명시적으로 보고됨 | SAM-6D, NOCTIS | 높음 |
| 반복 텍스처 → patch many-to-one | ✅ NOCTIS가 진단·해결 | NOCTIS (CT) | 높음 |
| sem_top5 → sem_mean(42템플릿) 개선 | ✅ 동일 처방 존재 | NOCTIS (appearance aggregation) | **매우 높음 — 독립 재발견** |
| 절대 임계 부재 / 상대순위·RRF 필요 | ✅ | MUSE (relative + uncertainty prior), OSR/EVT | 높음 |
| color 기여 0 = shape 편향이 기전 | ✅ 표현학습 측 근거 | DINOv2 원논문, arXiv 2402.04878 | 중간~높음 |
| 렌더-실사 갭, 실사 prototype 우위 | ✅ | GFreeDet(3DGS), FoundPose | 높음 |
| proposal miss가 FN의 지배 요인 | ✅ | NIDS-Net, BOP 2024 보고서 | 높음 |
| DINOv2 **블록 9 > 블록 11** (층 선택) | ⚠️ 직접 선례 미발견 | — | **novelty 후보** |
| Hue -5.6° 계통 편향 / PLY 정점색 prototype | ⚠️ 직접 선례 미발견 | — | **novelty 후보** |

---

## Integration Patterns Analysis (해법의 파이프라인 이식 관점)

이 절은 "논문이 있느냐"를 넘어 **각 논문의 메커니즘이 우리 코드의 어느 인터페이스에 꽂히는가**, 그리고 **이식 비용**을 분석한다.

### 기준선 확정: SAM-6D ISM 원논문의 정확한 점수 수식

원논문(arXiv 2311.15707v2) 확인 결과, ISM 매칭 점수는 다음과 같다:

```
s_sem  = top-K 평균 { cos(f_I^cls, f_Tk^cls) }        , k = 1..N_T (템플릿 수)
s_appe = (1/N_patch) Σ_j  max_i cos(f_I,j^patch, f_T_best,i^patch)
         └ f_T_best = s_sem이 최대인 "단 하나의" 템플릿
s_geo  = IoU(B_proposal, B_projected)
s_m    = (s_sem + s_appe + r_vis · s_geo) / (1 + 1 + r_vis)
```

_핵심 관찰 1:_ **appearance는 semantic argmax 템플릿 1장에만 의존한다.** 즉 semantic이 잘못된 템플릿을 고르면 appearance는 그 오류를 물려받는다 — 오류가 직렬로 전파되는 구조.
_핵심 관찰 2:_ **결합은 단순 산술평균이며, 어떤 항도 확률이 아니다.** 절대 임계가 없는 근본 원인이 수식에 그대로 드러난다.
_핵심 관찰 3:_ 원논문에는 **명시적 failure case 절이 없다.** occlusion이 s_geo 신뢰도를 해친다는 언급을 r_vis로 처리할 뿐, 근본 한계 논의가 부재하다 (신뢰도: 높음 — 원문 확인).
_Source: https://arxiv.org/html/2311.15707v2_

> 우리 파이프라인은 여기서 더 이탈해 있다: 운영 경로는 **YOLO + ViT-S/14 2단 게이트**이고 사용자가 보는 0.6~0.7은 `masked_appe`다. 즉 s_geo가 없는 축약판이며, 아래 이식 논의는 이 실제 구조 기준으로 읽어야 한다.

### 이식 대상 A — NOCTIS의 점수 재설계 (가장 저비용·고효과)

NOCTIS 전체 수식 (원문 확인):

```
s_sem  = top-5 평균 cos(cls_proposal, cls_template)      ← SAM-6D와 동일
s_appe^sub(T) = (1/N_patch) Σ_i [ 1(cdist(I,T,i) ≤ δ_CT) · max_j cos(patch_i, patch_j^T) ]
s_appe = MAX_over_all_templates ( s_appe^sub(T) )        ← ★ 여기가 SAM-6D와 다름
conf_p = avg(bbox_conf, mask_conf)
s_obj  = [ (s_sem + w_appe · s_appe) / 2 ] · conf_p      , w_appe = 2, δ_CT = 5
```

**cyclic thresholding(CT) 정의:** proposal patch `s` → 템플릿 최적 patch `t` → 다시 proposal 최적 patch `u` 로 왕복시켜, 그리드 상 `s`와 `u`의 유클리드 거리가 δ_CT=5 이하인 patch만 유효로 인정. 반복 텍스처가 만드는 many-to-one 매칭을 **점수 계산 전에 제거**한다.

**ablation (BOP 7-core mean AP):**

| 구성 | AP |
|---|---|
| semantic only | 0.464 |
| semantic + appearance (w=1) | 0.494 |
| semantic + confidence | 0.494 |
| semantic + appe(w=1) + conf | 0.512 |
| **full (w=2) + CT filter** | **0.520** |

런타임 0.990 s/image (RTX 4070 12GB).
_Source: https://arxiv.org/html/2507.01463v4_

**우리 발견과의 정밀 대조 (Step 2 주장 보정):**
Step 2에서 "우리의 `sem_top5 → sem_mean`(42평균)이 NOCTIS와 동일 처방"이라 적었으나, 원문 정독 결과 **정확히 동일하지는 않다**:
- NOCTIS도 semantic은 여전히 **top-5 평균**을 쓴다 (우리처럼 42 전체 평균이 아님)
- NOCTIS가 바꾼 것은 **appearance를 semantic-argmax 템플릿 1장이 아니라 전 템플릿에 대해 계산 후 MAX 집계**한 것
- **공유하는 통찰:** "점수를 argmax 템플릿 1장에 묶지 말고 템플릿 집합 전체로부터 뽑아내라"
- **차이:** 우리는 semantic 쪽을 mean으로, NOCTIS는 appearance 쪽을 max로 풀었다 → **우리 sem_mean(42)은 아직 문헌에 없는 변형**일 가능성이 있고, 이는 novelty 후보로 승격 가능 (신뢰도: 중간 — 추가 문헌 확인 필요)

**이식 비용 평가:**

| 메커니즘 | 우리 코드 접점 | 비용 | 예상 효과 |
|---|---|---|---|
| CT patch 필터 | masked_appe 계산부에 왕복 매칭 1회 추가 | patch 유사도 행렬 재사용 → **거의 무료** | 반복 텍스처(갈색 박스 표면!) FP 직격 |
| appearance MAX-over-templates | 42템플릿 비교 이미 수행 중 (**+0.026 ms 실측**) | **무료** | argmax 오류 전파 차단 |
| conf_p 가중 | YOLO conf 이미 보유 | 무료 | proposal 품질 반영 |
| w_appe = 2 | 상수 | 무료 | ablation상 +0.008 AP |

> **최우선 실행 후보: CT 필터.** 우리 최대 FP인 "갈색 택배박스 흡착"은 **평평하고 반복적인 골판지 표면** — NOCTIS가 CT로 정확히 겨냥한 many-to-one 매칭 상황이다.

### 이식 대상 B — MUSE의 상대 유사도 + 불확실성 prior

MUSE(BOP 2025 Classic Core/H3/Industrial **3트랙 1위**) 구성:
- **GeM(generalized mean pooling)** 으로 patch embedding 정규화 → global + local 표현 동시 포착
- **joint similarity = absolute + relative** 결합
- **uncertainty-aware object prior** 로 proposal 신뢰도 반영
_Source: https://arxiv.org/abs/2510.17866_

**우리 방향과의 대응:** 우리가 도출한 **class margin(Top1−Top2 gap)** 과 **RRF 상대순위** 는 MUSE의 relative similarity와 같은 계열. 다만 우리 실측에서 **"클래스 동시수락 8.78%, Top1−Top2 gap이 객체별로 8.7배 차이"** 라는 정량 근거를 이미 확보했으므로, MUSE를 그대로 이식하기보다 **우리 라벨 338박스로 relative score를 직접 캘리브레이션**하는 편이 유리하다.

_주의:_ MUSE 원문의 relative similarity 정의는 arXiv HTML이 미제공(404)이고 PDF 파싱 실패 — **수식 미확인 상태** (신뢰도: 낮음 — 초록 수준). 이식 전 원문 정독 필요.

### 이식 대상 C — GFreeDet의 3DGS 템플릿 (도메인 갭 우회)

GFreeDet은 CAD 렌더 대신 **onboarding 영상으로 Gaussian Splatting 재구성 → 그 Gaussian object로 템플릿 렌더**. BOP 2024 model-free 2D detection에서 **best overall + best fast** 수상, BOP-H3에서 **CAD 기반 방법과 대등한 성능**.
_Source: https://arxiv.org/html/2412.01552v4 , https://arxiv.org/abs/2412.01552_

**우리 실측과의 연결:** "HSV는 **실사 prototype이면 전 객체 0.93~1.00, 렌더는 0.50~1.00**" — 즉 우리는 렌더-실사 갭을 **색 채널에서 정량 측정**했고, GFreeDet은 그 갭을 **템플릿 생성 방식 자체를 바꿔** 없앤다.

**이식 비용:** 높음. 객체별 onboarding 영상 촬영 + 3DGS 학습 파이프라인 필요.
**저비용 대안(우리가 이미 시도):** 렌더 템플릿에 **공통 색 보정(-6°, 채도 ×1.3)** 적용 → HSV .847 → .959. GFreeDet의 1/100 비용으로 색 축의 갭 대부분을 회수. **이 "보정 상수 접근"은 문헌에서 미발견 — novelty 후보.**

### 이식 대상 D — NIDS-Net의 proposal 품질 (FN 축)

우리 FN의 **99.8%가 후처리 단계 proposal miss**(raw YOLO는 0.2%)이므로, 점수 개선(A/B)은 FN에 무력하다. FN은 별도 축이다.
- NIDS-Net: Grounding DINO + SAM으로 proposal 확보, **foreground feature average** + weight adapter
- 다만 NOCTIS는 NIDS-Net도 **"scene object 오라벨링·과대 bbox"** 가 남는다고 보고
- BOP 2024 보고서: **2D 검출이 unseen object 파이프라인의 primary bottleneck** (seen 대비 -35%)
_Source: https://arxiv.org/abs/2405.17859 , https://arxiv.org/abs/2504.02812_

**우리 상황의 특수성:** 우리 병목은 proposal *생성*이 아니라 **후처리(NMS·필터)** 다. 이미 `--nms-rank appe` 로 NMS 승자 결정을 YOLO conf → appearance 증거로 바꿔 인형 14/22 → 22/22를 비용 0으로 회복한 전례가 있다. **문헌은 생성기 교체를 말하지만, 우리 데이터는 후처리 튜닝이 더 싸다고 말한다.**

### 통합 우선순위 (비용 대비 효과)

| 순위 | 조치 | 출처 | 비용 | 대상 실패 모드 |
|---|---|---|---|---|
| 1 | **CT patch 필터** 도입 | NOCTIS | 거의 0 | 갈색 박스 FP (반복 텍스처) |
| 2 | **appearance MAX-over-42-templates** | NOCTIS | 0 (실측 +0.026ms) | argmax 오류 전파 |
| 3 | **conf_p 가중 + w_appe=2** | NOCTIS | 0 | 전반 AP |
| 4 | **relative score 캘리브레이션** (자체 338 라벨) | MUSE 계열 | 낮음 | 절대 임계 부재 |
| 5 | **렌더 색 보정 상수** 정식화 | 자체(novelty) | 이미 완료 | 도메인 갭 |
| 6 | proposal 후처리 재설계 | NIDS-Net/BOP | 중간 | FN 62% recall |
| 7 | 3DGS 템플릿 | GFreeDet | 높음 | 도메인 갭 근본 |

---

<!-- Content will be appended sequentially through research workflow steps -->
