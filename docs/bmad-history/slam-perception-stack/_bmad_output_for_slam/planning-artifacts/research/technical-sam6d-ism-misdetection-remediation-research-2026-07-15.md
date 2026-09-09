---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments: ['_bmad_output_for_slam/test-artifacts/report-sam6d-reloc-png-audit-2026-07-15.md']
workflowType: 'research'
lastStep: 6
research_type: 'technical'
research_topic: 'SAM-6D ISM 오인식(FP/FN) 해결 방법론'
research_goals: 'P1 갈색 택배박스 흡착(FP 377) / P2 거리·노출 recall 붕괴 / P3 인형 잔여 오류를 실제로 해결하는 구현 가능한 방법론 도출. 측정·감사가 아닌 해결책.'
user_name: 'ldh'
date: '2026-07-15'
web_research_enabled: true
source_verification: true
---

# 버리고 있던 신호를 되찾기: SAM-6D ISM 오인식 해결 방법론 종합 기술 리서치

**Date:** 2026-07-15 · **Author:** ldh · **Research Type:** technical

---

## Executive Summary

2,596프레임 전수 감사는 **FP 680 / FN 1,392**를 확정했지만, 해결책은 하나도 주지 않았습니다. 감사가 확정한 것은 오히려 **막다른 길들**이었습니다 — appe 임계값은 원리적으로 불가(관측조건이 정체성보다 점수를 더 좌우), PEM은 검증자가 아님(민무늬 택배박스에 pose_score 0.99~1.00), 그리고 오늘 시도한 "객체당 단일 슬롯 해제"는 FN을 하나도 회복하지 못하고 택배박스 FP만 9개 늘렸습니다. 본 리서치는 "무엇이 문제인가"가 아니라 **"이미 가진 자산으로 무엇을 새로 만들 수 있는가"**를 묻습니다.

**결론은 하나로 수렴합니다: 살 것이 없습니다. 우리는 이미 보유한 판별 신호를 구조적으로 버리고 있었고, 그 신호를 되찾는 세 가지 경로가 전부 신규 의존성 0입니다.** 첫째, 우리 파이프라인의 1차 게이트(`semantic_score`)는 DINOv2 **CLS 토큰**을 쓰는데, CLS/pooled 표현은 **shape-biased**임이 2025년 피어리뷰 2건으로 문서화돼 있습니다 — 줄무늬 초코하임과 민무늬 택배박스는 *coarse shape가 같은 상자*이므로 이 게이트가 둘을 못 가르는 것은 **정상 동작**입니다. 둘째, 우리는 이미 **42장의 텍스처 렌더와 그 회전(`cam_poses_level0.npy`)을 보유**하고 있고 `yolo_ism.py:639`가 이미 그 회전을 읽고 있습니다 — **PEM 비용 없이 pose 정렬 텍스처 비교가 ISM 단계에서 가능**하며, 이는 SAM-6D가 이미 shape-IoU에 쓰는 슬롯의 확장일 뿐입니다. 셋째이자 가장 강력하게, 우리는 **절대 임계값이 실패하고 상대 순위가 성공한다는 것을 이미 실증**했습니다(NMS 순위를 YOLO conf→appe로 바꿔 인형 라벨 14/22→22/22, 임계 튜닝 0). 이는 우연이 아니라 생체인식의 verification(1:1, 임계 필요) vs identification(1:N, argmax, 보정 불요) 구분, 그리고 화자인식의 **cohort/UBM 정규화**라는 20년 선례를 가진 형식적 원리입니다.

**그러나 정직한 한계가 셋 있습니다.** ① 코드 실측 결과 우리 DINOv2는 **ViT-S/14 depth=12**이지 FoundPose가 측정한 ViT-L/23이 아니므로 "18층" 처방은 **존재하지 않으며**, 층 스윕 효과는 미지수입니다[LOW]. ② `masked_appe_score`는 이미 patch token 기반이므로(웹 리서치의 "pooled를 쓴다"는 추정은 **우리 코드에 대해 거짓**) 개선 여지가 예상보다 작습니다. ③ **다중뷰 일관성은 P1을 못 고치고 오히려 강화합니다** — 택배박스는 진짜 정적 물체라 깨끗이 삼각측량되고 우리 Bernoulli `r`에서 높은 존재확률로 수렴합니다. 그리고 **cold-start FN은 어떤 시간 기법으로도 회복 불가**입니다.

**Key Technical Findings:**
- **shape 편향이 진짜 메커니즘** — "텍스처 맹목"이 아님. CLS 게이트가 상자 두 개를 유사하다 하는 건 설계대로의 동작 **[HIGH]**
- **검증은 PEM 앞에 둔다** — Viola-Jones→Cascade R-CNN→SAM-6D/GigaPose/MegaPose/FoundPose 전부 비싼 단계 앞에서 필터. PEM 뒤 기하 검증은 pose_score와 같은 실패 **[HIGH]**
- **상대 비교(cohort/UBM)가 무라벨 환경의 정답** — 절대 임계값은 보정을 요구하고 우리는 보정할 라벨이 없음 **[HIGH]**
- **우리 임계값 실패의 처방이 존재** — Hartigan's dip test + bootstrap + **소스별 부분집합 일관성**. 셋 다 라벨 불요. 3번이 "표본 90%가 한 데이터셋"을 사전 검출했을 검사 **[HIGH]**
- **P2 최대 레버는 비용 0의 재튜닝** — Mugcup 9FP/81FN·Sauce 1FP/75FN은 하류 게이트가 아니라 **YOLO-World 자체 임계값이 proposal을 굶기는** 신호
- **과노출은 우리 탓이 아님** — YOLO-World는 최경미 밝기 손상에서도 AP 39.3→33.5, 심하면 23.4 **[HIGH]**
- **Conformal prediction은 사도** — 라벨된 calibration set 필수 **[HIGH]**

**Technical Recommendations (비용 오름차순, 전부 신규 의존성 0):**
1. **DINOv2 층 스윕 0~11** (반나절) — 실패해도 저렴, 성공하면 최대 효과
2. **마스크 색 히스토그램 + 엣지 밀도** (반나절) — 줄무늬=이봉 vs 민무늬=단봉. LINEMOD 선례 +13%
3. **택배박스를 객체로 등록해 cohort 경쟁** (1일) — **P1 주력**. 기존 appe-rank NMS가 자동 해소
4. **YOLO-World 자체 conf 임계 객체별 하향** (0) — **P2 주력**
5. **모든 임계값에 dip test + bootstrap + 소스별 일관성 검사 의무화** — 재발 방지

---

## Table of Contents

1. [자산 인벤토리 (디스크 실측)](#0-자산-인벤토리-디스크-실측-기억-아님)
2. [Technology Stack Analysis](#technology-stack-analysis) — 판별 신호·render-and-compare·pose 검증·open-vocab·시간전파
3. [⚠️ 정정 (코드 실측으로 웹 리서치 2건 반증)](#️-정정-step-03-코드-실측--위-technology-stack-analysis의-2개-주장이-반증됨)
4. [Integration Patterns Analysis](#integration-patterns-analysis) — 환경 경계·JSON 계약·확장 지점 10개
5. [Architectural Patterns Analysis](#architectural-patterns-analysis) — 검증기 배치·무라벨 결정 구조
6. [Implementation Approaches](#implementation-approaches-and-technology-adoption) — 로드맵·GT 없는 QA·위험
7. [Strategic Technical Recommendations](#strategic-technical-recommendations)
8. [Future Technical Outlook](#future-technical-outlook)
9. [Research Methodology and Source Verification](#research-methodology-and-source-verification)
10. [다음 액션 제안](#다음-액션-제안)

---

## Technical Research Scope Confirmation

**Research Topic:** SAM-6D ISM 오인식(FP/FN) 해결 방법론
**Research Goals:** P1(갈색 택배박스 흡착 FP 377) / P2(거리·노출 recall 붕괴) / P3(인형 잔여 오류)를 **실제로 해결**하는 방법론. 측정·감사가 아님.

**사용자 지정 스코프 한정:** **"이미 가진 자산 재활용"** — 새 모델 학습·데이터 수집 없이, 보유 자산만으로 되는 방법 우선.

**Research Methodology:**
- 웹 기반 현재 데이터 + 출처 검증, 핵심 주장 다중 출처 교차 확인
- 불확실 정보에 신뢰도 등급(HIGH/MED/LOW) 명시
- 병렬 리서치 에이전트 3종(render-and-compare / open-vocab 판별축 / 시간전파) 동시 실행

**Scope Confirmed:** 2026-07-15

---

## 0. 자산 인벤토리 (디스크 실측, 기억 아님)

리서치의 모든 판단이 여기 근거합니다.

| 자산 | 실측 결과 | 함의 |
|---|---|---|
| **Vertex-color CAD PLY** | 객체별 ~60MB ascii, 589,824 verts, `x/y/z + nx/ny/nz + s/t(UV) + red/green/blue` | 텍스처가 **정점 색으로 살아있음**. UV 텍스처맵 배선 불필요 |
| **렌더 템플릿** | 객체 11종 × **42장**, 각각 `rgb_N.png` + `mask_N.png` + **`xyz_N.npy`(픽셀별 3D 좌표)** | **텍스처 렌더를 이미 보유.** xyz로 각 템플릿의 시점(R) 복원 가능 |
| **렌더러** | `trimesh`/`cv2`/`torch` OK. **`pyrender`/`nvdiffrast`/`open3d`/`OpenGL` 전부 미설치** | 실시간 렌더 경로는 신규 의존성 필요 → **템플릿 경로가 우선** |
| **SLAM** | ORB-SLAM3 궤적(2,730 pose, loop closure 검증) | 프레임별 카메라 pose 보유 |
| **ObjectMemory** | PoseBuffer + 시간정합 + 다중인스턴스 association + **Bernoulli 존재확률 r + FOV-aware P_D**, 87 tests | 랜드마크 재투영의 안전장치가 **이미 구현돼 있음** |
| **PEM** | 검출별 6D pose (R,t) 산출 중 | render-and-compare의 입력이 이미 있음 |

---

## Technology Stack Analysis

> **주의**: 본 스킬의 표준 하위 절(프로그래밍 언어 / DB / 클라우드 / IDE)은 6D pose 파이프라인 도메인에 부합하지 않아, Level-2 구조를 유지한 채 **하위 절을 도메인에 맞게 매핑**했습니다.

### 🔴 최우선 발견 — DINOv2의 판별 신호를 우리가 버리고 있다

**우리 진단("DINOv2가 텍스처를 버린다")은 맞았지만, 메커니즘이 더 정확히 규명됐고 그 결과 해법이 바뀝니다.**

- DINOv2의 **CLS/pooled 임베딩은 shape-biased**임이 문서화돼 있습니다. NeurIPS 2025 "Configural Shape Score" 연구에서 DINOv2·EVA-CLIP·SigLIP2가 **shape 민감도 최상위**로 측정됐습니다 — *"their global-consistency objectives appear uniquely effective at instilling holistic shape representations."* **[HIGH — 2025년 독립 2개 출처 일치]**
  _Source: https://arxiv.org/abs/2507.00493v1 , https://arxiv.org/pdf/2503.12453_
- **이것이 정확히 우리 증상입니다**: 줄무늬 초코하임과 민무늬 택배박스는 **coarse shape가 동일한 상자**이므로, shape 편향 전역 서술자가 둘을 유사하다고 채점하는 것은 *정상 동작*입니다. 문제는 "텍스처 맹목"이 아니라 **shape 편향**입니다.
- **결정적 반전**: 이 편향은 **최종층 / CLS-pooled 표현의 속성이지, 모든 층의 속성이 아닙니다.** FoundPose(ECCV 2024)가 이를 통제 실험으로 측정 — **23층 중 18층**의 patch token이 최종층보다 우수하며, 그 이유가 *"these descriptors carry stronger positional information than descriptors from the last layer, and show importance when semantic information is ambiguous due to object symmetries or **a lack of texture**"*. **[HIGH — 피어리뷰 논문의 통제된 ablation]**
  _Source: https://arxiv.org/abs/2311.18809 , https://github.com/facebookresearch/foundpose_

> **결론: 우리는 이미 로드해서 돌리고 있는 모델의 "잘못된 탭"을 읽고 있습니다.** `masked-appe` 게이트는 SAM-6D/CNOS ISM 표준 설계대로 pooled/CLS 수준 임베딩을 쓰는데, 그게 바로 shape 편향이 문서화된 그 표현입니다. **중간층 patch token은 우리가 필요한 텍스처/위치 신호를 보존한다고 문서화돼 있습니다.** 신규 모델 0, 신규 의존성 0, 다운로드 0 — **읽는 층만 바꾸면 됩니다.**

### Render-and-Compare: 우리 자산으로 가능한가

**경로 (b) — 템플릿 최근접 비교 (렌더러 불필요) ✅ 즉시 가능**

- 이미 42장의 텍스처 렌더 + `xyz_N.npy`로 각 템플릿의 R 복원 가능. PEM의 추정 R에서 42개 템플릿 R까지 geodesic 거리 → 최근접 선택 → `mask_N.png`로 마스킹한 `rgb_N.png`와 실제 crop 비교.
- **문헌 검증됨**: SAM-6D ISM 자체가 이 구조이고, **GigaPose(CVPR 2024)**가 이를 형식화·검증 — out-of-plane 회전을 **템플릿 최근접 탐색**으로 찾고 in-plane은 별도 처리. _Source: https://arxiv.org/abs/2311.14155_
- **문서화된 실패 모드**: SAM-6D 논문 스스로 *"a sharp drop after three views and fluctuates thereafter, indicating higher sensitivity to viewpoints"* — 42분할(≈15~20° 간격)의 양자화 오차가 실재. 최근접 템플릿 선택만으로는 **in-plane 회전·스케일·평행이동이 해결되지 않음**.
- **해법(GigaPose 설계에서 직수입)**: raw 템플릿 vs raw crop을 비교하지 말고, 최근접 템플릿은 **out-of-plane 전용**으로 쓰고, 잔여 **in-plane 회전+스케일+평행이동**을 PEM의 R,t와 템플릿 R + 카메라 내부파라미터로 해석적으로 계산해 **2D affine/homography로 워핑한 뒤** 비교. 3D 렌더러 없이 양자화 오차 대부분 제거.
- **비용**: 추가 연산 ≈ 0 (템플릿은 디스크에 이미 있음). 구현 ≈ 1일.
- **P1 예상 효과**: **MED-HIGH** — 정렬이 완벽하지 않아도 "줄무늬 있음/없음"은 분리 가능.

**경로 (a) — PLY 실렌더 (정확도 상위 옵션) ⚠️ 신규 의존성**

| 렌더러 | headless GPU | 우리 스택 적합도 |
|---|---|---|
| **nvdiffrast** | CUDA 래스터라이저, **OpenGL 컨텍스트 불필요** | **최적** — torch+GPU 이미 있음, **정점 색 보간 직접 지원**(우리 PLY가 정점 색이라 UV 배선 불필요) |
| pyrender | EGL/OSMesa 필요 | headless 서버 OpenGL/EGL 셋업 부담 |
| Open3D OffscreenRenderer | Filament + 자체 EGL | 무거운 신규 의존성 |
_Source: https://github.com/NVlabs/nvdiffrast , https://nvlabs.github.io/nvdiffrast/_

- **비용**: 프레임당 수 ms 예상이나 **589k verts 메시에 대한 인용 가능한 벤치마크 없음 [LOW-MED, 실측 필요]**. 2,596프레임 오프라인 배치엔 어느 쪽이든 무의미한 수준.
- **권고**: **(b)를 MVP로 먼저 출시**, 양자화가 한계로 확인되면(FN이 템플릿 경계 pose에 몰리면) (a)로 업그레이드.

### 판별 지표: 무엇이 줄무늬 상자와 민무늬 택배박스를 실제로 가르는가

| 지표 | 문헌 지위 | 즉시 가능 | P1 효과 | 비용 |
|---|---|---|---|---|
| **DINOv2 중간층 patch token** | **FoundPose가 이 모호성 부류에 대해 직접 검증** | **YES — 같은 백본, 다른 층** | **HIGH — 본 보고서 최고 근거** | **≈0. 신규 모델·의존성 없음** |
| **마스크 색 히스토그램** (HSV/Lab, χ²/Bhattacharyya) | 고전 CBIR. 6D pose 논문엔 드묾 → 우리 케이스엔 외삽 | YES (`cv2.calcHist`) | **HIGH** — 갈색/흰색 줄무늬는 hue/value **이봉분포**, 민무늬 tan은 **단봉** | 사소, <1ms |
| **마스크 내 edge/gradient 밀도** | **문헌 직접 선례**: Hinterstoisser LINEMOD + 색 검증이 정탐률 **+13%** | YES (`cv2.Sobel/Canny`) | **HIGH** — 인쇄 글자+줄무늬 엣지 vs 평평한 판지 | 사소 |
| Photometric NCC/L1 | DeepIM·MegaPose의 핵심 신호(용도는 pose 정제) | YES (정렬 후) | MED-HIGH — 정렬 품질 의존 | 저 |
| SSIM | 일반 화질 지표 | YES (skimage 소규모 신규 의존) | MED | 저 |
| LPIPS / CLIP / LBP | 이 과제엔 미검증 외삽 | NO (신규 의존성) | LOW-MED | 중 |
_Source(LINEMOD +13%): https://www.researchgate.net/publication/228842823_Gradient_Response_Maps_for_Real-Time_Detection_of_Texture-Less_Objects_

> **권고: 하나 고르지 말고 가장 값싸고 근거 강한 3개를 결합** — ① 마스크 색 히스토그램 + ② edge/gradient 밀도 + ③ DINOv2 중간층 patch token. **셋 다 한계비용 ≈0, 신규 의존성 0**, 셋 다 직접/근접 문헌 선례 보유.

### 6D Pose 검증/거부 (SAM-6D·MegaPose 이후)

- **ZePHyR (ICRA 2021)** — 우리 요구와 가장 근접한 기성품. 후보 pose로 모델을 투영하고 **투영된 각 모델 점마다 색+기하 차이를 함께 추출**해 점 기반 네트워크가 hypothesis 품질 점수를 회귀. Zero-shot(신규 객체 재학습 불요). **사전학습 가중치 공개.** **[HIGH]**
  _Source: https://arxiv.org/abs/2104.13526 , https://github.com/r-pad/zephyr_
  → **YES-with-moderate-work** (1~3일). 리스크: YCB-Video 학습 → 우리 스낵박스로의 도메인 갭 미검증.
- **MegaPose coarse classifier** — 렌더 RGB + 관측 RGB를 CNN에 넣어 채점하지만 **"객체 정체성은 옳다"를 전제**하고 pose 품질만 봄. **우리 PEM 진단이 아키텍처 계열 전체에 해당함이 확인됨.** 드롭인 해법 아님.
- **BOP Challenge 2024** 우승: **FreeZeV2.1**(FreeZe 기반, 2023 최고 대비 +22%). unseen 2D 검출 최고: **MUSE**(CNOS 대비 +21~29%). **[MED-HIGH]**
  _Source: https://arxiv.org/abs/2504.02812 , https://bop.felk.cvut.cz/home/_
- **FreeZe (ECCV 2024)** — 기하 파운데이션 모델(GeDi) + DINOv2를 융합, 학습 불요. 단 DINOv2를 **대응점 탐색에 융합**할 뿐 **별도 정체성 거부 단계는 없음**.
- **❌ 고전 기하 hypothesis verification (Aldoma GHV, ObjRecRANSAC, ICP fitness) — P1에 무효.** 둘 다 상자라 기하 inlier/occlusion 추론으로는 줄무늬를 볼 수 없음. 다중 hypothesis 경쟁이라는 *보완* 아이디어로만 유효.
  _Source: https://link.springer.com/chapter/10.1007/978-3-642-33712-3_37_

### Open-Vocab 판별축: 프롬프트 밖에서 신호 얻기

- **명시적 경쟁/네거티브 클래스 ✅ 즉시 가능, 수분 작업.** 택배박스를 `"cardboard shipping box"`로 **자기 이름을 갖게 해서** "maroon box"를 훔치지 못하게 함. **문서화된 표준 기법**: Roboflow YOLO-World 가이드 — *"Include objects you don't care about detecting to prevent false positives... adding 'car' as a class prevented the model from misclassifying vehicles as plates."* **우리 P1과 거의 동형.** **[HIGH]**
  _Source: https://blog.roboflow.com/yolo-world-prompting-tips/_
  ⚠️ 우리는 **프롬프트별 개별 pass** 구조라 네거티브가 pass 간 전파되지 않음 → **각 pass 안에 넣어야 함**. 이는 오히려 장점(객체별 네거티브 세트 튜닝 가능). 알려진 실패모드: 네거티브가 타깃과 **의미적으로 너무 가까우면 정탐도 억제**.
- **빈 문자열 `""` 배경 흡수 클래스** — FP 감소 보고. **[MED]** _Source: https://arxiv.org/pdf/2604.09920_
- **OCR / scene-text 존재 신호 ✅ 강력하고 직교적.** 진짜 상자엔 큰 인쇄 글자, 택배박스엔 없음. **텍스트 인식 불요 — "텍스트 영역이 있는가"만** (CRAFT/DBNet/PaddleOCR det head, ~5MB, CPU 가능). **연속 유사도 점수의 임계값이 아니라 범주형 신호이므로, "appe 임계값 불가" 결론을 우회합니다.** **[MED-HIGH, 우리 케이스 벤치마크는 없음 → 실측 필요]**
- **CLIP vs DINOv2 — 핵심 근거**: 2025년 통제 연구가 정확히 우리 두 케이스를 비교 — *"CLIP is more sensitive to high-level visual semantics — such as object type and **embedded text** — while DINO is more responsive to low-level visual attributes like colors and styles."* **[HIGH]**
  _Source: https://arxiv.org/html/2510.11835v1_
  → **인쇄 글자 단서엔 CLIP/SigLIP2 계열이 유리**. SigLIP 2의 **NaFlex 변형은 OCR/문서 이해에 명시적으로 튜닝**됨. _Source: https://arxiv.org/html/2502.14786v1_
  ⚠️ 단, 백본 교체는 **"거리/각도 교란"을 고치지 못함** — 그건 모델 선택 문제가 아니라 관측 조건 교란.
- **YOLOE (ICCV 2025) — 우리 42 템플릿을 이미지 프롬프트로 ✅ 최저 마찰 대안.** text + **visual(SAVPE, 참조 이미지+마스크 one-shot)** + prompt-free를 한 모델에서. **우리가 이미 쓰는 `ultralytics` 패키지**. 닫힌집합 YOLO 대비 **추가 FLOPs/지연 없음**. **P1/P2/P3를 동시에 건드릴 잠재력** — 텍스트 프롬프트 취약성 자체를 우회. **[MED — 우리 데이터 실측 필요]**
  _Source: https://arxiv.org/html/2503.07465v1 , https://docs.ultralytics.com/models/yoloe_
- **OWLv2** — 이미지 조건 one-shot 검출 지원, HF `transformers`로 즉시. 단 **high recall / low precision 영역**(P2엔 유리, P1엔 불리 — 우리 하류 게이트가 강하므로 감당 가능할 수도).
- **❌ T-Rex2 / DINO-X / T-Rex-Omni** — **네거티브 비주얼 프롬프트**라는 우리 문제에 가장 정확한 기법(LVIS rare +7.1 APr)이지만 **클라우드 API 전용, 공개 체크포인트 미확인** → 오프라인 배치엔 **사실상 사용 불가**. _Source: https://arxiv.org/html/2511.08997_

### P2 (recall 붕괴): 재학습 없는 문서화된 레버

| 기법 | 문서화된 효과 | 즉시 가능 | 비용 |
|---|---|---|---|
| **YOLO-World 자체 conf 임계 하향(객체별)** | 공식 가이드상 **0.1~5% conf에서도 유효 검출**. "different classes have different ideal thresholds" | **YES — 재튜닝뿐** | **0** |
| **CLAHE / 감마 보정** | 과/저노출 미검출 감소에 직접적. 단 CLAHE는 어두운 영역 과증폭으로 FP 소폭 증가 가능 | YES (순수 OpenCV) | ≈0 |
| **SAHI 타일링 추론** | recall 31.8%→86.4% 보고, 소형객체 +5~7% AP. **모델 불문, 파인튜닝 불요** | YES (`sahi` 패키지) | 타일 수 배 연산 |
| **프롬프트 앙상블** | template +1.3pt, 앙상블 +3.5pt, 합 최대 +5%. Detic이 7개 프롬프트 평균 사용 | YES-with-small-work | 오프라인 1회 |
| TTA (multi-scale+flip) | flip 2x / 3-scale 5.25x / 합 ≈10.5x 연산. **WBF 필요**(plain NMS 부적합) | YES | **가파름 — 최악 객체에만 선별 적용** |
| ❌ imgsz 960→1280 | **mAP@.5:.95 +1.2pt뿐, mAP@.5는 오히려 하락, 지연 4배** | 기술적 YES | **비권장** |

**🔑 P2 최우선**: Mugcup **9FP/81FN**, Sauce **1FP/75FN** 패턴은 **하류 게이트가 아니라 YOLO-World 자체 임계값이 proposal을 굶기고 있다**는 신호. **비용 0의 최고 레버.**

**과노출은 우리 탓이 아님 — 문서화된 아키텍처 속성**: Grounding DINO는 **최경미 밝기 손상에서도** AP 48.4 → 26.1(**-46% 상대**), YOLO-World는 39.3 → 33.5 → (severe) 23.4. **[HIGH]** _Source: https://arxiv.org/html/2405.14874v3 (Chhipa et al., ECCV 2024)_

### 시간 전파: 우리 SLAM + ObjectMemory 재활용

- **랜드마크 재투영(detection-by-projection) ✅ 우리 최강 레버.** 카메라 pose + 3D 랜드마크로 검출 실패 프레임에 **투영**해 ROI를 만들고 **값싼 검증**을 수행. 단일 정식 명칭 없음("map projection" / "active search" / "temporal priors as region proposals").
  - **선례**: **SEO-SLAM**이 정확히 이것을 구현 — 랜드마크를 현재 프레임에 투영, 객체 치수+카메라 pose로 bbox 생성, MLLM으로 재검증, 사라진 랜드마크용 **"Empty" 상태** 보유. **정량 성과: 의미 정확도 0.79 / FP 3건 (피드백 없는 기준선 0.52 / FP 10건)** — **[HIGH, 단 단일 데이터셋]** _Source: https://arxiv.org/html/2411.06752v1_
  - **"객체가 치워졌으면?" 안전장치 = 우리가 이미 가짐.** **Persistence Filter (Rosen/Mason/Leonard, ICRA 2016)**가 우리 Bernoulli `r`의 학술적 직계 — 생존분석/hazard 함수 기반 재귀 베이즈 추정, **준정적 환경에서 맵 특징이 사라지는 상황을 위해 설계**. **[HIGH]** _Source: https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/44821.pdf_ · Perpetua(IROS 2025)가 persistence+emergence로 확장 _https://arxiv.org/abs/2507.18808_
  - → **우리 ObjectMemory의 `r` + FOV-aware P_D는 인용된 시스템 대부분의 안전장치보다 이미 정교합니다.**
- **map→detection 루프를 닫는 시스템은 드묾.** SLAM++/Fusion++/QuadricSLAM/CubeSLAM/MaskFusion/MID-Fusion/Voxblox++/Kimera/DSP-SLAM/vMAP/ConceptGraphs 조사 결과 **대부분 단방향**. 진짜 예외 2건:
  - **Volumetric Semantically Consistent 3D Panoptic Mapping** — 누적 3D panoptic 일관성으로 프레임별 instance/semantic 라벨을 교정, **Voxblox++ 대비 +16.1 mAP**. **[HIGH — 본 조사 최대 정량 성과]** _Source: https://arxiv.org/abs/2309.14737_
  - Pillai & Leonard (RSS 2015) — 3D seed를 전 프레임에 투영해 특징 pooling. _https://arxiv.org/abs/1506.01732_
- **🚫 결정적 부정 결과 — 다중뷰 일관성은 P1을 못 고치고, 오히려 강화합니다.**
  택배박스는 **진짜로 존재하는 정적 물체**입니다. "maroon box"가 여러 시점에서 안정적으로 발화하면 **깨끗하게 삼각측량되고, 저분산 3D 위치를 얻고, 우리 것과 똑같은 Bernoulli `r` 체계에서 높은 존재확률로 수렴**합니다 — **정탐과 구별 불가.**
  - SEO-SLAM 설계가 이를 간접 확증: "유사 객체가 하나의 랜드마크로 융합"되는 문제의 해법으로 **의미 재검증(MLLM)**을 쓰지 기하 융합을 더 하지 않음.
  - 2026 distractor 벤치마크(SADL): 모델 오류가 **기하/시각적 일치가 가장 강한 후보에 집중** — 기하 일관성과 의미 정확성이 hard distractor에서 **역상관**. **[MED]** _Source: https://arxiv.org/pdf/2606.30393_
  - → **`r`은 이 실패 모드를 판별하지 못합니다.** 다중뷰는 *일시적 노이즈*용 도구이지 *안정적 오분류*용이 아님.
- **SAM2 / Det-SAM2 / DEVA — 8~19프레임 공백 메우기.** "간헐 검출 → 연속 전파 → 다음 검출로 재시드" 패턴의 검증된 청사진. **[HIGH — 우리 구조와 거의 turnkey 일치]** _Source: https://arxiv.org/abs/2309.03903 , https://arxiv.org/html/2411.18977v2_ · SAM2.1-B+ 5~37 FPS → 2,596프레임은 분 단위. ⚠️ SAM2 단독은 **의미 재검증이 없어** 공백 중 distractor가 지나가면 트랙을 **조용히 탈취**당함.
- **FoundationPose** — 우리 CAD(mm, 정점색)에 직접 적합, 트래킹 모드 ~32Hz, 추적 실패 시 자동 재추정 폴백. 마지막 유효 PEM pose로 시드 가능. **[HIGH 구조 적합도]** _Source: https://arxiv.org/html/2312.08344v2_ · BundleSDF는 CAD 불요/무거움 → **우리에겐 불필요**(CAD 있음).

### 정직한 상한 (P2)

**회복 가능**: 인접 프레임에서 검출된 객체의 8~19프레임 공백 — ByteTrack 기본 트랙 소실 버퍼가 **30프레임**이므로 우리 공백은 **교과서적 정상 범위**. _Source: https://trackers.roboflow.com/latest/trackers/bytetrack/_

**어떤 시간 기법으로도 회복 불가 — 명시할 것**:
- **(a) 실제 제거/이동** — 회복하면 그게 환각. 우리 `r` 감쇠가 **올바르게 실패하게** 하는 장치.
- **(b) 긴 가림** — SAM2는 가림/재등장에서 잘못된 마스크를 붙잡고 **그대로 전파**하는 사례 문서화. _https://arxiv.org/html/2410.16268v3_
- **(c) 심한 모션블러** — 원신호 손상. 결정층 문제 아님.
- **🔴 (d) Cold-start — 하드 리밋.** 트래커는 **기존 트랙을 전파할 뿐**. 새 트랙은 원 검출기 출력에서 시드됨. → **최초 검출 시점은 프레임별 원 검출기 recall에 엄격히 제한됨.** **[HIGH — tracking-by-detection 일반 원리]** 즉 **어떤 객체의 *첫* 목격도 도울 수 없고, 2번째~N번째만 도움.**
- **(e) 급격한 조명 변화** — "진짜 부재" vs "존재하나 외형 변화" 판별은 미해결 문제. _https://arxiv.org/html/2403.19242v1_

**우리 숫자에 대한 함의**: FN ~1,392 중 **인접 프레임에서 검출된 구간에 낀 것들이 회복 가능한 다수**. 2,596프레임 전체에서 *최초 등장*이거나 실제 가림과 겹친 FN은 **본 보고서의 어떤 방법으로도 안 고쳐짐** — 그건 원 검출기 recall 문제.

---

## 소스 정합성 / 이견

- 한 검색 스니펫이 *"DINOv2 does not lead to a high shape-bias"*라 주장했으나, 2025 NeurIPS 논문과 arXiv 2503.12453 **양쪽 피어리뷰 출처가 반대**를 말함. 독립 피어리뷰 2건을 채택하고 해당 스니펫은 구/타 벤치마크 오독으로 판단 → **"DINOv2는 pooled/global 수준에서 shape-biased" = HIGH**.
- MegaPose의 hypothesis당 렌더 비용(2.5s), PyTorch3D 사용 여부는 **검색엔진 요약 경유**(PDF 직접 fetch 실패) → **MED, 용량 산정에 쓰기 전 1차 출처 확인 필요**.
- nvdiffrast의 589k verts 메시 프레임당 비용은 **인용 가능한 벤치마크 없음** → **LOW-MED, 실측 필요**.

---

## ⚠️ 정정 (step-03 코드 실측) — 위 §Technology Stack Analysis의 2개 주장이 반증됨

step-02는 웹 문헌만으로 작성됐습니다. 코드베이스를 실측하니 **2건이 우리 코드에 대해 틀렸습니다.**

### 정정 ①: DINOv2는 ViT-L/23이 아니라 **ViT-S/14, depth=12**

`yolo_ism.py:49-51,62-73` — `vit_small(patch_size=14, img_size=518, block_chunks=0)`, `dinov2_vits14_pretrain.pth`, embed_dim **384**, **블록 12개**(`vision_transformer.py:336-347`).

⇒ **FoundPose의 "23층 중 18층"은 우리 모델에 존재하지 않습니다.** 유효 범위는 블록 **0~11**. 비례 환산하면 ~9~10층이지만, **그 비례가 성립한다는 근거는 없습니다 [LOW]**. FoundPose는 ViT-L에서 측정했습니다. → **층 스윕(0~11)을 직접 실측해야 함.**

### 정정 ②: `masked_appe_score`는 **이미 patch token 기반**입니다 (pooled/CLS 아님)

`yolo_ism.py:324-333`:
```python
sim = query_fg @ template_patches.T
return float(sim.max(dim=1).values.mean().clamp(0, 1))
```
query 전경 패치마다 최적 템플릿 패치를 max로 취하고 평균 — SAM-6D `loss.py compute_straight`와 동일. **pooled 벡터도, patch 평균도 아닙니다.**

⇒ step-02의 *"masked-appe 게이트는 pooled/CLS 임베딩을 쓴다"*는 **우리 코드에 대해 거짓**입니다. 실제 구조:

| 게이트 | 실제 표현 | shape 편향 해당? |
|---|---|---|
| `semantic_score` (`:238-242`) | **CLS 토큰** top-k 평균 | ✅ **해당** — 여기가 shape-biased |
| `masked_appe_score` (`:324-333`) | **최종층 patch token**, query-anchored max | ⚠️ 부분 — patch는 맞지만 **최종층** |

⇒ **수정된 주장**: "CLS→patch로 바꾼다"가 아니라 **"최종층 patch → 중간층 patch"**. **효과는 step-02가 예상한 것보다 작을 수 있습니다 [MED→LOW로 하향].** 다만 FoundPose의 ablation은 여전히 유효한 가설이고 비용이 0이므로 **실측 1순위**는 유지.

### 정정 ③ (호재): 템플릿 회전은 **이미 해석적으로 보유**, xyz 피팅 불필요

- 템플릿은 BlenderProc `Render/render_custom_templates.py:27,89`가 **`cam_poses_level0.npy`(= icosphere level0, 정확히 42뷰)** 순서대로 렌더 → **`rgb_N.png`의 N = `cam_poses_level0.npy`의 N행**.
- **`yolo_ism.py:639`가 이미 이걸 합니다**: `R = geo["poses"][best_t][:3, :3]` (`poses = np.load(cam_poses_level0.npy)  # [42,4,4]`, `:491,551`).
- ⇒ step-02의 "xyz_N.npy로 시점 복원"은 **불필요한 우회**였습니다. xyz는 NOCS(정규화 객체 좌표)이고 PEM이 점군으로 소비할 뿐(`render_custom_templates.py:135`), 어디서도 pose 복원에 안 씁니다.
- ⚠️ **불일치 발견**: 렌더는 level0(42뷰)인데 `run_pipeline.py:186`, `detector.py:158` 등은 `level=2`(642뷰)를 로드합니다. 우리 실행경로(`yolo_ism.py`)는 level0으로 정상.

---

## Integration Patterns Analysis

> **도메인 매핑**: 표준 하위 절(REST/GraphQL/gRPC, 메시지 큐, OAuth, 마이크로서비스)은 본 파이프라인에 해당 없음. **실제 통합 경계 = conda 환경 분리 + 디스크상 JSON 계약 + 프로세스 경계**로 매핑.

### 프로세스·환경 경계 (실측)

| 환경 | 담당 | 근거 |
|---|---|---|
| **`sam_yolo`** | **ISM**: YOLO-World + DINOv2 + MobileSAM | `build_ism_inputs_imu.py:19`, `run_e2e.sh:4-5,16,21`. ultralytics **8.4** |
| **`sam6d_ros_humble`** | **PEM** (gorilla + pointnet2 빌드됨), 템플릿 렌더 | `run_pem.sh:5,8`, `render_all_templates.sh:6`. ultralytics 8.0 — **YOLOWorld 없음** → `yoloworld_sidecar.py`가 존재하는 이유 |

**두 스테이지는 오직 디스크상 JSON으로만 통신합니다** (`run_pem.sh:22-27`).

⇒ **함의: 텍스처 검증기를 ISM 프로세스(`sam_yolo`) 안에 두면 이미 로드된 DINOv2 모델 객체를 그대로 재사용할 수 있습니다.** 신규 프로세스·IPC·모델 로드 0.

### 데이터 계약 (실측)

**ISM → PEM** (`run_inference_custom.py:169-177`): `detection_<obj>.json` = BOP 리스트, 키 `scene_id/image_id/category_id/bbox/score/segmentation`(COCO RLE). `det['score'] > det_score_thresh`(기본 0.2, `:58`)로 필터. **추가 키는 그대로 통과** (`:172`).

**PEM → 하류** (`run_inference_custom.py:296-311`) — **입력 dict를 제자리 변형해 재덤프**:
```python
pose_scores = out['pred_pose_score'] * out['score']      # :296
detections[idx]['score'] = float(pose_scores[idx])       # :306  ← ISM score를 덮어씀!
detections[idx]['R'] = list(pred_rot[idx].tolist())      # :307  3x3
detections[idx]['t'] = list(pred_trans[idx].tolist())    # :308  mm
```
🔴 **`:306`이 ISM score를 덮어씁니다.** ⇒ **ISM측 텍스처 점수는 반드시 다른 키로 태워야 PEM을 생존합니다.**

**PEM → ObjectMemory** (`adapters/sam6d_pem.py:56-60`): `R`/`t`만 요구, **나머지 JSON 키는 현재 버려짐** → 어댑터 확장 필요.

### 확장 지점 (파일:행 확정)

| # | 지점 | 위치 |
|---|---|---|
| 1 | **DINOv2 중간층 접근** | `model.get_intermediate_layers(x, n=[i], return_class_token=True)` — `vision_transformer.py:294-318`. `block_chunks=0`(`yolo_ism.py:68`)이라 not-chunked 경로 동작. **모델 수정 0**. 단 **별도 forward 1회** 필요(기존 `model(x, is_training=True)`는 최종층만 반환) |
| 2 | 배치 patch 토큰 (on-device) | `dinov2_forward_batch` `yolo_ism.py:126-139` |
| 3 | query 전경 패치 | `masked_query_patches` `yolo_ism.py:313-321` |
| 4 | **게이트 삽입 슬롯** | `masked_appe_score` 호출 `:629` ~ 게이트 판정 `:648` **사이** |
| 5 | **승리 템플릿 인덱스** | `best_t = argmax(tcls @ cls)` `:628` — `tappe[best_t]`(`:629`)와 `poses[best_t]`(`:639`)가 **동일 인덱스 공간** |
| 6 | **템플릿 회전(해석적)** | `poses = np.load(cam_poses_level0.npy)  # [42,4,4]` `:491,551` |
| 7 | 템플릿 특징 캐시 패턴 | `build_template_appe` blob `:227-232` |
| 8 | ISM 출력 컬럼 | CSV `fields` `:563-565` |
| 9 | **ObjectMemory 품질 가중** | **`features.py:98-103`** `detection_log_lr` 가산항 + `QualityWeights`(`:55-63`, frozen) |
| 10 | ObjectMemory 검출 필드 | `Sam6DDetection` `models.py:46-58` — **`depth_quality` 슬롯이 미사용 상태로 이미 존재** |

**ObjectMemory 확장이 특히 깔끔합니다** — `features.py:98-103`이 가산 구조:
```python
log_lr = (base + w_score*phi_score(score) + w_depth*phi_depth(...) + w_resid*phi_resid(...))
```
→ `+ w_texture * phi_texture(tex)` 한 항 추가. 가중치 0이면 기존과 동일(회귀 방지 계약이 `test_confidence_likelihood.py:32-33`에 이미 assert돼 있음).

---

## Architectural Patterns Analysis

> **도메인 매핑**: 표준 하위 절(마이크로서비스/모놀리스/서버리스, SOLID, 클린 아키텍처)은 해당 없음. **검증기 배치 아키텍처 + 무라벨 결정 구조**로 매핑.

### 🔴 결정: 검증은 **PEM 앞**에 둔다 (문헌 합의)

Viola-Jones → Cascade R-CNN의 직계: *"a lightweight detector first rejects the majority of easy negatives and feeds hard proposals to the next stage"* **[HIGH — 아키텍처 합의]**
_Source: https://arxiv.org/abs/1712.00726_

6D pose 계열 전부 동일: **SAM-6D ISM은 PEM 앞에서 전량 필터**(*"only proposals exceeding the matching threshold proceed to PEM"*), GigaPose는 coarse 검색→fine, MegaPose는 학습된 scoring network가 비싼 refiner 진입을 게이팅, FoundPose는 DINOv2 patch BoW 검색 후 top-K만 정제, ZePHyR는 아예 독립 hypothesis rater.
_Source: https://arxiv.org/abs/2311.15707 , https://arxiv.org/abs/2311.14155 , https://arxiv.org/abs/2212.06870 , https://arxiv.org/abs/2311.18809_

**⇒ PEM 뒤 검증(옵션 a)은 규범이 아니고, 함정이 있습니다**: PEM이 choco CAD를 택배박스에 피팅하면 **실루엣은 그럴듯하고 텍스처만 틀린** 렌더가 나옵니다. 즉 기하만 보는 사후 검증기는 **pose_score와 똑같이 실패**합니다(우리가 이미 겪은 그 실패).

**🔑 그리고 우리는 이미 그 슬롯을 갖고 있습니다.** SAM-6D ISM은 *"매칭된 템플릿의 알려진 회전 + proposal 점군 중심"*으로 **coarse pose를 PEM 호출 없이** 계산합니다 — 그리고 **`yolo_ism.py:639`가 정확히 이걸 하고 있습니다**(`R = geo["poses"][best_t][:3,:3]`). ⇒ **PEM 비용 없이 pose 정렬된 텍스처 비교가 ISM 비용으로 가능합니다.** 새 아키텍처 리스크가 아니라 **이미 있는 스테이지의 확장**(shape-IoU → 텍스처).

### 🔴 결정: 절대 임계값이 아니라 **상대 비교(cohort/UBM)** 로 간다

**우리는 이미 이걸 한 번 증명했습니다** — NMS 순위를 YOLO conf→appe로 바꿔 14/22→22/22, **임계값 튜닝 0**. 이건 우연이 아니라 **형식적으로 인정된 원리**입니다:

- **생체인식의 verification(1:1) vs identification(1:N)** 구분이 정확히 이것: *"Verification (1:1)... is a threshold test"* (FAR/FRR/EER 보정 필요) vs *"identification (1:N)... uses Rank-1 IR... inherently uses the argmax without requiring a similarity threshold."* **[HIGH]**
  ⇒ **상대/argmax 결정은 보정이 필요 없고, 절대/임계 결정은 필요합니다.** 우리가 게이트 3개를 세웠다 철회한 이유가 이걸로 설명됩니다.
- **화자인식의 cohort/UBM 정규화** = 우리가 찾던 "배경 프로토타입과 경쟁" 기법의 **20년 선례**: *"Score normalization models the alternative hypothesis using either the cohort or the universal background model... the client score is divided by the average cohort score."* **[HIGH]**
  _Source: https://arxiv.org/pdf/1609.08419_

**⇒ P1 주력 아키텍처**: `score = sim(crop, choco_템플릿) − sim(crop, 택배박스_프로토타입)` 또는 우도비. **절대 임계값 소멸.** 필요한 건 두 프로토타입이 존재하는 것뿐이고, **둘 다 있습니다**(진짜 42템플릿 + FP 코퍼스에서 뽑은 택배박스 crop들 — **라벨 불요**, "ISM이 이걸 choco라 했다"는 사실만 쓰므로).

**🔑 그리고 이건 우리 config 구조와 정확히 맞습니다.** `yolo_ism_objects.yaml`은 이미 N객체를 지원하고, cross-object NMS는 **이미 appe로 순위를 매깁니다**. **택배박스를 하나의 객체로 등록**하면 choco vs 택배박스가 자동으로 경쟁합니다 — 이게 §Open-Vocab의 "네거티브 클래스"와 §cohort 정규화가 **같은 지점에서 만나는** 곳입니다.

### 무라벨 신호 융합

| 방법 | **라벨 없이 가능?** | 비고 |
|---|---|---|
| **Reciprocal Rank Fusion / Borda** | **YES** | 원점수가 아니라 **후보 집합 내 순위**로 동작 → 서로 다른 스케일(히스토그램 거리/엣지 밀도/patch 유사도/NCC) 문제가 **소멸**. *"doesn't need vast amounts of labelled training data... especially in zero-shot settings"* |
| z-score / rank 정규화 후 결합 | **YES** | 표준. 상대 순서만 필요 |
| **kNN / Mahalanobis distance to 42 템플릿** | **YES** | 1-class 문제. ⚠️ Mahalanobis는 42점으로 고차원 공분산 추정 → **수치 취약**, PCA/shrinkage 필요 |
| Noisy-OR / Product-of-Experts | **형식은 YES, 입력이 NO** | 보정된 확률을 전제 → 보정이 곧 우리가 못하는 것. rank 정규화하면 결국 rank 기반으로 회귀 |
| Otsu / 2-GMM 골짜기 / KDE | **계산은 YES, 신뢰는 NO** | **GMM은 요청하면 항상 2개를 찾습니다** — 단봉 데이터에서도. **우리가 당한 그 실패** |
| **Hartigan's Dip Test** | **YES** ← 우리가 빠뜨린 조각 | 골짜기 유무가 아니라 **단봉성 귀무가설의 p-value**. 라벨 불요(분포 *모양* 검정) |
| Conformal prediction | **❌ NO — 사도(dead end)** | 라벨된 calibration set 필수. **우리 추측이 맞았음** |
| 합의 기반 pseudo-label | YES, **위험** | *"feedback loop where the model becomes increasingly confident in its wrong predictions"*. 신호들이 **상관 실패**하면 무력 |

**🔴 우리가 당한 임계값 실패의 구체적 처방** (전부 라벨 불요):
1. **Dip test** — 골짜기를 믿기 전 단봉성 기각 여부 확인
2. **Bootstrap 재표집** — 임계값 분산이 크면 불안정
3. **소스별 부분집합 일관성** — **bag별/조명별로 재계산해 골짜기 위치가 안정한지.** ← *이게 바로 "표본 90%가 sam_105314"를 사전에 잡아냈을 검사*

---

## Implementation Approaches and Technology Adoption

### 구현 로드맵 (비용 오름차순, 각 단계가 독립 검증 가능)

| # | 작업 | 대상 | 비용 | 리스크 |
|---|---|---|---|---|
| **1** | **DINOv2 층 스윕 0~11** — `get_intermediate_layers(x, n=[i])`로 masked-appe 재계산, 진짜 choco vs 택배박스 **분리 마진**을 층별 측정 | P1·P3 | **반나절, 의존성 0** | 낮음. **ViT-S/12라 FoundPose(ViT-L) 결과가 전이 안 될 수 있음 [LOW]** |
| **2** | **색 히스토그램 + 엣지 밀도** — 마스크 내 HSV 이봉성 / Sobel 밀도 | P1 | **반나절, `cv2`만** | 낮음. 노출 정규화 필요 |
| ~~3~~ | ~~cohort 상대 점수 — 택배박스 프로토타입을 객체로 등록~~ | ~~P1 주력~~ | — | **⛔ 사용자 기각(장면별 두더지잡기). 아래 §사용자 기각 참조** |
| **4** | **YOLO-World 자체 conf 임계 객체별 하향** | **P2 주력** | **0 — 재튜닝뿐** | FP 증가 → 하류 게이트가 흡수해야 |
| **5** | CLAHE/감마 전처리 | P2(토끼) | ≈0 | CLAHE는 어두운 영역 FP 소폭↑ |
| **6** | 템플릿 워핑 render-and-compare (`poses[best_t]` + PEM R,t로 in-plane 보정) | P1 | 1일, 의존성 0 | 42뷰 양자화(≈15~20°) |
| **7** | RRF로 1·2·6 융합 | P1 | 0.5일 | 신호 상관 실패 |
| **8** | ObjectMemory `w_texture` 항 추가 | P1·P2 | 0.5일 | 가중치 0 기본 → 회귀 없음 |
| **9** | 랜드마크 재투영 (`r` + FOV P_D 재사용) | P2 | 중 | **cold-start 못 고침** |

### 테스팅·QA — GT 없이 어떻게 검증하나

**우리가 이미 가진 검증 자산**: 2,596프레임 전수 감사 결과(객체별 FP/FN + 구체적 프레임 번호), 육안 확증 대조군 22장, `--nms-rank yolo`로 baseline 재현 가능.

- **회귀 방지 계약**: ObjectMemory는 `test_confidence_likelihood.py:32-33`이 "가중치 0 → 기존과 동일"을 이미 assert. 신규 항도 같은 패턴.
- **A/B 프로토콜(이번에 확립됨)**: 동일 프레임에 두 설정 실행 → **검출 수 불변 확인**(FP 인플레 원리적 배제) → **육안 확증 대조군으로 채점** → **뒤집힘 전수 육안 확인**. 75프레임으로 14/22→22/22를 검증한 그 방법.
- **⚠️ 이번에 배운 교훈**: 대조군 GT를 크롭 1개만 보고 만들면 틀립니다(먼 테이블의 진짜 곰을 놓쳐 3건 오채점). **전체 프레임을 봐야 함.**

### 위험 평가

| 위험 | 심각도 | 완화 |
|---|---|---|
| **ViT-S/12에서 층 스윕이 무효** | 중 | 비용이 반나절이라 **실패해도 저렴**. 실패 시 2·3번으로 이동 |
| **cohort 네거티브가 정탐 억제** | **높음** | 문헌이 명시한 실패모드. 타깃과 텍스트/시각적으로 충분히 분리. **검출 수 불변 A/B로 조기 검출** |
| 신호 상관 실패(융합 무력화) | 중 | 서로 다른 축 선택(색≠엣지≠patch), dip test + 소스별 일관성 |
| P2 임계 하향이 FP 폭증 | 중 | 하류 게이트가 흡수하는지 A/B. **P1 수정을 먼저** 넣어 방어선 확보 후 진행 |
| **cold-start FN은 원리적 불가** | — | **완화 불가.** 인정하고 예산을 다른 데 쓸 것 |

### 성공 지표

- **P1**: choco FP 377 → 목표 감소율. **검출 수 불변**이 부작용 없음의 증거(§15-3에서 확립한 지표)
- **P2**: Sauce FN 75 / Mugcup FN 81의 감소. **동시에 FP가 늘지 않을 것**
- **P3**: 육안 확증 대조군 22/22 유지 + 회귀 1건(105652 f288) 해소
- **공통**: 어떤 변경도 `--nms-rank yolo` baseline 재현성을 깨지 않을 것

---

## Strategic Technical Recommendations

### 세 문제의 성격이 서로 다름 — 같은 약을 쓰면 안 됨

| | P1 택배박스 FP 377 | P2 recall 붕괴 | P3 인형 잔여 |
|---|---|---|---|
| **성격** | **기하적으로 진짜, 의미적으로 틀림** | 검출기가 제안 자체를 안 함 | 물리적 유사도가 높음 |
| **효과 있는 것** | 텍스처/외형 재검증, cohort 상대 비교 | 검출기 임계 재튜닝, 노출 정규화, 시간 전파 | DINOv2 순위(이미 적용), 층 스윕 |
| **❌ 효과 없는 것** | **다중뷰 일관성**(오히려 강화), 기하 hypothesis verification, PEM pose_score, appe 절대 임계 | 시간 전파의 **cold-start**, imgsz 상향 | appe **절대 임계**(대역 완전 중첩) |

**핵심 통찰: P1에 대한 "더 많은 3D/기하"는 전부 오답입니다.** 택배박스는 진짜 물체라 기하가 더 확신할수록 틀립니다. SADL 벤치마크(2026)가 이를 일반화 — hard distractor에서 **기하 일관성과 의미 정확성은 역상관**.

### 실행 순서 (각 단계가 독립적으로 검증·롤백 가능)

**Phase 1 — 비용 0~반나절, 신규 의존성 0**
1. DINOv2 층 스윕 0~11 → 진짜 choco vs 택배박스 분리 마진 측정
2. 마스크 색 히스토그램 이봉성 + Sobel 엣지 밀도 측정
3. YOLO-World 자체 conf 임계를 객체별로 하향 (P2)

→ **Phase 1은 전부 "측정"이 아니라 "판별력 확인"입니다.** 셋 중 하나라도 진짜/가짜를 가르면 즉시 게이트화. 전부 실패하면 Phase 2로.

**Phase 2 — 1~2일, 신규 의존성 0**
4. 택배박스를 config에 객체로 등록(cohort) → 기존 appe-rank NMS가 자동 해소
5. 템플릿 워핑 render-and-compare (`poses[best_t]` + in-plane 보정)
6. RRF로 신호 융합

**Phase 3 — 신규 의존성 있음**
7. OCR/scene-text 존재 신호 (PaddleOCR det head ~5MB) — **범주형이라 "임계값 불가" 결론을 우회**
8. ObjectMemory `w_texture` 항
9. 랜드마크 재투영 (P2)

**Phase 4 — 큰 베팅 (현 시점 비권장)**
- YOLOE 비주얼 프롬프트(42템플릿을 이미지 exemplar로) — 같은 `ultralytics`라 마찰은 낮으나 파이프라인 전면 교체
- nvdiffrast 실렌더, ZePHyR

### 모든 임계값에 의무화할 검사 (재발 방지)

오늘 게이트 3개를 세웠다 전부 철회한 실패의 직접 처방:
1. **Hartigan's dip test** — 골짜기가 있는지가 아니라 **단봉성이 기각되는지**
2. **Bootstrap** — 임계값이 재표집에 안정한지
3. **🔴 소스별 부분집합 일관성** — bag별/조명별로 재계산해 골짜기 위치가 안정한지. **이것이 "표본 275건 중 90%가 sam_105314"를 사전에 잡아냈을 검사**

---

## Future Technical Outlook

- **BOP Challenge 2024 기준선 이동**: FreeZeV2.1이 2023 최고 대비 **+22%**, unseen 2D 검출은 MUSE가 CNOS 대비 **+21~29%**. 우리 SAM-6D ISM은 CNOS 계열이므로 **검출 단계 자체가 2세대 뒤처져 있습니다.** _https://arxiv.org/abs/2504.02812_
- **비주얼 프롬프트 검출기의 성숙**: YOLOE(ICCV 2025)가 text/visual/prompt-free를 한 모델에, 지연 페널티 0으로 통합. **우리 42 CAD 렌더가 그대로 이미지 exemplar가 됨** → 텍스트 프롬프트 취약성 자체가 소멸할 수 있음. 중기 최대 기회.
- **네거티브 비주얼 프롬프트**(T-Rex-Omni)가 우리 P1에 가장 정확한 기법이나 **클라우드 API 전용** → 공개 체크포인트가 나오면 재평가.
- **SAM3**(2025-11)이 text-promptable 검출+분할+추적을 통합 — 우리 YOLO-World→DINOv2→MobileSAM 스택 전체를 대체할 수 있으나 **현장 검증 데이터 부족**.
- **장기 관찰**: 우리가 겪은 "pose 추정기에 거부 헤드가 없다"는 구조적 공백은 ZePHyR(2021) 이후로도 주류가 채우지 않았습니다. render-and-compare 검증을 **별도 스테이지로 두는 설계**는 당분간 각자 만들어야 합니다.

---

## Research Methodology and Source Verification

**방법**: 병렬 리서치 에이전트 4종(render-and-compare/pose 검증 · open-vocab 판별축 · 시간전파 · 검증기 배치와 무라벨 융합) + 코드베이스 실측 에이전트 1종. 총 웹 도구 호출 91회.

**검증 원칙과 그 성과**:
- **코드 실측이 웹 리서치를 반증했습니다.** "masked-appe는 pooled 임베딩을 쓴다"(웹 추정) → **거짓**(이미 patch token). "FoundPose의 18층을 쓰라" → **우리 모델엔 존재하지 않음**(ViT-S/12). **문헌을 우리 코드에 확인 없이 적용했으면 잘못 구현했을 것입니다.**
- **출처 충돌 처리**: 한 스니펫이 "DINOv2는 shape-bias가 높지 않다"고 주장 → 2025년 피어리뷰 **2건이 반대** → 피어리뷰 채택, 스니펫 기각.

**신뢰도 미달 항목 (사용 전 실측 필요)**:
| 항목 | 등급 | 이유 |
|---|---|---|
| ViT-S/12에서 층 스윕이 유효한가 | **LOW** | FoundPose는 ViT-L/23에서 측정. 비례 환산 근거 없음 |
| MegaPose hypothesis당 렌더 비용 2.5s | MED | 검색엔진 요약 경유, PDF 직접 fetch 실패 |
| nvdiffrast 589k verts 프레임당 비용 | LOW-MED | 인용 가능한 벤치마크 없음 |
| OCR 게이트의 우리 케이스 효과 | MED-HIGH | 기전은 강하나 이 시나리오 벤치마크 없음 |
| YOLOE의 우리 데이터 성능 | MED | 실측 필요 |

**주요 출처**: SAM-6D `arxiv.org/abs/2311.15707` · FoundPose `2311.18809` · GigaPose `2311.14155` · MegaPose `2212.06870` · FreeZeV2 `2506.09784` · ZePHyR `2104.13526` · BOP 2024 `2504.02812` · Cascade R-CNN `1712.00726` · Configural Shape (NeurIPS 2025) `2507.00493v1` · CLIP vs DINO `2510.11835v1` · SigLIP2 `2502.14786v1` · YOLOE `2503.07465v1` · SAHI `2202.06934` · 밝기 손상 벤치마크(ECCV 2024) `2405.14874v3` · SEO-SLAM `2411.06752v1` · Persistence Filter (ICRA 2016) · Panoptic Mapping `2309.14737` · SADL `2606.30393` · cohort/UBM `1609.08419` · Roboflow YOLO-World 프롬프트 가이드

---

## 다음 액션 제안

본 리서치는 **Phase 1(반나절, 신규 의존성 0)이 전부 판별력 확인 실험**이 되도록 설계됐습니다. 셋 중 하나라도 진짜/가짜를 가르면 즉시 게이트화하고, 전부 실패하면 그 자체가 Phase 2로 가라는 신호입니다.

**선택지:**

1. **Phase 1을 지금 실행 (권장)** — DINOv2 층 스윕 + 색 히스토그램/엣지 밀도. **반나절, 신규 의존성 0.** 이미 감사로 확정된 진짜 choco 프레임과 택배박스 FP 프레임이 있으므로 분리 마진을 즉시 측정 가능. 실패해도 손실이 반나절.
2. **cohort(택배박스 객체 등록)부터** — P1 주력이고 우리가 이미 증명한 appe-rank NMS를 그대로 재사용. 다만 택배박스 프로토타입 템플릿을 만들어야 해서 1일.
3. **P2부터 (YOLO-World 임계 하향)** — 비용 0인데 FN 1,392의 상당수를 건드림. 단 FP가 늘 수 있어 P1 방어선 이후가 안전.
4. **리서치를 더 깊게** — YOLOE 비주얼 프롬프트나 OCR 게이트를 별도 조사.

**저는 1번을 권합니다.** 이유는 비용이 아니라 **의존 구조**입니다 — 층 스윕과 색/엣지 신호의 판별력을 모르면 cohort(2번)에서 *무엇을 기준으로* 경쟁시킬지 정할 수 없고, RRF 융합(Phase 2)에 넣을 신호도 고를 수 없습니다. Phase 1은 나머지 전부의 입력입니다.

**질문**: Phase 1을 지금 시작할까요? 아니면 P2(비용 0)를 먼저 손봐서 FN을 줄이는 쪽이 우선순위입니까?

---

## ⛔ 사용자 기각 (2026-07-15): Phase 2-1 cohort 이름 붙이기 취소

**사용자 지적**: *"새로운 장소에서 촬영할 때마다 잘못 판정되는 것을 전부 이름 훔치지 않도록 이름을 주려는 것인가? 그건 취소해라."*

**타당하며 수용합니다.** 택배박스를 `"cardboard shipping box"` 객체로 등록하는 방식은 **장면별·distractor별 패치**입니다. 새 촬영지마다 새 distractor(공기청정기·프린터·텀블러·소독제 펌프 등 — 감사가 이미 6종을 발견)가 나오고 그때마다 이름을 추가해야 하므로 **두더지잡기이고 일반화되지 않습니다.**

**리서치 오류 인정**: cohort/UBM은 원래 **impostor마다 이름을 붙이는 기법이 아니라 하나의 범용 배경 모델(Universal Background Model)** 을 쓰는 기법입니다. 이를 "장면별 네거티브 클래스"로 옮긴 것은 원 기법의 왜곡이었고, 마침 §Open-Vocab의 네거티브 프롬프트와 결론이 겹쳐 보여서 근거가 이중으로 강해 보이는 착시를 만들었습니다.

### 대체: 장면 독립적인 두 축만 남긴다

핵심 기준 — **우리 객체 자신만 참조하는가?**

| 방법 | 던지는 질문 | 장면 의존 | 채택 |
|---|---|---|---|
| ~~cohort 네거티브 이름~~ | "이게 택배박스인가?" | ❌ distractor마다 등록 필요 | **기각** |
| **템플릿 워핑 대조** (구 2-2) | "이게 **내 물체**가 이 각도에서 보이는 모습인가?" | ✅ 무관 | **주력 승격** |
| **1-class 거리** (42 템플릿까지 kNN/Mahalanobis, leave-one-out 자기거리를 기준분포로) | "이게 **내 물체 분포**에서 나왔나?" | ✅ 무관 | **채택** |

두 방법 모두 distractor의 정체를 **알 필요가 없습니다** — "내 물체가 아니다"만 판정하면 되므로 처음 보는 장소의 처음 보는 물체에도 그대로 동작합니다. 리서치 §Architectural Patterns의 "상대 비교" 원리는 유지되나, 비교 대상이 *명명된 distractor*가 아니라 **자기 템플릿 분포**로 바뀝니다.

**Phase 2 재구성**: 2-1 삭제 · 2-2(템플릿 워핑 대조) 주력 승격 · 1-class 거리 신규 추가 · 2-3(RRF 융합) 유지.

---

# Phase 1 실측 결과 (2026-07-15) — 성공. 단 이유는 예상과 달랐다

**표본**: choco 검출 84건을 **6개 데이터셋에서 각 14건씩 균등** 추출 후 육안 라벨링 → **진짜 36 / 가짜 45 / 애매 3(제외)**. 균등 표본은 의도적 — 2026-07-15 게이트 실패가 *275건 중 90%가 sam_105314*인 표본에서 나왔기 때문.

**라벨 분포가 데이터셋마다 극단적으로 다름** (단일 데이터셋 표본이 왜 위험한지의 실증):

| 데이터셋 | 진짜 | 가짜 |
|---|---:|---:|
| sam_105652 | **12** | 2 |
| sam_105314 | 10 | 4 |
| sam_105018 | 5 | 9 |
| sam_110104 | 5 | 8 |
| sam_110532 | 4 | 8 |
| **sam_110633** | **0** | **14** |

## 1-1. DINOv2 층 스윕 — ✅ 성공 (AUC 0.934 → **0.996**)

`best_t` 선택은 최종층 CLS로 **고정**하고 appe 계산 층만 변수로 격리. ViT-S/14 = **블록 12개**(0~11).

| 블록 | AUC | 진짜 중앙 | 가짜 중앙 |
|---:|---:|---:|---:|
| 0 | 0.294 | 0.693 | 0.721 |
| 3 | 0.764 | 0.667 | 0.609 |
| 5 | 0.851 | 0.626 | 0.553 |
| 7 | 0.890 | 0.578 | 0.518 |
| 8 | 0.917 | 0.628 | 0.559 |
| **9** | **0.996** | **0.637** | **0.561** |
| 10 | 0.972 | 0.593 | 0.537 |
| **11 (현재)** | **0.934** | 0.633 | 0.567 |

**검증 (전부 통과)**:
- **bootstrap 1000회**: blk9 AUC 0.996, **CI [0.985, 1.000]** (blk11은 0.934, CI [0.877, 0.982])
- **소스별 일관성**: blk9 = **1.000 / 1.000 / 1.000 / 1.000 / 0.938** (5개 데이터셋 전부). blk11 = 0.978 / **0.800** / 0.958 / 1.000 / 0.938
- **leave-one-dataset-out**: blk9 0.994~1.000 (blk11 0.922~0.971)

**⚠️ 근거의 지위 변경**: step-02는 이 아이디어를 FoundPose(ViT-L/23의 18층)에서 가져와 **[LOW]**로 낮춰 잡았습니다(우리는 ViT-S/12라 전이 근거 없음). **실측이 이를 지지했고, 심지어 비율도 유사합니다** — FoundPose 18/23 = 78%, 우리 9/12 = 75%. 단 **표본 1건의 일치이므로 인과가 아니라 정황**입니다.

## 1-2. 색/엣지 (신경망 0) — ✅ 성공, 그리고 **유일하게 진짜 이봉분포**

| 특징 | AUC | 진짜 중앙 | 가짜 중앙 |
|---|---:|---:|---:|
| **canny_density** | **0.973** | 0.218 | 0.062 |
| grad_mean | 0.962 | 92.8 | 29.2 |
| grad_norm (노출 정규화) | 0.938 | 1.010 | 0.345 |
| v_iqr_norm | 0.912 | 0.589 | 0.189 |
| v_cv | 0.890 | 0.343 | 0.189 |
| sat_mean | 0.259 | 70.1 | 82.1 (**역방향**) |

**`cv2.Canny` 한 줄이 AUC 0.973**입니다. 인쇄 글자+줄무늬의 엣지 밀도가 민무늬 판지의 **3.5배**.

## 1-3. 🔴 Hartigan's dip test — 우리 원래 실패의 형식적 해부

| 신호 | dip p-value | 판정 |
|---|---:|---|
| blk9 | **0.977** | **단봉 기각 실패** |
| blk11 | **0.909** | **단봉 기각 실패** |
| **canny_density** | **0.0009** | **다봉 확정** |

**blk9/blk11 점수 분포는 통계적으로 단봉입니다.** AUC가 0.996인데도!

⇒ **결정적 함의**: *라벨 없이 히스토그램 골짜기를 찾는 방식은 여기서 원리적으로 실패합니다.* 골짜기가 없으니까요. **제가 275건 히스토그램에서 "본" 0.59 골짜기는 통계적으로 존재하지 않았습니다.** dip test가 그 실패를 사후가 아니라 **사전에** 잡아냈을 것입니다.

⇒ **역으로, canny_density만이 진짜 이봉(p=0.0009)** — 유일하게 무라벨 임계값 탐색이 정당화되는 신호.

⇒ **방법론 교훈**: appe류 신호에 임계값을 세우려면 **라벨이 필요합니다**(우리는 이제 84건 보유). 무라벨 골짜기 탐색은 이 신호에선 금지.

## 1-4. ⚠️ 자기 정정 — choco 0.59 게이트 철회 근거가 **거짓이었음**

`configs/yolo_ism_objects.yaml`에 제가 쓴 철회 사유:
> *"the gate kills 35%-92% of choco depending on the dataset (sam_110633 max appe is 0.594 -> 92% killed, **nearly all of them the REAL box**)"*

**실측 결과 이 주장은 거짓입니다.** sam_110633의 choco 검출은 **총 24건**이고 **appe 상위 14건이 전부 민무늬 택배박스**(`<scratchpad>/top_sam_110633.png`), 균등 표본 14건도 **14/14 전부 가짜**. **진짜 초코하임은 0건.**

⇒ **110633에서 92%가 죽은 것은 FP가 죽은 것 = 정확한 동작이었습니다.** 저는 크롭을 보지 않고 *"많이 죽으니 진짜가 죽는 것"* 이라고 **추론**했고, 그 추론이 틀렸습니다.

**같은 84건 표본에 두 게이트를 적용한 실측 비교**:

| 게이트 | 진짜 유지 | 가짜 제거 |
|---|---:|---:|
| **blk9 @ 0.59** | **35/36 (97%)** | **43/45 (96%)** |
| blk11 @ 0.59 (내가 철회한 그것) | 31/36 (86%) | 38/45 (84%) |

**철회한 게이트조차 86%/84%로 나쁘지 않았습니다.** 제 철회는 과잉 반응이었습니다.

## 1-5. 임계값 안정성 — 정직한 한계

**blk9 데이터셋별 최적 임계**:

| 데이터셋 | 임계 | 진짜 최소 | 가짜 최대 | |
|---|---:|---:|---:|---|
| sam_105314 | 0.593 | 0.593 | 0.563 | **완전 분리** |
| sam_105652 | 0.599 | 0.599 | 0.561 | **완전 분리** |
| sam_110104 | 0.603 | 0.603 | 0.584 | **완전 분리** |
| sam_105018 | 0.605 | 0.605 | 0.574 | **완전 분리** |
| **sam_110532** | **0.647** | 0.589 | 0.602 | ⚠️ **중첩** |

**5개 중 4개가 0.593~0.605 (폭 0.013)** — 매우 안정. **sam_110532만 이탈**하며 유일하게 대역이 중첩합니다.

**고정 임계 0.59 적용 시**: 105018/105314/105652/110104/110633에서 **오류 0건**, sam_110532에서만 FN 1 + FP통과 2.

⇒ **한계**: bootstrap CI는 [0.5887, 0.6033]로 좁으나(sd 0.005), **소스별 폭 0.054는 제 자체 기준(0.049)을 근소하게 초과** → "안정"이라 단언할 수 없습니다. **110532가 무엇이 다른지는 미규명.**

## 1-6. Phase 1 종합

| 실험 | 예상 | 결과 |
|---|---|---|
| 1-1 층 스윕 | **[LOW]** — ViT-S/12라 전이 근거 없음 | ✅ **AUC 0.934→0.996**, 소스별 일관성 통과 |
| 1-2 색/엣지 | [HIGH] | ✅ **canny 0.973**, 유일한 진짜 이봉 |
| 1-3 YOLO 임계 (P2) | — | ⏸ 미실행 |

**"진짜와 가짜를 가르는 숫자가 존재하는가?" → 존재합니다. 두 개나.** 그리고 둘 다 신규 의존성 0입니다.

**남은 정직한 미지수**:
1. **라벨이 제 육안 판단**입니다. 라벨 오류가 있으면 AUC가 부풀려집니다. (크롭이 대체로 명확했으나 — "쵸코하임" 글자 vs 민무늬 판지 — 검증되지 않았습니다.)
2. **84건은 choco만**입니다. 다른 9개 객체에 blk9이 유효한지 미측정.
3. **110532 이탈 원인 미규명.**
4. 전수 적용 시 실효(choco FP 377 중 몇 건 제거)는 미측정.
5. **blk9은 별도 forward가 필요**합니다(기존 `model(x, is_training=True)`는 최종층만 반환) → 프레임당 비용 증가분 미측정.

## 1-7. 다음 액션 제안

1. **blk9을 다른 9개 객체로 확장 검증 (권장)** — choco에서만 되는 우연인지, 전 객체에 되는 성질인지. 객체별 40~60건 라벨링 필요(반나절~1일). **이게 통과해야 프로덕션 반영이 정당합니다.**
2. **바로 프로덕션 반영 후 전수 A/B** — blk9 + 0.59 게이트를 넣고 2,596프레임 재실행. 빠르지만 choco 외 객체에서 회귀 위험.
3. **canny_density를 독립 게이트로 추가** — 유일한 진짜 이봉이라 무라벨로도 정당. blk9과 직교(신경망 vs 픽셀)라 RRF 융합 후보.
4. **110532 이탈 원인 규명** — 조명/거리? 다른 데이터셋과 뭐가 다른지.
5. **P2(YOLO 임계 하향)로 이동** — 비용 0.

**권고: 1번.** 84건은 choco 한 객체이고, 우리는 오늘 이미 *"한 표본에서 좋아 보인 것"*에 두 번 속았습니다(275건 히스토그램 → 골짜기 허상, "92%가 진짜" → 거짓). **같은 실수를 세 번째로 하지 않으려면 확장 검증이 먼저입니다.**

---

# Phase 1b 확장 검증 (2026-07-15) — ❌ blk9 반증, ✅ 대신 기전 규명 + 더 나은 해법

**측정 가능 범위의 정직한 축소**: Sauce(FP 1)·Sikhye(2)·**Rabbit(0)** 은 **가짜가 없어 "분리"를 잴 대상 자체가 없음** → 제외. Mugcup(9) 빈약 → 제외. **측정 대상 = saffron·milk·Dinosaur·Febreze·Bear 5개** (choco 포함 6개).

프로토콜은 choco와 동일: 객체당 84건(6데이터셋 × 14건 균등) 육안 라벨링. **총 420장 추가 라벨링.**

## 1b-1. ❌ blk9은 일반화되지 않는다 — Phase 1 결론 반증

| 객체 | **최적 블록** | 그 AUC | blk9 | blk11(현재) |
|---|---:|---:|---:|---:|
| choco | **9** | 0.996 | 0.996 | 0.934 |
| saffron | **9** | 0.894 | 0.894 | 0.849 |
| milk | **2** | 0.946 | 0.796 | 0.882 |
| Febreze | **2** | 0.971 | 0.940 | 0.907 |
| Bear | **2** | 0.924 | **0.487** | 0.561 |
| Dinosaur | **0** | **1.000** | 0.664 | **0.466** |

**최적 블록이 객체마다 다릅니다.** blk9은 Bear에서 **0.487 — 우연보다 나쁩니다.** Phase 1의 "blk9으로 바꾸자"는 결론은 **choco 전용 우연이었습니다.** 확장 검증이 이를 잡아냈습니다.

## 1b-2. ✅ 기전 규명 — 왜 객체마다 다른가

| 객체 그룹 | b0 | **b2(초기)** | **b9(후기)** | b11 |
|---|---:|---:|---:|---:|
| **FP가 색이 다름** (milk/Febreze/Bear/Dinosaur) | 0.88 | **0.96** | 0.72 | 0.70 |
| **FP가 색이 같음** (choco/saffron) | 0.41 | 0.69 | **0.94** | 0.89 |

**초기 블록 = 색/저수준, 후기 블록 = 텍스처/의미.**
- 초코하임 vs 택배박스는 **둘 다 갈색** → 색으론 원리적으로 못 가름 → 후기 블록 필요
- 초록 공룡 vs 흰 토끼, 파란 Febreze vs 크림 샤프란 → **색이 다름** → 초기 블록이 즉시 해결

⇒ **단일 블록이 전 객체를 만족시키는 것은 원리적으로 불가능합니다.** step-02가 문헌에서 가져온 "특정 층을 골라라"는 처방 자체가 우리 문제에 부적합했습니다.

## 1b-3. 🔴 현재 최종 블록은 인형에 **유해**합니다

**Dinosaur AUC 0.466 = 우연보다 나쁨.** b11은 초록 공룡과 흰 토끼를 **반대로 순위**매깁니다. 이것이 [[ism-crossobject-nms-rank]]가 발견한 인형 혼동의 또 다른 얼굴입니다 — NMS 순위를 appe로 바꿔 개선했지만, **그 appe 자체가 인형에 대해선 나쁜 신호**였습니다.

## 1b-4. ✅ 해법: 두 블록 평균 (b2+b9)

**단순 평균**(순위 정규화 불필요 = **모집단 독립**, 프로덕션에 안전):

| 설정 | **평균 AUC** | **최악 객체** | choco | saffron | milk | Febreze | Bear | Dinosaur |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| **b11 (현재)** | 0.766 | **0.466** | 0.934 | 0.849 | 0.882 | 0.907 | 0.561 | 0.466 |
| b2 단독 | 0.870 | 0.550 | 0.550 | 0.827 | 0.946 | 0.971 | 0.924 | 1.000 |
| b9 단독 | 0.796 | 0.487 | 0.996 | 0.894 | 0.796 | 0.940 | 0.487 | 0.664 |
| **b2+b9 평균** | **0.890** | **0.713** | 0.861 | 0.888 | 0.918 | 0.971 | 0.713 | 0.991 |
| b2+b5 평균 | 0.890 | **0.750** | 0.750 | 0.861 | 0.918 | 0.959 | 0.852 | 1.000 |

**최악 객체가 0.466 → 0.713으로 올라갑니다.** 객체별 튜닝 0, 신규 의존성 0, 전 객체 동일 레시피.

**bootstrap 유의성 (1000회, b2+b9 vs 현재 b11)**:

| 객체 | ΔAUC | 95% CI | |
|---|---:|---|---|
| **Dinosaur** | **+0.519** | [+0.355, +0.690] | ✅ **유의미 개선** |
| **Bear** | **+0.148** | [+0.065, +0.240] | ✅ **유의미 개선** |
| **Febreze** | **+0.064** | [+0.005, +0.132] | ✅ **유의미 개선** |
| saffron | +0.041 | [−0.041, +0.117] | ~ 불확실 |
| milk | +0.033 | [−0.087, +0.189] | ~ 불확실 |
| **choco** | **−0.072** | [−0.152, +0.009] | ~ **불확실(악화 쪽)** |

⇒ **3개 유의미 개선, 2개 불확실(개선 쪽), 1개 불확실(악화 쪽).** 유의미한 악화는 **0건**.

**leave-one-object-out (과적합 방지 — 5개 객체로 쌍을 고르고 남은 1개에서 평가)**:

| 보류 객체 | 5개로 고른 쌍 | 보류 객체 AUC | 현재 b11 | |
|---|---|---:|---:|---|
| saffron | b2+b5 | 0.861 | 0.849 | 개선 |
| milk | b2+b9 | 0.918 | 0.882 | 개선 |
| Febreze | b2+b5 | 0.959 | 0.907 | 개선 |
| Bear | b2+b9 | 0.713 | 0.561 | 개선 |
| Dinosaur | b5+b11 | 0.758 | 0.466 | 개선 |
| **choco** | b2+b2 | **0.550** | 0.934 | ❌ **악화** |

**6개 중 5개**에서, 그 객체를 **보지 않고 고른** 쌍이 현재 설정을 이깁니다. **choco만 예외** — 유일하게 "FP가 같은 색"인 객체라 다른 5개가 choco를 대변하지 못합니다.

## 1b-5. 정직한 한계

1. **b2+b9는 66쌍을 이 표본에서 탐색한 결과**입니다. LOO가 부분적으로 방어하나 과적합 위험이 남습니다. 다만 **기전(초기=색, 후기=텍스처)이 사전에 예측한 조합**이라 순수 탐색은 아닙니다.
2. **FP 표본이 적습니다**: milk 8, Dinosaur 11, saffron 14, Bear 18, Febreze 21 (choco 45). CI가 넓은 이유.
3. **라벨이 제 육안 판단**입니다.
4. **choco가 b2+b9에서 소폭 악화**할 수 있습니다(CI가 0을 포함). choco는 FP의 55%라 이게 중요합니다.
5. **AUC는 순위 지표**입니다 — 실제 게이트 적용 시 FP/FN 실효는 미측정.
6. **b2+b9는 별도 forward가 필요**합니다. 비용 미측정.

## 1b-6. 다음 액션 제안

1. **b2+b9를 프로덕션 반영 후 전수 A/B (권장)** — 유의미한 악화 0건, 최악 객체 0.466→0.713, 인형 실패의 근본을 건드림. `--nms-rank` 때 확립한 A/B 프로토콜(검출 수 불변 + 대조군 채점) 그대로.
2. **choco만 b9 가중을 높이는 객체별 예외** — 효과는 최대일 수 있으나 **객체별 하이퍼파라미터 = 우리를 세 번 태운 그것.** 비권장.
3. **choco FP 표본을 늘려 b2+b9의 choco 악화 여부 확정** — 현재 CI가 0을 걸침. 반나절.
4. **P2(YOLO 임계 하향)로 이동** — 비용 0, 미착수.

**권고: 1번.** 단 **choco 소폭 악화 가능성을 알고 들어가는 것**이며, 전수 A/B가 그것을 판정할 것입니다. 인형(Dinosaur +0.519, Bear +0.148)의 개선 폭이 choco의 잠재 손실(−0.072)보다 훨씬 큽니다.

---

# Phase 1c 시각화 (2026-07-15) — 그리고 시각화가 새 사실을 드러냄

사용자 지적: *"순수 글로서 봐야하는 입장에서는 믿음이 가지 않는다."* 타당함. AUC 숫자만으로는 **무엇이 사라지고 무엇이 개선되는지** 보이지 않음.

| 그림 | 질문 | 파일 |
|---|---|---|
| **Fig 1** | 실제로 갈리는가? | `figures/fig1-separation-before-after.png` |
| **Fig 2** | 왜 갈리는가? | `figures/fig2-why-early-blocks-see-color.png` |
| **Fig 3** | **무엇이 잘려나가는가?** | `figures/fig3-what-a-gate-would-cut.png` |

## 1c-1. 🔴 Fig 3가 AUC보다 중요한 사실을 드러냄

**게이트는 점수 "하위"를 자릅니다.** 그러므로 실제로 중요한 건 전체 AUC가 아니라 **하위 구간의 구성**입니다. 하위 12건을 열어보니:

| 객체 | | 하위 12건 중 **오검출** | 하위 12건 중 **진짜** | |
|---|---|---:|---:|---|
| **Dinosaur** | **현재 (b11)** | **0** | **12** | 🔴 **게이트가 진짜 공룡 12마리를 자르고 토끼는 0마리** |
| | **제안 (b2+b9)** | **10** | 2 | ✅ 대부분 토끼가 잘림 |
| **Bear** | 현재 (b11) | 4 | 8 | |
| | 제안 (b2+b9) | 6 | 6 | 소폭 개선 |
| **choco** | 현재 (b11) | **12** | **0** | ✅ 완벽 |
| | **제안 (b2+b9)** | **12** | **0** | ✅ **동일 — 손실 없음** |

**두 가지 새 사실**:

1. **현재 설정은 Dinosaur에서 정확히 거꾸로입니다.** 하위 12건이 **전부 진짜 초록 공룡**입니다. 어떤 게이트를 걸든 **진짜만 죽이고 토끼는 하나도 안 죽입니다.** AUC 0.466이 추상적 숫자가 아니라 이런 뜻이었습니다.

2. **🔑 choco의 AUC 악화(−0.072)는 게이트가 작동하는 곳에서 나타나지 않습니다.** 하위 12건이 **양쪽 다 오검출 12/진짜 0**으로 동일합니다. AUC 하락은 분포 **중간**에서 일어났고, **게이트는 중간을 건드리지 않습니다.**

⇒ **b2+b9 채택의 최대 리스크로 지목했던 "choco 악화"가, 실사용 관점에선 나타나지 않습니다.** 이건 시각화 없이는 발견하지 못했을 사실입니다 — AUC 하나만 보고 있었다면 존재하지 않는 위험을 걱정하며 채택을 미뤘을 것입니다.

## 1c-2. ⚠️ 표현 오류 정정 — "오검출이 늘어난다"로 읽히게 씀

사용자 지적: *"제안한 방식은 오히려 오검출이 늘어난다는 것 아니야?"*

**제 표 표현이 잘못됐습니다.** *"하위 12건 중 오검출 0 → 10"* 은 **오검출이 0개에서 10개로 늘었다**로 읽힙니다. 실제 의미는 정반대입니다.

**검출 건수는 전혀 변하지 않습니다.** b2+b9는 검출을 만들지도 지우지도 않고 **점수만 바꿉니다**. Dinosaur 표본은 항상 진짜 73 + 오검출 11입니다. 바뀌는 건 **그 11마리 토끼가 순위에서 어디에 앉느냐**입니다:
- 현재: 토끼가 중간~상위에 흩어짐 → 하위 12건은 전부 진짜 → 게이트가 **진짜만 죽임**
- 제안: 토끼가 바닥으로 가라앉음 → 하위 12건에 토끼 10 → 게이트가 **토끼를 죽임**

"하위에 오검출 10건"은 **오검출이 잘릴 자리로 내려왔다**는 뜻입니다.

## 1c-3. ✅ 올바른 운영 지표: 무손실 오검출 제거율

**"진짜를 단 하나도 잃지 않는 임계값에서, 오검출을 몇 % 자를 수 있나?"** (`figures/fig4-lossless-fp-removal.png`)

| 객체 | 진짜 | 오검출 | **현재 (b11)** | **제안 (b2+b9)** |
|---|---:|---:|---:|---:|
| **초코하임** | 36 | 45 | 14/45 (**31%**) | **25/45 (56%)** ✅ |
| 샤프란 | 68 | 14 | 2/14 (14%) | 2/14 (14%) — 동일 |
| 서울우유 | 76 | 8 | 3/8 (38%) | **4/8 (50%)** ✅ |
| 페브리즈 | 63 | 21 | 1/21 (5%) | **6/21 (29%)** ✅ |
| 갈색 곰 | 58 | 18 | 0/18 (**0%**) | 0/18 (**0%**) — 둘 다 실패 |
| **초록 공룡** | 73 | 11 | 0/11 (**0%**) | **7/11 (64%)** ✅ |
| **합계** | | **117** | **20/117 (17%)** | **44/117 (38%)** |

**무손실 제거율이 17% → 38%로 2.2배.** 악화된 객체 **0건**.

**🔑 그리고 이 지표에서 choco는 31% → 56%로 오히려 크게 개선됩니다.** AUC로는 −0.072였는데 운영 지표로는 +25%p입니다. **제가 최대 리스크로 지목했던 것이 실제로는 최대 수혜 중 하나였습니다.**

**남은 실패**: 갈색 곰은 양쪽 다 0% — b2+b9로도 곰과 흰토끼/초록공룡을 무손실로 못 가릅니다. 샤프란도 14%로 정체.

## 1c-4. 방법론 교훈

**AUC는 전 구간 순위 지표이고, 게이트는 한쪽 꼬리만 씁니다.** 서로 다른 질문에 답합니다. AUC만 보고 있었으면 존재하지 않는 위험(choco 악화)을 걱정하며 채택을 미뤘을 것입니다. **게이트 변경 평가엔 반드시 "무손실 제거율"을 함께 볼 것.**

---

# Phase 2 전수 A/B (2026-07-15) — ❌ 배포 기각. 게이트 재보정 없이는 순 손실

`appe_blocks: [2, 9]`를 기본값으로 넣고 **2,596프레임 전수 재생성**(9분). 비교 대상 = 2026-07-15 baseline.

## 2-1. 검출 5,476 → 6,357 (+16.1%)

| 객체 | 기존 → 신규 | 증감 | 새로 생긴 검출의 정체 (데이터셋 균등 표본 육안) |
|---|---|---:|---|
| **choco** | 756 → **1,106** | **+350** | **~23/24가 민무늬 택배박스** ❌ |
| **milk** | 512 → 667 | +155 | ~12/22가 택배박스 ⚠️ |
| **Mugcup** | 453 → 553 | +100 | 분홍머그 5, 명백 FP 7, 나머지 흐려서 판정불가 ⚠️ |
| **Rabbit** | 189 → **293** | **+104** | **~16/20이 진짜 흰 토끼** ✅ **FN 회복** |
| Bear | 560 → 531 | **−29** | |

## 2-2. 원인 — 내가 만든 실수

**`appe_gate: 0.55`를 그대로 두고 점수 체계만 바꿨습니다.** b2+b9는 점수 분포를 **전반적으로 위로** 밉니다(중앙값: saffron 0.726→0.762, Rabbit 0.671→0.736). 게이트는 **절대값**이라 그대로 두면 훨씬 헐거워집니다.

⇒ **순위 개선(인형)은 얻었지만, 게이트가 풀려 택배박스가 대량 유입.** choco FP가 377 → **700+**로 늘어날 것으로 추정됩니다. **순 손실.**

## 2-3. 조치: 기본값 [11]로 되돌림

**되돌림 검증 통과** — 검출 18건, baseline과 최대 오차 1.19e-07, **비트 단위 복귀**.

## 2-4. 🔴 그리고 다음 관문이 함정임이 드러남

같은 검출 수를 재현하는 게이트를 객체별로 역산하면:

| saffron | choco | Sikhye | Sauce | Febreze | Bear | milk | Mugcup | Dinosaur | Rabbit |
|---|---|---|---|---|---|---|---|---|---|
| .647 | .593 | .646 | .644 | .589 | .550 | .585 | .679 | .652 | **.725** |

**0.550 ~ 0.725로 객체마다 다릅니다.** 즉 b2+b9를 배포하려면 **객체별 임계값**이 필요하고, 그건 오늘 이미 **세 번 철회한 바로 그 함정**입니다.

## 2-5. 살아남은 것 / 죽은 것

**살아남음**:
- 기전(블록2=색 / 블록9=무늬, 색이 같은 FP는 후기 블록 필요)은 **유효하고 재현 가능**
- 현재 최종 블록이 **인형에 유해**(Dinosaur AUC 0.466 = 우연 이하)하다는 사실
- 구현 코드 — `appe_blocks` 한 줄로 언제든 켤 수 있음, 비용 +0.1%, 되돌리기 비트 보장
- **Rabbit FN 회복이 실재**함을 확인(+104건 중 ~80%가 진짜) — 이건 [[ism-crossobject-nms-rank]]의 NMS 순위 수정과 합쳐진 효과

**죽음**:
- "무손실 제거 17%→38%"를 **그냥 얻을 수 있다**는 기대. 그건 객체별 최적 임계값을 안다는 가정이었고, 그 가정이 곧 함정

## 2-6. 다음 액션 제안

1. **게이트 함정을 정면으로 푼다** — 우리는 이제 **501건 라벨**이 있음. 객체별 임계값을 dip test + bootstrap + 소스별 일관성으로 검증하며 세운다. 과거 3회 실패와의 차이: 그때는 라벨 없이 히스토그램 골짜기를 봤고(dip test가 그 골짜기가 허상임을 확인), 지금은 교차 데이터셋 라벨이 있음.
2. **게이트가 아니라 순위만 쓴다** — appe_blocks는 [11] 유지하되, **cross-object NMS 순위에만** b2+b9를 쓴다. 게이트는 안 건드리므로 검출 수 불변 → FP 유입 0, 인형 라벨만 개선. **가장 안전.**
3. **P2(YOLO 임계 하향)로 이동** — 미착수, 비용 0.
4. **여기서 멈추고 정리** — 오늘 얻은 것(NMS 순위 수정, 기전 규명, 501 라벨, 감사 데이터)을 문서화하고 종료.

**권고: 2번.** 게이트를 건드리지 않으므로 오늘 A/B가 드러낸 실패 모드(게이트 헐거워짐)가 원리적으로 발생하지 않고, 측정된 이득(인형 순위)만 취합니다. 검출 수 불변이 그 자체로 안전 증거가 됩니다.

---

# Phase 3 순위 전용 경로 (2026-07-15) — ✅ 채택. 전수 검증 통과

Phase 2의 실패 원인은 **게이트와 순위를 하나의 점수로 묶어 놓은 것**이었습니다. 분리하면 됩니다.

## 3-1. 설계 — 두 개의 별도 점수

| 결정 | 성격 | 쓰는 점수 | 결과 |
|---|---|---|---|
| **"버릴까 말까"** (appe_gate) | **절대** — 임계값 필요 | `masked_appe` ← `appe_blocks: [11]` **기존 그대로** | **동작점 불변** |
| **"두 이름 중 누가 이기나"** (cross-object NMS) | **상대** — 임계값 불요 | `rank_appe` ← `nms_rank_blocks: [2, 9]` **신규** | 판별력 개선 |

**순위에는 합격선이 없으므로 헐거워질 선 자체가 없습니다.** ⇒ Phase 2를 죽인 실패 모드(게이트가 풀려 택배박스 유입)가 **원리적으로 발생 불가**.

이 원리가 §Architectural Patterns의 *"생체인식 verification(1:1, 임계 필요) vs identification(1:N, argmax, 보정 불요)"* 구분과 정확히 일치합니다. 리서치가 예측한 대로입니다.

## 3-2. 전수 2,596프레임 결과

**🔒 안전성 (전부 측정)**:

| 지표 | 결과 | |
|---|---|---|
| **총 검출 수** | **5,476 → 5,476** | ✅ **완전 불변 — FP 유입 0건이 증명됨** |
| 게이트 점수가 바뀐 검출 | **0건** | ✅ 게이트 무손상 |
| 육안 확증 대조군 | 22/22 → 22/22 | ✅ 유지 |
| 되돌리기 | `nms_rank_blocks` 삭제 | ✅ 즉시 |
| 추가 연산 | +0.1% | ✅ |

**개선 — 208프레임(2,596 중)에서 이름표가 바뀜**:

| 잃은 라벨 → 얻은 라벨 | 건수 | 감사가 기록한 문제와 대조 |
|---|---:|---|
| **Febreze → saffron** | **63** | Febreze FP 68 (감사: *"대부분 샤프란 저그에 오발화"*) ✅ 일치 |
| **Bear → Rabbit** | **42** | Rabbit **FP 0 / FN 258** ✅ 일치 |
| **Dinosaur → Rabbit** | **39** | 〃 |
| **Bear → Dinosaur** | **29** | Bear FP 90 ✅ 일치 |
| saffron → Febreze | 11 | (역방향) |
| milk ↔ choco | 14 | |

순 이동: **Rabbit +81, Bear −68, Febreze −53.** 감사가 예측한 방향 그대로입니다.

**육안 검증: 표본 16건 → 16건 전부 오답에서 정답으로.** 샤프란 저그 4, 흰 토끼 8, 초록 공룡 4. 오답 0건.

**🔑 오늘 아침 NMS 수정이 남긴 유일한 회귀(105652 f288, 진짜 공룡을 Bear로)도 해소**됐고, 무승부였던 f284도 정답으로 갔습니다.

## 3-3. 적용된 설정

```yaml
defaults:
  appe_blocks: [11]        # 게이트 — 기존 그대로 (건드리지 않음)
  nms_rank_blocks: [2, 9]  # NMS 순위 — 신규
```

## 3-4. 남은 문제 (이 변경의 범위 밖)

- **초코하임 택배박스 FP 377건** — 택배박스를 주장하는 **경쟁 라벨이 없어** 순위로 못 고칩니다. 게이트를 건드려야 하고 그건 객체별 임계값 함정(§2-4).
- **FN 1,392건** — 검출기가 후보를 안 내놓은 것은 순위로 못 살립니다.
- **Bear/saffron 무손실 제거율 0%/14%** — 여전히.

## 3-5. 발표 자료

`planning-artifacts/research/slides/` (16:9, 1920×1080):

| 슬라이드 | 내용 |
|---|---|
| 1 | 문제 — 현재 설정이 진짜 공룡을 최하위로 매김 (실제 크롭) |
| 2 | 원인 — 초기=색/후기=무늬, 두 무리가 교차하는 곡선 (실제 크롭) |
| 3 | 해법 — 왜 +0.1%인가 |
| 4 | 잠재력 — 무손실 제거율 17→38% (단서 명시) |
| 5 | 한계 |
| **6** | **전수 A/B가 게이트 변경을 기각 — 실패도 결과** |
| **7** | **실제 프레임 before/after — 같은 사진, 이름표만 바뀜** |
| **8** | **최종 — 안전성 지표 + 208프레임 내역** |

## 3-6. 다음 액션 제안

1. **여기서 확정하고 P2로 이동** — 순위 수정은 검증 완료·배포 가능. 다음은 **YOLO-World 자체 conf 임계 하향(비용 0)** 으로 FN 1,392를 건드림. 미착수.
2. **choco 게이트 함정 정면 돌파** — 501건 라벨 + dip test/bootstrap/소스별 일관성으로 객체별 임계값을 세운다. 오늘 3회 실패와의 차이는 라벨 보유.
3. **`--nms-rank` 기본값 재확인** — 현재 `appe`. `rank_appe`가 있으므로 그대로 두면 됨.

**권고: 1번.** FP는 오늘 680건 중 인형/Febreze 계열을 상당수 정리했고, 남은 최대 덩어리(choco 377)는 게이트 함정에 막혀 있습니다. **FN 1,392가 FP의 2배**이고 P2는 아직 손도 안 댔으며 비용이 0입니다.
