---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments: []
workflowType: 'research'
lastStep: 6
research_type: 'technical'
research_topic: '2025년 이후 object tracking 최신 동향 + 객체 landmark(ID+포즈)별 confidence 지표 방법론'
research_goals: 'object_memory 모듈의 landmark별 confidence(신뢰도) 지표를 설계하여 다양한 테스트 환경에서 객체 상태(active/lost/tentative 등)가 제대로 측정되도록 하는 방법론 확보'
user_name: 'ldh'
date: '2026-07-03'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-07-03
**Author:** ldh
**Research Type:** technical

---

## Research Overview

본 리서치는 두 질문에 답한다: (1) 2025년 이후 최신 연구는 object tracking을 어떻게 수행하는가, (2) 객체 landmark(ID+포즈)별 confidence 지표를 어떻게 설계해야 다양한 테스트 환경에서 객체 상태가 정확히 측정되는가. 조사는 웹 검색 11회(MOT 동향, 존재확률 필터, occlusion-aware 추적, semantic SLAM 데이터 연관, 아키텍처 프레임워크, 평가 지표)로 수행했으며 모든 주장에 출처를 병기했다.

핵심 결론: 검색된 표준 방법론은 **Bernoulli/IPDA 계열의 "존재확률 r"** — confidence를 휴리스틱 점수가 아니라 "객체가 실존할 확률"로 정의하고, 3줄의 재귀식으로 갱신하며, **시야 밖 미검출은 감점하지 않는 성질(P_D 연동)**이 수식에서 자동으로 나온다. 이는 현 object_memory의 최대 격차(카메라가 다른 곳을 볼 때 confidence가 부당하게 깎이는 문제)를 정확히 해결한다. 이산 상태기계(tentative/active/lost)는 폐기하지 않고 r의 임계 판정으로 파생시키는 하이브리드가 실무 표준이다.

상세 요약은 문서 말미의 **Research Synthesis — Executive Summary** 절 참조.

---

<!-- Content will be appended sequentially through research workflow steps -->

## Technical Research Scope Confirmation

**Research Topic:** 2025년 이후 object tracking 최신 동향 + 객체 landmark(ID+포즈)별 confidence 지표 방법론
**Research Goals:** object_memory 모듈의 landmark별 confidence(신뢰도) 지표를 설계하여 다양한 테스트 환경에서 객체 상태(tentative/active/lost 등)가 제대로 측정되도록 하는 방법론 확보

**Technical Research Scope:**

- Architecture Analysis - 최신(2025~) MOT/3D tracking·object-level SLAM 시스템 구조, memory 기반 트래커, landmark 관리 구조
- Implementation Approaches - 베이지안 existence probability, track score 기반, 공분산 기반 불확실성, 학습형 confidence 산정 방법론
- Technology Stack - 대표 오픈소스 구현과 Python 오프라인 러너(SAM-6D + ORB-SLAM3)로의 이식성
- Integration Patterns - 검출 score → association 게이트 → lifecycle 전이 → confidence 갱신 결합, 기존 상태기계와의 접점
- Performance Considerations - confidence 캘리브레이션 및 tracking 지표(HOTA/IDF1) 기반 검증, 환경별 임계값 설계

**Research Methodology:**

- Current web data with rigorous source verification
- Multi-source validation for critical technical claims
- Confidence level framework for uncertain information
- Comprehensive technical coverage with architecture-specific insights

**Scope Confirmed:** 2026-07-03

## Technology Stack Analysis

### Object Tracking 패러다임 지형 (2025~)

2025년 이후 MOT(Multi-Object Tracking) 연구는 크게 두 갈래로 정리된다.

1. **Tracking-by-Detection (TBD) — 여전히 production 주류.** 검출기(YOLO 계열 등) + 모션모델(Kalman) + association(IoU/Re-ID)의 모듈형 구조. ByteTrack(모든 검출 박스를 confidence 계층별로 2단 association), OC-SORT, DeepSORT 계열이 2026년 현재도 실무 표준으로 정리됨. _Source: [Forasoft — MOT trackers in production 2026](https://www.forasoft.com/learn/ai-for-video-engineering/articles-ai/multi-object-tracking-deepsort-bytetrack-ocsort)_
2. **End-to-End Transformer 계열 — 연구 주류.** MOTR/TrackFormer에서 출발해, track query가 프레임 간 identity를 직접 운반하는 구조. 2025년에는 spatio-temporal transformer(STMOTR), 멀티카메라 3D 트래킹(SynCL), temporal query denoising(TQD-Track) 등으로 확장. _Source: [IET CV 2025 — End-to-End MOT with Spatio-Temporal Transformers](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/cvi2.70052), [Transformers in MOT — literature review](https://arxiv.org/pdf/2406.16784), [MOTR](https://arxiv.org/pdf/2105.03247)_

추가 트렌드: open-vocabulary tracking 벤치마크(OVT-B)의 등장 — 텍스트 프롬프트 기반 검출(우리의 YOLO-World 사용과 동일 계열)이 tracking으로 확장되는 흐름. _Source: [OVT-B](https://arxiv.org/pdf/2410.17534), [CAMELTrack — 학습형 multi-cue association](https://arxiv.org/pdf/2505.01257)_

**우리 프로젝트와의 접점**: object_memory는 "저프레임(stride) 검출 + 외부 SLAM 포즈"라는 특수 조건이므로, end-to-end 계열보다 **TBD의 track management 기법**(lifecycle·score·existence probability)이 직접 이식 가능한 스택이다.

### Landmark Confidence 산정 방법론 스택 (핵심 조사 대상)

객체별 landmark(ID+포즈)에 신뢰도를 부여하는 방법론은 원리 수준에서 4계열로 분류된다.

**A. Count 기반 (M-of-N) — 현재 object_memory 방식의 정식 명칭.**
tentative → N프레임 중 M회 검출 시 confirmed, 연속 miss 시 deleted. 산업 구현(NVIDIA DeepStream NvDCF 등)의 기본값이며, 검출 confidence 임계(예: 0.7 이상 3연속)와 결합하기도 함. 단순하지만 임계값이 환경(fps, 검출률)에 민감. _Source: [USPTO — confirmed/tentative track patent](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9495778), [NvDCF Tracker Configuration](https://deepwiki.com/NVIDIA-AI-IOT/deepstream_reference_apps/7.2.1-nvdcf-tracker-configuration)_

**B. Track Score (log-likelihood ratio) 기반 — 고전이지만 가장 실용적인 업그레이드 경로.**
track score = "이 측정이 실제 트랙에서 나왔을 확률 / 오검출에서 나왔을 확률"의 로그비를 프레임마다 누적. 검출 hit는 score를 올리고 miss는 내리며, **확인은 SPRT(순차확률비검정) 임계, 삭제는 '최대 score 대비 하락폭' 임계**로 판정 — 우리의 gain/decay(+0.2/−0.1)를 확률적으로 정당화한 형태다. MATLAB Sensor Fusion Toolbox `trackScoreLogic`으로 표준화되어 있음. 오경보 밀도가 높은 환경에서 삭제 임계에 민감하다는 한계가 알려져 있고(Shiryayev SPRT 변형으로 보완), 이는 "다양한 테스트 환경" 요구사항과 직결된다. _Source: [MathWorks trackScoreLogic](https://www.mathworks.com/help/fusion/ref/trackscorelogic.html), [Shiryayev SPRT 기반 MHT](https://link.springer.com/article/10.1007/s11432-016-5570-4), [MHT + detection processing](https://pmc.ncbi.nlm.nih.gov/articles/PMC6928886/)_

**C. 베이지안 존재확률 (Random Finite Set / Bernoulli) — 이론적으로 가장 원리적.**
각 트랙을 Bernoulli 성분으로 모델링해 **존재확률 r ∈ [0,1]을 명시적 상태 변수**로 유지: 검출되면 베이즈 갱신으로 r↑, 시야 내 미검출이면 검출확률 P_D를 반영해 r↓, 시야 밖이면 r 유지(생존확률만 적용). PMBM 필터가 대표 (검출 전 객체=Poisson, 검출된 객체=multi-Bernoulli mixture). Python 레퍼런스 구현 존재. "시야 밖 미검출은 감점하지 않는다"는 성질이 occlusion/loop 환경에서 count 기반의 최대 약점을 해결한다. _Source: [PMBM 개요 및 conjugacy](https://arxiv.org/pdf/1605.06311), [PMBM for AV](https://arxiv.org/abs/2103.07783), [erikbohnsack/pmbm (Python)](https://github.com/erikbohnsack/pmbm), [ModTrack — PHD 필터 + 공분산 전파, 2026](https://arxiv.org/pdf/2603.15812)_

**D. 검출 품질/예측 신뢰도 연동형 (2025 최신 경향).**
검출기의 confidence·localization 품질을 association과 lifecycle에 직접 주입: prediction confidence로 association을 유도하고, regression confidence의 점진적 하락을 "객체가 가려지는 중"의 신호(weak/strong object)로 해석. Deep LG-Track(localization confidence 활용), dynamic-confidence 3D MOT(2025) 등. _Source: [Confidence-Based Trajectory Prediction (Sensors 2025)](https://www.mdpi.com/1424-8220/25/23/7221), [Deep LG-Track](https://arxiv.org/pdf/2504.01457), [dynamic-confidence 3D MOT](https://www.sciencedirect.com/science/article/pii/S0263224125012230), [prediction-confidence-guided DA (IEEE)](https://ieeexplore.ieee.org/document/9352500/)_

### Object-level SLAM에서의 landmark 신뢰도 관리

object SLAM 계열은 객체를 map landmark로 유지하며 존재/영속성(permanence)을 확률적으로 다룬다: 정적 객체를 확률밀도함수로 모델링하는 object-aware semantic mapping, 시공간 일관성+확률 전파로 동적 객체를 분리하는 접근, foundation model 피드백으로 잘못된 landmark를 교정하는 연구(잘못된 초기 landmark 교정 — 우리의 1번 문제와 동일 주제) 등. _Source: [object-aware semantic mapping w/ PDFs (Sci Rep)](https://www.nature.com/articles/s41598-026-40498-3), [Semantic SLAM survey (ScienceDirect 2025)](https://www.sciencedirect.com/science/article/pii/S2667305325001176), [Learning from Feedback — object SLAM + foundation models](https://arxiv.org/html/2411.06752v1), [LLM-enhanced object SLAM priors](https://arxiv.org/pdf/2509.21602)_

### 6D Pose 자체의 불확실성 정량화

포즈 측정값 하나하나의 신뢰도(측정 노이즈)를 얻는 스택: deep ensemble 기반 UQ, 렌더-비교 기반의 간단한 검증(MaskVal — 추정 포즈로 렌더한 마스크와 세그멘테이션 마스크의 일치도로 pose 품질 점수화), 부분 관측 영역을 명시 표기하는 UA-Pose. **MaskVal 방식은 SAM-6D 산출물(마스크+포즈+CAD)만으로 후처리 가능해 우리 파이프라인에 즉시 이식 가능성이 높다** (신뢰수준: 중 — 구현 난이도 검증 필요). _Source: [Deep Ensembles for 6D pose UQ](https://arxiv.org/abs/2403.07741), [MaskVal](https://arxiv.org/pdf/2409.03556), [UA-Pose](https://arxiv.org/abs/2506.07996), [pose distribution estimation](https://arxiv.org/pdf/2512.07211)_

### 기술 채택 동향 요약

- 연구는 end-to-end로 이동 중이나, **저지연·모듈형 요구가 있는 로보틱스 실무는 TBD + 원리적 track management**가 여전히 표준 (신뢰수준: 높음, 복수 소스 일치)
- confidence를 "부가 지표"가 아니라 **association·lifecycle의 1급 입력**으로 쓰는 것이 2025년의 공통 방향 (검출 confidence 계층화 → ByteTrack, 예측 confidence → DA 유도, regression confidence → occlusion 신호)
- 존재확률(r)과 포즈 불확실성(공분산)을 **분리된 두 지표**로 유지하는 것이 RFS 계열의 정석 — 단일 스칼라 confidence로 합치면 "위치는 확실한데 존재가 불확실"한 상태를 표현 못 함

## Integration Patterns Analysis

confidence가 파이프라인의 각 단계(검출 → association → lifecycle → 장기 재인식)에 어떻게 **결합**되는지의 패턴 분석. object_memory의 데이터 흐름(SAM-6D score → 게이트 → 상태기계 → confidence)에 직접 대응시킨다.

### 패턴 1: 검출 score의 계층적 소비 (BYTE 2단 association)

저-confidence 검출을 버리지 않고 **2단계로 나눠 소비**하는 것이 현대 TBD의 표준 통합 패턴이다: 1단계에서 고-score 검출을 기존 트랙과 매칭하고, 2단계에서 남은 트랙을 저-score 검출과 IoU만으로 매칭한다. 핵심 통찰은 "저-score 검출도 기존 트랙의 예측과 잘 맞으면 배경이 아니라 (가려지는 중인) 진짜 객체일 확률이 높다"는 것 — **score의 신뢰성은 트랙 문맥에 따라 달라진다**는 원리다. _Source: [ByteTrack (ECCV 2022)](https://www.ecva.net/papers/eccv_2022/papers_ECCV/papers/136820001.pdf), [BYTE 해설](https://www.alphaxiv.org/overview/2110.06864), [BoostTrack++ — tracklet 정보로 검출 보강](https://arxiv.org/pdf/2408.13003)_

**object_memory 접점**: 현재 score_floor는 단일 임계다. "이미 active인 landmark 근처의 저-score 검출은 수락(=BYTE 2단계), 신규 landmark 생성은 고-score만 허용"으로 나누면 FP 생성 억제와 occlusion 유지가 동시에 가능하다.

### 패턴 2: 가시성 연동 존재확률 갱신 (expected P_D) — 본 리서치의 핵심 발견

최신(2026-04) occlusion-aware MOT는 각 객체에 **센서 대비 가시성을 반영한 기대 검출확률 P_D를 부여**하고, "미검출" 증거의 감점량을 P_D에 비례시킨다: 시야 안·비가림인데 미검출이면 존재확률을 크게 깎고, 시야 밖/가림 상태의 미검출은 거의 깎지 않는다. 고전 IR 트래킹의 coast mode(가림 예측 시 트랙을 '관성 유지' 모드로 전환, 재출현 시 재잠금)도 같은 원리의 이산 버전이다. _Source: [Occlusion-Aware MOT via Expected Probability of Detection](https://arxiv.org/abs/2511.20239), [coast mode target tracking](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5948633/), [FOV 밖 트랙 유지 — occlusion-aware risk assessment](https://ar5iv.labs.arxiv.org/html/1809.04629)_

**object_memory 접점**: 우리는 SLAM 포즈(T_map_cam)와 landmark 포즈(T_map_obj)를 모두 알므로 **P_D를 계산할 수 있는 특권적 위치**에 있다 — landmark를 카메라 프레임에 재투영해 (a) 화면 안인지 (b) 깊이가 양수인지만 확인해도 "시야 밖 미검출은 감점하지 않는다"를 구현할 수 있다. 현재의 무조건 miss+1/decay−0.1은 카메라가 다른 곳을 보는 동안 객체 confidence를 부당하게 깎는다(SAM_loop2에서 잘 관측된 객체들이 lost가 된 주원인). 이것이 "다양한 테스트 환경에서 상태가 제대로 측정"되기 위한 최우선 통합 지점이다. (신뢰수준: 높음 — 원리는 복수 소스 일치, 재투영 구현은 기존 visualize 코드에 이미 존재)

### 패턴 3: 불확실성 반영 기하 게이팅 (Mahalanobis)

semantic SLAM/landmark 매핑의 표준 association은 (1) 클래스/semantic 일치 확인 후 (2) **Mahalanobis distance**(공분산 반영 거리)를 게이트로 쓰는 2단 하이브리드다. 고정 유클리드 임계(우리의 0.15 m)와 달리, 불확실성이 큰 초기 landmark엔 넓은 게이트, 관측이 쌓여 확실해진 landmark엔 좁은 게이트가 자동 적용된다. 모든 landmark와의 거리가 임계 초과일 때만 신규 landmark를 생성하는 것도 표준. _Source: [Freiburg data association 강의자료](http://ais.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/rob2-20-dataassociation.pdf), [BPDA-GMM — 베이지안 확률적 DA (2026)](https://arxiv.org/pdf/2606.04618), [LOSS-SLAM](https://arxiv.org/pdf/2404.04377), [Semantic SLAM 개요](https://www.emergentmind.com/topics/semantic-simultaneous-localization-and-mapping-semantic-slam)_

**object_memory 접점**: 현재 tentative_gate_mult=3.0은 이 원리의 수동 근사다. landmark별 위치 분산(관측들의 표본분산)을 유지하면 게이트가 데이터 적응형이 되어, 환경(검출 노이즈 수준)이 바뀌어도 임계 재튜닝이 불필요해진다.

### 패턴 4: 장기 재인식(re-ID)의 계단식 결합

lost 트랙 복구는 **기하 우선, appearance는 2차 복구 수단**으로 계단식(cascaded) 적용이 최고 성능이라는 것이 2026 체계적 연구의 결론이다. appearance 템플릿은 관측 embedding의 이동평균으로 유지(최근 가중)하는 것이 표준. 단 appearance는 조명·포즈 변화에 민감하므로 주 신호로 쓰면 오히려 ID 유지가 불안정해진다. _Source: [Does Appearance Help? (2026)](https://arxiv.org/html/2606.07233), [unsupervised re-ID + occlusion estimation](https://arxiv.org/pdf/2201.01297), [long-term re-ID benchmark](https://dl.acm.org/doi/10.1145/3503161.3548327)_

**object_memory 접점**: Story 5(long-term association)에서 remembered landmark 재인식은 (1) 클래스 (2) 맵 위치 반경 (3) 필요시 appearance 순의 계단식이 근거 있는 설계다. 정적 객체 가정에서는 맵 위치가 사람 추적의 appearance보다 훨씬 강한 신호이므로 (1)+(2)만으로 충분할 가능성이 높다 (신뢰수준: 중).

### 통합 아키텍처 요약 — object_memory 대응표

| 파이프라인 단계 | 현재 구현 | 조사된 표준 패턴 |
| --- | --- | --- |
| 검출 score 소비 | score_floor 단일 임계 | BYTE식 계층 소비: 신규 생성=고-score, 기존 유지=저-score 허용 |
| 미검출 처리 | 무조건 miss+1, conf−0.1 | expected P_D: 시야 밖/가림이면 감점 억제 (핵심 격차) |
| association 게이트 | 고정 0.15 m / 45° (+tentative ×3) | Mahalanobis + landmark별 분산 (데이터 적응형) |
| confidence 갱신 | 선형 ±(0.2/0.1) | track score(LLR) 또는 Bernoulli 존재확률 베이즈 갱신 |
| 장기 재인식 | 이름 키 재사용 | 클래스 → 맵 반경 → (appearance) 계단식 |

## Architectural Patterns and Design

### 상태 표현 아키텍처: 이산 상태기계 vs 연속 존재확률

핵심 설계 결정은 "객체 상태를 무엇으로 표현하는가"이다. 조사 결과 두 접근은 대립이 아니라 **계층 관계**가 정석이다.

- **IPDA/JIPDA 아키텍처 (원리적 기반)**: track 존재를 가정하지 않고 **존재확률 자체를 확률 이벤트로 모델링**, 매 프레임 data association과 존재확률을 하나의 재귀식으로 동시 갱신한다. 존재확률이 track 품질 지표를 겸해 true/false track 판별, 자동 initiation/termination까지 담당 — "생성·유지·삭제가 별도 휴리스틱이 아니라 하나의 확률 갱신의 임계 판정"이라는 구조다. _Source: [IPDA (Musicki & Evans, IEEE)](https://ieeexplore.ieee.org/document/293185/), [JIPDA](https://www.researchgate.net/publication/3003758_Joint_Integrated_Probabilistic_Data_Association_JIPDA), [Markov chain JIPDA](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5750805/)_
- **하이브리드 패턴 (실무 표준)**: 내부적으로 연속 지표(존재확률 r 또는 track score)를 유지하고, **이산 상태(tentative/confirmed/deleted)는 그 지표의 임계 판정으로 파생**시킨다. MATLAB `trackScoreLogic`, 산업 트래커(DeepStream) 모두 이 구조. 이산 상태는 소비자(표시·이벤트)용 인터페이스이고, 연속 지표가 진실의 원천(source of truth)이다.

**object_memory 설계 시사점**: 기존 상태기계(tentative/active/lost/remembered)는 유지하되, 전이 조건을 "관측/miss 카운트"에서 "**존재확률 r의 임계 통과**"로 교체하는 것이 조사된 정석 아키텍처다. 상태기계는 Unity 이벤트 계약(Story 6)의 인터페이스로 그대로 남는다. (신뢰수준: 높음)

### 모듈 분해 패턴: Stone Soup 파이프라인

영국 국방연구 계열 오픈소스 추적 프레임워크 Stone Soup은 track 관리를 5개 교체 가능한 컴포넌트로 표준 분해한다: **Detector → DataAssociator → Updater → Deleter → Initiator**. 처리 순서도 규정적이다 — (1) 활성 트랙과 검출을 association, (2) 매칭된 트랙은 Updater로 갱신, 미매칭 트랙은 **예측값으로 갱신(coast)**, (3) Deleter가 삭제 판정, (4) 남은 미매칭 검출을 Initiator가 신규 트랙으로. _Source: [Stone Soup Framework Design](https://stonesoup.readthedocs.io/en/latest/design.html), [Trackers](https://stonesoup.readthedocs.io/en/latest/stonesoup.tracker.html), [Initiators](https://stonesoup.readthedocs.io/en/v0.1b7/stonesoup.initiator.html)_

**object_memory 대응**: 현 `object_memory_runner`는 이 분해와 이미 유사하나 Deleter/Initiator가 명시 컴포넌트가 아니라 루프 안 인라인 코드다. fix3(pruning)에서 삭제 규칙을 Deleter 함수로, 신규 생성 규칙을 Initiator 함수로 분리하면 이 표준 구조와 일치하며, 이후 confidence 방식 교체(count→r) 시 Updater만 갈아끼우면 된다.

### 영속 객체 레이어: 3D Scene Graph (Kimera/Hydra)

로보틱스 상위 표현의 표준은 계층형 3D scene graph다: metric-semantic mesh 위에 **객체 레이어**(노드=영속 객체, 에지=공간 관계)를 두고, 장기 자율성을 위해 표현의 **영속성(persistence)**을 1급 요구사항으로 설계한다. Hydra는 이를 실시간·병렬 아키텍처로 확장했고, loop closure 시 **객체 노드의 포즈를 소급 갱신**하는 것이 특징이다. _Source: [Kimera — 3D dynamic scene graphs (IJRR)](https://journals.sagepub.com/doi/abs/10.1177/02783649211056674), [Hydra (arXiv)](https://arxiv.org/pdf/2201.13360), [Scene Representations for Robotic Spatial Perception (Annual Reviews)](https://www.annualreviews.org/content/journals/10.1146/annurev-control-040423-030709)_

**object_memory 대응**: 우리의 object_memory는 scene graph의 "객체 레이어"에 해당한다. Unity 3D Map 표시 목표를 고려하면, 객체 노드가 SLAM 맵 좌표에 anchoring되고 loop closure 후 궤적이 갱신되면 T_map_obj도 재계산 가능해야 한다는 설계 요구가 도출된다(현재 오프라인 러너는 최종 궤적을 쓰므로 자동 충족, 라이브 전환 시 고려 필요). (신뢰수준: 중 — 라이브 요구는 미래 시점)

### 설계 결정 요약 (ADR 형식)

| 결정 | 선택지 | 조사 근거 기반 권고 |
| --- | --- | --- |
| confidence의 본질 | 스칼라 휴리스틱 / track score(LLR) / 존재확률 r | **존재확률 r** (IPDA식) — 해석 가능(=P(객체 실존)), P_D 연동 자연스러움. 최소 변경 원하면 track score |
| 이산 상태 | 폐기 / 유지 | **유지** — r의 임계 파생으로 재정의, Unity 계약 인터페이스 |
| 포즈 불확실성 | r에 합산 / 분리 유지 | **분리** — r(존재)과 위치 분산(공간)은 다른 질문에 답함 |
| 모듈 구조 | 인라인 루프 / 컴포넌트 분해 | Initiator/Updater/Deleter **함수 분리** (Stone Soup 패턴) — 방법론 교체 비용 최소화 |
| 확장성 | — | landmark당 O(1) 갱신(재귀식)이라 오프라인/라이브 모두 문제없음; PMBM 전체 도입은 가설 폭발로 과잉 (신뢰수준: 높음) |

## Implementation Approaches and Technology Adoption

### 핵심 갱신 수식: Bernoulli 존재확률 재귀

landmark별 존재확률 r ∈ [0,1]의 표준 재귀식 (Bernoulli 필터 / IPDA 계열):

**예측 (매 프레임):**
```
r_pred = p_b · (1 − r) + p_s · r
```
p_s = 생존확률(정적 객체 가정이면 ≈1.0, 예: 0.999), p_b = 신생확률(기존 landmark엔 0). 정적 객체에서는 사실상 r_pred ≈ r.

**갱신 — 시야 내 미검출 시:**
```
r' = r · (1 − P_D) / (1 − r · P_D)
```
P_D = 그 프레임의 기대 검출확률. **P_D가 0이면(시야 밖/가림) r' = r — 감점 없음이 수식에서 자동으로 나온다.** 이것이 count 기반 대비 본질적 우위다.

**갱신 — 검출·수락 시:** r이 1을 향해 증가 (클러터 밀도를 무시하는 간이형은 r' ≈ 1에 근접시키거나, 검출 score를 반영해 `r' = r·s / (r·s + (1−r)·c)` 형태로 완화). _Source: [Bernoulli Filtering for Multi-Sensor Tracking (2026)](https://arxiv.org/abs/2606.09573), [IPDA (Musicki & Evans)](https://ieeexplore.ieee.org/document/293185/), [Bernoulli filter DOA tracking](https://pmc.ncbi.nlm.nih.gov/articles/PMC6210454/)_

전체 수식 중 구현에 필요한 것은 위 3줄이 전부다 — 순수 Python 스칼라 연산, landmark당 O(1). (신뢰수준: 높음)

### P_D(기대 검출확률) 추정 — 우리 파이프라인의 구현 방법

1. **FOV 판정**: T_map_obj를 T_map_cam⁻¹로 카메라 프레임에 재투영 → (a) z>0 (b) 픽셀 좌표가 이미지 내 → P_D_base (예: 0.7~0.9), 아니면 P_D ≈ 0. 재투영 코드는 `visualize_object_memory.py`에 이미 존재하므로 이식 비용 낮음.
2. **(선택) 거리 감쇠**: 검출기 유효 거리 밖이면 P_D 하향 — 9-bag audit에서 proposal-miss가 FN의 68%였으므로, 거리·크기별 P_D 캘리브레이션 근거 데이터(pseudo-GT 1241프레임)를 이미 보유.
3. **(선택) 상호 가림**: 다른 landmark가 시선을 막으면 P_D 하향 — 최신 연구가 이 방향이나 MVP에선 생략 가능. _Source: [Occlusion-Aware MOT via Expected P_D](https://arxiv.org/abs/2511.20239)_

### SAM-6D score의 의미와 측정 신뢰도로의 활용

SAM-6D ISM의 object matching score는 **semantic·appearance·geometric 3항의 결합**이다: (1) 제안영역-템플릿 클래스 embedding 코사인 유사도 top-K 평균, (2) 최적 매칭 템플릿과의 appearance 유사도, (3) 러프 포즈로 투영한 2D 박스와 제안 박스의 IoU. 즉 이 score는 "이 검출이 해당 객체일 신뢰도"이며 존재확률 갱신의 검출 항 입력으로 의미가 정합하다. 단 절대 임계가 아닌 forced-output 상대점수임(기존 분석과 일치)을 감안해 **캘리브레이션(예: bag별 분포 정규화) 후 사용** 권장. 포즈 측정 자체의 품질은 별도로 MaskVal식(추정 포즈 렌더 마스크 vs 세그 마스크 IoU) 후처리로 얻을 수 있다. _Source: [SAM-6D (arXiv)](https://arxiv.org/abs/2311.15707), [MaskVal](https://arxiv.org/pdf/2409.03556)_

### 검증 체계 (다양한 테스트 환경에서의 상태 측정 정확도)

- **트래킹 표준 지표**: HOTA = √(DetA·AssA) — 검출·association·위치 품질을 결합 평가. MIT 라이선스 오픈소스 TrackEval로 계산. ID 일관성은 IDSW(identity switch 수). 우리 오프라인 러너 출력(frame별 object_id+포즈)을 TrackEval 포맷으로 내보내면 bag별 정량 비교 가능. _Source: [TrackEval (GitHub)](https://github.com/JonathonLuiten/TrackEval), [MOT metrics 해설](https://miguel-mendez-ai.com/2024/08/25/mot-tracking-metrics)_
- **존재확률 자체의 검증**: pseudo-GT(9-bag audit 방식)로 "프레임 t에 객체 o가 실존" 라벨을 만들고, r의 캘리브레이션(r=0.8인 landmark가 실제로 80% 존재하는가)을 reliability diagram으로 확인 — 이것이 "상태가 제대로 측정"의 직접 검증이다.
- **환경별 시나리오 매트릭스**: 기보유 bag이 이미 occlusion(SAM_occlusion)/loop 재방문(loop1·2)/원거리·측면(circle) 축을 커버 — 환경별 임계 재튜닝 없이 동일 파라미터로 상태 정확도가 유지되는지가 합격 기준.

## Technical Research Recommendations

### Implementation Roadmap (기존 수정 계획과의 통합)

1. **[완료] fix2**: seen/accepted 분리 — 존재확률 도입의 전제(거부≠관측).
2. **fix3 (pruning)**: 예정대로 진행하되 Deleter/Initiator 함수 분리(Stone Soup 패턴)로 구현 — 이후 교체 비용 최소화. 임계는 임시로 count 기반 유지.
3. **fix-r (존재확률 도입)**: `confidence` 필드의 갱신 로직을 Bernoulli 재귀로 교체(Updater 교체). FOV 기반 P_D 1단계(시야 안/밖 이분)만 도입. 상태 전이·pruning 조건을 r 임계로 재정의 (예: r>0.6 승격, r<0.3 lost, r<0.05 삭제/remembered 판정).
4. **fix1 (re-seed)**: 오염 landmark는 r이 자연 하락 → 삭제 → 재-시드로 대부분 해소. 잔여 케이스(active 오염)만 증거 경쟁 로직 추가.
5. **fix4 (융합 개선)** + Mahalanobis 게이트: landmark별 위치 분산 유지가 선행 — 게이트 적응화와 융합 가중치 개선을 한 번에.
6. **검증**: TrackEval 도입 + r 캘리브레이션 리포트를 4-bag(+신규 SLAM_*_loop PEM 생성 후 7-bag)으로.

### Technology Stack Recommendations

- 순수 Python 스칼라 구현 유지(의존성 추가 불필요) — Bernoulli 재귀 3식 + FOV 재투영이면 충분
- 평가만 외부 도구: TrackEval(MIT), 필요시 erikbohnsack/pmbm은 참조 코드로만
- PMBM/JIPDA 풀 프레임워크 도입은 비권장 (클래스당 소수 landmark 규모에 과잉)

### Skill Development Requirements

- Bernoulli/IPDA 존재확률 갱신의 유도 이해(핵심 3식) — 본 문서 인용 논문 2편으로 충분
- 캘리브레이션 평가(reliability diagram) 기초

### Success Metrics and KPIs

| 지표 | 목표 |
| --- | --- |
| 상태 판정 정확도 | pseudo-GT 대비 프레임별 존재 판정 F1 상승 (bag별 임계 재튜닝 없이) |
| 유령 landmark | 최종 landmark 중 실존하지 않는 객체 0개 (현재 bag당 1~2개) |
| ID 안정성 | IDSW 0 유지 (정적 객체이므로 달성 가능해야 함) |
| r 캘리브레이션 | reliability diagram 편차 |r − 실측 존재율| < 0.15 |
| lifecycle thrash | active↔lost 전이 횟수 대폭 감소 (loop2 기준 39회 → 한 자릿수) |

---

# Research Synthesis: SLAM 연동 Object Memory를 위한 Landmark 존재확률 설계

## Executive Summary

2025년 이후 object tracking 연구는 transformer end-to-end(track query 기반)로 이동했지만, 저프레임 검출 + 외부 SLAM 포즈라는 object_memory의 조건에서는 **tracking-by-detection의 track management 방법론**이 직접 이식 가능한 자산이다. 그 방법론의 정점은 40년간 정제된 **존재확률(existence probability) 프레임워크**(IPDA → Bernoulli 필터 → PMBM)이며, 2025~26년 연구는 이를 검출 품질 연동(BYTE식 계층 소비), 가시성 연동(expected P_D)으로 현대화하고 있다.

우리 목표("다양한 테스트 환경에서 객체 상태가 제대로 측정")에 대한 답은 세 문장으로 요약된다. **첫째, confidence를 "객체가 실존할 확률 r"로 재정의하라** — 선형 gain/decay는 해석 불가능하지만 r은 캘리브레이션 검증이 가능하다. **둘째, 미검출의 감점을 기대 검출확률 P_D에 비례시켜라** — 시야 밖 객체는 감점되지 않으므로 occlusion/loop 환경에서 임계 재튜닝이 불필요해진다(우리는 SLAM 포즈로 P_D를 계산할 수 있는 특권적 위치에 있다). **셋째, 이산 상태기계는 r의 임계 파생으로 유지하라** — Unity 이벤트 계약의 인터페이스로 그대로 쓴다.

**Key Technical Findings:**

- 필요한 구현은 Bernoulli 재귀 3식뿐 — 순수 Python, landmark당 O(1), 의존성 추가 없음
- 시야 밖 미검출 무감점은 별도 규칙이 아니라 `r' = r(1−P_D)/(1−r·P_D)`에서 P_D=0일 때 자동 도출
- SAM-6D score(semantic+appearance+geometric 결합)는 존재확률 갱신의 검출 항으로 의미가 정합하나, forced-output 상대점수라 캘리브레이션 후 사용
- 존재확률(있는가)과 포즈 불확실성(어디인가)은 분리 유지가 정석 — 후자는 Mahalanobis 게이트의 재료
- 장기 재인식은 기하 우선 계단식(클래스 → 맵 반경 → appearance)이 2026 체계 연구의 결론
- 모듈 구조는 Stone Soup 5분해(Associator/Updater/Deleter/Initiator)를 따르면 방법론 교체 비용 최소화

**Technical Recommendations (우선순위순):**

1. fix3(pruning)을 Deleter/Initiator 함수 분리 구조로 구현 (임시로 count 임계 유지)
2. confidence 갱신을 Bernoulli 재귀로 교체 + FOV 기반 P_D 1단계(시야 안/밖 이분) 도입
3. 상태 전이·pruning 조건을 r 임계로 재정의 (승격 r>0.6 / lost r<0.3 / 종결 r<0.05 — 초기값, 캘리브레이션으로 조정)
4. TrackEval(HOTA/IDSW) + pseudo-GT 기반 r reliability diagram으로 검증 체계 구축
5. landmark별 위치 분산 유지 → Mahalanobis 게이트로 고정 임계(0.15 m) 대체

## Table of Contents

1. Research Overview (문서 상단)
2. Technical Research Scope Confirmation
3. Technology Stack Analysis — 패러다임 지형, confidence 방법론 4계열(A~D), object SLAM, 6D pose UQ
4. Integration Patterns Analysis — BYTE 계층 소비, expected P_D(핵심), Mahalanobis 게이팅, 계단식 re-ID, 대응표
5. Architectural Patterns and Design — IPDA 하이브리드 상태 표현, Stone Soup 분해, scene graph, ADR 표
6. Implementation Approaches — Bernoulli 재귀 3식, P_D 추정법, SAM-6D score 활용, 검증 체계
7. Technical Research Recommendations — 로드맵(fix2~fix4 통합), 스택 권고, KPI
8. Research Synthesis (본 절) — 경영 요약, 결론, 방법론·출처

## Risk Assessment

| 리스크 | 완화책 |
| --- | --- |
| P_D 오추정(시야 안인데 가림) → r 과소 | MVP는 FOV 이분만; 감점 하한(P_D_base<1)으로 완만한 하락 보장 |
| SAM-6D score 비캘리브레이션 → 검출 항 왜곡 | 초기엔 score를 r 갱신에 미반영(검출=고정 증거)하고 2차로 도입 |
| r 임계 초기값 부적합 | 4-bag pseudo-GT reliability diagram으로 데이터 기반 조정 |
| 재귀식 도입 회귀 | Updater 함수 교체 구조라 count 방식과 A/B 재실행 비교 가능(오프라인 러너 이점) |

## Research Methodology and Source Verification

- **검색 쿼리 11건**: MOT 동향 survey / 3D MOT confidence / object SLAM permanence / track lifecycle / PMBM / track score(SPRT) / 6D pose UQ / BYTE association / occlusion-aware P_D / re-ID memory / Mahalanobis DA / IPDA / Stone Soup / scene graph / Bernoulli 갱신식 / HOTA / SAM-6D score
- **검증 방식**: 핵심 주장(존재확률 정석, P_D 연동, 계단식 re-ID, 하이브리드 상태 표현)은 각 2개 이상 독립 소스로 교차 확인. 불확실 항목엔 신뢰수준(높음/중) 명기 — MaskVal 이식 난이도(중), 정적 객체에서 appearance 불필요(중), 라이브 loop closure 요구(중)
- **한계**: 검색 접근이 초록/요약 수준인 소스가 일부 존재(IPDA 원문 등 유료 논문). 재귀식은 Bernoulli 필터 공개 문헌으로 검증했으나 구현 시 단위 테스트로 성질(P_D=0 → r 불변 등)을 재확인할 것

## Conclusion

object_memory의 confidence는 "임의 스칼라"에서 "**가시성을 아는 존재확률**"로 승격되어야 하며, 이는 3줄의 수식과 이미 보유한 재투영 코드로 구현 가능하다. 이 전환 하나로 사용자가 지적한 세 가지 문제(오염 포즈 미교정, 유령 landmark 잔존, 환경별 상태 오판)의 공통 원인 — 증거의 질과 맥락을 무시하는 confidence — 이 제거된다.

## 다음 액션 제안

1. **(권장) fix3부터 로드맵 순서대로 진행** — pruning을 Deleter/Initiator 분리 구조로 구현한 뒤 Bernoulli r 교체(fix-r)로. 리서치 결과가 코드로 이어지는 최단 경로.
2. **fix-r 선행 도입** — pruning 없이 존재확률부터 교체해 4-bag before/after로 효과를 먼저 검증.
3. **검증 체계 우선 구축** — TrackEval + pseudo-GT reliability를 먼저 만들어 이후 모든 수정의 회귀 기준으로.
4. **리서치 심화** — expected P_D 논문(2511.20239)과 Bernoulli 필터 원문을 정독해 수식 유도까지 확인 후 착수.

질문: 다음 작업으로 (1) 로드맵 순서 진행, (2) 존재확률 선행 도입, (3) 검증 체계 우선 중 무엇을 선택하시겠습니까?

---

**Technical Research Completion Date:** 2026-07-03
**Source Verification:** 모든 기술 주장에 현행 공개 소스 인용
**Technical Confidence Level:** High — 핵심 결론은 복수 독립 소스 교차 검증
