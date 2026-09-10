---
stepsCompleted: ['init', 'literature-survey', 'candidate-table', 'deep-dive', 'final-recommendation']
inputDocuments: []
workflowType: 'research'
lastStep: 6
research_type: 'technical'
research_topic: 'GT-free SLAM localization-reliability metrics (ORB-SLAM3 / HDL-SLAM / RTAB-Map)'
research_goals: 'RPE를 대체할, GT/수동라벨/원본센서 place-recognition 없이 SLAM 출력물만으로 현재 위치 추정 신뢰도를 평가하는 2번 지표 선정'
user_name: 'ldh'
date: '2026-06-26'
web_research_enabled: true
source_verification: true
---

# Research Report: GT 없는 SLAM "현재 위치 추정 신뢰도" 평가 지표

**Date:** 2026-06-26
**Author:** ldh
**Research Type:** technical (SLAM 성능 지표 문헌 조사)
**대상 시스템:** ORB-SLAM3 · HDL graph SLAM · RTAB-Map

---

## Research Overview

세 SLAM 시스템 중 "현재 위치 추정을 가장 신뢰할 수 있는 것"을 고르기 위한 **GT-free 성능 지표**를 조사했다. 핵심 제약은 (1) GT trajectory 불필요, (2) 사람이 동일 지점을 수동 지정하지 않음, (3) 원본 RGB/LiDAR로 별도 place recognition을 돌리지 않음, (4) 가능한 한 SLAM 출력물(trajectory, pose graph, loop log, covariance, residual)만 사용. 6개 후보 범주(pose-graph residual / loop correction / trajectory consistency / map consistency / tracking quality / degeneracy)에 대해 6개 병렬 문헌 조사를 수행하고, 각 지표를 12개 기준(수식·입력·GT여부·3시스템 적용성·위치추정 관련성·기존지표 중복성·장단점·구현난이도·추천여부)으로 평가했다.

**결론 한 줄:** RPE의 대체 2번 지표로는 **"루프 폐쇄 자기일관성(Loop-Closure Self-Consistency) — 최적화 *이전* 루프 잔차/갭"** 을 **강력 추천**한다. 이는 SLAM 자신의 루프만 사용(외부 place recognition 불필요)하면서 *누적 드리프트 = 현재 위치 오차*를 직접 측정하고, "트래킹은 됐지만 위치가 틀린" 상태를 잡아내며, 단순 smoothness가 아니다. 실무 구현이 가장 쉬운 등가 변형은 **odometry(front-end) vs 최적화(back-end) trajectory 불일치량**이다.

---

## 1. Executive Summary

### 1.1 RPE가 현재 목적에 부적합한 이유

프레임 간 RPE(Sturm et al., 2012, TUM RGB-D benchmark)는 고정 window Δ에서 추정 상대운동과 GT 상대운동의 차이를 측정한다:
`RPE_i = (Q_i⁻¹ Q_{i+Δ})⁻¹ (P_i⁻¹ P_{i+Δ})` (P=추정, Q=GT).

목적에 부적합한 이유는 세 가지다.

1. **GT가 필요하다.** 정의상 Q(=GT 상대운동)가 있어야 한다. GT-free 제약과 정면으로 충돌한다. (GT 없이 "프레임 간 일관성"만 보면 그건 RPE가 아니라 아래 trajectory regularity 계열이 된다.)
2. **국소(local) 오차만 본다 → 점진적 드리프트에 사실상 눈을 감는다.** 짧은 Δ의 RPE는 각 스텝이 정확하면 작게 나온다. trajectory가 천천히·부드럽게 전역적으로 틀어져도(=가장 위험한 실패 모드) 단구간 RPE는 거의 영향을 안 받는다. 전역 오차는 Δ를 전체 길이까지 키워야 드러나는데 그건 ATE에 수렴한다.
3. **"부드러움"을 정확도로 오해한다.** RPE가 작다는 것은 국소적으로 매끄럽다는 뜻이지 현재 위치가 맞다는 뜻이 아니다. 과도하게 정규화된(over-smoothed) 추정기가 RPE에서 유리하게 보일 수 있다.

→ 즉 RPE는 "현재 위치 추정 품질"이라는 우리의 질문에 **직접 대응되지 않는다.** local smoothness ≠ 현재 위치 정확도.

### 1.2 GT 없이 대체 가능한 후보 Top 5

| 순위 | 후보 지표 | 핵심 아이디어 | 추천 강도 |
|---|---|---|---|
| 1 | **Loop-Closure 잔차/갭 (최적화 이전)** | SLAM 자신의 루프가 닫히기 직전, 현재 추정 pose와 매칭된 과거 pose의 불일치 = 그 루프 구간 누적 드리프트 | **강력 추천** |
| 2 | **Odom vs 최적화 trajectory 불일치량** | front-end 원시 trajectory와 back-end 최적화 trajectory의 pose별 차이 = 옵티마이저가 "틀렸다고 판단해 옮긴 양" = 드리프트 하한 | **강력 추천** (1의 구현 쉬운 등가물) |
| 3 | **Pose 공분산 D-opt/E-opt (정규화)** | 옵티마이저 정보행렬 H의 역(Σ=H⁻¹) 부피/최악방향 = 추정기 자기보고 불확실성 | 조건부 추천 (보조 축) |
| 4 | **Loop-constraint χ² 일관성 (검증 게이트)** | 루프 잔차의 Mahalanobis χ² 검정 = 잘못된 루프 탐지 | 조건부 추천 (1·2의 검증 동반자) |
| 5 | **MME / MOM (맵 선명도)** | 점군 국소 공분산 엔트로피 = 맵 crispness (드리프트 시 이중벽/ghosting) | 조건부 추천 (이미 사용자의 MME가 커버) |

> 명백히 **비추천**: jerk/속도-스파이크/pose-jump 같은 **trajectory smoothness 계열** — 순수 부드러움 측정이라 "부드럽지만 전역적으로 틀린" 궤적을 과대평가한다(=잡아야 할 실패를 보상). 단, 물리적 비현실 운동(텔레포트) 탐지용 sanity filter로만 보조 사용 가능.

### 1.3 최종 추천 (1~2개)

- **메인(강력 추천): 루프 폐쇄 자기일관성** = **최적화 이전 루프 잔차/갭**.
  실무 구현은 **odom vs 최적화 trajectory 불일치량**(2번)으로 시작하고, 가능하면 루프 이벤트별 갭(1번)으로 정밀화한다.
  반드시 **"발화한 루프 수 + χ² 일관성"** 을 함께 기록해, *루프를 안 닫아서 보정이 0인 SLAM을 "정확하다"고 잘못 보상하는 함정*을 차단한다.
- **선택적 보조(조건부 추천): 정규화 Pose 공분산(D-opt/E-opt)** — "트래킹은 OK인데 추정이 불확실"한 프레임을 프레임 단위로 등급화하는 confidence 축. 세 시스템 모두 정보행렬이 있어 공정 비교 가능(단, 절대값이 아니라 정규화/추세로).

---

## 2. Current Metric Set 분석

### 2.1 기존 3개 지표가 커버하는 평가 영역

| 기존 지표 | 측정 대상 | 커버하는 오차 축 | GT-free? | 한계 |
|---|---|---|---|---|
| **① 시작–끝 오차** | closed-loop에서 시작 pose와 끝 pose의 차이 (Sim(3) drift `T_drift = T_e·T_s⁻¹`, TUM-Mono Engel 2016 형식과 동일) | **전역 누적 드리프트** (전체 한 바퀴) | ✅ (시작=끝 위상 사실만 필요) | ① 한 번 닫힌 그 루프에만 유효, ② 내부(중간) 드리프트 못 봄, ③ **최적화된 최종 trajectory에서 재면** 시스템이 끝을 강제로 시작에 붙였을 때 인위적으로 0이 됨 |
| **③ MME** | 최종 점군 맵의 국소 선명도. 표준 용어는 **Mean Map Entropy** (Razlaw et al. ECMR 2015): `h(q_k)=½·ln|2πe·Σ(q_k)|`, 전체 평균 | **맵 기하 일관성** (드리프트 시 이중벽/blur) | ✅ | 전역 드리프트에 둔감하거나 오히려 *역방향*(점이 흩어지면 이웃 적어 공분산↓→MME가 "좋아짐"). 센서/밀도 의존 → **모달리티 교차비교 불공정** |
| **④ Tracking 성공률** | 정상 tracking/localization 유지 프레임 비율 | **강건성/커버리지** (binary 성공) | ✅ | 성공/실패 이분법만. "성공했지만 품질 낮음"을 구분 못 함. lost/reloc 카운트와 같은 축 |

### 2.2 세 지표가 이미 덮는 영역 vs 비어 있는 영역

세 지표를 오차 공간에 배치하면:

- **전역 드리프트(한 바퀴 끝점)** → ① 이 덮음
- **맵 기하 품질** → ③ 이 덮음
- **강건성(트래킹 유지율)** → ④ 가 덮음

**비어 있는 영역 (2번 대체 지표가 채워야 할 곳):**

1. **시간/구간별 누적 드리프트** — ①은 "끝점 1개 숫자"만 준다. 궤적 *중간*에서 언제·어디서 드리프트가 쌓였는지, 여러 바퀴(`three_laps`, `forward_backward_repeat`) 각 재방문에서 위치가 얼마나 어긋나는지를 ①은 못 본다.
2. **"트래킹은 성공인데 위치가 틀린" 상태** — ④의 이분법이 못 잡는, 성공한 트래킹의 *품질/불확실성* 등급.
3. **최적화 *이전*의 실제 추정 오차** — ①·③은 모두 최적화가 끝난 산출물에서 측정한다. 옵티마이저가 오차를 "재분배해 숨긴" 뒤를 보는 셈. front-end가 실제로 얼마나 틀렸는지(=현재 위치 추정의 raw 품질)는 비어 있다.
4. **잘못된 루프 폐쇄 탐지** — ①~④ 어느 것도 "loop closure가 잘못 적용됐는데 옵티마이저가 매끄럽게 만들어버린" 경우를 직접 신호하지 못한다.

→ 대체 2번 지표는 **(a) 구간/이벤트별 (b) 최적화 이전 (c) 외부 place recognition 없이** 드리프트를 측정하고, (d) 잘못된 루프를 감지할 수 있어야 한다. 이 4조건을 동시에 만족하는 것이 **루프 폐쇄 자기일관성**이다.

---

## 3. Literature Survey

문헌은 GT-free 평가가 크게 두 흐름으로 나뉜다: **(i) trajectory/그래프 자기일관성**(루프·pose-graph 잔차·odom vs map 불일치), **(ii) 맵/추정 introspection**(맵 엔트로피, 공분산, degeneracy). 단순 smoothness(jerk 등)는 SLAM 평가 표준 문헌이 **없으며**(motion-planning·biomechanics 출신), 정확도 지표가 아니라 plausibility filter로만 다뤄진다.

### 3.1 누적 드리프트를 GT-free anchor로 쓰는 흐름 (★ 핵심)

- **Engel, Usenko, Cremers, "A Photometrically Calibrated Benchmark for Monocular VO" (TUM-Mono), arXiv:1607.02555, 2016.** 시작/끝이 같은 지점이라는 사실만으로(=loop) 전체 trajectory GT 없이 누적 드리프트 `e_t/e_r/e_s, e_align`을 정의·랭킹. 시작–끝 오차의 엄밀한 Sim(3) 형식이며, "루프를 GT-free 기준점으로 쓴다"는 본 보고서 핵심 아이디어의 직접적 근거.
- **루프 폐쇄 갭 = 그 구간 누적 드리프트:** ORB-SLAM2/3(Mur-Artal & Tardós 2017, arXiv:1610.06475; Campos et al. 2021, arXiv:2007.11898)는 루프 검증 시 측정 상대변환 `T_meas`(Sim3 `mg2oLoopScw`)를 구하고, 그래프가 가진 odometry 함의 상대변환 `T_odom`과의 차 `‖log(T_meas⁻¹·T_odom)‖`가 곧 그 루프가 흡수해야 할 드리프트다. RTAB-Map(Labbé & Michaud, JFR 2019 / arXiv:2403.06341)·HDL(koide3 `loop_detector` + g2o)도 동일 구조.

### 3.2 Pose-graph 잔차 / χ² 일관성 흐름

- **Kümmerle et al., "g2o", ICRA 2011.** 그래프 비용 `F(x)=Σ e_ijᵀ Ω_ij e_ij` (각 항이 Mahalanobis 제곱). g2o `activeChi2()`, GTSAM `graph.error()`.
- **Latif, Cadena, Neira, "Robust Loop Closing Over Time" (RRR), IJRR 2013** / **Sünderhauf & Protzel, "Switchable Constraints", IROS 2012** / **Lee et al., EM loop-closures, IROS 2013.** 루프 edge 잔차의 χ² 검정·switch 변수로 *잘못된 루프*를 거부. 본 보고서의 "검증 게이트"(Metric 4) 근거.
- **RTAB-Map "Robust Graph Optimization"(wiki) — `RGBD/OptimizeMaxError`(기본 3.0):** 최적화 후 각 link의 `|오차|/σ`가 임계 초과면 루프 거부. **세 시스템 중 가장 즉시 노출되는 GT-free 그래프 잔차 신호.** Vertigo(switchable constraints)로 off된 루프 수도 카운트 가능.
- **주의(중요):** g2o/GTSAM 총 χ²는 *옵티마이저 만족도*이지 정확도가 아니다. 잘못된 루프가 과신(작은 공분산)으로 들어오면 옵티마이저가 전체를 휘어 맞추고 **총 χ²는 낮게** 유지된다(특히 대칭/반복 환경 perceptual aliasing에서 자기-일관된 다수의 잘못된 루프). → "낮은 잔차=정확"이 아니다. **anomaly detector(잔차 *스파이크*)** 로 써야 한다.

### 3.3 Odom vs 최적화 불일치 / 다중주행 정밀도

- **Fontan, Civera, Fischer, Milford, "Look Ma, No Ground Truth!", arXiv:2412.01116, 2024.** noise-augmentation으로 GTF-ATE(반복성) 정의. 동시에 기존 GT-free 신호들의 약점을 명시 — reprojection error("overfit 위험"), χ² 일관성("오차 *크기*가 아니라 불확실성과의 일관성만 잼"), APTE(VO 한정). → 우리가 잔차를 맹신하면 안 되는 직접 근거.
- **Du et al., "Task-driven SLAM Benchmarking", arXiv:2409.16573, 2024.** 반복 주행 Precision(웨이포인트 pose 분산)·Completeness. **사용자의 반복-바퀴 bag에 직접 부합**하나 precision≠accuracy(일관되게 틀려도 precise).
- **Cartographer (Hess et al., ICRA 2016):** 루프 시 trajectory의 *급격한 변화*(pose discontinuity)를 ~7초 보간으로 완화 — correction jump(Metric 3)가 실재함을 보여줌.

### 3.4 맵 일관성 흐름 (사용자 MME와 중복 영역)

- **Razlaw et al., ECMR 2015** — MME/MPV 원전 (`h=½ln|2πe·Σ_r|`, r=0.3 m). AIS-Bonn `pointcloud_evaluation_tool` 구현. **저자 스스로 "전역 일관 trajectory에서만 유효, ATE/RPE와 병행하라"고 명시** → 사용자가 MME를 다른 지표와 묶는 직관을 뒷받침.
- **Kornilova & Ferrer, "Be your own Benchmark" (MOM), arXiv:2106.11351, ECMR 2021.** MME·MPV가 수학적으로 거의 동일(둘 다 ≈λ_min)임을 보이고, 상호직교면 위에서만 재는 **MOM**이 **RPE와 선형 관계(Lemma IV.1)·강한 상관**임을 증명. 즉 *맵 일관성 계열끼리는 상호중복이 크다* → MME 옆에 MPV/또 다른 맵 일관성 지표를 더하는 건 정보 중복.
- **Adolfsson et al., CorAl, ECMR 2021 (arXiv:2109.09820).** joint vs separate 미분 엔트로피 차이로 *정합 오류* introspection.
- **Hu et al., MapEval, arXiv:2411.17928, RA-L/IROS 2025.** MME/MPV(무참조) + AWD/SCS(참조 필요) 통합 프레임워크·구현.

### 3.5 Tracking 품질 / 불확실성 introspection 흐름

- **ORB-SLAM(2015)/ORB-SLAM3(2021):** tracked map-point matches, inlier ratio(EPnP+RANSAC), reprojection χ²(인라이어 게이트 5.991 mono / 7.815 stereo), covisibility. 대부분 **로그로 안 남음 → 패치 필요.**
- **RTAB-Map(JFR 2019):** `info.reg.inliers/matches`, `info.reg.covariance`를 link별로 **이미 DB에 저장**. ICP fitness/inlier_rmse(Open3D 정의)도 사용.
- **HDL(koide3):** `InformationMatrixCalculator`가 scan-match **fitness score → sigmoid weight → edge 정보행렬**로 매핑(즉 HDL의 edge 정보는 fitness의 직접 함수). NDT transformation probability(Magnusson 2009 PhD, PCL `getTransformationProbability()`)는 패치로 노출.
- **핵심 교훈:** reprojection error(px)·ICP fitness(m/overlap)·NDT prob(무단위 likelihood)은 **모달리티 교차로 절대값 비교 불가.** 세 시스템 공정 비교가 되는 건 **정보행렬에서 유도한 pose 공분산**과 **정규화한 inlier/제약 수**뿐.

### 3.6 Degeneracy / Observability 흐름

- **Zhang, Kaess, Singh, "On Degeneracy of Optimization-based State Estimation", ICRA 2016.** 정보행렬 `A=JᵀJ`의 **최소 고유값 λ_min = degeneracy factor**, 임계 이하 방향은 unobservable → Solution Remapping.
- **Tuna et al., X-ICP, T-RO 2023** / **"Informed, Constrained, Aligned" 필드분석, arXiv:2408.11809, 2024.** 회전/병진 블록 분리해 고유값·조건수를 봐야 한다(전체 6×6은 단위 혼합으로 비교 불가)는 핵심 경고.
- **Carrillo et al. (ICRA 2012/2015), Rodríguez-Arévalo et al. (T-RO 2018), Placed et al. survey (T-RO 2023):** 공분산 Σ=H⁻¹의 **D-opt(det)·A-opt(trace)·E-opt(λ_max)** 최적성 기준. D-opt가 단조성 좋음.
- **시각 SLAM low-parallax/pure-rotation:** 병진 블록 고유값 붕괴 → 병진/스케일 unobservable(Lim et al., ICRA 2021). ORB-SLAM3/RTAB-Map 시각 front-end의 전형적 실패.
- **판단:** well-conditioned = *국소 관측가능*일 뿐 *전역 정확*은 아님. 공분산은 보통 **과낙관적**(선형화·association 오차 무시). → red-flag 탐지용(필요조건), 정확도 보증(충분조건) 아님.

---

## 4. Candidate Metric Table

GT?=GT 필요 / ML?=수동 라벨 필요 / PR?=원본센서 place recognition 필요 / O=ORB-SLAM3, H=HDL, R=RTAB-Map 적용성.

| Metric | Paper / Source | GT? | ML? | PR? | Required Outputs | O | H | R | 위치추정 정확도 관련성 | 기존지표 중복 | 구현난이도 | 추천 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **Loop-closure 잔차/갭 (최적화 이전)** | Engel 2016; ORB-SLAM2/3; RTAB-Map JFR | No | No | **No (자기 루프)** | 루프 이벤트 로그: 매칭 KF id, `T_meas`, 최적화 前 pose 2개 | ✔(LoopClosing.cc 패치) | ✔(loop_detector+g2o) | ✔(link DB) | **강** (구간 누적드리프트=현재 오차) | ①의 일반화(내부+이벤트별), MME/④와 낮음 | 중 | **강력 추천** |
| **Odom vs 최적화 trajectory 불일치** | 그래프-SLAM 드리프트 이론; arXiv:2408.01716 | No | No | No | 한 run의 raw front-end traj + 최적화 traj | ✔(pre-loop pose 로깅) | ✔(odom vs map) | ✔(/odom vs /mapData) | **강** (드리프트 하한) | ①과 일부, MME/④와 낮음 | **낮** | **강력 추천** (실무 1순위) |
| **Pose 공분산 D-opt/E-opt (정규화)** | Carrillo 2012/15; Rodríguez-Arévalo T-RO 2018 | No | No | No | 정보행렬/공분산 Σ=H⁻¹ (BA Hessian/edge info) | △(marginalize 패치) | ✔(edge info) | ✔(reg.covariance) | 중 (자기보고 불확실성, 과낙관·국소) | ④와 낮음, degeneracy와 동일근원 | 낮(R/H)~높(O) | 조건부 추천 |
| **Loop-constraint χ² 일관성** | Latif RRR IJRR 2013; Sünderhauf IROS 2012; RTAB `OptimizeMaxError` | No | No | No | 루프 edge 잔차 + 정보행렬 + 최적화 pose | ✔ | ✔ | ✔(내장) | 중 (validity 신호; 잘못된 루프 탐지) | 없음(검증축) | 중(낮 for R) | 조건부 추천 (1·2 동반) |
| **Pose-graph 총 χ² (activeChi2)** | Kümmerle g2o ICRA 2011 | No | No | No | 직렬화 그래프(.g2o / DB) | △(Optimizer 패치) | ✔ | △ | 약 (옵티마이저 만족도, gameable) | ①과 약, anomaly용 | 낮(H)~중 | 조건부 추천 (스파이크 탐지만) |
| **Degeneracy factor / 조건수 (블록분리)** | Zhang ICRA 2016; Tuna 2024 | No | No | No | scan-match/BA Hessian 고유값 | △ | ✔ | △ | 중 (필요조건; red-flag) | 공분산과 동일근원 | 중 | 조건부 추천 |
| **Inlier ratio / 제약 수 (정규화)** | ORB-SLAM 2015; RTAB JFR 2019 | No | No | No | per-frame inlier/match (정규화) | △(패치) | ✔(ICP corr) | ✔(이미 로그) | 중 (제약 풍부도) | ④와 일부 | 낮~중 | 조건부 추천 |
| **MOM (mutually-orthogonal)** | Kornilova arXiv:2106.11351 | No | No | No | 등록된 점군 + pose | △(dense만) | ✔ | ✔ | 중~강 (RPE와 선형상관) | **MME와 高중복(맵축)** | 중 | 조건부 추천 |
| **MME / MPV** | Razlaw ECMR 2015; MapEval 2025 | No | No | No | 최종 점군 .pcd | △(dense만) | ✔ | ✔ | 중 (간접; 국소) | =사용자 ③ 그 자체 | 낮(툴 존재) | (이미 채택=③) |
| **CorAl** | Adolfsson ECMR 2021 | No | No | No | 겹치는 scan 쌍 | △ | ✔ | ✔ | 중 (정합오류 introspection) | MME 보강 | 중 | 조건부 (보조) |
| **다중주행 Precision/Completeness** | Du arXiv:2409.16573 | No | △(웨이포인트 대응) | No | M회 반복 주행 pose | ✔ | ✔ | ✔ | 약 (precision≠accuracy) | Compl≈④ | 낮~중 | 조건부 추천 |
| **GTF-ATE (perturbation)** | Fontan arXiv:2412.01116 | No | No | No | 파이프라인 재실행(k·kΔ회)+noise | ✔ | △ | ✔(visual) | 약 (반복성, bias 못잡음) | 없음 | 중 | 비추천(랭킹용)/튜닝엔 유용 |
| **Reprojection err / ICP fitness / NDT prob (raw)** | ORB-SLAM; Magnusson 2009; Open3D | No | No | No | per-frame residual | ✔(O,reproj) | ✔(H,fitness/NDT) | ✔(R) | 중 (within-system) | ④와 일부 | 중 | 비추천(교차)/within-system 조건부 |
| **Jerk / LDLJ / 속도·각속도 스파이크 / pose-jump rate** | Sensors 2025; motion-planning | No | No(플랫폼한계) | No | 추정 traj + timestamp | ✔ | ✔ | ✔ | **없음 (순수 smoothness)** | ④와 일부(텔레포트) | 낮 | **비추천** (sanity filter만) |

---

## 5. Top Candidate Deep Dive

가장 타당한 3개를 심층 분석한다: **(A) 루프 폐쇄 잔차/갭(최적화 이전)**, **(B) odom vs 최적화 불일치량**, **(C) 정규화 Pose 공분산**.

### 5.A 루프 폐쇄 잔차/갭 (최적화 이전) — 메인 추천

**계산 절차**
1. 루프(또는 재방문) 이벤트 발화 시점 포착: 시스템이 현재 KF와 과거 KF의 매칭을 확정하는 순간.
2. 기하 검증이 산출한 측정 상대변환 `T_meas`(현재↔매칭, ORB는 Sim3) 기록.
3. 그래프가 *그 시점까지* 가진 odometry 함의 상대변환 `T_odom = pose_cur · pose_match⁻¹` (최적화 *이전* pose로) 계산.
4. 갭 = `g = log(T_meas⁻¹ · T_odom) ∈ se(3)` (mono는 sim(3)); 병진 `‖g_t‖`, 회전 `‖g_R‖` 분리 보고.

**수식**
```
g_t = ‖ trans( T_meas⁻¹ · T_odom ) ‖
g_R = ∠ rot( T_meas⁻¹ · T_odom )
```
이 갭이 곧 **해당 루프 구간의 누적 드리프트**다(= 그 순간 현재 위치 추정이 진실에서 벗어난 양의 직접 추정치).

**산출물 예시 (개념)**
```
loop_event, t=83.2s, matched_kf=#41, cur_kf=#612, g_t=0.74 m, g_R=3.1 deg, chi2=9.8 (DOF6, pass)
loop_event, t=151.0s, matched_kf=#41, cur_kf=#1190, g_t=1.93 m, g_R=6.7 deg, chi2=210.4 (FAIL→reject)
```
→ 2회차 재방문에서 갭이 0.74→1.93 m로 커짐 = 드리프트 누적 추세. 2번째는 χ² 폭증 → 잘못된 루프로 거부.

**해석**
- 작은 갭(검증 통과) = 그 구간 위치 추정이 정확.
- 큰 갭 = 누적 드리프트 큼 = "트래킹은 됐지만 위치가 틀림"을 *직접* 포착.
- χ² 폭증/erratic 큰 보정 = 잘못된 루프.
- **반드시 "발화 루프 수"와 함께 본다**: 루프가 0이면 "측정 불가/미검출"로 처리(작은 보정을 "정확"으로 오해 금지).

**시스템별 구현**
- **ORB-SLAM3:** `LoopClosing.cc`의 `mg2oLoopScw`(Sim3)와 `CorrectLoop()`/`OptimizeEssentialGraph()` 직전 KF pose 덤프. (현재 브랜치가 이미 `LoopClosing.cc`/`KeyFrameDatabase.cc`를 수정 중 → 훅 추가 용이.)
- **HDL:** `loop_detector`가 NDT/ICP 상대 pose + **fitness score** 반환; g2o edge 추가 전/후 vertex pose 덤프. fitness가 내장 confidence.
- **RTAB-Map:** 루프 link 추가 시 transform과 *최적화 후 잔차*가 DB(`rtabmap-databaseViewer`)·콜백으로 노출; χ² link 거부(`RGBD/OptimizeMaxError`) 내장.

**실패 케이스**
- 루프가 한 번도 안 닫히면 측정 불가(→ "정확"이 아니라 "미측정"으로). 
- 단조로운/대칭 환경의 perceptual aliasing → 자기-일관된 잘못된 루프 → χ²만으론 못 거를 수 있음(맵 MME blur·후속 RPE 스파이크로 교차검증).
- mono는 스케일 모호 → Sim3로 다뤄야.

### 5.B Odom vs 최적화 trajectory 불일치량 — 실무 1순위(메인의 쉬운 등가물)

**계산 절차**
1. 한 run에서 두 trajectory 추출: front-end 원시(odometry/tracking)와 back-end 최적화.
2. Sim(3)/SE(3) 정렬 후 pose별 차이 `d_i = ‖ T_opt,i ⊖ T_odom,i ‖`.
3. max/mean/RMS와 *공간 프로파일*(미폐쇄 경로 길이에 따라 증가) 보고.

**수식**
```
d_i = ‖ log( T_odom,i⁻¹ · T_opt,i ) ‖ ,   D = {mean, max, RMS}_i d_i
```

**산출물 예시**: ORB는 `CameraTrajectory`(front-end) vs 최종 최적화 KF trajectory; RTAB는 `/odom` vs `/mapData` 최적화 그래프; HDL은 raw scan-match odom vs 최적화 그래프 pose.

**해석**: 큰 보정 = 폐쇄 이전 드리프트가 컸음 = (그리고 루프로 안 감싸인 구간은 보정 후에도 여전히 틀림). 부드럽게 쌓인 드리프트도 잡는다(=smooth-but-wrong 포착). **함정:** 루프가 0이면 odom≡opt → d=0인데 둘 다 임의로 틀릴 수 있음 → 루프 수와 병행 필수.

**시스템별**: RTAB/HDL은 odom vs map 분리가 깨끗(최소 패치). ORB는 pre-loop pose 로깅 필요. **세 시스템 통틀어 구현난이도 가장 낮음 → 1차 도입 권장.**

**실패 케이스**: 루프 미발화 시 0(=미측정). 보정의 "크기"만으론 올바른 큰 보정 vs 잘못된 큰 보정 구분 불가 → χ²(5.A/Metric4)로 보강.

### 5.C 정규화 Pose 공분산 (D-opt / E-opt) — 보조 confidence 축

**계산 절차**
1. 옵티마이저 정보행렬 `H = JᵀΩJ`(BA/scan-match/edge)에서 pose 주변 공분산 `Σ = H⁻¹`.
2. 스칼라화: `D-opt = ½ ln det(Σ)`(불확실성 부피), `E-opt = λ_max(Σ) = 1/λ_min(H)`(최악 방향).
3. **회전/병진 블록 분리** 후 시스템별 baseline(run median 등)으로 **정규화**.

**수식**
```
Σ = H⁻¹ ;  D-opt = ½·ln det(Σ) ;  E-opt = λ_max(Σ)
(회전·병진 단위 혼합 회피 위해 Σ_rr, Σ_tt 분리)
```

**해석**: 큰 det/λ_max + 트래킹 OK = "성공했지만 불확실"(=목표 빈영역 2). 세 시스템 모두 정보행렬 보유 → *정규화 후* 공정 비교 가능. **주의:** 공분산은 과낙관적이며 *국소 관측가능*만 보증(전역 드리프트엔 둔감) → red-flag 보조로만.

**시스템별**: RTAB `info.reg.covariance`(이미 노출), HDL edge 정보행렬(fitness 함수), ORB는 `computeMarginals` 패치 필요(난이도 높음).

**실패 케이스**: 전역으로 부드럽게 틀려도 프레임별로는 well-conditioned → 정확으로 오인 가능(그래서 5.A/5.B와 묶어야). 절대값 교차비교 금지.

---

## 6. Final Recommendation

### 6.1 최종 4개 지표 구성

| # | 지표 | 측정 축 | 역할 |
|---|---|---|---|
| **1. 시작–끝 오차** | 전체 1바퀴 전역 누적 드리프트 (최종 trajectory) | 전역 정확도 종점 검증 |
| **2. 루프 폐쇄 자기일관성 (★ 신규 대체)** = 최적화 *이전* 루프 잔차/갭 + (실무) odom vs 최적화 불일치량, **루프수·χ²로 가드** | 구간/이벤트별·최적화 이전 누적 드리프트 + 잘못된 루프 탐지 | **현재 위치 추정 raw 품질** |
| **3. MME** | 맵 기하 선명도 | 맵 일관성 |
| **4. Tracking 성공률** | 트래킹 유지 비율 | 강건성/커버리지 |
| *(선택)* 정규화 Pose 공분산 | 프레임별 자기보고 불확실성 | "성공했지만 불확실" 등급 |

### 6.2 이 조합이 "현재 위치 추정 최고 SLAM 선택"에 적합한 이유

네 지표가 **상호 직교하는 4개 오차 축**을 덮는다 — 전역 종점(①), 구간별 raw 드리프트(②), 맵 기하(③), 강건성(④). 특히 신규 ②가 §2.2의 **빈 4영역**(구간별·최적화 이전·외부 PR 없음·잘못된 루프 탐지)을 정확히 채우면서, RPE가 못 하던 "현재 위치 정확도"에 직접 대응한다. 사용자의 **반복-바퀴 bag**(`three_laps`, `forward_backward_repeat`, `one_lap_back_and_forth`)은 루프가 여러 번 발화하므로 ②가 가장 잘 작동하는 데이터 조건이다.

### 6.3 [중요 판단 기준] 신규 2번 지표 검증

| 질문 | 답 |
|---|---|
| **낮으면 정말 현재 위치가 더 좋다고 말할 수 있나?** | **예(조건부).** 갭/불일치가 작고 *루프가 실제로 발화·χ² 통과*했다면, 그 구간 누적 드리프트가 작다는 직접 증거. 단 "루프 수>0" 가드 필수(아니면 "미측정"). |
| **부드럽기만 한 SLAM을 과대평가하지 않나?** | **아니다.** smoothness 지표가 아님. 부드럽게 누적된 드리프트도 루프가 닫힐 때 갭/보정으로 *그대로 드러난다*. (반대로 jerk류는 과대평가 → 비추천.) |
| **loop closure가 잘못 적용돼도 감지되나?** | **예.** 잘못된 루프는 큰·erratic 갭 + χ² 폭증(RRR/switchable/`OptimizeMaxError`)으로 잡힌다. 추가로 MME blur·후속 RPE 스파이크로 교차검증. |
| **트래킹 성공인데 위치 틀어진 상태를 감지하나?** | **예.** 이것이 핵심 강점. 큰 pre-opt 갭/불일치 = 성공한 트래킹의 누적 오차를 직접 노출. (보조로 공분산도.) |
| **ORB3/HDL/RTAB에 공정한가?** | **예.** 셋 다 그래프·루프·odom/map을 노출. ②는 모달리티 무관(거리 단위 드리프트)이라 reprojection/fitness처럼 단위 불일치 문제 없음. ORB만 LoopClosing 로깅 패치 필요(이미 그 파일 수정 중). |
| **MME와 같은 정보를 반복 측정하나?** | **아니다.** MME=맵 기하 선명도(공간), ②=trajectory 드리프트(시간/경로). 직교. (맵 일관성 2개를 쓰는 게 중복 — 그래서 MPV/추가 맵지표는 비추천.) |

### 6.4 추천 강도 요약

- **강력 추천:** 루프 폐쇄 자기일관성(최적화 이전 잔차/갭) ·그 실무 등가물 odom-vs-최적화 불일치량.
- **조건부 추천:** 정규화 Pose 공분산(D/E-opt), Loop-constraint χ² 게이트, pose-graph 총 χ²(스파이크 탐지만), degeneracy/조건수, 정규화 inlier 수, MOM, 다중주행 precision, within-system reprojection/fitness/NDT.
- **비추천:** jerk/LDLJ/속도·각속도 스파이크/pose-jump rate(순수 smoothness — sanity filter만), 교차시스템 raw reprojection/ICP fitness/NDT(단위 비교 불가), GTF-ATE(랭킹용; bias 못 잡음).

---

## 7. 구현 체크리스트 — 저장해야 할 로그/파일

**공통 (모든 시스템)**
- [ ] 최종 trajectory (TUM/KITTI 포맷) — front-end 원시 **및** back-end 최적화 *둘 다* (②에 필수)
- [ ] 타임스탬프 포함 per-frame/keyframe pose
- [ ] 루프 이벤트 로그: `{시각, cur_kf, matched_kf, T_meas, 최적화 前 pose_cur·pose_match, 최적화 後 pose, edge 정보행렬/χ²}`
- [ ] **발화한 루프 수** (가드용 — 0이면 ②는 "미측정")
- [ ] 최종 점군 맵 `.pcd` (③ MME / 선택 MOM용)
- [ ] 트래킹 상태 per-frame (OK/LOST/RELOC) — ④ 및 lost-segment 카운트
- [ ] (선택) per-frame pose 공분산 Σ 또는 정보행렬 H (회전/병진 블록 분리)

**ORB-SLAM3 (패치 필요)**
- [ ] `LoopClosing.cc`: `mg2oLoopScw`, `CorrectLoop()`/`OptimizeEssentialGraph()` 직전/직후 KF pose 덤프 → ② 갭·보정
- [ ] `Optimizer::PoseOptimization` 인라이어 수·reprojection χ² RMS (within-system 보조)
- [ ] (선택) `computeMarginals`로 pose 공분산
- [ ] Atlas sub-map 개수 (track break 지표)

**HDL graph SLAM**
- [ ] `loop_detector` 상대 pose + **fitness score**, g2o vertex pose(optimize 前/후), edge `chi2()`
- [ ] `InformationMatrixCalculator` edge 정보행렬 (fitness→weight)
- [ ] (선택) NDT `getTransformationProbability()` (패치)

**RTAB-Map (대부분 DB에 이미 존재)**
- [ ] `.db` 보존 → `rtabmap-databaseViewer`/`rtabmap-report`로 link transform·잔차·`info.reg.{inliers,matches,covariance}` 추출
- [ ] `RGBD/OptimizeMaxError`(기본 3.0) 루프 accept/reject·error ratio 로그
- [ ] `/odom` vs 최적화 `/mapData` (②)
- [ ] (선택) `Optimizer/Robust=true`(Vertigo) switched-off 루프 수

**분석 스크립트**
- [ ] 두 trajectory Sim(3)/SE(3) 정렬 → pose별 `d_i` (②)
- [ ] 루프 갭 `g_t/g_R` 시계열 + χ² 게이트
- [ ] MME: AIS-Bonn `pointcloud_evaluation_tool` 또는 `pip install map-metrics`/MapEval (r=0.3, dense cloud만; ORB sparse map points 금지)
- [ ] 모달리티 교차비교는 **정규화/추세**로만 (절대값 금지)

---

## 다음 액션 제안

조사는 완료됐습니다. 진행 방향을 선택해 주세요.

1. **② 지표 PoC 구현 (실무 우선)** — odom-vs-최적화 불일치량부터 세 시스템에 붙이고, 기존 4-bag 결과에 적용해 실제 수치를 뽑아볼까요? (가장 빠른 검증 경로)
2. **ORB-SLAM3 LoopClosing 로깅 패치** — 이미 수정 중인 `LoopClosing.cc`에 루프 갭/χ² 덤프 훅을 추가하는 구체 코드까지 설계할까요?
3. **지표 정규화/스코어링 설계** — 세 시스템을 공정 비교하기 위한 0~1 health score·정규화 공식을 정리할까요?
4. **보고서 보강** — 특정 절(예: 5장 deep dive, 수식 유도, 추가 논문)을 더 깊게 확장할까요?

**질문:** 지금 가장 필요한 것은 (a) 바로 돌릴 수 있는 구현인가요, 아니면 (b) 이 보고서 자체의 완성도/근거 보강인가요? 그리고 ②를 처음 적용할 대상은 어떤 bag(예: `three_laps`)으로 할까요?

---

*본 보고서는 6개 영역 병렬 문헌 조사(웹 검색·원문 확인) 결과를 종합했습니다. 일부 수식은 2차 출처에서 재구성했으며 해당 위치에 명시했습니다. jerk/속도 계열은 SLAM 정확도 평가 표준 문헌이 없어 plausibility filter로만 분류했습니다.*
