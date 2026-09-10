---
stepsCompleted: [1]
inputDocuments: []
workflowType: 'research'
lastStep: 2
research_type: 'technical'
research_topic: 'SLAM 평가용 데이터 수집 및 Ground Truth 확보 방법 (RealSense D455f + Velodyne VLP-16)'
research_goals: 'ORB-SLAM3 / RTAB-Map / HDL-Graph-SLAM 선택 근거가 되는 5대 성능지표(ATE, RPE, tracking robustness, 연산비용, 맵·내비 정성)를 D455f·VLP-16 센서로 측정하기 위한 데이터 수집 절차와 GT(Ground Truth) trajectory 확보 방법 도출'
user_name: 'ldh'
date: '2026-06-22'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-06-22
**Author:** ldh
**Research Type:** technical

---

## Research Overview

본 연구는 ORB-SLAM3 / RTAB-Map / HDL-Graph-SLAM 중 SLAM 알고리즘 선택의 정량적 근거가 되는 5대 성능지표(ATE, RPE, tracking robustness, 연산비용, 맵·내비 정성)를, 보유 센서(RealSense D455f, Velodyne VLP-16)와 **외부 GT 기준 장비가 전무한 실내 환경**에서 측정하기 위한 데이터 수집 절차 및 Ground Truth 확보 방법론을 도출한다. 모든 기술적 주장은 현재 공개 출처(논문/공식 문서)로 검증한다.

---

## Technical Research Scope Confirmation

**Research Topic:** SLAM 평가용 데이터 수집 및 Ground Truth 확보 방법 (RealSense D455f + Velodyne VLP-16)
**Research Goals:** ORB-SLAM3 / RTAB-Map / HDL-Graph-SLAM 선택 근거가 되는 5대 성능지표를 보유 센서로 측정하기 위한 데이터 수집 절차와 GT trajectory 확보 방법 도출

**확정된 제약 조건 (사용자 입력):**

- 환경: **실내** (→ GNSS/RTK-GPS GT 배제)
- 보유 센서: **RealSense D455f (RGB-D + IMU), Velodyne VLP-16** 만
- 외부 GT 기준 장비: **모션캡처(OptiTrack/Vicon) 없음 / Total Station 없음 / 고정밀 추가 LiDAR 없음**
- 수집 단계: **2단계** — ① Husky 무탑재(handheld) → ② Husky 탑재

**측정 대상 5대 지표:**

| # | 지표 | GT 필요성 |
|---|---|---|
| 1 | ATE RMSE (글로벌 정확도) | GT trajectory 필요 |
| 2 | RPE (드리프트, 객체 re-ID 핵심) | GT 또는 상대 일관성 |
| 3 | Tracking 성공률/loss | 로그 기반 (GT 불필요) |
| 4 | 연산비용 (ms/frame, CPU/RAM) | 프로파일링 (GT 불필요) |
| 5 | 맵 형태·내비 통합 | 정성 평가 |

**장비 없는 GT 확보 전략 (연구 핵심):**

1. Loop-closure / return-to-origin drift (GT-free 1순위 절대지표)
2. Fiducial marker(AprilTag/ArUco) 제어점 (저비용 sparse 절대 GT)
3. 오프라인 batch SLAM / COLMAP pseudo-GT (상대비교용)
4. Multi-run repeatability (재현성 지표)

**Research Methodology:** 현재 웹 데이터 + 엄격한 출처 검증, 다중 출처 교차검증, 불확실 정보 신뢰도 등급 표기.

**Scope Confirmed:** 2026-06-22

---

## Technology Stack Analysis — 데이터 수집·평가 도구체인

> 본 절은 일반 SW 스택(언어/DB/클라우드) 대신, 연구 주제에 맞춰 **SLAM 평가 파이프라인의 도구 스택**(데이터 수집 → 캘리브레이션 → GT 생성 → 정량 평가)을 분석한다. 모든 항목은 현재 공개 출처로 검증했으며, 신뢰도 등급(높음/중간/낮음)을 표기한다.

### 1. 데이터 수집 스택 (Data Acquisition)

- **미들웨어/기록:** ROS 2 + `ros2 bag record` (rosbag2). 두 센서 노드를 동시 구동 후 `-a`(전체 토픽) 또는 토픽 지정 기록이 표준 절차. *(신뢰도: 높음)*
- **드라이버:** RealSense → `realsense-ros` (ROS2 wrapper, D455 지원, depth/IR/RGB/IMU 토픽), Velodyne VLP-16 → ROS2 `velodyne_driver`(VLP-16 공식 지원). *(신뢰도: 높음)*
- **시간 동기화(핵심 난제):** LiDAR-카메라는 주파수·캡처시점이 달라 항상 시간 오프셋 존재. 소프트웨어 동기화는 `message_filters`(ApproximateTime)로 시간상 가까운 메시지쌍 정렬; 하드웨어 정밀 동기화는 **PTP**(IEEE-1588, 하드웨어 지원 필요). D455는 Inter-Cam Sync(Master/Slave)가 있으나 이는 카메라간 동기화로 LiDAR↔카메라에는 직접 적용 불가 → 실무적으로 **PTP 또는 PPS/타임스탬프 정렬 + 평가 시 EVO의 timestamp interpolation**으로 보정. *(신뢰도: 높음)*
  - _Source: [realsense-ros ROS2 wrapper](https://dev.realsenseai.com/docs/ros2-wrapper/), [velodyne_driver](https://index.ros.org/p/velodyne_driver/), [VLP-16+camera sync (ROS Answers)](https://answers.ros.org/question/332342/how-to-capture-same-time-velodyne-vlp-16-lidar-and-camera-image/)_

### 2. 캘리브레이션 도구체인 (Calibration)

- **카메라 intrinsic + IMU-camera:** **Kalibr** — multi-camera/multi-IMU/IMU-camera를 연속시간 batch(B-spline) 추정으로 캘리브, 시간 오프셋도 최적화 변수로 직접 추정. D455 RGB↔내장 IMU 캘리브에 적합. *(신뢰도: 높음)*
- **권장 순서:** intrinsic 먼저 → 이후 센서쌍별 extrinsic(camera-IMU → LiDAR-IMU → camera-LiDAR). LiDAR-camera는 타깃 기반(체커보드)·타깃리스(iKalibr 등) 모두 존재. *(신뢰도: 높음)*
  - _Source: [Kalibr / spatiotemporal calibration review (MDPI Sensors 2025)](https://www.mdpi.com/1424-8220/25/17/5409), [iKalibr (arXiv 2407.11420)](https://arxiv.org/pdf/2407.11420)_

### 3. 정량 평가 도구체인 (Evaluation)

- **표준 도구: EVO** (`evo_ape`, `evo_rpe`, `evo_traj`) — ATE(=APE)·RPE 계산, TUM/KITTI/EuRoC/ROS·ROS2 bag 포맷 지원, trajectory association·alignment(Umeyama Sim3/SE3)·시각화 제공. 추정-GT 타임스탬프는 선형보간으로 동기화 후 RMSE 산출. *(신뢰도: 높음)*
- **대안:** `rpg_trajectory_evaluation`(취리히대 RPG), SLAM Hive(클라우드 벤치마킹 스위트). *(신뢰도: 중간)*
- **지표 정의 합의:** ATE=정렬 후 전역 일관성(맵 정확도 평가에 적합), RPE=고정 구간 상대운동 오차=국소 정확도·드리프트(loop closure 정확도와 직결). → **본 과제의 객체 re-ID는 RPE가 1차 지표**라는 이전 결론과 문헌 정의가 일치. *(신뢰도: 높음)*
  - _Source: [evo 공식 문서](https://michaelgrupp.github.io/evo/), [Benchmarking SLAM: Metrics](https://aicompetence.org/benchmarking-slam/), [SLAM Hive (arXiv 2406.17586)](https://arxiv.org/html/2406.17586v1)_

### 4. Ground Truth 확보 방법 (★ 연구 핵심 — 장비 없는 실내) {#gt-methods}

문헌상 실내 GT는 MOCAP(sub-mm)·RTK(실외)가 골드 스탠다드이나, 본 과제는 **둘 다 불가**. 장비 없이 가능한 검증된 대안을 정확도 순으로 정리:

| 방법 | 원리 | 보유 자산으로 가능? | 정확도(문헌) | 신뢰도 |
|---|---|---|---|---|
| **A. LiDAR-Inertial SLAM pseudo-GT (Fast-LIO2)** | VLP-16+IMU로 고정밀 LIO 궤적 생성 → 시각 SLAM의 기준 | ✅ **VLP-16 보유로 즉시 가능** | 소규모 실내 **~3cm** (MOCAP 대비 검증) | 높음 |
| **B. Fiducial marker(AprilTag) GT** | 위치 실측한 태그를 prior로 factor-graph(TagSLAM/Poly-TagSLAM, GTSAM) 최적화 → vision GT | ✅ 프린터+줄자 | mm~cm급(거리·각도 의존) | 높음 |
| **C. Loop-closure / return-to-origin drift** | 폐궤적 후 시작점 복귀 오차 = 누적 드리프트(GT-free 절대지표) | ✅ 즉시 | 상대지표(절대 RMSE 아님) | 높음 |
| **D. AMCL on known map** | 사전 제작 2D 맵에 AMCL 입자필터 정합으로 2D pose GT | ✅ (Husky 탑재 단계) | 중간(맵 품질 의존) | 중간 |
| **E. COLMAP SfM pseudo-GT** | RGB 프레임 batch SfM 결과를 GT로(스케일 보정 필요) | ✅ RGB만 | 중간(복잡·동적 장면 취약, scale-free) | 중간 |
| **F. Multi-run repeatability** | 동일 궤적 반복 간 일관성(정밀도 평가) | ✅ 즉시 | 정밀도만(정확도 아님) | 높음 |
| **G. Wheel odometry 보조 GT** | Husky 휠 오도메트리(준-planar 보조 기준) | ✅ (탑재 단계) | 낮음(슬립·드리프트) | 낮음 |

**핵심 권고(미리보기):** 본 과제는 **A(Fast-LIO2 pseudo-GT)를 주 GT로, B(AprilTag) + C(loop drift)를 교차검증**하는 혼합 전략이 최적. 단, A로 시각 SLAM을 평가할 때 *LiDAR 계열(HDL)을 같은 LiDAR 기반 GT로 평가하면 상관(편향)*이 생기므로 공정성 주의 — Step 3에서 상세화.

- _Source: [Challenges of Indoor SLAM (arXiv 2306.08522)](https://arxiv.org/pdf/2306.08522), [TagSLAM (arXiv 1910.00679)](https://arxiv.org/pdf/1910.00679), [Look Ma No Ground Truth (arXiv 2412.01116)](https://arxiv.org/pdf/2412.01116), [Towards Robust Sensor-Fusion Ground SLAM (arXiv 2507.08364)](https://arxiv.org/pdf/2507.08364)_

### 5. 궤적 포맷 & 통합 패턴

- **포맷:** TUM(`timestamp tx ty tz qx qy qz qw`)이 RGB-D/실내에 사실상 표준, KITTI(3×4 행렬)는 시퀀스 인덱스 기반. SLAM 출력→TUM 변환 후 EVO 투입이 일반적. *(신뢰도: 높음)*
- **통합 흐름:** rosbag2(원천) → 각 SLAM 노드 재생/실행 → 추정 궤적(TUM) + GT 궤적(TUM) → EVO 정렬·평가 → 표/플롯. 5대 지표 중 1·2(ATE/RPE)는 이 경로, 3(tracking loss)은 SLAM 로그, 4(연산비용)는 별도 프로파일링. *(신뢰도: 높음)*

### 6. 연산비용 측정 도구 (지표 4)

- **참고 수치(환경 의존):** ORB-SLAM3 ≈ 13.6 ms/frame(13th Gen i7), tracking-only ~22–23 FPS; loop closure 시 CPU ~325%, 메모리 ~1.48 GB 보고 사례. 키프레임 증가에 따라 local BA 비용 증가. *(신뢰도: 중간 — 하드웨어·시퀀스 의존)*
- **측정 방법:** ROS2 환경에서 `top`/`htop`/`psutil` 또는 ROS 노드별 `ros2 run` 래퍼로 CPU%·RSS 샘플링, 프레임 처리시간은 SLAM 내부 타이머 로그. 동일 하드웨어·동일 bag으로 3종 동시 비교해야 공정. *(신뢰도: 높음)*
  - _Source: [Benchmark ORB-SLAM2 vs RTAB-Map](https://www.researchgate.net/publication/335345350_Benchmark_of_Visual_SLAM_Algorithms_ORB-SLAM2_vs_RTAB-Map), [VI-SLAM loop closing cost (arXiv 2408.01716)](https://arxiv.org/pdf/2408.01716)_


