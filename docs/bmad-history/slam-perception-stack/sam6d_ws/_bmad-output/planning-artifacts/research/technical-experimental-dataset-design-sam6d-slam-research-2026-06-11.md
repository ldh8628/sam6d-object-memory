---
stepsCompleted: [1]
workflowType: 'research'
research_type: 'technical'
research_topic: 'Experimental Dataset Design for Future SAM-6D + SLAM (Husky) System'
research_goals: '논문 Experiments 섹션을 구성할 수 있도록, SAM-6D+Object-aware SLAM 실시간 Husky 시스템 평가용 데이터셋 taxonomy·객체·환경·trajectory·GT·수집계획을 벤치마크 관행에 근거해 설계'
user_name: 'Ldh9501'
date: '2026-06-11'
source_verification: true
---

# Technical Research — Experimental Dataset Design for Future SAM-6D + SLAM System

> **관점**: milk 문제 해결이 아니라 **향후 논문 Experiments 설계**. 벤치마크 관행(BOP/T-LESS/YCB-V/YCBInEOAT/BCOT/TUM/KITTI/Bonn/OpenLORIS)에 근거.
> **현 자산 실측(읽기전용)**: bag 4종(전부 D455f RGB-D + tf_static), CAD 1종(Milk), 실물 7종, Husky+D455f+LiDAR 가용(예정).

---

## Executive Summary — 결론 5개

1. **현재 데이터로는 "단일객체 검출/거부/단프레임 pose"까지만 가능하고, 논문의 핵심 주장(multi-object·pose tracking·object-aware SLAM·loop closure)은 전부 불가.** 보유 bag(only_Milk·milk_0609·SLAM_with_milk_nomilk·SLAM_0609)은 **전부 RGB-D + tf_static만** — LiDAR·wheel odom·IMU·motion-GT·dynamic 토픽이 없고, **CAD가 Milk 1개뿐**이라 6D pose 정량평가(ADD-S)는 milk 외 불가.
2. **최우선 블로커는 데이터가 아니라 (a) 7개 객체 CAD 모델과 (b) per-frame 6D pose GT 파이프라인이다.** 벤치마크 GT의 표준 저비용 레시피 = **CAD + AprilTag 보드 앵커 + RealSense depth ICP 전파**(mocap 랩 불필요). 이 둘이 없으면 어떤 bag을 더 찍어도 논문 수치를 못 만든다.
3. **"많이"가 아니라 "축을 따라 다양하게".** 권위 벤치마크의 실제 test 규모는 작다(LM ~1k img/객체, YCB-V BOP test ~75장, HOPE 50 scene). **credible 최소 데이터셋 ≈ 10~25 시퀀스 × 수백~1000 프레임 + 정확한 GT.** 핵심은 BOP/T-LESS 축(저텍스처·대칭·가림·클러터·조명)을 객체와 시퀀스에 의도적으로 심는 것.
4. **데이터셋을 5레벨 taxonomy로 설계**(L1 단일객체 → L2 2~3 → L3 dense 5~10 → L4 +Navigation(SLAM) → L5 long/loop-closure), 각 레벨이 특정 논문 주장과 metric(ADD-S/AUC·BOP-AR / ATE·RPE / object-localization error)에 1:1 대응되게 한다.
5. **추천 다음 단계 = Quick Dev(GT 파이프라인 + 수집 프로토콜 PoC)**, PRD 아님. 3개월 계획: Phase1(7객체 CAD 스캔 + GT 파이프라인 + L1/L2 office), Phase2(L3 dense + L4 Husky navigation + dynamic), Phase3(L5 long/loop + 최종 벤치마크 재현).

---

## Current Dataset Gap Analysis (Task 1)

**보유 bag 실측(metadata.yaml)** — 전부 RealSense D455f, 핸드헬드 추정, color≈30fps:

| dataset | color msgs | 추정 길이 | 센서 토픽 | LiDAR/odom/IMU | motion GT |
|---|---|---|---|---|---|
| only_Milk | 867 | ~29s | color+depth+camera_info+tf_static | ✗ | ✗ |
| milk_0609 | 1964 | ~65s | 동일 | ✗ | ✗ |
| SLAM_with_milk_nomilk(milk_nomilk_bag) | ~2467 | ~82s | 동일 | ✗ | ✗ |
| SLAM_0609 | (db3 미추출, tar.gz만) | ? | 동일 추정 | ✗ | ✗ |

**가능한 실험(현재)**:
- ✅ 단일객체(milk) **2D detection / presence(no-object rejection)** — visibility 0/1 라벨로 P/R/F1 (이미 수행).
- ✅ milk **단프레임 6D pose** 정성/제한적 정량 — CAD 1종 보유.
- ✅ RGB-D-only **camera-motion 추정** 정성(궤적 GT 없어 ATE 불가).

**불가능한 실험(현재)**:
- ✗ **6D pose 정량(ADD-S/AUC)** milk 외 6개 객체 — **CAD 없음**.
- ✗ **Multi-object detection/pose** — 모든 bag이 단일(milk) 위주, dense clutter 없음.
- ✗ **Pose tracking 정량** — per-frame 6D pose GT 없음(YCBInEOAT/BCOT식 GT 부재).
- ✗ **Object-aware SLAM / mapping consistency / loop closure** — **LiDAR·odom·IMU·궤적 GT 없음**, 재방문/figure-8 시퀀스 없음 → ATE/RPE 산출 불가.
- ✗ **Dynamic robustness** — 사람/이동물체 시퀀스·mocap GT 없음(Bonn/TUM-dynamic식 불가).
- ✗ **Real-time on Husky** — 로봇 플랫폼 주행 데이터 자체가 없음.

> **결론**: 현재 자산은 "milk 인식 파일럿"에는 충분하나 **논문 Experiments의 행(行) 대부분을 채울 수 없다**. 갭의 본질은 ①객체 CAD ②6D/궤적 GT ③멀티객체·주행·동적·루프 시퀀스.

---

## Future Research Questions (Task 2) — 논문이 주장할 것

각 질문 = Experiments의 한 섹션 = 특정 데이터 요구를 정의.

| # | Research Question (주장) | 평가 metric | 필요 데이터 레벨 |
|---|---|---|---|
| RQ1 | SAM-6D가 **다양한 객체(저텍스처·대칭 포함)**에서 정확한 6D pose를 낸다 | ADD / ADD-S / AUC, BOP-AR(VSD/MSSD/MSPD) | L1, L2 |
| RQ2 | 시스템이 **no-object(부재)를 신뢰성 있게 거부**한다(FP 억제) | Precision/Recall/F1, FP rate | L1(+absent), L4 |
| RQ3 | **Multi-object(최대 10개)** 동시 검출·식별·pose가 가능하다 | per-class AR, ID 정확도, 가림 견고성 | L3 |
| RQ4 | **Pose tracking**이 단프레임 대비 시간적 안정성/drift를 개선한다 | per-frame ADD-S, 시간 jitter, track 유지율 | L4(연속) |
| RQ5 | **Object-aware SLAM**이 객체를 landmark로 써 **궤적 drift를 줄인다** | ATE/RPE (object on/off 비교), object-localization error | L4, L5 |
| RQ6 | **Mapping consistency / loop closure**가 재방문에서 일관된 object map을 유지한다 | loop-closure 시 ATE 개선, map 중복 객체율 | L5 |
| RQ7 | **Dynamic object 견고성**(사람/이동물체)에서 SLAM·pose가 무너지지 않는다 | dynamic vs static ATE 차이, pose 유지율 | L4-dynamic |
| RQ8 | **실시간 성능**(Husky 온보드)에서 latency/FPS가 운용 가능하다 | ms/frame, FPS, GPU/CPU 점유 | 전 레벨 |

> **핵심 메시지(논문 한 줄)**: *"실시간 mobile manipulator 위에서 SAM-6D 기반 multi-object 6D pose를 object-aware SLAM과 결합해, 부재 거부·동적 견고성·loop-closure 일관성을 동시에 달성한다."* → 이 한 줄을 8개 RQ가 떠받치고, 8개 RQ가 아래 taxonomy를 강제한다.

---

## Dataset Taxonomy (Task 3)

각 레벨에 **최소 bag 수·길이·환경**을 정의. (규모 근거: BOP/YCB-V/Bonn은 test가 수십 시퀀스·수백~1k 프레임 규모 — 다양성>볼륨.)

| Level | 정의 | 검증 RQ | 최소 bag | 길이/개 | 환경 | GT 필수 |
|---|---|---|---|---|---|---|
| **L1 Single Object** | 객체 1개 + 부재 구간 | RQ1, RQ2 | 객체당 2 (present/absent) → 7객체 ≈ 14 | 30~60s | Office/Lab 정적 | 6D pose + visibility |
| **L2 Multi (2~3)** | 2~3 객체 동시, 약한 가림 | RQ1, RQ3 | 4~6 | 60~90s | Office/Lab | 객체별 6D + ID |
| **L3 Dense (5~10)** | 5~10 객체, 강한 클러터·가림 | RQ3 | 3~5 | 60~120s | Lab/Warehouse | 객체별 6D + ID + 가림도 |
| **L4 Object+Navigation** | Husky 주행 + 객체, 연속 추적 | RQ4, RQ5, RQ7 | 6~10 (정적/동적 혼합) | 1~3 min | Office→Hallway→Lab | 카메라/로봇 궤적 + 객체 6D(+tracking) |
| **L5 Long / Loop** | 수십 m, 재방문·figure-8 | RQ5, RQ6 | 3~5 | 3~8 min | Hallway+Lab 회랑 | 궤적 GT + loop event + object map |

**합계(최소)**: 약 **30~40 시퀀스**(상한). MVD(Task 8)는 이 중 핵심만 추림.

---

## Object Selection Strategy (Task 4)

**원칙(BOP/T-LESS)**: pose를 어렵게 만드는 축을 고르게 덮는다 — 텍스처(고/저), 대칭성, 형상(박스/원통/비정형), 크기(소/대), 반사/투명. **저텍스처+대칭+반사가 SAM-6D(외관기반)의 약점**이라 반드시 포함해야 논문이 "어려운 케이스"를 주장 가능(현 milk = 저텍스처 흰 박스 = 이미 좋은 hard case 1종).

**현재 7개 실물 → 권장 매핑(축 커버 목표)**:

| 슬롯 | 원하는 속성 | 예시(7개에서 선택) | 검증 포인트 |
|---|---|---|---|
| 1 | 저텍스처·박스·흰색 | **Milk(보유 CAD)** | baseline, 외관 모호성 |
| 2 | 고텍스처·박스 | 시리얼/과자 박스 | 텍스처 이점 대조 |
| 3 | 원통·라벨 | 음료캔/병 | 회전 대칭(축대칭) → ADD-S 필요 |
| 4 | 대칭·저텍스처 | 컵/원통 무지 | 대칭 모호성 stress |
| 5 | 소형 | 작은 상자/튜브 | 원거리·해상도 한계 |
| 6 | 대형·비정형 | 박스 큰 것/가전 | 스케일 |
| 7 | 반사/광택 | 금속캔/플라스틱 광택 | depth 노이즈·반사 |

> **권장**: 최소 **대칭 1개 + 반사 1개 + 고텍스처 1개**를 반드시 포함(없으면 논문 "robustness across object types" 주장 약화). **7개 전부 CAD 확보가 선결**(아래 GT·Plan).

---

## Environment Design (Task 5)

| 환경 | 구성 | 왜 필요(어떤 RQ) |
|---|---|---|
| **Office** | 모니터·의자·박스(현 bag과 동일 도메인) | baseline 검출/pose, 기존 데이터와 연속성 (RQ1/2) |
| **Lab(복잡 배경)** | 잡다한 물체·유사색 배경(현 milk_0609 유형) | 클러터·유사외관 FP stress, dense multi-object (RQ3) |
| **Hallway(장거리)** | 긴 복도, 특징 적음 | odom drift·RPE, long sequence (RQ5) |
| **Warehouse/선반** | 다중 객체 선반·bin | dense·가림(IC-BIN/HOPE식) (RQ3) |
| **Dynamic** | 사람 이동·물체 이동(Bonn/TUM-dynamic식) | dynamic 견고성, SLAM·pose 동시 stress (RQ7) |

> 환경 다양성은 **도메인 일반화** 주장의 근거. 최소 **Office+Lab+Hallway+Dynamic 4종**. Warehouse는 dense(L3) 보강용 옵션.

---

## Husky Trajectory Design (Task 6)

| Trajectory | 패턴 | 검증(metric) |
|---|---|---|
| **Static Observation** | 제자리 회전(객체 응시) | 시점변화 대비 pose 안정(ADD-S vs yaw), tracking jitter (RQ4) |
| **Linear** | 직선 접근/통과 | scale·거리에 따른 검출/pose, odom **RPE**(국부 drift) (RQ5) |
| **Circle (Orbit)** | 객체 주위 360° | full-viewpoint pose 커버(템플릿 시점 일반화), object-centric (RQ1/RQ4) |
| **Figure-8** | 8자 = 교차 재방문 | **loop closure** 트리거, ATE 개선 측정 (RQ6) |
| **Long Mission** | 수십 m 왕복·다실 | 누적 drift, lifelong/relocalization, object map 일관성 (RQ5/RQ6) |

> KITTI가 loop-closure 표준인 이유 = 빈번한 재방문. **figure-8/재방문을 반드시 1개 이상** 포함해야 RQ6 주장 가능. orbit은 SAM-6D 템플릿(전구 42뷰) 시점 커버 검증과도 직결.

---

## Ground Truth Requirements (Task 7)

**기록해야 할 GT(재라벨링 비용 최소화 위해 수집 시점부터)**:

| GT | 방법 | 우선순위 |
|---|---|---|
| **객체 CAD/스캔 모델** | 7개 전부 스캔(photogrammetry / RealSense scan / BundleSDF) | **P0(선결)** — 없으면 ADD-S 불가 |
| **per-frame 객체 6D pose** | CAD + **AprilTag 보드 앵커** + **RealSense depth ICP 전파**(BOP 레시피) | **P0** |
| **객체 visible 0/1 + Object ID** | 라벨(현 visibility_labels 방식 확장: 객체별) | P0 |
| **카메라/로봇 궤적(SLAM pose)** | LiDAR-SLAM(ICP-to-map) 또는 total-station/RTK 기준; 가능하면 mocap | **P1**(L4/L5 필수) |
| **Loop closure event** | 재방문 타임스탬프·장소 태깅 | P1(L5) |
| **Map consistency** | object map 중복/일관 라벨 | P2(파생) |
| **센서 동기/캘리브** | 카메라 intrinsics+extrinsics, cam↔LiDAR extrinsic, 시간동기 | **P0**(모든 GT의 전제) |

> **저비용 6D GT 레시피(mocap 랩 없이, 권장)**: ①정확 CAD ②AprilTag 보드로 world↔object 앵커 고정(또는 첫 프레임 수동 정렬) ③매 프레임 **depth-ICP**로 pose 전파·정제. = YCBInEOAT/T-LESS GT 구축 방식. **지금부터 cam/LiDAR extrinsic·시간동기·tag 보드를 함께 기록**하면 나중 재라벨링이 거의 불필요.
>
> **궤적 GT**: mocap 없으면 **LiDAR-SLAM을 survey/total-station 보정 prior에 ICP 정합**(Newer College 방식) 또는 RTK(실외). 최소한 LiDAR를 항상 같이 녹화해 사후 GT 산출 여지를 남길 것.

---

## Minimal Viable Dataset (Task 8)

**조건**: 실물 7개, Husky, D455f+LiDAR, ROS2 bag. **목표 = 8개 RQ를 "최소한 한 줄씩" 채우는 가장 작은 집합.**

| 구성요소 | MVD 사양 |
|---|---|
| **선결물** | 7객체 CAD 스캔 + GT 파이프라인(AprilTag+depth-ICP) + cam/LiDAR 캘리브·동기 |
| **L1** | 핵심 3객체(저텍스처 milk / 대칭 원통 / 고텍스처 박스) × present+absent = **6 bag**, Office 정적 |
| **L2** | 3객체 동시 × 2환경(Office/Lab) = **4 bag** |
| **L3** | 5~7객체 dense × **2 bag**(Lab) |
| **L4** | Husky 주행: static-rotate·linear·orbit·**dynamic(사람)** = **4 bag**(Office/Hallway) |
| **L5** | figure-8 1 + long-mission 1 = **2 bag**(Hallway+Lab 회랑) |
| **합계** | **≈ 18 시퀀스** (+CAD 7), 총 ~25~40분 |

> 이 18 시퀀스로 RQ1~RQ8을 전부 한 번씩 검증 가능(다양성 우선, 볼륨 최소). 벤치마크 규모(10~25 시퀀스)와 정합.

---

## 3-Month Collection Plan (Task 9)

저장량 추정 근거: 현 RGB-D bag ≈ **~3 GB/분**(only_milk 1.3GB/29s, milk_0609 3.4GB/65s). +LiDAR ≈ **~4~5 GB/분**.

| Phase | 기간 | 산출 | bag/환경/객체 | 시간 | 저장 |
|---|---|---|---|---|---|
| **Phase 1 (즉시)** | ~4주 | **7객체 CAD 스캔 + GT 파이프라인 PoC + L1/L2** | 10 bag / Office·Lab 2 / 7객체 | ~15분 영상 + 스캔 | ~50~80 GB |
| **Phase 2 (중기)** | ~4주 | **L3 dense + L4 Husky navigation + dynamic** | 8~10 bag / +Hallway·Dynamic / 5~10객체 | ~20~30분 | ~120~180 GB |
| **Phase 3 (최종 실험)** | ~4주 | **L5 long/loop + 전 레벨 재현·정제 + 벤치마크화** | 5~8 bag / 회랑·다실 / 전체 | ~30~40분 | ~150~250 GB |
| **합계** | 3개월 | ~25~30 시퀀스 + 7 CAD | 4~5 환경 / 7~10 객체 | ~70~90분 | **~350~500 GB** (+백업) |

> **Phase 1의 80%는 데이터 촬영이 아니라 CAD 스캔 + GT 파이프라인 구축**에 써야 함(이게 없으면 Phase 2/3 데이터가 라벨 불가가 됨). LiDAR는 Phase 1부터 항상 같이 녹화(사후 궤적 GT 여지).

---

## 벤치마크 근거 요약(인용)

- **6D pose 다양성/메트릭**: BOP(LM-O·T-LESS·YCB-V·HB·ITODD·IC-BIN·TUD-L), ADD/ADD-S/AUC, BOP-AR=VSD/MSSD/MSPD. T-LESS=저텍스처·대칭 산업부품 전용. [bop.felk.cvut.cz/datasets](https://bop.felk.cvut.cz/datasets/), [T-LESS arXiv:1701.05498](https://arxiv.org/abs/1701.05498), [BOP'19](https://bop.felk.cvut.cz/challenges/bop-challenge-2019/)
- **Pose tracking**: YCBInEOAT(동적객체·정적카메라, 9 vid×~1k, 수동 6D GT), BCOT(markerless, 2-cam 삼각측량, 404 seq/126k). [se(3)-TrackNet arXiv:2007.13866](https://arxiv.org/pdf/2007.13866), [BCOT CVPR'22](https://ar3dv.github.io/BCOT-Benchmark/)
- **Object SLAM**: Fusion++/QuadricSLAM/CubeSLAM/DSP-SLAM/vMAP — TUM·ScanNet·Replica·KITTI·Redwood 사용, object-localization·reconstruction + ATE/RPE. [DSP-SLAM](https://jingwenwang95.github.io/dsp-slam/), [vMAP arXiv:2302.01838](https://arxiv.org/pdf/2302.01838)
- **궤적/루프/동적 GT**: TUM RGB-D(mocap), EuRoC(Leica+Vicon), KITTI(RTK, loop 표준), Newer College(LiDAR ICP-to-survey), OpenLORIS(OptiTrack+LiDAR, lifelong), Bonn/TUM-dynamic(OptiTrack). ATE/RPE 정의. [ATE/RPE](https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf), [Bonn dynamic](https://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/), [OpenLORIS](https://lifelong-robotic-vision.github.io/dataset/scene.html)
- **GT 저비용 레시피**: BOP render-and-align + ICP, AprilTag/ArUco 보드. [AprilTag](https://github.com/AprilRobotics/apriltag)
- **규모 norm**: 단프레임 test는 수백~1k/객체, tracking은 ~9k(YCBInEOAT), credible 최소 ≈ 10~25 시퀀스.

---

## 한계 / 가정(추측 금지)

1. SLAM_0609는 db3 미추출(tar.gz만) — 토픽은 타 bag과 동일(RGB-D+tf_static)로 추정. 실제 확인 필요.
2. bag 길이는 color msg/30fps 추정(metadata duration 필드 파싱 실패).
3. 저장량은 현 RGB-D bag bitrate 외삽 + LiDAR 가정치 → 실제 LiDAR(Ouster/Velodyne) 모델에 따라 변동.
4. mocap 랩 부재 가정(있으면 GT 정확도↑·파이프라인 단순화). Husky wheel-odom·IMU 토픽 가용성은 미확인(있으면 RPE/VIO GT 보강).
5. SAM-6D는 객체별 CAD 필수 → 7객체 CAD 확보가 모든 multi-object pose 실험의 hard 선결조건.

---

## Recommended Next BMAD Step

**→ Quick Dev** (PRD/추가 Research 아님).

이유: 갭의 본질이 "무엇을 주장할지"(이 보고서가 정의)보다 **"GT를 어떻게 만들지"의 실행**에 있다. 데이터 수집을 시작하기 전에 **GT 파이프라인 PoC**가 동작해야 뒤따르는 모든 bag이 라벨 가능해진다. PRD는 시스템 통합 단계에서, 추가 Research는 특정 알고리즘(object-SLAM 백엔드) 선택 시 별도로.

권장 Quick-Dev 범위:
1. **객체 CAD 스캔 워크플로 PoC** — 1~2개 객체를 RealSense/photogrammetry로 스캔→정합→PLY(현 Milk CAD 포맷과 호환) 검증.
2. **per-frame 6D pose GT 파이프라인 PoC** — AprilTag 보드 앵커 + depth-ICP 전파로 기존 only_Milk/milk_0609 프레임에 milk 6D GT를 자동 부여, ADD-S 산출까지 end-to-end 1회 검증.
3. **수집 프로토콜 체크리스트** — bag 녹화 시 필수 토픽(color/depth/cam_info/LiDAR/odom/imu/tf/tag)·캘리브·동기 기록 표준 문서화.

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 research .md 표준 — 이 절로 마무리.

**핵심 판정**: 현재 데이터는 단일객체 검출/거부까지만 가능. 논문 주장(multi-object·tracking·object-aware SLAM·loop)을 위해선 **①7객체 CAD ②6D/궤적 GT 파이프라인 ③5레벨 taxonomy 18~30 시퀀스**가 필요하며, **선결 블로커는 GT 파이프라인**이다. 다음은 **Quick-Dev(GT 파이프라인 PoC)** 권장.

선택지:
- **(A) Quick-Dev: CAD 스캔 + 6D GT 파이프라인 PoC** *(추천)* — 데이터 수집 전 라벨링 실행가능성부터 확증(가장 큰 리스크 해소).
- **(B) Quick-Dev: 수집 프로토콜·디렉터리·bag 녹화 체크리스트 표준화** 먼저 — Phase1 촬영 즉시 시작 가능하게.
- **(C) 추가 Technical Research: object-aware SLAM 백엔드 선택**(QuadricSLAM/DSP-SLAM/vMAP 등 vs Husky 실시간 제약) — 시스템 아키텍처 확정용.

→ **어느 쪽으로 진행할까요?** (추천: A — GT 실행가능성이 전체 데이터 계획의 단일 최대 리스크)
