---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments: []
workflowType: 'research'
lastStep: 6
research_type: 'technical'
research_topic: '이더넷 스위치 연결 2대 노트북 간 ROS2 멀티센서 timestamp 동기화 및 bag 수집'
research_goals: '동기화된 ROS2 bag 데이터를 수집하여 SLAM+SAM 통합(맵에서 객체 위치 추정)에 활용'
user_name: 'ldh'
date: '2026-07-07'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-07-07
**Author:** ldh
**Research Type:** technical

---

## Research Overview

본 리서치는 이더넷 스위치로 연결된 노트북 2대(A: RealSense D455f+VLP-16+Xsens IMU+Husky A200, B: RealSense D455f)에서 **timestamp로 상호 동기화된 ROS2 bag 데이터를 수집**하는 방법을, 실기 검증 불가 제약 하에 공식문서·논문·커뮤니티 소스 기반으로 조사한 것이다. 최종 용도는 SLAM(현재 위치) × SAM(객체 6D pose) → 맵 상 객체 위치 추정이다.

핵심 결론은 **"동기화는 단일 기법이 아니라 서로 직교하는 여러 계층의 정렬 문제"** 라는 것이다: ① 이더넷 클록 동기(chrony/PTP)가 두 노트북의 타임스탬프 라벨을 공통 도메인에 올리는 **필수 기반**이고, ② 카메라 9핀 HW 동기는 두 D455f의 노출 순간을 맞추는 **선택적 정밀화**이며, ③ LiDAR·IMU는 셔터형 센서가 아니라 **정확한 타임스탬프 + 보간/디스큐**로 다루는 것이 정석이다. 아키텍처는 **"머신별 로컬 MCAP bag 수집 + 클록 동기 + 사후 병합"** 이 실제 분산 SLAM 데이터셋 논문들의 표준이며 신뢰성·대역폭 면에서 단일 DDS 도메인 통합 수집보다 우월하다.

전체 Executive Summary·전략 권고·구현 로드맵은 하단 **Research Synthesis** 절에 종합되어 있으며, 실행 가능한 설정 파일과 커맨드는 **Implementation Approaches** 절에 정리되어 있다.

---

<!-- Content will be appended sequentially through research workflow steps -->

## Technical Research Scope Confirmation

**Research Topic:** 이더넷 스위치 연결 2대 노트북 간 ROS2 멀티센서 timestamp 동기화 및 bag 수집
**Research Goals:** 동기화된 ROS2 bag 데이터를 수집하여 SLAM+SAM 통합(맵에서 객체 위치 추정)에 활용

**하드웨어/구성 전제:**

- 노트북 A: RealSense D455f + Velodyne VLP-16 + Xsens IMU + Husky A200
- 노트북 B: RealSense D455f
- 두 노트북은 이더넷 스위치로 연결
- 두 D455f는 9핀 외부 케이블 하드웨어 동기화 (A=master, B=slave) — 가정
- 제약: 현재 노트북 1대·센서 미보유 → 실기 검증 불가, 문헌/공식소스 기반 조사만 가능

**확정된 기술 리서치 범위 (7개 영역):**

1. 시간 동기화 아키텍처 (PTP/IEEE 1588, NTP/chrony, 이더넷 스위치 경유 클록 정렬)
2. 센서별 타임스탬프 소스 (D455f global time, VLP-16, Xsens IMU, Husky)
3. 분산 수집 구성 (로컬 개별 bag + 사후정렬 vs 단일 DDS 도메인 통합 수집)
4. 하드웨어 동기화(D455f master/slave 9핀) 연동 및 검증
5. 대역폭/처리량 검증 (USB 버스 · 네트워크(스위치) · 디스크 I/O · CPU/DDS)
6. 실행 파일/설정 산출물 (각 노트북 launch·record 커맨드, PTP/chrony conf)
7. SLAM+SAM 활용 적합성 (message_filters 정합, 오프라인 재생 정렬)

**리서치 방법론:**

- 최신 공개 웹 자료 + 공식 문서(Intel RealSense, Velodyne, Xsens, Clearpath Husky, ROS2/DDS, linuxptp) 기반
- 핵심 기술 주장은 다중 소스 교차검증
- 불확실 정보는 신뢰도(confidence) 레벨 명시, "가정/미검증" 항목 명확히 구분

**Scope Confirmed:** 2026-07-07

---

## Technology Stack Analysis

> 이 주제의 "기술 스택"은 프로그래밍 언어가 아니라 **① 센서 ROS2 드라이버 ② 시간동기화(linuxptp/chrony) ③ rosbag2 저장 ④ DDS/rmw** 4개 레이어다. 각 레이어에서 timestamp가 어디서 찍히는지가 전체 동기화 품질을 결정한다.

### Layer 1 — 센서별 ROS2 드라이버 & 타임스탬프 소스

핵심 원리: **모든 센서 타임스탬프는 결국 "센서 하드웨어 클록" 또는 "호스트 시스템 클록" 중 하나로 귀결**된다. 크로스-노트북 동기화를 위해서는 (a) 각 센서를 최대한 호스트 클록 도메인으로 정렬시키고, (b) 두 호스트의 시스템 클록을 PTP/NTP로 맞추는 2단계가 필요하다.

| 센서 | 드라이버(패키지) | 핵심 파라미터 | 타임스탬프 동작 |
|---|---|---|---|
| RealSense D455f (A,B) | `realsense2_camera` (realsense-ros 4.5x ↔ librealsense 2.5x) | `<module>.global_time_enabled` (기본 true), `depth_module.inter_cam_sync_mode` | global_time=on이면 librealsense가 카메라↔호스트 클록 선형회귀로 **호스트 epoch(ms)** 로 변환해 header.stamp에 기록 → 드리프트 제거. off면 sensor/USB 도메인 |
| RealSense HW sync | 동일 | `inter_cam_sync_mode`: 0=none,1=Master,2=Slave,3=Full Slave(depth+color),4=Genlock | 9핀 Pin5(SYNC)↔Pin5, Pin9(GND)↔Pin9 배선. **프레임 노출**만 정렬하고 타임스탬프는 각자 클록 → global_time과 병행 필요 |
| Velodyne VLP-16 (A) | `velodyne_driver` (ros2 branch) + `velodyne_pointcloud` | `gps_time`(기본 false), `time_offset` | gps_time=false → **호스트 UDP 패킷 도착시각**. true → 패킷 내 디바이스 시각(단, GPS/PPS+NMEA 배선 필수). **네이티브 PTP 경로 없음** |
| Xsens IMU (A) | 커뮤니티 `bluespace_ai_xsens...` **또는** 공식파생 `xsenssupport/Xsens_MTi_ROS_Driver` | 공식판 `time_option`: 0=MTi UTC, 1=SampleTimeFine(HW), 2=host | bluespace 기본 = **호스트 수신시각**(고레이트에서 지터 경고). 공식판은 `time_option`으로 HW시각 선택 가능 |
| Husky A200 (A) | `clearpath_*` 통합스택 (`robot.yaml`, Humble 주력·Jazzy 문서화) | `/etc/clearpath/robot.yaml` | odom/imu 모두 **호스트 stamp**(MCU 시리얼 경유). 하드웨어 클록 없음 → 호스트 NTP/PTP에 의존 |

_핵심 시사점: D455f는 `global_time_enabled=true` 확인 + HW sync는 A=mode1/B=mode2로 노출 정렬, Xsens는 공식 드라이버 `time_option`으로 HW시각, VLP-16·Husky는 호스트 클록에 얹힘._
_Source: https://dev.intelrealsense.com/docs/ros2-wrapper , https://github.com/ros-drivers/velodyne/blob/ros2/velodyne_driver/README.md , https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client , https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/_

### Layer 2 — 시간동기화 스택 (두 노트북 시스템 클록 정렬)

ROS2는 기본적으로 각 머신의 **시스템 클록(CLOCK_REALTIME)** 으로 header.stamp를 찍는다. 따라서 크로스-노트북 비교 가능성은 전적으로 두 리눅스 시스템 클록을 얼마나 잘 맞추느냐에 달려 있다(ROS2 자체는 아무것도 안 함).

| 방식 | 타임스탬프 방식 | 노트북 2대+일반 스위치 정확도 | 비고 |
|---|---|---|---|
| PTP (linuxptp: ptp4l+phc2sys) | HW (NIC PHC) | sub-µs ~ 수십 ns | 두 NIC 모두 PHC 필요 (`ethtool -T` 확인). Intel i210/i225/i226 O, Realtek/USB NIC 대부분 X |
| PTP | SW 타임스탬핑 | 수 µs ~ ~10µs | HW 없으면 chrony 대비 이점 거의 소멸 |
| **chrony/NTP** | SW | **~2µs(조용한 LAN) ~ 수십 µs, 1ms 이내** | **일반 노트북 권장 기본값**. 특별 HW 불필요 |
| chrony/NTP | HW (`hwtimestamp *`) | 수십 ns | NIC가 PHC 지원 시 PTP급 |
| ntpd(구형) | SW | ~234µs | chrony보다 열등 |

_실무 권장: `ethtool -T`로 PHC 유무 확인 → PHC 없으면(일반 노트북) **chrony** 마스터/클라이언트 구성으로 수십µs 달성(센서 정합엔 충분). PHC 있고 <100µs 필요 시 chrony `hwtimestamp *` 또는 ptp4l+phc2sys. 비-PTP 스위치는 boundary/transparent clock이 아니라 큐잉 지터로 µs급이 상한._
_Source: https://chrony-project.org/examples.html , https://quantum5.ca/2023/01/26/microsecond-accurate-time-synchronization-lan-with-ptp/ , https://free5gc.org/blog/20250709/20250709/ , https://scottstuff.net/posts/2025/05/20/time-nics/_

### Layer 3 — rosbag2 저장 스택

| 항목 | 발견 | 권장 |
|---|---|---|
| 저장 플러그인 | Humble 기본=sqlite3, Iron/Jazzy 기본=MCAP. MCAP이 고처리량에서 동등~우수 | **MCAP** (Humble은 `rosbag2_storage_mcap` 설치) |
| 압축 | MCAP none/lz4/zstd. zstd 기본레벨은 CPU바운드(~9.9MB/s 실측)로 고레이트 기록 throttle | 무압축 또는 **lz4/fastwrite**; depth/pointcloud는 image_transport/point_cloud_transport 상류압축 |
| 드롭 방지 | `--max-cache-size`(기본~100MB) 확대, recorder QoS reliable/keep_all 오버라이드, bag-split 경계 드롭 주의 | 버스트 흡수 위해 캐시 상향 + QoS 오버라이드 |

_Source: https://mcap.dev/guides/benchmarks/rosbag2-storage-plugins , https://foxglove.dev/blog/mcap-as-the-ros2-default-bag-format , https://github.com/ros2/rosbag2/issues/1430_

### Layer 4 — DDS/rmw & 멀티머신 수집 아키텍처

| 항목 | 발견 |
|---|---|
| rmw 구현 | CycloneDDS/FastDDS. FastDDS가 대용량 pointcloud 단일구독 안정성 우위. 대용량 메시지는 RTPS 프래그먼트화 → 프래그먼트 1개 손실 시 전체 재전송 |
| 디스커버리 | 기본 멀티캐스트. 매니지드 스위치/IGMP snooping이 디스커버리 차단 흔함 → FastDDS Discovery Server(유니캐스트) 또는 CycloneDDS `<Peers>` 유니캐스트, Jazzy `ROS_STATIC_PEERS` |
| 도메인 격리 | `ROS_DOMAIN_ID` 일치 필요, 옵션(a) 로컬수집 시 `ROS_LOCALHOST_ONLY=1`/`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 완전 격리 가능 |

_핵심: ROS2엔 아직 1급 분산 레코더가 없음(rosbag2 issue #1548) → "각 머신 로컬 수집 후 사후 병합"이 사실상 표준._
_Source: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html , https://github.com/ros2/rosbag2/issues/1548 , https://github.com/ros2/rmw_cyclonedds/issues/292_

### Technology Adoption Trends (이 도메인 현재 표준)

- **분산 SLAM 데이터셋 논문들**(Race-Against-the-Machine, SMapper, Rosario v2, FusionPortableV2, S3E)은 거의 모두 **머신별 로컬 rosbag + NTP/PTP 클록 동기화 + 캡처 전후 오프셋 로깅** 패턴을 채택.
- rosbag2 저장 포맷은 sqlite3 → **MCAP**으로 이동(Foxglove 생태계).
- 시간동기화는 GNSS/PPS grandmaster가 하이엔드, 노트북 2대 급은 chrony(soft) ~ PTP(HW NIC)가 현실적 범위.
_Source: https://arxiv.org/pdf/2311.02667 , https://arxiv.org/pdf/2509.09509 , https://arxiv.org/pdf/2404.08563_

---

## Integration Patterns Analysis

> 이 도메인의 "통합 패턴"은 REST/gRPC가 아니라 **수집 아키텍처 · 오프라인 시각정합(message_filters) · bag 병합/재생 · SLAM×SAM 데이터 흐름**이다.

### 수집 아키텍처 패턴: 옵션 (a) vs 옵션 (b)

| 기준 | (a) 머신별 로컬 bag + 사후 timestamp 정렬 | (b) 단일 DDS 도메인 통합 수집 |
|---|---|---|
| **신뢰성** | **최상**. 캡처 중 네트워크·디스커버리·대역폭 의존 없음 (loopback/로컬 DDS만) | 낮음·네트워크 의존. B의 데이터가 실시간으로 스위치를 통과 → 손실/혼잡/디스커버리 히컵이 곧 bag 결손 |
| **대역폭** | 캡처 중 네트워크 부하 0, 각 bag 로컬 디스크 기록 | D455f 원본(color+depth ~60MB/s)이 1GbE의 절반 소모 → **원본 스트림 네트워크 전송은 포화 위험** |
| **디스커버리** | 무관(`ROS_LOCALHOST_ONLY=1`로 완전 격리 가능) | 멀티캐스트 디스커버리를 매니지드 스위치/IGMP snooping이 자주 차단 → FastDDS Discovery Server(유니캐스트) 또는 CycloneDDS `<Peers>` 필요 |
| **클록 동기 요구** | **필수** — header.stamp가 각 머신 클록. slop 이내로 정렬돼야 정합 | 여전히 필수. bag은 recv-time으로 기록돼 재생순서는 맞지만, header.stamp 융합엔 클록동기 필요 |
| **SLAM 데이터셋 적합성** | ✅ 실제 논문 표준 패턴 | 저레이트 토픽·단일 통합파일이 하드 요건일 때만 |

**권장: 옵션 (a)** — 신뢰성·대역폭 헤드룸을 극대화하고 어려운 요구(클록 동기)를 어차피 융합에 필요한 곳으로 집중시킨다. ROS2엔 1급 분산 레코더가 없어(rosbag2 #1548) "로컬 수집 + 사후 병합"이 사실상 표준.
_Source: https://arxiv.org/pdf/2311.02667 , https://github.com/ros2/rosbag2/issues/1548 , https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html , https://husarion.com/tutorials/ros2-tutorials/6-robot-network/_

### 시각 정합 패턴: message_filters (ROS2)

두 노트북의 센서를 오프라인에서 융합하는 핵심 "API". header.stamp 기준으로 N개 스트림을 정렬한다.

- **ExactTime** (`sync_policies::ExactTime`): 모든 입력의 stamp가 정확히 동일해야 콜백 → HW 트리거 아니면 과도하게 엄격.
- **ApproximateTime** (`ApproximateTimeSynchronizer`): 허용오차 내 최신 집합 매칭. **`slop` 파라미터**(초)가 매칭 허용 스프레드. `(max(stamps)-min(stamps)) < slop`.
- Python 시그니처: `ApproximateTimeSynchronizer(fs, queue_size, slop, allow_headerless=False, ...)`. `allow_headerless=True`면 header 없는 메시지에 현재 ROS시각 대입(정확도 저하).

**클록 요구:** 두 정책 모두 header.stamp를 비교하므로 **두 머신 클록이 slop 이내로 동기**돼야 함. 안 그러면 상수 클록오프셋이 센서지연으로 위장돼 매칭 실패 또는 오매칭. **slop은 "잔여 클록스큐 + 전송지터"를 흡수하되 오매칭을 피할 만큼(느린 센서 주기의 ~½)** 잡는 게 관건.
_Source: https://docs.ros.org/en/humble/p/message_filters/doc/index.html , https://github.com/ros2/message_filters/blob/rolling/doc/index.rst_

### bag 병합·오프라인 재생 패턴

- `ros2 bag play`는 **기록(log) 타임스탬프** 순으로 재생. 여러 bag을 넘기면 **글로벌 기록시각으로 인터리브된 단일 스트림**으로 취급.
- ⇒ **정렬은 전적으로 "기록 시점에 클록이 동기였는가"에 의존.** 동기된 상태로 동시 기록된 두 bag은 정렬 재생, 오프셋이 있었으면 그 오프셋이 log time에 각인됨.
- **병합:** `rosbag2_transport::bag_rewrite`(=`ros2 bag convert` 다중입력)로 시각순 단일 bag 생성.
- **알려진 오프셋 보정:** play-time per-bag 시프트 기능 없음 → bag_rewrite로 타임스탬프 재작성 후 병합, 또는 융합 노드에서 header.stamp 비교 시 오프셋 적용.
- **재생 시 `use_sim_time`:** 기록 시엔 실 wall/HW stamp 사용(sim time 금지). 재생 시 `ros2 bag play --clock`이 `/clock` 발행 → 하위 노드 `use_sim_time:=true`로 bag 시각 추종.
_Source: https://github.com/ros2/rosbag2/issues/668 , https://github.com/ros2/rosbag2 , https://docs.ros.org/en/jazzy/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html_

### SLAM × SAM 통합 데이터 흐름 (최종 목적 정합)

```
[노트북 A]  RealSense(A) RGB-D ─┐
            VLP-16 LiDAR ───────┼─► ORB-SLAM3/RTAB → T_map_cam(t)  (현재 위치)
            Xsens IMU ──────────┘
            Husky odom ─────────► (보조 오도메트리/EKF)

[노트북 B]  RealSense(B) RGB-D ──► SAM-6D ISM+PEM → T_cam_obj(t)  (객체 6D pose)

동기화 접착제:  PTP/chrony로 A·B 시스템클록 정렬  +  D455f 9핀 HW sync(A=master,B=slave)
정합:  message_filters ApproximateTime(slop) 로 T_map_cam(t) ↔ T_cam_obj(t) 같은 t 매칭
결과:  T_map_obj = T_map_cam(t) · T_cam_B_cam_A · T_camB_obj(t)  → 맵 상 객체 위치
```

- **관건:** A의 SLAM pose와 B의 SAM object pose를 **동일 t로 묶으려면** 두 노트북 클록동기(soft chrony 수십µs면 충분 — SLAM/객체는 수십ms 케이던스)와 **A↔B 카메라 간 외부 캘리브레이션(extrinsic `T_camB_camA`)** 이 필수. HW sync(9핀)는 두 D455f의 노출순간을 맞춰 모션블러/시차 오차를 줄임.
- 이는 기존 objectmemory 파이프라인(SLAM traj × SAM pem × bag → T_map_obj)의 **멀티-노트북 확장판**에 해당.
_Source: https://docs.ros.org/en/humble/p/message_filters/doc/index.html , https://arxiv.org/pdf/2404.08563_

---

## Architectural Patterns and Design

### System Architecture Patterns — 권장 전체 구조

**패턴: "분산 로컬 수집 + 공유 시간베이스 + 사후 병합" (Distributed-capture / Shared-clock / Post-merge)**

```
                    ┌──────────────── Ethernet Switch (1GbE) ────────────────┐
                    │  용도 = ①시간동기(chrony/PTP) ②디스커버리(옵션) ③원격 트리거   │
                    │  ※ 원본 센서 스트림은 스위치로 흘리지 않음                      │
                    └────────┬───────────────────────────────────┬───────────┘
                             │                                   │
        ┌────────────────────┴─────────────┐        ┌────────────┴───────────────┐
        │  노트북 A  (Time Master)          │        │  노트북 B  (Time Slave)      │
        │  chrony server + (opt) ptp4l gm  │        │  chrony client              │
        │                                  │        │                            │
        │  realsense2_camera (A, master)   │        │  realsense2_camera (B,slave)│
        │    global_time_enabled=true      │        │    global_time_enabled=true │
        │    inter_cam_sync_mode=1          │        │    inter_cam_sync_mode=2/3  │
        │  velodyne_driver (gps_time=false) │        │                            │
        │  xsens (time_option=0 UTC)        │        │                            │
        │  clearpath a200 (host stamp)      │        │                            │
        │        │                          │        │        │                   │
        │   ros2 bag record  ─► bag_A.mcap  │        │   ros2 bag record ─►bag_B.mcap│
        │        (로컬 NVMe)                │        │        (로컬 NVMe)           │
        └──────────────────────────────────┘        └────────────────────────────┘
                    9핀 HW sync cable: A Pin5→B Pin5, A Pin9→B Pin9 (노출 동기)
```

- **결정 1 — 로컬 수집(옵션 a):** 각 노트북이 자기 bag을 로컬 NVMe에 기록. 스위치는 센서 데이터가 아니라 "시간·제어"만 나름. 이유: D455f 원본이 1GbE 절반 소모 + DDS 프래그먼트 손실 위험 회피.
- **결정 2 — 이중 동기 축:** (i) **소프트웨어 축** = chrony/PTP로 두 시스템 클록 정렬(header.stamp 도메인 통일), (ii) **하드웨어 축** = 9핀 sync로 두 D455f 노출순간 정렬(모션블러/시차 억제). 둘은 독립·상보적.
- **결정 3 — 마스터 집중:** 시간 마스터 = 노트북 A(센서 다수·Husky 탑재). B는 클라이언트+D455f slave.
_Source: https://github.com/ros2/rosbag2/issues/1548 , https://chrony-project.org/examples.html , https://dev.intelrealsense.com/docs/ros2-wrapper_

### Design Principles and Best Practices

- **"타임스탬프는 최대한 상류에서 확정"** — D455f global_time=on, Xsens time_option=0(UTC)로 드라이버 단계에서 호스트/UTC 도메인 정착. VLP-16·Husky는 호스트 stamp이므로 호스트 클록동기가 이들의 정확도를 결정.
- **캡처 전후 오프셋 로깅(정량 감사)** — 각 run 전후로 `chronyc tracking`(또는 `pmc GET CURRENT_DATA_SET`) 값을 별도 기록 → 잔여 스큐를 오프라인에서 정량·보정. (분산 SLAM 데이터셋 논문들의 공통 관행)
- **관심사 분리** — 수집(record)과 융합(message_filters)을 분리. 기록은 실 wall/HW stamp, 융합·정합은 오프라인 재생 단계에서.
- **재현성** — bag 옆에 `metadata.yaml`(센서·해상도·sync모드·클록방식·오프셋로그) 동봉.
_Source: https://arxiv.org/pdf/2311.02667 , https://arxiv.org/pdf/2404.08563_

### Scalability and Performance Patterns (대역폭 배치)

경로별 병목 예산(848×480@30 기준, [calc]=사양계산):

| 경로 | 부하 | 한계 | 판정 |
|---|---|---|---|
| USB3 (A/B 각 D455f) | depth+color+IMU ~49–61MB/s(~390–490Mbps) | USB3 실효 ~3.2Gbps, 단 컨트롤러 공유 시 프레임드롭 | **카메라마다 독립 USB3 컨트롤러 배정** |
| 네트워크(스위치) | 시간·제어만(수 Mbps) | 1GbE 실효 ~118MB/s | 여유 大 (원본 스트림 미전송이 전제) |
| 디스크 I/O (A) | D455f 61 + VLP-16 7–10 ≈ **68–71MB/s** 총합 | DRAM탑재 TLC NVMe >500MB/s | 여유 大. 단 **QLC/SD/USB 매체·장시간 SLC소진 시 병목** |
| CPU/DDS | 직렬화 + (압축 시) zstd ~9.9MB/s로 throttle | — | **무압축/lz4**, `--max-cache-size`↑, QoS reliable/keep_all |

- **VLP-16은 자체 100Mbps 이더넷**(~7.5Mbps wire)이라 A의 별도 NIC/서브넷으로 받는 게 스위치 트래픽과 무관하게 안전.
_Source: https://mcap.dev/guides/benchmarks/rosbag2-storage-plugins , https://dev.realsenseai.com/docs/multiple-depth-cameras-configuration/ , https://data.ouster.io/downloads/velodyne/user-manual/vlp-16-user-manual-revf.pdf_

### Integration and Communication Patterns

- **스위치의 3가지 용도만:** ① chrony/PTP 시간동기 트래픽, ② (옵션 b 채택 시) DDS 디스커버리 — 이땐 유니캐스트(FastDDS Discovery Server / CycloneDDS `<Peers>`)로 멀티캐스트 차단 회피, ③ 동시 시작/정지 원격 트리거(ssh/서비스).
- **VLP-16 UDP**는 전용 링크로 분리 권장(스위치 공유 시 브로드캐스트/멀티캐스트 부하 주의).
- **동시 record 시작:** 두 노트북에서 `ros2 bag record`를 ssh 팬아웃 또는 공유 시작신호로 거의 동시 기동(클록 동기 상태에서 log time이 정렬됨).
_Source: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html , https://github.com/eclipse-cyclonedds/cyclonedds/issues/1323_

### Data & Deployment Architecture

- **저장 포맷:** MCAP(Humble은 `rosbag2_storage_mcap` 설치), 무압축 또는 lz4. bag당 `metadata.yaml` 동봉.
- **디렉토리 레이아웃(제안):** `session_YYYYMMDD/{A/bag_A.mcap, B/bag_B.mcap, sync/clock_offset_pre.txt, sync/clock_offset_post.txt, calib/T_camB_camA.yaml, metadata.yaml}`
- **배포/OS:** 두 노트북 동일 ROS2 distro(Humble 또는 Jazzy) 통일 — Clearpath는 Humble 주력·Jazzy 문서화. rmw 통일(둘 다 CycloneDDS 또는 둘 다 FastDDS).
- **보안:** 폐쇄 LAN 전제라 인증/암호화는 비핵심(필요 시 SROS2). 스위치는 신뢰 네트워크 가정.
_Source: https://docs.clearpathrobotics.com/docs/ros/installation/upgrading/ , https://foxglove.dev/blog/mcap-as-the-ros2-default-bag-format_

---

## 설계 결정 로그 (Q&A로 확정된 사항)

이 리서치 중 사용자와의 문답으로 다음 설계 결정이 확정되었다.

- **[결정 A] A→B pose/TF 전송 = ROS2/DDS 네이티브 채택** (raw 이더넷 소켓 기각). 근거: payload가 pose+TF로 tiny → raw 소켓의 대역폭 이점 무의미, `tf2`가 stamped transform 버퍼/보간을 위해 존재, `/tf_static`·`/robot_description` transient_local latching이 URDF/extrinsic 전파에 최적. DDS 유일 단점(멀티캐스트 디스커버리)은 유니캐스트 Discovery Server로 해결. 클록 동기 요구는 전송방식과 무관하게 필요.
- **[결정 B] 클록 동기 vs 카메라 HW 동기 = 직교(orthogonal)**. 클록 동기(이더넷)는 "타임스탬프 라벨" 정렬(필수 기반), 9핀 HW 동기는 "두 카메라 노출 순간" 정렬. A=SLAM/B=단일카메라-SAM 구조에선 HW 동기는 정밀도 옵션(삼각측량 안 함 → SLAM 궤적 보간으로 흡수 가능).
- **[결정 C] LiDAR·IMU 촬영순간 하드웨어 정렬 = 미채택**. VLP-16은 셔터 트리거 자체가 없는 연속 스캐너(PPS discipline+Phase Lock만 가능, 그나마 PPS 소스 없음), IMU는 연속 고레이트 샘플러 → **정확한 타임스탬프 + 보간/디스큐가 정석**. 노트북은 깨끗한 HW PPS 생성 불가(USB-serial DTR 해킹 지터 ~125µs–1ms). 진짜 필요 시 MCU mimic-GNSS(±µs) 추가가 업그레이드 경로.
- **[결정 D] 카메라↔IMU 외부 케이블 동기 = 포기(사용자 확정)**. 각 카메라의 9핀 포트가 1개뿐이라 카메라↔카메라(A master/B slave) 동기에 이미 점유됨. IMU는 클록 동기 + SW 타임스탬프(SampleTimeFine)에 의존.

---

## Implementation Approaches and Technology Adoption

> 이 절은 님의 원 질문("A·B 각각 어떤 설정, 어떤 실행 파일")에 대한 **실행 가능한 구현 산출물**이다. IP는 예시(A=192.168.1.10, B=192.168.1.20)로 표기, 실제 값으로 치환.

### 0단계 — 사전 점검 & 공통 전제

```bash
# [A,B 공통] NIC 하드웨어 타임스탬프(PHC) 지원 여부 확인 — chrony/PTP 정확도 결정
ethtool -T <iface>     # 'PTP Hardware Clock: 1' + hardware-* 있으면 HW, 없으면 SW(일반 노트북)
```

- 두 노트북 **동일 ROS2 distro·동일 rmw**로 통일(Husky 주력 = Humble, 또는 Jazzy). rmw는 둘 다 CycloneDDS 또는 둘 다 FastDDS.
- 스위치는 신뢰 폐쇄망 가정. VLP-16 UDP는 가급적 A의 별도 NIC로 수신.

### 1단계 — 시간 동기화 설정 (이더넷, 필수 기반)

**노트북 A = 시간 마스터 (chrony 서버).** `/etc/chrony/chrony.conf`:
```conf
# 오프라인이면 상위 인터넷 소스 없이 자체 권위 소스로:
local stratum 10
# (인터넷 되면) pool 대신: server <upstream> iburst
allow 192.168.1.0/24            # B 서브넷에 시각 제공
hwtimestamp *                    # ethtool -T 가 HW 지원일 때만 (수십 ns), SW면 이 줄 생략
driftfile /var/lib/chrony/chrony.drift
rtcsync
makestep 1.0 3
```

**노트북 B = 시간 클라이언트.** `/etc/chrony/chrony.conf`:
```conf
server 192.168.1.10 iburst minpoll 2 maxpoll 4 xleave   # A를 시각원으로, 짧은 폴+interleaved
hwtimestamp *                    # B NIC가 HW 지원일 때만
driftfile /var/lib/chrony/chrony.drift
rtcsync
makestep 1.0 3
```

```bash
# [A,B] 적용·확인
sudo systemctl restart chronyd
chronyc tracking          # Last/RMS offset 확인 (SW면 ~수십µs, HW면 수십 ns 목표)
chronyc sources -v
chronyc ntpdata           # HW 타임스탬핑 실제 사용 여부
```

**(옵션) 두 NIC 모두 PHC 지원 + <100µs 필요 시 PTP:** `hwtimestamp *`를 chrony에 두는 게 가장 단순(권장). 별도 운용 시 `sudo ptp4l -f /etc/linuxptp/ptp4l.conf -i <iface> -H -m [-s]` + `sudo phc2sys -s <iface> -c CLOCK_REALTIME -w -m`.
_Source: https://chrony-project.org/examples.html , https://quantum5.ca/2023/01/26/microsecond-accurate-time-synchronization-lan-with-ptp/_

### 2단계 — 센서 드라이버 실행 (타임스탬프 상류 확정)

**노트북 A — RealSense D455f (Master):**
```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=cam_A \
  depth_module.global_time_enabled:=true \
  rgb_camera.global_time_enabled:=true \
  enable_sync:=true \
  depth_module.inter_cam_sync_mode:=1 \       # 1 = Master (트리거 방출)
  depth_module.profile:=848x480x30 \
  rgb_camera.profile:=848x480x30 \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2
```

**노트북 B — RealSense D455f (Slave):**
```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=cam_B \
  depth_module.global_time_enabled:=true \
  rgb_camera.global_time_enabled:=true \
  enable_sync:=true \
  depth_module.inter_cam_sync_mode:=2 \       # 2 = Slave(depth), 통합 RGB+depth 동시동기는 3(Full Slave)
  depth_module.profile:=848x480x30 \
  rgb_camera.profile:=848x480x30
```
- 9핀 배선: A Pin5(SYNC)↔B Pin5, A Pin9(GND)↔B Pin9. `global_time_enabled=true`가 두 카메라 stamp를 각 호스트 클록으로 올리고, chrony가 두 호스트를 맞춰 stamp가 공통 도메인이 됨.

**노트북 A — Velodyne VLP-16:**
```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
# 파라미터: gps_time:=false (PPS/GPS 없음 → 호스트 도착시각). time_offset:=0.0
```

**노트북 A — Xsens IMU (공식 파생 드라이버):**
```bash
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
# param/xsens_mti_node.yaml:
#   time_option: 1     # SampleTimeFine(HW clock) 기반 → 지터 최소(IMU preintegration 유리)
#                      #   (또는 2=host: A 호스트클록에 바로 얹힘, 가장 단순)
#   enable_outputConfig: true  # 최초 1회 SampleTimeFine/UTC 출력 활성화
```
- IMU는 A 로컬 SLAM에만 쓰이므로 A 클록 도메인에 있으면 충분. 결정 D로 카메라와의 HW 배선은 없음.

**노트북 A — Husky A200:**
```bash
# /etc/clearpath/robot.yaml (platform: a200) 기반 bringup
ros2 launch clearpath_robot platform.launch.py    # odom/imu 호스트 stamp, EKF는 platform/odom/filtered
```
_Source: https://dev.intelrealsense.com/docs/ros2-wrapper , https://github.com/ros-drivers/velodyne , https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client , https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/_

### 3단계 — 데이터 수집 (머신별 로컬 MCAP bag)

**QoS 오버라이드**(드롭 방지, `qos_override.yaml`):
```yaml
/velodyne_points:
  reliability: reliable
  history: keep_all
```

**노트북 A:**
```bash
ros2 bag record -o session/A/bag_A -s mcap \
  --max-cache-size 1073741824 \
  --qos-profile-overrides-path qos_override.yaml \
  /cam_A/color/image_raw /cam_A/aligned_depth_to_color/image_raw \
  /cam_A/color/camera_info /cam_A/imu \
  /velodyne_points /imu/data \
  /a200_XXXX/platform/odom /a200_XXXX/sensors/imu_0/data \
  /tf /tf_static
```

**노트북 B:**
```bash
ros2 bag record -o session/B/bag_B -s mcap \
  --max-cache-size 1073741824 \
  /cam_B/color/image_raw /cam_B/aligned_depth_to_color/image_raw \
  /cam_B/color/camera_info /cam_B/imu
```
- MCAP + 무압축/lz4(zstd 기본레벨은 CPU바운드 ~9.9MB/s로 throttle 위험). Humble이면 `rosbag2_storage_mcap` 설치 필요.
- **두 record를 거의 동시에 기동**(ssh 팬아웃/공유 시작신호). 클록 동기 상태면 log time이 정렬됨.
_Source: https://mcap.dev/guides/benchmarks/rosbag2-storage-plugins , https://github.com/ros2/rosbag2/issues/1430_

### 4단계 — (온라인 시) A→B pose/TF 전송 [결정 A]

오프라인 데이터셋이면 A의 `/tf`가 bag_A에 이미 기록되므로 **실시간 전송 불필요**. 실시간 온-B 매핑이 필요할 때만:
```bash
# 멀티캐스트가 스위치에서 막히면 FastDDS Discovery Server(유니캐스트)
# 노트북 A (서버 + 노드):
fastdds discovery -i 0 -l 192.168.1.10 -p 11811 &
export ROS_DISCOVERY_SERVER="192.168.1.10:11811"
# 노트북 B (클라이언트 노드):
export ROS_DISCOVERY_SERVER="192.168.1.10:11811"
# A가 /tf /tf_static /robot_description(transient_local) 발행 → B가 lookupTransform로 소비
```
_Source: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html_

### Testing and Quality Assurance (동기 품질 검증 — 실기 없이도 절차 확립)

1. **클록 오프셋 캡처 전후 로깅**(정량 감사, 논문 공통 관행):
   ```bash
   chronyc tracking > session/sync/clock_offset_pre.txt    # run 전
   # ... 수집 ...
   chronyc tracking > session/sync/clock_offset_post.txt   # run 후 → 잔여 스큐 정량
   ```
2. **드롭 검증:** `ros2 bag info session/A/bag_A` 의 메시지 수 ↔ `ros2 topic hz` 예상치 비교.
3. **공유 이벤트 크로스체크(GT-free):** 두 카메라가 같은 LED 점멸/박수를 관측 → 각 bag에서 그 순간의 stamp 차이 = 실제 크로스-노트북 오차.
4. **오프라인 정합 확인:** `ros2 bag play session/A/bag_A session/B/bag_B --clock` 후 `message_filters` ApproximateTime(slop≈½ 느린센서주기)로 매칭율 확인.
_Source: https://answers.ros.org/question/391514/detect-messages-dropped-by-ros2-bag-record/ , https://docs.ros.org/en/humble/p/message_filters/doc/index.html_

### Deployment and Operations — 디렉토리 레이아웃 & 재현성

```
session_YYYYMMDD_HHMM/
├── A/bag_A.mcap
├── B/bag_B.mcap
├── sync/clock_offset_pre.txt   sync/clock_offset_post.txt
├── calib/T_camB_camA.yaml       # ★ 두 카메라 외부 캘리브레이션 (SAM→SLAM맵 필수)
└── metadata.yaml                # 센서·해상도·sync모드·클록방식(chrony/PTP)·distro·rmw
```

## Technical Research Recommendations

### Implementation Roadmap (점진적)

1. **M1 — 시간베이스:** chrony 마스터/클라이언트 구성 + `chronyc tracking`으로 오프셋 확인(SW면 수십µs면 OK).
2. **M2 — 단일 노트북 검증:** A에서 전 센서 드라이버+bag record, 드롭 0 확인(대역폭 예산 검증).
3. **M3 — 카메라 쌍:** A/B D455f global_time + 9핀 master/slave, 공유이벤트로 크로스 오프셋 측정.
4. **M4 — 외부 캘리브레이션:** `T_camB_camA` 산출(체커보드/공통 특징) — SAM 객체를 SLAM 맵으로 옮기는 필수 항목.
5. **M5 — 오프라인 융합:** `T_map_obj = T_map_camA(t)·T_camA_camB·T_camB_obj(t)` 파이프라인, 기존 objectmemory 확장.
6. **M6(옵션) — 정밀화:** Husky 고속에서 오차 크면 MCU mimic-GNSS로 하드웨어 캡처 동기 추가.

### Technology Stack Recommendations

- 시간: **chrony**(기본), NIC PHC 있으면 `hwtimestamp *`/PTP. 저장: **MCAP** 무압축/lz4. rmw: 통일. 정합: **message_filters ApproximateTime**.

### Risk Assessment and Mitigation

| 리스크 | 완화 |
|---|---|
| 일반 노트북 NIC PHC 없음 → µs급 한계 | chrony로 수십µs 확보(센서 수십ms 케이던스엔 충분) |
| USB3 컨트롤러 공유 프레임드롭 | 카메라마다 독립 컨트롤러 |
| zstd throttle/디스크 QLC | 무압축/lz4 + TLC NVMe + 캐시↑ |
| 멀티캐스트 디스커버리 차단 | Discovery Server 유니캐스트 |
| **T_camB_camA 누락 시 SAM→맵 불가** | M4 외부 캘리브레이션 선결 |

### Success Metrics and KPIs

- 크로스-노트북 클록 오프셋(공유이벤트) < slop, bag 드롭율 0%, message_filters 매칭율 ≥ 목표, 정적 객체 `T_map_obj` 반복오차(맵 상 cm급).

---

## Research Synthesis — 2대 노트북 ROS2 멀티센서 동기 수집: 종합 기술 리서치

### Executive Summary

이더넷 스위치로 연결된 두 노트북에서 timestamp 동기화된 ROS2 데이터를 수집하는 문제의 본질은 **"하나의 동기화"가 아니라 서로 다른 것을 정렬하는 직교(orthogonal) 계층들의 조합**이다. 가장 흔한 오해 — "이더넷으로 클록을 맞추면 모든 센서가 동기된다" — 는 부분적으로만 옳다. 이더넷 클록 동기(chrony/PTP)는 두 노트북이 "지금 몇 시인지"에 합의하게 하여 **타임스탬프 라벨을 공통 도메인에 올리는 필수 기반**이지만, 그것이 두 카메라가 같은 순간에 노출하도록(9핀 HW 동기) 만들지는 못한다. 다행히 본 리서치의 대상 파이프라인(A=SLAM, B=단일카메라 SAM-6D)은 두 카메라로 같은 점을 동시 삼각측량하지 않으므로, 클록 동기 + SLAM 궤적 보간만으로도 `T_map_obj` 합성이 가능하며 HW 동기는 정밀화 옵션에 머문다.

수집 아키텍처 측면에서는 **"머신별 로컬 MCAP bag + 클록 동기 + 사후 병합"** 이 명확한 정답이다. 이는 실제 분산 SLAM 데이터셋 논문들(FusionPortableV2, S3E, SMapper, Race-Against-the-Machine 등)이 예외 없이 채택한 패턴으로, 단일 DDS 도메인 통합 수집 대비 (i) 캡처 중 네트워크·대역폭·디스커버리 의존이 없어 신뢰성이 높고, (ii) D455f 원본 스트림(~60MB/s)이 1GbE를 포화시키는 위험을 회피한다. ROS2에 1급 분산 레코더가 아직 없다는 사실(rosbag2 #1548)이 이 선택을 더욱 강제한다. 대역폭 예산은 A 총합 ~68–71MB/s로 TLC NVMe에 여유롭게 들어가나, USB3 컨트롤러 카메라별 분리·zstd 회피·QLC 매체 주의가 실무 관건이다.

결론적으로 권장 구성은: **chrony(A=서버/B=클라이언트)로 시스템 클록 정렬 → 각 센서 드라이버에서 타임스탬프를 상류 확정(D455f global_time_enabled, Xsens SampleTimeFine) → 두 D455f를 9핀 master/slave로 노출 정렬 → A/B 각각 로컬 MCAP bag 기록(무압축/lz4, 캐시 상향, QoS reliable) → 캡처 전후 클록 오프셋 로깅 → 오프라인에서 message_filters ApproximateTime + LiDAR 디스큐로 융합**이다. 유일하게 남는 필수 선결 항목은 두 카메라 간 외부 캘리브레이션(`T_camB_camA`)으로, 이것 없이는 B의 객체 pose를 A의 맵으로 옮길 수 없다.

**Key Technical Findings:**

- **동기의 3계층 직교성:** ① 클록 동기(이더넷/PTP/chrony, 필수) ② 카메라 노출 HW 동기(9핀, 선택) ③ LiDAR/IMU 타임스탬프+보간/디스큐(정석). 하나가 다른 하나를 대체하지 못함.
- **일반 노트북 정확도 현실:** NIC PHC(하드웨어 타임스탬프) 없으면 PTP 이점 소멸 → chrony로 수십µs 달성이 현실적이며, 센서 수십ms 케이던스엔 충분.
- **아키텍처:** 머신별 로컬 수집이 표준·권장(옵션 a). 단일 DDS 통합(옵션 b)은 대역폭·디스커버리 리스크로 저레이트/단일파일 요건에 한정.
- **A→B pose/TF 전송:** ROS2/DDS 네이티브가 raw 이더넷 소켓보다 우월(payload tiny, tf2 시간보간, transient_local latching). 오프라인 융합이면 실시간 전송 자체가 불필요.
- **LiDAR/IMU 하드웨어 캡처 동기:** 현재 키트만으론 불가(GPS·sync보드 없음, 노트북 PPS 생성 불가)하나, 연속 샘플러 특성상 불필요. 진짜 필요 시 MCU mimic-GNSS(±µs)가 업그레이드 경로.
- **필수 선결:** `T_camB_camA` 외부 캘리브레이션 — SAM 객체를 SLAM 맵으로 변환하는 데 시간동기 못지않게 결정적.

**Technical Recommendations (Top 5):**

1. **chrony 기반 클록 동기부터 구축**(A=서버, B=클라이언트). `ethtool -T`로 PHC 확인 후 있으면 `hwtimestamp *`, 없으면 SW로 수십µs 수용.
2. **머신별 로컬 MCAP bag 수집** — 무압축/lz4, `--max-cache-size` 상향, QoS reliable/keep_all. 원본 스트림은 절대 네트워크로 흘리지 않음.
3. **타임스탬프 상류 확정** — D455f `global_time_enabled=true` + 9핀 master(A)/slave(B), Xsens `time_option` HW 클록, 캡처 전후 `chronyc tracking` 오프셋 로깅.
4. **A→B는 ROS2/DDS**(필요 시 Discovery Server 유니캐스트). 오프라인 데이터셋이면 `/tf`를 bag_A에 기록해 실시간 전송 생략.
5. **`T_camB_camA` 외부 캘리브레이션을 M4 필수 마일스톤으로** 선결 — 이후 objectmemory 파이프라인을 멀티-노트북으로 확장.

### Table of Contents

1. Research Overview (문서 상단)
2. Technical Research Scope Confirmation — 범위·전제
3. Technology Stack Analysis — 센서 드라이버/시간동기/rosbag2/DDS 4계층
4. Integration Patterns Analysis — 수집 아키텍처·message_filters·재생/병합·SLAM×SAM 흐름
5. Architectural Patterns and Design — 권장 전체 구조·대역폭 배치
6. 설계 결정 로그 (Q&A 확정: A~D)
7. Implementation Approaches — 설정 파일·실행 커맨드·검증·로드맵
8. Research Synthesis (본 절) — 종합·전략 권고·결론

### 종합 서론 — 왜 이 문제가 어려운가

멀티센서 로봇 데이터 수집에서 "동기화"라는 단어는 최소 세 가지 서로 다른 것을 뭉뚱그린다. 두 컴퓨터가 같은 시각에 합의하는 것(클록 동기), 두 센서가 같은 물리적 순간에 측정하는 것(캡처 동기), 그리고 서로 다른 시각에 찍힌 데이터를 사후에 엮는 것(타임스탬프 정합). 이 세 가지는 각각 다른 메커니즘·다른 하드웨어·다른 정확도를 가지며, 실제 rig 설계의 대부분의 혼란은 이들을 구분하지 못하는 데서 온다. 본 리서치의 가장 큰 기여는 이 세 계층을 명확히 분리하고, 주어진 하드웨어(GPS 없음, 일반 노트북, 소비자 스위치)에서 각 계층이 무엇을 할 수 있고 무엇을 할 수 없는지를 소스 기반으로 확정한 것이다.

특히 카메라와 LiDAR·IMU의 근본적 차이 — 카메라는 이산적 셔터 순간을 갖지만 LiDAR는 연속 회전 스캐너이고 IMU는 연속 고레이트 샘플러 — 를 이해하면, "왜 카메라는 9핀으로 하드웨어 동기하는데 LiDAR·IMU는 안 하는가"라는 질문이 자연스럽게 풀린다. 답은 "할 수 없어서"가 아니라 "그 센서들에겐 타임스탬프+보간이 옳은 방법이라서"이다.

### 성능·대역폭 분석 (종합)

- **USB3:** D455f 각 ~49–61MB/s(~390–490Mbps). 컨트롤러 공유 시 프레임드롭 → 카메라별 독립 컨트롤러 필수.
- **네트워크:** 시간·제어만 나르면 1GbE(~118MB/s 실효)에 여유 大. 원본 스트림 전송은 절대 회피.
- **디스크:** A 총합 ~68–71MB/s → TLC NVMe(>500MB/s) 여유. QLC/SD/장시간 SLC 소진 시 병목.
- **CPU/DDS:** zstd 기본레벨 ~9.9MB/s throttle → 무압축/lz4. 캐시 상향으로 write-stall 버스트 흡수.

### 리스크 평가 및 완화 (종합)

| 리스크 | 영향 | 완화 |
|---|---|---|
| 노트북 NIC PHC 부재 | 동기 정확도 µs급 상한 | chrony 수십µs 수용(센서 케이던스 충분), 필요시 PHC NIC/PTP |
| USB3 컨트롤러 공유 | 프레임 드롭 | 카메라별 독립 컨트롤러 |
| 저장 압축/매체 | 기록 throttle·드롭 | MCAP 무압축/lz4, TLC NVMe, 캐시↑ |
| 멀티캐스트 디스커버리 차단 | A↔B 통신 두절 | FastDDS Discovery Server / CycloneDDS Peers 유니캐스트 |
| **T_camB_camA 누락** | **SAM→맵 변환 불가** | M4 외부 캘리브레이션 선결 |
| Husky 고속 이동 시 보간 잔차 | 객체 위치 오차 | (옵션) MCU mimic-GNSS 하드웨어 캡처 동기 |

### 향후 전망 및 확장

- **근기(즉시):** 본 문서의 M1–M5 로드맵으로 실기 도착 시 즉시 구축 가능. 실기 검증에서 크로스-노트북 오프셋·드롭율·매칭율을 KPI로 측정.
- **중기:** objectmemory 파이프라인을 멀티-노트북·멀티-객체로 확장(기존 Story 자산 재사용).
- **정밀 확장:** Husky 고속 시나리오·삼각측량 요구 발생 시 MCU grandmaster(mimic-GNSS)로 전 센서 ±µs 하드웨어 캡처 동기 도입.

### 리서치 방법론 및 소스 검증

- **방법:** 실기 검증 불가 제약 하에 7개 병렬 리서치 서브에이전트로 (센서 드라이버, PTP/chrony, rosbag2/대역폭, 멀티머신 DDS, VLP-16 PPS, Xsens sync, mimic-GNSS) 공식문서·논문·GitHub·커뮤니티를 교차조사.
- **소스 등급:** 1차 공식문서(Intel RealSense, Velodyne 매뉴얼, Movella/Xsens 데이터시트, Clearpath, chrony/linuxptp, ROS2 docs) + 논문(arXiv 데이터셋/동기화) + 실측 벤치(chrony examples, Quantum5, mcap.dev).
- **불확실성 표기:** [FACT]/[calc]/[uncertain] 구분. 미검증 항목(특정 Xsens 모델 sync 스펙, 노트북별 PHC 유무, distro별 --clock 동작)은 실기에서 확인 필요로 명시.
- **한계:** 모든 정량치는 사양·타 하드웨어 실측 기반 추정이며, 본 rig 실측 아님.

### 결론 — 다음 액션 제안

본 리서치로 "A·B 각각 어떤 설정, 어떤 실행 파일로 동기화된 ROS2 bag을 수집하는가"에 대한 실행 가능한 답과 그 근거·리스크가 확립되었다. 실기 도착 전까지 준비할 수 있는 다음 액션은:

- **[선택지 1] 구현 스펙 산출물화** — 본 문서의 설정/커맨드를 실제 launch 파일·chrony.conf·record 스크립트·`metadata.yaml` 템플릿으로 리포지토리에 커밋(실기 도착 즉시 실행 가능한 형태).
- **[선택지 2] 외부 캘리브레이션 절차 상세화** — `T_camB_camA` 산출 파이프라인(체커보드/공통특징, ROS2 도구)을 별도 스펙으로 심화(현재 최대 미해결 리스크).
- **[선택지 3] 실기 검증 체크리스트 작성** — `ethtool -T` 결과 분기, 공유이벤트 오프셋 측정, 드롭율 KPI를 담은 M1–M3 검증 런북.
- **[선택지 4] MCU mimic-GNSS 설계 조사** — Husky 고속/삼각측량 대비 하드웨어 캡처 동기 업그레이드를 별도 리서치로.

**질문:** 위 중 어느 방향으로 진행할까요? 아니면 본 리서치 문서를 이대로 확정하고 별도 구현 스펙(BMAD implementation-artifacts)으로 넘어갈까요?

---

**Technical Research Completion Date:** 2026-07-07
**Research Period:** 현재 시점 공식문서·논문·커뮤니티 종합 (실기 미검증)
**Source Verification:** 모든 핵심 주장 다중 소스 교차검증, [FACT]/[calc]/[uncertain] 구분
**Technical Confidence Level:** High (문서화 근거), 단 정량치는 사양기반 추정·실기 검증 권장

_본 문서는 이더넷 스위치 연결 2대 노트북 ROS2 멀티센서 timestamp 동기 수집에 대한 권위 있는 기술 참조 문서이며, SLAM+SAM 통합 데이터 수집의 설계·구현 의사결정 기반을 제공한다._
