# 설계도: 멀티센서 timestamp 동기 수집 — 단일머신 검증 (ROS2 Jazzy)

> **이 파일의 용도:** PC에서 설계 → 노트북으로 이관 → 노트북에서 이 문서를 따라 **실질 검증 개발**을 진행하기 위한 자기완결적 런북(runbook).
> **대상 환경:** Ubuntu 24.04 (Noble) + **ROS2 Jazzy Jalisco**
> **작성일:** 2026-07-07 · **상태:** 실기 미검증 초안(노트북에서 검증하며 결과 채움)
> **상위 리서치:** `planning-artifacts/research/technical-ros2-multilaptop-sensor-timestamp-sync-research-2026-07-07.md`

---

## 0. 컨텍스트 (왜 이 검증을 하는가)

### 최종 목표
이더넷 스위치로 노트북 A·B 연결. A = D455f(master)+VLP-16(LAN)+Husky(USB)+Xsens IMU, B = D455f(slave, 9핀으로 A와 연결). timestamp 동기된 ROS2 bag을 각 머신 로컬 수집 → 오프라인에서 SLAM(A) × SAM-6D(B) 융합 → **맵 상 객체 위치** `T_map_obj = T_map_camA(t)·T_camA_camB·T_camB_obj(t)`.

### 이 문서의 범위 = "현재 회사(단일 노트북)" 검증
현재 가용: **노트북 1대 + D435i + VLP-16**. → 최종 "노트북 A"에서 Husky/Xsens/2번째 카메라를 뺀 **단일머신 로컬 수집 파이프라인 전체**를 미리 검증한다. **D435i는 D455f의 소프트웨어/대역폭 대역**(동일 `realsense2_camera` 드라이버·파라미터·IMU 내장)이다.

### 지금 검증 가능 / 연기
| 검증 가능(이 문서) | 연기(2대 만난 뒤) |
|---|---|
| 드라이버 설치·기동, global_time 동작, VLP-16 네트워크, MCAP 기록, **대역폭/드롭**, NIC PHC 판정, 오프라인 융합 툴링 | A↔B 클록동기, 카메라 9핀 HW동기, Husky 통합, `T_camB_camA` 캘리브 |

### ⚠️ 하드웨어 갭 (기억)
- 최종엔 **D455f 2대** 필요, 현재 두 사이트 합쳐 D455f 1대뿐 → **1대 추가 확보 필요**.
- 카메라 9핀 HW 동기는 카메라 2대가 있어야 검증됨(본 문서 범위 밖).

---

## 1. 환경 셋업 (Jazzy)

### 1.1 OS/ROS2
```bash
# Ubuntu 24.04 (Noble) 확인
lsb_release -a            # Noble 24.04 여야 Jazzy 설치 가능

# ROS2 Jazzy 설치 (ros-jazzy-desktop) — docs.ros.org/en/jazzy 절차
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 1.2 패키지
```bash
sudo apt update
# RealSense (D435i/D455f 공용 드라이버)
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
#   ↑ Jazzy 바이너리 미제공/구버전이면 소스빌드: realsense-ros ros2-development 브랜치 + librealsense2
# Velodyne VLP-16
sudo apt install ros-jazzy-velodyne
# 유틸
sudo apt install ros-jazzy-message-filters chrony ethtool
# (Jazzy는 rosbag2 기본 저장포맷이 MCAP → 별도 플러그인 설치 불필요)
```

### 1.3 rmw / 디스커버리 (단일머신이라 최소)
```bash
# Jazzy 기본 rmw = FastDDS. 최종 2대 통합 시 rmw 통일이 중요하므로 지금 선택 고정 권장.
# (옵션) CycloneDDS 사용 시:
# sudo apt install ros-jazzy-rmw-cyclonedds-cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 단일머신 격리 (Jazzy 신규)
```
> **결정 필요:** 최종에 쓸 rmw(FastDDS vs CycloneDDS)를 지금 고정. 이 문서는 FastDDS 기본 가정.

---

## 2. 하드웨어 토폴로지 (현재 사이트)

```
     ┌──────────── Dumb Gigabit Switch ────────────┐
     │                                             │
   노트북(D435i USB3, VLP-16 데이터 수신)        VLP-16 (interface box, LAN)
   192.168.1.10                                  192.168.1.201 (기본, 웹UI로 확인)
```
- **IP 계획:** 노트북 `192.168.1.10/24`, VLP-16 기본 `192.168.1.201`(매뉴얼/웹UI로 실제값 확인 후 서브넷 확정).
- 스위치는 **더미 Gigabit** 권장(매니지드면 IGMP snooping이 향후 DDS 디스커버리 방해 가능).
- D435i는 **USB3 포트**(가급적 단독 컨트롤러)에 연결.

---

## 3. 검증 태스크 (V0–V8) — 각 태스크: 목표 / 실행 / 합격기준

### V0. NIC PHC 판정 ★ (최종 시간동기 경로 분기점)
- **목표:** 노트북 NIC가 하드웨어 타임스탬프(PHC)를 지원하는지 → chrony(SW) vs PTP(HW) 결정.
- **실행:** `ethtool -T <iface>`
- **합격기준:** 결과 기록. `PTP Hardware Clock: 1` + `hardware-*` 있으면 HW 가능, 없으면(대부분 노트북) chrony-SW로 수십µs 경로 확정.

### V1. RealSense D435i 기동 + global_time 확인
- **목표:** 드라이버 기동, header.stamp가 host epoch 도메인인지.
- **실행:**
```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=cam_A \
  depth_module.global_time_enabled:=true \
  rgb_camera.global_time_enabled:=true \
  enable_sync:=true \
  depth_module.depth_profile:=848x480x30 \
  rgb_camera.color_profile:=848x480x30 \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2
# 확인
ros2 topic hz /cam_A/color/image_raw
ros2 topic echo /cam_A/color/image_raw --field header.stamp --once
date +%s.%N
```
- **합격기준:** ~30Hz 유지, `header.stamp` 초 값이 `date` 시스템시각과 근접(같은 epoch 도메인) → global_time 동작. IMU 토픽(`/cam_A/imu`) 발행 확인.
> 참고: Jazzy realsense-ros 파라미터명은 버전에 따라 `depth_module.profile` vs `depth_module.depth_profile` 차이 가능 → `ros2 param list`로 실제명 확인 후 조정.

### V2. VLP-16 네트워크 + 드라이버
- **목표:** 스위치 경유 VLP-16 데이터 수신.
- **실행:**
```bash
# NIC 고정 IP 192.168.1.10/24 설정 후
ping 192.168.1.201                 # VLP-16 응답 확인
# 웹UI(http://192.168.1.201)에서 IP/데이터 목적지 확인
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
#   params: gps_time:=false, time_offset:=0.0
ros2 topic hz /velodyne_points
```
- **합격기준:** `ping` OK, `/velodyne_points` ~10Hz PointCloud2 수신, rviz2로 점군 시각화.

### V3. 통합 동시 수집 (MCAP)
- **목표:** 카메라+LiDAR 동시 기록.
- **실행:** (`qos_override.yaml`은 §5 부록)
```bash
ros2 bag record -o session/val_A \
  --max-cache-size 1073741824 \
  --qos-profile-overrides-path qos_override.yaml \
  /cam_A/color/image_raw /cam_A/aligned_depth_to_color/image_raw \
  /cam_A/color/camera_info /cam_A/imu \
  /velodyne_points
#   Jazzy 기본 저장포맷 = mcap (-s mcap 생략 가능, 명시해도 무방)
```
- **합격기준:** bag 생성, `ros2 bag info session/val_A`에 모든 토픽 존재.

### V4. ★대역폭/드롭 검증 (M2 핵심)
- **목표:** 목표 해상도에서 드롭 0 확인 = 최종 노트북 A 대역폭 예산 검증.
- **실행:** 5~10분 기록 후
```bash
# 예상 메시지수(rate×시간) ↔ 실제 비교
ros2 bag info session/val_A          # 각 토픽 Count
# 사전에 ros2 topic hz로 실제 rate 측정해 둘 것
iostat -x 5                          # 기록 중 디스크 %util (병목 확인)
```
- **합격기준:** color/depth ≈ 30Hz×기록초, velodyne ≈ 10Hz×기록초, **드롭율 <1%**. 디스크 %util 여유. 장시간(30분+)에도 드롭 유지 시 SLC소진/QLC 병목 없음 확인.
- **실패 시:** 해상도/프레임 하향, 압축(V5), 캐시 상향, USB 컨트롤러 분리.

### V5. 압축/저장 튜닝
- **목표:** 무압축 vs lz4 vs zstd 처리량·드롭 비교.
- **실행:** 각 모드로 V3 반복 (MCAP 압축은 record 시 storage 옵션/변환으로).
- **합격기준:** 무압축 또는 lz4에서 드롭 0. **zstd 기본레벨은 CPU바운드(~10MB/s)로 throttle 예상** → 회피 확정.

### V6. global_time 드리프트 관찰
- **목표:** 장시간 기록에서 카메라 stamp가 시스템시각 대비 드리프트 없는지(global_time 회귀 동작).
- **실행:** 10분 기록 중 주기적으로 `header.stamp - date` 차이 로깅.
- **합격기준:** 차이가 시간에 따라 발산하지 않음.

### V7. 오프라인 융합 툴링 (message_filters + 디스큐)
- **목표:** 최종 융합 코드경로 선검증.
- **실행:**
```bash
ros2 bag play session/val_A --clock
# 별도 노드: message_filters ApproximateTimeSynchronizer(
#   [/cam_A/color/image_raw, /velodyne_points], queue=10, slop=0.05)
# 콜백에서 동시프레임 수신 확인 + LiDAR 디스큐(IMU/odom 보간) 테스트
```
- **합격기준:** 동기 콜백이 안정적으로 발생, slop 조정으로 매칭율 변화 관찰. 디스큐 전/후 점군 왜곡 감소 확인.

### V8. 구현 산출물 확정
- **목표:** 위에서 검증된 파라미터로 launch/params/record 스크립트를 리포에 커밋(최종 A 노트북 이식용).
- **합격기준:** 스크립트만으로 V1–V4 재현.

---

## 4. 검증 결과 기록표 (노트북에서 채움)

| 태스크 | 결과 | 측정값/비고 |
|---|---|---|
| V0 NIC PHC | ☐ HW ☐ SW | `ethtool -T` 출력: |
| V1 realsense global_time | ☐ pass ☐ fail | stamp vs date 차이: |
| V2 VLP-16 수신 | ☐ pass ☐ fail | rate: |
| V3 통합기록 | ☐ pass ☐ fail | |
| V4 드롭율 | ☐ <1% ☐ 초과 | color/depth/velodyne count: , 디스크%util: |
| V5 압축 | 확정: ☐ none ☐ lz4 | zstd throttle 여부: |
| V6 드리프트 | ☐ 안정 ☐ 발산 | |
| V7 융합툴 | ☐ pass ☐ fail | slop: , 매칭율: |
| 최종 rmw 결정 | ☐ FastDDS ☐ Cyclone | |

---

## 5. 부록 — 설정 파일 초안 (노트북에서 그대로 사용)

### 5.1 `qos_override.yaml`
```yaml
/velodyne_points:
  reliability: reliable
  history: keep_all
  depth: 100
```

### 5.2 `chrony_A_server.conf` (2대 통합 시 A=마스터용, 지금은 미사용·준비만)
```conf
local stratum 10
allow 192.168.1.0/24
# hwtimestamp *        # V0에서 PHC 확인되면 주석 해제
driftfile /var/lib/chrony/chrony.drift
rtcsync
makestep 1.0 3
```

### 5.3 `chrony_B_client.conf` (2대 통합 시 B=클라이언트용, 준비만)
```conf
server 192.168.1.10 iburst minpoll 2 maxpoll 4 xleave
# hwtimestamp *
driftfile /var/lib/chrony/chrony.drift
rtcsync
makestep 1.0 3
```

### 5.4 `record_A.sh`
```bash
#!/usr/bin/env bash
set -e
SESSION="session/$(hostname)_$(date +%Y%m%d_%H%M%S)"   # 스크립트 실행시각(호스트)
ros2 bag record -o "$SESSION" \
  --max-cache-size 1073741824 \
  --qos-profile-overrides-path "$(dirname "$0")/qos_override.yaml" \
  /cam_A/color/image_raw /cam_A/aligned_depth_to_color/image_raw \
  /cam_A/color/camera_info /cam_A/imu \
  /velodyne_points
```

### 5.5 디렉토리 레이아웃
```
session_YYYYMMDD_HHMM/
├── val_A.mcap
├── sync/clock_offset_pre.txt   post.txt   # 2대 통합 시
└── metadata.yaml                # 센서·해상도·sync모드·distro(Jazzy)·rmw·global_time
```

---

## 6. Jazzy 전환에 따른 유의점 (Humble 대비)

- **Ubuntu 24.04 필수** (22.04면 Jazzy 불가).
- **rosbag2 기본 = MCAP** → `rosbag2_storage_mcap` 별도설치 불필요.
- **realsense-ros Jazzy 바이너리**가 지연될 수 있음 → 없으면 소스빌드(librealsense2 + realsense-ros ros2-development).
- **Clearpath Husky는 Humble이 주력, Jazzy는 업그레이드 대상**(신규·검증 적음). Husky는 최종 통합 단계 항목이므로 그때 Jazzy 지원 상태 재확인.
- **Xsens 공식 드라이버(ros2)** Jazzy 빌드 여부 → 다른쪽에 확인 요청.
- 디스커버리: Jazzy `ROS_STATIC_PEERS`/`ROS_AUTOMATIC_DISCOVERY_RANGE` 활용 가능(2대 통합 시).

---

## 7. 다른쪽(2주 뒤)에 부탁 — 환경 패리티 (요약)
1. **동일 Jazzy + Ubuntu 24.04 + 동일 rmw** + realsense2_camera + Xsens 공식 드라이버.
2. D455f **9핀 커넥터 실물 확인 + sync 케이블 지참**.
3. D455f global_time 검증 + `inter_cam_sync_mode` 파라미터 확인.
4. Xsens `time_option`/SampleTimeFine 출력 + 짧은 bag 샘플.
5. 그쪽 노트북 `ethtool -T`(PHC) + NVMe/USB/NIC 스펙 공유.

---

## 8. 다음 액션 제안

- **[선택지 1]** 이 설계도를 노트북에 이관해 **V0부터 순차 검증** 시작 → 결과표 채우며 파라미터 확정.
- **[선택지 2]** 검증 전에 **launch/params 실제 파일**(`bringup_A.launch.py`, `realsense_A.yaml` 등)까지 이 리포에 먼저 생성해 함께 이관.
- **[선택지 3]** 2대 통합 대비 **두 번째 리눅스 머신 확보** 후 chrony/DDS 크로스머신 검증을 이 문서에 V9로 추가.
- **[선택지 4]** 다른쪽 부탁 사항을 **별도 요청서(.md)** 로 분리해 전달.

**질문:** 노트북 이관 후 바로 검증(1)로 갈까요, 아니면 실제 launch/params 파일까지 만들어(2) 함께 넘길까요?
