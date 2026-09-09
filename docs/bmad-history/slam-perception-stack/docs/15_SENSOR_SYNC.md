# 15. 2대 노트북 동기 수집 (`sensor_sync_ws`)

SLAM + SAM 동시 수집을 위한 **타임스탬프 동기 데이터 수집 킷**.
colcon 패키지가 **아니고** 빌드가 필요 없는 loose 실행 파일 세트다.

> 대상 환경이 다른 워크스페이스와 다르다: **Ubuntu 24.04 + ROS 2 Jazzy**
> (다른 5개는 Humble). apt로 설치한 `realsense2_camera` / `velodyne` 를 참조한다.

## 1. 설계 원칙 — 동기화는 직교하는 3계층이다

| 계층 | 필수/선택 | 내용 |
|---|---|---|
| 1. 클록 동기 | **필수** | 두 머신의 시계를 맞춘다 (chrony/PTP). 이게 안 되면 나머지가 무의미 |
| 2. 카메라 하드웨어 동기 | 선택 | D435i 9핀 커넥터로 셔터 동기 |
| 3. LiDAR·IMU | — | 하드웨어 동기가 불가하므로 **타임스탬프 + 보간**으로 처리 |

수집 방식은 **각 머신 로컬 MCAP 기록 → 사후 병합**이 표준이다
(네트워크 실시간 전송은 드롭·지터로 데이터를 망친다).

**최대 리스크는 `T_camB_camA`(두 카메라 간 extrinsic) 캘리브레이션이다.**
실제로 260714 데이터셋에서 이걸 안 해서 객체 맵이 흩어졌다 → `docs/14_OBJECTMEMORY.md` 참고.

## 2. 구성

```
sensor_sync_ws/
├─ launch/bringup_A.launch.py    # D435i + VLP-16 통합 기동
├─ config/realsense_A.yaml       # global_time 등 realsense 파라미터
├─ config/qos_override.yaml      # 기록 드롭 방지 QoS
├─ scripts/check_env.sh          # V0: OS/ROS/NIC PHC 점검
├─ scripts/record_A.sh           # V3/V4: MCAP 기록
├─ scripts/check_global_time.sh  # V1/V6: stamp vs 시스템시각 확인
├─ scripts/verify_drops.sh       # V4: bag info 드롭 점검
├─ time_sync/chrony_A_server.conf / chrony_B_client.conf
└─ metadata_template.yaml
```

설계 런북(V0~V8 검증 절차)은
`_bmad_output_for_slam/implementation-artifacts/blueprint-sensor-sync-validation-jazzy.md`.

## 3. 실행

```bash
source /opt/ros/jazzy/setup.bash            # ← 이 워크스페이스만 시스템 ROS Jazzy 사용

bash scripts/check_env.sh <iface>           # 0) 환경 점검 (실제 이더넷 인터페이스명)
ros2 launch launch/bringup_A.launch.py      # 1) 센서 기동 (별도 터미널)

ros2 topic hz /cam_A/color/image_raw        # 2) 확인
ros2 topic hz /velodyne_points
bash scripts/check_global_time.sh

bash scripts/record_A.sh                    # 3) 기록 (5~10분)
bash scripts/verify_drops.sh session/<bag>  # 4) 드롭 점검
```

2대 구성 시 `time_sync/chrony_A_server.conf`(마스터) / `chrony_B_client.conf`(슬레이브)를
각각 배치한 뒤 `chronyc tracking` 으로 오프셋을 확인한다.

## 4. 현장에서 확인해야 하는 항목 (`# TODO(실기)` 표시)

- **realsense 파라미터명이 realsense-ros 버전마다 다르다** (`depth_profile` vs `profile`).
  `ros2 param list /cam_A/camera` 로 실제 이름을 확인한 뒤 `config/realsense_A.yaml` 을 고칠 것.
- **VLP-16 IP/서브넷**(기본 192.168.1.201)을 웹UI로 확인하고 노트북 IP를 맞출 것.
- **토픽명**(`aligned_depth_to_color` 등) 실제 발행명 확인.

## 5. 기존 데이터셋과의 관계

`data_slam/0724_chungbuk` 의 RGB-D / IMU / LiDAR 3센서는 **이미 같은 epoch 시계를 쓰고 있음이
확인**됐다(추가 오프셋 보정 불필요). 이 킷은 그 조건을 새 수집에서도 보장하기 위한 것이다.
