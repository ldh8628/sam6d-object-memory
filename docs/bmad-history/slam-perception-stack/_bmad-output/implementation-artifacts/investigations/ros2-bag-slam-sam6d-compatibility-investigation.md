# Investigation: ROS 2 bag 기반 SLAM·SAM-6D 호환성

## Hand-off Brief

1. **무엇을 조사하는가.** 사용자가 제공하는 ROS 2 bag 데이터만으로 ORB-SLAM3, RTAB-Map, HDL-Graph-SLAM, SAM-6D를 실행할 수 있는지 검증한다.
2. **현재 상태.** 저장소 문서에서 네 파이프라인이 서로 다른 센서 입력과 bag 외부 자산을 요구한다는 첫 확정 증거를 확보했으며, 실제 bag은 아직 제공되지 않았다.
3. **다음에 필요한 것.** 알고리즘별 런치·설정·토픽 계약을 조사한 뒤 실제 bag의 메타데이터와 비교해야 한다.

## Case Info

| Field            | Value |
| ---------------- | ----- |
| Ticket           | N/A |
| Date opened      | 2026-07-30 |
| Status           | Active |
| System           | Ubuntu 22.04.5 LTS, Linux 6.8.0-136-generic x86_64 |
| Evidence sources | 저장소 문서·소스·설정·버전 관리; 실제 ROS 2 bag은 미제공 |

## Problem Statement

사용자 요청: “ros2 bag 데이터만 주어주면 orb3-slam, rtab-map, hdl-graph-slam, sam-6d를 돌릴 수 있는지 검증”

검증 대상은 bag 하나가 네 알고리즘의 필수 센서 토픽, 메시지 타입, TF, CameraInfo/캘리브레이션, 시간 동기 조건을 만족하는지와 bag 외부 런타임 자산·환경이 준비되어 있는지 여부다.

## Evidence Inventory

| Source | Status | Notes |
| ------ | ------ | ----- |
| `README.md` | Available | 스택 구성, 센서 데이터 흐름, 외부 자산 요구사항 |
| 알고리즘별 `docs/1x_*.md` | Available | 실행·빌드·함정 문서; 세부 조사 대기 |
| 워크스페이스 소스·런치·설정 | Available | 실제 입력 계약 조사 대기 |
| Git 저장소 | Available | HEAD `1f221f6`; 사용자 설치로 생긴 추적되지 않은 BMad 관련 파일 존재 |
| 실제 ROS 2 bag | Available | `data_slam/0729/`의 SQLite3 bag 3개; 메타데이터·SQLite·첫 메시지 CDR 분석 완료 |
| 실제 실행 로그·출력 | Missing | end-to-end 실행 성공 여부 미검증 |
| 설치 상태 정적 점검 | Available | `scripts/verify_setup.sh` 실행; 빌드·대용량 자산·경로 상태 확인 |
| 기존 테스트 결과 | Missing | 현재 worktree에 `latest_test`/`test_results` 증거 없음 |
| 이슈 트래커 | Missing | 이번 로컬 데이터 호환성 판정에는 직접 관련 없음 |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | 데이터셋 포맷과 bag 메타데이터 조사 | High | Done | 세 bag의 metadata·SQLite·첫 메시지 분석 |
| 2 | ORB-SLAM3 입력·캘리브레이션·자산 계약 추적 | High | Done | RGB-D/IMU, vocabulary, YAML |
| 3 | RTAB-Map 입력·TF·동기화 계약 추적 | High | Done | RGB-D, CameraInfo, odom/TF |
| 4 | HDL-Graph-SLAM 입력·TF·센서 계약 추적 | High | Done | PointCloud2, IMU/GPS 여부 |
| 5 | SAM-6D 입력·CAD·가중치·출력 계약 추적 | High | Done | RGB-D, intrinsics, object assets |
| 6 | 설치·빌드·대용량 자산 상태 점검 | High | Done | 모든 workspace 미빌드; 주요 자산 미복원 |
| 7 | 실제 bag 메타데이터와 네 입력 계약 비교 | High | In Progress | 증거 경계 확보; 인과 판정 단계 대기 |
| 8 | bag 재생 기반 smoke/E2E 실행 | High | Blocked | 빌드·자산 복원 및 입력 변환 필요 |

## Timeline of Events

| Time | Event | Source | Confidence |
| ---- | ----- | ------ | ---------- |
| 2026-07-30 | 호환성 조사 요청 접수 | 사용자 요청 | Confirmed |
| 2026-07-30 | BMad 조사 환경을 Python 3.11.15에서 활성화 | `bmad-py311` 실행 결과 | Confirmed |
| 2026-07-30 | 저장소가 RGB-D/IMU와 VLP-16을 별도 입력으로 사용하며 bag 데이터 자체는 포함하지 않음을 확인 | `README.md:24`, `README.md:54` | Confirmed |

## Confirmed Findings

### Finding 1: 네 파이프라인은 단일한 센서 입력 계약을 공유하지 않는다

**Evidence:** `README.md:54`

**Detail:** ORB-SLAM3와 RTAB-Map은 D455F RGB-D(+IMU) 계열을, HDL-Graph-SLAM은 VLP-16 LiDAR를 사용한다. 따라서 “ROS 2 bag”이라는 포맷만으로 호환성이 성립하지 않으며 bag에 필요한 센서 스트림이 모두 있어야 한다.

### Finding 2: bag 외부 자산 없이는 일부 알고리즘이 기동할 수 없다

**Evidence:** `README.md:70`

**Detail:** ORB-SLAM3 vocabulary와 SAM-6D weights·CAD·template는 별도 대용량 자산이다. 그러므로 “bag 데이터만”을 문자 그대로 해석하면 네 알고리즘 전체 실행은 불가능하다.

## Deduced Conclusions

### Deduction 1: 검증은 데이터 호환성과 런타임 준비 상태를 분리해야 한다

**Based on:** Finding 1, Finding 2

**Reasoning:** bag 토픽이 충분해도 외부 모델·캘리브레이션·빌드가 없으면 실행되지 않으며, 환경이 완성되어도 필요한 센서 토픽이 bag에 없으면 처리할 수 없다.

**Conclusion:** 최종 판정은 알고리즘별로 (A) bag 입력 충족, (B) 외부 자산·환경 충족, (C) 실제 재생 성공의 세 단계로 기록한다.

## Hypothesized Paths

### Hypothesis 1: ROS 2 bag 하나만 제공하면 네 알고리즘을 모두 실행할 수 있다

**Status:** Open

**Theory:** bag에 RGB, depth, CameraInfo, IMU, LiDAR, TF 등 모든 필수 데이터가 포함되어 있고 저장소의 실행 환경과 외부 자산이 준비되어 있다.

**Supporting indicators:** 저장소는 네 알고리즘을 한 스택으로 묶고 데이터 변환·분석 도구를 포함한다.

**Would confirm:** 알고리즘별 입력 계약을 충족하는 실제 bag 메타데이터와 네 파이프라인의 성공적인 smoke/E2E 실행.

**Would refute:** 필수 센서 토픽·타입·TF·캘리브레이션 누락 또는 외부 자산/빌드 누락으로 하나 이상의 파이프라인이 기동·처리하지 못함.

**Resolution:** 미결정.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| RGB-depth의 실제 공간 정렬 상태와 depth scale | RGB-D 기하학의 유효성을 완전히 확정할 수 없음 | String calibration/extrinsic으로 변환 후 정렬 결과 샘플 검증 |
| 변환된 표준 bag | ORB/RTAB/SAM-6D full pose의 직접 구독 성공을 검증할 수 없음 | 표준 CameraInfo·정렬 depth·정상 토픽을 생성하는 변환 수행 |
| 실제 실행 로그와 생성 출력 | 최종 실행 가능성을 확정할 수 없음 | 자산 복원·빌드 후 bag 재생 smoke test |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Error origin | 해당 없음 — 호환성 탐색 조사 |
| Trigger | ROS 2 bag 재생과 알고리즘별 노드 실행 |
| Condition | bag 입력 계약과 외부 환경·자산 조건을 모두 만족해야 함 |
| Related files | `docs/03_DATASETS.md`, `docs/10_ORBSLAM3.md`, `docs/11_HDL_GRAPH_SLAM.md`, `docs/12_RTABMAP.md`, `docs/13_SAM6D.md` |

## Conclusion

**Confidence:** Low

현재 확정 가능한 결론은 “bag 파일만 있으면 무조건 네 알고리즘 모두 실행 가능”하지 않다는 것이다. 저장소 근거상 입력 센서 종류와 외부 자산 요구가 서로 다르며, 실제 bag이 없어 데이터별 호환성은 아직 판정할 수 없다.

## Recommended Next Steps

### Fix direction

해당 없음 — 먼저 알고리즘별 입력 계약과 현재 설치 상태를 확정한다.

### Diagnostic

문서·런치·설정에서 필수 토픽과 메시지 타입을 추출하고, 설치 자산을 점검한 뒤 실제 bag 메타데이터와 비교한다.

## Reproduction Plan

1. 실제 bag의 저장 플러그인, 토픽, 메시지 타입, 메시지 수, 기간을 수집한다.
2. 각 알고리즘 환경에서 런치 파일이 로드되는지 확인한다.
3. bag을 `--clock`과 필요한 remap으로 재생한다.
4. pose/map/detection 출력 및 오류 로그를 수집한다.
5. 알고리즘별 PASS / CONDITIONAL / FAIL 판정을 기록한다.

## Side Findings

- 저장소 README는 데이터셋 약 380 GB가 저장소와 자산 아카이브에 포함되지 않는다고 명시한다 (`README.md:24`).
- BMad 설치로 `.agents/`, `node_modules/`, `package.json`, `package-lock.json`이 추적되지 않은 상태이며 조사에서는 변경하지 않는다.

## Follow-up: 2026-07-30

### New Evidence

- 실제 검증 대상 bag 3개가 `data_slam/0729/` 아래에 제공됐다.
- `0729_long_circle_inner`: 약 4.9 GB, SQLite3 bag `recording_20260729_090903.db3`.
- `0729_smaill_circle_inner`: 약 2.9 GB, SQLite3 bag `recording_20260729_085631.db3`.
- `0729_small_8`: 약 6.3 GB, SQLite3 bag `recording_20260729_085915.db3`.
- 세 bag 모두 `metadata.yaml`, `recording_summary.json`, `rgbd_timestamp_associations.json`, `session_manifest.json`을 포함한다.
- Windows 원본과 Linux 복사본을 `rsync --dry-run --checksum`으로 비교한 결과 차이가 없었다.

### Additional Findings

- 기존 Missing Evidence 중 “실제 bag 경로와 `ros2 bag info`”는 경로 제공으로 부분 해소됐다. 아직 메타데이터·메시지 내용 분석은 수행 전이다.

### Updated Hypotheses

- Hypothesis 1은 계속 Open이다. 세 bag의 토픽과 네 알고리즘의 실제 입력 계약을 대조해야 확정 또는 반박할 수 있다.

### Backlog Changes

- Backlog #7 “실제 bag 메타데이터와 네 입력 계약 비교”를 `Blocked`에서 `Open`으로 변경할 수 있는 새 증거가 확보됐다.
- Backlog #8 “bag 재생 기반 smoke/E2E 실행”은 메타데이터·환경 점검 후 진행한다.

### Updated Conclusion

**Confidence:** Low

실제 bag이 확보되어 정적 호환성 검증을 시작할 수 있다. 현재 단계에서는 bag 내용 분석 전이므로 알고리즘별 입력 가능 여부는 아직 판정하지 않는다.

## Follow-up: 2026-07-30 #2

### New Evidence

#### 실제 bag 공통 구조

- 세 bag은 rosbag2 metadata version 5, SQLite3, CDR 직렬화이며 각각 `PRAGMA quick_check=ok`다 (`data_slam/0729/0729_long_circle_inner/metadata.yaml:1`, `data_slam/0729/0729_smaill_circle_inner/metadata.yaml:1`, `data_slam/0729/0729_small_8/metadata.yaml:1`).
- 각 bag은 113개 토픽을 갖지만 실제 센서 스트림은 depth `sensor_msgs/msg/Image` 1개와 color `sensor_msgs/msg/Image` 1개뿐이다. 나머지 대부분은 RealSense SDK 설정을 직렬화한 `std_msgs/msg/String`이다 (`data_slam/0729/0729_long_circle_inner/metadata.yaml:23`).
- color 샘플은 640×480 `bgr8`, frame `Color`; depth 샘플은 640×480 `mono16`, frame `Depth`다. 두 stream의 메시지 수는 각 bag에서 동일하다.
- `0729_long_circle_inner`는 color/depth 각 3,391개, 약 113.03초다 (`data_slam/0729/0729_long_circle_inner/recording_summary.json:24`).
- `0729_smaill_circle_inner`는 color/depth 각 2,001개, 약 66.70초다 (`data_slam/0729/0729_smaill_circle_inner/recording_summary.json:24`).
- `0729_small_8`은 color/depth 각 4,365개, 약 145.5초다 (`data_slam/0729/0729_small_8/recording_summary.json:22`).
- 표준 `sensor_msgs/msg/CameraInfo`, `sensor_msgs/msg/Imu`, `sensor_msgs/msg/PointCloud2`, `sensor_msgs/msg/LaserScan`, `tf2_msgs/msg/TFMessage`는 세 bag 모두 0개다. `camera_info`와 `tf/ref_0`라는 이름의 토픽은 존재하지만 실제 타입은 `std_msgs/msg/String`이다 (`data_slam/0729/0729_long_circle_inner/metadata.yaml:653`, `data_slam/0729/0729_long_circle_inner/metadata.yaml:665`).
- bag 저장 timestamp는 0부터 시작하는 상대시간이다. 별도 `recording_summary.json`과 `rgbd_timestamp_associations.json`에 host epoch timestamp가 보관돼 있다.

#### 알고리즘 입력 계약

- ORB-SLAM3 wrapper는 RGB `Image`와 color 좌표계에 등록된 depth `Image`를 요구하며 허용 depth encoding은 `16UC1` 또는 `32FC1`이다. CameraInfo는 구독하지 않고 세션별 settings YAML을 요구한다 (`orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp:400`, `orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp:456`, `orbslam_ws/src/ORB_SLAM3/src/Frame.cc:997`).
- RTAB-Map RGB-D 경로는 RGB `Image`, 등록 depth `Image`, 표준 `CameraInfo`를 동기 입력으로 요구한다. 외부 JSON에서 CameraInfo를 생성하는 저장소 helper가 대안이다 (`rtabmap_ws/src/rtabmap_ros/rtabmap_sync/src/impl/CommonDataSubscriberDepth.cpp:506`, `rtabmap_ws/src/rtabmap_ros/rtabmap_util/scripts/json_to_camera_info.py:110`).
- HDL-Graph-SLAM 최소 입력은 `sensor_msgs/msg/PointCloud2`이며 기본 토픽 `/velodyne_points`는 remap 가능하다 (`hdlgraphslam_ws/src/hdl_graph_slam/launch/hdl_graph_slam_501.launch.py:38`, `hdlgraphslam_ws/src/hdl_graph_slam/src/hdl_graph_slam_ros2/prefiltering_node.cpp:43`).
- SAM-6D ISM-only는 color `Image`만으로 처리 가능하지만, 완전한 6D pose는 color `Image`, color-aligned 16UC1-mm depth, 유효한 color CameraInfo K, weights·CAD·templates를 요구한다 (`sam6d_ws/yolo_ism_object_n.py:684`, `sam6d_ws/tools/build_pem_inputs.py:41`, `sam6d_ws/tools/build_pem_inputs.py:122`, `sam6d_ws/ASSETS_REQUIRED.md:91`).

#### 현재 런타임 상태

- `scripts/verify_setup.sh`의 읽기 전용 점검에서 ORB vocabulary, SAM-6D weights·CAD·templates가 모두 없고 네 workspace의 `install/setup.bash`와 ORB core library도 없었다. 점검 항목 정의는 `scripts/verify_setup.sh:28`, `scripts/verify_setup.sh:44`에 있다.
- Conda 환경은 ORB-SLAM3, HDL-Graph-SLAM, RTAB-Map, SAM-6D용이 존재하지만 `sam_yolo`는 없다.
- 원본 PC 절대경로가 남은 파일 72개가 탐지됐다 (`scripts/verify_setup.sh:53`).
- 현재 worktree에는 기존 테스트 결과나 실제 실행 로그가 없다.

### Additional Findings

- 데이터 증거는 Available이다: 파일 복사 무결성, SQLite 무결성, 토픽·타입·개수, 첫 이미지 메시지 형식까지 확인했다.
- 소스 계약 증거는 Available이다: 네 알고리즘의 실제 구독 경로와 외부 자산 요구를 추적했다.
- 실행 증거는 Missing이다: 현재 빌드·자산·경로 상태 때문에 smoke/E2E 실행이 아직 성립하지 않는다.
- 버전 관리 증거는 Available이다: HEAD `1f221f6`; bag과 BMad 설치 산출물은 추적되지 않은 상태다.
- 이슈 트래커 증거는 Missing이나 로컬 bag 입력 적합성 판단에는 비핵심이다.

### Updated Hypotheses

- Hypothesis 1은 아직 Open이다. 토픽 계약만 보면 세 bag을 네 알고리즘 모두에 **직접** 넣을 수 없다는 반증이 강하지만, 정식 상태 전이는 다음 인과·반증 단계에서 기록한다.

### Backlog Changes

- Backlog #1~#6을 Done으로 변경했다.
- Backlog #7을 In Progress로 변경했다.
- Backlog #8은 변환·빌드·자산 복원 전까지 Blocked다.

### Updated Conclusion

**Confidence:** Medium

증거 경계는 충분히 확보됐다. 세 bag은 손상되지 않은 RealSense SDK-style RGB-D rosbag2지만 표준 RGB-D SLAM/SAM-6D bag 계약과 다르며 LiDAR가 전혀 없다. 다음 단계에서 알고리즘별 직접 사용 / 변환 후 사용 / 사용 불가 판정을 확정한다.
