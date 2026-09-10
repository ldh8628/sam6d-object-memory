# ORB-SLAM3 + D455f IMU 결합 타당성 기술 조사 보고서

- **작성일:** 2026-06-25
- **조사 유형:** 기술 조사 / 코드 분석 (구현 없음)
- **대상:** `orbslam_ws/` (ORB-SLAM3 기반 RGB-D 위치추정 파이프라인)
- **목표:** D455f 카메라(또는 연결 IMU)의 IMU 값을 추가 입력으로 사용하여 위치추정 정확도·안정성을 개선할 수 있는지 판단
- **근거 원칙:** 모든 결론은 (a) 로컬 코드 `파일:라인`, (b) ORB-SLAM3 논문/공식 GitHub, (c) Intel/realsense-ros 공식 문서에 근거. 추측은 "가정"으로 명시.

---

## 1. Executive Summary

### 결론: **조건부 적용 가능 (Conditionally Feasible)**

| 레이어 | 상태 | 판정 |
|---|---|---|
| ORB-SLAM3 **코어 엔진** | `IMU_RGBD=5` enum + RGB-D-Inertial 처리 로직 + 참조 예제 모두 보유 | ✅ **이미 지원 (코어 수정 불필요)** |
| ROS2 **래퍼**(`orbslam3_ros2`) | RGB/Depth 2개만 구독, `System::RGBD`로 고정, IMU 코드 전무 | ❌ **현재는 IMU 수용 불가 → 래퍼 수정 필요** |
| **하드웨어** (D455f) | D455f는 D455와 동일한 6축 IMU 내장 | ✅ **사용 가능 (단, 실측 확인 필요)** |
| **데이터** (현재 bag) | `data/rgbd_bag`에 IMU 토픽 없음 | ❌ **IMU 포함 bag 재수집 필요** |

**한 줄 요약:** "ORB3-SLAM"은 실제로는 공식 **ORB-SLAM3**(UZ-SLAMLab)이며, 이 코어는 **이미 RGB-D-Inertial(`IMU_RGBD`) 모드를 구현**하고 있고 워크스페이스 안에 D435i용 RGB-D-Inertial 참조 예제까지 들어 있다. 따라서 **코어를 수정할 필요가 없고**, 막혀 있는 부분은 오직 **ROS2 래퍼가 IMU 토픽을 받아 `TrackRGBD(..., vImuMeas)`로 넘기지 않는다는 점**과 **현재 데이터에 IMU가 없다는 점**이다. 즉 핵심 작업은 "코어 확장"이 아니라 "래퍼 배선 + 데이터 재수집 + camera-IMU 캘리브레이션"이다.

> ⚠️ **중요 정정:** 사용자가 제시한 "D안: ORB-SLAM3 내부를 수정하여 RGB-D-Inertial 확장"은 **불필요**하다. 코어는 이미 RGB-D-Inertial을 지원한다 (§2.B, §6 참조).

---

## 2. Current `orbslam_ws` Pipeline Analysis

### 2.A "ORB3-SLAM" 명칭 정체
- 코드상 실체는 **공식 ORB-SLAM3** (`src/ORB_SLAM3/`, `src/orbslam3_core/`). 커스텀 알고리즘 이름이 아니라 UZ-SLAMLab ORB-SLAM3 그대로이며, ROS2 래퍼만 별도(`src/orbslam3_ros2/`).
- `src/ORB_SLAM3`와 `src/orbslam3_core`는 **동일 구조의 ORB-SLAM3 두 트리**이며 둘 다 IMU 지원 버전(§2.B). (래퍼가 어느 쪽을 링크하든 IMU 지원 여부는 동일.)

### 2.B 현재 입력 토픽 (래퍼 = `rgbd_node.cpp`)
| 입력 | 토픽 | 근거 |
|---|---|---|
| RGB | `/camera/camera/color/image_raw` | `rgbd_node.cpp:236` (param 기본값) |
| Depth | `/camera/camera/aligned_depth_to_color/image_raw` | `rgbd_node.cpp:237` |
| 동기화 | `message_filters` **ApproximateTime<Image, Image>** (2-입력만) | `rgbd_node.cpp:24-26, 282-287` |
| CameraInfo | 토픽 구독 아님 — intrinsic은 **YAML에서 읽음** (`data/orbslam3_d455f_640x480_rgbd.yaml`) | YAML `Camera1.fx` 등 |

### 2.C 현재 센서 모드
- **`ORB_SLAM3::System::RGBD`** 로 고정. 근거: `rgbd_node.cpp:232`.
- 트래킹 호출: `slam_->TrackRGBD(rgb, depth, timestamp)` — **IMU 인자(`vImuMeas`) 미전달**(빈 벡터 기본값으로 호출). 근거: `rgbd_node.cpp:319`.

### 2.D 현재 실행 구조
- 노드: `RgbdNode` (executable, `orbslam3_ros2`).
- Launch: `src/orbslam3_ros2/launch/orb_slam.launch.py` — `rgb_topic`, `depth_topic`, `world_frame`, `camera_frame` 등만 파라미터로 전달, **IMU 관련 파라미터 없음**.
- 배치 실행: `scripts/run_orbslam_four_bags.py` + `scripts/configs/*.yaml` — 토픽 정의에 RGB/Depth만 존재, IMU 미처리.

### 2.E IMU 관련 코드 존재 여부 (래퍼)
- **미발견.** `grep -n "imu\|IMU\|Imu\|gyro\|accel"` → `orbslam3_ros2/src/` 에서 결과 없음.
- `sensor_msgs/msg/Imu` 구독자 없음. IMU 파라미터/콜백/버퍼 없음.

### 2.F 설정 YAML
- `data/orbslam3_d455f_640x480_rgbd.yaml`: `Camera.type: "PinHole"`, intrinsic/depth 설정만 존재. **IMU.NoiseGyro / IMU.NoiseAcc / IMU.GyroWalk / IMU.AccWalk / IMU.Frequency / IMU.T_b_c1 모두 부재.**
- `src/orbslam3_ros2/config/no_cli_rgbd.yaml`: IMU 설정 없음.

### 2.G 데이터 가용성 (현재 bag)
- `data/rgbd_bag/metadata.yaml` 토픽 목록: `color/image_raw`, `aligned_depth_to_color/image_raw`, `color/camera_info`, `aligned_depth_to_color/camera_info`, `color/metadata`, `depth/metadata`, `/tf_static`.
- **IMU 토픽 없음** (`imu`/`gyro`/`accel` 부재). → 기존 bag으로는 VI 검증 불가, **재수집 필요**.

**소결:** 현재 파이프라인은 순수 RGB-D 모드(IMU=0)이며, 래퍼·설정·데이터 어디에도 IMU 입력 경로가 없다. 그러나 코어 엔진은 IMU_RGBD를 완전히 구현하고 있다(§6).

---

## 3. ORB-SLAM3 Paper Findings

출처: Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual–Inertial, and Multimap SLAM", IEEE T-RO 37(6):1874-1890, 2021. (arXiv:2007.11898)

### 3.1 논문이 명시한 센서 구성
- 카메라: Monocular, Stereo, RGB-D (pinhole + fisheye).
- Visual-Inertial: **Monocular-Inertial, Stereo-Inertial** — 이 둘만 논문의 평가/기여로 명시.
- **RGB-D-Inertial은 논문 제목/초록/실험 테이블에 명시되지 않음.** 모든 VI 평가는 EuRoC(mono/stereo-inertial), TUM-VI(fisheye mono/stereo-inertial) 기준.

### 3.2 IMU 사용 방식
- **Tightly-coupled** feature-based VI. **IMU 초기화 단계까지 포함해 MAP(Maximum-a-Posteriori) 추정에 전적으로 의존**하는 것이 핵심 기여.
- 역할: IMU preintegration → visual-inertial **initialization(스케일·중력방향·자이로/가속도 bias 추정)** → tracking 강건화(저텍스처·모션블러·일시적 가림에서 관성으로 보강) → Atlas(multimap) 기반 loop closing/map merging과 결합.

### 3.3 정량적 이득
- 이전 기법 대비 **2~5배 정확**.
- Stereo-inertial: EuRoC 드론 평균 **약 3.6 cm**.
- 빠른 핸드헬드: TUM-VI room **약 9 mm**.

### 3.4 논문 기준 결론
- D455f **RGB-D + IMU를 논문이 직접 검증한 구성은 아님** → 논문만 보면 "비공식".
- 논문 구조상 가장 타당하고 검증된 대안은 **Stereo-Inertial**. Mono-Inertial은 지원되나 스케일 관측성이 약해 상대적으로 덜 강건.
- 기대 개선점: 회전 빠른 구간/특징 부족 구간에서 tracking 손실 감소, 절대 스케일/중력 정합 향상, 드리프트 감소. 한계: IMU 캘리브레이션·시간동기화 품질에 성능이 크게 좌우되며, 초기화에 충분한 가속/모션이 필요.

---

## 4. ORB-SLAM3 GitHub README Findings

출처: github.com/UZ-SLAMLab/ORB_SLAM3 (README, `include/System.h`, `Examples/`), issue #80.

### 4.1 README가 나열하는 센서 모드
- Visual: Monocular, Stereo, RGB-D. Visual-Inertial: **Monocular-Inertial, Stereo-Inertial** (pinhole + fisheye).
- **소스 코드 enum에는 6종 존재** (= 로컬과 동일):
  ```cpp
  enum eSensor{ MONOCULAR=0, STEREO=1, RGBD=2,
                IMU_MONOCULAR=3, IMU_STEREO=4, IMU_RGBD=5 };
  ```
  → **코드 레벨에서 `IMU_RGBD`(RGB-D-Inertial)는 존재**하지만, 공식 README/문서가 전면에 내세우는 inertial **실행 예제는 Stereo/Mono-Inertial 뿐**.

### 4.2 IMU 예제 존재 여부
- 공식 예제 데이터셋: **EuRoC**(pinhole+IMU), **TUM-VI**(fisheye+IMU).
- RealSense: **D435i 기준 Stereo-Inertial 예제** 제공 (`stereo_inertial_realsense_D435i`), T265 언급. **공식 RealSense 예제는 D435i이며 D455 아님.**
- IMU YAML 키 형식 (공식 `RealSense_D435i.yaml` 실측):
  ```yaml
  IMU.NoiseGyro: 1e-3
  IMU.NoiseAcc:  1e-2
  IMU.GyroWalk:  1e-6
  IMU.AccWalk:   1e-4
  IMU.Frequency: 200.0
  IMU.T_b_c1: !!opencv-matrix   # body(IMU) -> camera 4x4 변환
  ```
  ※ 질문의 `Tbc`는 구표기. **최신 코드는 `IMU.T_b_c1` 사용.**

### 4.3 README 기준 적용 가능성 판단
- D455f RGB-D + IMU는 **공식 "지원 모드 목록"에는 없음** → 공식적으로 보증된 경로 아님.
- 단, **코드에 `IMU_RGBD`가 구현되어 있고**(아래 §6) **로컬 워크스페이스에 RGB-D-Inertial 예제(`rgbd_inertial_realsense_D435i.cc`)가 실제로 존재** → "비공식이지만 코드상 지원되며 참조 구현이 있는" 상태. 따라서 **원본 코어 수정 없이 래퍼 작업만으로 가능.**

---

## 5. D455f / RealSense IMU Availability Check

### 5.1 하드웨어
- **D455**: 6축 IMU(3축 가속도 + 3축 자이로) 내장.
- **D455f**: D455 + IR pass filter 변형. Intel 사양상 **IMU 동일 내장**("with IMU"). → 하드웨어상 IMU 사용 가능.
- 출처: intelrealsense.com/depth-camera-d455/ , store.intelrealsense.com (D455f "with IMU"), intel.com D455f spec.

### 5.2 realsense-ros(ROS2)에서 IMU 토픽이 나오는 조건
- 파라미터: `enable_gyro:=true`, `enable_accel:=true` → 개별 raw 스트림.
- `unite_imu_method:=linear_interpolation` 또는 `copy` → gyro/accel을 합친 단일 `imu` 토픽 발행. (`copy`가 더 안정적이라는 보고)
- 출처: github.com/IntelRealSense/realsense-ros (ros2-master README).

### 5.3 실제 확인 명령어 (현재 환경에서 실행 권장)
```bash
# 1) 드라이버 띄운 상태에서 IMU 토픽 존재 확인
ros2 topic list | grep -Ei 'imu|gyro|accel'

# 2) 실제 데이터/레이트 확인
ros2 topic hz   /camera/camera/imu            # 통합 토픽(약 200Hz 기대)
ros2 topic hz   /camera/camera/gyro/sample
ros2 topic hz   /camera/camera/accel/sample
ros2 topic echo /camera/camera/imu --once     # 메시지 타입/필드 확인
ros2 topic info /camera/camera/imu            # 타입이 sensor_msgs/msg/Imu 인지

# 3) 기존/신규 bag에 IMU가 들어있는지
ros2 bag info orbslam_ws/data/rgbd_bag        # 현재 bag → IMU 없음(이미 확인됨)
```

### 5.4 예상 토픽 후보 (네임스페이스에 따라 변동)
- 통합: `/camera/camera/imu` (또는 `/camera/imu`, `/camera/imu/data`)
- 개별: `/camera/camera/gyro/sample`, `/camera/camera/accel/sample`
- (이미지 측은 기존: `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`)

### 5.5 IMU 토픽이 없을 때의 해석
1. 드라이버에서 `enable_gyro`/`enable_accel`(+`unite_imu_method`)가 꺼져 있다 → **재실행으로 해결**.
2. 모델이 IMU 미탑재 변형 → D455f는 IMU 탑재이므로 가능성 낮음. 실측으로 확정 필요.
3. 외부 IMU 필요 여부: D455f IMU가 정상 출력되면 **외부 IMU 불필요**. 다만 D455 IMU의 VI 적합성 문제(§아래 위험)가 심각하면 별도 IMU가 대안이 될 수 있음.

### 5.6 D455 IMU를 ORB-SLAM3 VI에 쓸 때 알려진 이슈
- D455 IMU가 VI SLAM에서 불안정/크래시한 사례 다수 보고 (ORB-SLAM3 issue #346).
- 권장 조치: **Auto Exposure Priority OFF**(프레임레이트 변동→시간정렬 붕괴 방지), **Global Time ON**(타임스탬프 일관성).
- `unite_imu_method`로 합성한 IMU 샘플이 VI에 부적합하다는 보고 (realsense-ros issue #2221) → 개별 gyro/accel + 자체 보간이 더 안전할 수 있음(로컬 D435i 예제도 보간 방식 사용).
- camera-IMU 캘리브레이션은 **kalibr + Allan variance** 권장 (ORB-SLAM3 Calibration_Tutorial.pdf).

---

## 6. Feasibility Decision

| 질문 | 답 | 근거 |
|---|---|---|
| 코어가 RGB-D-Inertial을 지원하나? | **예** | `System.h:93` `IMU_RGBD=5`; `System.cc:386-388` IMU_RGBD일 때 `GrabImuData` 호출; `TrackRGBD(...,vImuMeas,...)` 시그니처(`System.h:116`); `ImuTypes.h` `IMU::Point`; **`orbslam3_core/Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc` 참조 예제 존재** |
| 코어 수정이 필요한가? | **아니오** | 위 기능 모두 기구현 |
| 래퍼가 IMU를 받을 구조인가? | **아니오 (현재)** | `rgbd_node.cpp` IMU 코드 전무, `System::RGBD` 고정(`:232`), `TrackRGBD` IMU 미전달(`:319`) |
| 래퍼 수정으로 가능한가? | **예** | IMU 구독자 추가 + 버퍼 + `IMU_RGBD`로 생성 + `vImuMeas` 전달이면 됨 (D435i 예제가 그대로 청사진) |
| 후단 fusion 대안 가능한가? | **예** | `robot_localization` EKF로 ORB-SLAM pose + IMU 후단 융합 (loosely-coupled) |
| 하드웨어/데이터? | HW ✅ / 데이터 ❌ | D455f IMU 내장 / 현재 bag IMU 없음 → 재수집 필요 |

---

## 7. Recommended Implementation Direction

### ⭐ 1순위 (권장): 래퍼를 `IMU_RGBD`(RGB-D-Inertial)로 확장 — 사용자의 "D안"이되 코어 수정은 불필요
**근거:** 코어가 이미 지원하고, 워크스페이스에 D435i RGB-D-Inertial 참조 구현이 있으며, **기존 RGB-D 자산(aligned depth, intrinsic, 스케일, 평가 스크립트)을 그대로 유지**하면서 IMU만 얹을 수 있다. tightly-coupled라 후단 EKF보다 tracking 강건화 효과가 크다.
- 장점: 기존 파이프라인/메트릭 스케일 보존, 코어 무수정, 참조 예제 존재, depth로 스케일 보강.
- 단점/위험: RGB-D-Inertial은 논문/공식 README가 정량검증하지 않은 "반(半)공식" 모드, camera-IMU 캘리브레이션·시간동기화 필수, D455 IMU 안정성 이슈.

### 대안 A — RGB-D 유지 + 후단 EKF fusion (`robot_localization`) [사용자 "A안"]
- 장점: ORB-SLAM3/래퍼 거의 무수정, 가장 낮은 리스크, 빠른 PoC. IMU로 고주파 pose 보간·드리프트 평활.
- 단점: loosely-coupled라 **tracking 자체의 손실(특징 부족 구간 lost)은 근본 해결 못 함**. 캘리브레이션 부담은 작음(IMU→base TF 정도).

### 대안 B — Stereo-Inertial 전환 [사용자 "C안"] (가장 검증된 모드)
- 장점: 논문/공식 예제가 가장 강하게 검증(EuRoC 3.6cm). 강건성 최상.
- 단점: D455f의 **IR 스테레오 스트림(infra1/infra2) 입력으로 전환** 필요 → 현재 RGB+aligned-depth 파이프라인/bag을 사실상 새로 구성, depth 자산 폐기, 래퍼도 stereo_inertial 노드로 대체. 변경 폭 최대.

### 비권장 — Monocular-Inertial 전환 [사용자 "B안"]
- depth를 버리고 스케일 관측성이 약해지는 방향이라, 이미 metric depth가 있는 환경에서는 합리적이지 않음.

> **추천 로드맵:** 먼저 **대안 A(후단 EKF)**로 저비용 baseline을 잡고 개선폭을 측정 → 효과가 부족하거나 tracking 손실이 핵심 문제면 **1순위(IMU_RGBD 래퍼 확장)**로 진행 → 그래도 강건성이 부족하면 **대안 B(Stereo-Inertial)**를 최후 카드로.

---

## 8. Required Code Changes If Approved Later (설계만, 구현 X)

### 8.1 1순위(IMU_RGBD 래퍼 확장) 시 수정 예상 파일
| 파일 | 변경 내용 |
|---|---|
| `src/orbslam3_ros2/src/rgbd_node.cpp` | ① `sensor_msgs::msg::Imu` 구독자 추가(고주파, SensorDataQoS) ② IMU 버퍼(뮤텍스 보호) ③ `System` 생성 인자 `RGBD → IMU_RGBD`(`:232`) ④ 프레임 timestamp 직전까지의 IMU 샘플을 `vector<ORB_SLAM3::IMU::Point>`로 모아 `TrackRGBD(rgb, depth, t, vImuMeas)` 전달(`:319`) — **로직은 `Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc`를 청사진으로** |
| `src/orbslam3_ros2/launch/orb_slam.launch.py` | `imu_topic` 파라미터 추가, IMU 설정 YAML 경로 전달 |
| `src/orbslam3_ros2/config/no_cli_rgbd.yaml`, `data/orbslam3_d455f_*.yaml` | `IMU.NoiseGyro/NoiseAcc/GyroWalk/AccWalk/Frequency`, `IMU.T_b_c1`(body→camera 4x4), `IMU.InsertKFsWhenLost` 등 IMU 블록 추가 (D455f **자체** 캘리브레이션 값) |
| `scripts/run_orbslam_four_bags.py`, `scripts/configs/*.yaml` | 토픽 정의에 IMU 토픽 추가, IMU 설정 YAML 선택 |
| (코어) `src/ORB_SLAM3`, `src/orbslam3_core` | **수정 불필요** |

추가 ROS2 subscriber: `sensor_msgs/msg/Imu`. (개별 gyro/accel 채택 시 두 토픽 구독 + 자체 보간; `unite_imu_method` 합성 토픽은 VI 부적합 보고 있으므로 검토 필요.)

### 8.2 대안 A(후단 EKF) 시
- 코어/래퍼 거의 무수정. `robot_localization`의 `ekf_node` 추가 launch + 파라미터(YAML): 입력=ORB-SLAM odom/pose + `/camera/.../imu`, 출력=fused odom. ORB pose를 nav_msgs/Odometry로 publish하는 어댑터만 필요할 수 있음.

### 8.3 필요한 calibration (공통, VI 경로)
- Camera-IMU **extrinsic `T_b_c1`** (kalibr).
- **time offset**(td) 및 IMU **noise/random-walk** 파라미터(Allan variance).
- RealSense optical frame ↔ IMU frame ↔ ROS TF 정합(좌표축 부호 주의).

---

## 9. Validation Plan

### 9.1 비교 지표 (RGB-D only baseline 대비)
- **ATE/RPE** (절대/상대 궤적 오차) — `evo` 또는 기존 `evaluation/` 스크립트 사용.
- Tracking 손실 횟수/lost 구간 길이, 재로컬라이즈 빈도.
- 스케일 일관성, 중력 정렬, 루프 클로저 전/후 드리프트.

### 9.2 방법
1. **동일 bag**(IMU 포함)으로 RGB-D only vs RGB-D-Inertial vs (RGB-D+EKF) 3안을 같은 입력에 재생.
2. 기존 산출물 구조(`output/LC_before`, `output/LC_after`, `SLAM_*` 4-bag)와 동일 포맷으로 CameraTrajectory/KeyFrameTrajectory 비교, top-view 시각화 재사용.
3. **Loop closure 영향**: `LC_before/LC_after` 비교 방식 그대로 적용해 IMU가 루프 클로저 정합/맵 머징에 주는 영향 확인.
4. **빠른 회전/저텍스처 구간**을 의도적으로 포함한 bag으로 강건성 차이 측정.

### 9.3 데이터/재생 검증
- `ros2 bag record`에 포함할 토픽: color/image_raw, aligned_depth_to_color/image_raw, color/camera_info, **imu(또는 gyro+accel)**, tf_static.
- IMU **≈200Hz**, 카메라 **30FPS**, timestamp 동기화(Global Time ON) 확인.
- QoS: IMU는 SensorData(BEST_EFFORT) 가능성 — 구독자 QoS 불일치 시 무수신 주의. `ros2 topic hz`로 실제 수신 검증.

---

## 10. Final Recommendation

### 핵심 검증 질문 직답
1. **ORB-SLAM3가 RGB-D+IMU를 공식 지원?** → **부분.** 논문/공식 README는 Mono/Stereo-Inertial만 검증·예제화. 그러나 **코드에는 `IMU_RGBD`가 구현**되어 있고 워크스페이스에 RGB-D-Inertial 참조 예제 존재 → "비공식이지만 코드상 지원".
2. **현재 래퍼가 IMU를 받는 구조인가?** → **아니오.** RGB/Depth만 구독, `System::RGBD` 고정, `TrackRGBD` IMU 미전달.
3. **가장 현실적인 개선 방향?** → **1차로 A안(후단 EKF, 저리스크 baseline) → 효과 부족 시 "코어 무수정 IMU_RGBD 래퍼 확장"(가장 자산 보존적)**. C안(Stereo-Inertial)은 최고 강건성이나 파이프라인 재구성 비용 최대. **D안의 "코어 수정"은 불필요**(이미 지원).
4. **D455f IMU 실제 사용 가능?** → **하드웨어상 가능.** 단 §5.3 명령으로 실토픽·레이트 실측, Auto-Exposure-Priority OFF / Global Time ON, D455 IMU VI 안정성 이슈 사전 검증 필요.
5. **수정 파일?** → §8.1 (래퍼 `rgbd_node.cpp`, launch, config/data YAML, 배치 스크립트; 코어 무수정).
6. **반드시 확인할 위험 요소?**
   - 🚩 **데이터 부재**: 현재 bag에 IMU 없음 → 재수집이 선결 (최우선 블로커).
   - 🚩 **D455 IMU의 VI 부적합 보고**(issue #346, #2221): 크래시·불안정 가능 → 소규모 실증 먼저.
   - 🚩 **camera-IMU 캘리브레이션/시간동기화** 품질이 성능 좌우.
   - 🚩 **RGB-D-Inertial 비공식**: 공식 정량검증 부재로 튜닝 부담.
   - 🚩 IMU 초기화에 충분한 모션(가속) 필요 — 정적/저속 구간 많으면 초기화 실패 가능.

### 지금 구현해도 되나?
**아니오 — 먼저 2가지 확인 실험 필요.**
1. **D455f IMU 실토픽 확인 실험** (§5.3): 토픽/타입/레이트/타임스탬프 정상 여부.
2. **IMU 포함 bag 재수집** (§9.3): color + aligned_depth + imu(또는 gyro/accel) + camera_info.

이 둘이 통과되면 → **저비용 A안(후단 EKF) PoC로 개선폭 정량 측정** → 효과/필요성 확인 후 **IMU_RGBD 래퍼 확장**으로 진입.

### `/bmad-quick-dev` 또는 `/bmad-architect` 진행 전 승인 기준 (Gate)
- [ ] D455f가 `sensor_msgs/msg/Imu`를 정상 발행(≈200Hz, Global Time)함을 `ros2 topic hz/echo`로 확인.
- [ ] IMU 포함 테스트 bag ≥1개 확보 및 `ros2 bag info`로 토픽 검증.
- [ ] camera-IMU 캘리브레이션(`T_b_c1`, noise/walk, td) 확보 계획 합의.
- [ ] 비교 baseline(현 RGB-D only ATE/RPE) 수치 고정.
- [ ] 1차 접근(A안 vs IMU_RGBD) 선택 합의.

---

## 다음 액션 제안

선결 확인(실측) 없이 바로 구현 단계로 넘어가면 D455 IMU 이슈·데이터 부재로 헛작업 위험이 큽니다. 다음 중 선택해 주세요:

- **(1) 실측 검증 먼저** — 제가 §5.3 / §9.3 확인 명령을 정리한 체크리스트(또는 검증 스크립트 초안)를 만들고, 박정철님이 카메라 연결 환경에서 IMU 토픽·bag을 실측 → 결과를 가지고 타당성을 확정.
- **(2) 저비용 A안(후단 EKF) 설계로 진행** — `robot_localization` 기반 loosely-coupled 융합의 상세 설계/파라미터안을 `/bmad-architect` 또는 `/bmad-quick-dev`로 작성.
- **(3) IMU_RGBD 래퍼 확장 상세 설계** — `rgbd_inertial_realsense_D435i.cc`를 청사진으로 `rgbd_node.cpp` 변경 상세 설계서 작성(구현 전 단계).

**질문:** 먼저 (1) 실측 검증부터 가시겠습니까, 아니면 실측은 박정철님이 별도로 진행하고 제가 (2)/(3) 설계 문서를 먼저 준비할까요? 그리고 D455f IMU를 실제로 켤 수 있는 환경(카메라 물리 연결)이 지금 가능한 상태인지 알려주시면 경로를 확정하겠습니다.
