---
stepsCompleted: [1, 2, 3]
inputDocuments: []
workflowType: 'research'
lastStep: 1
research_type: 'technical'
research_topic: 'D435i 기반 5/10 다중 객체 테이블탑 데이터 수집 시나리오 설계'
research_goals: '다중 객체 파이프라인(YOLO-ISM+PEM+objectmemory)의 고속·경량화 검증에 적합한 촬영 시나리오(객체 배치, 카메라 운용, 세부 조건) 정의'
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

[Research overview and methodology will be appended here]

---

<!-- Content will be appended sequentially through research workflow steps -->

## Technical Research Scope Confirmation

**Research Topic:** D435i 기반 5/10 다중 객체 테이블탑 데이터 수집 시나리오 설계
**Research Goals:** 다중 객체 파이프라인(YOLO-ISM → PEM → objectmemory)의 고속·경량화 검증(ⓐ)에 적합한 촬영 시나리오(객체 배치, 카메라 운용, 세부 조건) 정의. 추후 모델 자체 경량화(ⓑ)는 후속 계획으로 유보.

**Technical Research Scope:**

- 센서 특성 분석 — D435i vs D455f 차이(베이스라인, RGB 셔터, FOV, IMU, depth 최적 거리)가 촬영 조건에 주는 제약
- 배치 설계 — 객체 간 간격/폐색 수준, 배경/조명, 5+5 세트 분할 기준
- 카메라 운용 패턴 — 궤적 종류/속도/높이/각도, SLAM(IMU-RGBD)과 ISM/PEM 요구 동시 만족 조건
- 기존 파이프라인 정합성 — 기존 SAM_* 4bag 포맷/토픽/해상도 호환, stride-10 프레임 예산
- 벤치마크 관례 참조 — BOP/YCB-Video 류 테이블탑 6D 데이터셋 촬영 관례 (웹 검증)

**Research Methodology:**

- 현재 웹 데이터 + 엄격한 출처 검증
- 핵심 기술 주장 다중 출처 교차 확인
- 불확실 정보는 확인됨/추정 신뢰도 구분
- 코드베이스(기존 bag 처리 경로) 근거와 웹 근거 병기

**Scope Confirmed:** 2026-07-07

## 기술 스택 분석 (센서 + 녹화 도구 체인)

### D435i vs D455(f) 핵심 사양 차이 [확인됨 — 웹 교차 검증]

| 항목 | D455(f) — 기존 | D435i — 이번 | 촬영 조건에 주는 의미 |
|---|---|---|---|
| 스테레오 베이스라인 | 95 mm | 50 mm | D435i는 원거리 depth 오차↑ → **가까이서 촬영해야 함** |
| 이상적 depth 범위 | 0.6–6 m (<2%@4m) | **0.3–3 m** (<2%@2m) | 테이블탑 근접 촬영에는 오히려 D435i가 유리 (MinZ ~28cm) |
| RGB 셔터 | **글로벌** | **롤링** | D435i는 빠른 카메라 이동 시 RGB 왜곡/블러 → **이동 속도 제한 필요** |
| Depth 셔터 | 글로벌 | 글로벌 | depth는 양쪽 동일 조건 |
| Depth FOV | 87°×58° | 87°×58° | 동일 |
| RGB FOV | 90°×65° | **69°×42°** | D435i는 화각이 좁음 → 같은 장면을 담으려면 **더 멀리/높이** 필요 (depth 이상범위와 트레이드오프) |
| IMU | 내장 | 내장 | IMU-RGBD SLAM 경로 유지 가능 |
| RGB 해상도 | 최대 1280×800 | 최대 1920×1080 | 어차피 640×480 운용이라 무관 |

_Sources: [RealSense D435i 제품 페이지](https://www.realsenseai.com/products/depth-camera-d435i/), [D455 제품 브리프](https://www.mouser.com/pdfDocs/D455ProductBriefv90.pdf), [RealSense 카메라 선택 가이드](https://www.intelrealsense.com/which-device-is-right-for-you/), [D400 시리즈 datasheet](https://cdrdv2-public.intel.com/841984/Intel-RealSense-D400-Series-Datasheet.pdf)_

- D435 계열의 depth 최적 해상도는 **848×480** (1280×720 아님) — [librealsense #11180](https://github.com/IntelRealSense/librealsense/issues/11180) [확인됨]
- D435i 정확도는 거리의 대략 1~2% 수준, 3 m 초과부터 급격히 저하 — [OpenELAB D435i 가이드](https://openelab.io/blogs/learn/intel-realsense-d435i-depth-camera-guide-complete-overview) [확인됨]
- D455**f**는 D455에 IR 필터가 추가된 변형으로 위 표의 D455 수치와 동일 취급 [추정 — f 변형 개별 스펙 미검증]

### 기존 파이프라인이 요구하는 녹화 스택 [확인됨 — 코드 근거]

- 소비 토픽 (`tools/build_ism_inputs_imu.py:41-43`): `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/color/camera_info` — **aligned depth 필수** (realsense-ros `align_depth.enable:=true`)
- SLAM 설정 (`orbslam_ws/data/orbslam3_d455f_640x480_rgbd_imu.yaml`): 640×480@30fps, DepthMapFactor 1000, IMU 200Hz, `T_b_c1`은 bag 내 `/camera/camera/extrinsics/*` 메시지에서 계산 — **D435i용 yaml 신규 작성 필요** (intrinsics/extrinsics가 다름)
- realsense-ros ROS2에서 IMU는 `enable_gyro:=true enable_accel:=true unite_imu_method:=2(linear_interpolation)`로 단일 `/camera/camera/imu` 토픽 생성 — [realsense-ros SLAM wiki](https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i) [확인됨]. 기존 d455f yaml이 IMU.Frequency 200을 가정하므로 동일 방식 권장.

### 스택 결론

D435i 전환은 파이프라인 코드 변경 없이 가능(토픽/포맷 동일)하나, 촬영 조건 설계에 3가지 제약을 만든다: ① **카메라-테이블 거리 0.4~2.0 m 유지** (depth 최적대역), ② **RGB 롤링 셔터 → 회전/이동 속도 상한** (특히 빠른 패닝 금지), ③ **좁은 RGB 화각(69°) → 10객체 전체 프레이밍 시 거리 확보 필요**. 준비물로 D435i용 ORB-SLAM3 yaml 1개 신규 작성이 필요하다.

## 통합 패턴 분석 (촬영 bag ↔ 처리 체인 인터페이스)

### 선행 문서와의 관계 [확인됨 — 로컬]

`sam6d_ws/research/data-collection-plan-unity-objectid.md`(2026-06-25)가 시나리오 S0~S8, GT 전략, 체크리스트를 이미 정의한다. 이번 수집은 그 계획의 **"객체 수 1→2~3→5 단계 확대" 축을 5/10으로 확장**하는 성능 검증 특화판이며, 아래 2가지를 보정한다:
- 구계획의 토픽 이름에 `/camera/camera/` 이중 네임스페이스가 빠져 있음 (실제 소비 코드와 불일치)
- IMU 토픽이 "권장"으로만 표기 — 현행 SLAM 경로(IMU-RGBD)에서는 필수로 격상

### 토픽 계약 (녹화 필수 목록) [확인됨 — 코드/기존 bag 근거]

```
필수 (build_ism_inputs_imu.py:41-43 + ORB-SLAM3 IMU-RGBD):
  /camera/camera/color/image_raw                    # ISM+PEM+SLAM 입력
  /camera/camera/aligned_depth_to_color/image_raw   # PEM depth (사후 복구 불가)
  /camera/camera/color/camera_info                  # intrinsics
  /camera/camera/imu                                # IMU-RGBD SLAM (unite_imu_method)
필수 (D435i 신규 yaml 산출용, 1회만 있으면 됨):
  /camera/camera/extrinsics/depth_to_color
  /camera/camera/extrinsics/depth_to_gyro           # T_b_c1 계산 (기존 d455f yaml과 동일 절차)
권장:
  /camera/camera/aligned_depth_to_color/camera_info, /tf_static
```

realsense-ros 기동 파라미터: `align_depth.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2 enable_sync:=true`, color/depth **640×480@30fps** (기존 SAM_* bag과 동일 조건 유지 — 성능 비교의 통제 변수). depth 스트림은 D435 계열 최적인 848×480으로 올리는 선택지가 있으나, aligned_depth는 color 해상도로 리샘플되므로 정합성 우선으로 640×480 유지 권장 [추정 — align 후 화질 이득 미검증].

### 녹화 설정 [확인됨 — 웹]

- **압축 미사용 권장**: rosbag2 file 압축(zstd)은 압축 중 신규 bag에 토픽이 기록되지 않아 메시지 유실 사례가 보고됨 — [rosbag2 #978](https://github.com/ros2/rosbag2/issues/978), [ROS Answers #415497](https://answers.ros.org/question/415497/). 기존 SAM_* bag도 무압축 sqlite3이므로 동일 유지.
- 드롭 방지: `--max-cache-size 1073741824`(1GB) 지정 사례 — [rosbag2 #1579](https://github.com/ros2/rosbag2/issues/1579). 녹화 직후 `ros2 bag info`로 **color 메시지 수 ≈ duration×30** 확인을 수집 프로토콜에 포함.
- 데이터량 추정: 640×480 RGB(3B)+depth(2B)×30fps ≈ 66MB/s → **2분 bag ≈ 8GB** [추정 — 무압축 상한, 실측 필요]. 디스크 여유 확인 필수.

### 처리 시간 예산 (bag 길이 설계의 근거) [확인됨 — 2026-07-06 실측 공식]

steady-state 공식 `frame_ms ≈ 100~120(ISM) + 110×N_det(PEM, vis 포함)`과 startup(ISM ~30s, PEM ~7s) 기준:

| 장면 | 프레임당 검출 수(상한) | steady 프레임당 | 2분 bag(stride-10, ~360f) 예상 SAM 체인 |
|---|---|---|---|
| 5객체 세트 A/B | ~5 | ~0.67s | 37s(고정비) + 240s ≈ **4.6분** |
| 10객체 전체 | ~10 | ~1.2s | 37s + 435s ≈ **7.9분** |

→ 기존 4bag(47~102s)보다 3~5배 길어짐: **PEM이 검출 수에 선형**이라 다중 객체 bag에서 처음으로 PEM이 명확한 단독 병목이 된다(ⓐ 목적에 정확히 부합 — PEM frame 배치화/--no-vis 개선의 실측 근거 데이터가 됨). top_k=3 게이트 통과 수에 따라 실제 N_det은 상한보다 낮을 수 있음 [추정].

### 평가 훅 (촬영 시 확보해야 처리 후 평가가 가능한 것) [확인됨 — 기존 방법 재사용]

- **pseudo-GT 감사**: 9-bag audit 방식(1,241프레임 precision/recall) 재사용 — 촬영 시 **layout 사진 + 객체 좌표 메모**만 있으면 됨
- **per-candidate 분석**: `sam6d_debug.csv` 자동 산출 (코드 변경 0)
- **objectmemory 평가**: 구계획의 `layout.json`(`{objects:[{gt_id,class,position_m}]}`) + `notes.md` 관례 유지
- **런타임 곡선**: 5 vs 10 객체 bag이 "객체 수별 runtime" 축의 실측점 — probe의 PerfCollector 재사용 가능

