---
title: 'ORB-SLAM3·RTAB-Map 4 RGB-D bag 실행 및 결과 추출'
type: 'feature'
created: '2026-06-17'
status: 'done'
baseline_commit: '503e871dc4ff6dd4b4d4b03b21dd3f939ea4bbf3'
context:
  - '{project-root}/rtabmap_ws/BUILD_CONDA_NOTES.md'
  - '{project-root}/rtabmap_ws/scripts/run_four_slam_bags.py'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `data_slam`의 4개 RealSense RGB-D bag에 대해 ORB-SLAM3와 RTAB-Map을 **실제로 새로 실행**해
trajectory·keyframe·map 결과를 얻어야 한다. orbslam_ws는 빌드 실패(rgbd_node 없음), 기존 산출물은 과거
다른 머신 결과라 재사용 불가. (시각화 PNG·비교 보고서는 분리되어 deferred-work.md로 보류됨.)

**Approach:** ① ORB-SLAM3 빌드를 고쳐 rgbd_node 생성 → ② 두 SLAM을 각 bag에 bag 재생과 함께 실행 →
③ 각 `output/<dataset>/`에 정규화된 결과 파일(trajectory.txt, keyframes.txt, map pcd) 저장.

## Boundaries & Constraints

**Always:**
- 실제 `ros2 bag play` 재생 기반으로 결과를 **새로 생성**. 입력은 `data_slam/<dataset>/`(metadata.yaml 포함 디렉터리).
- 경로는 프로젝트 루트(`/home/ldh9501/temp_ws/CLI_environment`) 기준. (`~/CLI_environment` → 이 루트로 매핑)
- 실행 env 분리: ORB-SLAM3 = conda `orbslam3`, RTAB-Map = conda `rtabmap`(robostack, 이미 빌드됨).
- 4 dataset × 2 SLAM = 8 실행. 각 실행 stdout/stderr 로그를 해당 output 폴더에 보존.
- 결과 파일명 정규화: ORB·RTAB 공통 `trajectory.txt`·`keyframes.txt`, 맵은 ORB=`map_points.pcd`(sparse), RTAB=`dense_map.pcd`.

**Ask First:**
- ORB-SLAM3 빌드를 합리적 시도(C++ 표준 상향 등) 후에도 고칠 수 없을 때.
- 특정 bag이 두 시스템 모두에서 트래킹 실패(궤적 산출 불가)할 때.

**Never:**
- 기존 결과 파일 재사용·복사로 "새 결과" 위장, 궤적/지표 날조.
- 원본 bag(`data_slam`) 수정·이동.
- 시각화 PNG·비교 보고서 생성(이번 범위 밖 — deferred-work.md).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| ORB 정상 실행 | dataset bag, rgbd_node 빌드됨 | `orbslam_ws/output/<ds>/`에 trajectory.txt·keyframes.txt·map_points.pcd + run 로그 | — |
| RTAB 정상 실행 | dataset bag, rtabmap 빌드됨 | `rtabmap_ws/output/<ds>/`에 trajectory.txt·keyframes.txt·dense_map.pcd + run 로그 | — |
| 트래킹 부분 실패 | 일부 구간 추적 손실 | 가용 구간만으로 결과 저장, 로그에 손실 기록, 계속 진행 | 빈 궤적이면 해당 실행을 로그에 명시하고 다음 dataset 진행 |
| Camera intrinsics 불일치 | bag camera_info ≠ 기존 yaml | bag camera_info 기준으로 ORB settings yaml 생성/보정 | 불일치 시 bag 값 우선 |
| bag 입력 형태 | `data_slam/<ds>/`(metadata.yaml+db3) | `ros2 bag play <dir>`로 재생 | 잘못된 경로면 즉시 실패·보고 |

</frozen-after-approval>

## Code Map

- `orbslam_ws/build_orbslam3.log` -- 빌드 실패 근원 로그(첫 실오류: `sigslot/signal.hpp`의 `std::decay_t/enable_if_t` 미인식 = C++11 컴파일)
- `orbslam_ws/src/ORB_SLAM3/CMakeLists.txt` (+ 포함 cmake) -- C++ 표준 11→14/17 상향 지점
- `orbslam_ws/src/orbslam3_ros2/{launch/orb_slam.launch.py, config/no_cli_rgbd.yaml, src/rgbd_node.cpp}` -- RGBD 노드, bag 자동재생, 출력(CameraTrajectory.txt/KeyFrameTrajectory.txt/pcd) 설정
- `orbslam_ws/data/orbslam3_d455f_640x480_rgbd.yaml` -- ORB 카메라 settings(intrinsics, DepthMapFactor=1000)
- `rtabmap_ws/scripts/run_four_slam_bags.py` -- RTAB 오케스트레이션 원형(하드코딩 `/home/etri`·`orbslam_ws/data`, Korean dataset명 → 수정 필요)
- `rtabmap_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py` -- RTAB 실행 인자(rgb/depth/info topic, approx_sync, database_path), `rtabmap-export`로 poses/cloud 추출
- `data_slam/<dataset>/` -- 입력 4 bag (topics: `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/color/camera_info`)

## Tasks & Acceptance

**Execution:**
- [x] `orbslam_ws/src/ORB_SLAM3/CMakeLists.txt` -- C++ 표준을 14(필요시 17)로 상향. orbslam3 env에서 `colcon build`로 ORB_SLAM3 + orbslam3_ros2 빌드 성공, `install`에 rgbd_node 생성 확인. 추가 conda 빌드 이슈는 `BUILD_CONDA_NOTES.md` 패턴 적용
- [x] `scripts/run_orbslam_four_bags.py` (신규) -- 4 dataset 각각 rgbd_node를 bag 재생과 함께 실행(use_sim_time, sync). 종료 시 ORB가 쓰는 CameraTrajectory.txt→`trajectory.txt`, KeyFrameTrajectory.txt→`keyframes.txt`, sparse→`map_points.pcd`로 `orbslam_ws/output/<ds>/`에 저장. bag camera_info로 settings intrinsics 정합 확인
- [x] `scripts/run_rtabmap_four_bags.py` (run_four_slam_bags.py 기반) -- 입력 `data_slam/<ds>/`, ENV_PREFIX·경로를 프로젝트 루트로 교정. 실행 후 `rtabmap-export`로 poses→`trajectory.txt`/keyframe poses→`keyframes.txt`, cloud→`dense_map.pcd`를 `rtabmap_ws/output/<ds>/`에 저장
- [x] 8 실행 산출물 정규화·검수 -- 각 output 폴더에 trajectory.txt·keyframes.txt·맵 pcd가 비어있지 않게 생성됐는지 라인수/포인트수로 확인, 로그 보존

**Acceptance Criteria:**
- Given 4개 bag, when 두 SLAM 실행, then `orbslam_ws/output/<ds>/`(trajectory.txt·keyframes.txt·map_points.pcd)와 `rtabmap_ws/output/<ds>/`(trajectory.txt·keyframes.txt·dense_map.pcd)가 새 타임스탬프로 존재한다.
- Given 각 trajectory.txt, when 열면, then 0이 아닌 다수 pose 행(timestamp x y z qx qy qz qw)이 있다.
- Given 각 map pcd, when 헤더를 보면, then POINTS > 0 이다.
- Given rgbd_node, when `find install -name rgbd_node`, then 실행파일이 존재한다(빌드 성공).

## Design Notes

- ORB 빌드 근원: `build_orbslam3.log` 첫 실오류가 `sigslot/signal.hpp`의 `std::decay_t/enable_if_t/remove_pointer_t` 미인식 → ORB_SLAM3가 C++11로 컴파일. 해결: `CMAKE_CXX_STANDARD 14`(또는 17) 강제. conda 빌드 시 Python/sysroot 힌트가 더 필요하면 `BUILD_CONDA_NOTES.md` 적용.
- ORB 출력: rgbd_node는 `no_cli_rgbd.yaml`의 output.dir로 CameraTrajectory.txt/KeyFrameTrajectory.txt(TUM: ts x y z qx qy qz qw)와 sparse/dense pcd를 shutdown 시 저장. dataset별 config(bag.path·output.dir)를 생성해 재사용.
- RTAB 출력: bag 재생 종료 후 노드 SIGINT → `rtabmap-export --cloud --poses --poses_format 11 --ascii`로 추출. RTAB는 노드=keyframe이므로 keyframes.txt는 노드 poses(또는 동일 poses)로 채움.
- 두 시스템 모두 frame_id=`camera_color_optical_frame`, approx_sync(±0.05), use_sim_time 사용.

## Verification

**Commands:**
- `source ~/miniconda3/etc/profile.d/conda.sh && conda activate orbslam3 && find orbslam_ws/install -name rgbd_node` -- expected: 실행파일 경로 출력
- `for d in SLAM_forward_backward_repeat SLAM_one_lap SLAM_one_lap_back_and_forth SLAM_three_laps; do echo "== $d =="; ls -la orbslam_ws/output/$d rtabmap_ws/output/$d; done` -- expected: 각 폴더에 trajectory.txt·keyframes.txt·맵 pcd
- `for d in .../output/*/trajectory.txt; do wc -l "$d"; done` -- expected: 각 파일 행 수 > 0

**Manual checks:**
- 각 trajectory.txt 앞부분과 map pcd 헤더를 열어 좌표·POINTS 수가 타당한지 확인.

## Suggested Review Order

**빌드 수정 (진입점)**

- ORB-SLAM3 빌드 실패의 근본 원인을 고친 단일 지점: C++ 표준을 14로 고정(C++11은 sigslot, C++17은 bool++ 실패)
  [`ORB_SLAM3/CMakeLists.txt:18`](../../../orbslam_ws/src/ORB_SLAM3/CMakeLists.txt#L18)

- 중복된 stray 클론 패키지(C++11)를 colcon에서 제외 — 빌드 충돌 제거
  [`orbslam3_core/COLCON_IGNORE:1`](../../../orbslam_ws/src/orbslam3_core/COLCON_IGNORE#L1)

**ORB-SLAM3 실행 하베스트**

- dataset별 launch를 띄우고 자체종료 대기(launch가 bag종료→SIGINT→저장→shutdown 처리)
  [`run_orbslam_four_bags.py:88`](../../../orbslam_ws/scripts/run_orbslam_four_bags.py#L88)

- ORB 출력(CameraTrajectory/KeyFrame/sparse)을 스펙 파일명으로 정규화
  [`run_orbslam_four_bags.py:115`](../../../orbslam_ws/scripts/run_orbslam_four_bags.py#L115)

- 성공 판정: trajectory·keyframe 둘 다 검사해 트래킹 실패를 성공으로 오판 방지
  [`run_orbslam_four_bags.py:156`](../../../orbslam_ws/scripts/run_orbslam_four_bags.py#L156)

**RTAB-Map 실행 하베스트**

- launch+bag재생+kill+export 핵심 루프, 경로/ENV를 프로젝트 루트 기준으로 교정
  [`run_rtabmap_four_bags.py:45`](../../../rtabmap_ws/scripts/run_rtabmap_four_bags.py#L45)

- bag 재생 watchdog 타임아웃 — 멈춘 player가 배치를 무한정 막지 못하게 함
  [`run_rtabmap_four_bags.py:94`](../../../rtabmap_ws/scripts/run_rtabmap_four_bags.py#L94)

- poses_format11→trajectory/keyframes 정규화, cloud→dense_map.pcd(비유한 좌표·ragged 행 방어)
  [`run_rtabmap_four_bags.py:128`](../../../rtabmap_ws/scripts/run_rtabmap_four_bags.py#L128)
