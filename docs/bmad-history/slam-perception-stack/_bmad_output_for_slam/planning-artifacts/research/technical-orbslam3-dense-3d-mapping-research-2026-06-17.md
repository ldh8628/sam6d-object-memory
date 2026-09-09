---
stepsCompleted: [1, 2, 3]
inputDocuments: []
workflowType: 'research'
lastStep: 3
research_type: 'technical'
research_topic: 'ORB-SLAM3 dense 3D 맵 생성 방법'
research_goals: 'ORB-SLAM3의 sparse feature 맵 대신 시각적으로 풍부한 dense 3D 맵을 생성하는 방법을 조사하여, RealSense RGB-D + ROS2 환경(현 orbslam_ws)에 적용 가능한 실용적 옵션을 정리'
user_name: 'ldh'
date: '2026-06-17'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-06-17
**Author:** ldh
**Research Type:** technical

---

## Research Overview

ORB-SLAM3는 특징점(sparse feature) 기반 SLAM으로, 산출 맵이 sparse map point cloud라 시각적으로 환경 구조가 드러나지 않는다. 본 리서치는 ORB-SLAM3의 정확한 pose/keyframe을 활용해 **dense 3D 맵**을 생성하는 방법을 조사하되, 사용자 선택에 따라 **RealSense RGB-D + ROS2(현 `orbslam_ws`) 환경에 바로 적용 가능한 depth fusion** 접근에 초점을 둔다.

---

## Technical Research Scope Confirmation

**Research Topic:** ORB-SLAM3 dense 3D 맵 생성 방법
**Research Goals:** sparse feature 맵 대신 시각적으로 풍부한 dense 3D 맵 생성 방법 조사 → RealSense RGB-D + ROS2 환경 적용 옵션 정리

**확정 초점 (사용자 선택):** RGB-D depth fusion 실전 적용
- 현 RGB-D(RealSense D455 640×480) + ROS2에 바로 적용 가능한 depth fusion 위주
- TSDF/voxel/surfel 계열(Voxblox, OpenVDB/nvblox, InfiniTAM, ElasticFusion 등) 및 단순 point 누적
- 기존 `orbslam3_ros2` dense 노드(이미 `orbslam3_dense_map.pcd` 생성) 강화 관점 포함
- ROS2 통합 패턴(keyframe pose + depth 연동), 실시간성·메모리·품질 트레이드오프

**보조 맥락(간략):** ORB-SLAM3 dense 포크, 오프라인 MVS, Photo-SLAM(3DGS) 등은 비교 맥락으로만 언급.

**Research Methodology:** 최신 웹 자료 + 다중 출처 교차검증 + 불확실 정보 신뢰도 표기.

**Scope Confirmed:** 2026-06-17

---

<!-- Content will be appended sequentially through research workflow steps -->

## Technology Stack Analysis

> 모든 항목은 웹 검색으로 교차검증함. ORB-SLAM3는 **native로 dense 맵을 만들지 않으며**, Atlas는 sparse keyframe+MapPoint 그래프만 보유함이 공식 repo 및 다수 논문에서 확인됨 ([UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), [IEEE 10730184](https://ieeexplore.ieee.org/document/10730184/)). 따라서 dense 맵은 **별도 맵핑 스레드/노드를 붙여** ORB-SLAM3의 최적화된 keyframe pose + RGB-D depth로 생성하는 것이 표준.

### 핵심 접근 (sparse→dense 한눈에)

| 접근 | 방식 | 실시간 | HW | 루프클로저 정합 | 현 환경 적합도 |
|---|---|---|---|---|---|
| **Keyframe depth backprojection** (point 누적) | KF pose로 depth 역투영→voxel 누적 | O | CPU | 재생성 필요(대부분 미지원) | ★★★ (현 노드가 이미 이 방식) |
| **TSDF/voxel fusion** (Voxblox/nvblox/VDBFusion) | pose+depth를 부피 적분→mesh | O(GPU)/O | CPU·GPU | 대부분 미보정(submap 필요) | ★★ |
| **Surfel** (ElasticFusion/InfiniTAM) | surfel + deformation graph | O | GPU | 일부 보정 | ★ (ROS2·유지보수 부재) |
| **Photorealistic 3DGS/NeRF** (Photo-SLAM) | Gaussian splatting 외관맵 | O(GPU) | 강GPU | 트래커 의존 | ☆ (고품질·고비용, 별도 경로) |

### ORB-SLAM3 dense 포크 / 라이브러리

ORB-SLAM3에 dense point cloud 스레드를 직접 붙인 공개 구현:
- **fishmarch/ORB-SLAM3-Dense** — 가장 단순·정석. "추정된 keyframe pose에서 depth를 역투영해 dense pointcloud 생성"이 정확히 목표 기법. ([repo](https://github.com/fishmarch/ORB-SLAM3-Dense))
- **YWL0720/YOLO_ORB_SLAM3_with_pointcloud_map** — dense cloud + YOLO 동적객체 제거. RGB-D, ROS Noetic 옵션. 단 dense 스레드 병목으로 **카메라 ~15Hz 이하** 제약. ([repo](https://github.com/YWL0720/YOLO_ORB_SLAM3_with_pointcloud_map))
- **Orwlit/ORB_SLAM3_Dense_Mapping**, **huashu996/ORB_SLAM3_Dense_YOLO** — dense + (octree/YOLO) 변형. ([Orwlit](https://github.com/Orwlit/ORB_SLAM3_Dense_Mapping))
- 계보 원형(성숙): **gaoxiang12/ORBSLAM2_with_pointcloud_map** — `PointCloudMapping` 스레드, KF마다 `generatePointCloud()`, `pcl::VoxelGrid`(기본 0.04m) 다운샘플, condition_variable로 트래킹 비차단. 단 **루프클로저 시 재정합 안 함**(insert 시점 pose로 stitch). 폐합 보정판: **ORB-SLAM2_RGBD_DENSE_MAP**. ([gaoxiang12](https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map))

> 표준 기법(역투영 스레드): KF의 최적화 pose `T_wc`로 각 픽셀 depth `z=D/1000`을 `x=(u-cx)z/fx, y=(v-cy)z/fy`로 역투영→`p_w=T_wc·p_c`+RGB→전역 cloud 누적→`VoxelGrid`(0.01~0.05m)+`StatisticalOutlierRemoval`. 트래킹과 별도 스레드. **루프클로저 시 KF별 cloud를 보관했다가 보정 pose로 재투영·재필터**해야 정합 유지(이게 핵심 한계).

### RGB-D fusion 라이브러리 (외부 pose 입력형) + ROS2

| 라이브러리 | 출력 | HW | ROS2 | 비고 |
|---|---|---|---|---|
| **nvblox / isaac_ros_nvblox** | TSDF mesh·ESDF·Nav2 costmap | **NVIDIA GPU/Jetson** | **1급(Humble)** | depth+외부 pose(tf/odom) 입력, ORB-SLAM3 pose 대체 가능. 실시간. standalone `donceykong/nvblox_ros2`. ([nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)) |
| **Voxblox** | TSDF·ESDF·mesh | CPU | **ROS1만** | TF에서 pose 조회+pointcloud 적분. 실시간 CPU. Humble은 브리지/포팅 필요. ([repo](https://github.com/ethz-asl/voxblox)) |
| **VDBFusion** (OpenVDB) | TSDF mesh | CPU | ROS1 래퍼 | `integrate(points, pose)` — pose 직접 입력, **오프라인 재투영에 이상적**(폐합 후 전체 재융합). ([repo](https://github.com/PRBonn/vdbfusion)) |
| **Open3D ScalableTSDFVolume** | colored mesh/cloud | CPU(+CUDA Tensor) | 라이브러리 | `volume.integrate(rgbd, intrinsic, inv(pose))` — ORB 궤적 파일 직접 소비. 오프라인·프로토타입·GT 메쉬. ([docs](https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html)) |
| **OctoMap (octomap_server2)** | occupancy octree | CPU | **ROS2** | photorealistic 아님. 경량 충돌/내비 맵. TF로 cloud 삽입. ([index](https://index.ros.org/p/octomap_server/)) |
| **InfiniTAM v3 / ElasticFusion** | TSDF/surfel | GPU | 없음 | 실시간+(일부)폐합 보정의 교본이나 ROS2·유지보수 부재. ([InfiniTAM](https://www.robots.ox.ac.uk/~victor/infinitam/)) |

참고 — **RTAB-Map**(이미 비교한 그 시스템)은 KF cloud를 그래프 노드에 묶어두고 폐합 후 **보정 pose로 재조립**하므로 단일 TSDF 융합기가 못 하는 루프클로저 정합을 사실상 무료로 제공. dense cloud·OctoMap·mesh export 지원, ROS2 1급. ([rtabmap](http://introlab.github.io/rtabmap/))

### Photorealistic 3DGS/NeRF (보조 맥락)

- **Photo-SLAM** (CVPR 2024) — **ORB-SLAM3 트래킹 위에 3D Gaussian Splatting**을 얹은 직계. mono/stereo/RGB-D, 실시간 GPU, Jetson AGX Orin 가능. 출력은 photorealistic 렌더링용 Gaussian 맵(워터타이트 mesh 아님). ([page](https://huajianup.github.io/research/Photo-SLAM/), [repo](https://github.com/HuajianUP/Photo-SLAM))
- MonoGS·SplaTAM·Gaussian-SLAM·NeRF-SLAM 등은 자체 photometric 트래킹/DROID 기반이라 ORB pose 재사용은 아님(강GPU 요구). mono densify는 CNN-SLAM/learned-depth(MiDaS·Depth-Anything) 경로. ([SplaTAM](https://spla-tam.github.io/), [CNN-SLAM](https://arxiv.org/abs/1704.03489))

### 결정 축 — 루프클로저 정합
- **재융합 안 함(fuse-once):** Voxblox·nvblox·VDBFusion·Open3D·OctoMap → 늦은 폐합 시 ghosting. 보정 pose로 재융합 필요(VDBFusion/Open3D는 오프라인이라 저렴).
- **submapping 정공법:** **Voxgraph**(Voxblox SDF submap + c-blox 그래프 최적화) — odom+cloud 입력(ORB 적합), 단 ROS1. ([voxgraph](https://github.com/ethz-asl/voxgraph))
- **내장 deformation:** ElasticFusion(surfel)·InfiniTAM submap — 연구용, ROS2 없음.

### RealSense + ROS2 통합 고려사항
- **aligned depth 사용**(`/camera/.../aligned_depth_to_color/image_raw`), depth=mm → `DepthMapFactor=1000`, **color 스트림 intrinsics**로 역투영(현 `orbslam3_d455f_640x480_rgbd.yaml`과 일치). ([RealSense_D435i.yaml](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/RGB-D-Inertial/RealSense_D435i.yaml))
- depth 3~4m 초과·0값 게이팅, dense 스레드가 throughput 병목(해상도/voxel/keyframe 조절), D435i면 RGBD-Inertial로 pose 품질↑→cloud 품질↑.
- ROS2+RealSense ORB-SLAM3 예: [HVKHVK/ORB_SLAM3_ROS2_REALSENSE](https://github.com/HVKHVK/ORB_SLAM3_ROS2_REALSENSE).

## Integration Patterns Analysis

> 여기서 "통합"은 dense 맵핑을 현 `orbslam_ws`(orbslam3_ros2 `rgbd_node`) + ROS2에 배선하는 방식. 데이터 인터페이스는 ROS2 topic/tf + PCL/PointCloud2.

### 현 시스템 진단 (코드 근거)

`orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp`는 **이미 dense 맵 기능을 내장**(`orbslam3_dense_map.pcd` 생성). 그러나:
- `rgbd_node.cpp:316` — `accumulateDenseMapIfNeeded(rgb, depth, Twc)`를 **매 트래킹 프레임**마다 호출하며, 사용하는 pose는 `Twc = Tcw.inverse()`, 즉 **그 순간의 live 트래킹 pose**(최적화 전).
- 즉 **keyframe 최적화 pose가 아니라 프레임 pose로 누적**하고, **BA/루프클로저로 pose가 보정돼도 dense cloud를 재융합하지 않음** → 늦은 보정이 반영 안 됨(drift·중복벽 가능).
- `frame_stride=5, pixel_stride=4, voxel=0.03m`로 decimate → "dense이긴 하나" RTAB-Map(0.9~3M점) 대비 덜 조밀.
- **결론:** RTAB-Map이 더 dense·정합적이었던 핵심 차이 = "KF 그래프 노드에 cloud를 묶어 보정 pose로 재조립". 같은 원리를 ORB 쪽에 적용하면 동등 품질 달성 가능.

### 패턴 A — In-process dense 스레드 강화 (권장, 최소 변경)

기존 `rgbd_node`를 개선:
1. **누적 기준을 frame→keyframe로 전환**: 매 프레임 대신 ORB-SLAM3 `System`의 keyframe 단위로 누적하고, **각 KF별 cloud를 KF id와 함께 보관**. (`fishmarch/ORB-SLAM3-Dense`, `gaoxiang12` `PointCloudMapping` 패턴)
2. **루프클로저/BA 후 재투영**: 맵 업데이트 시 보관한 KF cloud를 **보정된 `T_wc`로 재투영 후 전역 cloud 재구성 + VoxelGrid 재필터**. (이것이 RTAB-Map식 정합)
3. **밀도 상향**: `frame_stride 5→2`, `pixel_stride 4→2`, `voxel 0.03→0.01~0.02`, `StatisticalOutlierRemoval` 추가. depth 3~4m 게이팅 유지.
4. 데이터 인터페이스 불변: `/orbslam3/dense_map`(PointCloud2) 퍼블리시 + shutdown 시 pcd 저장. RViz2로 즉시 확인.
- 장점: 한 노드 안에서 완결, pose가 "공짜로" 정확, 실시간 가능. 단점: C++ 수정 필요, 메모리↑.

### 패턴 B — 외부 ROS2 fusion 노드 연동 (디커플링)

ORB-SLAM3는 **pose(tf/`nav_msgs/Odometry`)만 퍼블리시**, RealSense가 RGB-D 퍼블리시 → 외부 융합 노드가 둘을 구독해 dense 맵 생성:
- **nvblox**(`isaac_ros_nvblox` 또는 standalone `donceykong/nvblox_ros2`): depth + 외부 pose 입력 지원(Isaac VSLAM 자리에 ORB pose) → 실시간 TSDF mesh·ESDF·Nav2 costmap. **NVIDIA GPU/Jetson 필수**, ROS2 1급. ([nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox))
- **octomap_server2**: `/orbslam3/dense_map` 또는 depth→cloud를 tf로 삽입 → occupancy octree(내비용, 경량, CPU, ROS2). ([octomap_server](https://index.ros.org/p/octomap_server/))
- 배선: `rgbd_node`가 이미 `/orbslam3/pose`(PoseStamped) 퍼블리시 중 → tf broadcaster 추가하면 외부 노드가 즉시 활용. 장점: ORB 코드 거의 불변·모듈 교체 자유. 단점: 융합기 대부분 루프클로저 재정합 미지원(ghosting) → submapping(Voxgraph, ROS1) 필요.

### 패턴 C — 오프라인 재융합 (최고 품질, 현 평가 워크플로우와 합치)

SLAM은 pose만 쓰고, KF pose + depth(또는 KF별 RGB-D)를 덤프 → **Open3D `ScalableTSDFVolume`** 또는 **VDBFusion**으로 배치 융합:
- Open3D: 프레임마다 `volume.integrate(rgbd, intrinsic, inv(pose))` → colored mesh. ([Open3D RGBD integration](https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html))
- **루프클로저는 최종 최적화 pose로 전체 재융합**하면 자연 흡수(오프라인이라 저렴) → ghost 없음, 가장 깨끗한 mesh.
- 현재 8-run 산출물(trajectory.txt = TUM pose, bag의 RGB-D)과 직접 호환 → 별도 ROS 불필요, `rtabmap` env의 Open3D 설치만으로 가능. 장점: 최고 품질·GT 메쉬. 단점: 실시간 아님.

### 데이터 형식·인터페이스 요약
- pose: `geometry_msgs/PoseStamped`(현 `/orbslam3/pose`) / tf2 / `nav_msgs/Odometry`. 융합기는 보통 tf 조회.
- depth: `sensor_msgs/Image`(16UC1 mm, aligned_to_color) + color `camera_info` intrinsics.
- dense 맵: `sensor_msgs/PointCloud2`(현 `/orbslam3/dense_map`), PCL `PointXYZRGB`, 저장은 `.pcd`/`.ply`/mesh.

### 권장 매핑 (현 환경)
| 목적 | 권장 패턴 |
|---|---|
| 시각화 품질↑ 최소 노력 | **C (Open3D 오프라인 재융합)** — 현 산출물·env로 바로 |
| 실시간 dense + GPU 보유 | **B (nvblox)** |
| ORB 단일 노드 내 완결·실시간 | **A (KF 기반 + 폐합 재투영으로 강화)** |
| 내비게이션 맵 | B (octomap_server2) |


## 핵심 발견 및 권고 (Conclusion)

### 결정적 발견 — ORB-SLAM3는 이미 dense 맵을 생성하고 있었다
현 `orbslam3_ros2` 노드는 RGB-D dense 누적 기능을 내장하고 있고, 실제로 dense pcd를 생성해 두었다:

| dataset | ORB `orbslam3_dense_map.pcd` | (참고) ORB sparse `map_points.pcd` | RTAB `dense_map.pcd` |
|---|---:|---:|---:|
| one_lap | 440,666 | 12,516 | 930,057 |
| one_lap_back_and_forth | 569,763 | 12,900 | 1,755,368 |
| forward_backward_repeat | 596,378 | 14,739 | 2,052,602 |
| three_laps | 857,449 | 14,881 | 3,023,927 |

즉 직전 비교에서 ORB가 "안 보였던" 것은 알고리즘 한계가 아니라 **시각화에 sparse `map_points.pcd`(1.2~1.5만 점)를 썼기 때문**이며, dense 맵(44~86만 점)을 그리면 RTAB과 동급으로 구조가 드러난다. (구현으로 `dense_map_topview.png` 8장 생성·검증함.)

### 권고 (현 RealSense RGB-D + ROS2 환경)
1. **즉시(0 비용):** dense 시각화에 `orbslam3_dense_map.pcd` 사용 — 이미 생성됨. → 본 리서치에서 `slam_comparison_report/make_dense_view.py`로 구현 완료.
2. **품질 개선(권장, 패턴 A):** 노드의 dense 누적을 **프레임 live pose → keyframe 최적화 pose** 기준으로 바꾸고, **루프클로저/BA 후 KF cloud 재투영**(RTAB-Map 방식). `frame_stride`/`voxel` 상향. → 외곽 분사/중복벽 노이즈 제거, 더 조밀·정합.
3. **최고 품질(패턴 C):** ORB 최적화 trajectory + bag RGB-D를 Open3D/VDBFusion으로 오프라인 재융합, 최종 pose로 전체 재융합 → ghost 없는 mesh.
4. **실시간 + GPU:** 외부 `nvblox`(ROS2 1급) 연동(패턴 B).

### 구현 산출물
- `slam_comparison_report/make_dense_view.py` — ORB/RTAB dense pcd 컬러 Top-View(X–Z) 8장 (`<output>/<ds>/dense_map_topview.png`).

*리서치 + 구현 완료: 2026-06-17. 출처는 각 절의 인용 URL 참조.*
