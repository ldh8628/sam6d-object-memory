# Technical Research: SLAM-Based Camera Pose Stabilization for SAM-6D Pose Jitter Reduction

Date: 2026-06-10
Language: Korean

## Executive Conclusion

정지 객체의 SAM-6D pose jitter 문제에서 SLAM을 붙이는 것만으로는 충분하지 않다. 핵심 관계는 다음이다.

```text
T_WO(t) = T_WC(t) * T_CO(t)
```

여기서 `T_WC`는 SLAM camera/world pose이고, `T_CO`는 SAM-6D가 매 frame 추정하는 object-in-camera pose이다. 고정 카메라 또는 저속 카메라 환경에서 `T_WC`가 안정적이어도 `T_CO`가 흔들리면 `T_WO`도 흔들린다. 따라서 단순히 ORB-SLAM3/RTAB-Map/hdl_graph_slam의 pose를 곱하는 구조는 camera pose noise만 줄이고, SAM-6D의 object measurement noise는 줄이지 못한다.

가장 높은 성공 확률의 실무 해법은 `SAM-6D -> One Euro Filter/EKF -> publish`이다. SLAM을 쓴다면 목적은 pose smoothing 자체가 아니라 `T_WO`를 static object landmark state로 유지하는 것이다. 이 경우 권장 구조는 `RTAB-Map 또는 ORB-SLAM3 camera pose + SAM-6D T_CO -> external object landmark filter/factor graph -> stable T_WO`이다.

## 1. Repository Analysis

### 1.1 조사한 실제 로컬 경로

사용자가 지정한 정확한 경로 `~/CLI_environment/ORB_SLAM3`, `~/CLI_environment/hdl_graph_slam`, `~/CLI_environment/rtabmap*`는 일부 존재하지 않았다. 실제 코드 위치는 다음과 같다.

| System | 실제 로컬 코드 경로 | 비고 |
|---|---|---|
| ORB-SLAM3 core | `/home/etri/CLI_environment/orbslam_ws/src/ORB_SLAM3` | RGB-D 지원 core |
| ORB-SLAM3 ROS2 wrapper | `/home/etri/CLI_environment/orbslam_ws/src/orbslam3_ros2` | `/orbslam3/pose`, `/orbslam3/odom`, TF publish |
| hdl_graph_slam | `/home/etri/CLI_environment/hdlgraphslam_ws/src/hdl_graph_slam` | ROS2 포팅/구현 |
| RTAB-Map core | `/home/etri/rtabmap` | CLI_environment 밖에 존재 |
| RTAB-Map ROS2 | `/home/etri/rtabmap_ws/src/rtabmap_ros` | CLI_environment 밖에 존재 |
| RTAB config | `/home/etri/rtabmap_ws/config/tiers_indoor02_sauna_rtabmap.yaml` | RGB-D camera topic 설정 |

### 1.2 ORB-SLAM3

Local evidence:
- ROS2 wrapper: `/home/etri/CLI_environment/orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp`
- Core tracking: `/home/etri/CLI_environment/orbslam_ws/src/ORB_SLAM3/src/Tracking.cc`
- Optimizer declarations: `/home/etri/CLI_environment/orbslam_ws/src/ORB_SLAM3/include/Optimizer.h`

| 항목 | 분석 |
|---|---|
| Localization 방식 | RGB-D frame을 `slam_->TrackRGBD(rgb, depth, timestamp)`에 넣고 `Tcw`를 얻는다. wrapper는 `Twc = Tcw.inverse()`를 publish한다. |
| State Representation | live pose는 `Sophus::SE3f`; loop/merge에는 `Sim3`; map은 keyframe/MapPoint 기반 pose graph + BA. |
| Motion Model | 있음. `Tracking.cc`에서 velocity가 있으면 `TrackWithMotionModel()`, 실패하면 reference keyframe tracking fallback. |
| Temporal Smoothing | 명시적 low-pass smoothing 없음. 안정성은 motion model, local map tracking, pose optimization, local BA, relocalization에서 나온다. |
| Loop Closure | 있음. place recognition, loop/merge, essential graph/global BA 계열. |
| Pose Optimization | `PoseOptimization`, `LocalBundleAdjustment`, `BundleAdjustment`, `OptimizeEssentialGraph`, `OptimizeSim3`. |
| Pose Covariance | ROS2 wrapper에서 covariance를 채우지 않음. `/orbslam3/odom` covariance는 기본값에 가깝다. |
| ROS Topic/API | `/orbslam3/pose`, `/orbslam3/odom`, `/orbslam3/tracking_state`, TF `world_frame -> camera_frame`, map points. |
| 실시간 pose stability 확보 | many feature/map point constraints + motion model + BA. 그러나 publish pose 자체에 jitter filter는 없음. |

Code points:
- `rgbd_node.cpp:181-193`: `/orbslam3/tracking_state`, `/orbslam3/pose`, `/orbslam3/odom` publisher 생성.
- `rgbd_node.cpp:300-315`: `TrackRGBD` 호출 후 `Tcw.inverse()`를 publish.
- `Tracking.cc:1932-1956`: motion model/reference keyframe 기반 tracking 분기.

판단:
- SAM-6D와 결합하기 쉬운 camera pose source이다.
- 그러나 covariance가 없어서 EKF/factor graph의 camera prior weighting을 직접 하려면 tracking status, reprojection residual, inlier 수 등 core statistic 확장이 필요하다.
- object jitter를 줄이려면 ORB-SLAM3 내부 BA를 고치기보다 외부 object landmark filter/factor graph가 낮은 위험이다.

### 1.3 hdl_graph_slam

Local evidence:
- Scan matching odometry: `/home/etri/CLI_environment/hdlgraphslam_ws/src/hdl_graph_slam/src/hdl_graph_slam_ros2/scan_matching_odometry_node.cpp`
- Graph core: `/home/etri/CLI_environment/hdlgraphslam_ws/src/hdl_graph_slam/include/hdl_graph_slam/graph_slam.hpp`
- Graph node: `/home/etri/CLI_environment/hdlgraphslam_ws/src/hdl_graph_slam/src/hdl_graph_slam_ros2/graph_slam_node.cpp`

| 항목 | 분석 |
|---|---|
| Localization 방식 | 3D LiDAR point cloud scan matching odometry + keyframe + graph SLAM. |
| State Representation | `Eigen::Isometry3d`, `g2o::VertexSE3`, `g2o::EdgeSE3`; plane/point prior constraints. |
| Motion Model | formal constant velocity model은 아님. 이전 scan transform `prev_trans_`를 registration initial guess로 사용. |
| Temporal Smoothing | 명시적 filter 없음. keyframe matching, threshold rejection, graph optimization으로 안정화. |
| Loop Closure | 있음. `graph_slam_node`에서 loop detector 결과를 SE3 edge로 추가. |
| Pose Optimization | g2o sparse optimizer, SE3 edge, plane/floor/GPS/IMU prior edge. |
| Pose Covariance | `scan_matching_odometry_node` publish odometry에는 covariance를 채우지 않음. |
| ROS Topic/API | 기본 `/odom`, `/scan_matching_odometry/transform`, `/scan_matching_odometry/status`, TF `odom -> base_frame`. |
| 실시간 pose stability 확보 | NDT/ICP scan matching, keyframe threshold, too-large transform rejection, periodic graph optimization. |

Code points:
- `scan_matching_odometry_node.cpp:185-224`: keyframe target에 현재 scan을 align하고 odom 계산.
- `scan_matching_odometry_node.cpp:226-233`: 너무 큰 delta transform reject.
- `scan_matching_odometry_node.cpp:274-282`: odometry publish, covariance 없음.
- `graph_slam.hpp:48-72`: SE3 node/edge.
- `graph_slam.hpp:92-111`: SE3-point, SE3 prior edge.

판단:
- SAM-6D의 RGB-D camera와 같은 sensor frame이 아니다. LiDAR/base pose를 쓰려면 `T_WB * T_BC * T_CO`가 필요하고, camera-LiDAR extrinsic 및 timestamp sync 오차가 새 jitter source가 된다.
- hdl_graph_slam은 robust global base pose에는 유용하지만, SAM-6D camera-frame jitter 제거 목적에는 세 시스템 중 가장 직접성이 낮다.

### 1.4 RTAB-Map

Local evidence:
- ROS odometry: `/home/etri/rtabmap_ws/src/rtabmap_ros/rtabmap_odom/src/OdometryROS.cpp`
- ROS SLAM wrapper: `/home/etri/rtabmap_ws/src/rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp`
- Core links: `/home/etri/rtabmap/corelib/include/rtabmap/core/Link.h`
- Core optimizer: `/home/etri/rtabmap/corelib/include/rtabmap/core/Optimizer.h`
- Local config: `/home/etri/rtabmap_ws/config/tiers_indoor02_sauna_rtabmap.yaml`

| 항목 | 분석 |
|---|---|
| Localization 방식 | RGB-D/stereo/LiDAR graph-based SLAM. 현재 local config는 RGB-D camera topic을 사용한다. |
| State Representation | `rtabmap::Transform` SE3-like transform, graph node/signature, `Link` constraints, optimizer backend TORO/g2o/GTSAM/Ceres/CVSBA. |
| Motion Model | odometry process에 TF guess를 넣을 수 있고, velocity guess/twist publish 경로가 있음. |
| Temporal Smoothing | explicit output low-pass filter는 아님. F2M/F2F odometry, local map, loop closure, graph optimization이 temporal consistency 역할. |
| Loop Closure | 있음. appearance-based loop closure가 graph constraint를 만들고 optimizer가 graph error를 줄인다. |
| Pose Optimization | `Optimizer::optimize`, graph links, loop closure, landmark, pose prior constraints. |
| Pose Covariance | odom covariance와 localization covariance publish 경로가 있음. |
| ROS Topic/API | config 기준 `/rtabmap/pose`, `/rtabmap/cloud_map`; CoreWrapper에는 `localization_pose`; odometry는 `nav_msgs/Odometry` + TF. |
| 실시간 pose stability 확보 | RGB-D odometry covariance, graph correction `mapToOdom`, localization covariance, loop/proximity/landmark detection. |

Code points:
- `tiers_indoor02_sauna_rtabmap.yaml:1-13`: `/cam_1/aligned/color/image_raw`, `/cam_1/aligned/depth/image_rect_raw`, frame `cam_1_depth_optical_frame`, `/rtabmap/pose`.
- `OdometryROS.cpp:747-750`: `odometry_->process(data, guess_, &info)`.
- `OdometryROS.cpp:818-825`: `info.reg.covariance`를 odometry pose covariance에 반영.
- `CoreWrapper.cpp:2293-2298`: `rtabmap_.process(...)` 후 `mapToOdom_ = rtabmap_.getMapCorrection()`.
- `CoreWrapper.cpp:2324-2343`: `localization_pose`와 localization covariance publish.
- `Link.h:49-64`: `kPosePrior`, `kLandmark`, information matrix.
- `Optimizer.h:64-70`: TORO, g2o, GTSAM, Ceres, CVSBA optimizer type.

판단:
- 세 시스템 중 SAM-6D RGB-D pipeline과 가장 잘 맞는다. 같은 RGB-D camera topic/frame을 쓰고 covariance도 ROS message에 나타난다.
- RTAB-Map core에는 landmark link가 있지만, SAM-6D 6D object pose를 직접 RTAB graph의 landmark로 넣는 것은 core/DB/signature modification이 필요하다. 1차 구현은 외부 temporal consistency node가 더 현실적이다.

## 2. Localization Method Comparison

| System | Sensor/Frame 적합성 | State | Optimization | Loop Closure | Covariance | SAM-6D 결합 난이도 | Jitter 문제 적합성 |
|---|---:|---|---|---|---|---:|---:|
| ORB-SLAM3 | 높음, RGB-D camera | SE3 + Sim3 + keyframe map | pose opt/local BA/global BA | 있음 | wrapper 없음 | 중 | 중상 |
| hdl_graph_slam | 낮음, LiDAR/base 중심 | SE3 graph + plane/point | g2o graph opt | 있음 | odom 없음 | 높음 | 낮음 |
| RTAB-Map | 높음, 현재 RGB-D config 존재 | SE3 graph + Link constraints | TORO/g2o/GTSAM 등 | 있음 | odom/localization covariance 있음 | 중 | 상 |

문헌 기준으로도 ORB-SLAM3는 RGB-D를 포함한 visual/visual-inertial multi-map SLAM이고 MAP estimation, loop/merge, BA를 통해 real-time robust localization을 목표로 한다. hdl_graph_slam은 3D LiDAR real-time 6DOF graph SLAM이며 NDT scan matching odometry와 loop detection 및 GPS/IMU/floor constraints를 제공한다. RTAB-Map은 RGB-D/stereo/LiDAR graph-based SLAM으로 loop closure constraint를 graph에 추가하고 graph optimizer가 error를 줄이는 구조다.

## 3. Jitter Root Cause Analysis

### 3.1 수식

```text
T_WO(t) = T_WC(t) * T_CO(t)

where:
T_WO: object pose in world/map frame
T_WC: camera pose in world/map frame from SLAM
T_CO: object pose in camera frame from SAM-6D
```

작은 perturbation을 Lie algebra로 보면 다음처럼 근사할 수 있다.

```text
delta_xi_WO ~= Ad_(T_CO^-1) delta_xi_WC + delta_xi_CO
```

즉 world object pose jitter는 camera pose jitter와 object measurement jitter가 모두 만든다. 하지만 고정 카메라/정지 객체 환경에서 `T_WC`가 거의 고정인데 `T_CO`가 frame마다 바뀐다면 원인은 대부분 SAM-6D measurement noise이다.

### 3.2 왜 SLAM pose는 안정적인데 SAM-6D object pose는 흔들리는가

SLAM은 매 frame 하나의 object crop만 보는 것이 아니라 다수의 배경 feature, map point, scan point, keyframe, loop constraint를 시간축으로 누적한다. ORB-SLAM3는 motion model + local map + BA, hdl_graph_slam은 keyframe scan matching + graph optimization, RTAB-Map은 visual odometry + loop closure graph optimization을 사용한다.

반면 SAM-6D는 기본적으로 frame-wise object pose estimator이다. mask proposal, RGB-D crop, depth noise, point matching, correspondence score, symmetry/texture ambiguity가 frame마다 조금씩 달라지면 `T_CO`가 흔들린다. 이 흔들림은 SLAM의 `T_WC`를 곱해도 사라지지 않는다.

### 3.3 현재 문제의 원인 분류

| 환경 | 주요 원인 | 설명 |
|---|---|---|
| 카메라 완전 고정, 객체 정지 | Object Pose Noise | `T_WC`는 constant에 가깝고 `T_CO`가 흔들림. |
| 카메라 저속 이동, SLAM 안정 | Mostly Object Pose Noise + small Camera Pose Noise | SLAM tracking noise가 일부 있지만 SAM-6D measurement noise가 더 큼. |
| 카메라 저속 이동, SLAM unstable/lost | Both | `T_WC`와 `T_CO`가 모두 흔들림. |
| hdl_graph_slam LiDAR pose를 camera pose로 간접 사용 | Both + extrinsic/time sync noise | `T_WB * T_BC * T_CO`에서 `T_BC`/timestamp 오차가 추가됨. |

## 4. Fusion Architecture Comparison

정량값은 로컬 코드 구조와 일반 ROS2 실시간 budget을 기준으로 한 예상치이다. 실제 수치는 동일 bag에서 raw/filtered pose stddev로 검증해야 한다.

| Architecture | 구조 | 예상 성능 | 구현 난이도 | FPS 영향 | 예상 Jitter 감소량 | 판단 |
|---|---|---:|---:|---:|---:|---|
| A | SAM-6D -> One Euro Filter -> Publish | 정지/저속에 매우 좋음, lag 작음 | 매우 낮음 | <1% | 40-70% | 내일 구현 1순위 |
| B | SAM-6D -> EKF -> Publish | covariance/gating 가능, 정지 prior 쉬움 | 낮음-중 | <2% | 50-75% | 실무 2순위 |
| C | ORB-SLAM3 Pose + SAM-6D Pose -> Factor Graph -> Stable Object Pose | world-frame stability 높음 | 중-상 | 3-8% | 60-85% | 연구/1-3주 |
| D | HDL Graph SLAM Pose + SAM-6D Pose -> Graph Optimization | LiDAR-base 안정성은 좋지만 camera 결합 리스크 큼 | 상 | 5-15% | 30-65% | 현재 문제에는 낮은 우선순위 |
| E | RTAB-Map Pose + SAM-6D Pose -> Temporal Consistency Layer | RGB-D frame/covariance가 좋아 실용적 | 중 | 3-8% | 55-80% | SLAM fusion 실무 1순위 |
| F | SLAM World Frame + Object Landmark Tracking -> Stable Object Pose | 정지 객체 조건을 가장 직접 활용 | 상 | 5-10% | 70-90% | 석사 논문 주제 1순위 |

### 4.1 Architecture A/B가 SLAM보다 먼저인 이유

객체가 정지하고 camera-frame pose를 publish하는 시스템이라면 jitter는 `T_CO`에 있다. 따라서 `T_CO` 자체를 SE3-aware smoothing하는 것이 가장 직접적이다. One Euro Filter는 translation/rotation 각각에 low-latency cutoff를 적용할 수 있고, EKF는 state를 `x=[p, q, v, omega]` 또는 정지 객체라면 `x=[p, q]`로 두고 measurement update만 적용할 수 있다.

### 4.2 Architecture C/E/F의 핵심 factor

SLAM을 의미 있게 쓰려면 object landmark `T_WO`를 state로 둔다.

```text
Variables:
  X_t = T_WC(t)    camera pose from SLAM, fixed or prior
  O   = T_WO       static object landmark pose

Measurement:
  Z_t = T_CO^SAM(t)

Residual:
  r_t = Log( Z_t^-1 * (X_t^-1 * O) )

Static prior:
  O_t = O_(t-1), or one persistent landmark O
```

Camera pose를 fixed로 놓으면 계산량이 작다. Camera pose까지 optimize하면 더 정확할 수 있지만 SLAM core와 충돌하고 latency가 늘어난다. ROS2 실시간에서는 first version을 fixed camera prior + object landmark update로 시작하는 것이 맞다.

## 5. Related Research Summary

| Work/System | 관련성 | 이 조사에 주는 결론 |
|---|---|---|
| ORB-SLAM3 | RGB-D 포함 visual/visual-inertial multi-map SLAM, MAP/BA 기반 real-time localization | camera pose prior로 좋지만 object jitter는 직접 해결하지 않음 |
| hdl_graph_slam | 3D LiDAR graph SLAM, NDT odometry, loop/floor/GPS/IMU constraints | base/LiDAR pose 안정화용. SAM-6D camera pose 안정화에는 extrinsic/sync 부담 큼 |
| RTAB-Map | RGB-D/stereo/LiDAR graph SLAM, loop closure + graph optimizer | 현재 RGB-D SAM-6D와 frame/source가 가장 잘 맞는 SLAM fusion 후보 |
| SAM-6D | zero-shot 6D pose, segmentation + pose estimation, frame-wise structure | temporal consistency가 기본 목적이 아니므로 jitter layer 필요 |
| FoundationPose | pose estimation과 tracking을 통합, RGB-D tracking에서 temporal cue 활용 | SAM-6D 후처리보다 큰 구조 변경이지만 tracking 방식의 장점 확인 |
| BundleSDF/BundleTrack 계열 | object-centric RGB-D video tracking/reconstruction | 정확도/안정성은 높지만 연산량과 시스템 변경이 큼 |
| TemporalPose | single-view pose estimator의 temporal inconsistency를 factor graph로 해결 | 본 문제의 thesis-level 방법과 가장 직접적으로 맞음 |
| SLAM-Super-6D | robust pose graph optimization으로 6D object pose 학습/보정에 SLAM 활용 | SLAM은 단순 smoothing이 아니라 pose graph/uncertainty 모델과 결합할 때 의미가 큼 |
| Object-level/Semantic SLAM | object를 landmark로 두고 camera/object pose를 함께 최적화 | 정지 객체 jitter reduction의 이론적 기반 |

검색 기준으로 직접적인 공개 사례 `SAM-6D + ORB-SLAM3/RTAB-Map으로 jitter reduction`은 강하게 확인되지 않았다. 가장 가까운 연구 방향은 temporal consistent 6D pose factor graph, object-level SLAM, SLAM-assisted 6D pose learning/fusion이다.

## 6. Implementation Difficulty Ranking

쉬운 순서:

1. SAM-6D output에 One Euro Filter 적용
2. SAM-6D output에 SE3 EMA 적용
3. SAM-6D output에 EKF 적용, static object process noise를 작게 설정
4. ORB-SLAM3 또는 RTAB-Map pose를 곱해 `T_WO_raw` publish/debug
5. RTAB-Map `/rtabmap/pose` 또는 `localization_pose` covariance를 사용한 object landmark EKF
6. ORB-SLAM3 `/orbslam3/pose`를 사용한 object landmark factor graph
7. RTAB-Map core landmark link에 SAM object를 직접 넣기
8. hdl_graph_slam graph에 6D object landmark factor 추가
9. ORB-SLAM3 core BA에 object landmark state 추가
10. Neural temporal fusion/SSM/transformer tracking layer 학습

## 7. Expected Jitter Reduction Ranking

효과가 큰 순서:

1. SLAM world frame + static object landmark factor graph: 70-90%
2. RTAB-Map/ORB-SLAM3 camera pose + object landmark EKF: 60-85%
3. EKF with static prior directly on SAM-6D `T_CO`: 50-75%
4. One Euro Filter on SE3 pose: 40-70%
5. SE3 EMA: 35-65%
6. ORB-SLAM3 pose multiply only: 0-30%, camera motion noise가 있을 때만 의미 있음
7. RTAB-Map pose multiply only: 10-30%, world frame에서만 의미 있음
8. hdl_graph_slam pose multiply only: 0-20%, extrinsic/sync가 나쁘면 악화 가능

## 8. Recommended Architecture

### 8.1 내일 바로 구현해야 한다면

Architecture A/B를 적용한다.

```text
SAM-6D PoseArray / TF
  -> object id별 SE3 One Euro Filter
  -> optional static hold gate
  -> filtered PoseArray / TF
```

필수 구현 포인트:
- translation은 x/y/z 각각 One Euro 또는 EMA.
- rotation은 quaternion sign correction 후 SO(3) log map에서 smoothing하거나 slerp 기반 EMA.
- object id별 state 유지.
- detection recall은 건드리지 않는다. measurement를 drop하지 않고 pose만 smooth한다.
- confidence가 낮을 때는 update gain만 낮추고 publish는 마지막 stable pose를 TTL 동안 유지한다.
- metric: translation stddev, rotation geodesic stddev, latency, FPS.

### 8.2 SLAM을 붙여야 한다면

RTAB-Map 또는 ORB-SLAM3 pose를 사용해 world-frame object landmark layer를 외부 node로 만든다.

```text
RGB-D -> SAM-6D -> T_CO(t)
RGB-D -> RTAB-Map or ORB-SLAM3 -> T_WC(t)

T_WO_raw(t) = T_WC(t) * T_CO(t)
T_WO_hat   = StaticObjectEKF/FactorGraph(T_WO_raw, covariance, robust gate)

publish:
  /sam6d/object_pose_world_raw
  /sam6d/object_pose_world_filtered
  /sam6d/object_pose_camera_filtered = T_CW(t) * T_WO_hat
```

권장 SLAM 선택:
- 1순위: RTAB-Map. 이유는 local config가 SAM-6D와 같은 RGB-D camera stream/frame을 쓰고, odometry/localization covariance 경로가 있다.
- 2순위: ORB-SLAM3. 이유는 RGB-D camera pose가 빠르고 안정적이나 covariance가 없다.
- 3순위: hdl_graph_slam. 이유는 LiDAR/base pose라 camera extrinsic/time sync 리스크가 크다.

## 9. Immediate Implementation Plan

### Phase 1: Low-risk, 1 day

1. SAM-6D PoseArray/TF raw topic을 그대로 보존한다.
2. `sam6d_pose_stabilizer` ROS2 node를 추가한다.
3. object id별 One Euro Filter 또는 SE3 EMA를 적용한다.
4. raw/filtered를 동시에 publish한다.
5. 정지 객체 bag에서 다음 metric을 자동 계산한다.
   - translation stddev mm
   - rotation geodesic stddev degree
   - mean latency ms
   - dropped/reinitialized frame count
   - FPS 변화

Expected:
- 코드 변경 작음.
- SAM-6D recall 영향 없음.
- FPS 영향 <1%.
- 즉시 체감 가능한 jitter 감소.

### Phase 2: 1-2 weeks

1. RTAB-Map 또는 ORB-SLAM3 pose를 구독한다.
2. `T_WO_raw = T_WC * T_CO`를 publish한다.
3. RTAB-Map covariance가 있으면 object EKF measurement covariance에 반영한다.
4. static object prior를 둔다.

```text
O_k = O_(k-1) + w,  w ~ N(0, Q_static)
Z_k = Log( T_WO_raw^-1 * O_k ) + v
```

5. robust gate는 outlier 제거가 아니라 measurement gain 감소용으로 사용한다. recall/publish 유지.

Expected:
- world-frame object pose가 안정화된다.
- 저속 camera motion에서도 유리하다.
- FPS 영향 2-5%.

### Phase 3: Thesis-level

1. Sliding-window factor graph를 구성한다.
2. Variables:
   - fixed/prior camera poses `T_WC(t)`
   - static object pose `T_WO`
   - optional object velocity/twist
3. Factors:
   - SAM-6D observation factor: `r_t = Log(Z_t^-1 * (T_WC(t)^-1 * T_WO))`
   - static object prior
   - robust kernel for wrong pose hypotheses
   - covariance model from SAM-6D score/depth residual/mask quality
4. Compare:
   - no filter
   - One Euro
   - EKF
   - RTAB/ORB fixed camera prior + object factor graph
   - optional hdl_graph_slam base pose branch

Expected contribution:
- "Static object-aware temporal consistency layer for zero-shot RGB-D 6D pose estimation"으로 석사 논문 수준 신규성 확보 가능.
- SAM-6D backbone은 변경하지 않고 post-estimation optimizer를 제안할 수 있다.

## 10. Thesis-Level Research Direction

### A. 내일 바로 구현 가능한 방법

`SAM-6D -> SE3 One Euro Filter/EKF -> publish`. SLAM 없이 시작한다. 문제의 주원인이 `T_CO`이므로 가장 빠르고 정확한 첫 조치다.

### B. 1-2주 구현

`RTAB-Map/ORB-SLAM3 pose + SAM-6D pose -> static object EKF`. `T_WO`를 persistent state로 유지하고, 필요하면 `T_CW * T_WO_hat`로 camera-frame filtered pose도 publish한다.

### C. 석사 논문 수준

`SLAM-informed static object factor graph`. 핵심 기여는 SAM-6D measurement uncertainty, static prior, robust temporal association, real-time sliding window optimization이다.

### D. SCI 논문 수준

SAM-6D/FoundPose 계열 zero-shot estimator의 uncertainty를 학습하거나, object-centric SSM/temporal transformer를 factor graph와 결합하여 정지/저속/occlusion/ambiguous symmetry 상황을 모두 다루는 방법. 단, 현재 시스템 목표인 FPS 악화 5-10% 이내를 만족시키려면 distillation 또는 lightweight online optimizer가 필요하다.

## Final Q&A

### Q1. ORB-SLAM3를 붙이면 실제로 jitter가 감소하는가?

단순히 `T_WO = T_WC^ORB * T_CO^SAM`만 publish하면 제한적으로만 감소한다. 카메라 pose noise 성분은 줄지만 SAM-6D의 camera-frame object noise는 그대로 남는다. 고정 카메라에서는 감소폭이 작다. object landmark EKF/factor graph까지 넣으면 감소한다.

### Q2. hdl_graph_slam을 붙이면 실제로 jitter가 감소하는가?

직접적이지 않다. hdl_graph_slam은 LiDAR/base pose 안정화 시스템이다. SAM-6D RGB-D camera와 fusion하려면 `T_WB * T_BC * T_CO`가 필요하고, extrinsic/timestamp 오차가 추가된다. 현재 문제 해결용으로는 우선순위가 낮다.

### Q3. RTAB-Map을 붙이면 실제로 jitter가 감소하는가?

세 SLAM 중 가능성이 가장 높다. 현재 config가 같은 RGB-D camera topic/frame을 쓰고 covariance publish 경로가 있다. 다만 RTAB-Map pose를 곱하는 것만으로는 부족하고, static object landmark layer가 필요하다.

### Q4. 세 개 중 어떤 SLAM이 가장 적합한가?

RTAB-Map > ORB-SLAM3 > hdl_graph_slam. RTAB-Map은 RGB-D pipeline과 covariance가 장점이다. ORB-SLAM3는 빠르고 camera pose가 좋지만 covariance가 없다. hdl_graph_slam은 sensor frame mismatch가 크다.

### Q5. SLAM보다 One Euro Filter / EKF / Factor Graph가 더 효과적인가?

현재 문제에는 그렇다. jitter의 주 원인이 `T_CO`이면 SLAM은 직접 원인을 건드리지 못한다. One Euro/EKF는 바로 `T_CO`를 안정화한다. 최고 성능은 SLAM camera pose를 보조 prior로 쓰는 object factor graph다.

### Q6. 가장 높은 성공 확률의 개발 로드맵은?

1. Day 1: SAM-6D output SE3 One Euro/EKF stabilizer.
2. Week 1: raw/filtered metric logger와 정지 객체 benchmark bag 구축.
3. Week 2: RTAB-Map 또는 ORB-SLAM3 `T_WC` 결합, `T_WO` static object EKF.
4. Month 1-2: sliding-window factor graph, uncertainty model, robust temporal association.
5. Thesis: SAM-6D 변경 없이 zero-shot RGB-D 6D pose의 static object temporal consistency layer 제안 및 평가.

## References

- ORB-SLAM3: multi-map visual/visual-inertial SLAM with RGB-D support, MAP estimation and BA.
- hdl_graph_slam: 3D LiDAR graph SLAM with NDT scan matching odometry, loop detection, GPS/IMU/floor constraints.
- RTAB-Map: RGB-D/stereo/LiDAR graph-based SLAM with loop closure constraints and graph optimization.
- SAM-6D: CVPR 2024 zero-shot 6D pose via SAM-based instance segmentation and pose estimation.
- FoundationPose: unified 6D pose estimation and tracking of novel objects with RGB-D temporal tracking.
- TemporalPose: factor graph approach for temporally consistent object 6D pose estimation for robot control.
- SLAM-Super-6D: SLAM-supported semi-supervised 6D pose estimation using robust pose graph optimization.
