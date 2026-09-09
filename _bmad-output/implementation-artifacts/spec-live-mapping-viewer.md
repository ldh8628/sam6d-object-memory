---
title: 'Live ORB-SLAM Mapping Viewer'
type: 'feature'
created: '2026-09-02'
status: 'done'
route: 'one-shot'
---

# Live ORB-SLAM Mapping Viewer

## Intent

**Problem:** 신규 두 카메라 녹화 중 ORB-SLAM3 맵 형성 상태를 확인할 실시간 창이 없었다.

**Approach:** 기존 Pangolin viewer를 opt-in으로 재사용하고, ORB 준비 후 녹화를 시작하며 최종 Atlas/URDF 후처리는 그대로 유지한다.

## Suggested Review Order

**Live capture entry point**

- 실시간 전용 설정을 생성하고 불필요한 ROS map-point 복사를 끈다.
  [`create_map_urdf.py:97`](../../integration/create_map_urdf.py#L97)

- ORB READY를 확인한 뒤에만 원본 녹화를 시작한다.
  [`create_map_urdf.py:279`](../../integration/create_map_urdf.py#L279)

**Viewer lifecycle**

- 기본값은 꺼 둔 채 설정으로만 Pangolin viewer를 활성화한다.
  [`rgbd_node.cpp:97`](../../orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp#L97)

- 종료 전에 viewer 읽기 스레드를 멈춰 Atlas 저장 경쟁을 막는다.
  [`System.cc:526`](../../orbslam_ws/src/ORB_SLAM3/src/System.cc#L526)

**Configuration and check**

- launch 설정을 ROS node parameter까지 전달한다.
  [`orb_slam.launch.py:234`](../../orbslam_ws/src/orbslam3_ros2/launch/orb_slam.launch.py#L234)

- 생성 설정의 rate 1.0과 viewer/resource 계약을 자체검사한다.
  [`create_map_urdf.py:423`](../../integration/create_map_urdf.py#L423)
