# Story 0 SAM_circle Bag Usability Check

- bag path: `/home/ldh9501/temp_ws/CLI_environment/data_slam/rgbd_imu_sdk_bag/SAM_circle`
- resolved bag dir: `/home/ldh9501/temp_ws/CLI_environment/data_slam/rgbd_imu_sdk_bag/SAM_circle/bag`
- folder exists: yes
- metadata exists: yes
- ros2 bag info: failed/unavailable
- **verdict: 부분 사용 가능**

## Reasons
- RGB-D/camera_info present, but missing: SLAM pose / TF, SAM-6D object output

## Source Topic Summary

- RGB: yes
- depth: yes
- camera_info: yes
- IMU: yes
- /tf: no
- /tf_static: no
- SLAM pose topic: no
- SAM-6D object output: no

## Topic Table

| name | type | count | ts_min | ts_max | ts_samples |
| --- | --- | --- | --- | --- | --- |
| `/camera/camera/color/image_raw` | sensor_msgs/msg/Image | 1322 | 1782722649877759696 | 1782722694462134224 | 1782722649877759696, 1782722649912415440, 1782722649946696912, 1782722649981002704, 1782722650015283408 |
| `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/msg/Image | 1322 | 1782722649877759696 | 1782722694462134224 | 1782722649877759696, 1782722649912415440, 1782722649946696912, 1782722649981002704, 1782722650015283408 |
| `/camera/camera/color/camera_info` | sensor_msgs/msg/CameraInfo | 1322 | 1782722649877759696 | 1782722694462134224 | 1782722649877759696, 1782722649912415440, 1782722649946696912, 1782722649981002704, 1782722650015283408 |
| `/camera/camera/aligned_depth_to_color/camera_info` | sensor_msgs/msg/CameraInfo | 1322 | 1782722649877759696 | 1782722694462134224 | 1782722649877759696, 1782722649912415440, 1782722649946696912, 1782722649981002704, 1782722650015283408 |
| `/camera/camera/imu` | sensor_msgs/msg/Imu | 8988 | 1782722649597755088 | 1782722694485745872 | 1782722649597755088, 1782722649602832080, 1782722649607912912, 1782722649612996304, 1782722649618082768 |

## Policy Check

- fake folders created: no (no fake bag/dataset/SAM-output folders were created)

