# Object Memory Run Report

- slam trajectory: `../../slam_comparison/output/SAM_occlusion/orbslam3_noimu/CameraTrajectory.txt`
- pem dir: `../../sam6d_ws/outputs/pem_inputs/SAM_occlusion`
- bag: `../../data_slam/rgbd_imu_sdk_bag/SAM_occlusion`

## Formula

`T_map_obj = T_map_cam (SLAM) * T_cam_obj (SAM-6D)`  [translations in metres]

## Run Summary

- SLAM poses loaded: 795
- bag color timestamps: 795
- frames with detections: 53
- frames fused (SLAM ok): 53
- frames skipped (no SLAM pose): 0
- total detections: 105

## Persistent Objects (object_memory owns object_id)

| object_id | object_name | status | obs | missed | confidence | T_map_obj xyz (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Bear | active | 51 | 0 | 1.00 | (0.219, 0.262, 1.056) |
| 2 | Rabbit | active | 51 | 0 | 1.00 | (0.222, 0.258, 1.058) |
| 3 | milk | lost | 3 | 9 | 0.00 | (-0.188, 0.402, 3.916) |

## Lifecycle Transitions

| object_id | from -> to | reason | stamp |
| --- | --- | --- | --- |
| 1 | tentative -> active | promoted | 1782722484.202 |
| 2 | tentative -> active | promoted | 1782722484.202 |
| 3 | tentative -> active | promoted | 1782722505.382 |
| 3 | active -> lost | missed_threshold | 1782722506.383 |
| 3 | lost -> active | redetected | 1782722507.051 |
| 3 | active -> lost | missed_threshold | 1782722508.386 |

## Decision Summary

- new_tentative: 3
- short_term_match: 102

