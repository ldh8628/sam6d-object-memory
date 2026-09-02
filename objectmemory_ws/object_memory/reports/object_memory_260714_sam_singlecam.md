# Object Memory Run Report

- slam trajectory: `/home/ldh9501/temp_ws/CLI_environment/orbslam_ws/output/260714/sam_110104/CameraTrajectory.txt`
- pem dir: `/home/ldh9501/temp_ws/CLI_environment/sam6d_ws/outputs/pem_inputs/sam_110104`
- bag: `/home/ldh9501/temp_ws/CLI_environment/data_slam/260714_frame_data/converted/sam_110104`

## Formula

`T_map_obj = T_map_cam (SLAM) * T_cam_obj (SAM-6D)`  [translations in metres]

## Run Summary

- SLAM poses loaded: 2731
- bag color timestamps: 547
- frames with detections: 21
- frames fused (SLAM ok): 21
- frames skipped (no SLAM pose): 0
- total detections: 51

## Persistent Objects (object_memory owns object_id)

| object_id | object_name | status | obs | missed | confidence | T_map_obj xyz (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Dinosaur | deleted | 4 | 4 | 0.04 | (-2.709, 0.429, -1.142) |
| 2 | Sauce_high | lost | 4 | 7 | 0.05 | (-2.404, 0.450, -1.435) |
| 3 | choco_hazelnut_high | deleted | 2 | 5 | 0.04 | (-2.386, 0.447, -0.976) |
| 4 | Febreze_high | active | 6 | 2 | 0.94 | (-2.797, 0.430, -1.332) |
| 5 | Rabbit | deleted | 1 | 4 | 0.02 | (-2.370, 0.437, -1.210) |
| 6 | Bear | lost | 5 | 4 | 0.25 | (-1.056, 0.400, -1.207) |
| 7 | milk | active | 5 | 1 | 0.96 | (-0.606, 0.441, -1.266) |
| 8 | saffron | active | 10 | 0 | 0.99 | (-0.708, 0.318, -1.499) |
| 9 | Mugcup_high | deleted | 1 | 4 | 0.05 | (-1.145, 0.450, -1.426) |
| 10 | Sikhye_high | deleted | 1 | 4 | 0.05 | (-1.135, 0.460, -1.576) |
| 11 | choco_hazelnut_high | active | 4 | 4 | 0.51 | (-0.836, 0.348, -1.358) |
| 12 | Rabbit | deleted | 1 | 4 | 0.03 | (-2.375, 0.439, -1.228) |
| 13 | Mugcup_high | active | 3 | 0 | 0.94 | (-1.128, 0.452, -1.422) |
| 14 | Bear | tentative | 1 | 3 | 0.06 | (-2.824, 0.421, -1.121) |
| 15 | Sauce_high | active | 2 | 1 | 0.65 | (-2.392, 0.387, -1.008) |
| 16 | Bear | tentative | 1 | 0 | 0.55 | (-1.107, 0.444, -1.587) |

## Lifecycle Transitions

| object_id | from -> to | reason | stamp |
| --- | --- | --- | --- |
| 1 | tentative -> active | promoted | 1783994482.456 |
| 2 | tentative -> active | promoted | 1783994490.474 |
| 3 | tentative -> active | promoted | 1783994490.474 |
| 4 | tentative -> active | promoted | 1783994498.482 |
| 1 | active -> lost | missed_threshold | 1783994514.170 |
| 11 | tentative -> active | promoted | 1783994530.202 |
| 7 | tentative -> active | promoted | 1783994530.202 |
| 3 | active -> lost | missed_threshold | 1783994530.202 |
| 6 | tentative -> active | promoted | 1783994545.898 |
| 1 | lost -> active | redetected | 1783994545.898 |
| 5 | tentative -> deleted | existence_floor | 1783994545.898 |
| 1 | active -> lost | missed_threshold | 1783994553.903 |
| 3 | lost -> deleted | low_evidence | 1783994553.903 |
| 1 | lost -> active | redetected | 1783994561.920 |
| 9 | tentative -> deleted | existence_floor | 1783994569.925 |
| 10 | tentative -> deleted | existence_floor | 1783994569.925 |
| 8 | tentative -> active | promoted | 1783994577.614 |
| 13 | tentative -> active | promoted | 1783994585.616 |
| 1 | active -> lost | missed_threshold | 1783994601.653 |
| 12 | tentative -> deleted | existence_floor | 1783994609.328 |
| 1 | lost -> deleted | low_evidence | 1783994617.341 |
| 2 | active -> lost | missed_threshold | 1783994617.341 |
| 15 | tentative -> active | promoted | 1783994625.349 |
| 6 | active -> lost | missed_threshold | 1783994641.041 |

## Decision Summary

- new_tentative: 16
- short_term_match: 35

