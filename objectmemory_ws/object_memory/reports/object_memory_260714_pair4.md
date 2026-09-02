# Object Memory Run Report

- slam trajectory: `/home/ldh9501/temp_ws/CLI_environment/orbslam_ws/output/260714/slam_110109/CameraTrajectory.txt`
- pem dir: `/home/ldh9501/temp_ws/CLI_environment/sam6d_ws/outputs/pem_inputs/sam_110104`
- bag: `/home/ldh9501/temp_ws/CLI_environment/data_slam/260714_frame_data/converted/sam_110104`

## Formula

`T_map_obj = T_map_cam (SLAM) * T_cam_obj (SAM-6D)`  [translations in metres]

## Run Summary

- SLAM poses loaded: 2730
- bag color timestamps: 547
- frames with detections: 21
- frames fused (SLAM ok): 21
- frames skipped (no SLAM pose): 0
- total detections: 51

## Persistent Objects (object_memory owns object_id)

| object_id | object_name | status | obs | missed | confidence | T_map_obj xyz (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Dinosaur | deleted | 1 | 3 | 0.02 | (-0.441, 0.168, 1.641) |
| 2 | Sauce_high | active | 2 | 2 | 0.73 | (-0.356, 0.141, 1.480) |
| 3 | choco_hazelnut_high | active | 2 | 1 | 0.77 | (-0.107, 0.125, 1.862) |
| 4 | Dinosaur | active | 2 | 2 | 0.38 | (-2.204, 0.113, 2.976) |
| 5 | Febreze_high | active | 2 | 2 | 0.44 | (-1.983, 0.089, 2.988) |
| 6 | Rabbit | tentative | 1 | 3 | 0.06 | (0.312, 0.140, 2.518) |
| 7 | Sauce_high | tentative | 1 | 3 | 0.09 | (0.532, 0.101, 2.504) |
| 8 | choco_hazelnut_high | tentative | 1 | 3 | 0.08 | (0.077, 0.197, 2.477) |
| 9 | Bear | deleted | 1 | 3 | 0.03 | (-2.368, -0.035, 3.309) |
| 10 | Febreze_high | active | 2 | 2 | 0.45 | (-2.244, 0.120, 1.660) |
| 11 | Sauce_high | deleted | 1 | 4 | 0.04 | (-2.145, 0.106, 1.984) |
| 12 | milk | tentative | 2 | 1 | 0.23 | (-0.155, 0.210, 0.071) |
| 13 | saffron | tentative | 2 | 1 | 0.34 | (0.049, 0.091, 0.136) |
| 14 | Bear | tentative | 1 | 1 | 0.31 | (-2.732, 0.120, 0.891) |
| 15 | Mugcup_high | active | 2 | 0 | 0.84 | (-2.339, 0.168, 1.031) |
| 16 | Sikhye_high | tentative | 1 | 1 | 0.43 | (-2.378, 0.152, 0.888) |
| 17 | choco_hazelnut_high | tentative | 1 | 1 | 0.27 | (-2.602, 0.035, 1.032) |
| 18 | saffron | deleted | 1 | 0 | 0.01 | (-2.220, 0.013, 0.466) |
| 19 | choco_hazelnut_high | tentative | 1 | 2 | 0.15 | (0.223, 0.094, 0.792) |
| 20 | milk | tentative | 1 | 1 | 0.35 | (0.227, 0.228, 0.922) |
| 21 | saffron | tentative | 1 | 1 | 0.42 | (0.421, 0.076, 0.986) |
| 22 | Bear | tentative | 1 | 3 | 0.05 | (-1.630, -0.056, 4.995) |
| 23 | Dinosaur | tentative | 1 | 3 | 0.08 | (-1.577, 0.124, 3.212) |
| 24 | Febreze_high | tentative | 1 | 2 | 0.19 | (-1.358, 0.077, 3.247) |
| 25 | Rabbit | tentative | 1 | 2 | 0.14 | (-0.363, 0.148, 1.466) |
| 26 | Bear | tentative | 1 | 1 | 0.30 | (-0.098, 0.120, 1.743) |
| 27 | milk | tentative | 1 | 2 | 0.17 | (0.339, 0.186, 1.818) |
| 28 | saffron | tentative | 1 | 2 | 0.17 | (0.414, 0.021, 1.637) |
| 29 | Mugcup_high | tentative | 1 | 2 | 0.21 | (-1.451, 0.117, -0.827) |
| 30 | choco_hazelnut_high | tentative | 1 | 2 | 0.16 | (-1.253, 0.078, -0.529) |
| 31 | saffron | tentative | 1 | 0 | 0.62 | (-1.357, 0.030, -0.298) |
| 32 | Bear | tentative | 1 | 0 | 0.48 | (-1.296, 0.131, 1.695) |
| 33 | Mugcup_high | tentative | 1 | 0 | 0.61 | (-1.084, 0.138, 1.605) |
| 34 | saffron | tentative | 1 | 1 | 0.14 | (-1.129, -0.088, 2.040) |
| 35 | milk | tentative | 1 | 1 | 0.36 | (-0.839, 0.184, -0.354) |
| 36 | saffron | active | 2 | 0 | 0.89 | (-0.605, 0.037, -0.243) |
| 37 | Bear | tentative | 1 | 2 | 0.13 | (-2.404, 0.129, 1.848) |
| 38 | Sauce_high | active | 2 | 1 | 0.82 | (-0.354, 0.125, 3.183) |
| 39 | Febreze_high | tentative | 1 | 1 | 0.36 | (-1.807, 0.112, 1.218) |
| 40 | Bear | tentative | 1 | 0 | 0.55 | (-1.997, 0.137, 1.213) |
| 41 | saffron | tentative | 1 | 0 | 0.58 | (-2.131, -0.021, 1.698) |

## Lifecycle Transitions

| object_id | from -> to | reason | stamp |
| --- | --- | --- | --- |
| 18 | tentative -> deleted | existence_floor | 1783994530.202 |
| 2 | tentative -> active | promoted | 1783994553.903 |
| 4 | tentative -> active | promoted | 1783994561.920 |
| 5 | tentative -> active | promoted | 1783994561.920 |
| 3 | tentative -> active | promoted | 1783994569.925 |
| 1 | tentative -> deleted | existence_floor | 1783994585.616 |
| 10 | tentative -> active | promoted | 1783994601.653 |
| 9 | tentative -> deleted | existence_floor | 1783994601.653 |
| 38 | tentative -> active | promoted | 1783994625.349 |
| 15 | tentative -> active | promoted | 1783994641.041 |
| 11 | tentative -> deleted | existence_floor | 1783994641.041 |
| 36 | tentative -> active | promoted | 1783994649.060 |

## Decision Summary

- new_tentative: 41
- short_term_match: 10

