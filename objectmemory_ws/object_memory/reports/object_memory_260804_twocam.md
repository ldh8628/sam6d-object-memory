# Object Memory Run Report

- slam trajectory: `/home/ldh9501/temp_ws/CLI_environment/output_slam/260804_office/orb3_slam/longcircle2/CameraTrajectory.txt`
- pem dir: `/home/ldh9501/temp_ws/CLI_environment/sam6d_ws/outputs/pem_inputs/260804_SAM_longcircle2`
- bag: `/home/ldh9501/temp_ws/CLI_environment/data_slam/260804_office/260804_SAM_converted/longcircle2/ros2_standard`

## Formula

`T_map_obj = T_map_cam (SLAM) * T_cam_obj (SAM-6D)`  [translations in metres]

## Run Summary

- SLAM poses loaded: 2166
- bag color timestamps: 2130
- frames with detections: 119
- frames fused (SLAM ok): 119
- frames skipped (no SLAM pose): 0
- total detections: 289

## Persistent Objects (object_memory owns object_id)

| object_id | object_name | status | obs | missed | confidence | T_map_obj xyz (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Dinosaur | deleted | 1 | 5 | 0.03 | (-0.858, 0.213, -0.556) |
| 2 | Mugcup_high | remembered | 45 | 11 | 0.05 | (-0.817, 0.269, -0.339) |
| 3 | saffron | deleted | 3 | 0 | 0.00 | (-0.295, 0.062, -0.090) |
| 4 | Bear | lost | 39 | 9 | 0.16 | (-1.097, 0.257, -0.289) |
| 5 | saffron | deleted | 3 | 6 | 0.03 | (-0.918, 0.082, -0.708) |
| 6 | Mugcup_high | deleted | 1 | 0 | 0.01 | (-1.316, 0.132, 0.563) |
| 7 | Sikhye_high | lost | 27 | 8 | 0.28 | (-1.160, 0.254, -0.458) |
| 8 | choco_hazelnut_high | deleted | 4 | 11 | 0.05 | (-1.197, 0.250, -0.629) |
| 9 | milk | active | 24 | 7 | 0.44 | (-1.272, 0.286, -0.780) |
| 10 | Rabbit | active | 25 | 7 | 0.44 | (-1.352, 0.229, -0.995) |
| 11 | Sauce_high | active | 29 | 3 | 0.93 | (-1.387, 0.187, -1.165) |
| 12 | Sikhye_high | deleted | 1 | 0 | 0.01 | (-1.880, 0.060, -0.781) |
| 13 | saffron | deleted | 1 | 0 | 0.01 | (-1.687, -0.019, -1.458) |
| 14 | choco_hazelnut_high | deleted | 1 | 3 | 0.04 | (-1.143, 0.156, -1.298) |
| 15 | saffron | deleted | 2 | 6 | 0.04 | (-1.087, 0.025, -0.900) |
| 16 | Febreze_high | deleted | 2 | 5 | 0.04 | (-1.064, 0.135, -1.211) |
| 17 | saffron | active | 28 | 0 | 0.99 | (-0.914, 0.138, -0.782) |
| 18 | milk | deleted | 1 | 4 | 0.04 | (-1.042, 0.199, -1.326) |
| 19 | choco_hazelnut_high | active | 13 | 1 | 0.40 | (-1.031, 0.196, -1.217) |
| 20 | Febreze_high | remembered | 10 | 2 | 0.05 | (-0.923, 0.187, -0.969) |
| 21 | Dinosaur | remembered | 10 | 10 | 0.05 | (-0.848, 0.221, -0.515) |
| 22 | Sikhye_high | deleted | 1 | 0 | 0.01 | (-1.195, 0.126, 0.558) |
| 23 | Febreze_high | deleted | 1 | 4 | 0.03 | (-1.348, 0.250, -0.876) |
| 24 | choco_hazelnut_high | deleted | 1 | 5 | 0.03 | (-1.215, 0.205, -0.722) |
| 25 | saffron | lost | 12 | 2 | 0.08 | (-1.066, 0.085, -0.902) |
| 26 | milk | deleted | 1 | 5 | 0.03 | (-1.247, 0.080, -0.829) |
| 27 | saffron | deleted | 1 | 0 | 0.01 | (-0.540, -0.018, -1.413) |
| 28 | choco_hazelnut_high | deleted | 1 | 5 | 0.04 | (-1.183, 0.183, -0.551) |
| 29 | milk | tentative | 1 | 3 | 0.05 | (-1.064, 0.219, -1.325) |

## Lifecycle Transitions

| object_id | from -> to | reason | stamp |
| --- | --- | --- | --- |
| 2 | tentative -> active | promoted | 1785831513.120 |
| 3 | tentative -> deleted | existence_floor | 1785831514.136 |
| 1 | tentative -> deleted | existence_floor | 1785831514.469 |
| 4 | tentative -> active | promoted | 1785831515.485 |
| 5 | tentative -> active | promoted | 1785831516.850 |
| 5 | active -> lost | missed_threshold | 1785831517.866 |
| 6 | tentative -> deleted | existence_floor | 1785831518.547 |
| 7 | tentative -> active | promoted | 1785831519.898 |
| 5 | lost -> deleted | low_evidence | 1785831520.231 |
| 8 | tentative -> active | promoted | 1785831520.914 |
| 9 | tentative -> active | promoted | 1785831520.914 |
| 2 | active -> lost | missed_threshold | 1785831520.914 |
| 10 | tentative -> active | promoted | 1785831521.251 |
| 11 | tentative -> active | promoted | 1785831521.596 |
| 12 | tentative -> deleted | existence_floor | 1785831521.596 |
| 2 | lost -> remembered | long_term_memory | 1785831522.274 |
| 13 | tentative -> deleted | existence_floor | 1785831523.628 |
| 15 | tentative -> active | promoted | 1785831524.310 |
| 8 | active -> lost | missed_threshold | 1785831524.310 |
| 4 | active -> lost | missed_threshold | 1785831524.643 |
| 14 | tentative -> deleted | existence_floor | 1785831524.643 |
| 17 | tentative -> active | promoted | 1785831524.978 |
| 7 | active -> lost | missed_threshold | 1785831525.326 |
| 8 | lost -> deleted | low_evidence | 1785831525.326 |
| 15 | active -> lost | missed_threshold | 1785831525.326 |
| 4 | lost -> remembered | long_term_memory | 1785831525.659 |
| 9 | active -> lost | missed_threshold | 1785831525.659 |
| 19 | tentative -> active | promoted | 1785831526.002 |
| 7 | lost -> remembered | long_term_memory | 1785831526.342 |
| 15 | lost -> deleted | low_evidence | 1785831526.342 |
| 16 | tentative -> deleted | existence_floor | 1785831526.342 |
| 9 | lost -> remembered | long_term_memory | 1785831530.072 |
| 18 | tentative -> deleted | existence_floor | 1785831530.072 |
| 20 | tentative -> active | promoted | 1785831530.405 |
| 10 | active -> lost | missed_threshold | 1785831530.405 |
| 21 | tentative -> active | promoted | 1785831531.421 |
| 2 | remembered -> active | redetected | 1785831531.421 |
| 10 | lost -> remembered | long_term_memory | 1785831531.421 |
| 11 | active -> lost | missed_threshold | 1785831532.104 |
| 11 | lost -> remembered | long_term_memory | 1785831533.119 |
| 4 | remembered -> active | redetected | 1785831534.818 |
| 4 | active -> lost | missed_threshold | 1785831535.151 |
| 4 | lost -> active | redetected | 1785831535.500 |
| 19 | active -> lost | missed_threshold | 1785831535.833 |
| 21 | active -> lost | missed_threshold | 1785831535.833 |
| 20 | active -> lost | missed_threshold | 1785831536.167 |
| 22 | tentative -> deleted | existence_floor | 1785831536.849 |
| 21 | lost -> remembered | long_term_memory | 1785831536.849 |
| 7 | remembered -> active | redetected | 1785831537.865 |
| 20 | lost -> remembered | long_term_memory | 1785831538.209 |
| 10 | remembered -> active | redetected | 1785831538.882 |
| 9 | remembered -> active | redetected | 1785831538.882 |
| 19 | lost -> remembered | long_term_memory | 1785831538.882 |
| 17 | active -> lost | missed_threshold | 1785831539.230 |
| 11 | remembered -> active | redetected | 1785831539.564 |
| 23 | tentative -> deleted | existence_floor | 1785831539.897 |
| 2 | active -> lost | missed_threshold | 1785831540.246 |
| 17 | lost -> remembered | long_term_memory | 1785831540.246 |
| 24 | tentative -> deleted | existence_floor | 1785831540.914 |
| 2 | lost -> remembered | long_term_memory | 1785831541.262 |
| 20 | remembered -> active | redetected | 1785831542.278 |
| 17 | remembered -> active | redetected | 1785831542.611 |
| 20 | active -> lost | missed_threshold | 1785831542.611 |
| 19 | remembered -> active | redetected | 1785831542.960 |
| 25 | tentative -> active | promoted | 1785831542.960 |
| 4 | active -> lost | missed_threshold | 1785831542.960 |
| 17 | active -> lost | missed_threshold | 1785831542.960 |
| 7 | active -> lost | missed_threshold | 1785831543.294 |
| 20 | lost -> remembered | long_term_memory | 1785831543.294 |
| 26 | tentative -> deleted | existence_floor | 1785831543.627 |
| 17 | lost -> remembered | long_term_memory | 1785831543.627 |
| 9 | active -> lost | missed_threshold | 1785831543.976 |
| 10 | active -> lost | missed_threshold | 1785831543.976 |
| 17 | remembered -> active | redetected | 1785831561.642 |
| 4 | lost -> remembered | long_term_memory | 1785831561.642 |
| 7 | lost -> remembered | long_term_memory | 1785831561.642 |
| 17 | active -> lost | missed_threshold | 1785831567.736 |
| 20 | remembered -> active | redetected | 1785831568.079 |
| 10 | lost -> remembered | long_term_memory | 1785831568.079 |
| 11 | active -> lost | missed_threshold | 1785831568.079 |
| 17 | lost -> active | redetected | 1785831568.419 |
| 9 | lost -> remembered | long_term_memory | 1785831568.419 |
| 27 | tentative -> deleted | existence_floor | 1785831568.419 |
| 21 | remembered -> active | redetected | 1785831568.753 |
| 2 | remembered -> active | redetected | 1785831569.101 |
| 4 | remembered -> active | redetected | 1785831571.133 |
| 11 | lost -> remembered | long_term_memory | 1785831571.133 |
| 4 | active -> lost | missed_threshold | 1785831571.467 |
| 4 | lost -> active | redetected | 1785831571.800 |
| 20 | active -> lost | missed_threshold | 1785831571.800 |
| 19 | active -> lost | missed_threshold | 1785831572.149 |
| 20 | lost -> remembered | long_term_memory | 1785831572.816 |
| 7 | remembered -> active | redetected | 1785831573.165 |
| 19 | lost -> remembered | long_term_memory | 1785831573.165 |
| 21 | active -> lost | missed_threshold | 1785831573.165 |
| 7 | active -> lost | missed_threshold | 1785831573.847 |
| 17 | active -> lost | missed_threshold | 1785831573.847 |
| 21 | lost -> remembered | long_term_memory | 1785831574.181 |
| 7 | lost -> active | redetected | 1785831574.863 |
| 9 | remembered -> active | redetected | 1785831575.530 |
| 10 | remembered -> active | redetected | 1785831575.879 |
| 17 | lost -> remembered | long_term_memory | 1785831575.879 |
| 11 | remembered -> active | redetected | 1785831576.212 |
| 25 | active -> lost | missed_threshold | 1785831576.558 |
| 2 | active -> lost | missed_threshold | 1785831577.228 |
| 25 | lost -> remembered | long_term_memory | 1785831577.567 |
| 28 | tentative -> deleted | existence_floor | 1785831577.567 |
| 17 | remembered -> active | redetected | 1785831577.911 |
| 19 | remembered -> active | redetected | 1785831578.244 |
| 25 | remembered -> active | redetected | 1785831578.244 |
| 2 | lost -> remembered | long_term_memory | 1785831578.244 |
| 17 | active -> lost | missed_threshold | 1785831578.244 |
| 17 | lost -> active | redetected | 1785831578.593 |
| 25 | active -> lost | missed_threshold | 1785831578.593 |
| 20 | remembered -> active | redetected | 1785831578.927 |
| 25 | lost -> active | redetected | 1785831578.927 |
| 17 | active -> lost | missed_threshold | 1785831578.927 |
| 17 | lost -> active | redetected | 1785831579.260 |
| 25 | active -> lost | missed_threshold | 1785831579.260 |
| 19 | active -> lost | missed_threshold | 1785831579.260 |
| 20 | active -> lost | missed_threshold | 1785831579.260 |
| 25 | lost -> active | redetected | 1785831579.609 |
| 20 | lost -> remembered | long_term_memory | 1785831579.609 |
| 19 | lost -> active | redetected | 1785831579.943 |
| 4 | active -> lost | missed_threshold | 1785831579.943 |
| 25 | active -> lost | missed_threshold | 1785831579.943 |
| 7 | active -> lost | missed_threshold | 1785831580.276 |

## Decision Summary

- new_tentative: 29
- short_term_match: 260

