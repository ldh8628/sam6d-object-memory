# Object Memory Run Report

- slam trajectory: `../input/260804_SAM_longcircle2/slam/CameraTrajectory.txt`
- pem dir: `../input/260804_SAM_longcircle2/pem`
- bag: `../input/260804_SAM_longcircle2/bag`

## Formula

`T_map_obj = T_map_cam (SLAM) * T_cam_obj (SAM-6D)`  [translations in metres]

## Run Summary

- SLAM poses loaded: 2119
- bag color timestamps: 2130
- frames with detections: 119
- frames fused (SLAM ok): 118
- frames skipped (no SLAM pose): 1
- total detections: 289

## Persistent Objects (object_memory owns object_id)

| object_id | object_name | status | obs | missed | confidence | T_map_obj xyz (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Mugcup_high | remembered | 44 | 9 | 0.03 | (-0.048, 0.048, 0.557) |
| 2 | saffron | deleted | 3 | 0 | 0.00 | (0.155, 0.002, -0.011) |
| 3 | Bear | remembered | 39 | 9 | 0.03 | (0.036, -0.049, 0.798) |
| 4 | saffron | deleted | 3 | 5 | 0.03 | (-0.412, -0.146, 0.623) |
| 5 | Mugcup_high | deleted | 1 | 0 | 0.01 | (0.892, -0.272, 0.896) |
| 6 | Sikhye_high | lost | 27 | 8 | 0.06 | (-0.153, -0.065, 0.878) |
| 7 | choco_hazelnut_high | deleted | 4 | 9 | 0.03 | (-0.322, -0.081, 0.934) |
| 8 | milk | lost | 25 | 7 | 0.14 | (-0.493, -0.066, 1.041) |
| 9 | Rabbit | lost | 25 | 7 | 0.14 | (-0.704, -0.126, 1.124) |
| 10 | Sauce_high | active | 29 | 3 | 0.86 | (-0.878, -0.166, 1.164) |
| 11 | Sikhye_high | deleted | 1 | 0 | 0.01 | (-0.447, -0.465, 1.551) |
| 12 | saffron | deleted | 1 | 0 | 0.01 | (-1.130, -0.423, 1.406) |
| 13 | choco_hazelnut_high | deleted | 1 | 3 | 0.03 | (-1.023, -0.110, 0.927) |
| 14 | saffron | active | 37 | 0 | 0.99 | (-0.563, -0.102, 0.695) |
| 15 | Febreze_high | deleted | 2 | 4 | 0.04 | (-0.955, -0.105, 0.845) |
| 16 | milk | deleted | 1 | 3 | 0.05 | (-1.073, -0.037, 0.833) |
| 17 | choco_hazelnut_high | lost | 13 | 1 | 0.22 | (-0.972, -0.042, 0.808) |
| 18 | Febreze_high | remembered | 10 | 2 | 0.03 | (-0.767, -0.043, 0.727) |
| 19 | Dinosaur | remembered | 10 | 8 | 0.03 | (-0.259, -0.000, 0.613) |
| 20 | saffron | deleted | 1 | 3 | 0.03 | (-0.354, -0.158, 0.616) |
| 21 | Sikhye_high | deleted | 1 | 0 | 0.01 | (0.925, -0.210, 0.677) |
| 22 | Febreze_high | deleted | 1 | 3 | 0.03 | (-0.362, -0.088, 1.022) |
| 23 | choco_hazelnut_high | deleted | 1 | 4 | 0.03 | (-0.333, -0.111, 0.929) |
| 24 | saffron | deleted | 2 | 4 | 0.03 | (-0.686, -0.251, 0.796) |
| 25 | saffron | deleted | 1 | 0 | 0.01 | (-1.157, -0.093, 0.309) |
| 26 | choco_hazelnut_high | deleted | 1 | 4 | 0.04 | (-0.321, -0.143, 0.920) |
| 27 | saffron | tentative | 2 | 2 | 0.13 | (-0.713, -0.215, 0.715) |
| 28 | milk | deleted | 1 | 3 | 0.03 | (-1.091, -0.008, 0.845) |

## Lifecycle Transitions

| object_id | from -> to | reason | stamp |
| --- | --- | --- | --- |
| 1 | tentative -> active | promoted | 1785831506.852 |
| 2 | tentative -> deleted | existence_floor | 1785831507.534 |
| 3 | tentative -> active | promoted | 1785831508.884 |
| 4 | tentative -> active | promoted | 1785831509.567 |
| 4 | active -> lost | missed_threshold | 1785831511.265 |
| 5 | tentative -> deleted | existence_floor | 1785831511.946 |
| 6 | tentative -> active | promoted | 1785831513.297 |
| 4 | lost -> deleted | low_evidence | 1785831513.297 |
| 1 | active -> lost | missed_threshold | 1785831513.630 |
| 7 | tentative -> active | promoted | 1785831514.312 |
| 8 | tentative -> active | promoted | 1785831514.312 |
| 9 | tentative -> active | promoted | 1785831514.649 |
| 10 | tentative -> active | promoted | 1785831514.995 |
| 11 | tentative -> deleted | existence_floor | 1785831514.995 |
| 1 | lost -> remembered | long_term_memory | 1785831515.328 |
| 7 | active -> lost | missed_threshold | 1785831517.026 |
| 12 | tentative -> deleted | existence_floor | 1785831517.026 |
| 14 | tentative -> active | promoted | 1785831517.709 |
| 3 | active -> lost | missed_threshold | 1785831517.709 |
| 15 | tentative -> active | promoted | 1785831518.042 |
| 6 | active -> lost | missed_threshold | 1785831518.042 |
| 7 | lost -> deleted | low_evidence | 1785831518.042 |
| 13 | tentative -> deleted | existence_floor | 1785831518.042 |
| 8 | active -> lost | missed_threshold | 1785831518.377 |
| 3 | lost -> remembered | long_term_memory | 1785831518.724 |
| 15 | active -> lost | missed_threshold | 1785831518.724 |
| 6 | lost -> remembered | long_term_memory | 1785831519.058 |
| 17 | tentative -> active | promoted | 1785831519.400 |
| 8 | lost -> remembered | long_term_memory | 1785831519.400 |
| 15 | lost -> deleted | low_evidence | 1785831519.400 |
| 16 | tentative -> deleted | existence_floor | 1785831519.740 |
| 9 | active -> lost | missed_threshold | 1785831519.740 |
| 18 | tentative -> active | promoted | 1785831523.804 |
| 9 | lost -> remembered | long_term_memory | 1785831524.153 |
| 10 | active -> lost | missed_threshold | 1785831524.153 |
| 19 | tentative -> active | promoted | 1785831524.820 |
| 1 | remembered -> active | redetected | 1785831524.820 |
| 10 | lost -> remembered | long_term_memory | 1785831525.835 |
| 3 | remembered -> active | redetected | 1785831528.216 |
| 3 | active -> lost | missed_threshold | 1785831528.550 |
| 19 | active -> lost | missed_threshold | 1785831528.550 |
| 3 | lost -> active | redetected | 1785831528.899 |
| 17 | active -> lost | missed_threshold | 1785831528.899 |
| 18 | active -> lost | missed_threshold | 1785831528.899 |
| 19 | lost -> remembered | long_term_memory | 1785831529.566 |
| 17 | lost -> remembered | long_term_memory | 1785831529.914 |
| 18 | lost -> remembered | long_term_memory | 1785831529.914 |
| 21 | tentative -> deleted | existence_floor | 1785831530.248 |
| 20 | tentative -> deleted | existence_floor | 1785831530.248 |
| 6 | remembered -> active | redetected | 1785831531.264 |
| 9 | remembered -> active | redetected | 1785831532.280 |
| 8 | remembered -> active | redetected | 1785831532.280 |
| 14 | active -> lost | missed_threshold | 1785831532.280 |
| 10 | remembered -> active | redetected | 1785831532.963 |
| 1 | active -> lost | missed_threshold | 1785831532.963 |
| 14 | lost -> remembered | long_term_memory | 1785831532.963 |
| 22 | tentative -> deleted | existence_floor | 1785831532.963 |
| 1 | lost -> remembered | long_term_memory | 1785831533.978 |
| 23 | tentative -> deleted | existence_floor | 1785831533.978 |
| 18 | remembered -> active | redetected | 1785831535.676 |
| 14 | remembered -> active | redetected | 1785831535.676 |
| 3 | active -> lost | missed_threshold | 1785831535.676 |
| 6 | active -> lost | missed_threshold | 1785831536.010 |
| 18 | active -> lost | missed_threshold | 1785831536.010 |
| 17 | remembered -> active | redetected | 1785831536.359 |
| 24 | tentative -> deleted | existence_floor | 1785831536.359 |
| 18 | lost -> remembered | long_term_memory | 1785831536.359 |
| 3 | lost -> remembered | long_term_memory | 1785831536.692 |
| 9 | active -> lost | missed_threshold | 1785831536.692 |
| 6 | lost -> remembered | long_term_memory | 1785831537.025 |
| 8 | active -> lost | missed_threshold | 1785831537.375 |
| 17 | active -> lost | missed_threshold | 1785831537.375 |
| 17 | lost -> active | redetected | 1785831537.708 |
| 9 | lost -> remembered | long_term_memory | 1785831537.708 |
| 18 | remembered -> active | redetected | 1785831561.477 |
| 8 | lost -> remembered | long_term_memory | 1785831561.477 |
| 10 | active -> lost | missed_threshold | 1785831561.477 |
| 25 | tentative -> deleted | existence_floor | 1785831561.818 |
| 19 | remembered -> active | redetected | 1785831562.151 |
| 1 | remembered -> active | redetected | 1785831562.500 |
| 10 | lost -> remembered | long_term_memory | 1785831564.183 |
| 3 | remembered -> active | redetected | 1785831564.532 |
| 3 | active -> lost | missed_threshold | 1785831564.865 |
| 18 | active -> lost | missed_threshold | 1785831564.865 |
| 3 | lost -> active | redetected | 1785831565.199 |
| 17 | active -> lost | missed_threshold | 1785831565.199 |
| 18 | lost -> remembered | long_term_memory | 1785831565.881 |
| 17 | lost -> remembered | long_term_memory | 1785831566.214 |
| 19 | active -> lost | missed_threshold | 1785831566.214 |
| 6 | remembered -> active | redetected | 1785831566.563 |
| 19 | lost -> remembered | long_term_memory | 1785831566.897 |
| 6 | active -> lost | missed_threshold | 1785831567.246 |
| 6 | lost -> active | redetected | 1785831568.262 |
| 8 | remembered -> active | redetected | 1785831568.928 |
| 14 | active -> lost | missed_threshold | 1785831568.928 |
| 9 | remembered -> active | redetected | 1785831569.277 |
| 1 | active -> lost | missed_threshold | 1785831569.277 |
| 10 | remembered -> active | redetected | 1785831569.611 |
| 14 | lost -> remembered | long_term_memory | 1785831569.957 |
| 26 | tentative -> deleted | existence_floor | 1785831569.957 |
| 14 | remembered -> active | redetected | 1785831571.309 |
| 1 | lost -> remembered | long_term_memory | 1785831571.309 |
| 17 | remembered -> active | redetected | 1785831571.643 |
| 18 | remembered -> active | redetected | 1785831572.326 |
| 3 | active -> lost | missed_threshold | 1785831572.659 |
| 17 | active -> lost | missed_threshold | 1785831572.659 |
| 18 | active -> lost | missed_threshold | 1785831572.659 |
| 6 | active -> lost | missed_threshold | 1785831573.007 |
| 18 | lost -> remembered | long_term_memory | 1785831573.007 |
| 17 | lost -> active | redetected | 1785831573.341 |
| 8 | active -> lost | missed_threshold | 1785831573.341 |
| 9 | active -> lost | missed_threshold | 1785831573.341 |
| 3 | lost -> remembered | long_term_memory | 1785831573.675 |
| 28 | tentative -> deleted | existence_floor | 1785831573.675 |
| 17 | active -> lost | missed_threshold | 1785831573.675 |

## Decision Summary

- new_tentative: 28
- rejected: 2
- short_term_match: 259

