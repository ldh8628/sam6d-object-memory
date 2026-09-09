# 03. 데이터셋 — 레포에도 아카이브에도 없다 (직접 복사)

원시 ROS2 bag은 **합계 약 380 GB**다. 이미 압축된 이미지 스트림이라 tar/zstd로 줄지 않아
아카이브에 넣지 않았다. **외장하드 또는 `rsync` 로 직접 복사**해야 한다.

> 단, `data_slam/` 안의 **변환·분석 스크립트와 bag metadata.yaml 은 이 레포에 포함**돼 있다.
> (`data_slam/0724_chungbuk/*.py`, `data_slam/260714_frame_data/*.py`, `converted/*/metadata.yaml`)
> 데이터만 채워 넣으면 스크립트가 그대로 돈다.

## 0. 데이터 없이 지금 당장 할 수 있는 것

| 가능 | 내용 |
|---|---|
| ✅ | ObjectMemory **pytest 79개 전부** (합성 데이터로 동작, bag 불필요) |
| ✅ | 5개 워크스페이스 **colcon build** 전부 |
| ✅ | SAM-6D 템플릿 재렌더(CAD만 있으면) |
| ❌ | SLAM 궤적 재현, SAM-6D 추론, E2E 파이프라인 |

## 1. 복사 대상

### (a) `data_slam/` — 319 GB, SLAM 메인 데이터

| 하위 | 크기 | 내용 |
|---|---:|---|
| `0724_chungbuk/SLAM_RGBD_IMU_LIDAR/` | 82 GB | 9세션, RGB-D + IMU + **VLP-16 LiDAR**. LiDAR 준-GT 산출에 씀 |
| `0724_chungbuk/SAM_RGBD/` | 55 GB | 같은 장소 SAM용 RGB-D |
| `0724_chungbuk/converted/` | 62 GB | 위를 `convert_recording.py` 로 변환한 **표준 ROS2 bag** ← SLAM이 실제로 읽는 것 |
| `260714_frame_data/SLAM_data/` | 37 GB | D455 2카메라 세션(SLAM 쪽 8세션) |
| `260714_frame_data/SAM_data/` | 33 GB | 〃 (SAM 쪽 8세션) |
| `260714_frame_data/converted/` | 37 GB | 변환된 표준 bag |
| `rgbd_bag/` | 15 GB | `SLAM_one_lap`, `SLAM_three_laps`, `SLAM_one_lap_back_and_forth`, `SLAM_forward_backward_repeat` |

### (b) `sam6d_ws/data/ros2_bag/` — 44 GB, SAM-6D 입력

`SAM_circle`, `SAM_loop1`, `SAM_loop2`, `SAM_occlusion` (rgbd_imu_sdk_bag 4종 — ObjectMemory 결과의 근거),
`sam_105018` … `sam_110633`(260714 세션), `milk_*`, `high/low_texture_*` 등.

필수 토픽:
```
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

### (c) `rtabmap_ws/data/` — 15 GB

`SLAM_1바퀴`, `SLAM_3바퀴`, `SLAM_1바퀴 후 와리가리`, `SLAM_직전 반복주행`
(`data_slam/rgbd_bag` 과 같은 녹화의 RTAB-Map 실험용 사본)

### (d) `orbslam_ws/data/rgbd_bag/` — 157 MB

`no_cli_rgbd.yaml` 이 기본으로 읽는 **스모크 테스트용 짧은 bag**. 이것만 있어도
ORB-SLAM3 기동 확인은 가능하다. 용량이 작으니 **가장 먼저 복사할 것.**

## 2. 복사 방법

```bash
# 원본 PC에서 (예: 새 PC 이름 newpc, 대상 루트 ~/slam-perception-stack)
rsync -aP --info=progress2 \
  /home/ldh9501/temp_ws/CLI_environment/orbslam_ws/data/rgbd_bag \
  newpc:~/slam-perception-stack/orbslam_ws/data/

rsync -aP --info=progress2 \
  /home/ldh9501/temp_ws/CLI_environment/sam6d_ws/data/ros2_bag \
  newpc:~/slam-perception-stack/sam6d_ws/data/

rsync -aP --info=progress2 \
  /home/ldh9501/temp_ws/CLI_environment/data_slam/ \
  newpc:~/slam-perception-stack/data_slam/
```

외장하드를 쓸 경우 `rsync -aP <원본> /media/<usb>/` 후 새 PC에서 반대로.
**`rsync -aP` 는 중단 후 재개가 되므로 대용량엔 `cp` 보다 이쪽을 쓸 것.**

디스크가 부족하면 다른 위치에 두고 심링크로 연결한다:

```bash
ln -sfn /mnt/bigdisk/data_slam  ~/slam-perception-stack/data_slam
ln -sfn /mnt/bigdisk/SAM_circle ~/slam-perception-stack/sam6d_ws/data/ros2_bag/SAM_circle
```

## 3. bag 포맷 함정 (반드시 읽어라)

1. **SDK 포맷 ≠ ROS2 bag.**
   `0724_chungbuk/SLAM_RGBD_IMU_LIDAR/` 와 `260714_frame_data/SLAM_data/` 는 RealSense **SDK 녹화 포맷**이고
   **depth가 color에 정렬돼 있지 않다.** SLAM에 바로 넣으면 안 된다.
   → `data_slam/0724_chungbuk/convert_all.py` / `260714_frame_data/convert_recording.py` 로
   `converted/` 를 만든 뒤 그것을 쓴다.

2. **rosbag2 버전 불일치.**
   `0724_chungbuk` 의 **LiDAR bag(`lidar_bags_v5`)만 rosbag2 v9(Jazzy)** 로 기록돼 Humble에서 열리지 않는다.
   → `data_slam/0724_chungbuk/downgrade_bag_metadata.py` 로 metadata 버전을 낮춘다.

3. **Velodyne 스윕이 반 바퀴씩 들어온다.**
   한 메시지가 360°가 아니라 절반이라 **2개를 병합**해야 정상 스캔이 된다
   (`merge_velodyne_sweeps.py`). 안 하면 겹침이 3~5%로 떨어져 정합이 깨진다.

4. **카메라 intrinsic이 세션마다 다르다.**
   0724 데이터의 카메라는 **D455F**(일부 문서의 "d435i"는 오기), 세션별로 `fx` 가 다르다.
   → `make_orb_settings.py` / `make_orb_settings_sdk.py` 가 세션별 settings yaml을 만든다.

5. **260714 데이터에는 IMU가 없다.** `pair0`, `pair5` 는 삭제됨. 2카메라(SAM/SLAM) 간
   **extrinsic이 미보정**이라 객체 맵이 흩어진다(알려진 미해결 이슈).

## 4. 각 데이터셋의 알려진 결과 (재현 기준값)

| 데이터 | 결과 |
|---|---|
| 0724_chungbuk 9세션 | ORB-SLAM3 8/9 성공, drift 7/8 ≤ 2.6%. **183942 세션만 실패**(사람이 흰 파티션을 들고 지나가 4.35 m 환영이동 + 끝↔시작 loop 미검출) |
| 0724 LiDAR 준-GT | hdl_graph_slam 9/9 PASS, 오차 0.012~0.238 m. 다만 분해능이 ~0.25 m 라 그 이하 비교는 무의미 |
| ORB vs RTAB | 중앙값 ORB 0.019 / RTAB 0.043, **최악값 ORB 2.395 / RTAB 0.257**. 밀도는 ORB 30 Hz vs RTAB 10.5 Hz → **ORB의 이점은 밀도지 정확도가 아니다** |
| 260714 pair4 | ORB3 2730 pose·loop 닫힘, SAM-6D PEM 51/51, ObjectMemory 21/21 융합 성공 |
| SAM-6D ISM (사람 GT 959건) | 실파이프라인 F1 0.7408 (Phase3 appe_v2+crossNMS 적용 시) |

상세 근거는 `_bmad_output_for_slam/planning-artifacts/research/` 및
`data_slam/0724_chungbuk/RESULTS_0724_chungbuk.md` 참고.
