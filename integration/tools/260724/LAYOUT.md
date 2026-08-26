# 0724 충북 데이터셋 — 폴더 구조와 세션 이름

2026-08-04 재구성. 경로만 보고 **어느 기계가 무엇을 수집했는지** 알 수 있도록 바꿨다.

```
data_slam/0724_chungbuk/
├── husky_a200_0881/            Husky A200 탑재 · Linux · 호스트 cpr-a200-0881
│   ├── raw/<세션>/                 수집 원본
│   │     <세션>.db3                  RealSense D455F #419222301870 (SDK 포맷, depth 미정렬)
│   │     cpr-a200-0881_velodyne_*/   Velodyne VLP-16
│   │     xsens/                      Xsens MTi 200Hz
│   │     rgbd_timestamp_associations.json   color↔depth 페어링 + host epoch ns
│   ├── rgbd_standard/<세션>/       표준 ROS2 RGB-D bag (aligned depth, epoch 스탬프)
│   └── lidar_bags_v5/              Velodyne metadata만 v9→v5 재작성, db3는 symlink
└── laptop_win_qn90665/         휴대 노트북 · Windows · 호스트 BOOK-QN90665CQN
    ├── raw/<세션>/                 RealSense D455F #408222300890 (RGB-D 단독)
    └── rgbd_standard/<세션>/       표준 ROS2 RGB-D bag
```

두 카메라는 **서로 다른 물리 장비**이며 같은 세션을 동시에 녹화했다. 시각은 동일 epoch
축이라 `rgbd_standard` 끼리 바로 정합된다.

## 세션 이름

이름 = `<주행형태><바퀴수><순번>_<HHMMSS>`. 형태와 바퀴 수는 LiDAR 기준궤적에서 실측했다
(단일 루프는 중심각 적산, 8자는 총 요각/720°). 앞부분이 같은 camA/camB가 한 쌍이다.

| 세션 | 형태 | 바퀴 | 경로 | 루프 크기 | camA (husky) | camB (laptop) |
|---|---|---|---|---|---|---|
| eight1   | 직선 복도 + 사각 2엽 8자 | 1 | 27.8m | 2.6×4.6m | `eight1_173704` | `eight1_173704` |
| eight3a  | 8자 | 3 | 55.4m | 4.2×5.6m | `eight3a_180633` | `eight3a_180632` |
| circle3a | 단일 루프 | 3 | 26.4m | 3.6×2.7m | `circle3a_181235` | `circle3a_181234` |
| circle3b | 단일 루프 | 3 | 27.3m | 3.3×2.9m | `circle3b_181520` | `circle3b_181519` |
| circle3c | 단일 루프(둥근 사각) | 3 | 36.7m | 4.6×3.4m | `circle3c_182005` | `circle3c_182004` |
| circle3d | 단일 루프(큰 타원) | 3 | 44.3m | 5.6×4.0m | `circle3d_183550` | `circle3d_183550` |
| eight3b  | 8자 | 3 | 64.3m | 5.6×3.9m | `eight3b_183942` | `eight3b_183941` |
| eight3c  | 8자 | 3+ | 70.4m | 5.6×4.8m | `eight3c_185421` | `eight3c_185421` |

접미사 HHMMSS는 각 기계가 실제로 녹화를 시작한 시각이며, 두 기계가 최대 1초 어긋난다.
기존 로그·리포트가 이 6자리로 세션을 부르므로 추적성을 위해 남겼다.

## 세션별 주의사항

- **circle3d** — camB가 **앞 102.4초(전체의 63%)만** 녹화하고 끊겼다. camA는 161.2초 전체.
  또한 camB의 `metadata.yaml`과 `rgbd_timestamp_associations.json`이 원본에서 각각
  누락·0바이트여서 db3만으로 재생성했다. epoch는 RealSense 프레임 메타데이터의
  Global Time에서 복원했고, 정상 세션으로 검증 시 recorder 원본 대비 **−7.9 ± 18.6 ms**
  차이다(페어링 자체는 6412/6412 완전 일치).
- **eight3b** — camB db3가 **0바이트**(수집 측 녹화 실패). 실질적으로 camA 단독 세션이다.
  ORB-SLAM3가 이 세션에서만 크게 틀리는데, 사람이 흰 파티션을 들고 지나가 4.35m 환영
  이동이 생겼고 이를 지울 loop closure가 검출되지 않은 탓이다. 재현 3/3.
  `_rep1`, `_rep2`는 그 재현 실행이다.
- **eight1** — 정지 비율 20%로 가장 높고, 시작↔끝 영상이 겹치지 않아 종점 실측이 유일하게
  불가능한 세션이다.
- **circle3d** — LiDAR 기준궤적이 0.238m로 이 데이터셋에서 가장 나쁘다(ORB는 0.017m).
  이 기준의 분해능이 ~0.25m라는 근거이며, 그보다 작은 차이로 순위를 매기면 안 된다.

## 원본 → 입력 데이터 변환

**`build_inputs.py`가 정본 진입점**이다. `raw/` 한 쪽만 있으면 파이프라인이 먹을 수 있는
`rgbd_standard/`, ORB 설정, 실행 매니페스트까지 전부 만들어 준다.

```bash
conda activate sam6d_ros_humble      # rclpy/rosbag2_py/numpy 필요

python3 build_inputs.py --list                              # 발견한 것 확인
python3 build_inputs.py --rig all --dry-run                 # 계획만
python3 build_inputs.py --rig laptop_win_qn90665 --jobs 4   # 카메라 B 전량
python3 build_inputs.py --rig husky_a200_0881 --only eight3b_183942 --force
python3 build_inputs.py --rig all --repair-only             # 사이드카만 복구
python3 build_inputs.py --dataset /다른/데이터셋 --list      # 다른 데이터셋
```

rig 이름 = **디렉터리 이름**이다. `raw/` 폴더를 가진 하위 디렉터리를 rig로 자동 인식하므로
새 장비를 추가할 때 코드를 고칠 필요가 없다. `rigs.json`은 설명과 스위치만 얹는 선택 파일이라
지워도 기본값으로 동작한다.

하는 일:

1. **탐색** — `<rig>/raw/` 아래 세션을 찾고, 0바이트 db3(`eight3b_183941`)처럼 애초에
   못 쓰는 것은 이유를 붙여 건너뛴다.
2. **복구** — recorder가 중간에 죽어 `metadata.yaml`이 없거나
   `rgbd_timestamp_associations.json`이 0바이트인 세션을 db3만으로 되살린다. 후자는
   장식이 아니라 **모든 프레임의 epoch 스탬프 공급원**이라 없으면 변환 자체가 시작되지
   않는다. epoch는 RealSense 프레임 메타데이터의 Global Time 필드에서 복원하며, 정상
   세션 대조 시 recorder 원본과 −7.9 ± 18.6 ms 차이에 페어링은 완전 일치다.
3. **변환** — `convert_recording.py`를 세션당 한 프로세스로 돌린다(depth 정렬이 numpy
   CPU 바운드). 정렬 구현은 검증본이라 그쪽에 그대로 두고 이 스크립트는 몰기만 한다.
4. **검증** — 종료코드를 믿지 않고 **만들어진 bag을 열어 컬러 프레임 수를 세서**
   기대치와 비교한다. 불일치면 `SHORT`로 표시하고 매니페스트에서 빼며 exit 1.
5. **산출** — `settings/<rig>/orb_<세션>.yaml`, `manifest_<rig>.json`,
   `build_report_<rig>.json`. 매니페스트·리포트는 **덮어쓰지 않고 병합**하되 이번 실행이
   건드린 세션의 옛 항목은 먼저 지운다. 그래야 `--only` 한 세션만 돌려도 나머지가 살아남고,
   방금 SHORT 난 세션이 예전 "정상" 기록을 유지하지 못한다.

⚠ **설정 파일은 rig별 디렉터리에 둔다.** 두 rig가 `eight1_173704`, `circle3d_183550`,
`eight3c_185421` 세 이름을 공유하는데 내부 파라미터는 다르다(예: eight1 fx가 카메라 A
388.358 vs 카메라 B 387.097). 평평한 `settings/`를 쓰면 카메라 B가 카메라 A 설정을 조용히
덮어쓴다. 실제로 한 번 발생했다.

ORB-SLAM3 설정은 변환본이 아니라 **SDK 원본**에서 뽑는다. 변환본 camera_info는 RTAB-Map
호환 때문에 D=0인데 컬러 영상에는 왜곡이 남아 있기 때문이다.

`convert_all.py`는 husky 전용 구버전으로, 참조 때문에 남겨만 뒀다.
`manifest_0724.json`과 `convert_all_report.json`은 새 산출물과 내용이 완전히 같아 지웠다
(두 벌이 있으면 한쪽만 갱신되어 어긋난다). SLAM 실행은
`python3 run_slam_0724.py manifest_husky_a200_0881.json --method orbslam3`.

### 다른 노트북으로 옮기기

데이터셋에 고유한 것이 코드에 하나도 박혀 있지 않다. 아래 **3개 파일을 한 디렉터리에**
복사하면 그 기계에서 그대로 돈다.

| 파일 | 역할 |
|---|---|
| `build_inputs.py` | 파이프라인 본체(탐색·복구·변환·검증·기록) |
| `convert_recording.py` | 프레임 단위 depth 정렬 + 표준 bag 작성 (`data_slam/260714/`에 있음) |
| `make_orb_settings_sdk.py` | ORB-SLAM3 설정 YAML 생성. ORB를 안 쓰면 생략 가능 |

`rigs.json`은 선택이다. 도구 탐색 순서는 **스크립트 옆 → 저장소의 형제 디렉터리 → 그 외
형제 디렉터리** 순이고, `--converter`/`--settings-tool`로 직접 지정할 수도 있다.
데이터셋 위치는 `--dataset`으로 준다(기본값은 스크립트가 있는 디렉터리).

낯선 데이터셋으로 실제 검증했다: `rigs.json` 없음, rig 이름 `my_rig_01`, 세션 이름
`scan_alpha`, 사이드카 2종 모두 삭제, 0바이트 세션 1개 혼입 → 자동 인식·자동 복구
(113토픽/4767쌍 재생성)·변환·검증·매니페스트까지 전부 통과했다.

**코드를 고칠 때**: 각 단계가 한 가지 일만 하는 클래스라 보통 한 곳만 손대면 된다.
토픽 이름이 다르면 `SdkStreams`/`OutputTopics`, 디렉터리 이름이 다르면 `Layout`,
변환기가 다르면 `Converter`를 상속해 `command()`만 바꾸면 된다.

### 현황

- **husky**: 8세션 전부 변환 완료 (42,911프레임, 62GiB)
- **laptop**: `circle3d_183550` 1세션만 변환 (3,070프레임, 4.4GiB). 나머지 6세션 미변환.

## 이전 이름 대조

`slam_comparison/{output,visualize}`의 결과 폴더와 `configs/`, `settings/`, `logs/`도
같은 이름으로 함께 바꿨다. 아래 이름을 쓰던 문서(`RESULTS_0724_chungbuk.md`,
`DESIGN_lidar_pseudo_gt.md`, `EVAL_ALL_0724_gt.md`)는 기록물이라 그대로 뒀다.

| 이전 | 현재 |
|---|---|
| `SLAM_RGBD_IMU_LIDAR/260724/recording_20260724_<t>` | `husky_a200_0881/raw/<세션>` |
| `converted/recording_20260724_<t>` | `husky_a200_0881/rgbd_standard/<세션>` |
| `lidar_bags_v5/` | `husky_a200_0881/lidar_bags_v5/` |
| `SAM_RGBD/recording_20260724_<t>` | `laptop_win_qn90665/raw/<세션>` |

세션 디렉터리를 바꾸면서 안쪽 RealSense `.db3` 파일명과 그 bag의 `metadata.yaml`도 같이
맞췄다(`convert_recording.py`가 디렉터리명으로 payload를 찾기 때문). Velodyne·Xsens
하위 bag은 센서 정체와 자체 시각을 담고 있어 이름을 그대로 뒀다.

**삭제된 것**: `recording_20260724_172101`(9.7초 정지 클립, 세 방법 모두 실패한 데이터
한계), SAM측 `184306`(0바이트), `184445`(SLAM 짝 없는 camB 단독 녹화 — 게다가 recorder가
clock_fit scale 1.634로 68.4초를 111.7초로 늘려 스탬프해 타임스탬프 신뢰 불가),
`20260727_172002`(07-27 테스트 흔적, 매니페스트만).
