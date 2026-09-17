# 다른 PC 에 data_slam / data_slam_converted 를 붙일 때

전제: 그 PC 는 `/mnt/nvme1/sam_slam.tar.gz`(2026-08-21, `CLI_environment` 전체, **`data_slam` 은 없음**)를
푼 상태다. 여기에 이번 재구성 결과를 얹으려면 아래를 해야 한다.

## 0. ★ 두 폴더는 반드시 **같이** 옮긴다

`data_slam_converted` 안의 심볼릭 16개가 **전부** `../../../../data_slam/` 을 가리킨다
(`lidar/`, `imu/` 의 payload). 따라서

- `data_slam` 과 `data_slam_converted` 를 **같은 부모 디렉터리 아래** 함께 두면 그대로 살아난다.
- `data_slam_converted` 만 보내야 한다면 `cp -rL` / `rsync -L` 로 실체화한다(+5 GB).
- 안 그러면 **HDL-Graph-SLAM(LiDAR)과 IMU 평가가 불가능**하다. 카메라만 쓰면 무관하다
  (`SAM/`, `SLAM/` 은 실물 파일이다).

용량: `data_slam_converted` 62 G(링크 그대로) / 67 G(실체화). `data_slam` 은 별도.

## 1. 코드·설정 반영 — 151 파일

그 PC 의 코드는 **옛 레이아웃**(`data_slam/0807_chungbuk/{SLAM,SAM}/260807/recording_…`,
`integration/data/<날짜>/<세션>/{slam,sam}`)을 전제한다. 아래를 가져가야 새 레이아웃에서 돈다.

| 분류 | 개수 | 무엇 |
|---|---|---|
| **`converting_launch/`** (신규) | 변환기 | `convert_all.py` + `convert_recording.py` + `calib_offset.py` + `downgrade_bag_metadata.py` + `verify_layout.py` + `known_clock_offsets.json` |
| **`integration/tools/`** (신규) | 36 | `data_slam/*/` 에 섞여 있던 스크립트를 옮긴 것. ★**경로 깊이 `parents[1]`→`parents[2]` 수정**이 들어 있다(안 고치면 RTAB/hdl 실행이 죽는다). `tools/0724/` 의 조밀화 헬퍼 3개는 `data_slam.tar.gz` 에서 회수한 것 |
| `integration/pipeline/` | 7 | `common.py`(`RAW_ROOT`/`DATA_ROOT`/`resolve_session`/`raw_dirs`/`write_manifest`), 나머지는 `slam`/`sam` → `SLAM`/`SAM`, `lidar_bag` → `lidar` |
| `integration/` 최상위 | 3 | `run_integration.py`(`DATA_DIR` → `data_slam_converted`), `make_review.py`(`/ros2_standard` 제거), `make_review_multi.py` |
| `orbslam_ws/scripts/` | 40 | config 38 개의 `bag.path`, 러너 2 개의 `DATA_ROOT` → `data_slam/no_program` |
| `sam6d_ws/_ism_research_2026_07/` | 26 | `CONV` 상수 19 개 + `_legacy_bag_names/` 별칭 폴더(옛 `sam_105018_dense` 이름을 새 경로로 잇는다) |
| `sam6d_realtime/` | 15 | `{realtime,temp}/*.yaml` 의 bag 경로 + ★**`realtime/sam6d_realtime_node.py` 손상 수정** |
| `rtabmap_ws/scripts/` | 1 | `DATA_ROOT` |
| `objectmemory_ws/input/` | 7 | 소실 bag 사유 README (`metadata.yaml` → `LOST_BAG_metadata.yaml` 개명 포함) |
| `.gitignore` | 1 | `data_slam_converted/` 무시 + `converting_launch/` 추적 |

### 1-1. 반드시 챙겨야 할 개별 수정

- **`sam6d_realtime/realtime/sam6d_realtime_node.py`** — 커밋 `cb3c5f7` 에 손상이 있다.
  `_on_pose` 8줄이 `_load_pem` 한가운데(418행)에 끼어들어 모델 생성·체크포인트 적재·
  `trimesh.load_mesh` 원숭이패치가 전부 `_on_pose` 로 빨려 들어갔다 → PEM 미생성,
  매 프레임 `ValueError: string is not a file: <객체이름>`. 그 8줄을 `_load_pem` 뒤로 옮기면 된다.
  (운영은 어차피 `sam6d_split.launch.py` 가 빠르다 — 11.5 Hz vs 3.3 Hz.)
- **`integration/tools/*/` 의 경로 깊이** — `data_slam/<x>/` 에서 `integration/tools/<x>/` 로
  한 단계 깊어졌으므로 `ROOT = HERE.parents[1]` → `parents[2]`, `parents[2]` → `parents[3]`.
### 1-2. 심볼릭 링크는 git 에 없다 — 그 PC 에서 다시 만들어야 한다

```bash
cd <repo>
mkdir -p integration/data
for d in data_slam_converted/*/; do n=$(basename $d); ln -sfnT ../../data_slam_converted/$n integration/data/$n; done
for d in data_slam/no_program/*/; do n=$(basename $d)
  ln -sfnT ../../../data_slam/no_program/$n sam6d_ws/data/ros2_bag/$n
  ln -sfnT ../../data_slam/no_program/$n     rtabmap_ws/data/$n; done
ln -sfnT ../../data_slam/no_program/orbslam_smoke_rgbd orbslam_ws/data/rgbd_bag
ln -sfnT ../../converting_launch/convert_recording.py  sam6d_realtime/data/convert_recording.py
```
확인: `find . -xtype l -not -path "./.git/*"` 가 비어야 한다.

## 2. 지워야 할 중복 데이터

**A. 이미 대체됨 — 지금 지워도 되는 것**

| 그 PC 의 경로 | 용량 | 대체하는 새 경로 |
|---|---|---|
| `integration/data/0807_chungbuk/190624__irregular_multi_loop/slam` | 14.60 G | `data_slam_converted/260807_chungbuk/190624__irregular_multi_loop/SLAM` |
| `integration/data/0807_chungbuk/185223__circle_ccw_8laps/slam` | 13.60 G | `data_slam_converted/260807_chungbuk/185223__circle_ccw_8laps/SLAM` |
| `integration/data/0807_chungbuk/185223__circle_ccw_8laps/sam` | 13.60 G | `data_slam_converted/260807_chungbuk/185223__circle_ccw_8laps/SAM` |
| `integration/data/0807_chungbuk/190624__irregular_multi_loop/sam` | 9.52 G | `data_slam_converted/260807_chungbuk/190624__irregular_multi_loop/SAM` |
| `integration/data/0807_chungbuk/182735__u_turn_cw/slam` | 2.97 G | `data_slam_converted/260807_chungbuk/182735__u_turn_cw/SLAM` |
| `integration/data/0807_chungbuk/182735__u_turn_cw/sam` | 2.97 G | `data_slam_converted/260807_chungbuk/182735__u_turn_cw/SAM` |
| `sam6d_ws/data/ros2_bag/two_table_around/bag` | 6.17 G | `data_slam/no_program/two_table_around` |
| `sam6d_ws/data/ros2_bag/high_texture_around` | 5.63 G | `data_slam/no_program/high_texture_around` |
| `sam6d_ws/data/ros2_bag/low_texture_around/bag` | 4.94 G | `data_slam/no_program/low_texture_around` |
| `sam6d_ws/data/ros2_bag/two_table_around_goback/bag` | 4.39 G | `data_slam/no_program/two_table_around_goback` |
| `sam6d_ws/data/ros2_bag/milk_nomilk_bag` | 4.22 G | `data_slam/no_program/milk_nomilk_bag` |
| `sam6d_ws/data/ros2_bag/milk_0609` | 3.45 G | `data_slam/no_program/milk_0609` |
| `sam6d_ws/data/ros2_bag/two_table_diagonal2/bag` | 3.42 G | `data_slam/no_program/two_table_diagonal2` |
| `sam6d_ws/data/ros2_bag/high_texture_far_close` | 3.36 G | `data_slam/no_program/high_texture_far_close` |
| `sam6d_ws/data/ros2_bag/260804_SAM_longcircle2` | 3.28 G | `data_slam_converted/260804_office/longcircle2/SAM` |
| `sam6d_ws/data/ros2_bag/two_table_goback/bag` | 2.87 G | `data_slam/no_program/two_table_goback` |
| `sam6d_ws/data/ros2_bag/low_texture_far_close/bag` | 2.29 G | `data_slam/no_program/low_texture_far_close` |
| `sam6d_ws/data/ros2_bag/two_table_diagonal1/bag` | 2.20 G | `data_slam/no_program/two_table_diagonal1` |
| `sam6d_ws/data/ros2_bag/only_milk` | 1.32 G | `data_slam/no_program/only_milk` |
| `rtabmap_ws/data/SLAM_3바퀴/bag` | 6.62 G | `data_slam/no_program/SLAM_three_laps` |
| `rtabmap_ws/data/SLAM_직전 반복주행/bag` | 3.98 G | `data_slam/no_program/SLAM_forward_backward_repeat` |
| `rtabmap_ws/data/SLAM_1바퀴 후 와리가리/bag` | 3.36 G | `data_slam/no_program/SLAM_one_lap_back_and_forth` |
| `rtabmap_ws/data/SLAM_1바퀴/bag` | 1.66 G | `data_slam/no_program/SLAM_one_lap` |
| `orbslam_ws/data/rgbd_bag` | 0.16 G | `data_slam/no_program/orbslam_smoke_rgbd` |
| **합계** | **120.6 G** | |

**B. 아직 대체 안 됨 — 재변환 전까지 두어야 하는 것**

| 그 PC 의 경로 | 용량 |
|---|---|
| `integration/data/0807_chungbuk/185901__multi_loop_figure_eight/sam` | 15.51 G |
| `integration/data/0807_chungbuk/185901__multi_loop_figure_eight/slam` | 15.51 G |
| `integration/data/0807_chungbuk/184128__irregular_multi_loop_ccw_8turns/slam` | 12.49 G |
| `integration/data/0807_chungbuk/184128__irregular_multi_loop_ccw_8turns/sam` | 12.49 G |
| `integration/data/0807_chungbuk/183839__irregular_multi_loop_ccw_2turns/sam` | 6.05 G |
| `integration/data/0807_chungbuk/183839__irregular_multi_loop_ccw_2turns/slam` | 6.04 G |
| `integration/data/0807_chungbuk/182943__large_loop_then_small_loop_cw/slam` | 5.76 G |
| `integration/data/0807_chungbuk/182943__large_loop_then_small_loop_cw/sam` | 5.76 G |
| `integration/data/0807_chungbuk/183504__straight_then_circle_ccw/slam` | 5.76 G |
| `integration/data/0807_chungbuk/183504__straight_then_circle_ccw/sam` | 5.76 G |
| **합계** | **91.1 G** |

**C. bag 이지만 중복 아님 — SLAM 산출물, 보존**

| 경로 | 용량 |
|---|---|
| `hdlgraphslam_ws/output/vlp16_bag_hdl_graph_slam_20260608_104604/hdl_recorded_bag` | 0.42 G |
| `integration/output/0807_185223_circle8/slam_rtabmap/odom_bag` | 0.01 G |
| `integration/output/0807_185223_circle8/slam_hdl/hdl_recorded_bag` | 0.01 G |
| `output_slam/260804_office/rtabmap_dense/longcircle2/odom_bag` | 0.00 G |
| `output_slam/260804_office/hdl_graph_slam/longcircle2/hdl_recorded_bag` | 0.00 G |
| `output_slam/260804_office/hdl_graph_slam/stablization/hdl_recorded_bag` | 0.00 G |
| **합계** | **0.44 G** |

### 정리

- **A 를 지우면 즉시 120.6 GB 회수.** 전부 `data_slam` / `data_slam_converted` 에 같은 것이 있다.
  - `no_program` 계열 17개는 **바이트 동일**(이 PC 에서 전체 md5 로 확인함).
  - 0807 3세션(6 bag)과 `260804_SAM_longcircle2` 는 **재변환본으로 대체**된 것이다.
    프레임 수·길이는 같지만 바이트 동일은 아니다(왜곡계수 D 가 실제값으로 채워진 새 변환기 산출).
- **B 91.1 GB 는 아직 재변환하지 않은 0807 5세션**이다. 지우기 전에
  `convert_all.py --inputs:=data_slam/260807_chungbuk --outputs:=data_slam_converted/260807_chungbuk`
  를 돌려 대체본을 만들 것. 원본(SDK)이 `data_slam` 에 있으므로 언제든 다시 만들 수 있다.
- **C 0.44 GB 는 SLAM 이 만들어 낸 출력 bag** 이라 중복이 아니다. 그대로 둔다.

전부 정리하면 **A 120.6 + B 91.1 = 211.7 GB** 가 빠지고, 그 PC 에도
"ros2 bag 은 `data_slam` / `data_slam_converted` 에만" 원칙이 선다.

## 3. 검사

```bash
python3 converting_launch/verify_layout.py --inputs:=data_slam_converted \
        --against:=converting_launch/baseline_before.json
find . -name metadata.yaml -not -path "./.git/*" \
     -not -path "./data_slam/*" -not -path "./data_slam_converted/*" \
  | while read f; do d=$(dirname "$f"); ls "$d"/*.db3 >/dev/null 2>&1 && echo "★ $d"; done
```
두 번째 명령에서 SLAM **출력** bag(위 C) 말고 다른 것이 나오면 중복이 남은 것이다.
