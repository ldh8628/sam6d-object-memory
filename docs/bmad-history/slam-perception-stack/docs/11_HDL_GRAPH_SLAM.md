# 11. HDL-Graph-SLAM (`hdlgraphslam_ws`)

VLP-16 LiDAR 그래프 SLAM. 이 스택에서는 **카메라 SLAM의 준-GT(pseudo ground truth)**
를 만드는 데 쓴다 (0724_chungbuk 9세션 전부 PASS, 오차 0.012~0.238 m).

## 1. 구성

```
hdlgraphslam_ws/src/
├─ hdl_graph_slam/     ← 본체 (launch, config, nodelet)
├─ ndt_omp/            ← NDT 정합 (멀티스레드)
└─ fast_gicp/          ← GICP 정합 (thirdparty 포함, 45 MB)
```

## 2. 빌드

```bash
cd hdlgraphslam_ws
source ~/miniconda3/etc/profile.d/conda.sh
conda activate hdl_graph_slam_humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 3. 실행

```bash
ros2 launch hdl_graph_slam hdl_bag_run.launch.py config:=no_cli_vlp16.yaml
```

`src/hdl_graph_slam/config/no_cli_vlp16.yaml`:

```yaml
bag:
  path: data/vlp16_bag        # ← LiDAR bag을 여기에 두거나 심링크
  rate: 0.5
hdl_graph_slam:
  launch_file: hdl_graph_slam_501.launch.py
  raw_points_qos: reliable
output:
  dir: auto
```

쉘 래퍼로도 실행 가능하며, 환경변수로 파라미터를 덮어쓴다:

```bash
HDL_RAW_POINTS_TOPIC=/velodyne_points \
HDL_LAUNCH_FILE=hdl_graph_slam_501.launch.py \
bash scripts/run_hdl_graph_slam.sh
```

| 환경변수 | 기본값 |
|---|---|
| `HDL_RAW_POINTS_TOPIC` | `/velodyne_points` |
| `HDL_POINTS_TOPIC` | `/filtered_points` |
| `HDL_RAW_POINTS_QOS` | `best_effort` (tutorial launch면 `reliable`) |
| `HDL_DISTANCE_FAR_THRESH` | `20.0` |
| `HDL_ENABLE_FLOOR` | `true` |
| `HDL_SOURCE_OPT_ROS` | `auto` — `ros2` 가 PATH에 있으면 `/opt/ros` source 생략 |

## 4. LiDAR 준-GT 파이프라인 — 반드시 지켜야 할 3가지

0724_chungbuk 데이터로 준-GT를 만들 때 실제로 걸렸던 함정이다.

1. **Velodyne 스윕이 반 바퀴씩 들어온다.**
   한 메시지가 360°가 아니라 절반이라 겹침이 3~5%까지 떨어진다.
   → **연속한 2개를 병합**해야 한다: `data_slam/0724_chungbuk/merge_velodyne_sweeps.py`

2. **LiDAR bag만 rosbag2 v9(Jazzy) 포맷**이라 Humble에서 열리지 않는다.
   → `data_slam/0724_chungbuk/downgrade_bag_metadata.py`

3. **정지 중에는 키프레임이 생기지 않는다.**
   그래프가 듬성해져 궤적 비교가 불가능해지므로 **조밀화가 필수**다:
   `data_slam/0724_chungbuk/densify_lidar_trajectory.py`

관련 스크립트 묶음: `data_slam/0724_chungbuk/{run_lidar_gt.py, finish_lidar_gt.py,
validate_lidar_gt.py, plot_lidar_gt.py, dump_hdl_graph.py}`
설계 문서: `data_slam/0724_chungbuk/DESIGN_lidar_pseudo_gt.md`

## 5. 한계 (해석할 때 중요)

- 준-GT의 **분해능이 약 0.25 m** 다. 그보다 작은 차이로 SLAM 우열을 논하지 말 것.
  실제로 183550 세션에서는 LiDAR 쪽이 카메라보다 나빴다.
- **미해결**: `T_velo_cam`(LiDAR↔카메라 extrinsic) 캘리브레이션이 아직 없다.
  현재 비교는 궤적 형상 정합에 의존한다.

## 6. 스크립트

| 스크립트 | 용도 |
|---|---|
| `scripts/run_hdl_graph_slam.sh` | 단일 실행 래퍼 |
| `scripts/run_hdl_bag_benchmark.sh` | bag 벤치마크 |
| `scripts/export_hdl_results.sh` | 결과 추출 |
| `scripts/build_tiers_lidar_comparison.py` | TIERS 데이터 비교 |
| `scripts/inspect_pointcloud_bag.py`, `summarize_hdl_bag.py` | bag 점검 |
| `scripts/check_hdl_state.sh` | 노드 상태 확인 |
