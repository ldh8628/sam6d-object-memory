# 10. ORB-SLAM3 (`orbslam_ws`)

RGB-D(+선택적 IMU) 카메라 SLAM. 이 스택에서 **카메라 pose 공급자** 역할을 하며
ObjectMemory가 이 궤적 위에 객체를 올린다.

## 1. 구성

```
orbslam_ws/
├─ src/ORB_SLAM3/          ← 실제로 빌드·링크되는 코어 (수정본)
│   ├─ Vocabulary/ORBvoc.txt   ← 아카이브 01 로 복원
│   └─ lib/libORB_SLAM3.so     ← 빌드 산출물(레포에 없음)
├─ src/orbslam3_ros2/      ← ROS2 래퍼 (rgbd_node.cpp, launch, config)
├─ data/                   ← 카메라 settings yaml + 스모크용 bag(별도 복사)
└─ scripts/                ← 배치 실행·평가 스크립트
```

> `src/orbslam3_core/` (원본 UZ-SLAMLab 클론, 1.6 GB)는 이관에서 제외했다. 빌드에 쓰이지 않는다.
> 필요하면 `git clone https://github.com/UZ-SLAMLab/ORB_SLAM3` (commit `4452a3c`).

## 2. 빌드

```bash
cd orbslam_ws
source ~/miniconda3/etc/profile.d/conda.sh
conda activate orbslam3                 # ← ROS2 Humble이 이 env 안에 있다
colcon build --symlink-install --executor sequential --parallel-workers 1 \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

- **`conda activate orbslam3` 없이 빌드하면 실패한다.** CMakeLists가
  `$ENV{CONDA_PREFIX}/lib/libOpenGL.so.0` 를 직접 참조한다(Pangolin/OpenGL).
- `--parallel-workers 1` 은 메모리 보호용. 줄이지 말 것.
- ORB_SLAM3 코어(`libORB_SLAM3.so`, `libDBoW2.so`, `libg2o.so`)가 없으면
  `src/ORB_SLAM3/build.sh` 를 먼저 돌려야 할 수 있다.

## 3. 실행

```bash
ros2 launch orbslam3_ros2 orb_slam.launch.py config:=no_cli_rgbd.yaml
```

config는 `src/orbslam3_ros2/config/*.yaml`. launch가 워크스페이스 루트를 자동 탐지하므로
config 안의 경로는 **워크스페이스 상대경로**로 써도 된다.

주요 키:

| 키 | 의미 |
|---|---|
| `bag.path` | 재생할 ROS2 bag (상대경로 = ws 루트 기준) |
| `bag.rate` | 재생 배속. **비교 실험 시 반드시 통일할 것**(과거 ORB만 0.5로 준 하네스 결함이 있었음) |
| `orbslam3.vocabulary_path` | `src/ORB_SLAM3/Vocabulary/ORBvoc.txt` (아카이브 01) |
| `orbslam3.settings_path` | 카메라 intrinsic yaml. **세션마다 다르다** |
| `runtime.use_imu` | IMU 사용 여부 |
| `runtime.localization_mode` | 순수 위치추정 모드 (⚠ 아래 4절) |
| `dense_map.*` | 조밀맵 생성·저장(pcd/ply) |

토픽 기본값:
`/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`,
출력 pose `/orbslam3/pose`, 맵포인트 `/orbslam3/map_points`.

## 4. 알려진 동작·함정

1. **IMU 배선은 구현돼 있고, `MultiThreadedExecutor` 가 필수다.** 단일 스레드면 멈춘다.
   부드러운 모션에서 IMU의 이득은 정확도보다 **처리량·강건성** 쪽이다.

2. **재위치추정(relocalization)에서 "순수 localization 모드"는 identity를 반환하며 실패한다.**
   검증된 방법은 **Atlas 재사용 + normal(mapping) 모드 merge**다:
   - settings yaml에 Atlas 키를 넣고 `.osa` 맵 파일을 **cwd로 복사**해야 로드된다.
   - 이 방식으로 105018 세션 99.1% 정합 달성.
   - 동시에 여러 실험을 돌릴 땐 `ROS_DOMAIN_ID` 를 분리해 토픽 충돌을 막는다.

3. **조밀맵 ghosting**: 최적화된 pose로 재투영하는 경로가 기본 ON이다(회귀 방지).
   다만 drift가 작은 bag에서는 효과가 미미하며, 표면을 선명하게 하려면 TSDF가 필요하다.

4. **시종점 오차 ≠ drift.** 실제로 다른 지점에서 끝난 세션이 있다
   (183942는 1.137 m 떨어진 곳에서 종료). `data_slam/0724_chungbuk/end_start_match.py` 로
   영상 검증을 거치지 않은 시종점 오차 수치는 신뢰하지 말 것.

5. **SO3 NaN 크래시**가 병렬 실행 시 4/9 세션에서 발생했고, **단독 재실행으로 전량 복구**됐다.
   배치 실행이 실패하면 먼저 단독으로 다시 돌려볼 것.

## 5. 스크립트

| 스크립트 | 용도 |
|---|---|
| `scripts/run_orbslam_four_bags.py` | rgbd_bag 4종 배치 실행 |
| `scripts/run_imu_compare.py` | IMU on/off 비교 |
| `scripts/compute_slam_metrics.py` | 궤적 지표(RPE 등) 계산 |
| `scripts/eval_relocalization.py` | 재위치추정 평가 |
| `scripts/loop_consistency_orbslam.py` / `_rtabmap.py` | loop 일관성 |
| `scripts/plot_dense_live_vs_kf.py`, `plot_threeway_topview.py` | 시각화 |
| `scripts/configs/*.yaml` | 실행 프리셋. **절대경로 포함 → `scripts/fix_paths.sh` 대상** |

## 6. 스모크 테스트

```bash
conda activate orbslam3 && source install/setup.bash
ls -lh src/ORB_SLAM3/Vocabulary/ORBvoc.txt          # 145 MB 있어야 함
ros2 launch orbslam3_ros2 orb_slam.launch.py config:=no_cli_rgbd.yaml
# 로그에 "Loading ORB-SLAM3 vocabulary" → "Vocabulary loaded!" 가 나오면 기동 성공
```
(`data/rgbd_bag` 이 필요 — `docs/03_DATASETS.md`)
