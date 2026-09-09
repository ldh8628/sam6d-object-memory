# 12. RTAB-Map (`rtabmap_ws`)

RGB-D SLAM 비교군. ORB-SLAM3와 같은 bag을 돌려 **맵 품질 / 궤적 정확도 / 밀도**를 대조한다.

## 1. 구성

```
rtabmap_ws/
├─ src/rtabmap/        ← RTAB-Map 코어 (13 MB)
├─ src/rtabmap_ros/    ← ROS2 래퍼
├─ config/             ← 실행 config + TIERS 카메라 intrinsic json
└─ scripts/            ← 배치 실행·결과 추출
```

문서 2종이 워크스페이스 안에 있다:
- `BUILD_CONDA_NOTES.md` — **conda `rtabmap` env 빌드 절차(검증됨, 9패키지 전부 성공)**
- `TRANSFER_NOTES.md` — 시스템 ROS(`/opt/ros/humble`) 기준 절차 + apt 의존성

## 2. 빌드 (conda 방식 — 이쪽이 검증된 경로)

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate rtabmap          # robostack 계열: activate만으로 ROS2 Humble이 잡힌다
cd rtabmap_ws
rm -rf build install log        # 클린 빌드 시

PYV=3.11
export MAKEFLAGS="-j6"          # ★ 코어 빌드 OOM 방지. 48코어여도 반드시 제한
colcon build --parallel-workers 4 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython_INCLUDE_DIR="$CONDA_PREFIX/include/python${PYV}" \
  -DPython_LIBRARY="$CONDA_PREFIX/lib/libpython${PYV}.so" \
  -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
source install/setup.bash
```

소요 ~20분 (rtabmap 코어만 ~7분).
**Python 경로 4개를 명시적으로 넘기지 않으면** conda의 python을 못 찾아 실패한다.

### 시스템 ROS로 빌드할 경우

`TRANSFER_NOTES.md` 참고. 아래 apt 의존성이 추가로 필요하다:

```bash
sudo apt install ros-humble-apriltag-msgs ros-humble-aruco-msgs \
  ros-humble-aruco-opencv-msgs ros-humble-octomap-msgs ros-humble-grid-map-ros
```

## 3. 실행

```bash
conda activate rtabmap
python scripts/run_rtabmap_four_bags.py          # data_slam 4개 bag 배치 실행
python scripts/export_rtabmap_outputs.py --help  # 궤적/키프레임/조밀맵 추출
```

환경변수로 조절:

| 변수 | 기본값 | 의미 |
|---|---|---|
| `RTAB_BAG_RATE` | `1.0` | bag 재생 배속. 낮추면 프레임 드롭이 줄어 drift가 개선된다 |
| `RTAB_ARGS` | `-d` | RTAB-Map 인자. 기본은 **네이티브 실시간 설정** |
| `RTAB_ODOM_ARGS` | `` | odometry 노드 인자 |
| `RTAB_POSES_ONLY` | 미설정 | 설정 시 조밀맵 조립을 건너뛰어 빠르게 pose만 추출 |
| `RTAB_OUTPUT_SUBDIR` | `` | 출력 하위 디렉터리 |

> `RTAB_ARGS` 를 공격적으로 주면(DetectionRate 0 / LinearUpdate 0 / AngularUpdate 0)
> 프레임마다 loop closure + 노드 생성을 강제해 **실시간이 아니게 되고 백로그·drift가 생긴다.**
> 기본값 `-d` 를 유지하라.

## 4. ORB-SLAM3와 비교할 때 (공정 비교 규칙)

과거 비교에서 실제로 발견된 하네스 결함 2건이 있었다. 반복하지 말 것.

1. **재생 배속을 통일하라.** 예전엔 ORB만 `rate 0.5`를 받아 2배의 처리 시간을 얻고 있었다.
2. **RTAB의 0.9 Hz는 알고리즘 성능이 아니라 "저장 대상(그래프 노드)" 때문이다.**
   두 결함을 고친 뒤 밀도 격차는 33배 → **3배**(ORB 30 Hz vs RTAB 10.5 Hz)로 줄었다.

교정 후 결론:

| 지표 | ORB-SLAM3 | RTAB-Map |
|---|---|---|
| 궤적오차 중앙값 | 0.019 m | 0.043 m |
| **최악값** | **2.395 m** | **0.257 m** |
| 궤적 밀도 | 30 Hz | 10.5 Hz |
| 맵 선명도(MME) | 낮음 | **높음(crisp)** |
| 폐색 강건성 | **강건** | 커버리지 드롭 주의 |

→ **ORB의 이점은 밀도와 폐색 생존이지 정확도가 아니다.**
(중앙값 차이 0.019 vs 0.043은 준-GT 분해능 0.25 m 이하라 판정 불가.)

3. ⚠ `rgbd_odometry` 는 **프레임을 버리므로 병렬 실행 시 처리량이 반토막** 난다. 단독 측정 필수.
4. ⚠ `exact_sync` 는 스탬프가 동일해도 **결과가 악화**됐다(되돌림).

## 5. 알려진 이식성 문제

`scripts/run_four_slam_bags.py` 에 `/home/etri/CLI_environment/...` 절대경로가 남아 있다
(`ENV_PREFIX`). → `bash scripts/fix_paths.sh --apply` 로 치환된다.
`run_rtabmap_four_bags.py` 는 이미 프로젝트 루트 상대경로로 재작성돼 있어 안전하다.
