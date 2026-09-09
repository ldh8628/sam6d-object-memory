# 90. 이식성 문제와 함정 모음

새 PC에서 이 스택을 돌릴 때 실제로 걸렸던(또는 걸릴) 것들. **한 번은 읽고 시작해라.**

---

## A. 절대경로 하드코딩 — 70개 파일

원본 PC 경로가 코드에 박혀 있다.

| 원본 경로 | 등장 횟수 | 주로 어디 |
|---|---:|---|
| `/home/ldh9501/temp_ws/CLI_environment/orbslam_ws` | 62 | `orbslam_ws/scripts/configs/*.yaml` (실행 프리셋) |
| `/home/ldh9501/temp_ws/CLI_environment/data_slam` | 32 | 위 config들의 bag 경로 |
| `/home/ldh9501/temp_ws/CLI_environment/sam6d_ws` | 21 | `sam6d_ws/tools/*.sh` 의 `ROOT=` |
| `/home/ldh9501/miniconda3/envs/{sam_yolo,sam6d_ros_humble}` | 22 | `sam6d_ws/tools/*.sh` 의 python 절대경로 |
| `/home/etri/CLI_environment/rtabmap_ws` | 4 | `rtabmap_ws/scripts/run_four_slam_bags.py` 의 `ENV_PREFIX` |

**해결:**

```bash
bash scripts/fix_paths.sh            # dry-run
bash scripts/fix_paths.sh --apply    # 적용
```

기본값은 `NEW_ROOT`=이 레포 루트, `NEW_CONDA`=`$CONDA_PREFIX` 로부터 추론(없으면 `$HOME/miniconda3`),
`NEW_DATA`=`$NEW_ROOT/data_slam`. 데이터를 다른 디스크에 뒀다면:

```bash
NEW_DATA=/mnt/bigdisk/data_slam bash scripts/fix_paths.sh --apply
```

> 이 스크립트는 자기 자신(`scripts/fix_paths.sh`, `scripts/verify_setup.sh`)과
> `_env_specs/`, 연구문서는 건드리지 않는다.

**반대로, 경로가 이미 견고한 것들** (건드릴 필요 없음):
- `orbslam3_ros2/launch/orb_slam.launch.py`, `hdl_bag_run.launch.py`
  → `get_package_share_directory` 로 워크스페이스 루트를 자동 탐지한다.
- `rtabmap_ws/scripts/run_rtabmap_four_bags.py` → `Path(__file__).parents[2]` 사용.
- `hdlgraphslam_ws/scripts/run_hdl_graph_slam.sh` → `BASH_SOURCE` 기준 상대경로.

---

## B. 환경 관련

1. **`/opt/ros` 는 없다.** ROS 2 Humble이 conda(robostack) 안에 있다.
   `sudo apt install ros-humble-*` 로 시작하지 말 것.
   예외: `sensor_sync_ws` 만 시스템 ROS 2 **Jazzy** 를 쓴다.

2. **SAM-6D는 env 2개**(`sam_yolo`=ISM, `sam6d_ros_humble`=PEM). ultralytics 충돌 때문에
   의도적으로 분리했고, **CUDA 계열도 다르다**(13 vs 12.9). 통일하려 하지 마라.

3. **ORB-SLAM3는 `conda activate orbslam3` 상태에서 빌드해야 한다.**
   CMakeLists가 `$ENV{CONDA_PREFIX}/lib/libOpenGL.so.0` 를 직접 참조한다.

4. **RTAB-Map은 `MAKEFLAGS="-j6"` 로 제한.** 48코어여도 코어 빌드가 OOM 난다.
   그리고 Python 경로 4개(`-DPython_EXECUTABLE` 등)를 명시해야 conda python을 찾는다.

5. **GPU 고정 빌드가 안 맞을 수 있다.** 원본은 RTX PRO 6000 Blackwell / 드라이버 580.159.04.
   `full.yml`·`explicit.txt` 가 실패하면 `requested_cmds.txt` 로 느슨하게 재설치.

---

## C. 데이터 관련

6. **SDK 녹화 포맷은 depth가 정렬돼 있지 않다.** `convert_recording.py` / `convert_all.py` 로
   표준 ROS2 bag(`converted/`)을 만든 뒤 사용할 것.

7. **rosbag2 v9(Jazzy) bag은 Humble에서 안 열린다.** (0724 LiDAR bag)
   → `downgrade_bag_metadata.py`

8. **Velodyne은 반 바퀴씩 들어온다.** 2개 병합 필수(`merge_velodyne_sweeps.py`).
   안 하면 겹침 3~5%로 정합 실패.

9. **정지 중엔 키프레임이 안 생긴다.** LiDAR 궤적 조밀화 필수(`densify_lidar_trajectory.py`).

10. **카메라는 D455F**(일부 문서의 "d435i"는 오기), **세션마다 fx가 다르다.**
    세션별 settings를 `make_orb_settings.py` 로 생성할 것.

---

## D. 알고리즘 해석상의 함정

11. **CAD는 mm 단위여야 한다.** meter `.ply` → **에러 없이 검출 0건**.
    `*.ply.orig_meter` 백업이 있으면 현재 `.ply` 는 이미 mm.

12. **`appe_v2_blocks=[2,9]` 와 `appe_v2_gate=0.605` 는 쌍이다.**
    블록만 바꾸고 게이트를 0.55로 두면 2026-07-15의 회귀가 재현된다(택배박스 대량 유입).

13. **DINOv2는 vits14 고정.** 다른 크기로 바꾸면 점수 체계가 어긋난다.

14. **ISM 점수는 확률이 아니라 코사인 유사도**다(실효 범위 0.55~0.80).
    "0.64면 낮다" 같은 판단을 하지 마라 — 우유의 천장이 0.661이다.

15. **오프라인 평가(per-prompt YOLO)와 실파이프라인(shared-pass YOLO) 수치를 섞지 마라.**
    같은 설정에서 F1이 .8087 vs .7408로 다르게 나온다.

16. **ObjectMemory ROS 노드는 MultiThreadedExecutor 필수.** 단일 스레드면 멈춘다.

17. **ORB 순수 localization 모드는 identity를 반환하며 실패한다.**
    Atlas 재사용 + **normal 모드 merge** 를 쓸 것. `.osa` 는 **cwd에 복사**해야 로드된다.

18. **시종점 오차 ≠ drift.** 실제로 다른 곳에서 끝난 세션이 있다(183942: 1.137 m).
    `end_start_match.py` 로 영상 검증 없이 그 수치를 drift로 해석하지 마라.

19. **SLAM 비교는 재생 배속을 통일해야 한다.** 과거 ORB만 rate 0.5를 받던 하네스 결함이 있었다.
    또 `rgbd_odometry` 는 프레임을 버려 **병렬 실행 시 처리량이 반토막** 난다(단독 측정 필수).

20. **준-GT 분해능은 ~0.25 m.** 그 이하 차이로 우열을 논하지 마라.

---

## E. 이관 과정에서 의도적으로 뺀 것

| 항목 | 이유 | 복구 방법 |
|---|---|---|
| `orbslam_ws/src/orbslam3_core/` (1.6 GB) | 빌드에 안 쓰이는 원본 클론 | `git clone https://github.com/UZ-SLAMLab/ORB_SLAM3` (commit `4452a3c`) |
| `sam6d_ws/weights_transfer/` (1.6 GB) | 구 이관용 분할본, `assets-02`가 대체 | 불필요 |
| `sam6d_ws/_ism_research_2026_07/` (2.3 GB) | 종료된 연구 중간산출물 | 결론은 `_bmad_output_for_slam/.../research/*.md` 에 있음 |
| `sam6d_ws/_perf_bottleneck_probe_2026_07_06/` (124 MB) | 프로파일링 프로브(삭제=원상복구) | 불필요 |
| `template/fail/` (695 MB) | 실패한 렌더 | 불필요 |
| `slam_comparison/{output,visualize}` (5.2 GB) | 재실행으로 재생성 | 스크립트는 포함돼 있음 |
| 각 ws `output/`, `outputs/` (40 GB+) | 산출물 | 재실행 |
| `build/ install/ log/` | 머신 종속 | `colcon build` |
| ROS2 bag 일체 (380 GB) | 압축 불가 | `docs/03_DATASETS.md` |
