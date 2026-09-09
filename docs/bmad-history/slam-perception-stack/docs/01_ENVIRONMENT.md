# 01. 실행 환경 — conda 5개 (ROS 2는 시스템이 아니라 conda 안에 있다)

## 0. 가장 중요한 전제

**원본 PC에는 `/opt/ros` 가 없다.** ROS 2 Humble은 전부 conda-forge + **robostack-humble**
채널로 설치돼 있고, `conda activate` 하면 ROS가 자동으로 잡힌다
(`ROS_DISTRO=humble`, `rclpy` import OK, `colcon`/`ros2` CLI도 env 안에 있음).

→ 새 PC에서 `sudo apt install ros-humble-desktop` 부터 시작하지 마라. 아래 env만 만들면 된다.
→ 스크립트 중 `source /opt/ros/humble/setup.bash` 를 조건부로 호출하는 것들이 있는데
  (`hdlgraphslam_ws/scripts/run_hdl_graph_slam.sh` 의 `HDL_SOURCE_OPT_ROS=auto`),
  `ros2` 명령이 이미 PATH에 있으면 건너뛴다. 그대로 두면 된다.

## 1. 환경 5개

| env | 원본 크기 | 쓰는 곳 | 사양 파일 |
|---|---:|---|---|
| `orbslam3` | 832 MB | ORB-SLAM3 빌드·실행 | `_env_specs/orbslam3.*` |
| `hdl_graph_slam_humble` | 4.2 GB | HDL-Graph-SLAM 빌드·실행 | `_env_specs/hdl_graph_slam_humble.*` |
| `rtabmap` | — | RTAB-Map 빌드·실행 | `_env_specs/rtabmap.*` |
| `sam_yolo` | 5.6 GB | SAM-6D **ISM** (YOLO-World / DINOv2 / MobileSAM) | `_env_specs/sam_yolo.*` |
| `sam6d_ros_humble` | 15 GB | SAM-6D **PEM**, ROS 노드, ObjectMemory(pytest) | `_env_specs/sam6d_ros_humble.*` |

부수적으로 `convert_ros1bag_to_ros2bag`, `hdl_graph_slam_noetic` 도 원본 PC에 있었으나
이번 이관 범위에는 넣지 않았다(ROS1 bag 변환이 필요할 때만 별도로 만들면 된다).

## 2. env 당 파일 4종의 의미

- **`<env>.requested_cmds.txt`** — 원본 PC에서 이 환경을 만들 때 실제로 친 conda/mamba 명령.
  **재구축 1순위.** 버전 핀이 느슨해 새 드라이버/GPU에 맞게 풀린다.
- `<env>.full.yml` — 빌드 문자열까지 고정. `conda env create -f <파일>`.
- `<env>.explicit.txt` — 패키지 URL 목록. `conda create -n <env> --file <파일>`. 가장 엄격.
- `<env>.pip.txt` — `pip freeze` 결과. conda env를 만든 뒤 `pip install -r` 로 얹는다.

> `sam6d_ros_humble.history.yml` 은 **없다.** conda의 알려진 버그
> (`CondaValueError: Requested package 'pillow' is not found in 'explicit_packages'`)로
> `--from-history` export가 실패한다. `requested_cmds.txt` 를 쓸 것.

## 3. 권장 절차

```bash
cd _env_specs
cat README.md                                  # 원본 생성 명령 전체
cat orbslam3.requested_cmds.txt                # 1순위: 이걸 한 줄씩 실행
cat hdl_graph_slam_humble.requested_cmds.txt
cat sam6d_ros_humble.requested_cmds.txt

# sam_yolo 는 ROS 없이 pip 위주
conda create -n sam_yolo python=3.11 -y
conda activate sam_yolo
pip install -r sam_yolo.pip.txt

# rtabmap 은 requested_cmds 가 없다 (사후 export). full.yml 로 생성
conda env create -n rtabmap -f rtabmap.full.yml

# 나머지 env 의 pip 의존성 얹기
conda activate sam6d_ros_humble && pip install -r sam6d_ros_humble.pip.txt
```

`requested_cmds` 방식이 실패하면 `conda env create -f <env>.full.yml` 로 대체.

## 4. GPU — 여기가 제일 깨지기 쉽다

원본 PC: **NVIDIA RTX PRO 6000 Blackwell Max-Q**, 드라이버 **580.159.04**.

- `sam6d_ros_humble`: `pytorch=2.7.1=cuda129_generic_py311` (conda-forge, **CUDA 12.9**)
- `sam_yolo`: `torch==2.12.0`, `torchvision==0.27.0` (pip, **CUDA 13 계열**)

두 환경의 CUDA 계열이 **서로 다르다. 의도된 분리이므로 통일하려 하지 마라.**
새 PC의 GPU/드라이버가 다르면 `full.yml`/`explicit.txt` 의 고정 빌드가 오히려 안 맞을 수 있다.
그 경우 `requested_cmds.txt` 로 느슨하게 다시 푸는 쪽이 안전하다.

`sam_yolo.pip.txt` 에 git 의존성이 하나 있어 설치 시 **네트워크가 필요**하다:
`clip @ git+https://github.com/ultralytics/CLIP.git@577b3cfa...`

## 5. 빌드 시 주의

- **ORB-SLAM3**: `orbslam_ws/src/orbslam3_ros2/CMakeLists.txt` 가
  `$ENV{CONDA_PREFIX}/lib/libOpenGL.so.0` 를 참조한다. 즉 **`conda activate orbslam3` 한 상태에서**
  `colcon build` 해야 OpenGL/Pangolin이 잡힌다. `orbslam3.requested_cmds.txt` 의
  glew / libgl-devel / libegl-devel / xorg-* 설치 줄을 빠뜨리지 말 것.
- **RTAB-Map**: conda 환경이라 Python 경로를 명시적으로 넘겨야 한다(→ `docs/12_RTABMAP.md`).
  메모리 때문에 `MAKEFLAGS="-j6"` 로 제한한다(48코어여도 코어 빌드가 OOM 난다).
- **SAM-6D PEM**: `gorilla-core` + `pointnet2` 를 env 안에서 빌드해야 한다.
