# 00. START HERE — 에이전트(codex)용 최우선 문서

이 레포는 **ORB-SLAM3 / HDL-Graph-SLAM / RTAB-Map / SAM-6D / ObjectMemory** 5개
파이프라인의 **코드·설정·문서 전부**다. 실행에 필요한 대용량 바이너리(가중치·CAD·
템플릿·ORB 어휘사전)는 GitHub에 올릴 수 없어 **별도 압축 아카이브**로 배포된다.

> 이 문서를 읽는 에이전트에게: 아래 "복구 절차"를 **순서대로** 수행하면 된다.
> 절차 중 판단이 필요한 지점은 각 단계에서 명시했다. 임의로 순서를 바꾸지 말 것
> (특히 3번 경로 치환은 빌드보다 먼저 해야 한다).

---

## 0. 이 레포에 무엇이 있고 무엇이 없는가

| | 포함 | 크기 |
|---|---|---|
| ✅ 소스코드 (5개 알고리즘 + ROS2 패키지) | 예 | ~104 MB |
| ✅ launch / config / 실행 스크립트 | 예 | — |
| ✅ conda 환경 사양 (`_env_specs/`) | 예 | 360 KB |
| ✅ 연구·분석 문서 (`_bmad_output_for_slam/`, 각 ws `docs/`) | 예 | 5 MB |
| ❌ 모델 가중치 / 체크포인트 | **아카이브 02** | 2.7 GB |
| ❌ CAD `.ply` (자체 스캔, 비공개) | **아카이브 03** | 622 MB |
| ❌ SAM-6D 렌더 템플릿 | **아카이브 04** | 202 MB |
| ❌ ORB-SLAM3 어휘사전 `ORBvoc.txt` | **아카이브 01** | 80 MB |
| ❌ 입력 데이터셋(ROS2 bag, ~380 GB) | **어디에도 없음** — 원본 PC에서 직접 복사 | — |
| ❌ colcon `build/ install/ log/` | 새 PC에서 재생성 | — |

**즉, 아카이브 4개(합 3.6 GB)를 풀면 "실행 가능한 상태"가 되고, 데이터셋은 따로 옮겨야 한다.**

---

## 1. 복구 절차 (순서 고정)

### STEP 1 — 레포 배치

```bash
git clone <이 레포 URL> ~/slam-perception-stack
cd ~/slam-perception-stack
```

> 경로는 자유. 단 아래 `$ROOT` 는 이 디렉터리를 가리킨다고 가정한다.
> `data_slam/`(데이터셋)은 이 레포 **바로 아래**에 두는 것을 기본 레이아웃으로 삼는다.
> 원본 PC 레이아웃이 `<루트>/{orbslam_ws, sam6d_ws, ..., data_slam}` 이었기 때문이다.

### STEP 2 — 대용량 자산 복원

아카이브는 이 레포의 **GitHub Release `assets-v1`** 에 있다.

```bash
gh release download assets-v1 --repo ldh8628/slam-perception-stack --dir ~/slam-stack-assets
bash scripts/restore_assets.sh ~/slam-stack-assets
```

- SHA256 검증 → 분할파일 결합 → **레포 루트 기준 원래 경로 그대로** 해제한다.
- 상세 매핑표: **`docs/02_ASSETS.md`**

### STEP 3 — 원본 PC 절대경로 치환 ⚠ 빌드보다 먼저

실행 스크립트·config 78개 파일에 원본 PC 절대경로가 박혀 있다.

```bash
bash scripts/fix_paths.sh            # dry-run: 무엇이 바뀌는지 확인
bash scripts/fix_paths.sh --apply    # 실제 적용
```

- 치환 내용과 남는 문제는 **`docs/90_PORTABILITY_AND_PITFALLS.md`** 참고.

### STEP 4 — conda 환경 5개 생성

```bash
cat _env_specs/README.md      # 원본 PC의 실제 생성 명령이 들어 있다
```

- **이 프로젝트는 시스템 ROS(`/opt/ros`)를 쓰지 않는다.** ROS 2 Humble이
  conda(robostack) 안에 들어 있다. `sudo apt install ros-humble-*` 로 시작하지 말 것.
- 상세: **`docs/01_ENVIRONMENT.md`**

| env | 쓰는 곳 |
|---|---|
| `orbslam3` | ORB-SLAM3 빌드·실행 |
| `hdl_graph_slam_humble` | HDL-Graph-SLAM 빌드·실행 |
| `rtabmap` | RTAB-Map 빌드·실행 |
| `sam_yolo` | SAM-6D ISM (YOLO-World/DINOv2/MobileSAM) |
| `sam6d_ros_humble` | SAM-6D PEM, ROS 노드, ObjectMemory |

### STEP 5 — 워크스페이스 빌드

각 알고리즘 문서의 "빌드" 절을 그대로 따른다. **환경마다 빌드 명령이 다르다.**

| 알고리즘 | 문서 |
|---|---|
| ORB-SLAM3 | `docs/10_ORBSLAM3.md` |
| HDL-Graph-SLAM | `docs/11_HDL_GRAPH_SLAM.md` |
| RTAB-Map | `docs/12_RTABMAP.md` |
| SAM-6D | `docs/13_SAM6D.md` |
| ObjectMemory | `docs/14_OBJECTMEMORY.md` |
| 2대 노트북 동기 수집 | `docs/15_SENSOR_SYNC.md` |

### STEP 6 — 데이터셋 확보

```bash
cat docs/03_DATASETS.md
```

bag이 없으면 SLAM/SAM-6D 어느 것도 재현 실행이 불가능하다. 단, ObjectMemory의
**pytest(79개)는 데이터 없이도 전부 통과**하므로 이것으로 코드 무결성부터 확인할 수 있다.

### STEP 7 — 점검

```bash
bash scripts/verify_setup.sh
```

conda env / 자산 / 빌드 / 잔존 절대경로 / 데이터셋을 한 번에 체크한다.

---

## 2. 데이터 흐름 (왜 이 5개가 한 레포에 있는가)

```
       [D455 RGB-D + IMU]            [VLP-16 LiDAR]
              │                             │
   ┌──────────┴──────────┐                  │
   ▼                     ▼                  ▼
ORB-SLAM3            RTAB-Map        HDL-Graph-SLAM
(카메라 궤적·희소맵)  (조밀맵 비교군)   (LiDAR 준-GT 궤적)
   │                     └──── slam_comparison ────┘
   │                          (ORB vs RTAB 정량비교)
   │  T_map_cam (30 Hz)
   ▼
ObjectMemory  ◄────── T_cam_obj + class + score
(객체 ID·장기기억 맵)          ▲
                               │
                            SAM-6D
                    ISM(무엇이·어디) → PEM(6D pose)
```

- **ORB-SLAM3**: 카메라 pose 공급자. ObjectMemory의 시간정합 대상.
- **HDL-Graph-SLAM**: LiDAR 기반 준-GT. 카메라 SLAM의 drift 검증용(0724 데이터셋 9/9 PASS).
- **RTAB-Map**: 비교군. 맵이 더 선명(MME↓)하지만 밀도는 ORB가 3배.
- **SAM-6D**: 2단 구조. ISM(인스턴스 분할·인식) → PEM(6D 자세). env가 서로 다르다.
- **ObjectMemory**: SAM-6D 검출을 SLAM pose로 map 좌표계에 올려 객체 단위 장기기억을 만든다.

---

## 3. 에이전트가 자주 틀리는 지점 (먼저 읽어라)

1. **`/opt/ros/humble` 을 찾지 마라.** 없다. conda env activate 만으로 ROS가 잡힌다.
2. **SAM-6D는 env가 2개다.** ISM=`sam_yolo`, PEM=`sam6d_ros_humble`.
   ultralytics 버전 충돌 때문에 의도적으로 분리한 것이니 통합하지 말 것.
3. **CAD는 mm 단위여야 한다.** meter 단위 `.ply`를 넣으면 검출이 **조용히 0건**이 된다.
   (`*.ply.orig_meter` 백업이 있으면 현재 `.ply`는 이미 ×1000 된 것이다.)
4. **ObjectMemory ROS 노드는 MultiThreadedExecutor 필수.** 단일 스레드면 멈춘다.
5. **rosbag2 버전**: 0724_chungbuk의 LiDAR bag은 rosbag2 v9(Jazzy)라 Humble에서 안 열린다.
6. **경로 치환(STEP 3)을 건너뛰면** 스크립트가 원본 PC 경로를 그대로 읽고 실패한다.

전체 목록: `docs/90_PORTABILITY_AND_PITFALLS.md`
