# integration — 하나의 실행 파일로 SLAM bag + SAM bag 동시 처리

```bash
cd /home/ldh9501/temp_ws/CLI_environment
python3 integration/run_integration.py <pair>            # pair = integration/data/<pair>
```

`run_integration.py` 하나가 입력 폴더 한 개(SLAM bag + SAM bag)를 받아

| 스테이지 | 입력 | 처리 | 결과 |
|---|---|---|---|
| 1 `slam` | SLAM bag | ORB-SLAM3 RGB-D (`conda: orbslam3`) | 궤적 / map points / dense map |
| 2 `sam` | SAM bag | SAM-6D ISM → PEM (`conda: sam_yolo` → `sam6d_ros_humble`) | 객체별 6D pose + 오버레이 |
| 3 `fuse` | 1+2 | object_memory 융합 | **SLAM map 좌표계 객체 지도 + 상면도 1장** |

를 순서대로 돌리고 `integration/output/<pair>/` 아래로 모읍니다.
1·2를 별도 프로세스로 띄우는 이유는 ORB-SLAM3(ROS 2)와 SAM-6D
(ultralytics/torch)의 conda 환경이 한 인터프리터에 공존할 수 없기 때문입니다.
3은 `objectmemory_ws/object_memory`가 순수 표준 라이브러리라 그대로 import해서
쓰므로 별도 환경이 필요 없고, 실행 스크립트 자체도 표준 라이브러리만 씁니다
(시스템 `python3`로 실행).

## Stage 3 융합 규약

기준 좌표계는 **SLAM 카메라의 map 프레임**(= ORB-SLAM3 첫 카메라 포즈)입니다.

```
T_map_obj = T_map_camSLAM(t) · X_camSLAM_camSAM · T_camSAM_obj
```

* `X`는 **검출이 아니라 포즈 스트림에 곱합니다**(`T_map_camSAM = T_map_camSLAM · X`).
  결과 위치는 같지만, 존재확률 필터의 P_D(시야 모델)가 실제로 검출을 만든
  SAM 카메라의 절두체를 보게 되어 랜드마크가 전부 "시야 밖"으로 취급되는 문제를
  피합니다.
* `X` 기본값은 `--extrinsic auto` = SAM bag 옆 `calib/*.npy`가 있으면 그걸 쓰고,
  없으면 손으로 잰 리그 추정치(오른쪽 0.215 m, 아래 0.380 m, 우측 yaw 90°)로
  넘어가며 경고를 찍습니다. **X가 틀리면 같은 물체가 시점마다 다른 곳에 찍혀
  카메라 경로를 따라 번집니다** — 260804 longcircle2에서 캘리브 X는 인스턴스
  29개, 손측정 리그 추정치는 125개였습니다.
* 두 노트북 시계가 어긋난 데이터면 `--time-offset <초>`로 SAM 프레임 스탬프를
  옮깁니다(겹치는 구간이 없으면 경고가 나옵니다).
* 프레임→타임스탬프는 SAM bag의 컬러 토픽 순서에서 직접 읽습니다(sqlite3, 읽기 전용).

## 입력 배치

```
integration/data/<pair>/
    SLAM/{metadata.yaml, *.db3}                   # 경로에 "slam" 포함 -> SLAM
    SAM/{metadata.yaml, *.db3}                    # 경로에 "sam"  포함 -> SAM
    lidar/, imu/, calib/, info.json               # converting_launch/convert_all.py 산출
    [slam/settings/orb_*.yaml]                    # 있으면 ORB 카메라 설정으로 사용
```

* bag 디렉터리는 `metadata.yaml` 유무로 자동 탐색되므로 중간 폴더 이름은 자유입니다.
* 용량이 크므로 심볼릭 링크를 권장합니다.
  ```bash
  ln -sfnT <원본 SLAM 세션> integration/data/<pair>/slam
  ln -sfnT <원본 SAM 세션>  integration/data/<pair>/sam
  ```
* 자동 분류가 애매하면 `--slam-bag/--sam-bag`으로 직접 지정합니다.
* ORB 설정 yaml이 세션에 없으면 bag의 `color/camera_info`로 자동 생성합니다
  (이때 `Stereo.b`만 추정 불가 → `--baseline`, 기본 0.095 m = D455 계열).

## 출력

```
integration/output/<pair>/
    bag_info.json      두 bag의 토픽/내부 파라미터/길이 (probe 결과)
    run_summary.json   무엇이 얼마나 걸려 돌았고 산출물이 어디 있는지
    slam/  trajectory.txt (= CameraTrajectory.txt, TUM 포맷)  keyframes.txt
           orbslam3_map_points.pcd  orbslam3_dense_map.pcd
           orb_config.yaml  orb_settings.yaml  orbslam3_launch.log
    sam/   sam_objects.json   프레임×객체별 6D pose(R, t_mm) + score 요약
           pem_inputs/frame_*/vis_pem_all.png     프레임별 전 객체 pose 오버레이
           pem_inputs/frame_*/pem_<obj>/sam6d_results/detection_pem.json
           ism.log  pem.log  viz.log
    fused/ object_map.json          map 좌표계 영속 객체(=최종 결과)
           map_topview.svg          궤적 + 객체 상면도 한 장
           object_memory_report.md  라이프사이클/결정 리포트
```

`object_map.json`의 객체 한 건:

```json
{"object_id": 16, "object_name": "saffron", "status": "active",
 "confidence": 0.98, "observations": 34, "missed": 3,
 "xyz": [-0.926, 0.145, -0.775], "quat_xyzw": [...], "T_map_obj": [[...4x4...]]}
```

`status`는 object_memory의 라이프사이클입니다 — `active`(현재 확인됨),
`remembered`(오래 관측된 뒤 시야에서 사라짐), `lost`, `tentative`, `deleted`
(유령으로 판단해 폐기, 상면도에는 그리지 않음).

SAM-6D 툴들은 bag 이름과 출력 경로가 `sam6d_ws` 내부로 하드코딩되어 있어,
`sam6d_ws/data/ros2_bag/integration_<pair>` 와
`sam6d_ws/outputs/pem_inputs/integration_<pair>` 두 심볼릭 링크를 만들어
연결합니다. 실제 파일은 `integration/output/` 아래에 있고 기존 툴은 수정하지
않았습니다.

## 자주 쓰는 옵션

```bash
# 실제 지도를 만들려면 SAM 샘플을 촘촘히 (기준 밀도)
python3 integration/run_integration.py <pair> --sam-stride 10 --max-frames 400

python3 integration/run_integration.py <pair> --probe-only        # bag 확인만
python3 integration/run_integration.py <pair> --stage slam        # ORB만
python3 integration/run_integration.py <pair> --stage sam --max-frames 60
python3 integration/run_integration.py <pair> --stage fuse        # 융합만 재실행
python3 integration/run_integration.py <pair> --extrinsic <X.npy> --time-offset 6.6
python3 integration/run_integration.py <pair> --rate 0.5 --no-dense
python3 integration/run_integration.py <pair> --sam-objects milk Mugcup_high
```

* `--max-frames N` (기본 30): SAM bag에서 균등 간격으로 뽑는 컬러 프레임 수.
  `--sam-stride N`을 주면 N프레임마다로 바뀝니다. 전 프레임 처리는 매우 느립니다.
  **융합은 밀도에 민감합니다**: 승격(active)에 관측 2회가 필요해서 샘플이 얇으면
  대부분 tentative로 남습니다. 검출 프레임이 40 미만이면 경고를 찍습니다.
* `--rate`: `ros2 bag play` 배속. 1.0이면 bag 길이만큼 실시간으로 재생됩니다.
* `--assoc-gate`(기본 0.20 m), `--time-tolerance`(기본 0.2 s)로 융합을 조입니다.
* 스테이지를 따로 돌려도 `run_summary.json`은 다른 스테이지 결과를 보존합니다.

## 검증 (2026-08-10, `260804_longcircle2` = 260804_office longcircle2 2대 카메라)

한 번의 명령(`--sam-stride 10 --max-frames 400`)으로:

| 스테이지 | 결과 |
|---|---|
| ORB-SLAM3 | 2166 pose, 87 s, dense map 30 MB |
| SAM-6D | 213프레임 샘플 → 121프레임에서 10객체 296검출, PEM 288/296 성공 |
| 융합 | 288검출/121프레임 전부 융합, 인스턴스 33개 중 생존 12개 |

**융합 정합성 확인**: 동일 입력(기존 `output_slam` 궤적 + 기존 PEM 트리 + 캘리브 X)에서
이 구현의 결과가 검증된 기존 리포트
(`objectmemory_ws/.../reports/object_memory_260804_twocam.md`)의 12개 인스턴스와
id·이름·관측수·xyz(mm 단위)까지 일치합니다.

## 눈으로 확인하기 — `make_review.py` (영상 + HTML)

```bash
/home/ldh9501/miniconda3/envs/sam_yolo/bin/python integration/make_review.py \
    --pair 260804_longcircle2 \
    --object-map integration/output/260804_longcircle2/reloc/fused/object_map.json
```

`integration/output/<pair>/review/`에 다음을 만듭니다.

| 파일 | 내용 |
|---|---|
| `sync_view.mp4` | 위: SLAM/SAM 카메라가 **같은 순간**에 본 RGB 두 패널 · 아래: 같은 순간의 두 카메라 위치·시선 방향 2D 지도 |
| `review.html` | 위 영상 재생 + 프레임 단위 이동/배속 + 현재 시각의 두 카메라 좌표 표시 + 객체 표 |
| `sync_data.json` | 프레임별 (시각, bag 프레임 번호, 두 카메라 xyz) |
| `poster.jpg` | 영상 첫 프레임(HTML 포스터) |

`review.html`을 브라우저로 열면 됩니다(`google-chrome review.html`). 좌우 화살표=1프레임 이동,
스페이스=재생/일시정지. 데이터는 HTML에 인라인이라 로컬 `file://`에서도 동작합니다.

전제: **두 궤적이 같은 맵 좌표계**여야 합니다. 즉 SAM bag이 SLAM 카메라의 Atlas 안에서
relocalize된 결과(`--sam-traj`)를 써야 하며, 기본값은 `orbslam_ws/output/260804_gate/`를 가리킵니다.
다른 세션은 `--slam-traj/--sam-traj/--map-points`로 지정하세요.

주요 옵션: `--fps`(기본 15), `--start/--duration`(구간만), `--trail`(궤적 꼬리 길이 초),
`--crf`(화질/용량). 72초 전체가 15fps에서 약 14 MB, 생성 13초.

패널 라벨의 `dt`는 그 프레임이 기준 시각에서 얼마나 떨어졌는지(±수 ms)라, 동기가 맞는지
프레임마다 직접 확인할 수 있습니다.

## 다음 단계

* `map_topview.svg`는 궤적+객체 평면도까지입니다. 3D(dense map + 객체 박스) 뷰나
  Unity/RViz 연동은 별도 작업입니다.
* extrinsic이 없는 신규 데이터셋은 리그 추정치로 떨어지므로, 새 리그를 쓸 때는
  hand-eye 캘리브(`calib/X_camSLAM_camSAM.npy`)를 먼저 만드는 게 정확도에
  가장 큰 영향을 줍니다.
* 두 노트북으로 수집한 원본은 시계 오차가 있으므로 변환 단계에서 보정하거나
  `--time-offset`을 줘야 합니다.
