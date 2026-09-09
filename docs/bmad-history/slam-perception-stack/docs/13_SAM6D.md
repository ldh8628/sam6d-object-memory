# 13. SAM-6D (`sam6d_ws`) — 인식(ISM) + 6D 자세추정(PEM)

RGB-D 프레임에서 **어떤 물체가 어디에 있고(ISM) 어떤 자세인지(PEM)** 를 구한다.
ObjectMemory의 입력을 만드는 단계.

## ⚠ 가장 먼저 알아야 할 3가지

1. **conda env가 2개다.** ISM=`sam_yolo`, PEM=`sam6d_ros_humble`.
   ultralytics 버전이 충돌해서 **의도적으로 분리**한 것이다. 하나로 합치려 하지 마라.
2. **운영 ISM은 원본 SAM-6D의 `detector.py` 가 아니다.**
   실제 실행 경로는 이 저장소의 자체 구현 **`yolo_ism.py` / `yolo_ism_object_n.py`** 이며,
   제안기는 FastSAM이 아니라 **YOLO-World + MobileSAM** 이다.
   (원본 `sam6d_master/SAM-6D/` 는 PEM과 렌더러 때문에 유지한다.)
3. **CAD는 mm 단위.** meter `.ply`를 넣으면 **에러 없이 검출 0건**이 된다.

## 1. 구성

```
sam6d_ws/
├─ yolo_ism.py                 ← ISM 코어 (단일객체 계보)
├─ yolo_ism_object_n.py        ← ISM 멀티객체 실행기 (★ 운영 진입점)
├─ ism_hsv.py                  ← HSV 색 하드게이트 (Phase 1B/1C)
├─ ism_training_free_gate.py   ← 학습 없는 게이트 개선 (기본 OFF)
├─ configs/yolo_ism_objects.yaml  ← ★ 모든 운영 파라미터가 여기 있다
├─ config/camera_intrinsics.json
├─ tools/                      ← 파이프라인 실행·평가·진단 스크립트 (60여 개)
├─ src/sam6d_ros/              ← ROS2 노드 (sam6d_inference_node, sam6d_multiobject_node)
├─ sam6d_master/SAM-6D/        ← 원본 SAM-6D (PEM + Render + ISM 원본)
├─ data/cad/                   ← 아카이브 03
├─ template/                   ← 아카이브 04
└─ weights/, *.pt              ← 아카이브 02
```

## 2. 빌드 / 준비

```bash
cd sam6d_ws
conda activate sam6d_ros_humble
colcon build --symlink-install
source install/setup.bash
```

PEM은 `gorilla-core` + `pointnet2` 가 env 안에 빌드돼 있어야 한다(`_env_specs` 참고).
ISM만 쓸 거라면 colcon 빌드 없이 `sam_yolo` env에서 파이썬 스크립트를 직접 실행해도 된다.

## 3. 파이프라인 (2단)

```
bag ─► [ISM: sam_yolo env] ─► 프레임별 (mask RLE, class, score, bbox)
                                    │
                                    ▼  build_pem_inputs / build_ism_inputs_imu
                              RGB / aligned depth / camera.json / seg RLE / CAD / 템플릿
                                    │
                              [PEM: sam6d_ros_humble env]
                                    ▼
                              detection_pem.json  (T_cam_obj, score)
```

### End-to-end 한 방에

```bash
bash tools/run_imu_e2e.sh <bag> [stride]     # 예: bash tools/run_imu_e2e.sh SAM_circle 10
```
Stage A(ISM, `sam_yolo`) → Stage B(PEM, `sam6d_ros_humble`) → Stage C(결과 재배치)를 순서대로 수행한다.
**스크립트 안에 원본 PC의 `ROOT`/python 절대경로가 박혀 있으니 `scripts/fix_paths.sh` 를 먼저 돌릴 것.**

### 단계별 실행

```bash
# ISM 단독 (sam_yolo)
conda activate sam_yolo
python yolo_ism_object_n.py --config configs/yolo_ism_objects.yaml \
    --bag data/ros2_bag/SAM_circle --stride 10 --max-frames 0

# PEM 입력 번들 생성
python tools/build_pem_inputs.py --bag SAM_circle --max-frames 3    # 스모크용 3프레임

# PEM 배치 (sam6d_ros_humble)
conda activate sam6d_ros_humble
bash tools/run_pem.sh SAM_circle
# 또는 python tools/run_pem_batch.py --bags SAM_circle
```

| 인자 | 기본값 | 의미 |
|---|---|---|
| `--config` | `configs/yolo_ism_objects.yaml` | 객체·게이트 설정 |
| `--bag` | — | ROS2 bag 경로 |
| `--stride` | 0 | 프레임 간격 (0=전부) |
| `--max-frames` | 0 | 0=무제한 |
| `--rebuild-features` | off | 템플릿 특징 캐시 강제 재생성 |
| `--device` | 자동 | `cuda:0` 등 |

## 4. `configs/yolo_ism_objects.yaml` — 운영 파라미터

이 파일 하나가 인식 성능 전체를 지배한다. 값에는 전부 실측 근거가 달려 있으니
**주석을 읽지 않고 숫자만 바꾸지 마라.**

| 키 | 현재값 | 의미 |
|---|---|---|
| `weights` | `yolov8m-worldv2.pt` | YOLO-World 제안기 |
| `top_k` | 3 | 객체·프레임당 제안 수 |
| `score_threshold` | 0.02 | YOLO 신뢰도 하한 (Phase 1A에서 전객체 통일) |
| `similarity_threshold` | 0.35 | semantic(cls) 1차 게이트 |
| `appe_gate` | 0.55 | masked-appearance 2차 게이트 |
| `appe_blocks` | `[11]` | appearance에 쓰는 DINOv2 블록 |
| `hsv_gate_enabled` | **true** | Phase 1C HSV 색 하드게이트 |
| `hsv_gate_threshold` | 0.12140 | LOO 6폴드에서 동일하게 나온 동결값 |
| `hsv_hue_shift_deg` / `hsv_sat_gain` | -6 / 1.3 | 렌더↔실사 도메인갭 보정(레퍼런스 쪽에 적용) |
| `appe_v2_enabled` | **true** | Phase 3 appearance-v2 |
| `appe_v2_blocks` / `appe_v2_gate` | `[2,9]` / 0.605 | ⚠ **블록과 게이트는 반드시 쌍으로 움직인다** |
| `cross_object_nms_enabled` | **true** | 같은 박스를 두 라벨이 주장할 때 억제 |
| `cross_object_nms_iou` | 0.9 | 동일 박스로 볼 IoU |
| `nms_rank_blocks` | `[2,9]` | NMS 승자 결정용 랭킹 점수 |
| `training_free_gate_enabled` | false | Phase 2.2 (기본 OFF) |

등록 객체(멀티객체): `milk, choco_hazelnut_high/low, Febreze_high/low, Mugcup_high/low,
saffron, Sauce_high/low, Sikhye_high/low, Bear, Rabbit, Dinosaur`

### 절대 하지 말아야 할 변경

- **`appe_v2_blocks=[2,9]` 를 쓰면서 게이트를 0.55로 두는 것.**
  2026-07-15에 이걸로 회귀가 났다(점수 분포가 올라가 갈색 택배박스가 대량 유입).
  블록을 바꾸면 게이트도 0.605로 같이 옮겨야 한다.
- **`appe_gate` 전역 상향.** 특정 객체의 FP를 잡으려다 다른 객체 TP를 죽인다.
- **DINOv2 체크포인트를 vits14 이외로 교체.** 점수 스케일 체계가 전부 어긋난다.

## 5. 성능 기준값 (재현 시 비교용)

사람이 라벨링한 GT 959셀 기준:

| 설정 | TP | FP | FN | F1 |
|---|---:|---:|---:|---:|
| Phase 1C (appe block11, gate 0.55) | 645 | 63 | 314 | .7738 |
| + cross-object NMS | 643 | 31 | 316 | .7875 |
| appe_v2[2,9] gate 0.605 | 688 | 96 | 271 | .7894 |
| **현재 기본값(둘 다 ON)** | **670** | **28** | **289** | **.8087** |

실제 파이프라인(shared-pass YOLO) 기준으로는 F1 .7162 → **.7408**.
⚠ 오프라인 평가(per-prompt YOLO)와 실파이프라인(shared-pass) 수치를 **섞어 비교하지 마라.**

남은 최대 문제:
- 잔존 FP 28건 중 24건이 **초코하임이 갈색 택배박스에 흡착**하는 케이스(오래된 별건 문제).
- FN의 주원인은 게이트가 아니라 **localization(YOLO 제안 위치)** 이다.
  개선 방향으로 **YOLOE union + 라우팅 제거**가 권장돼 있다(설치된 ultralytics에 이미 존재, 추론 5s vs 현행 38s).
- 인형류(Bear/Dinosaur)는 색·형태 점수의 변별력 자체가 낮다(AUC .57/.43).

## 6. ROS2 노드

| 노드 | 설명 |
|---|---|
| `sam6d_ros/sam6d_inference_node.py` | 단일객체 추론 노드 |
| `sam6d_ros/sam6d_multiobject_node.py` | 멀티객체 노드 |

```bash
ros2 launch sam6d_ros sam6d_inference.launch.py config:=<...>
```

> ⚠ **ObjectMemory 라이브 노드와 연결하려면 수정이 필요하다.**
> `sam6d_multiobject_node.py` 는 현재 `geometry_msgs/PoseArray` 를 publish 하는데,
> 여기엔 **객체 ID가 없다**(이름이 오버레이에만 존재). ObjectMemory 노드를 구동하려면
> `vision_msgs/Detection3DArray` 로 바꿔야 한다. 자세한 내용은 `docs/14_OBJECTMEMORY.md`.

## 7. 성능(속도) 병목

실측 프로파일: **MobileSAM 36% > DINOv2 cls 26% > patch 중복 9% > YOLO 상수 8%**,
템플릿 매칭은 0.8%로 사실상 무료.
추가로 **운영 진입점이 배치 `recognize_frame` 을 안 쓰고 있어** 프레임당 130 ms → 32 ms 여지가 있고,
**YOLO를 프롬프트마다 10패스** 돌리는 것이 전체의 48%다(`multi_label` 1패스로 recall 98.9% 유지 가능).

## 8. 스모크 테스트

```bash
conda activate sam_yolo && cd sam6d_ws
ls -lh sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth \
       sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/dinov2/dinov2_vits14_pretrain.pth \
       mobile_sam.pt weights/clip/ViT-B-32.pt
ls data/cad/*/*.ply | head
python tools/build_pem_inputs.py --bag <bag> --max-frames 3
```

## 9. 템플릿 재생성 (아카이브 04 없이 갈 때)

```bash
bash tools/render_all_templates.sh                     # 전체 객체 42뷰
# 단일 객체 CPU 렌더
python sam6d_master/SAM-6D/Render/render_custom_templates_cpu.py --help
```
Dinosaur는 렌더 색이 실물과 어긋나 **레퍼런스 hue +9 (OpenCV 단위, = +18°)** 보정이
캐시 빌드 시점에 적용된다. 재렌더 시 이 보정이 유지되는지 확인할 것.
