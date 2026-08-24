# sam6d_realtime — 실시간 SAM-6D (ROS 2 Jazzy) 이식용 최소 묶음

`ros2 bag play` 나 실제 카메라가 뿌리는 RGB + aligned depth 를 구독해서
물체를 찾고(ISM) 6D 포즈를 구해(PEM) `/sam6d/detections` 로 내보내는 **한 개짜리 노드**와,
그 노드가 돌기 위해 **실제로 필요한 것만** 모아 둔 폴더다.
검증·실험 과정에서 생긴 산출물, 논문용 스크립트, 안 쓰는 가중치, 오프라인 배치 도구는 들어 있지 않다.

대상 환경: conda env **`sam6d`** (ROS 2 Jazzy). 만드는 법은 `_env_specs/sam6d.requested_cmds.txt`.

---

## 1. 예전 실시간 노드와 무엇이 다른가

기존 `sam6d_ws/src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py`(2026-06)는 객체마다
`recognize()` 를 따로 부르는 구조여서 7~8월에 들어간 프레임 단위 개선이 **전부 빠져 있었다.**
이 노드는 오프라인 배치(`tools/build_pem_inputs.py`)와 **같은 함수**
`yolo_ism_object_n.recognize_frame_auto()` 를 부른다. 그래서 아래가 그대로 적용된다.

| 개선 | 무슨 일을 하나 | 어디서 켜지나 |
|---|---|---|
| **multi_label** | 박스 하나가 임계 넘는 프롬프트를 전부 유지 (예전엔 top-1만) | config `yolo_multi_label: true` + 노드가 YOLO NMS 를 감쌈 |
| **relative assignment** | 박스마다 주인 객체를 먼저 뽑고, 절대 게이트는 그 뒤에 | config `relative_assignment_enabled: true` |
| **colour tie-break** | 형태로 구분 안 되는 짝(인형 3종)은 HSV 색이 승자를 결정 | config `color_tiebreak_template_sim: 0.60` |
| **cross-object NMS** | 같은 박스를 두 라벨이 주장하면 `rank_appe` 로 하나만 남김 | config `cross_object_nms_enabled: true` |

기존에도 적용돼 있던 것(HSV 하드게이트, appe 블록 [2,9]·게이트 0.605)은 그대로 유지된다.
기동 로그가 이 값들을 전부 찍으므로 **켜져 있는지 눈으로 확인할 수 있다**:

```
[ism] relative_assignment=True colour_tiebreak=0.6 cross_object_nms=True hsv_gate=True appe_blocks=[2, 9]@0.605
```

그리고 요구사항이던 **"모든 프레임을 처리하지 않는다"** 가 구조로 들어가 있다.
한 장을 처리하는 동안 들어온 프레임은 미들웨어(큐 깊이 1)와 busy 플래그가 버리고,
끝나면 그 시점의 최신 프레임부터 다시 시작한다.

---

## 2. 폴더 구성

```
sam6d_realtime/
├── realtime/                         ← 이번에 새로 쓴 것
│   ├── sam6d_realtime_node.py        실시간 노드 본체
│   ├── launch/sam6d_realtime.launch.py   노드+bag재생+정상종료를 한 번에
│   ├── run_bag_example.yaml          bag 재생용 실행 설정
│   └── run_live_example.yaml         실제 카메라용 실행 설정
├── yolo_ism.py                       ← sam6d_ws 원본을 그대로 복사(수정 금지)
├── yolo_ism_object_n.py              ← 〃  (recognize_frame_auto 등 판정 본체)
├── ism_hsv.py                        ← 〃  (HSV 게이트 authority)
├── ism_training_free_gate.py         ← 〃  (기본 OFF지만 import 되므로 필요)
├── configs/yolo_ism_objects.yaml     ← 〃  (운영 config)
├── tools/build_hsv_template_cache.py ← 〃  (HSV 캐시 재빌드용, 아래 함정 참고)
├── mobile_sam.pt, yolov8m-worldv2.pt 세그멘터 / YOLO-World 가중치
├── sam6d_master/SAM-6D/
│   ├── Instance_Segmentation_Model/  DINOv2(ViT-S/14) 정의 + 체크포인트
│   └── Pose_Estimation_Model/        PEM 코드 + 체크포인트(1.6 GB)
├── template/<객체>/templates/        렌더 42장 (rgb/mask/xyz)
├── outputs/yolo_ism_object_n/template_features/   ISM 템플릿 특징 + HSV 기준값
└── assets/
    ├── pem_templates/<객체>.pt       PEM 템플릿 특징 (미리 구움)
    ├── model_points/<객체>.npy       CAD 에서 뽑은 8192점
    └── clip/ViT-B-32.pt              YOLO-World 의 텍스트 인코더 (5절 참고)
```

루트에 ISM 파일들이 그대로 놓인 이유: 이 파일들은 **자기 위치 기준으로** config·체크포인트·
캐시 경로를 계산한다. 원본 배치를 그대로 흉내 내면 **한 줄도 고치지 않고** 쓸 수 있고,
그래야 오프라인 배치와 판정이 어긋날 일이 없다. 원본이 바뀌면 그 4개 파일만 다시 복사하면 된다.

## 3. 뺀 것과 이유

| 뺀 것 | 크기 | 이유 |
|---|---|---|
| **CAD `.ply` 전부** | 919 MB | PEM 이 CAD 에서 쓰는 건 8192 점뿐이다 → `assets/model_points/<객체>.npy`(1 MB)로 대체 |
| YOLO 사이드카 (`yoloworld_sidecar.py`) | — | 새 env 에 YOLO-World 가 직접 들어가서 별도 프로세스·유닉스 소켓이 불필요 |
| 오프라인 배치 도구 (`build_pem_inputs.py`, `run_pem_batch.py`, `viz_*`) | — | 실시간 경로에서 안 쓴다. 오프라인은 계속 `sam6d_ws` 에서 돌린다 |
| 연구·검증 산출물 (`_ism_research_*`, `_bmad-output`, `outputs/pem_inputs/*`) | 수십 GB | 실행에 관여하지 않는다 |
| 비활성 객체 5종(`*_low`)의 템플릿 | 350 MB | config 에서 `enabled: false` |
| 안 쓰는 가중치(`sam_b.pt`, `yoloe-*.pt`, `yolov8s-worldv2.pt`) | 429 MB | 실행 경로가 부르지 않는다 |
| PEM 학습 코드(`train.py`, `test_bop.py`, `provider/training_dataset.py` 계열) | — | 추론만 한다 |

전체 크기 **약 2.7 GB** (이 중 1.6 GB 가 PEM 체크포인트).

---

## 4. 노트북 이식 절차

```bash
# (1) 노트북에 env 만들기 — _env_specs/sam6d.requested_cmds.txt 를 그대로 따라간다
#     ROS 2 Jazzy + torch 2.7.1(cu129) + ultralytics 8.4 + pointnet2 재빌드

# (2) 이 폴더를 통째로 옮긴다  ★ 반드시 mtime 을 보존할 것 ★
tar -czf sam6d_realtime.tar.gz sam6d_realtime/       # tar 는 mtime 을 보존한다
#   또는
rsync -a sam6d_realtime/ user@laptop:/path/sam6d_realtime/
#   ✗ 하면 안 되는 것: cp -r (mtime 이 바뀐다 → 5절 함정 참고)

# (3) pointnet2 CUDA 확장은 노트북에서 다시 빌드해야 한다 (env 사양서 5번 항목)
conda activate sam6d
cd <원본 sam6d_ws>/sam6d_master/SAM-6D/Pose_Estimation_Model/model/pointnet2
export TORCH_CUDA_ARCH_LIST="8.6;8.9;9.0;12.0+PTX" CUDA_HOME=$CONDA_PREFIX
pip install --no-build-isolation .
#   ※ 이 폴더에도 같은 소스가 sam6d_master/.../model/pointnet2 에 들어 있으니 거기서 빌드해도 된다

# (4) 확인
conda activate sam6d && export ROS_DOMAIN_ID=72
cd /path/sam6d_realtime
python realtime/sam6d_realtime_node.py --config realtime/run_live_example.yaml
#   → 기동 로그에 [hsv] ... ACTIVE 가 10줄, [ism] 줄에 True 세 개가 보이면 정상
```

## 4b. 인터넷 없이 돌리기 (검증 완료)

**설치할 때만 인터넷이 필요하고, 실행할 때는 필요 없다.** 2026-08-13 실측 확인:
모든 HTTP(S) 경로를 죽은 주소로 막고 전체 실행 → 다운로드 시도 0건, 적재 6.0 초,
199 프레임 처리(2.53 Hz)·검출 115건으로 정상 완주했다.

실행 시 인터넷을 부르는 것은 원래 딱 두 개였고 둘 다 막아 두었다.

| 원래 받아오던 것 | 지금 |
|---|---|
| CLIP 텍스트 인코더 `ViT-B-32.pt` (354 MB) | `assets/clip/` 에 포함 + 경로 고정 |
| `clip`·`ftfy`·`regex` 파이썬 패키지 (ultralytics 가 첫 실행 때 자동 설치) | 환경 구축 4번 단계에서 미리 설치 |

즉 **설치 단계에서 `pip install clip ftfy regex` 를 빠뜨리면** 첫 실행에서 인터넷을 찾다가 멈춘다.
정상이면 실행 후 출력 폴더에 `weights/clip/` 이 생기지 않는다(생겼다면 또 받아온 것).

## 5. ⚠ 반드시 알아야 할 함정

* **HSV 캐시는 템플릿 파일의 크기·mtime 지문을 검사한다.** 지문이 안 맞으면 캐시가 무효가 되고,
  원래 코드는 그때 **조용히 통과**시킨다(fail-open = 색 게이트가 꺼진 것과 같다).
  그래서 이 노드는 그 상황에서 **시끄럽게 죽도록** 만들어 두었다.
  그런 메시지를 보면 복구는 한 줄이다:
  ```bash
  python tools/build_hsv_template_cache.py --force
  ```
* **CLIP 텍스트 인코더(354 MB)는 원래 첫 실행 때 인터넷에서 내려받는다.** YOLO-World 의
  `set_classes()` 가 `<현재 폴더>/weights/clip/ViT-B-32.pt` 를 받으러 간다 — 오프라인
  노트북이면 거기서 멈춘다. 그래서 사본을 `assets/clip/` 에 넣고 노드가 그쪽을 보도록
  고정해 두었다. 실행 후 출력 폴더에 `weights/clip/` 이 새로 생겼다면 그 고정이 풀린
  것이니(=인터넷에서 또 받은 것) 확인할 것.
* **`conda activate sam6d` 를 하고 실행할 것.** env 의 python 바이너리를 직접 부르면
  conda-forge torch 가 헤더/라이브러리 경로를 base 환경 기준으로 잡는다(빌드 시 특히 치명적).
* `torchvision==0.24 is incompatible with torch==2.7` 경고는 ultralytics 의 자체 호환표 때문이며
  **무시해도 된다** — conda-forge 가 짝지어 배포하는 조합이고, PEM 포즈가 기존 환경과
  소수점 6자리까지 같음을 확인했다.
* ROS_DOMAIN_ID 는 **72** 를 쓴다(ORB-SLAM3 실시간이 71을 쓴다).
* 실제 카메라는 `align_depth.enable:=true` 와 `global_time_enabled:=true` 가 필수다.

---

## 5b. ★ 권장 실행 구조 — 수신 / 추론 프로세스 분리

고속 영상 구독(30 Hz × RGB·depth)과 무거운 추론을 **같은 파이썬 인터프리터**에 두면
GIL 때문에 추론 스레드가 굶어 **초 단위 정체**가 생긴다. 원인 규명은
`outputs/test/NOTES.md` 19~21절에 있고, 요지는 이렇다.

* 정체 구간에 추론 스레드는 CPU 를 5% 만 쓰고 3 만 번 넘게 잠들었다 깬다
* 같은 프로세스의 다른 스레드는 코어 1.5 개를 쓰고 있다 (자원 부족이 아니다)
* **ROS 를 걷어내고 같은 코드를 300 회 돌리면 ISM 최대 107 ms · PEM 최대 156 ms** 로 정체가 없다
* 재생 속도·타이머 주기·torch 스레드·GC 등 부하 노브는 전부 효과가 없었다

그래서 프로세스를 둘로 나눈다.

```
realtime/sam6d_receiver_node.py   ROS 전용. 구독 → /dev/shm 에 최신 프레임만 덮어쓰기 → 결과 발행
realtime/sam6d_infer.py           ROS 없음. /dev/shm 에서 최신 프레임을 집어 ISM+PEM
realtime/sam6d_core.py            추론 알맹이 (노드와 같은 함수·같은 순서)
realtime/shm_channel.py           락 없는 seqlock 공유메모리 통로 (밀리면 덮어쓴다)
```

```bash
conda activate sam6d && export ROS_DOMAIN_ID=72 && cd /path/sam6d_realtime
ros2 launch realtime/launch/sam6d_split.launch.py config:=realtime/run_split_example.yaml
```

실측 (0807/185223, 같은 bag·같은 기계):

| | 단일 프로세스 | **분리** |
|---|---|---|
| 처리 프레임 | 704 (2.32 Hz) | **1,817 (6.16 Hz)** |
| 1 초 넘는 프레임 | 59 개 (8.4%) | **0 개 (0.0%)** |
| 프레임당 중앙 / p99 / 최대 | 162 / 3,490 / **9,596** ms | 47 / 282 / **430** ms |
| 촬영→결과 지연 중앙 / 최대 | 0.20 / **9.64** s | 0.07 / **0.52** s |

판정은 바뀌지 않는다 — 양쪽이 모두 처리한 프레임에서 **객체 집합 100% 일치**.

## 6. 실행 (단일 프로세스판)

```bash
conda activate sam6d && export ROS_DOMAIN_ID=72 && cd /path/sam6d_realtime

# bag 재생 (노드 기동 → 준비 완료 대기 → 재생 → 끝나면 정상 종료까지 자동)
ros2 launch realtime/launch/sam6d_realtime.launch.py config:=realtime/run_bag_example.yaml

# 실제 카메라 (카메라는 별도 터미널에서 먼저 띄운다)
ros2 launch realtime/launch/sam6d_realtime.launch.py config:=realtime/run_live_example.yaml
```

준비 완료를 고정 시간으로 기다리지 않는다. 모델 적재가 끝나면 노드가 `/sam6d/status` 를
내보내고, 런치는 그 첫 메시지를 보고 나서 bag 을 튼다.

### 터미널을 두 개 쓰고 싶다면

`bag.play: false` 로 두면 런치는 **노드만** 띄운다. 그러면 재생은 사람이 따로 한다.

```bash
# 터미널 1
conda activate sam6d && export ROS_DOMAIN_ID=72
ros2 bag play <bag 경로> --clock --rate 1.0

# 터미널 2
conda activate sam6d && export ROS_DOMAIN_ID=72 && cd /path/sam6d_realtime
ros2 launch realtime/launch/sam6d_realtime.launch.py config:=<bag.play=false 인 yaml>
```

동작은 같지만 **앞부분이 잘린다**: 노드는 모델을 올리는 데 약 6 초가 걸리므로, bag 을 먼저
틀면 그 6 초치 프레임은 아무도 안 받는다. 한 터미널짜리(`bag.play: true`) 쪽은 준비 완료를
기다렸다 재생을 시작하므로 그 손실이 없다. 처음부터 다 보고 싶으면 한 터미널 방식을 쓸 것.

## 7. 나오는 것

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/sam6d/detections` | `vision_msgs/Detection3DArray` | `header.stamp` = **촬영 시각**, `results[0].hypothesis.class_id` = 객체 이름, `.score` = PEM 점수, `.pose` = 카메라 기준 `T_cam_obj` [m] |
| `/sam6d/status` | `std_msgs/String` (JSON) | 처리 Hz, 건너뛴 장수, 마지막 프레임의 단계별 소요 ms |

포즈를 **map 좌표로 바꾸지 않고 카메라 기준 그대로** 내보낸다. 지도 좌표 변환은
"그 프레임을 찍던 순간의 SLAM 포즈"를 아는 쪽(object memory)이 해야 옳다.

출력 폴더(`output.dir`)에는 `detections.jsonl`(검출 한 건 = 한 줄), `frames.jsonl`(처리한
프레임 한 장 = 한 줄), `run_meta.json`(설정·적용된 게이트·최종 요약)이 쌓인다.

## 7b. 터미널 2 개로 돌리는 법 (이식 후 바로 쓰는 절차)

**공통** — 두 터미널 모두 먼저:

```bash
conda activate sam6d
export ROS_DOMAIN_ID=72
cd /path/sam6d_realtime
```

### (1) bag 재생 + SAM-6D

```bash
# 터미널 A  — SAM-6D (수신 + 추론 두 프로세스를 한 번에 띄운다). 먼저 켠다.
ros2 launch realtime/launch/sam6d_split.launch.py config:=realtime/run_bagplay_split.yaml
#   → "[infer] 준비 완료" 가 뜰 때까지 기다린다 (모델 적재 30~60 초)
#   → 그전에 "아직 영상이 한 장도 안 왔다" 경고가 한 번 뜨는데 정상이다

# 터미널 B  — 재생
ros2 bag play <bag 경로> --clock --rate 1.0
```

재생이 끝나면 추론 프로세스가 30 초 뒤 스스로 종료하고 `output/live_bag/` 에 결과가 남는다.
(`frames.jsonl` · `detections.jsonl` · `receiver.jsonl` · `masks/` · `run_meta.json`)

영상(3 분할 mp4)을 만들려면 — **bag 재생판에서만 가능하다**:

```bash
conda activate sam_yolo        # rosbags 가 있는 env
python tools/render_result_video.py --bag <bag 경로> --run output/live_bag \
       --out out.mp4 --fps 15 --step 2
```

### (1b) 시각화 터미널 추가 (선택)

세 번째 터미널에서 뷰어를 띄우면 **mp4 와 똑같은 3 분할 화면을 실시간으로** 본다.
ROS 를 쓰지 않고 **수신·추론이 쓰는 공유메모리를 그대로 읽는다** — 그래서

* ROS 토픽을 하나도 늘리지 않는다(30 Hz 영상을 또 구독하지 않는다)
* 별개 프로세스라 GIL 을 다투지 않는다
* **bag 이든 카메라든 똑같이 동작한다** (카메라에서도 녹화가 된다)

```bash
# 터미널 C — 아무 때나(추론이 뜬 뒤) 붙였다 떼도 된다
python realtime/sam6d_viewer.py                       # 창으로 보기 (q 로 종료)
python realtime/sam6d_viewer.py --save live.mp4       # 보면서 녹화
python realtime/sam6d_viewer.py --headless --save live.mp4   # 화면 없이 녹화만 (SSH)
python realtime/sam6d_viewer.py --scale 0.6           # 창이 크면 축소
```

화면 구성은 `LIVE | CAD 점군 | 3D 경계상자` + 아래 Δrot 이력 띠이고,
헤더에 입력 Hz · 처리한 프레임 수 · 단계별 ms · 결과가 화면에 머문 시간이 나온다.

**비용 실측**: 뷰어를 켠 채로 수신 **29.9 Hz** · 추론 **9.4 Hz** · 프레임 중앙 28 ms ·
1 초 초과 0 개 — 끄고 돌렸을 때(26.6 / 9.2 Hz / 28 ms)와 차이가 없다.

> 다른 방법도 있지만 이유가 있어 이쪽을 택했다.
> **RViz2** 는 `vision_msgs/Detection3DArray` 를 그리지 못해 `MarkerArray` 발행기를 따로
> 붙여야 하고, 영상까지 보려면 30 Hz 이미지를 **한 번 더 구독**해야 한다(전송 부하 증가).
> 지도 좌표계에서 여러 물체를 3D 로 보고 싶을 때만 쓰는 것이 낫다.
> **rqt_image_view** 도 오버레이 이미지를 발행하는 노드가 추가로 필요하다.

### (2) RealSense D455f 직결 + SAM-6D

```bash
# 터미널 A  — 카메라 드라이버. 먼저 켠다.
ros2 launch realsense2_camera rs_launch.py \
     align_depth.enable:=true enable_sync:=true \
     rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30

# 터미널 B  — SAM-6D
ros2 launch realtime/launch/sam6d_split.launch.py config:=realtime/run_live_split.yaml
```

끝낼 때는 터미널 B 에서 Ctrl-C (카메라판은 스스로 종료하지 않는다).
결과는 `output/live_cam/` 에 쌓인다.
시각화가 필요하면 **터미널 C 에서 (1b) 의 뷰어를 그대로 띄우면 된다** — 카메라판에서는
`tools/render_result_video.py` 를 쓸 수 없으므로(영상이 저장되지 않는다),
영상 기록이 필요하면 `--save` 로 뷰어가 녹화하게 한다.

**카메라판에서 먼저 확인할 것**

```bash
ros2 topic hz /camera/camera/color/image_raw          # 30 Hz 나오는지
ros2 topic list | grep aligned_depth                  # 정렬 depth 가 있는지
ros2 topic echo --once /sam6d/status std_msgs/msg/String   # 수신·처리 수
```

* 토픽 이름이 다르면 `run_live_split.yaml` 의 `topics:` 를 실제 이름으로 고친다
  (드라이버 버전에 따라 네임스페이스가 `/camera/...` 로 하나만 붙기도 한다).
* `align_depth.enable:=true` 가 없으면 depth 가 컬러에 정렬되지 않아 포즈가 전부 틀린다.
* 프로파일 인자 이름은 드라이버 버전마다 다르다(`rgb_camera.profile` / `rgb_camera.color_profile`).
  `ros2 param list /camera/camera` 로 확인할 것.
* **영상이 0 장이면 QoS 불일치다** — `run_live_split.yaml` 의 `qos_reliability` 를
  `best_effort` 로 바꾼다(수신 노드가 경고를 찍는다).

## 8. 검증 결과 (2026-08-13, 워크스테이션 RTX PRO 6000)

**(1) 판정이 오프라인 정본과 같은가 — 같다.**
0807 세션에서 147 프레임마다 뽑은 **61 프레임**을 두 경로에 똑같이 넣고 비교했다.

| | 검출 수 | 프레임별 객체 집합 |
|---|---|---|
| 오프라인 정본 `sam6d_ws/tools/build_pem_inputs.py` (env `sam_yolo`, torch 2.12) | 95 | — |
| 이 폴더 (env `sam6d`, torch 2.7.1) | 95 | **61/61 완전 일치, 불일치 0** |

박스 좌표까지 95건 중 94건이 정확히 같고, 1건만 y 가 1 px 다르다(환경 간 반올림).
즉 **개선 4종이 실시간 경로에 실제로 들어왔고, 코드를 옮기면서 어긋난 곳이 없다.**

**(2) 처리 속도 — 0807 세션 295초 전체 재생, 785 프레임 처리(평균 2.59 Hz), 오류 0**

| 화면 안 객체 수 | 프레임 수 | 프레임당 시간(중앙) |
|---|---|---|
| 0개 | 591 | **27 ms** |
| 1개 | 76 | 414 ms |
| 2개 | 45 | 737 ms |
| 3개 | 35 | 800 ms |
| 4개 | 25 | 1.02 s |
| 5개 | 12 | 1.14 s |

단계별(중앙/p90/최대): YOLO 9 / 125 / 1549 ms · ISM 21 / 195 / 1433 ms · PEM 0 / 621 / 4018 ms.
검출 437건(Rabbit 120, choco 74, Bear 57, Sikhye 49, saffron 40, milk 39, Dinosaur 32,
Febreze 15, Sauce 9, Mugcup 2), PEM 점수 중앙 0.680.

**비용은 사실상 PEM 이고 객체 1개당 0.45~0.5 초씩 늘어난다**(YOLO·ISM 은 중앙값 각 5~6 ms).
객체가 없는 프레임은 26 ms 로 끝난다.

ISM 은 평소 20 ms 안팎이지만 **YOLO 박스가 많은 프레임에서 튄다**(실측 박스 14개 → 1.2 s,
33개 → 2.6 s). 상대 할당이 "박스마다 전 객체와 비교"하는 구조라 박스 수에 비례하기 때문이다.
`frames.jsonl` 의 `n_boxes` 로 확인할 수 있다.

첫 프레임은 CUDA 커널 컴파일 때문에 원래 8 초 넘게 걸렸는데(YOLO 5.3 s + ISM 4.4 s),
기동 시 가짜 프레임 한 장을 태우는 예열(0.8 s)을 넣어 정상 범위로 내렸다.

**(2b) 오프라인 배치 결과와의 대조 — 260804 office longcircle2 (bag 72.2 s)**

| | 처리 프레임 | 검출 | 검출 있는 '초' | 프레임당 검출 |
|---|---|---|---|---|
| 오프라인 `--stride 10` | 199 (시간 균일) | 342 | 47/72 | 1.72 |
| 실시간 | 173 (2.2 Hz) | 117 | 34/72 | 0.68 |

**판정 품질은 같다**: 같은 프레임 40장에서 ISM 결정이 **40/40 일치**(유일한 차이 1건은
작은 객체에서 PEM 이 실패한 것으로, 오프라인도 342건 중 8건 같은 실패를 낸다).

**차이는 어디서 오나 — 알고리즘이 아니라 시간 예산의 편향이다.** 객체가 있는 프레임은
평균 918 ms, 빈 프레임은 168 ms 로 **5.5 배** 비싸다. 실시간은 정해진 시간 안에서만
처리하므로 객체가 많은 구간일수록 프레임을 적게 집는다(검출 있는 49장에 45 초, 빈 124장에
21 초를 썼다). 그래서 "프레임당 검출률"은 낮게 보이지만, 이는 표본 편향이지 인식 성능 차이가
아니다. 실제로 잃는 것은 **시간 커버리지 47초 → 34초**다.
줄이려면 PEM 을 객체마다 따로 돌리는 대신 프레임 단위로 묶어야 한다(가장 큰 개선 여지).

**(3) 실시간성 — 뒤처짐이 누적되지 않는다.**
처리한 프레임과 그 순간 들어와 있던 최신 프레임의 시각 차이는 **중앙 0.03 s / p90 0.80 s /
최대 4.04 s** 로, 295초 내내 **커지지 않는다**(커지면 큐가 밀린다는 뜻이다).
들어온 4978장 중 785장을 처리하고 4193장을 건너뛰었다 — 설계대로다.

노트북은 이보다 2~3배 느릴 수 있다. 줄이려면 순서대로:
`ism.objects` 로 그 장면에 없는 물체 제외 → `overlay_every: 0` → 재생 속도 낮추기.

## 8b. 다른 기계로 옮겼을 때 — 성능 비교와 판정 검증

옮긴 기계(특히 노트북)에서 **느려지는 것은 정상**이고, **판정은 같아야 한다.** 둘을 따로
확인할 수 있게 고정 프레임 8 장과 벤치마크를 같이 넣어 두었다. ROS 를 쓰지 않으므로
미들웨어·GIL 영향이 빠지고 **순수 처리 성능**만 나온다.

```bash
conda activate sam6d && cd /path/sam6d_realtime
python tools/bench_offline.py --frames bench_frames --n 200 --out bench_laptop.json
python tools/bench_compare.py bench_workstation.json bench_laptop.json
```

`bench_compare.py` 는 단계별 시간 배율, 처리율, **GPU 클럭·전력·온도의 시작→끝**(발열로 인한
성능 저하가 보인다), 그리고 **프레임별 판정 일치율**을 같이 찍는다.

**기준선 (이 저장소에 포함, `bench_workstation.json`)**
RTX PRO 6000 Blackwell Max-Q (300 W 한도) · Ryzen Threadripper 9960X · torch 2.7.1/cu129

| 단계 | 중앙 | p90 | 최대 |
|---|---|---|---|
| YOLO | 5 ms | 6 | 6 |
| ISM | 30 ms | 45 | 100 |
| PEM | 35 ms | 64 | 66 |
| **전체** | **81 ms** | 102 | 155 |

객체 수별 프레임 시간(중앙/최대): 0 개 23/30 · 1 개 74/94 · 2 개 86/155 · 3 개 103/112 ms.
쉬지 않고 돌렸을 때 **14.0 Hz**. 실행 중 GPU 는 **214 W** 를 끌어 쓰고 클럭은
2,272 → 2,212 MHz (최대 3,090), 온도 53 → 63 °C 였다.

**노트북에서 예상되는 것**

* **속도 저하는 정상이다.** 이 워크로드는 GPU 를 214 W 쓰는 무거운 작업이고, 노트북 GPU 는
  보통 전력 한도가 절반 수준이며 SM 수도 적다. **1.5~2.5 배 느려지면 정상 범위**로 본다.
* **발열로 인한 추가 저하도 정상이다.** 워크스테이션조차 10 분 만에 클럭이 조금 내려갔다.
  `bench_compare.py` 의 클럭·온도 시작→끝을 보면 얼마나 떨어지는지 바로 보인다.
* **판정(어떤 객체를 검출하는가)은 같아야 한다.** 다르면 하드웨어가 아니라 자산·설정 문제다
  (HSV 캐시 stale, 템플릿 mtime, config 차이). 일치율이 100% 가 아니면 5 절 함정부터 확인할 것.
* **PEM 의 포즈 값은 같은 기계에서도 실행마다 다르다.** 초기 포즈를 무작위 가설 추출로
  만들기 때문이다(`model_utils.py:221`, 씨앗 없음). 하드웨어 탓이 아니다.
* ⚠ **pointnet2 CUDA 확장은 반드시 그 기계에서 다시 빌드**해야 한다(4 절 3 단계).
  `TORCH_CUDA_ARCH_LIST` 의 `12.0+PTX` 가 RTX 50 시리즈(sm_120)를 덮는다.

**⚠ 영상 토픽에 BEST_EFFORT 를 쓰지 말 것.** 900 KB 짜리 Image 는 UDP 로 쪼개져 가는데
best-effort 독자는 조각 하나만 잃어도 샘플을 통째로 버린다. 실측(NOTES 24 절): 같은 30 Hz
입력에서 **RELIABLE 30.0 Hz vs BEST_EFFORT 3.7 Hz**, 큐를 30 으로 키워도 그대로였다.
그래서 수신 노드 기본값은 `qos_reliability: reliable`, `qos_depth: 30` 이다. '최신 것만 처리'는
QoS 가 아니라 **공유메모리 덮어쓰기**로 한다.
카메라가 best_effort 로만 발행하면 reliable 구독은 데이터를 못 받으므로 그때만 바꾼다
(10 초간 무입력이면 경고가 뜬다).

이 수정 뒤 수신 9.9 → **26.6 Hz**(입력의 89%), 추론 7.4 → **9.2 Hz** 가 되었다.
그래도 **병목은 여전히 GPU 가 아니다** — 추론은 쉬지 않으면 14~23 Hz 를 낼 수 있다.
카메라를 직접 붙이면 `receiver.jsonl` 의 `deliver_ms`·기록 Hz 를 꼭 다시 잴 것.

## 9. 원본과 동기화

`sam6d_ws` 의 ISM 이 개선되면 **아래 5개만 다시 복사**하면 된다(수정 없이 그대로 씀).

```bash
SRC=<repo>/sam6d_ws
rsync -a $SRC/{yolo_ism.py,yolo_ism_object_n.py,ism_hsv.py,ism_training_free_gate.py} .
rsync -a $SRC/configs/yolo_ism_objects.yaml configs/
```

템플릿 특징이나 HSV 기준값의 의미가 바뀌는 개선(블록 변경, hue 보정 등)이면
`outputs/yolo_ism_object_n/template_features/` 도 함께 복사해야 한다.

## 10. 재빌드가 필요할 때 (여기 없는 원본 자산)

| 필요한 작업 | 원본 위치 |
|---|---|
| PEM 템플릿 특징 재생성 | `sam6d_ws/template/<객체>/templates` (여기 포함돼 있음) |
| CAD 포인트 재추출 / CAD 자체가 필요 | `sam6d_ws/data/cad/<객체>/*.ply` (**여기 없음**) |
| 비활성 객체(`*_low`) 사용 | `sam6d_ws/template/<객체>/templates` (**여기 없음**) |
