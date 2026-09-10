---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments: []
workflowType: 'research'
lastStep: 1
research_type: 'technical'
research_topic: 'sam6d_ws YOLO-World→semantic/appearance score→template selection pipeline 시간 성능 분석 및 계측 설계'
research_goals: '실제 코드 기반 pipeline 재구성, 병목 후보 Top5 도출, timing instrumentation 설계 (per_frame/per_object/stage_summary schema), 정확도 개선 아님'
user_name: 'ldh'
date: '2026-07-06'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-07-06
**Author:** ldh
**Research Type:** technical

---

## Research Overview

본 조사는 sam6d_ws의 `입력 이미지 → YOLO-World 객체 10개 인식 → semantic/appearance score → template 선택(PEM 진입 직전)` pipeline의 시간 성능 병목을 파악하기 위한 기술 조사이다. 정확도 개선이 아니라 **실제 코드 기반 시간 성능 분석 설계**가 목적이며, 코드 수정·fake 데이터·임의 benchmark 수치 생성은 하지 않았다. 실제 파일에서 확인한 내용은 "확인됨", 코드 판독 기반 판단은 "추정"으로 구분했다.

조사 방법: (1) 병렬 코드베이스 탐색 에이전트로 프로젝트 구조·call path를 파일:라인 단위로 추적하고 주요 주장 전부를 본 조사에서 직접 열람 검증, (2) 웹 공식 소스(Ultralytics/PyTorch/ROS2/SAM-6D 논문)로 스택 동작 특성을 교차 검증, (3) 병목 후보 Top 5와 최소 침습 계측 설계(3종 CSV 스키마 + Validation Plan V1~V8)를 다음 dev 단계에서 그대로 구현 가능한 수준으로 산출했다.

핵심 결론은 문서 하단 **Executive Summary** 및 **Bottleneck Candidate Top 5 / Instrumentation Design** 절 참조.

---

<!-- Content will be appended sequentially through research workflow steps -->

## Technical Research Scope Confirmation

**Research Topic:** sam6d_ws의 `입력 이미지 → YOLO-World 객체 10개 인식 → semantic/appearance score → template 선택 (PEM 진입 직전까지)` pipeline 시간 성능 분석 및 계측 설계

**Research Goals:** 실제 코드 기반 pipeline 재구성, 병목 후보 Top 5 도출, timing instrumentation 설계 (per_frame/per_object/stage_summary schema). 정확도 개선 아님.

**Technical Research Scope:**

- Project Structure Map — sam6d_ws 실행 관련 파일/config/template/output 위치 정리
- Actual Pipeline Reconstruction — entrypoint부터 template 선택까지 실제 call path (파일명/함수명 기준)
- Object-10 / Template Selection 처리 분석 — batch vs per-object 반복, cache 여부, 중복 연산, scaling 특성
- Bottleneck Candidate Top 5 — 코드 근거 기반, "확인됨" vs "추정" 구분, 성능 수치 날조 금지
- Instrumentation Design + Timing Schema — per_frame_timing.csv / per_object_timing.csv / stage_summary.csv, GPU sync/warm-up, 최소 침습 삽입 위치
- Validation Plan + Next BMAD Skill 추천

**제외 범위:** 코드 수정, 새 기능 구현, fake 데이터/benchmark 생성, object_memory/Unity dedup, SLAM pose association, 정확도 개선, PEM 내부 최적화 (단, PEM 연결 interface는 문서화)

**Research Methodology:**

- 실제 파일/코드에서 확인한 내용만 "확인됨", 코드 판독 기반 판단은 "추정" 표기
- 웹 검색은 GPU/ROS2 timing 측정 방법론(torch.cuda.synchronize, warm-up, callback 지연 지표)의 현행 모범 사례 검증에 사용
- 다음 개발자가 그대로 instrumentation 구현에 착수 가능한 구체성 확보

**Scope Confirmed:** 2026-07-06

---

## Technology Stack Analysis

> 이 조사는 코드베이스 중심이므로, 본 절은 일반 기술 트렌드가 아니라 **sam6d_ws에서 실제 사용 중인 스택(코드에서 확인됨)** 과 **웹 공식 소스로 검증한 해당 스택의 동작 특성**을 기록한다.

### 핵심 발견: 두 개의 ISM 구현 공존 (확인됨)

- **(A) SAM-6D 원본 ISM**: `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py` (`compute_semantic_score`/`compute_appearance_score`/`compute_geometric_score`, fusion `detector.py:392`). — **참조 구현이며, 이번 분석 대상 entrypoint에서는 호출되지 않음.**
- **(B) 실제 실행 경로**: `yolo_ism.py`의 자체 재구현. `yolo_ism_object_n.py`, `src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py`, `tools/build_ism_inputs_imu.py`가 모두 `import yolo_ism as yi`로 (B)를 사용. YOLO-World 박스가 proposal, MobileSAM이 mask 생성.
- 시간 성능 계측 대상은 **(B) 경로**이다.

### 실행 환경 / 언어 (확인됨)

| 항목 | 내용 | 근거 |
|---|---|---|
| 언어 | Python (SAM-6D 원본 env는 3.9.6; pyc 캐시상 3.11~3.13 실행 흔적 혼재) | `sam6d_master/SAM-6D/environment.yaml:1,10` |
| conda env 분리 | `sam_yolo`(YOLO-World ISM, ultralytics 8.4대) / `sam6d_ros_humble`(PEM·렌더·ROS 노드) / `sam6d_test`(SAM-6D 배치) | `tools/run_e2e.sh` 주석: ultralytics 8.4(YOLOWorld) vs 8.0(FastSAM) 충돌로 단일 프로세스 공유 불가 |
| 미들웨어 | ROS2 Humble (rclpy, cv_bridge, message_filters) | `sam6d_multiobject_node.py:26-31` |
| Device | `cuda:0` 기본, 미가용 시 cpu 폴백 | `yolo_ism_object_n.py:216,224-225`, `configs/yolo_ism_objects.yaml:11` |

### 핵심 라이브러리 / 모델 (import 문 기준 확인됨)

| 컴포넌트 | 라이브러리/모델 | weight | 코드 위치 |
|---|---|---|---|
| 객체 검출 | ultralytics YOLO-World (`yolov8m-worldv2.pt` 57MB, `yolov8s-worldv2.pt` 25MB) | 루트 `*.pt` | `yolo_ism_object_n.py:247-254`, sidecar `tools/yoloworld_sidecar.py` |
| Mask 생성 | ultralytics `SAM` → MobileSAM (`mobile_sam.pt` 40MB) | 루트 | `yolo_ism.py:246-255` (`build_segmentor`/`segment_box`) |
| Feature 추출 | SAM-6D vendored DINOv2 **ViT-S/14** (dim 384) | `.../Instance_Segmentation_Model/checkpoints/dinov2/dinov2_vits14_pretrain.pth` | `yolo_ism.py:62-67` (`build_dinov2`), `model/dinov2.py:15` |
| Pose (후속) | SAM-6D PEM (`sam-6d-pem-base.pth`) — 이번 범위 밖, interface만 문서화 | `Pose_Estimation_Model/checkpoints/` | `sam6d_multiobject_node.py:166-190` |
| 수치/영상 | torch, numpy, opencv, PIL, scipy | — | `yolo_ism.py:36-40` |

참고: SAM-6D 원본 경로용 FastSAM(`FastSAM-x.pt`/`FastSAM-s.pt`), `sam_b.pt`, CLIP `ViT-B-32.pt`도 존재하나 (B) 실행 경로에서는 미사용 (확인됨).

### Config / Template / 데이터 위치 (확인됨)

- `configs/yolo_ism_objects.yaml` — (B) 경로의 중심 config. `defaults`: `top_k: 3`, `score_threshold: 0.02`, `imgsz: 960`, `similarity_threshold: 0.35`(semantic 1차 게이트), `appe_gate: 0.55`(2차 게이트), `match_topk: 5`, `use_mask: true`, `stride: 10`. `objects[]`: enabled 10객체 (milk, choco_hazelnut_high, Febreze_high, Mugcup_high, saffron, Sauce_high, Sikhye_high, Bear, Rabbit, Dinosaur) + per-object threshold 오버라이드.
- Template: `template/<obj>/templates/` — 객체당 **42 view** × (`rgb_N.png`, `mask_N.png`, `xyz_N.npy`). 렌더는 오프라인 사전 수행 (`tools/render_all_templates.sh`).
- Template feature 디스크 캐시: `outputs/yolo_ism_object_n/template_features/<name>_cls.pt`(예: Bear 67KB) + `<name>_appe.pt`(예: Bear 9MB) — startup 1회 로드 (`yolo_ism.py:138-161, 164-208`).
- CAD: `data/cad/<obj>/*.ply`; 입력 bag: `data/ros2_bag/`; ROS 노드 파라미터: `src/sam6d_ros/config/params.yaml`, `no_cli_*.yaml`.

### 스택 동작 특성 웹 검증 (공식 소스 확인됨)

1. **YOLO-World `set_classes()`**: prompt-then-detect + offline vocabulary. text embedding은 `set_classes()` 시 1회 CLIP 인코딩되어 `txt_feats`로 캐시 — 매 추론 재계산 없음. 클래스 수 증가 시 추론 비용의 정량 스케일링은 공식 문서에 없음(미확인).
   _Source: https://docs.ultralytics.com/models/yolo-world/_
2. **SAM-6D ISM 논문 구조** (arXiv:2311.15707, CVPR 2024): 템플릿 42개(CNOS 계승), DINOv2 cls/patch embedding, s_sem = proposal×template cosine의 top-K 평균, s_appe = best template 대상 patch-wise max cosine 평균, s_geo = rough-pose 투영 bbox IoU × visibility, 최종 s_m=(s_sem+s_appe+r_vis·s_geo)/(2+r_vis). 논문은 ViT-L이나 **본 repo (B) 경로는 ViT-S/14 사용, geometric+fusion 없이 순차 2-게이트로 단순화** (확인됨).
   _Source: https://arxiv.org/html/2311.15707v2_
3. **PyTorch GPU timing**: CUDA 커널은 비동기 — 동기화 없이는 launch 시간만 측정됨. `torch.cuda.Event(enable_timing=True)` + `torch.cuda.synchronize()` + `elapsed_time()`이 표준. warm-up 필수(첫 호출 cuBLAS 로딩 예시 2775.5μs vs 2번째 22.4μs).
   _Source: https://docs.pytorch.org/tutorials/recipes/recipes/benchmark.html_
4. **ROS2 지연/드롭 지표**: latency = 수신 시각 − `msg.header.stamp` (`ros2 topic delay`와 동일 원리). QoS keep_last/depth 초과 시 오래된 메시지 드롭 → depth와 drop률 직결. executor 지연의 공식 단일 지표는 부재(미확인) — 계층별 timestamp 프로파일링이 관행.
   _Source: https://design.ros2.org/articles/qos.html, https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html_
5. **FastSAM**: YOLOv8-seg 기반 all-instance segmentation 1회 forward + prompt는 후처리 선택(box는 IoU 매칭). — 단 (B) 경로의 mask는 FastSAM이 아닌 **MobileSAM bbox-prompt** (`yolo_ism.py:252`)임에 유의.
   _Source: https://arxiv.org/pdf/2306.12156, https://docs.ultralytics.com/models/fast-sam/_

_Section written: 2026-07-06_

---

## Integration Patterns Analysis

> 본 절은 pipeline 컴포넌트 간 **실제 연결 구조(IPC/토픽/디스크 브릿지)** 를 코드로 확인해 기록한다. 모든 항목은 파일:라인 근거가 있는 "확인됨"이며, timing 계측 시 각 interface 경계가 측정 포인트가 된다.

### 1. 컴포넌트 토폴로지 — 왜 소켓/디스크 브릿지가 존재하는가 (확인됨)

ultralytics 버전 충돌(YOLOWorld는 8.4, SAM-6D 원본 FastSAM 경로는 8.0)로 conda env가 분리되어, 실시간 경로는 **2-프로세스 + 2종 IPC** 구조다:

```
[sam_yolo env]                          [sam6d_ros_humble env]
yoloworld_sidecar.py  ←Unix socket→  sam6d_multiobject_node.py  ←ROS2 topics→  bag/카메라
(YOLO-World 상주)                      (DINOv2+MobileSAM+PEM 상주)
                                           ↕ /dev/shm 파일 왕복
                                        PEM get_test_data
```

- sidecar 근거: `tools/yoloworld_sidecar.py:2-15` (docstring), 노드 연결 `sam6d_multiobject_node.py:106-108`
- 오프라인 경로(`yolo_ism_object_n.py`)는 단일 프로세스(`sam_yolo`)라 sidecar 불필요 — YOLO를 직접 로드(`:247-254`)

### 2. ROS2 입력 integration (확인됨)

- 토픽: `rgb_topic=/camera/camera/color/image_raw`, `depth_topic=/camera/camera/aligned_depth_to_color/image_raw`, `caminfo_topic` (`sam6d_multiobject_node.py:50-52`)
- RGB+Depth 동기화: `ApproximateTimeSynchronizer(queue_size=10, slop=0.05)` (`:113-116`). queue_size는 필터별 저장 메시지 셋 수, slop은 동기 허용 시간차(초) — 콜백 처리 속도가 입력 속도보다 느리면 큐 초과분부터 드롭됨. _Source: [message_filters docs](https://docs.ros.org/en/ros2_packages/jazzy/api/message_filters/message_filters.html), [ApproximateTimeSynchronizer tutorial](http://docs.ros.org/en/kilted/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Python.html)_
- CameraInfo는 최초 1회만 K 저장(`_caminfo_cb`, `:147-150`); K 수신 전 프레임은 콜백 초입에서 반환(`:195-196`)
- 출력: `PoseArray` publish `~/poses` depth 10 (`:111, :234`) + overlay PNG 저장(`save_every` 주기, `:236-237, :245-252`)
- **단일 콜백 동기 처리**: `_cb`(`:194-239`)가 sidecar 질의→ISM→PEM→publish→PNG 저장까지 전부 동기 수행. 콜백 소요시간이 프레임 주기를 넘으면 그대로 drop으로 이어지는 구조. 콜백 끝에 이미 총 소요 ms 로그 존재(`:238-239`, `time.time()` 기반 1개 값만).

### 3. YOLO-World sidecar 프로토콜 (확인됨)

- 전송(요청): length-prefix 없이 고정 12B 헤더(`>iii` h,w,c) + **raw BGR 전체 프레임**(h×w×c uint8) (`yoloworld_sidecar.py:10, :74-81`; 노드 측 `:153-157`). 예: 640×480×3 ≈ 0.92MB/frame이 소켓으로 매 프레임 복사됨.
- 응답: 4B 길이 + JSON `[{"cls", "box", "conf"}, ...]` (`:93-94`)
- sidecar 내부: `set_classes(unique_prompts)` startup 1회(`:60`), 요청당 `yolo.predict(bgr, conf=min_score, imgsz=640)` 1회(`:82`)
- **블로킹 동기 왕복**: 노드의 `_yolo_boxes()`는 sendall→recv 완료까지 콜백 스레드 블로킹(`sam6d_multiobject_node.py:153-157`). 이 왕복 시간 = 직렬화 + 소켓 전송 + YOLO 추론 + JSON 응답의 합이며 현재 분리 계측 불가.
- **설정 불일치 발견**: sidecar는 `imgsz=640` 하드코딩(`yoloworld_sidecar.py:82`), 오프라인 `yolo_ism_object_n.py`는 config `imgsz: 960` 사용(`:274`, `configs/yolo_ism_objects.yaml`). 실시간/오프라인 YOLO 결과와 시간이 서로 다른 조건임 — 벤치마크 비교 시 반드시 기록해야 할 변수 (확인됨).

### 4. ISM→PEM interface (확인됨) — 이번 병목 범위의 하류 경계

**실시간 (ROS 노드, `_pem_pose` `sam6d_multiobject_node.py:166-191`):** accepted 객체마다
1. `/dev/shm/sam6d_rt/<obj>/`에 `rgb.png`, `depth.png` cv2.imwrite + `camera.json` (`:169-171`)
2. mask를 `mask_to_rle_pytorch`로 uncompressed RLE 변환 → `detection.json` (BOP 형식, `:172-178`)
3. `ric.get_test_data(...)`가 방금 쓴 파일들을 **다시 읽어** PEM 입력 구성 (`:179-181`) — 즉 detection당 PNG 인코딩/디코딩 왕복이 /dev/shm에서 발생
4. PEM template feature는 startup 1회 사전계산(`self._tem`, `:99-101, :141-145`), CAD 포인트는 `.npy` 캐시(`_patch_cad_cache`, `:120-139`)
5. `self.pem(inp)` 추론(`:184-186`) → R/t → PoseArray

**오프라인 (batch):** `tools/build_ism_inputs_imu.py`가 frame별 번들(rgb/depth/camera.json/detection_<obj>.json RLE)과 `manifest.csv`를 기록(`:244-293`) → `tools/run_pem.sh`가 manifest 순회하며 `run_inference_custom.py`(PEM)를 번들마다 실행(`run_pem.sh:16-28`).

→ **interface 계약 = "디스크(또는 shm)상의 rgb.png + depth.png + camera.json + detection JSON(RLE seg) + CAD/template dir"**. 이번 계측 범위의 종점은 detection JSON이 만들어지기 직전(= ISM accepted + mask 확보)이고, `_pem_pose` 진입 이후는 후속 단계로 분리 측정한다.

### 5. 데이터 흐름 요약 (확인됨)

```
[frame_input]   bag/카메라 → (ROS: ApproxTimeSync RGB+depth | offline: resolve_frames cv2.imread/AnyReader)
[prompts]       configs/yolo_ism_objects.yaml → build_prompt_groups() → unique_prompts (10 enabled 객체)
[detection]     YOLO-World predict 1회/frame (offline 직접 | ROS sidecar 소켓 왕복) → cls별 box 라우팅
[per-object]    recognize(): conf필터+top_k → {crop→DINOv2 cls→semantic score}×proposal → 1차 게이트(0.35)
[mask]          MobileSAM segment_box(best bbox) — appearance 계산의 전제
[appearance]    masked_query_patches + best_t=argmax(tcls@cls) → masked_appe_score → 2차 게이트(0.55)
[template 선택] best_t (semantic argmax 기반 단일 best template) = 초기 pose 후보의 view
[output]        accepted → mask RLE + bbox → detection JSON → (PEM interface)
```

_Section written: 2026-07-06_

---

## Architectural Patterns Analysis — Actual Pipeline Reconstruction

> 본 절은 frame 1개 처리의 **실제 call path(파일:라인 검증 완료)** 와 객체 수/템플릿 수에 대한 **스케일링 구조**를 기록한다. 모든 라인 번호는 본 조사에서 직접 열람하여 확인했다.

### 1. Frame 1개 처리 call path — 오프라인 `yolo_ism_object_n.py` 기준 (확인됨)

**Startup (1회):** `main()`(`:220`) → `load_config`(`:222`) → `yi.build_dinov2`(`:235`, `yolo_ism.py:62-73`) → `prepare_objects`(`:236`, template cls/appe feature 로드·캐시) → `build_prompt_groups`(`:237`) → `yi.build_segmentor`(MobileSAM, `:244`) → `YOLOWorld(weights)` + `set_classes(unique_prompts)`(`:250-254`)

**Per-frame 루프 (`:268-320`):**

| # | stage | 코드 | 연산 위치 |
|---|---|---|---|
| 1 | frame_input | `yi.resolve_frames` — `cv2.imread` 또는 rosbag 디코드 (`yolo_ism.py:345-396`) | CPU+Disk |
| 2 | preprocess | `cv2.cvtColor(BGR2RGB)` (`:272`) | CPU |
| 3 | yolo_world | `yolo.predict(bgr, conf=min_score, imgsz=960)` — **frame당 1회** (`:274`) | GPU |
| 4 | box routing | prompt index별 그룹핑, Python 루프 (`:277-286`) | CPU |
| 5 | per-object recognize | groups 이중 루프 (`:290-293`) → `recognize()` ×최대 10객체 | 아래 상세 |
| 6 | output_packaging | accepted당 overlay jpg `cv2.imwrite`(`:307`) + **매 frame combined jpg `cv2.imwrite`**(`:318`) + CSV row 메모리 축적(`:313-320`, 파일 기록은 종료 시 일괄 `:322-329`) | CPU+Disk |

**`recognize()` 내부 (객체 1개당, `:138-196`):**

| # | sub-stage | 코드 | 연산 위치 |
|---|---|---|---|
| 5a | 후보 필터 | conf threshold + top_k(3) (`:145-146`) | CPU (미미) |
| 5b | **normalize_rgb(전체 이미지)** | `:158` → `yolo_ism.py:100-104` (H×W×3 float32 변환+정규화) | CPU — **객체마다 반복** |
| 5c | proposal 루프 (≤top_k=3) | `crop_resize_pad`(`yolo_ism.py:80-97`, F.interpolate+F.pad, CPU tensor) → `dinov2_forward(want_patch=False)`(`:164`) | CPU 전처리 + **GPU batch=1 forward** ×proposal |
| 5d | semantic score | `tcls[42,384] @ cls[384]` → top-5 평균 (`yolo_ism.py:214-218`) — cls는 `.cpu()`로 내려온 상태(`yolo_ism.py:111`) | CPU matmul (소형) |
| 5e | 1차 게이트 | `best_sem >= similarity_threshold(0.35)` (`:170`) | CPU |
| 5f | **patch token 재-forward** | 통과 시 `dinov2_forward(best crop, want_patch=True)` (`:175`) | **GPU batch=1 — 5c와 동일 crop 중복 forward** |
| 5g | mask 생성 | `yi.segment_box` → ultralytics `SAM(mobile_sam.pt)` bbox-prompt (`:185`, `yolo_ism.py:252-262`) + mask `.cpu().numpy()` | GPU + GPU→CPU 전송 |
| 5h | masked appearance | `masked_query_patches`(AvgPool2d 커버리지 필터, `yolo_ism.py:265-273`) → `best_t=argmax(tcls@cls)`(`:190`) → `masked_appe_score`: `q_fg[≤256,384] @ tappe[best_t].T` (`yolo_ism.py:276-285`) | CPU matmul (소형) |
| 5i | 2차 게이트 | `masked_appe >= appe_gate(0.55)` (`:192`) | CPU |

**ROS2 노드 경로 차이 (확인됨):** 3번이 sidecar 소켓 왕복(`_yolo_boxes`, imgsz=640)으로 대체되고, 6번 대신 accepted당 `_pem_pose`(PEM interface, 범위 밖)가 실행됨. 5번 `recognize()`는 **동일 함수를 공유** (`sam6d_multiobject_node.py:217`).

### 2. Object-10 Processing Analysis (확인됨)

- **YOLO-World: 객체 수 무관 상수 비용.** 10 prompt는 `set_classes` 1회 설정, text embedding은 설정 시 1회 계산·캐시(Ultralytics 공식 문서 확인). frame당 predict 1회.
- **DINOv2: 객체 수 × proposal 수에 선형.** 객체당 최대 top_k(3)회 cls forward + accepted 후보당 1회 patch forward → **frame당 최대 약 10×3+10 = 40회의 batch=1 GPU forward**. batch=1 소형 입력(224×224)의 반복 launch는 CPU측 kernel-launch/framework 오버헤드가 GPU 실행보다 커져 GPU가 유휴화되는 전형적 패턴(웹 확인). _Source: [PyTorch CUDA Graphs blog](https://pytorch.org/blog/accelerating-pytorch-with-cuda-graphs/), [Framework Tax paper](https://arxiv.org/pdf/2302.06117)_
- **MobileSAM: semantic 게이트 통과 객체 수에 선형.** 객체당 `segment_box` 1회 — 전체 이미지 인코딩을 포함하는 SAM 호출이 객체마다 반복됨 (`yolo_ism.py:254`).
- **확인된 중복 연산 3건:**
  1. `normalize_rgb(rgb)` — 전체 이미지 float 변환·정규화가 `recognize()` 내부(`:158`)에 있어 **객체마다(≤10회/frame) 동일 계산 반복**. frame당 1회면 충분.
  2. **best proposal의 DINOv2 중복 forward** — `dinov2_forward`는 forward 안에서 patch token을 항상 계산하고 want_patch=False면 버림(`yolo_ism.py:110-115`). 5c에서 이미 forward한 crop을 5f에서 다시 forward → 통과 객체당 1회의 GPU forward가 순수 낭비.
  3. **동일 bbox의 cross-object 재계산** — proposal crop의 cls token은 객체와 무관한 값인데, 같은 prompt를 공유하거나 겹치는 box가 여러 객체 후보로 들어가면 객체마다 crop+forward를 반복. cls를 box 단위로 1회 계산해 전 객체가 재사용 가능한 구조임에도 현재는 객체×proposal로 스케일.
- **객체 수 증가 시 시간 증가 예상 구간 (추정, 측정 필요):** DINOv2 forward 횟수(선형×top_k) > MobileSAM 호출(선형, 통과율 의존) > normalize_rgb(선형) > score matmul(선형이나 절대량 미미).

### 3. Template Selection Analysis (확인됨)

- **로딩**: startup 1회, 디스크 캐시(`<name>_cls.pt` [42,384] / `<name>_appe.pt` list of [Np_i,384]) → 메모리 상주(`o["tcls"]`, `o["tappe"]`). cache miss 시에만 42장 렌더 이미지 DINOv2 추출(`yolo_ism.py:138-208`). **frame 루프 내 disk IO 없음.**
- **matching 구조**: 
  - semantic: proposal cls × 42 template cls cosine → top-5 평균 (`yolo_ism.py:216-218`)
  - template 선택: `best_t = argmax(tcls @ cls)` — **top-1 단일 best template** (`yolo_ism_object_n.py:190`). SAM-6D 원본과 달리 top-k template 후보 리스트를 만들지 않음.
  - appearance: best_t 1개의 masked patch feature와만 비교 (`yolo_ism.py:284`)
- **template 수 증가 시**: cls matmul [N_tmpl,384]@[384]와 argmax만 선형 증가 — CPU 소형 연산이라 **42→수백 규모까지는 병목 가능성 낮음 (추정)**. 오히려 template 수는 startup 캐시 빌드 시간(cache miss 시 N_tmpl회 DINOv2 forward)과 메모리(appe: template당 ≤256×384 float)에 영향.
- **결론 (추정, 측정으로 확인)**: template 선택 자체는 경량이며, 시간 비용의 중심은 **query 쪽 feature 추출(DINOv2)과 mask 생성(MobileSAM)**에 있다.

### 4. 아키텍처 특성 요약 — 계측 설계에 주는 함의

1. 순차 단일 스레드/단일 스트림 파이프라인(오프라인·ROS 공통) → stage 경계에 벽시계 타이머만 넣어도 합이 total과 일치해야 하며, GPU stage는 synchronize 필요.
2. per-object 루프(`recognize`)가 시간의 지배 후보 → **per_object 계측이 per_frame 계측과 동급으로 중요**.
3. 중복 연산 3건은 계측 스키마에 해당 sub-stage를 분리해 정량화 근거를 확보해야 함 (예: `dinov2_cls_ms` vs `dinov2_patch_ms`, `normalize_rgb_ms`).
4. 출력부(`cv2.imwrite` combined 매 frame + accepted overlay)는 Disk IO stage로 분리 측정 — 실험 산출물 저장이 순수 pipeline 시간을 오염시키는 정도를 판단.

_Section written: 2026-07-06_

---

## Bottleneck Candidate Top 5

> **모든 순위는 측정 전 추정이다.** 근거는 코드 구조(확인됨)이며, 순위 자체는 instrumentation 실행 후 확정한다.

### 후보 1: DINOv2 batch=1 반복 forward (객체×proposal 스케일링)

- **파일/함수**: `yolo_ism.py:107-115` `dinov2_forward`; 호출 루프 `yolo_ism_object_n.py:160-167`(cls) + `:175`(patch)
- **loop 구조**: frame당 [객체 ≤10] × [proposal ≤top_k 3] cls forward + [semantic 통과 객체] × 1 patch forward = **최대 ~40회 batch=1 GPU forward**
- **유형**: GPU + algorithmic scaling + CPU↔GPU transfer(매 호출 `.cpu()` `yolo_ism.py:111,114`)
- **근거 (확인됨)**: batch=1·224×224 소형 입력의 반복 launch는 kernel-launch/framework 오버헤드가 지배해 GPU가 유휴화되는 패턴(웹 검증). 게다가 patch forward는 cls forward와 동일 crop의 순수 중복(모델 forward는 patch token을 항상 계산, want_patch=False면 폐기).
- **확인 방법**: `dinov2_cls_ms`(proposal별 합)과 `dinov2_patch_ms`를 분리 계측 + forward 횟수 기록. 객체 1/5/10개 스윕에서 선형 증가 기울기 확인.
- **예상 metric**: `per_object_timing.dinov2_cls_ms`, `dinov2_patch_ms`, `dinov2_forward_count`

### 후보 2: MobileSAM `segment_box` — 객체당 전체 이미지 SAM 호출

- **파일/함수**: `yolo_ism.py:252-262` `segment_box`; 호출 `yolo_ism_object_n.py:185`
- **loop 구조**: semantic 게이트 통과 객체마다 1회. ultralytics `SAM(bgr, bboxes=[box])`은 **호출마다 전체 이미지 인코딩** 수행(bbox는 디코더 prompt일 뿐).
- **유형**: GPU + algorithmic scaling(통과 객체 수 선형) + GPU→CPU(mask `.cpu().numpy()` `:257`)
- **근거 (확인됨)**: 10객체 전부 semantic 통과 시 동일 이미지의 SAM 인코딩이 10회 반복. 같은 frame의 image embedding을 재사용하지 않음.
- **확인 방법**: `mask_mobilesam_ms`를 객체별 계측, 통과 객체 수와의 상관 확인. 단일 frame에서 호출 1회 vs N회 시간 비교.
- **예상 metric**: `per_object_timing.mask_ms`, `per_frame_timing.stage_mask_generation_ms`

### 후보 3: YOLO-World predict + (ROS) sidecar 소켓 왕복

- **파일/함수**: 오프라인 `yolo_ism_object_n.py:274`(imgsz=960); ROS `sam6d_multiobject_node.py:153-157` `_yolo_boxes` ↔ `tools/yoloworld_sidecar.py:82`(imgsz=640 하드코딩)
- **loop 구조**: frame당 1회(상수). ROS는 raw BGR ~0.9MB 소켓 복사 + 블로킹 동기 왕복 포함.
- **유형**: GPU(추론) + ROS2/IPC(sidecar) — 현재 왕복 내부(직렬화/전송/추론/응답)가 분리 계측 불가
- **근거 (확인됨)**: imgsz 960은 640 대비 약 2.25배 픽셀 — 오프라인이 실시간보다 느린 조건. 상수 비용이지만 절대량이 클 수 있음(측정 전 추정).
- **확인 방법**: 오프라인 `stage_yolo_world_ms`; ROS는 노드측 왕복 시간 + sidecar 내부 predict 시간(sidecar에 자체 타이머) 이중 계측으로 소켓 오버헤드 분리. imgsz 640/960 비교 런.
- **예상 metric**: `stage_yolo_world_ms`, `sidecar_rtt_ms`, `sidecar_infer_ms`(sidecar 로그)

### 후보 4: 출력부 Disk IO — 매 frame `cv2.imwrite` + (ROS) /dev/shm PNG 왕복

- **파일/함수**: `yolo_ism_object_n.py:307`(accepted overlay), `:318`(**combined jpg 매 frame 무조건**), findContours/drawContours `:304-306`; ROS `_pem_pose`의 PNG write→get_test_data 재read `sam6d_multiobject_node.py:169-181`(PEM interface 경계, 참고용)
- **유형**: Disk IO + CPU(jpg 인코딩)
- **근거 (확인됨)**: combined 이미지는 accepted 유무와 무관하게 frame마다 인코딩·기록. 해상도에 비례하는 상수 비용이 pipeline 시간에 합산됨.
- **확인 방법**: `stage_output_packaging_ms` 분리 계측 + `--no-vis`류 비교 런(저장 끄고 재측정)으로 순수 pipeline 시간과의 차이 정량화.
- **예상 metric**: `stage_output_packaging_ms`

### 후보 5: per-object CPU 전처리 중복 — `normalize_rgb` ×10 + CPU `crop_resize_pad`

- **파일/함수**: `yolo_ism.py:100-104` `normalize_rgb`(호출 `yolo_ism_object_n.py:158`, **recognize 내부 = 객체마다**), `yolo_ism.py:80-97` `crop_resize_pad`(CPU tensor에서 F.interpolate+F.pad)
- **유형**: CPU post-processing + 중복 연산(algorithmic)
- **근거 (확인됨)**: H×W×3 uint8→float32 변환+정규화(예: 848×480 기준 ~4.9MB float 텐서 생성)가 frame당 최대 10회 반복. crop resize도 GPU가 아닌 CPU에서 proposal마다 수행.
- **확인 방법**: `normalize_rgb_ms`(객체별) 계측 — frame당 합계가 유의미한지 확인. 1회 hoisting 시 절감량은 객체 수×단가로 즉시 계산 가능.
- **예상 metric**: `per_object_timing.normalize_rgb_ms`, `crop_ms`

> **범위 밖이지만 기록**: ROS 실시간 총 시간에서는 PEM 추론(`_pem_pose`)이 지배적일 가능성이 있으나(추정), 본 조사 범위(template 선택까지)에서는 위 5개가 대상. `_cb` 총 시간과 ISM-until-template-selection 시간을 분리 기록해 PEM 비중도 자연히 드러나게 한다.

---

## Instrumentation Design (최소 침습)

### 설계 원칙

1. **기본 OFF**: `--perf-out DIR` CLI 플래그(오프라인) / ROS 파라미터 `perf_out`(노드)가 지정된 때만 활성 — 미지정 시 기존 동작·출력 완전 동일.
2. **코드 수정 최소화**: 신규 모듈 `tools/perf/stage_timer.py` 1개 추가 + 기존 파일에는 `with timer.stage("...")` 래핑만 삽입. `recognize()` 시그니처 불변(모듈 전역 collector 사용, 비활성 시 no-op).
3. **GPU 정확도**: collector 활성 + device=cuda면 stage 경계에서 `torch.cuda.synchronize()` 후 `time.perf_counter()`. 근거: CUDA 커널 비동기 — 동기화 없이는 launch 시간만 측정됨(PyTorch 공식 recipe, 앞 절 인용). synchronize 자체가 파이프라이닝을 없애므로 **계측 ON 런의 total이 계측 OFF 런보다 다소 느려질 수 있음을 run_meta에 명시**하고, 계측 OFF 총 시간도 대조군으로 1회 측정.
4. **warm-up**: 처음 `warmup_frames`(기본 3) frame은 CSV에 기록하되 `is_warmup=1` 컬럼으로 표시 — summary 계산에서 제외. 첫 호출 cuBLAS/cudnn 초기화 비용 격리(웹 검증됨).

### Timer 삽입 위치 (오프라인 `yolo_ism_object_n.py`)

| stage name | 감싸는 코드 | 라인 |
|---|---|---|
| `frame_input` | `resolve_frames` yield 간격(직전 yield 종료~bgr 반환) | `:268` |
| `preprocess` | `cv2.cvtColor` | `:272` |
| `yolo_world` | `yolo.predict(...)` | `:274` |
| `box_routing` | prompt 그룹핑 루프 | `:277-286` |
| `recognize_total` | `recognize()` 호출 전체 (object_name 태깅) | `:293` |
| `output_packaging` | overlay/combined imwrite + CSV row | `:294-320` |

### Timer 삽입 위치 (`recognize()` 내부 — per-object sub-stage)

| sub-stage name | 감싸는 코드 | 라인 |
|---|---|---|
| `normalize_rgb` | `yi.normalize_rgb(rgb)` | `:158` |
| `dinov2_cls` | proposal 루프의 crop+forward 합 (proposal별 누적, `crop_ms` 분리 선택) | `:160-167` |
| `semantic_score` | `yi.semantic_score` 합 | `:165` |
| `dinov2_patch` | patch 재-forward | `:175` |
| `mask_mobilesam` | `yi.segment_box` | `:185` |
| `appearance_score` | `masked_query_patches`+`best_t argmax`+`masked_appe_score` | `:188-191` |

### ROS 노드 추가 계측 (`sam6d_multiobject_node.py`)

- `_cb` 진입 시: `recv_latency_ms = now − rgb_msg.header.stamp` (frame drop/지연 지표, ROS2 표준 방식 — 앞 절 인용)
- `sidecar_rtt_ms`: `_yolo_boxes` 왕복(`:204`); sidecar 프로세스에도 `predict` 자체 시간 로깅 추가(`yoloworld_sidecar.py:82` 전후) → RTT−infer = 직렬화+소켓 비용
- `ism_until_template_ms`: recognize 루프 종료까지(계측 범위 종점) vs `cb_total_ms`(기존 `:238` 로그와 동일 구간) — 차이가 PEM+publish+저장 비중
- drop 추정: 처리 frame 수 vs bag 메시지 수(`ros2 bag info`) 비교, `frame_i` 및 stamp 기록으로 사후 계산

### 기록 필드 처리

- `run_id`: `<bag명>_<YYYYmmdd-HHMMSS>` + `run_meta.json`에 config 전체 사본, imgsz, device, 계측 on/off, warmup_frames 기록
- `frame_index`/`stamp`: 오프라인은 `frame_idx`(resolve_frames), ROS는 `header.stamp`
- `object_name`/`object_index`: config objects 순서 기준
- `proposal_count`: recognize의 `num_proposals`(필터 후) + 필터 전 YOLO box 수(`detection_count`)
- `template_count`: `o["tcls"].shape[0]`(=42), `selected_template_id`: `best_t`(`:190`), 미도달 시 −1
- GPU 메모리: frame 종료 시 `torch.cuda.memory_allocated()/memory_reserved()` (MB). allocated=텐서 실사용, reserved=캐싱 할당자 보유(≥allocated). _Source: [memory_allocated docs](https://docs.pytorch.org/docs/stable/generated/torch.cuda.memory.memory_allocated.html), [PyTorch forums](https://discuss.pytorch.org/t/difference-between-allocated-and-reserved-memory/87278)_
- 저장 위치: `outputs/perf/<run_id>/{per_frame_timing.csv, per_object_timing.csv, stage_summary.csv, run_meta.json}` (+ ROS는 `ros_frame_timing.csv`)

---

## Proposed Timing Schema

### per_frame_timing.csv

`run_id, frame_index, stamp, is_warmup, image_width, image_height, object_prompt_count, detection_count, proposal_count_total, template_count_total, stage_frame_input_ms, stage_preprocess_ms, stage_yolo_world_ms, stage_box_routing_ms, stage_recognize_total_ms, stage_output_packaging_ms, total_until_template_selection_ms, gpu_memory_allocated_mb, gpu_memory_reserved_mb, notes`

- (ROS 전용 추가 컬럼) `recv_latency_ms, sidecar_rtt_ms, sidecar_infer_ms, cb_total_ms, pem_total_ms`
- `total_until_template_selection_ms` = frame_input~recognize_total 합(output_packaging 제외본도 계산 가능하도록 stage 분리 유지)

### per_object_timing.csv

`run_id, frame_index, stamp, object_index, object_name, detection_count_for_object, proposal_count_for_object, template_count_for_object, normalize_rgb_ms, dinov2_cls_ms, dinov2_forward_count, semantic_score_ms, dinov2_patch_ms, mask_ms, appearance_score_ms, recognize_total_ms, selected_template_id, best_sem, masked_appe, decision, accepted, notes`

- `decision`: 기존 recognize의 `no-object(no-proposal) / no-object(below-sim) / no-object(below-appe) / detected` 그대로 기록 → **게이트별 조기 종료가 시간 분포에 미치는 영향**(below-sim 객체는 mask/appe 비용 0) 분석 가능

### stage_summary.csv (run 종료 시 집계, warm-up 제외)

`run_id, stage_name, count, mean_ms, median_ms, p90_ms, p95_ms, max_ms, min_ms, std_ms, percent_of_total_mean, bottleneck_rank`

---

## Validation Plan (계측 구현 후 실행 설계)

측정은 전부 실제 bag/frame으로 수행하며(fake 데이터 금지), 각 런은 `run_meta.json`으로 조건을 완전 기록한다.

| # | 조건 | 방법 | 확인 가설 |
|---|---|---|---|
| V1 | 객체 1 / 5 / 10개 | config `enabled` 토글 3세트, 동일 bag·stride | DINOv2·MobileSAM 시간의 객체 수 선형성 (후보 1·2) |
| V2 | top_k 1 vs 3 | defaults.top_k 변경 | proposal 수 스케일링 분리 |
| V3 | imgsz 640 vs 960 | defaults.imgsz 변경 | YOLO 비용 및 sidecar 불일치 영향 (후보 3) |
| V4 | template feature cache cold vs warm | `--rebuild-features` on/off | startup 시간 차이(frame 시간엔 무영향 가설 확인) |
| V5 | 출력 저장 on/off | perf 런 시 imwrite skip 플래그 추가 | 후보 4의 절대량 |
| V6 | GPU warm-up | 첫 3 frame `is_warmup` 제외 전/후 통계 비교 | 초기화 비용 격리 |
| V7 | (ROS) bag 실속도 재생 | `ros2 bag play` 정속 + drop률(처리수/메시지수), recv_latency 추이 | ROS2 병목 여부 |
| V8 | 계측 off 대조군 | 동일 조건 1런, 총 시간만 로그 | synchronize 계측 오버헤드 정량화 |

steady-state 통계는 warm-up 제외 mean/median/p90/p95로 보고하고, 병목 순위는 `percent_of_total_mean` 기준으로 확정한다.

_Section written: 2026-07-06_

---

# Research Synthesis — SAM-6D ISM Pipeline 시간 성능 분석 설계 최종 보고

## Executive Summary

sam6d_ws의 객체 인식 pipeline(YOLO-World → semantic → MobileSAM mask → appearance → template 선택)의 실제 실행 경로를 파일:라인 단위로 재구성했다. **핵심 전제 발견: 이 pipeline은 SAM-6D 원본 ISM(`detector.py`)이 아니라 `yolo_ism.py`의 자체 재구현이며**, 오프라인 스크립트(`yolo_ism_object_n.py`)·ROS2 노드(`sam6d_multiobject_node.py`)·배치 도구가 모두 이 코드를 공유한다. 따라서 계측도 이 경로에 삽입해야 한다.

**가장 의심되는 병목 Top 3 (⚠️ 측정 전 추정 — 근거는 코드 구조 확인됨):**

1. **DINOv2 batch=1 반복 forward** — frame당 최대 ~40회(10객체×top_k 3 cls + 통과 객체당 patch 재추출). 224×224 소형 입력의 batch=1 반복은 kernel launch 오버헤드가 GPU 실행을 지배하는 전형 패턴이고, patch forward는 동일 crop의 순수 중복 연산이다.
2. **MobileSAM 객체당 전체 이미지 인코딩** — semantic 통과 객체마다 `segment_box`가 이미지 인코딩을 처음부터 반복. 같은 frame의 image embedding 재사용 없음.
3. **YOLO-World + (ROS) sidecar 소켓 왕복** — frame당 1회 상수 비용이나 절대량이 클 수 있고, 실시간 경로는 raw BGR ~0.9MB 소켓 복사 + 블로킹 왕복이 합산되며 현재 내부 분리 계측이 불가능. 오프라인 imgsz=960 vs sidecar imgsz=640 하드코딩 불일치도 발견됨.

판단 근거: template matching 자체(42×384 CPU matmul + argmax)는 경량임이 코드로 확인되어, 시간의 중심은 query 쪽 feature 추출(GPU forward 횟수)에 있다고 추정된다. 이 가설을 확정하기 위한 최소 침습 계측(기본 OFF, `tools/perf/stage_timer.py` + stage 래핑)과 3종 CSV 스키마, Validation Plan V1~V8을 설계했다.

**주요 확인 사실:** 중복 연산 3건(`normalize_rgb` 객체마다 재계산, best proposal DINOv2 이중 forward, 동일 bbox cls의 객체별 재계산), 매 frame combined jpg 무조건 기록, template feature는 startup 디스크 캐시로 frame 루프 내 IO 없음, YOLO-World text embedding은 `set_classes` 1회 캐시.

## Table of Contents

1. Technical Research Scope Confirmation — 조사 범위/제외 범위
2. Technology Stack Analysis — 실사용 스택 + 두 ISM 구현 공존 + 웹 검증 5건
3. Integration Patterns Analysis — env 분리/sidecar 프로토콜/ISM→PEM interface/데이터 흐름
4. Architectural Patterns Analysis — frame call path 재구성/Object-10 분석/Template Selection 분석
5. Bottleneck Candidate Top 5 — 후보별 파일·함수·유형·근거·확인 방법
6. Instrumentation Design — timer 삽입 위치/GPU sync/warm-up/저장 위치
7. Proposed Timing Schema — per_frame / per_object / stage_summary CSV
8. Validation Plan — V1~V8 비교 실행 조건
9. Project Structure Map (하단)
10. Next BMAD Skill Recommendation / 다음 액션 제안 (하단)

## Project Structure Map (확인됨)

| 경로 | 역할 |
|---|---|
| `yolo_ism.py` / `yolo_ism_object_n.py` | **실행 경로의 핵심** — ISM 자체 재구현 / 멀티객체 오프라인 entrypoint |
| `configs/yolo_ism_objects.yaml` | 멀티객체 동작점 config (10 enabled 객체, threshold/imgsz/top_k) |
| `src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py` | 실시간 ROS2 멀티객체 노드 (setup.py 미등록) |
| `tools/yoloworld_sidecar.py` | YOLO-World Unix socket 서버 (`sam_yolo` env) |
| `tools/build_ism_inputs_imu.py` + `tools/run_pem.sh` | 오프라인 ISM→PEM 브릿지 (manifest.csv) |
| `sam6d_master/SAM-6D/` | 원본 SAM-6D (ISM은 참조용, PEM은 실사용) |
| `template/<obj>/templates/` | 객체당 42 view 렌더 (rgb/mask png + xyz npy) |
| `outputs/yolo_ism_object_n/template_features/` | DINOv2 template feature 디스크 캐시 (`*_cls.pt`/`*_appe.pt`) |
| `data/cad/`, `data/ros2_bag/` | CAD ply, 입력 bag |
| `outputs/` | 실험 산출물 (계측 출력 권장 위치: `outputs/perf/<run_id>/`) |
| 모델 weight | 루트 `yolov8m-worldv2.pt`, `mobile_sam.pt`; `.../checkpoints/dinov2/dinov2_vits14_pretrain.pth`; PEM `sam-6d-pem-base.pth` |

## Research Methodology and Source Verification

- **코드 검증**: 병렬 탐색 에이전트 2개(구조/call path) 결과를 본 세션에서 `yolo_ism.py:40-299`, `yolo_ism_object_n.py:138-330`, `sam6d_multiobject_node.py` 전체, `yoloworld_sidecar.py` 전체 직접 열람으로 재확인. 문서 내 모든 라인 번호는 직접 확인본.
- **웹 검증 (공식/1차 소스)**: Ultralytics YOLO-World·FastSAM docs, SAM-6D 논문(arXiv:2311.15707), PyTorch benchmark recipe·cuda.Event·memory docs, PyTorch CUDA Graphs blog, Framework Tax(arXiv:2302.06117), ROS2 QoS design/Humble docs, message_filters docs, FastSAM 논문(arXiv:2306.12156). 미확인 2건은 본문에 명시(YOLO-World 클래스 수 스케일링 정량치, ROS2 executor 공식 단일 지표).
- **한계**: 모든 병목 순위는 측정 전 추정이며, 성능 수치는 일절 제시하지 않았다. 확정은 Validation Plan 실행 후.

## Next BMAD Skill Recommendation

1. **`/bmad-create-architecture`** (또는 곧바로 2번) — `stage_timer.py` 모듈 구조·collector 설계 확정이 필요하면 수행. 본 보고서의 Instrumentation Design이 이미 구체적이므로 생략 가능.
2. **`/bmad-quick-dev`** (권장 다음 단계) — timing logger + CSV report 구현. 본 보고서의 "Timer 삽입 위치" 표와 스키마를 spec으로 그대로 사용.
3. **`/bmad-technical-research` 또는 성능 리뷰** — Validation Plan V1~V8 실행 결과(실측 CSV) 기반 병목 확정 분석.

## 다음 액션 제안

1. **[권장] `/bmad-quick-dev`로 계측 구현 착수** — `tools/perf/stage_timer.py` + `yolo_ism_object_n.py` 오프라인 경로 계측(가장 재현 쉬운 경로)부터. ROS 노드 계측은 2차.
2. **sidecar imgsz 불일치(640 vs 960) 먼저 정리** — 계측 전 조건 통일 여부 결정 필요 (통일 없이 측정하면 실시간/오프라인 비교 불가).
3. **계측 없이 빠른 사전 확인** — 기존 `_cb` 총 시간 로그(`sam6d_multiobject_node.py:238`)만으로 PEM 포함/제외 대략 비중을 1개 bag에서 먼저 보고 우선순위 재조정.

어느 방향으로 진행할까요?

---

**Technical Research Completion Date:** 2026-07-06
**Source Verification:** 모든 외부 사실은 공식/1차 소스 인용, 모든 코드 사실은 파일:라인 직접 확인
**Confidence:** 코드 구조 사실 = 높음(확인됨) / 병목 순위 = 측정 전 추정
