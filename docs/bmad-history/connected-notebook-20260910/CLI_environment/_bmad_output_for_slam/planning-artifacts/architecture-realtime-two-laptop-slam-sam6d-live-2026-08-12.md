# 실시간 전환 설계 — 2노트북 SLAM + SAM-6D + Object Memory + 라이브 GUI

작성일: 2026-08-12 · 대상: `integration/` 오프라인 파이프라인의 실시간(ROS 2) 재구성
확정 전제(사용자 결정): **카메라 2대 리그(현행 규약 유지)** · **SLAM 백엔드 3종 전환 가능(hdl은 hdl_localization 도입)** ·
**GUI 최종 목표는 Unity 3D, 1차는 노트북 로컬 실시간 확인** · **SLAM 카메라 노트북 + SAM 카메라 노트북 2대에서 실행**

**2026-08-12 2차 결정 (착수 조건 확정)**

| 항목 | 결정 | 설계 영향 |
|---|---|---|
| 백엔드 구현 순서 | **ORB3 → RTAB → hdl** | ORB3 순수 localization 패치(§6.1)가 **1순위 작업**으로 승격 |
| 검증 단계 | **1단계: 노트북 1대에서 SLAM 실시간 검증 → SAM 실시간 검증**, 2단계: 2노트북 실환경 운용 시험 | 1단계에서는 **클록 동기·네트워크 리스크가 존재하지 않음**(R2/R6 2단계로 이연). 1단계 성공 = "각 파이프라인이 라이브로 돈다", 2단계 성공 = "둘이 같은 시계 위에서 융합된다" |
| Husky URDF 카메라 마운트 | **추후 등록** | 그때까지 `static_transform_publisher`로 임시 배선(§8.1). 계약(tf2 조회)은 지금 고정하므로 URDF 등록 시 코드 변경 0 |
| Velodyne | **실시간 구동 포함**(hdl_graph_slam) | LiDAR가 1단계 bringup부터 포함. hdl은 백엔드 순서상 3순위지만 **센서 배선은 처음부터** |
| 착수 지점 | **SLAM 실시간부터** | §15 로드맵 재정렬 — S3/S5(SLAM 경로)가 S1/S2(SAM 경로)보다 앞섬 |

---

## 1. 요구 6단계 → 컴포넌트 매핑

| # | 사용자 요구 | 담당 컴포넌트 | 현 상태 |
|---|---|---|---|
| 1 | 실시간 입력으로 맵 생성(3종 중 1) | `rt_bringup` mapping 런치 + 백엔드 노드 | ORB3/RTAB 가능, hdl 가능 |
| 2 | 최종 맵 저장 | `rt_map_manager` + 백엔드별 저장 규약 | ORB3 .osa(제약 있음)/RTAB .db/hdl SaveMap.srv |
| 3 | 저장 맵에서 실시간 위치추정(localization) | `rt_bringup` localize 런치 + `rt_slam_adapter` | ORB3 **가능(코어 3줄 패치 필요, §6.1)** / RTAB 네이티브 / **hdl 미보유(패키지 도입)** |
| 4 | 동시에 SAM-6D 객체 인식·포즈 | `sam6d_multiobject_node`(수정) + yoloworld 사이드카 | 노드 존재, **출력 계약 교체 필요** |
| 5 | Object Memory로 변화 인식 | `object_memory_node`(Phase B) | 존재, 미검증(라이브 배선 안 됨) |
| 6 | 실시간 GUI(맵 + 현재 위치 + 객체) | `rt_gui_bridge` + 웹 뷰 → 이후 Unity | **없음(신규)** |

---

## 2. 현 자산 실사 — 무엇이 이미 있고 무엇이 없나

**있는 것 (재사용)**

| 자산 | 경로 | 실시간 관련 사실 |
|---|---|---|
| ORB-SLAM3 RGB-D 래퍼 | `orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp` | `/orbslam3/pose`(PoseStamped), `tracking_state`(String), map points/dense 발행, `localization_mode` 파라미터, Atlas 저장/로드(settings 키) |
| ORB3 런치 배선 | `.../launch/orb_slam.launch.py:72` | `runtime.localization_mode` 노출 |
| RTAB-Map | `rtabmap_ws/.../rtab_slam.launch.py` | `database_path`, `delete_db_on_start` 이미 파라미터화 |
| hdl_graph_slam | `hdlgraphslam_ws/src/hdl_graph_slam` | `srv/{SaveMap,DumpGraph,LoadGraph}.srv` 보유, ndt_omp·fast_gicp 동봉 |
| SAM-6D 실시간 노드 | `sam6d_ws/src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py` | 모델 상주(DINOv2/MobileSAM/PEM/템플릿 1회 로드), RGB+aligned depth 동기 콜백, YOLO는 유닉스 소켓 사이드카 |
| YOLO 사이드카 | `sam6d_ws/tools/yoloworld_sidecar.py` | `sam_yolo` env 상주 서버(환경 분리 해결책) |
| Object Memory 스트리밍 엔진 | `objectmemory_ws/.../pipeline/object_memory_runner.py::StreamingObjectMemory.step()` | 오프라인/실시간 공용 코어, 다중 인스턴스·존재확률 필터 |
| 라이브 ROS 노드(Phase B) | `objectmemory_ws/.../ros/object_memory_node.py` | PoseBuffer + 캡처시각 정합, MultiThreadedExecutor 배선까지 완료 |
| 융합 좌표 규약 | `integration/README.md` §Stage 3 | `T_map_obj = T_map_camA(t)·X·T_camB_obj`, **X는 포즈 스트림에 곱함**(P_D 시야모델 때문) |
| 2D 격자맵 로직 | `integration/pipeline/gridmap.py` | 관측단위 log-odds — GUI 배경맵 생성에 그대로 재사용 |
| 리뷰 HTML/영상 | `integration/make_review.py` | 웹 뷰 레이아웃·데이터 직렬화 관용구 재사용 |
| 2노트북 수집 킷 | `sensor_sync_ws/` | 클록 동기 리서치·검증 스크립트 기반 |

**없는 것 (신규 개발)**

1. **모드 전환/맵 아티팩트 관리자** — 매핑↔로컬라이제이션 세션 관리, 맵 저장·매니페스트.
2. **백엔드 무관 포즈 계약** — `/slam/pose` + `/slam/status` 어댑터 3종.
3. **hdl localization** — 워크스페이스에 없음(`hdl_localization`은 별도 리포지토리). 벤더링 + ROS 2 빌드 필요.
4. **SAM 노드 출력 계약** — 현재 `PoseArray`라 **객체 이름/점수가 소실**됨. `Detection3DArray`로 교체 필수.
5. **Object Memory 출력 계약** — 현재 `PoseArray`. GUI/Unity에는 id·name·status·confidence가 필요.
6. **GUI 및 그 데이터 계약** — 전무.
7. **2노트북 실시간 클록 가드** — 오프라인은 `--time-offset`으로 사후 보정했지만 실시간에는 불가.

---

## 3. 시스템 토폴로지

```
┌─ 노트북 A (SLAM) ─────────────────────────┐        ┌─ 노트북 B (SAM) ───────────────────────┐
│ realsense cam_A  ─┐                        │        │ realsense cam_B ─┐                     │
│ (+ velodyne, hdl일 때)                     │        │                  │                     │
│                   ├→ SLAM 백엔드            │        │                  ├→ yoloworld_sidecar  │
│                   │   (ORB3 | RTAB | hdl)   │        │                  │   (sam_yolo env)    │
│                   │        │                │        │                  │        ↑ UNIX sock  │
│                   │        ↓                │        │                  ├→ sam6d_multiobject  │
│                   │  rt_slam_adapter        │        │                       (sam6d_ros_humble)│
│                   │   /slam/pose, /slam/status       │                          │             │
│                   │        │                │  LAN   │                          ↓             │
│                   │        ├──────────────────────── /sam6d/detections (Detection3DArray)     │
│                   │        ↓                │        │                    (+ /camB/color/compressed 5Hz) │
│                   │  rt_object_memory ──→ /object_map/landmarks                                │
│                   │        ↓                │        └────────────────────────────────────────┘
│                   └→ rt_gui_bridge (WebSocket JSON) ──→ 브라우저(1차) / Unity(최종)
└────────────────────────────────────────────┘
```

**배치 근거**

* **영상은 네트워크를 건너지 않는다.** 각 카메라는 자기 노트북의 소비자(SLAM/SAM)에게만 로컬로 흐른다. LAN을 넘는 것은 검출(수 KB/frame·수 Hz), 포즈, 객체맵, GUI용 저해상도 압축영상뿐 → 유선 기가비트로 충분, 무선도 가능.
* **object_memory는 노트북 A**에 둔다. 포즈 스트림(15~30 Hz)이 로컬이 되고, 네트워크를 건너는 것은 저빈도 검출뿐이라 네트워크 지연이 정합 품질에 미치는 영향이 최소화된다(지연은 PoseBuffer 캡처시각 정합이 흡수).
* GPU가 두 노트북에 분산되어 SLAM과 SAM이 서로 자원을 뺏지 않는다 — 현행 오프라인의 유일한 실질 이점 유지.
* DDS: 두 노트북 동일 `ROS_DOMAIN_ID`(예: 71 — 도메인 0은 과거 타 세션 토픽 충돌 사고가 있었음). 유선 서브넷 고정, 멀티캐스트 미지원 스위치면 Discovery Server 또는 `ROS_STATIC_PEERS` 사용.

---

## 4. 인터페이스 계약 (설계의 척추)

백엔드·GUI·객체 파이프라인을 갈아끼울 수 있게 하는 핵심. **먼저 고정하고 나머지를 여기에 맞춘다.**

| 토픽 | 타입 | 주기 | 발행자 | 의미 |
|---|---|---|---|---|
| `/slam/pose` | `geometry_msgs/PoseStamped` | 15~30 Hz | `rt_slam_adapter` | `T_map_camA`. **header.stamp = 그 포즈를 만든 이미지의 캡처 시각**, frame_id=`map` |
| `/slam/status` | `rt_msgs/SlamStatus` | 5 Hz | `rt_slam_adapter` | `mode`(MAPPING/LOCALIZING), `tracking`(OK/LOST/RELOCALIZING), `map_id`, `since_reloc_s`, `backend` |
| `/slam/map_grid` | `nav_msgs/OccupancyGrid` | latched 1회 + 갱신 | `rt_map_manager` | 저장된 맵의 2D 상면도(gridmap.py 산출) |
| `/slam/map_cloud` | `sensor_msgs/PointCloud2` | latched 1회 | `rt_map_manager` | 저장된 맵 점군(GUI 3D/Unity 배경) |
| `/slam/map_correction` | `geometry_msgs/TransformStamped` | 이벤트 | `rt_slam_adapter` | 전역 보정(merge/loop) 발생 시 좌표 델타 — §10 |
| `/sam6d/detections` | `vision_msgs/Detection3DArray` | 1~3 Hz | `sam6d_multiobject_node` | `results[0].hypothesis.class_id`=객체명, `.score`, `results[0].pose.pose`=`T_camB_obj`(m). **header.stamp = RGB 캡처 시각** |
| `/object_map/landmarks` | `rt_msgs/ObjectMap` | 검출마다 + 1 Hz | `rt_object_memory` | id·name·status·confidence·obs·missed·`T_map_obj`·last_seen |
| `/object_map/markers` | `visualization_msgs/MarkerArray` | 동일 | `rt_object_memory` | RViz 디버그용 |
| `/camB/color/compressed` | `sensor_msgs/CompressedImage` | 5 Hz | `image_transport` 리퍼블리셔 | GUI 미리보기(원본 스트림은 로컬 유지) |

**불변식**

* **INV-RT-1**: 모든 스탬프는 "센서 캡처 시각"이다. 발행 시각·수신 시각을 쓰지 않는다(RealSense `global_time_enabled: true`).
* **INV-RT-2**: `/slam/pose`는 **map 프레임이 확정된 뒤에만** 발행된다. 로컬라이제이션 미완료 구간의 포즈는 발행 금지(§6 ORB3 함정).
* **INV-RT-3**: 검출은 절대 `map` 프레임으로 변환해서 보내지 않는다. 카메라 프레임(`T_camB_obj`) 그대로 보내고, 포즈 정합은 object_memory가 캡처시각 기준으로 수행한다(늦은 검출을 옳게 처리하는 유일한 방법).
* **INV-008(기존 유지)**: 포즈가 없는 프레임의 검출은 융합하지 않고 버린다.

`rt_msgs`는 신규 인터페이스 패키지(3개 메시지). Unity/웹은 ROS 메시지를 직접 보지 않고 §11의 JSON 스키마만 본다.

---

## 5. 운영 모드와 맵 아티팩트

### 5.1 세션 상태기계 (`rt_map_manager`)

```
IDLE ──start_mapping──> MAPPING ──stop_mapping──> SAVING ──> MAP_READY
MAP_READY ──start_localization(map_id)──> LOCALIZING ⇄ (RELOCALIZING)
```

* 매핑과 로컬라이제이션은 **별도 프로세스 수명**이다(같은 프로세스에서 모드 전환하지 않는다). ORB3는 Atlas 저장이 `Shutdown()` 시점에만 일어나고, RTAB은 `Mem/IncrementalMemory` 가 기동 인자이며, hdl은 아예 다른 노드다 — 세 백엔드 모두 프로세스 재기동이 가장 단순하고 안전하다.
* 매니저는 런치를 자식 프로세스로 띄우고 `SIGINT`로 정상 종료시킨 뒤 산출물을 아티팩트 폴더로 수거한다.

### 5.2 맵 아티팩트 규약

```
maps/<site>/<map_name>/
    manifest.json          # 백엔드, 생성시각, 카메라 intrinsic, 사용 extrinsic, 좌표 규약, 파일 목록, 체크섬
    grid.yaml + grid.pgm   # 2D 상면도 (gridmap.py) — GUI 배경
    cloud.pcd              # 점군 (GUI/Unity 3D 배경)
    orb3/  map_<name>.osa  settings_mapping.yaml  settings_localize.yaml
    rtabmap/ rtabmap.db
    hdl/  globalmap.pcd  graph_dump/
```

`manifest.json`이 "이 맵은 어느 백엔드로 만들었고, 어떤 카메라/외부파라미터 가정 위에 서 있는가"를 못박는다. 로컬라이제이션 기동 시 매니저가 현재 카메라 intrinsic과 매니페스트를 대조해 불일치면 거부한다(과거 세션별 fx가 달랐던 사고 방지).

---

## 6. 백엔드별 저장/로드·로컬라이제이션 설계 (함정 포함)

### 6.1 ORB-SLAM3 — 순수 localization은 **가능하다**. 원인은 기동 경로 버그였다.

> **2026-08-12 정정**: 초판은 "순수 localization 모드가 RGB-D Atlas에서 동작하지 않는다"고 적었다. 이는 PoC 증상만 옮긴 것으로, 소스 확인 결과 **기능 부재가 아니라 우리 쪽 기동 경로 문제**였다. 아래가 코드 근거와 수정안이다.

**증상의 진짜 원인 (`orbslam_ws/src/ORB_SLAM3` 소스 확인)**

1. `src/System.cc:171` — Atlas를 로드한 직후 코어가 **무조건 `mpAtlas->CreateNewMap()`을 호출**한다. 즉 로드된 맵은 Atlas 안에 얌전히 들어 있지만 **현재 맵(active map)은 텅 빈 새 맵**이 된다.
2. `src/Tracking.cc:1899` — 상태가 `NOT_INITIALIZED`이므로 RGB-D는 `StereoInitialization()`으로 간다. 이 함수는 **현재 프레임을 원점(identity)으로 새 맵을 초기화**할 뿐 `Relocalization()`을 시도하지 않는다. → 전 프레임 identity 포즈의 정체.
3. `src/Tracking.cc:3617` — `Relocalization()`은 `DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap())`로 후보를 찾고, `src/KeyFrameDatabase.cc`가 **현재 맵에 속한 키프레임만** 남긴다. 현재 맵이 빈 새 맵이면 **후보는 구조적으로 0개**다.
4. localization 모드는 LocalMapping을 정지시키므로 새 키프레임이 생기지 않고 → LoopClosing이 받을 키프레임도 없어 → **merge도 영원히 일어나지 않는다.** 그래서 normal 모드(=LocalMapping 동작 → 키프레임 생성 → LoopClosing이 "Merge detected")에서만 결과가 나왔던 것이다.

**즉, 순수 localization에 필요한 재료는 이미 전부 있다.**
`Tracking::Relocalization()`(`Tracking.cc:3609`)은 BoW 후보 검색 + PnP RANSAC + 최적화의 완전한 구현이고,
로드된 키프레임은 `Map::PostLoad()`에서 `pKFDB->add(pKFi)`(`src/Map.cc:469`)로 **KeyFrameDatabase에 이미 등록**된다.
막힌 곳은 단 하나 — **활성 맵이 로드된 맵이 아니라는 것**.

**수정안 (코어 최소 패치, `System.LoadAtlasFromFile`이 설정된 경우에만 분기)**

```cpp
// System.cc — Atlas 로드 직후
//   기존: mpAtlas->CreateNewMap();            // 빈 새 맵을 활성화 → 리로컬 불가
//   변경: 로드된 맵 중 키프레임이 가장 많은 맵을 활성 맵으로 지정
mpAtlas->ChangeMap(pLoadedMap);               // Atlas.h:80 에 이미 존재하는 API
// Tracking 생성 이후
mpTracker->mState = Tracking::LOST;           // Tracking.h:131, public 멤버
```

이러면 첫 프레임부터 `Track()`이 "시스템은 초기화됨 + LOST" 분기로 들어가
`Tracking.cc:2043`의 `bOK = Relocalization();`을 타고 **로드된 맵의 키프레임 DB에서 현재 위치를 찾는다.**
성공 후에는 `TrackWithMotionModel` / `TrackLocalMap`이 기존 맵의 맵포인트로 추적을 이어가고,
LocalMapping이 꺼져 있으므로 **맵은 변경되지 않는다** — 요구사항 그대로의 "only localization".

부수 효과로 §10의 전역 보정 문제도 사라진다(맵 고정 → RTAB/hdl과 동일한 성질).

**설계 반영**

* 파라미터 `localization_mode: true` → 위 패치 경로 사용(맵 불변, 순수 위치추정).
* 검증 중 리로컬 성공률이 낮게 나오면(조명/시점 변화 등) **fallback으로 기존에 실증된 merge 모드**(atlas 로드 + normal 모드, 260714 PoC 99.1%)를 파라미터로 선택할 수 있게 남겨 둔다. 이때만 §10 전역 보정 처리가 필요하다.
* 어느 경로든 **리로컬 성립 전 포즈는 발행하지 않는다**(INV-RT-2). `/slam/status.tracking = RELOCALIZING` → 성공 시 `OK`. 판정은 stdout 파싱이 아니라 `mState`/현재 맵 id로 한다.

### 6.1.1 실행 검증 결과 (2026-08-12, ROS 2 Jazzy `orb3slam` env)

패치를 적용해 실제로 돌린 결과. 입력은 bag 재생(`rate 1.0` = 실시간 등가), 출력은 `orbslam_ws/output/260714_jazzy/`.

| 실행 | 결과 |
|---|---|
| **매핑** `slam_105023` | 2004프레임, **맵 1개**(id 0, KF 189, 맵포인트 23853), track **평균 18.7 ms**(p95 24.5) → 53 fps 여유. `.osa` 61 MB 저장 성공 |
| **순수 localization — 같은 카메라** `slam_105319` | **성공**. 첫 프레임대에서 `Relocalized!!`, **1482프레임 중 1481포즈(99.9%)**, track 20.2 ms. **`.osa` md5 불변** = 맵 고정 확인 |
| ↳ 정합 검증(핵심 증거) | 로컬라이즈된 궤적의 각 포즈에서 **매핑 궤적까지 최근접 거리 median 4.7 cm**, p90 54 cm, **100%가 1 m 이내**. 즉 105319가 105023의 경로 위에 정확히 얹혔다 — "그럴듯한 숫자"가 아니라 실제 정합 |
| 순수 localization — 교차 카메라 `sam_105018` | 리로컬 3회뿐, 최근접 거리 median 181 m = 실패. **아키텍처상 불필요한 경로**(아래) |

**패치는 2건이 되었다.** 1차(활성 맵 전환)만으로는 부족했고, 실행이 두 번째 결함을 드러냈다.

* **패치 1** `Tracking::ActivatePriorMapForLocalization()` — 로드된 맵을 활성화하고 `mState=LOST`로 두어 `Relocalization()` 경로로 보낸다.
* **패치 2** `Tracking.cc`의 LOST 에스컬레이션 차단 — 리로컬 실패가 쌓이면 스톡 ORB3는 `CreateMapInAtlas()`로 **"이 맵을 포기하고 새 맵을 시작"**한다(`Tracking.cc:2279`, 맵 KF>10일 때). 사전 맵 위에서 위치만 추정하는 모드에서 이것은 **항상 잘못**이다 — 실제로 교차 카메라 실행에서 맵이 8개 생성되며 패치 1과 프레임마다 싸웠고 궤적이 오염됐다(median 178 m). 이제 `IsLocalizingOnPriorMap()`이면 **LOST를 유지한 채 계속 리로컬을 시도**한다. 이것이 "못 찾으면 계속 찾는다"는 요구 그대로의 동작이다.

**대조 실험으로 원인 확정 (2026-08-12)** — 질의 bag(`sam_105018_dense`)·패치·설정을 고정하고 **맵을 만든 카메라만** 바꿨다.

| 맵 제작 카메라 | 질의 | 결과(질의 궤적 → 맵 궤적 최근접 거리) |
|---|---|---|
| **SAM 카메라**(`sam_105314`) | SAM 105018 | **2003포즈, median 12.2 cm, 100% < 1 m** — 정상 |
| SLAM 카메라(`slam_105023`) | SAM 105018 (동일) | 1400포즈, median 181 m — 실패 |
| SLAM 카메라(`slam_105023`) | SLAM 105319 | 1481포즈, median 4.7 cm, 100% < 1 m — 정상 |

→ **패치·SAM 데이터 모두 무죄. 원인은 리그의 ~90° 시선 차이**다. 같은 시각·같은 방이라도 두 카메라는 같은 장면을 보고 있지 않으므로 BoW 리로컬이 후보를 찾지 못한다. 결정 변수는 시간이 아니라 시점이다. (SAM 카메라 단독 매핑도 정상: 1483프레임·맵 1개.)

**⚠ 실험이 드러낸 추가 요구사항 — VO 폴백 노출.** ORB3 localization 모드는 맵 포인트를 잃으면 `mbVO=true`로 **프레임 간 매칭만으로 계속 추적**한다(미매핑 구역 통과용 설계). 그 포즈는 맵에 고정되지 않은 자유 드리프트인데 스톡 ORB3는 이를 외부에 알리지 않아, 실패 실행에서 **180 m 떨어진 좌표가 아무 경고 없이 정상 포즈로 발행**됐다(노드가 실시간 발행한 `CameraTrajectory_live.txt`와 사후 저장본이 일치 → 저장 로직 인공물이 아님). 객체 융합이 이 값을 받으면 지도가 즉시 오염된다.
→ **S5c 확장**: `/slam/status`는 `OK / RELOCALIZING / LOST`에 더해 **맵 고정 여부(`mbVO`)**를 실어야 하고, **VO 상태의 포즈는 `/slam/pose`로 내보내지 않는다**(INV-RT-2 강화).

### 6.1.2 0807 충북 실환경 재방문 검증 (2026-08-12) — 실제 운용 조건

260714는 같은 세션 반복이라 조건이 쉽다. 0807은 **같은 카메라·같은 장소지만 경로·객체/매대 배치·조도가 모두 다른** 실제 재방문 조건이다. 맵=`182943__large_loop_then_small_loop_cw`(18:30), 질의=`183504`(+5분), `190624`(+36분, 조도 차 최대).

**결과 1 — 초기 위치추정은 즉시 성공한다.** 두 세션 모두 **첫 프레임에서 리로컬**됐다. 같은 카메라·같은 장소면 배치와 조도가 달라도 붙는다.

**결과 2 — 문제는 로스트 이후의 복구다.** 스톡 ORB3는 앵커를 잃으면 VO 폴백으로 넘어가 **수백 km까지 발산하고 다시는 맵으로 돌아오지 않는다**. 그 포즈는 0이 아니라서 래퍼가 그대로 발행한다(`rgbd_node.cpp`는 `Tcw.isZero()`일 때만 발행을 건너뛴다).

| | 스톡(VO 폴백 ON) | **패치 3 (사전맵에서 VO 억제)** |
|---|---|---|
| 183504 | 3715 포즈, median 8356 m, 앵커 구간 1790프레임 뒤 발산 | **771 포즈 전부 앵커**, median **0.14 m**, max 1.3 m |
| 190624 | 3228→ 발산, 앵커 857프레임(9%) | **3228 포즈 전부 앵커**, median **0.18 m**, 98% < 1 m |
| 맵 불변 | ○ | ○ (md5 동일) |

**패치 3**: 사전 맵 모드에서 `mbVO`(맵 포인트 매칭 <10)이면 추측 항법 대신 **LOST로 두고 리로컬을 계속 시도**한다. "없는 포즈"가 "지어낸 포즈"보다 안전하다는 원칙 — 객체 융합은 틀린 포즈를 영구히 지도에 새기지만, 빠진 포즈는 검출 한 건을 건너뛸 뿐이다.

**남은 트레이드오프(미해결)**: 패치 3은 짧은 가림에도 즉시 리로컬로 떨어져 커버리지를 깎는다. 183504에서 스톡의 앵커 연속 구간(1790프레임 ≈ 60 s)이 패치 후 33 s로 줄었다(190624는 반대로 28 s → 163 s로 크게 개선). 최적안은 **VO를 짧게 허용하되(예: 1초) 그 안에 재앵커되지 않으면 LOST로 강등 + 앵커 상태만 발행**하는 하이브리드다. S5c에서 정한다.

**부수 발견 — 매핑 크래시로 맵 전손.** 첫 0807 매핑 시도가 ORB3의 알려진 `SO3::exp failed! omega: -nan`으로 죽었고, 저장이 `Shutdown()`에서만 일어나므로 **125초 주행분이 통째로 사라졌다**(재실행으로 복구, 0724에서도 9세션 중 4건 전례). → **S5a에 주기적 맵 체크포인트 저장을 추가**한다(코어 `SaveAtlas`를 public 노출 + ROS 서비스).

**교차 카메라 리로컬은 이 아키텍처에 필요 없다.** 최종 시스템에서 위치추정을 하는 것은 **SLAM 카메라뿐**이고, SAM 카메라의 포즈는 §8의 tf 체인(`T_map_camA · X`)으로 유도된다. 105018 실행은 "SAM 카메라 bag을 SLAM 카메라 맵에 넣는" 경우라 리로컬 난이도만 높고 운용 시나리오와 무관하다. 과거 PoC가 이 조합에서 99.1%를 얻은 것은 merge 모드(place recognition + Sim3 검증)가 BoW 리로컬보다 관대하기 때문이며, **필요해지면 merge 모드가 그 용도로 남아 있다**.

**저장/로드 경로 (변동 없음)**

* 저장: `System.SaveAtlasToFile: "<name>"`. 코어가 `"./"+name+".osa"`로 **cwd 상대경로를 강제**(`System.cc:1501`)하므로, 매니저가 노드의 **cwd를 맵 폴더로 지정**해 띄운다. 저장은 `Shutdown()`에서만 일어나므로 **정상 종료(SIGINT)가 곧 저장 경로**다 — 강제 종료 시 맵 소실.
* 로드: `System.LoadAtlasFromFile: "<name>"` + **`.osa`를 실행 cwd에 복사**.

### 6.2 RTAB-Map — 가장 표준적

* 매핑: 현행 런치 + `delete_db_on_start: true`, `database_path=<맵폴더>/rtabmap.db`.
* 로컬라이제이션: 같은 db + `Mem/IncrementalMemory=false`(+`delete_db_on_start=false`). 맵이 **고정**되므로 전역 보정 문제 없음.
* 어댑터: `/rtabmap/localization_pose` 또는 tf `map→cam_A_optical` 조회 → `/slam/pose`. 재위치추정 전에는 map→odom이 없으므로 자연히 발행되지 않는다(게이팅 무료).
* 과거 교훈 반영: RTAB 입력 동기는 **exact_sync + RELIABLE QoS**라야 정상(BEST_EFFORT 유실 시 odom 급감). 실시간 로컬 카메라이므로 만족 가능.

### 6.3 hdl_graph_slam — localization 노드 신규 도입

* 매핑: 현행 hdl 런치. 종료 전 `/hdl_graph_slam/save_map`(SaveMap.srv)로 `globalmap.pcd`, `dump_graph`로 그래프 저장.
* 로컬라이제이션: **`hdl_localization`을 `hdlgraphslam_ws`에 벤더링**(ROS 2 포트). 구성 = globalmap.pcd + NDT/GICP 정합(`ndt_omp`, `fast_gicp` 이미 동봉) + (선택)IMU 예측 UKF + 초기 포즈. 초기 포즈는 (a) 파라미터 지정, (b) RViz `2D Pose Estimate`, (c) `hdl_global_localization` 중 택1 — 1차는 (a)+(b).
* **좌표 함정**: hdl은 **LiDAR를 위치추정**한다. 객체 융합에는 `T_map_camA`가 필요하므로 **`T_velo_camA` 정적 캘리브가 선결**이다(현재 미완 항목). 미보정 상태로 hdl 경로를 쓰면 객체가 계통 오프셋을 갖는다.
* **선행 확인**: 원본 `/velodyne_points`가 실제와 **좌우 반전**돼 있다는 기록이 있다. 실시간 hdl 경로를 살릴 때 드라이버 설정에서 이 문제부터 재확인해야 한다(궤적 정합만으로는 거울상이 검출되지 않음).
* 난이도상 **hdl은 3순위**로 두고, 계약(`/slam/pose`)만 먼저 맞춰 두면 나중에 끼워넣는 비용이 작다.

---

## 7. 시간·동기 설계 (2노트북에서 가장 큰 리스크)

### 7.0 "2노트북 클록"이 무슨 문제인가 (구체)

이 시스템의 융합은 딱 한 줄이다 — **"이 검출이 찍힌 순간에 SLAM 카메라는 어디에 있었나"**.

```
검출(노트북 B에서 t_B 스탬프) ──→ PoseBuffer.lookup(t_B) ──→ 그 시각의 T_map_camA ──→ T_map_obj
```

포즈 스탬프는 **노트북 A의 시계**로, 검출 스탬프는 **노트북 B의 시계**로 찍힌다.
두 시계가 Δ만큼 어긋나 있으면, 검출을 **Δ 전(또는 후)의 로봇 위치**에 붙이게 된다. 오차는 그대로

> 객체 위치 오차 ≈ 로봇 속도 × Δ

* Δ = 1 ms, 0.5 m/s → 0.5 mm (무시 가능)
* Δ = 100 ms → 5 cm (association gate 0.20 m를 갉아먹기 시작)
* Δ = 6.6 s → **3.3 m** ← 260804 데이터에서 실제로 발생한 값. 오프라인이라 사후 상수 보정으로 살렸지만, 실시간에는 그 사치가 없다.

즉 카메라 하드웨어 동기(genlock)의 문제가 아니라 **두 호스트의 절대 시각이 같은가**의 문제이며,
chrony/PTP로 sub-ms까지 맞추면 **사실상 소멸하는 리스크**다. 진짜 위험은 기술적 난이도가 아니라
**설정을 잊은 채 돌리는 것**(전례가 있음) — 그래서 아래에 런타임 가드를 넣는다.
(참고: 향후 두 카메라가 한 컴퓨터에 붙는 구성이 되면 이 항목 전체가 사라진다.)

### 7.1 계층별 설계

| 계층 | 설계 |
|---|---|
| 호스트 클록 | 유선 링크에서 노트북 A를 `chrony` 서버, B를 클라이언트(또는 둘 다 동일 상위 NTP). NIC가 지원하면 `linuxptp` 권장. 목표 \|offset\| < 2 ms |
| 카메라 스탬프 | 두 RealSense 모두 `global_time_enabled: true` → 이미지 스탬프가 호스트 클록 기준 캡처 시각 |
| 하드웨어 genlock | 선택. 0807 장비에서 depth 계열 genlock 동작이 확인됨. 융합은 포즈 보간으로 흡수하므로 **필수는 아님** |
| 런타임 가드 | `rt_clock_guard` 노드: 양 노트북이 서로의 하트비트 스탬프로 offset을 상시 추정 → `/slam/status`에 실어 GUI 표시. 경고 > 5 ms, **차단 > 20 ms**(융합 중단 + GUI 경고) |
| 정합 | object_memory의 `PoseBuffer.lookup(capture_stamp)`가 담당. 버퍼 창 = 5~10 s(현행 기본 10 s 유지) ≥ SAM 지연 + 네트워크 지연 |

`--time-offset` 류의 사후 보정은 **실시간 경로에서 금지**한다. 대신 가드가 실패를 즉시 드러낸다.

---

## 7.5 개발 환경 실사 (2026-08-12 확인) — 착수 전 알아야 할 4가지

| 사실 | 확인 방법 | 계획에 주는 영향 |
|---|---|---|
| ROS 2 **Humble**이 `/opt/ros`가 아니라 **conda 환경**(RoboStack)에 있다 (`orbslam3`, `sam6d_ros_humble`, `rtabmap`, `hdl_graph_slam_humble`) | `conda-meta`에 `ros-humble-*` | 런치·빌드는 전부 conda env 활성화 전제. 신규 `realtime_ws`도 같은 방식으로 빌드 |
| **이 워크스테이션에는 카메라가 없다** (`/dev/video*` 없음, RealSense USB 미검출) | `lsusb`, `ls /dev/video*` | 여기서의 "실시간"은 **`ros2 bag play`를 라이브 소스로** 쓴다. 노드 입장에서 카메라와 구분되지 않으므로 유효한 검증이며, **실제 카메라 검증은 노트북에서**(1단계 계획과 일치) |
| `vision_msgs`가 **`sam6d_ros_humble`에만** 있다 | 각 env `share/` 조회 | object_memory·어댑터가 도는 env(`orbslam3`)에 `ros-humble-vision-msgs` 설치 필요 (S1 착수 전 준비물) |
| `realsense2_camera`·`velodyne` 패키지가 **어느 conda env에도 없다** | 동일 | 드라이버는 현장 노트북에 apt로 설치돼 있어야 한다(`sensor_sync_ws/README.md` 전제). 워크스테이션 개발은 bag 재생으로 진행 |

**⚠ 배포판 불일치 주의**: `sensor_sync_ws`의 수집 킷은 **Jazzy** 기준으로 작성돼 있는데, 이 저장소의 알고리즘 환경은 전부 **Humble**이다. ROS 2는 배포판 간 통신을 보장하지 않으므로(Iron 이후 타입 해시 변경), **2노트북 단계에서 두 대의 배포판을 반드시 일치**시켜야 한다. 1단계(단일 노트북)에서는 문제되지 않으므로, 2단계 착수 전 결정 항목으로 남긴다.

---

## 8. 좌표계 설계 — **URDF 기반 tf2 트리** (2026-08-12 변경)

`.npy` 캘리브 파일로 `X`를 들고 다니는 대신, **Husky URDF에 센서 마운트를 등록**하고 tf2로 조회한다.
이것이 더 단순할 뿐 아니라 여러 문제를 한꺼번에 없앤다.

```
map ─T_map_base(t)─> base_link ─URDF(고정)─> cam_A_link ─> cam_A_color_optical_frame
                          │
                          ├─URDF(고정)─> cam_B_link ─> cam_B_color_optical_frame ─T_camB_obj─> object
                          └─URDF(고정)─> velodyne
```

* `X = T_camA_camB`는 이제 파일이 아니라 **tf2 조회 결과**다: `lookup(cam_A_color_optical_frame → cam_B_color_optical_frame)`. 리그를 바꾸면 URDF 한 곳만 고친다.
* **hdl 경로의 `T_velo_camA` 문제가 자동 해소된다** — velodyne도 같은 URDF 트리의 한 링크일 뿐이다(§6.3의 선결 과제가 사라짐).
* SLAM 어댑터는 `T_map_camA` 대신 **`T_map_base`를 발행**하는 편이 자연스럽다(백엔드가 무엇을 위치추정하든 base_link로 환산). 그러면 백엔드 교체가 tf 트리에 전혀 영향을 주지 않는다.
* **`X`는 검출이 아니라 포즈 스트림에 곱한다**(현행 규약 유지): `T_map_camB = T_map_camA · X`. 결과 위치는 같지만 존재확률 필터의 시야모델 `P_D`가 실제 검출을 만든 카메라의 절두체를 보게 된다.

**URDF로 갈 때 반드시 지킬 2가지 (이 저장소에서 실제로 사고가 났던 지점)**

1. **link 프레임 ≠ optical 프레임.** URDF의 `camera_link`는 REP-103(x=전방, z=상방)이지만, SAM-6D가 내놓는 `T_camB_obj`는 **광학 프레임**(z=전방, x=우, y=하) 기준이다. 둘 사이에는 고정된 ∓90° 회전이 두 번 들어간다. 이걸 빠뜨리면 객체맵이 **조용히 90° 돌아간 채** 나온다(이 프로젝트에 90.1° extrinsic 추정치와 "choco 90° 뒤집힘" 전례가 있다). RealSense 드라이버가 발행하는 `*_color_optical_frame`을 **끝단으로 명시**하고, 융합은 반드시 optical 프레임끼리 연결한다.
2. **URDF 값은 설계치이지 실측치가 아니다.** 병진은 줄자로 cm 정확도가 나오지만 **회전은 안 나온다**. 3 m 거리에서 yaw 2° 오차 = 객체 위치 10 cm 오차다. 과거 동일 데이터에서 캘리브 X는 생존 인스턴스 29개, 손측정 리그값은 125개(같은 물체가 시점마다 번짐)였다.
   → **URDF를 정본으로 삼되, 최초 1회 검증**을 넣는다: 두 카메라로 각각 ORB3를 돌려 `integration/rig_offset.py`(hand-eye)로 상대자세를 추정하고 URDF 값과 대조. 회전 차이 > 2°면 URDF 쪽을 실측값으로 갱신한다. 이건 캘리브 파이프라인을 유지하자는 게 아니라 **URDF 수치를 한 번 검산**하자는 것이다.

* `manifest.json`에 그 맵을 만들 때 쓴 URDF 버전/체크섬을 기록해, 리그를 바꾼 뒤 옛 맵을 쓰는 사고를 막는다.

### 8.1 URDF 등록 전까지의 임시 배선

카메라 마운트 URDF 등록은 추후이므로, 그때까지는 `static_transform_publisher`(런치 파라미터)로 같은 tf 링크를 발행한다.

```
base_link → cam_A_link → cam_A_color_optical_frame      (RealSense 드라이버가 optical 링크는 스스로 발행)
base_link → cam_B_link → cam_B_color_optical_frame
base_link → velodyne
```

* **소비자 코드는 처음부터 tf2 조회만 한다.** 값의 출처가 static publisher냐 URDF냐는 소비자에게 보이지 않으므로, URDF 등록 시 **코드 변경이 0**이고 런치 설정만 바뀐다.
* 임시값의 출처는 기존 추정치(리그 hand-eye 90.38°/0.426 m 등)와 줄자 실측이며, `manifest.json`에 `extrinsic_source: "static_tf(provisional)"`로 남겨 URDF 확정본과 구분한다.
* SLAM 단독 검증(1단계 전반부)에는 이 값이 전혀 필요 없다 — **카메라 간 변환은 융합 단계에서만 쓰인다.** 따라서 URDF 지연이 SLAM 실시간 착수를 막지 않는다.

---

## 9. 비동기·지연 처리

### 9.1 지연 예산 (워크스테이션 실측 기반, 노트북은 2~3배 가정)

| 구간 | 실측/근거 | 노트북 추정 |
|---|---|---|
| 캡처 → 이미지 발행 | ~30 ms | ~30 ms |
| YOLO 사이드카 | 프롬프트별 다중 패스가 병목(1-pass multi_label 개선안 보유) | 80~250 ms |
| ISM recognize | **0.12 s/frame**(실측) | 0.2~0.4 s |
| PEM | **0.11 s/detection**(warm, 상주 모델) | 0.2~0.3 s × N |
| 검출 발행 + LAN | < 15 ms | < 15 ms |
| object_memory step | 355 det에 0.1 s → 사실상 0 | ~0 |
| **검출 end-to-end** | **0.3~0.8 s** | **0.7~1.5 s** |

* 이 값은 목표 주기가 아니라 **한 사이클의 소요 시간**이다. 스케줄은 §9.2의 자유 실행이며, 처리량은 `1 / (그때그때의 소요 시간)`으로 객체 수와 검출 수에 따라 변동한다.
* SLAM 포즈는 15~30 Hz, 객체 갱신은 그보다 한 자릿수 느리다 — **비대칭이 정상**이며 PoseBuffer 캡처시각 정합이 이를 전제로 설계돼 있다.

### 9.2 프레임 정책 — **고정 주기가 아니라 자유 실행(free-running)**

목표 동작(확정): *"한 프레임 처리가 끝나면, 그 직후 들어오는 최신 프레임을 즉시 처리한다."*
고정 Hz 스케줄이 아니라 **처리량 = 1 / 처리시간**이며, 객체 수·검출 수에 따라 매 프레임 달라진다.

```
[RGB/Depth 콜백]  →  슬롯 1칸에 최신 쌍을 덮어쓰기만 함 (오래된 프레임은 즉시 폐기)
                          ↓  (워커가 놀고 있으면 곧바로 집어감)
[워커 1개]        →  YOLO 사이드카 → 객체 10개 ISM 게이트 → 통과분 PEM → 발행 → 다시 슬롯 확인
```

* 큐를 쓰지 않는다. 큐는 곧 **지연 누적**이고, 늦게 처리된 프레임은 이미 쓸모없는 과거다.
* 폐기된 프레임은 손실이 아니다. 검출은 캡처 시각 스탬프를 달고 나가며(INV-RT-1), object_memory가 그 시각의 포즈로 정합하므로 **처리 시각이 들쭉날쭉해도 공간 정확도는 훼손되지 않는다**.
* 현재 코드의 `ApproximateTimeSynchronizer(queue_size=10, slop=0.05)`는 백로그를 쌓으므로 **큐 1 + 최신 교체 슬롯 + 전용 워커 스레드**로 바꾼다(S1 범위).
* 관측 가능성: 매 프레임 `capture→publish` 지연과 **드롭률**을 `/slam/status`·GUI에 노출한다. 자유 실행에서는 "몇 Hz로 도는가"가 성능 지표가 아니라 **지연 분포**가 지표다.

**자유 실행이 만드는 2차 효과 (설계 반영 필요)**

* **승격 임계**: active 승격에 관측 2회가 필요하므로, 처리 속도가 곧 "물체를 몇 초 봐야 지도에 올라오는가"를 결정한다. 속도 개선은 정확도가 아니라 **라이프사이클 반응성**으로 나타난다.
* 존재확률 갱신 기준은 §9.3에서 별도로 다룬다(초판의 "경과시간 기준으로 전환" 서술은 **정정됨**).

### 9.3 존재확률 갱신 기준 — 프레임 단위(증거) vs 시간 단위(존속)

> **2026-08-12 정정**: 초판은 "자유 실행이면 감점을 경과 시간 기준으로 바꿔야 한다"고 적었다. 코드 확인 결과 **틀렸다.** 미검출 감점은 프레임 기준이 옳고, Δt는 다른 항에 들어가야 한다.

**미검출 감점은 프레임 기준이 맞다 — 바꾸지 않는다.**
`core/existence.py`의 갱신은 베이즈 **측정 모델**이다.

```
미검출, 시야 안 :  r' = r(1 - P_D) / (1 - r·P_D)
미검출, 시야 밖 :  P_D = 0  →  r' = r        (무감점)
```

"한 프레임을 처리했는데 시야 안에 있어야 할 물체가 없었다"는 **관측 증거**이고,
**처리하지 않은 프레임은 어떤 증거도 만들지 않는다.** 시간의 경과 자체는 물체 유무를 말해주지 않는다.
이 저장소는 이미 같은 철학을 택했다 — 시야 밖이면 아무리 오래 지나도 무감점이며, 이 규칙이 유령 thrash를 크게 줄인 수정이었다.
감점을 시간 기준으로 돌리면 **"보지도 않고 없다고 판단"**하게 되어 이 자산을 정면으로 깨뜨린다.

**두 방식의 실제 차이** (`PD_BASE=0.5`, `R_LOST=0.3`, active r=0.9, 시야 안 연속 미검출)

| 미검출 횟수 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| r | 0.818 | 0.692 | 0.529 | 0.360 | **0.220 → lost** |

* 프레임 기준(현행): 5회 관측 필요 → 2 Hz면 2.5 s, 0.5 Hz면 10 s. **"몇 초 만에 알아채는가"가 처리 속도에 의존**한다.
* 시간 기준: 처리 속도와 무관하게 일정. 겉보기 일관성은 얻지만 **증거 없이 확신을 깎는다.**
* 속도 의존성은 버그가 아니라 올바른 동작이다 — 많이 볼수록 빨리 확신하고, 느린 기계는 더 신중하다.

**Δt가 들어가야 할 자리 3곳**

| # | 항목 | 현 상태 | 조치 |
|---|---|---|---|
| a | **survival(존속) 항** | `existence.py` 주석: *"Static objects are assumed to survive (survival probability 1), so there is no separate prediction step."* 그런데 본 시스템의 목표는 **"사라지거나 이동하는 객체 인식"** = 비정적 세계 | 예측 단계 `r ← p_S·r`, `p_S = exp(-Δt/τ)` 추가. 의미는 **"안 보는 동안 누가 치웠을 수 있다"**. 특히 현재 감점을 완전히 면제받는 `remembered`에 필요(30분 전 관측이 방금 관측과 같은 confidence). τ는 분 단위라 증거항(초 단위)과 간섭하지 않음 |
| b | `TENTATIVE_MAX_AGE = 100` **frames** | 증거가 아니라 타임아웃인데 프레임 단위 → 자유 실행에서 **50~200 s로 요동** | **초 단위로 변경** |
| c | 증거 과대계상 | 연속 프레임은 독립 시행이 아님(30 Hz로 같은 장면 = 독립 30회 아님 → 과확신). **1~2 Hz 자유 실행은 독립 가정이 잘 성립하는 영역**이라 현재는 문제 아님 | 병렬 워커로 5~10 Hz가 되면 증거율 상한 또는 Δt 가중 도입 |

`PROMOTE_HITS=2`, `MIN_OBS_LONGTERM=5`는 **관측 횟수**이므로 그대로 둔다(증거의 개수지 시간이 아니다).

**프레임 기준이라서 반드시 지켜야 할 계약**

* **검출 0건인 프레임도 빈 `Detection3DArray`를 발행해야 한다.** miss 갱신은 메시지 도착으로만 일어나므로, "못 찾았으니 발행 생략"하면 **사라진 객체를 영원히 알아채지 못한다.** 현재 노드는 무조건 발행하고 있으며(성질 유지 중), S1 전환 시 깨지기 쉬우므로 수용 기준에 포함한다.
* 부수 효과로 **안전한 실패**를 얻는다 — SAM 파이프라인이 죽으면 존재확률도 정지한다(기억이 유령처럼 증발하지 않음). 시간 기준이면 SAM이 죽은 동안 지도의 객체가 조용히 전부 사라진다.

**속도를 올릴 여지 (이미 조사돼 있는 두 건 — 새 연구 불필요)**

1. **YOLO 사이드카 1-pass화**: 현재 프롬프트별로 패스를 반복해 10객체에서 전체 시간의 약 48%를 차지한다. `multi_label` 단일 패스로 바꾸면 recall 98.9%를 유지하며 이 비용이 사실상 1회로 줄어든다.
2. **ISM 배치 진입점(`recognize_frame`) 사용**: 운영 경로가 아직 객체별 `recognize()`를 반복 호출한다. 배치 경로로 바꾸면 실측 130 ms → 32 ms.

두 가지만 적용해도 자유 실행 주기가 크게 개선된다. **먼저 이것부터 하고, 병렬 컨테이너는 그 다음**이다.

> **향후 다중 워커(컨테이너 2~3개) 메모** — 지금 범위 아님. 다만 계약에 미리 못박아 둘 것: 워커가 여러 개면 **검출이 시간순으로 도착하지 않는다**. object_memory의 `step()`은 단조 증가 시각을 가정하므로, 그때는 브리지에 **최대 지연 폭만큼의 재정렬 버퍼**(스탬프 순 정렬 후 투입)를 넣으면 코어 로직을 건드리지 않고 확장된다.
* QoS: 카메라→SAM은 로컬이므로 SensorData(BEST_EFFORT). 카메라→RTAB은 RELIABLE + exact_sync. `/sam6d/detections`·`/object_map/*`는 RELIABLE(작고 놓치면 안 됨). LAN 횡단 토픽은 KEEP_LAST(5).
* GPU: 노트북 B에서 사이드카(YOLO)와 본 노드가 같은 GPU를 쓴다. 메모리 상한 확인 필요(워크스테이션 96 GB는 참고가 안 됨).

---

## 10. 전역 보정(merge/loop closure) 시 객체맵 정합 — 놓치기 쉬운 이슈

이미 융합된 랜드마크는 융합 당시의 map 좌표에 박혀 있다. 그런데 SLAM이 loop closure나 Atlas merge를 하면 **map 원점 대비 과거 포즈가 이동**한다 → 객체맵과 SLAM 맵이 어긋난다.

| 백엔드 | 로컬라이제이션 중 맵 변형 | 대응 |
|---|---|---|
| RTAB (`Mem/IncrementalMemory=false`) | 없음(맵 고정) | 불필요 |
| hdl_localization | 없음(globalmap 고정) | 불필요 |
| **ORB3 — §6.1 순수 localization 패치 경로** | **없음**(LocalMapping 정지 → 맵 불변) | 불필요 |
| ORB3 — merge 모드(fallback) | 있음 | 필요 |

**§6.1 정정에 따라 이 이슈는 fallback 경로에서만 남는다.** 순수 localization이 정상 동작하면 세 백엔드 모두 맵이 고정이므로 이 절 전체가 불필요해진다.

설계: `rt_slam_adapter`가 전역 보정 발생 시 `/slam/map_correction`(델타 변환)을 발행하고, `rt_object_memory`가 `apply_map_correction(T)`로 모든 랜드마크에 좌측 곱을 적용한다.
**단계적 처리**: 1차 구현은 보정 이벤트를 **감지·로깅만** 하고(오차 수용), 2차에서 실제 재변환을 넣는다. 사전 구축 맵 안에서의 로컬라이제이션은 보정 폭이 작을 것으로 예상되나, 크기를 먼저 측정한 뒤 판단하는 것이 옳다.

---

## 11. GUI 설계 — 1차 로컬 뷰, 최종 Unity

**핵심 결정: GUI는 ROS 메시지를 직접 보지 않는다.** `rt_gui_bridge` 한 노드가 ROS를 구독해 **WebSocket + JSON 단일 스키마**로 내보내고, 1차 브라우저 뷰와 최종 Unity가 **같은 스키마**를 소비한다. Unity 단계에서 데이터 계층 작업이 0이 되고, 렌더링만 남는다.

```jsonc
// ws://<노트북A>:8765  — 10 Hz 스트림
{"t": 1786512345.123,
 "slam": {"backend": "orb3", "mode": "LOCALIZING", "tracking": "OK",
          "pose": {"xyz": [1.2, 0.0, -3.4], "quat_xyzw": [...]},
          "clock_offset_ms": 0.8, "sam_latency_ms": 940},
 "objects": [{"id": 16, "name": "saffron", "status": "active", "confidence": 0.98,
              "observations": 34, "missed": 3, "last_seen": 1786512344.9,
              "xyz": [...], "quat_xyzw": [...]}],
 "camera_b_jpeg": "<base64, 5Hz>"}
// 최초 1회 전송: {"map": {"grid": {...png/base64, resolution, origin}, "cloud_url": "/map/cloud.pcd"}}
```

* **1차(브라우저, 노트북 A)**: 저장된 맵 2D 상면도 위에 현재 위치·시선 방향, 객체 마커(색=status, 크기/투명도=confidence), 객체 표(이름/상태/관측수/최종관측), 상태바(tracking, 클록 offset, SAM 지연, 검출 Hz), SAM 카메라 미리보기. `make_review.py`의 HTML/SVG 관용구를 재사용해 개발량을 줄인다.
* **개발자용**: RViz2 설정 파일 동봉(`/slam/map_cloud` + `/slam/pose` + `/object_map/markers`) — 배선 정합성 눈으로 확인용, 개발량 0.
* **최종(Unity)**: 3D 맵은 스트리밍하지 않는다. 맵 아티팩트의 `cloud.pcd`(또는 메시 변환본)를 **한 번 로드**하고, 실시간으로는 위 JSON의 포즈·객체만 받는다. Unity는 WebSocket 클라이언트만 있으면 되고 ROS-TCP-Connector 의존이 없다.
* 좌표: JSON은 **ROS 우수(z-up) 그대로** 보내고 Unity 측에서 좌수계 변환을 담당한다(변환 지점을 한 곳으로 고정 — `integration/to_zup.py`의 교훈).

---

## 12. 저장소 구조와 변경 목록

**신규 워크스페이스 `realtime_ws/src/`**

| 패키지 | 내용 |
|---|---|
| `rt_msgs` | `SlamStatus.msg`, `ObjectLandmark.msg`, `ObjectMap.msg` |
| `rt_slam_adapter` | `orb3_adapter`, `rtabmap_adapter`, `hdl_adapter` — 백엔드 출력 → `/slam/pose`·`/slam/status`·`/slam/map_correction` 정규화 |
| `rt_map_manager` | 세션 상태기계, 맵 아티팩트 저장/로드/매니페스트, 백엔드 런치 기동·정상종료, `grid.pgm` 생성(gridmap.py 호출) |
| `rt_object_memory` | 기존 `object_memory_node.py`를 패키지화 + `ObjectMap`/MarkerArray 발행 + `X` 파라미터 + 맵보정 훅 |
| `rt_gui_bridge` | WebSocket JSON 서버 + 정적 웹앱(`web/`) |
| `rt_bringup` | `laptopA_mapping.launch.py`, `laptopA_localize.launch.py`, `laptopB_sam.launch.py`, `config/<site>.yaml`, RViz 설정 |

**기존 파일 수정(최소 침습)**

| 파일 | 변경 | 위험 |
|---|---|---|
| `sam6d_ws/.../sam6d_multiobject_node.py` | 출력 `PoseArray` → `Detection3DArray`(이름·점수 보존), 입력 keep-latest + 워커 스레드(자유 실행), 헤더 스탬프 캡처시각 보존, (선택)`recognize_frame` 배치 경로 | 낮음(발행부·콜백 국소 변경, 이름·점수·포즈를 이미 손에 쥐고 있음) |
| `sam6d_ws/tools/yoloworld_sidecar.py` | 프롬프트별 반복 패스 → `multi_label` 단일 패스 | 낮음(연구 완료, recall 98.9% 확인) |
| **`orbslam_ws/src/ORB_SLAM3/src/System.cc`** | Atlas 로드 시 `CreateNewMap()` → 로드된 맵을 활성화(`ChangeMap`) + `mState=LOST` (§6.1). **`LoadAtlasFromFile`이 설정된 경우에만 분기** → 매핑 동작 무변경 | 중(코어 패치지만 3줄 수준, 벤더링된 소스라 수정 가능) |
| `orbslam_ws/.../rgbd_node.cpp` | 리로컬 상태 발행(`mState`/맵 id 기반), 리로컬 성립 전 포즈 발행 억제, 정상종료 시 Atlas 저장 보장, `T_map_base` 발행 | 중(코어 접근자 필요, 빌드) |
| `hdlgraphslam_ws/src/` | `hdl_localization` 벤더링 + ROS 2 빌드 | 높음 |
| `objectmemory_ws/.../object_memory_node.py` | `rt_object_memory`로 이관 + tf2에서 `X` 조회 + 존재확률 감점을 **경과시간 기준**으로(§9.2) | 낮음(엔진 코어 로직 무수정) |
| Husky URDF (신규/갱신) | `cam_A`, `cam_B`, `velodyne` 마운트 등록(optical 프레임 포함) | 낮음, 단 §8의 2가지 함정 주의 |

**오프라인 자산은 건드리지 않는다.** `integration/run_integration.py`는 회귀 기준(ground truth)으로 계속 살려 둔다 — §13의 패리티 검증이 이것에 의존한다.

---

## 13. 검증 전략 — 하드웨어 없이 대부분 검증 가능

| 레벨 | 방법 | 합격 기준 |
|---|---|---|
| **L0 단위** | 어댑터 좌표 수학, JSON 스키마, 매니페스트 왕복, keep-latest 정책 | 기존 79~87개 테스트 무회귀 + 신규 테스트 통과 |
| **L1 재생 패리티(핵심)** | 기존 오프라인 궤적·검출을 ROS 토픽으로 재생 → 라이브 경로 통과 → 결과를 `integration/output/260804_longcircle2/fused/object_map.json`과 비교 | 생존 인스턴스 **동일**, id·name 동일, xyz 오차 < 1 cm(같은 입력이므로 사실상 일치해야 함) |
| **L2 2프로세스 bag 재생** | 두 bag을 각각 `ros2 bag play`로 실시간 배속 재생(가능하면 두 노트북에서) + 라이브 SLAM·라이브 SAM 노드 | 검출 드롭률 < 20%, 융합 랜드마크가 오프라인 대비 **집합적으로 일치**(위치 오차 < 5 cm) |
| **L3 로컬라이제이션** | 세션 X로 맵 생성 → 세션 Y로 로컬라이제이션 | ORB3: merge 성립까지 시간 측정, 포즈 산출 프레임 비율 ≥ 90%(PoC 99.1% 기준) |
| **L4 필드** | 실제 2노트북 + 2카메라 | 체크리스트: 클록 offset < 2 ms, tracking OK 비율, SAM Hz, GUI 지연 < 300 ms, 객체 승격 성공 |

L1이 이 설계의 안전망이다 — **실시간 전환이 결과를 바꾸지 않았음**을 검증된 오프라인 산출물로 증명한다.

---

## 14. 리스크 등록부

| # | 리스크 | 영향 | 완화 |
|---|---|---|---|
| R1 | ORB3 순수 localization 코어 패치가 예상대로 동작하지 않음(리로컬 성공률 저조 등) | 3단계 품질 저하 | 근본 원인은 규명됨(§6.1). 패치 실패 시 **실증된 merge 모드로 fallback**(PoC 99.1%) + 발행 게이팅. 위험도 초판 대비 **하향** |
| R2 | 두 노트북 클록(과거 6.6 s 사고) | 객체가 Δ×속도만큼 어긋난 위치에 융합 | `rt_clock_guard` 상시 감시, 20 ms 초과 시 융합 차단 + GUI 경고 |
| R3 | URDF 회전값이 실측과 다름 | 객체 번짐(29 vs 125 인스턴스 전례), 광학 프레임 혼동 시 90° 회전 | URDF를 정본으로 하되 **최초 1회 hand-eye 검산**(>2°면 URDF 갱신), optical 프레임 끝단 명시, manifest에 URDF 체크섬 기록 |
| R4 | hdl_localization 벤더링/빌드 + velodyne 좌우 반전 의혹 | hdl 경로 지연 | 3순위 배치, 계약만 먼저 확정. **`T_velo_camA` 문제는 URDF 도입으로 해소**(§8) |
| R5 | 노트북 B GPU 성능 부족 | 객체 갱신 < 1 Hz → 승격 실패 | YOLO 1-pass multi_label 개선 적용, 객체 수 축소, `PROMOTE_HITS` 재튜닝, 최악의 경우 추론 오프로드 |
| R6 | DDS discovery/대역 (LAN) | 검출 유실 | 유선 고정, 도메인 격리(0 금지), 영상 비횡단 원칙, Discovery Server 대안 |
| R7 | ORB3 전역 보정으로 객체맵 어긋남 | 정확도 저하 | §10 단계적 대응(1차 측정, 2차 재변환) |
| R8 | 맵 저장 실패(ORB3 강제종료) | 맵 소실 | 정상종료 경로 강제 + 저장 후 매니페스트 체크섬 검증 |

---

## 15. 구현 로드맵 (스토리 분해)

### 15.0 재정렬된 실행 순서 (2026-08-12 2차 결정 반영)

"SLAM 실시간부터, 백엔드는 ORB3 → RTAB → hdl, 1단계는 노트북 1대" 결정에 따라 아래 순서로 진행한다.
**단계 1(단일 노트북)** — 각 파이프라인이 라이브로 도는가:

```
A. ORB3 실시간 맵 생성 → 맵 저장            (S5a)
B. 저장 맵에서 순수 localization             (S5b, 코어 3줄 패치)
C. /slam/pose·/slam/status 계약 + 게이팅     (S5c)
D. RTAB 동일 3종 (A~C)                      (S3)
E. hdl + Velodyne 실시간 (매핑까지)          (S7a)
F. SAM-6D 실시간 단독 검증                   (S1)
G. object_memory 융합 + L1 패리티            (S2)
H. 1차 GUI                                   (S4)
```

**단계 2(2노트북 실환경)** — 둘이 같은 시계 위에서 융합되는가: 클록 동기·네트워크·필드 시험(S6), hdl localization(S7b), Unity(S8).

`rt_msgs`(S0)는 A와 병행해 먼저 고정한다 — C가 그 계약 위에 서기 때문이다.

### 15.1 스토리 상세

| S | 스토리 | 산출 | 수용 기준 | 선행 |
|---|---|---|---|---|
| **S0** | 인터페이스 계약 확정 | `rt_msgs`, JSON 스키마 문서 | 3 백엔드·GUI·Unity 요구를 모두 표현 가능함을 문서로 검토 | — |
| **S1** | SAM 노드 출력 교체 + 자유 실행 스케줄 | `sam6d_multiobject_node` 수정 (+사이드카 1-pass, ISM 배치 경로) | bag 재생에서 `Detection3DArray`의 이름·점수·캡처스탬프가 오프라인 산출과 일치, 큐 백로그 0, 사이클 시간 개선 측정, **검출 0건 프레임도 빈 메시지 발행**(§9.3) | S0 |
| **S1b** | 센서 tf 배선 (임시 static → 이후 URDF) | static_transform_publisher 런치 → Husky URDF | `camA_optical→camB_optical` 조회값이 기존 캘리브 X와 회전 2° 이내 일치. **URDF 등록은 추후**, 소비자 코드는 tf2 조회라 무변경(§8.1) | — |
| **S2** | `rt_object_memory` + L1 패리티 | 패키지 + 패리티 테스트 | 오프라인 `object_map.json`과 인스턴스 완전 일치(**미검출 감점 로직 무변경**), `TENTATIVE_MAX_AGE` 초 단위 전환, survival 항은 opt-in으로 추가 후 기본 OFF 시 무회귀 | S1,S1b |
| **S3** | RTAB 매핑/저장/로컬라이제이션 + 어댑터 | 런치 + `rt_map_manager` v1 | 맵 저장 후 재기동해 로컬라이제이션 포즈 발행 | S0 |
| **S4** | 1차 GUI(브리지 + 웹) | `rt_gui_bridge` + `web/` | 브라우저에서 맵·현재 위치·객체가 실시간 갱신, 지연 < 300 ms | S2,S3 |
| **S5a** | ORB3 실시간 매핑 → 맵 저장 | `rt_map_manager` v0(ORB3 전용), 매니페스트 | 매핑 종료 시 `.osa` + `cloud.pcd` + `manifest.json` 생성, 강제종료 시 실패를 명확히 보고, **Atlas 내 맵 개수를 매니페스트에 기록하고 >1이면 경고**(§5.3) | — |
| **S5b** | **ORB3 순수 localization 패치** | `System.cc` 3줄 패치(로드 시에만 분기) | 260714 105023 맵 + 105018 쿼리에서 **Relocalization 경로로** 포즈 산출, merge 모드 기존 결과(1985 pose·99.1%)와 대조, **매핑 동작 무회귀** | S5a |
| **S5c** | `/slam/pose`·`/slam/status` 계약 + 리로컬/VO 게이팅 | `rgbd_node.cpp` 확장(코어 `mbVO` 접근자 포함), `rt_slam_adapter` | 리로컬 성립 전 포즈 미발행, **VO 폴백 상태의 포즈도 미발행**(§6.1.1), 상태 전이가 토픽으로 관측 가능 | S0,S5b |
| **S7a** | hdl + Velodyne 실시간 매핑 | 런치 + 맵 저장(SaveMap.srv) | LiDAR 라이브 입력으로 매핑·저장 동작, bag v5/v9 함정 회피 | S3 |
| **S6** | 2노트북 실배선(클록 가드, 네트워크, 배포판 통일, 필드 시험) | `laptopA/B` 런치, 체크리스트 | L4 통과 | S4,S5c |
| **S7b** | hdl_localization 도입 | 벤더링 | 3종 전환이 런치 인자만으로 동작 | S6,S7a |
| **S8** | Unity 뷰어 | Unity 프로젝트 | 동일 JSON으로 3D 맵+포즈+객체 표시 | S4 |

**S5a→S5b가 최우선**이다(2차 결정: SLAM 실시간 우선). 특히 S5b는 **기존에 검증된 결과가 있는 유일한 작업**이다 — 260714 105023 맵 + 105018 쿼리에서 merge 모드가 1985 pose·99.1%를 냈으므로, 순수 localization 패치의 성패를 **같은 데이터로 즉시 판정**할 수 있다. 리스크 R1을 가장 이른 시점에, 가장 싸게 제거하는 순서다.

---

## 16. 다음 액션 제안

2차 결정으로 착수 지점이 확정되었다: **S5a → S5b**(ORB3 실시간 매핑·저장 → 순수 localization 패치).

### 16.1 첫 작업 상세 (S5b, 승인 대기)

**적용할 패치** — `orbslam_ws/src/ORB_SLAM3/src/System.cc`, Atlas 로드 분기 안에서만 동작:

```cpp
// 기존 (System.cc:171)
mpAtlas->CreateNewMap();                    // 활성 맵 = 빈 새 맵 -> 리로컬 후보 0개

// 변경
Map* pLoaded = <로드된 맵 중 KF 수 최대>;    // mpAtlas->GetAllMaps()
mpAtlas->ChangeMap(pLoaded);                // Atlas.h:80, 기존 API
// (Tracking 생성 이후)
mpTracker->mState = Tracking::LOST;          // Tracking.h:131, public -> 첫 프레임부터 Relocalization
```

* **매핑 경로는 건드리지 않는다** — `mStrLoadAtlasFromFile`이 비어 있으면 기존 코드 그대로 흐른다.
* 빌드 범위: `colcon build --packages-select orbslam3_core orbslam3_ros2` (conda `orbslam3` env).

**판정 데이터(이미 결과가 있는 것)**: 260714 `105023` 맵 + `105018` 쿼리.
merge 모드 기존 실측 = **1985/2004 pose(99.1%), centroid 거리 0.32 m**.

| 판정 | 기준 |
|---|---|
| 성공 | 순수 localization으로 non-zero pose 산출, 궤적이 merge 모드 결과와 유사 범위, **`.osa` 파일 불변** |
| 부분 | 포즈는 나오나 리로컬 성공률 저조 → 파라미터(리로컬 임계) 조정 후 재판정 |
| 실패 | 여전히 identity → **merge 모드 fallback 확정**, §10 전역 보정 처리를 로드맵에 복원 |

**롤백**: 패치는 3줄이며 git으로 즉시 되돌릴 수 있다. 실패해도 기존 merge 경로는 그대로 남는다.

### 16.2 진행 확인이 필요한 것

* 위 패치를 **적용하고 빌드**할까요? (`orbslam3_core` 재빌드가 필요하며 시간이 걸립니다)
* S5a(맵 저장·매니페스트)와 S5b(패치) 중 어느 것부터 볼까요 — **S5b 먼저**를 권합니다. 기존 맵(`map_260804_longcircle2`, 260714 세션)이 이미 있어 저장 단계 없이 바로 판정이 가능하고, 최대 리스크를 가장 싸게 제거합니다.
* 2단계(2노트북) 착수 전 결정 사항: **두 노트북의 ROS 2 배포판 통일**(현재 알고리즘 환경 Humble vs 수집 킷 Jazzy) — §7.5.

### 16.3 이후 대기 중인 결정

* 1차 GUI 실행 위치(SLAM 노트북 vs 원격 모니터링 PC) — S4 착수 시점에 필요.
* Husky URDF 카메라 마운트 등록 시점 — 그때까지 §8.1 임시 static tf로 진행.

---

## 부록 A. 초판(2026-08-12 오전) 대비 정정 이력

| 항목 | 초판 | 정정 |
|---|---|---|
| ORB3 순수 localization | "동작하지 않음, merge 모드만 가능" | **가능**. 원인은 `System.cc:171`의 `CreateNewMap()`으로 활성 맵이 빈 새 맵이 되어 리로컬 후보가 0개가 되는 것(§6.1). 3줄 패치로 해결, R1 위험도 하향 |
| 카메라간 extrinsic | `calib/*.npy` hand-eye 파일 | **URDF + tf2 조회**로 전환. `T_velo_camA` 미보정 문제도 동시 해소(§8) |
| SAM-6D 스케줄 | "1~3 Hz 주기" 서술 | **자유 실행**(처리 완료 직후 최신 프레임). 주기가 아니라 지연 분포가 지표(§9.2) |
| 존재확률 감점 기준 | "경과 시간 기준으로 전환" | **프레임(관측 증거) 기준 유지**가 옳음. Δt는 survival 항·타임아웃에만 도입(§9.3) |
| ORB3 전역 보정(§10) | 필수 대응 항목 | 순수 localization에서는 맵 고정이라 **불필요**, merge fallback에서만 유효 |
