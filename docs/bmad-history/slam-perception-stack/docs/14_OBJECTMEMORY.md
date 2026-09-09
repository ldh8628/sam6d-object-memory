# 14. ObjectMemory (`objectmemory_ws`)

SAM-6D의 프레임 단위 검출을 **SLAM pose로 map 좌표계에 올려, 객체 단위 장기기억**을 만든다.
"이 물체를 아까 저기서 봤다"를 유지하는 계층 — 객체 ID·존재확률·다중 인스턴스 관리.

ROS2 커스텀 메시지·Unity·SAM-6D 소스에 의존하지 않는 **독립 파이썬 모듈**로 설계돼 있다.

## 1. 구성

```
objectmemory_ws/object_memory/
├─ src/core/         models, transforms, state_machine, memory_store,
│                    existence(존재확률), fusion, features, visibility, pose_buffer
├─ src/adapters/     slam_trajectory, slam_map, sam6d_pem, bag_frame_index, cad_extents
├─ src/pipeline/     object_memory_runner.py  (오프라인 + StreamingObjectMemory)
├─ ros/              object_memory_node.py    (라이브 ROS2 노드)
├─ scripts_test/     pytest 87개 + 실행기 + 평가기
└─ reports/          Story별 결과 문서
```

## 2. 데이터 없이 바로 검증 (권장 첫 단계)

```bash
conda activate sam6d_ros_humble
cd objectmemory_ws/object_memory
python -m compileall src scripts_test
python -m pytest scripts_test -q          # 87개 중 84 passed / 3 skipped
```

> **이 레포 상태 그대로 실측 확인함(2026-07-29): `84 passed, 3 skipped`.**
> skip 3개는 실제 bag이 있어야 도는 것들이다(`check_sam_circle_bag` 계열).

**이 스택에서 데이터 없이 완전 검증이 가능한 유일한 컴포넌트다.** 새 PC 셋업 직후
코드 무결성을 확인하는 용도로 먼저 돌려라.

> ⚠ `objectmemory_ws/input/<bag>/bag/bag_0.db3` 는 **원본 PC에서도 이미 끊어진 심링크**다
> (`data_slam/rgbd_imu_sdk_bag/...` 를 가리키는데 그 디렉터리가 없다).
> 각 디렉터리의 `metadata.yaml` 은 커밋돼 있으니, 실제 bag을 구해 심링크를 다시 걸면 된다.

## 3. 오프라인 실행 (파일 3개 입력)

```bash
python scripts_test/run_object_memory.py \
  --slam-traj <ORB-SLAM3 궤적 파일> \
  --pem-dir   <SAM-6D PEM 출력 디렉터리> \
  --bag       <원본 ROS2 bag> \
  --source-slam-id orbslam3_noimu
```

입력 3종이 합쳐져 `T_map_obj`(map 좌표계 객체 자세)가 나온다.

주요 파라미터:

| 인자 | 기본 | 의미 |
|---|---|---|
| `--time-tolerance` | 0.05 | pose↔검출 시간정합 허용오차(초) |
| `--assoc-trans-gate` | 0.20 | 인스턴스 연관 반경(m) |
| `--assoc-rot-gate` | -1.0 | 회전 게이트(음수=끔) |
| `--r-init / --r-promote / --r-lost / --r-retire` | 0.7 / 0.6 / 0.3 / 0.05 | 존재확률 임계 |
| `--pd-base` | 0.6 | 검출확률 P_D |
| `--clutter-ratio` | 0.1 | 클러터 비율 |
| `--tentative-max-age` | 100 | 잠정 트랙 최대 수명 |
| `--min-obs-longterm` | 5 | 장기기억 승격 최소 관측수 |
| `--legacy-confidence` | off | 구(舊) 히트카운터 방식으로 되돌림 |

관련 실행기: `run_realtime_object_memory.py`(스트리밍), `run_c1_map.py`(4 bag 맵),
`run_object_memory_extrinsic.py`(2카메라 extrinsic 적용), `visualize_object_memory.py`.

## 4. 라이브 ROS2 노드

```bash
python ros/object_memory_node.py
```

| 방향 | 토픽(파라미터) | 타입 |
|---|---|---|
| in | `pose_topic` = `/orbslam3/pose` | `geometry_msgs/PoseStamped` (~30 Hz) |
| in | `detections_topic` = `/sam6d/detections` | `vision_msgs/Detection3DArray` (수 Hz, **늦게 도착**) |
| in | `caminfo_topic` | `sensor_msgs/CameraInfo` (1회) |
| out | `~/landmarks` | `geometry_msgs/PoseArray` (map frame) |

핵심 규칙 2가지:

1. **`MultiThreadedExecutor` 필수.** 단일 스레드면 멈춘다.
2. **검출 메시지의 `header.stamp` 는 SAM이 처리한 프레임의 "촬영 시각"이어야 한다.**
   노드는 그 과거 시점의 카메라 pose를 조회한다(시간정합). 이게 늦게 도착하는 SAM 결과를
   SLAM-보정된 위치에 올리는 방식이며, 오프라인 경로와 **알고리즘이 바이트 단위로 동일**함이
   검증돼 있다(`scripts_test/eval_async.py` PARITY).

### ⚠ SAM-6D 쪽에 필요한 단 하나의 수정

`sam6d_ws/src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py` 는 지금 `geometry_msgs/PoseArray`
를 publish 하는데 **객체 ID가 사라진다**(이름이 오버레이에만 있음).
publish 블록에서 이미 name/score/pose를 다 들고 있으므로, 각각을
`Detection3D` + `ObjectHypothesisWithPose` 로 감싸 `Detection3DArray` 로 바꾸면 된다.
**커스텀 인터페이스 패키지는 필요 없다**(`vision_msgs` 가 이미 env에 있음).

## 5. 알고리즘 요약

- **존재확률 r (Bernoulli)**: 히트 카운터가 아니라 **검출별 품질 가능도비**로 갱신한다
  (logit + Σwφ; score·depth·residual 가중). 시야 밖(FOV 밖)에서는 감점하지 않는다.
- **다중 인스턴스**: 같은 이름의 물체가 여러 개일 수 있다.
  `live_ids_by_name` + spawn-on-miss, 연관 반경 0.20 m, 회전 게이트 off.
- **생명주기**: tentative → confirmed(장기기억) → lost → retire.

## 6. 검증된 결과 / 남은 이슈

| 데이터 | 결과 |
|---|---|
| `SAM_occlusion` | 객체 3개 lifecycle 재현 |
| `SAM_loop1` | saffron 3→96 관측, thrash 대폭 감소 |
| 4 bag 품질가중 도입 후 | 카메라 겹침 2→0, 유령 수명 11→3, circle 과승격 해소 |
| 4 bag 실시간(C1) | choco 0.92 안정, Rabbit remembered, dropped 0 |
| 260714 pair4 | 21/21 융합 성공 |

남은 이슈:
- **260714 2카메라 간 extrinsic 미보정** → 객체 맵이 흩어진다(가장 큰 미해결 항목).
- `SAM_loop2` choco는 **PEM pose 자체가 불안정**(상류 문제).
- 저관측 2차 인스턴스가 과승격되는 경향 → `PROMOTE_HITS` 상향이 다음 후보.
- `SAM_loop1` choco가 품질가중 도입 후 강등되는 경계 케이스(가중 튜닝 대상).
- 진짜 stride-1 실시간 평가는 SAM 결과 재생성이 필요하다.

## 7. 참고 문서 (레포 내)

- `objectmemory_ws/object_memory-baseline-reconstruction-guide-2026-07-01.md`
- `objectmemory_ws/object_memory/reports/story*.md` (Story 0/1/2/4/5)
- `objectmemory_ws/object_memory/reports/realtime_async_design.md`
- `objectmemory_ws/output/*/` 실행 리포트 md (시각화 png/gif는 용량상 제외)
