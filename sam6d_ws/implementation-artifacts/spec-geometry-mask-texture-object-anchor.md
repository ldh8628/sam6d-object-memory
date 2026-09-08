---
title: '기하–Mask–Texture 후보 선택 및 SLAM Object Anchor 통합'
type: 'feature'
created: '2026-08-24'
status: 'done'
baseline_commit: 'NO_VCS'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** PEM이 geometry top-1을 그대로 fine refinement에 넘겨 silhouette·texture 증거와 후보 수렴을 선택에 반영하지 못하고, 반복 관측한 정적 객체를 SLAM map에 안전하게 고정·재사용하는 런타임 상태기가 없다.

**Approach:** 기존 ISM/depth 전처리와 6000→300 생성·기하 채점을 보존한 채 300개를 Mask→Texture→symmetry-aware 수렴 순으로 필터링해 실제 후보 하나를 refine하고, 유효 SLAM 관측으로 등록·해제되는 세션 한정 Object Anchor와 두 출력 모드를 통합한다.

## Boundaries & Constraints

**Always:** 원본 ISM mask는 비교용으로 보존하고 depth 교집합은 3D 생성에만 사용한다. 6000/300 후보와 3점 잔차, 196 관측점–1024 CAD점 geometry score 및 proposal identity를 보존한다. Mask/Texture 경계값은 포함(`>=`)하고 기본값은 각각 0.420998/0.449562다. 수렴군은 대칭 인식 20°/25 mm이며 최대 군집, geometry-rank tie-break, 50% 점유 조건으로 실제 geometry 최상 후보를 고른다. Fine 결과가 실패하거나 재검증을 통과하지 못하면 Detection3D를 거부한다. Anchor는 유효한 100 ms 이내 TRACKING_OK pose로 얻은 최근 유효 관측 20개 중 16개 수렴 시 medoid로 등록하고, map 변경/프로세스 재시작 때 초기화하며 5회 중 4회 pose 불일치 시 해제한다. 등록 Anchor는 낮은 IoU만으로 폐기하지 않는다.

**Ask First:** 기존 6000/300 생성 수치나 임계값을 자동 조정하는 변경, 저장된 Anchor 복원, 자동 map 전환, 외부 메시지 패키지 의존성 추가.

**Never:** geometry/texture 결합 점수나 평균 pose를 만들지 않는다. 정상/오인식 사례를 저장·온라인 학습하지 않는다. 비추적 manifest/checksum을 수정·삭제하지 않는다. Anchor로 ISM bbox를 보정하지 않는다. 기존 91.75%를 전체 성공률로 재사용하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 후보 선택 | Mask·Texture 통과 후보와 지배 수렴군 | 군집 내 geometry 최상 실제 후보를 refine | 단계별 전멸/수렴 부족 사유로 Detection3D 거부 |
| Fine 검증 | 선택된 coarse pose | 8192점 Mask·Texture 재검증 통과 pose | Fine 또는 최종 검증 실패 시 coarse fallback 금지 |
| Anchor 등록 | 같은 map의 유효 관측 20개 | 16개 이상 수렴군 medoid 등록 | 무효 SLAM/누락 frame은 분모에서 제외 |
| Anchor 해제 | 등록 후 유효 shadow 검증 5개 | 4개 pose 불일치 시 해제 | 낮은 IoU만으로 해제하지 않음 |
| Anchor A/B | ISM 연계 또는 FOV 조건 | camera frame 기존 pose 형식 출력 | SLAM 무효면 일반 필터 통과 결과만 출력 |

</frozen-after-approval>

## Code Map

- `sam6d_master/SAM-6D/Pose_Estimation_Model/run_inference_custom.py` -- 원본 ISM mask와 3D용 depth mask 분리, projection용 8192점 입력.
- `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py` -- 300 후보 Mask/Texture/수렴 선택과 fine 재검증.
- `sam6d_master/SAM-6D/Pose_Estimation_Model/model/{coarse,fine}_point_matching.py` -- 선택 결과와 refinement 검증 상태 전달.
- `realtime/{verify_config,sam6d_core,slam_pose_memory}.py` -- 공통 설정, 거부/진단, 동적 Anchor 상태·projection.
- `realtime/{shm_channel,sam6d_infer,sam6d_receiver_node}.py` -- timestamp 일치 SLAM pose를 split 추론에 전달하고 기존 Detection3D 발행 유지.
- `/home/etri/orbslam3_ws/src/orbslam3_ros2/{src/rgbd_node.cpp,launch/orb_slam.launch.py}` -- Atlas/localization/map_id와 TRACKING_OK map pose 노출.
- `tests/` -- 후보 경계·거부·대칭 수렴 및 Anchor 등록/해제/A-B 회귀.

## Tasks & Acceptance

**Execution:**
- [x] PEM 입력과 후보 선택/검증을 순차 정책으로 변경하고 8192점 proxy rendering을 적용한다.
- [x] core 출력 거부·진단과 동적 Anchor 등록/해제/A-B 출력을 연결한다.
- [x] split IPC/ROS 및 ORB-SLAM wrapper에 timestamp·map_id·localization 설정을 노출한다.
- [x] 순수 단위/회귀 테스트와 설정 예제를 갱신한다.

**Acceptance Criteria:**
- Given 동일 seed와 입력, when 후보 생성 회귀를 실행하면 6000 잔차와 top-300 identity/value가 기존 기준과 일치한다.
- Given 임계값 경계와 대칭 후보, when 순차 선택하면 경계값은 통과하고 단계별 실패 사유 및 실제 geometry 최상 후보가 정확하다.
- Given 20개의 유효 map 관측, when 16개/15개가 수렴하면 각각 Anchor 등록/미등록이며 map 변경과 4/5 불일치가 초기화/해제를 수행한다.
- Given 등록 Anchor와 ISM 누락, when 두 모드를 실행하면 `ism_associated`만 억제되고 `fov_always`는 FOV 조건에서 출력한다.

## Spec Change Log

## Design Notes

Anchor pose 거리와 후보 pose 거리는 같은 손으로 선언한 객체별 symmetry table을 사용한다. Anchor 출력은 pipeline shadow 결과와 별도 authority이며 `pose_source`, `map_id`, 상태와 거부 사유를 진단에 남긴다.

## Verification

**Commands:**
- `pytest -q` -- 관련 단위·회귀 테스트 통과.
- `python -m py_compile realtime/*.py` -- Python 통합 경로 문법 정상.
- `colcon build --packages-select orbslam3_ros2` -- wrapper 빌드 성공(ROS 환경 사용 가능 시).

## Suggested Review Order

**후보 선택과 최종 검증**

- 300개 후보의 Mask→Texture→대칭 수렴 정책과 실제 후보 선택을 정의한다.
  [`model_utils.py:392`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L392)

- 8192 CAD점의 후보별 adaptive splat silhouette와 진단값을 계산한다.
  [`model_utils.py:266`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L266)

- Mask 생존 후보만 chunk 단위 Texture 계산 후 선택 결과를 coarse에 연결한다.
  [`model_utils.py:761`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L761)

- Fine pose를 같은 Mask·Texture 임계값으로 재검증하며 coarse fallback을 금지한다.
  [`model_utils.py:1165`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L1165)

- 원본 ISM mask와 depth 전용 mask를 분리하고 8192점을 전달한다.
  [`run_inference_custom.py:200`](../sam6d_master/SAM-6D/Pose_Estimation_Model/run_inference_custom.py#L200)

**Anchor 상태기와 출력 권한**

- 유효 관측 20개 등록, medoid, 4/5 해제와 A/B 출력을 캡슐화한다.
  [`slam_pose_memory.py:89`](../realtime/slam_pose_memory.py#L89)

- SAM-6D shadow 결과로 Anchor를 갱신하고 출력 pose source를 선택한다.
  [`sam6d_core.py:375`](../realtime/sam6d_core.py#L375)

- 고정 임계값과 대칭 선언을 단일 공통 설정으로 정규화한다.
  [`verify_config.py:45`](../realtime/verify_config.py#L45)

**SLAM 시간·Map 경계**

- RGB를 보류해 동일 timestamp pose만 연결하고 Map 변경 시 캐시를 폐기한다.
  [`sam6d_receiver_node.py:169`](../realtime/sam6d_receiver_node.py#L169)

- timestamped SE(3)와 UTF-8 map_id를 seqlock IPC로 함께 전달한다.
  [`shm_channel.py:27`](../realtime/shm_channel.py#L27)

- ORB 실제 tracking `OK`에서만 같은 시각의 map pose를 발행한다.
  [`rgbd_node.cpp:324`](../../../orbslam3_ws/src/orbslam3_ros2/src/rgbd_node.cpp#L324)

- Atlas 경로와 localization-only 설정을 검증·materialize한다.
  [`orb_slam.launch.py:234`](../../../orbslam3_ws/src/orbslam3_ros2/launch/orb_slam.launch.py#L234)

**회귀와 실험 보고**

- 기존 6000/300 proposal identity와 수치를 frozen legacy 계산과 비교한다.
  [`test_pem_candidate_generation_regression.py:1`](../tests/test_pem_candidate_generation_regression.py#L1)

- 후보 경계값·단계별 거부·대칭 선택·Fine 실패를 검증한다.
  [`test_pem_candidate_verification.py:52`](../tests/test_pem_candidate_verification.py#L52)

- Anchor 등록·초기화·해제·낮은 IoU·A/B 동작을 검증한다.
  [`test_slam_pose_memory.py:43`](../tests/test_slam_pose_memory.py#L43)

- 동일 입력 두 모드의 coverage·오출력·안정성·ISM 복구를 분리 보고한다.
  [`compare_anchor_modes.py:93`](../tools/compare_anchor_modes.py#L93)
