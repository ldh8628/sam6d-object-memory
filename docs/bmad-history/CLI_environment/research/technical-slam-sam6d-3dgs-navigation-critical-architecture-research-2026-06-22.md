# SLAM + SAM-6D 기반 Object Persistence 기술 연구

**Date:** 2026-06-22  
**Language:** Korean  
**Target Runtime:** ROS2 Jazzy + RGB-D + SAM-6D + selected SLAM runtime  
**Assumption:** LI-Init 완료, Fast-LIO2 완료, Runtime SLAM 선택 완료, Coordinate Frame 설계 완료  
**Priority Change:** Unity, Nav2, 3DGS, SLAM 비교는 후순위. 본 보고서의 중심은 **Object Persistence**이다.

## 1. Executive Judgment

현재 연구 질문은 다음 한 문장으로 정리된다.

> SLAM이 현재 위치를 제공하고 SAM-6D가 객체 pose를 제공할 때, 객체가 시야에서 사라졌다가 수 분 뒤 또는 loop 주행 후 다시 등장하면 동일 Object ID로 인식할 수 있는가?

결론은 조건부 가능이다. `SLAM pose + SAM-6D pose`만 곱해서 `T_map_object`를 저장하는 구조는 짧은 데모에서는 동작할 수 있지만, 장기 Object Persistence라고 부르기에는 부족하다. 장기 persistence의 핵심은 pose 계산이 아니라 **object landmark memory + data association + lifecycle + loop closure 대응 + ground truth 기반 평가**이다.

석사 논문 주제로는 가능성이 있다. 단, "SLAM pose와 SAM-6D pose를 연결했다"는 수준은 단순 엔지니어링이다. 논문 가치가 생기는 지점은 다음 중 하나를 정량적으로 증명할 때다.

1. SAM-6D 단일 프레임 pose를 object landmark로 승격했을 때 장시간 미관측 후 ID consistency가 유지되는가.
2. Loop closure 또는 map frame correction 이후에도 기존 object landmark ID와 pose가 안정적으로 보정되는가.
3. 동일 외형 객체 2-5개, 부분 가림, 조명 변화, 객체 이동이 있을 때 ID switch를 줄이는 data association 방법을 제안할 수 있는가.
4. RGB-D + SLAM + SAM-6D만으로 재현 가능한 Object Persistence benchmark와 평가 프로토콜을 만들 수 있는가.

실현 가능성 점수는 MVP 기준 **7/10**, 연구 논문 기준 **6/10**이다. 실패 리스크는 SAM-6D runtime보다 data association과 데이터셋/GT 설계에 더 크다.

## 2. Scope Reframing

이미 완료된 것으로 보는 항목:

- LI-Init: LiDAR-IMU extrinsic/time initialization 완료
- Fast-LIO2: pseudo-GT/reference trajectory 생성 가능
- Runtime SLAM: 운용 SLAM 선택 완료
- Coordinate Frame: `map`, `odom`, `base_link`, `camera_*`, `object_*` frame 계약 완료

따라서 본 문서에서 우선순위를 낮추는 항목:

- ATE/RPE 중심 SLAM 비교
- Nav2 goal sending
- Unity visualization
- 3DGS map alignment
- SLAM 알고리즘 선택 논쟁

본 문서에서 우선순위를 높이는 항목:

- Object Landmark
- Object Observation
- Data Association
- Object Database
- Loop Closure Handling
- Object Lifecycle
- Object Re-identification
- Object Persistence Dataset / Ground Truth / Metrics

## 3. 핵심 판단: SLAM Pose + SAM-6D Pose만으로 가능한가

### 3.1 수식상 가능한 최소 형태

SAM-6D가 camera frame 기준 object pose를 제공한다고 하면:

```text
T_map_object(t) = T_map_base(t) * T_base_camera * T_camera_object_sam6d(t)
```

또는 SLAM이 camera pose를 직접 제공하면:

```text
T_map_object(t) = T_map_camera(t) * T_camera_object_sam6d(t)
```

이 값을 저장하면 "객체의 world pose memory"는 만들 수 있다. 같은 객체가 나중에 다시 관측되었을 때 새 `T_map_object`가 기존 landmark 근처에 있으면 같은 ID로 연결할 수 있다.

### 3.2 가능한 조건

`SLAM + SAM-6D pose only` 방식이 가능한 조건은 좁다.

| 조건 | 필요 수준 | 이유 |
|---|---:|---|
| 객체가 정지 | 필수 | 위치 기반 association이 성립하려면 실제 object pose가 유지되어야 한다. |
| 객체 간 간격 | 최소 0.5-1.0 m 이상 권장 | SAM-6D/SLAM pose noise와 객체 크기보다 association gate가 커야 한다. |
| 동일 외형 객체 수 | 1개 또는 충분히 떨어진 소수 | 외형이 같으면 pose proximity 외에는 구분 근거가 약하다. |
| SLAM drift | object gate보다 작아야 함 | 수 분 뒤 drift가 20-50 cm를 넘으면 nearest landmark가 흔들린다. |
| Loop closure 처리 | map correction 반영 가능해야 함 | 이전 landmark pose가 과거 map 기준으로 남으면 현재 pose와 비교가 깨진다. |
| SAM-6D pose 안정성 | translational jitter가 gate보다 작아야 함 | frame-wise pose estimator라 jitter가 누적 association 오류를 만든다. |
| RGB-D time sync | SLAM pose와 RGB-D timestamp 차이가 작아야 함 | 이동 중 시간 오차는 object world pose 오차로 바로 변환된다. |

### 3.3 불가능하거나 약한 조건

아래 조건에서는 pose only persistence가 쉽게 실패한다.

- 동일 외형 객체가 가까이 여러 개 있음: nearest-neighbor association이 ID switch를 만든다.
- 객체가 이동함: 같은 ID를 유지해야 할지 새 landmark로 봐야 할지 pose만으로 결정할 수 없다.
- Loop closure 후 `map -> odom` 또는 graph pose가 크게 보정됨: 저장된 object pose도 같이 보정해야 한다.
- 부분 가림: SAM-6D pose가 튀거나 detection dropout이 발생해 duplicate landmark가 생긴다.
- 장시간 미관측: 환경 변화, drift, 조명 변화, viewpoint 변화가 누적된다.
- false positive: SAM-6D가 잘못 낸 pose가 object DB에 confirmed landmark로 승격될 수 있다.

### 3.4 결론

`SLAM Pose + SAM-6D Pose`는 Object Persistence의 **measurement source**일 뿐이다. Persistence를 구현하려면 최소한 다음 상태가 필요하다.

- 관측 단위: `ObjectObservation`
- 영속 객체 단위: `ObjectLandmark`
- 관측-랜드마크 연결: `DataAssociation`
- 시간에 따른 상태 전이: `Lifecycle`
- 평가 가능 로그: observation history + association decision log

## 4. Object Persistence MVP 설계

목표는 추가 센서 없이, RGB-D + SAM-6D + 기존 SLAM을 유지하면서 가장 적은 개발량으로 persistence를 검증하는 것이다.

### 4.1 MVP 원칙

- SAM-6D 내부 모델은 수정하지 않는다.
- SLAM backend도 수정하지 않는다.
- ROS2 Jazzy node 2-4개와 SQLite/JSONL 로그로 시작한다.
- 처음에는 factor graph를 만들지 않고, `map` frame object landmark store로 시작한다.
- 논문용 확장은 association model과 loop closure handling에서 만든다.

### 4.2 최소 ROS2 구성

```text
RGB-D topics + CameraInfo
        |
        v
SAM-6D node
        |
        | Object pose in camera optical frame
        v
object_observation_node
        |
        | ObjectObservation in map frame
        v
object_persistence_node
        |
        +--> object_database.sqlite / observations.jsonl
        |
        +--> /object_landmarks
        +--> /object_events
        +--> /tf object_<uuid> optional
```

### 4.3 ObjectObservation

SAM-6D 결과를 그대로 landmark로 저장하지 말고, 매 관측을 immutable observation으로 저장한다.

권장 필드:

```text
observation_id
timestamp
frame_id_source
object_class_or_template_id
sam6d_score
T_camera_object
T_map_camera_at_observation
T_map_object
pose_covariance_estimate
rgb_crop_path optional
depth_crop_path optional
mask_or_bbox optional
point_count_depth_valid
viewpoint_vector
source_bag_name
```

### 4.4 ObjectLandmark

랜드마크는 "현재 믿고 있는 물리 객체 instance"이다.

권장 필드:

```text
object_uuid
class_or_template_id
pose_mean_map
pose_covariance
extent_or_bbox_3d
descriptor_rgb optional
descriptor_depth optional
first_seen
last_seen
observation_count
miss_count
state: candidate | confirmed | lost | moved | retired
static_probability
map_version_or_slam_epoch
last_association_score
```

### 4.5 Data Association MVP

첫 MVP는 너무 복잡하게 시작하지 않는다.

Association score:

```text
S = w_pose * pose_gate_score
  + w_class * class_match
  + w_size * size_or_extent_match
  + w_view * viewpoint_compatibility
  + w_score * sam6d_confidence
```

권장 gate:

- class/template ID가 다르면 기본적으로 다른 객체로 처리
- translation Mahalanobis distance 또는 Euclidean distance 사용
- rotation distance는 대칭 객체에서는 낮은 가중치
- 같은 frame에서 하나의 observation은 하나의 landmark에만 연결
- Hungarian matching 또는 greedy matching으로 시작
- association score가 threshold 이하이면 새 candidate landmark 생성

MVP threshold 예시:

| 항목 | 초기값 |
|---|---:|
| static object translation gate | 0.25-0.50 m |
| rotation gate | 20-45 deg |
| candidate -> confirmed | 3회 이상 관측 |
| confirmed -> lost | 10-30초 또는 N frame 미관측 |
| lost -> confirmed 복귀 | 기존 landmark와 gate 통과 |
| moved 판단 | 같은 appearance/template인데 pose 차이가 0.75-1.5 m 이상 |

### 4.6 Lifecycle

```text
candidate: 처음 발견됨. false positive 가능성이 높음.
confirmed: 여러 관측에서 안정적으로 확인됨.
lost: 시야에서 사라졌지만 DB에는 유지.
reacquired: lost 상태에서 동일 ID로 다시 연결됨.
moved: 같은 물체로 보이나 위치가 크게 바뀜.
retired: 장시간 재등장하지 않거나 실험 종료 후 inactive.
```

논문/평가에서는 `reacquired` 이벤트가 가장 중요하다. Object Persistence의 성능은 "계속 보이는 tracking"보다 "사라졌다가 다시 등장한 뒤 같은 ID로 복귀하는가"로 측정해야 한다.

## 5. 구현 난이도 평가

| Module | 개발 난이도 | 예상 기간 | 실패 가능성 | 핵심 리스크 |
|---|---:|---:|---:|---|
| ObjectObservation Store | 낮음 | 1-2일 | 10% | 메시지 필드/시간 동기 누락 |
| TF-based pose projection | 낮음-중 | 1-3일 | 20% | timestamp extrapolation, optical frame 축 |
| Object Database | 낮음 | 1-2일 | 10% | schema 변경 관리 |
| Landmark State Update | 중 | 2-4일 | 25% | covariance 부재, jitter 누적 |
| Data Association MVP | 중 | 3-7일 | 35% | gate tuning, 동일 객체 ambiguity |
| Object Lifecycle | 중 | 2-4일 | 25% | false positive confirmed 승격 |
| Evaluation Logger | 낮음-중 | 2-4일 | 20% | GT와 prediction 시간 정렬 |
| Loop Closure Handling | 중-상 | 1-2주 | 45% | SLAM graph correction을 object DB에 반영하는 API 부족 |
| Object Re-identification | 상 | 2-4주 | 55% | descriptor 품질, 동일 외형 객체 구분 |
| Moved Object Handling | 상 | 1-3주 | 50% | 같은 ID 유지 vs 새 ID 생성 기준 |
| Dataset/GT Pipeline | 중-상 | 1-3주 | 40% | 수동 라벨 비용, 평가 일관성 |
| ROS2 Integration Hardening | 중 | 1-2주 | 30% | QoS, bag replay clock, GPU latency |

가장 작은 성공 경로는 `ObjectObservation Store -> Data Association MVP -> Lifecycle -> Evaluation Logger`이다. Loop closure와 re-ID는 2차 연구 확장으로 두는 것이 현실적이다.

## 6. Object Persistence 논문 가치 평가

### 6.1 단순 엔지니어링인 경우

다음 수준에 머물면 논문 기여가 약하다.

- `T_map_object = T_map_camera * T_camera_object`를 계산해 DB에 저장
- nearest-neighbor distance threshold로 같은 객체 판단
- 한두 개 객체 데모만 제시
- 정량 metric 없이 RViz/Unity에서 ID가 유지되는 화면만 제시

이 경우 구현 가치는 있지만 연구 가치는 낮다.

### 6.2 석사 논문으로 가능한 경우

다음 세 가지를 충족하면 석사 논문 주제로 가능하다.

1. **문제 정의:** "SAM-6D 기반 6D object pose observation을 SLAM map frame의 persistent object landmark로 유지하는 문제"를 명확히 정의한다.
2. **방법:** pose/geometry/appearance/temporal lifecycle을 결합한 association 방법을 제안한다. 최소 baseline은 pose-only, 개선안은 pose + depth geometry + appearance descriptor이다.
3. **평가:** A-H 시나리오 dataset을 직접 수집하고 ID Switch, ID Consistency, Re-ID Accuracy, Track Recall/Precision, Pose Error after Reacquisition을 정량 평가한다.

### 6.3 연구 기여 후보

| 후보 | 논문 가치 | 난이도 | 평가 가능성 |
|---|---:|---:|---:|
| Pose-only object landmark persistence baseline | 낮음-중 | 낮음 | 높음 |
| SAM-6D confidence/depth residual 기반 covariance estimation | 중 | 중 | 중 |
| 장시간 미관측 후 re-acquisition data association | 중-상 | 중 | 높음 |
| Loop closure-aware object landmark correction | 상 | 중-상 | 중 |
| 동일 외형 다중 객체 re-ID | 상 | 상 | 중 |
| 이동 객체 lifecycle: same instance moved vs new object | 상 | 상 | 중 |
| RGB-D/SAM-6D Object Persistence benchmark | 중-상 | 중 | 높음 |

추천 논문 제목 방향:

```text
Long-Term Object Persistence from Zero-Shot 6D Object Pose Observations in RGB-D SLAM
```

또는 한국어:

```text
RGB-D SLAM과 SAM-6D 기반 장기 객체 영속성 지도화 및 재식별 평가
```

## 7. 검증용 데이터 수집 설계

현재 단순 SLAM 데이터만으로는 Object Persistence 평가에 충분하지 않다. 기존 데이터는 robot trajectory, loop/revisit, SLAM map 품질을 평가하는 데는 유용하지만, object ID persistence를 평가하려면 객체 등장/사라짐/재등장, 동일 외형 객체, 객체 이동, 부분 가림, 조명 변화, 장시간 미관측이 의도적으로 포함되어야 한다.

### 7.1 공통 수집 규칙

- 모든 bag은 `/clock` replay 가능하도록 기록한다.
- RGB, depth, camera_info, TF, SLAM pose, IMU, LiDAR/Fast-LIO2 reference를 함께 기록한다.
- SAM-6D raw output과 persistence output을 별도 topic으로 저장한다.
- 각 객체에는 실험자가 알 수 있는 physical ID를 부여한다. 예: `milk_01`, `milk_02`, `box_a`.
- 가능하면 객체 바닥 위치를 tape marker로 표시한다.
- 각 시나리오는 최소 3회 반복한다.
- GT 라벨은 frame 단위 전체 라벨보다 event + keyframe 라벨 방식으로 시작한다.

### 7.2 Scenario A: 객체 등장 -> 사라짐 -> 재등장

| 항목 | 정의 |
|---|---|
| 수집 방법 | 카메라가 객체를 5-10초 관측한 뒤 객체가 시야 밖이 되도록 이동하거나 가림막을 넣고, 30-90초 후 같은 객체를 다시 관측한다. |
| 필요한 길이 | 1-2분/sequence, 객체 1-3개, 3회 반복 |
| 평가 가능 항목 | reacquisition ID 유지, duplicate landmark 생성, lost->confirmed transition |
| 성공 조건 | 재등장 첫 3회 유효 관측 안에 기존 ID로 연결, ID switch 0, duplicate landmark 0 또는 자동 merge |

### 7.3 Scenario B: 객체를 보고 멀리 이동 후 복귀

| 항목 | 정의 |
|---|---|
| 수집 방법 | 객체를 가까이서 관측하고 5-15 m 또는 가능한 최대 거리까지 이동한 뒤 다른 구역을 보고 복귀한다. |
| 필요한 길이 | 2-4분/sequence |
| 평가 가능 항목 | SLAM drift에 대한 pose gate 안정성, 장시간 lost 상태 유지, 복귀 시 ID consistency |
| 성공 조건 | 복귀 후 같은 physical object에 같은 object_uuid 부여, pose error가 사전 정의 gate 내 유지 |

### 7.4 Scenario C: Loop Closure 발생

| 항목 | 정의 |
|---|---|
| 수집 방법 | 객체를 보고 한 바퀴 loop를 돌아 같은 위치를 다시 지나간다. RTAB-Map/Fast-LIO2/선택 SLAM에서 loop 또는 graph correction 로그를 보존한다. |
| 필요한 길이 | 2-5분/sequence, 최소 1 loop closure event |
| 평가 가능 항목 | loop 전후 object landmark pose correction, ID 유지, duplicate 생성 여부 |
| 성공 조건 | loop 이후 기존 ID 유지, loop correction 전후 object pose jump가 DB에 기록되거나 보정됨, 동일 객체 duplicate 없음 |

### 7.5 Scenario D: 동일 외형 객체 2-5개

| 항목 | 정의 |
|---|---|
| 수집 방법 | 같은 물체 2-5개를 0.5 m, 1.0 m, 2.0 m 간격으로 배치하고 여러 각도에서 관측한다. |
| 필요한 길이 | 2-3분/sequence, 간격 조건별 3회 |
| 평가 가능 항목 | ID switch, association ambiguity, nearest-neighbor 실패율 |
| 성공 조건 | 객체 간 1 m 이상 조건에서 IDF1 0.8 이상, ID switch/object/lap 0.2 이하 |

### 7.6 Scenario E: 객체 위치 이동

| 항목 | 정의 |
|---|---|
| 수집 방법 | 객체를 관측한 뒤 시야 밖에서 0.5-2.0 m 이동시키고 다시 관측한다. |
| 필요한 길이 | 2분/sequence, 이동 거리 3조건 |
| 평가 가능 항목 | moved state 검출, same physical ID 유지 여부, old pose invalidation |
| 성공 조건 | 이동 객체를 duplicate static landmark로 방치하지 않음. `moved` 이벤트 또는 same object pose update 발생 |

### 7.7 Scenario F: 부분 가림

| 항목 | 정의 |
|---|---|
| 수집 방법 | 객체의 25%, 50%, 75%를 가리는 가림막을 사용하고 관측 각도를 바꾼다. |
| 필요한 길이 | 1-2분/sequence, occlusion level별 3회 |
| 평가 가능 항목 | SAM-6D dropout, false pose, lifecycle robustness, association confidence |
| 성공 조건 | 50% 가림에서 ID switch 0 또는 낮은 false confirmation, 75% 가림에서는 unknown 처리 허용 |

### 7.8 Scenario G: 조명 변화

| 항목 | 정의 |
|---|---|
| 수집 방법 | 동일 객체를 정상 조명, 어두운 조명, 측면 조명, 반사/그림자 조건에서 반복 관측한다. |
| 필요한 길이 | 1-2분/condition |
| 평가 가능 항목 | appearance descriptor 안정성, SAM-6D confidence 변화, pose-only fallback |
| 성공 조건 | 조명 변화 후에도 같은 위치 정지 객체는 같은 ID로 복귀. confidence 하락 시 false new ID를 만들지 않음 |

### 7.9 Scenario H: 장시간 미관측

| 항목 | 정의 |
|---|---|
| 수집 방법 | 객체를 보고 5-10분 동안 다른 구역을 주행하거나 정지/재시작을 포함한 뒤 다시 관측한다. |
| 필요한 길이 | 5-10분/sequence |
| 평가 가능 항목 | long-term lost memory, DB persistence, timestamp/map version handling |
| 성공 조건 | 재등장 시 기존 ID 유지, 시스템 재시작 후에도 object DB에서 동일 ID 복구 가능 |

## 8. Ground Truth 생성 방법

Object Persistence GT는 trajectory GT와 다르다. 필요한 것은 "각 관측이 어떤 physical object에 속하는가"와 "그 object의 실제 상태가 무엇인가"이다.

### 8.1 최소 GT

가장 현실적인 최소 GT는 event/keyframe 기반이다.

```text
sequence_id
timestamp_start
timestamp_end
physical_object_id
visible: true/false
state: present | occluded | moved | removed
approx_pose_map optional
bbox_2d optional
comment
```

수동으로 모든 frame을 라벨링하지 않고, 각 이벤트 구간과 keyframe만 라벨링한다. Persistence 평가에는 모든 pixel mask보다 ID event GT가 더 중요하다.

### 8.2 권장 GT 단계

| 단계 | 방법 | 비용 | 정확도 | 용도 |
|---|---:|---:|---:|---|
| GT-0 | 실험 로그 + 물체 marker + event timestamp | 낮음 | 중 | MVP 가능성 판단 |
| GT-1 | keyframe RGB bbox + physical object ID 수동 라벨 | 중 | 중-상 | ID metrics |
| GT-2 | keyframe 6D pose 수동/반자동 라벨 | 상 | 상 | pose after reacquisition |
| GT-3 | external motion capture/AprilTag/marker | 상 | 상 | 논문 고정밀 GT |

현재 장비 조건에서는 GT-1까지가 현실적이다. 6D pose GT가 없어도 Object Persistence 논문은 가능하다. 다만 pose accuracy 논문이 아니라 ID persistence 논문으로 범위를 명확히 해야 한다.

### 8.3 Fast-LIO2의 역할

Fast-LIO2는 object ID GT가 아니다. 역할은 다음으로 제한한다.

- robot trajectory reference
- loop/drift 구간 확인
- SLAM map correction 규모 추정
- object world pose 계산의 reference frame 안정성 평가

Fast-LIO2가 좋아도 physical object ID 라벨은 별도로 필요하다.

## 9. 평가 지표 설계

### 9.1 필수 지표

| Metric | 정의 | 왜 필요한가 |
|---|---|---|
| ID Switch Count | 같은 physical object가 다른 predicted ID로 바뀐 횟수 | persistence의 핵심 실패 |
| ID Consistency | physical object별 dominant predicted ID 비율 | 장시간 ID 안정성 |
| Re-ID Accuracy | lost 후 재등장 이벤트에서 기존 ID로 연결한 비율 | 연구 질문에 직접 대응 |
| Track Recall | GT object visible 구간 중 object가 검출/연결된 비율 | detection 누락 평가 |
| Track Precision | predicted landmark 중 실제 GT object에 대응되는 비율 | false landmark 평가 |
| Duplicate Landmark Rate | 한 physical object에 여러 predicted ID가 생긴 비율 | DB 품질 평가 |
| Time-to-Reacquire | 재등장 후 기존 ID 확정까지 걸린 시간/관측 수 | 실시간성 평가 |
| Pose Error after Reacquisition | 재등장 후 object pose가 GT/reference와 다른 정도 | ID뿐 아니라 위치 품질 평가 |

### 9.2 추천 종합 지표

- **IDF1**: identity precision/recall 조화 평균. MOT 계열에서 identity 보존 평가에 유용하다.
- **HOTA/AssA**: detection과 association을 분리해 볼 수 있어, SAM-6D detection 실패와 persistence association 실패를 구분하기 좋다.
- **MOTA는 보조 지표**: detection error 영향이 커서 Object Persistence 핵심 지표로는 부족하다.

MVP에서는 TrackEval 포맷으로 변환 가능한 CSV를 만든다.

```text
frame,timestamp,object_id,x,y,z,qx,qy,qz,qw,score
```

2D MOT 포맷을 그대로 쓰기 어렵다면 custom evaluator를 먼저 만들고, 이후 TrackEval 스타일의 IDF1/HOTA 계산으로 확장한다.

### 9.3 성공 기준 초안

| 난이도 | 성공 기준 |
|---|---|
| MVP Pass | Scenario A/B에서 Re-ID Accuracy >= 0.8, ID Switch <= 1/sequence |
| Research Baseline | A/B/C/H에서 IDF1 >= 0.75, duplicate rate <= 0.2 |
| Strong Result | D/E/F/G 포함 평균 IDF1 >= 0.70, Re-ID Accuracy >= 0.75 |
| Failure | 재등장 이벤트 절반 이상에서 새 ID 생성 또는 동일 외형 객체에서 지속적 ID switch |

## 10. 현재 보유 데이터 활용 가능성 평가

현재 보유 데이터:

- `SLAM_one_lap`
- `SLAM_three_laps`
- `SLAM_forward_backward_repeat`
- `SLAM_one_lap_back_and_forth`

이 데이터들은 SLAM trajectory/revisit/loop 성격은 있으나, object persistence 전용 시나리오로 설계된 데이터가 아니다. 따라서 A-H 충족률은 낮게 봐야 한다.

### 10.1 시나리오별 충족률

| Dataset | A 등장-사라짐-재등장 | B 멀리 이동 후 복귀 | C Loop Closure | D 동일 외형 2-5개 | E 객체 이동 | F 부분 가림 | G 조명 변화 | H 장시간 미관측 | 총평 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| SLAM_one_lap | 20% | 40% | 40% | 0% | 0% | 10% | 0% | 10% | loop/revisit 예비 확인용 |
| SLAM_three_laps | 30% | 60% | 70% | 0% | 0% | 10% | 0% | 30% | C/H 예비 실험에 가장 유용 |
| SLAM_forward_backward_repeat | 30% | 50% | 20% | 0% | 0% | 10% | 0% | 20% | B와 짧은 reacquisition 예비용 |
| SLAM_one_lap_back_and_forth | 40% | 60% | 50% | 0% | 0% | 10% | 0% | 25% | A/B/C 일부 예비용 |

### 10.2 활용 가능 항목

보유 데이터로 가능한 것:

- SLAM pose와 RGB-D timestamp alignment 점검
- object observation projection pipeline 점검
- candidate/lost/reacquired lifecycle dry run
- loop/revisit 구간에서 landmark duplicate 발생 여부 예비 확인
- evaluator 입출력 포맷 검증

보유 데이터로 부족한 것:

- 동일 physical object ID GT
- 동일 외형 다중 객체 ambiguity
- 객체 이동
- 의도적 부분 가림
- 의도적 조명 변화
- 장시간 미관측 후 재등장
- 재등장 성공/실패를 논문 수준으로 주장할 반복 실험

### 10.3 권장 사용 방식

보유 데이터는 논문 본 실험이 아니라 **pipeline bring-up dataset**으로 분류한다.

1. `SLAM_three_laps`로 loop/revisit 상황에서 object landmark store가 깨지지 않는지 확인한다.
2. `SLAM_one_lap_back_and_forth`로 가까운 재방문 상황의 ID 유지 dry run을 한다.
3. `SLAM_forward_backward_repeat`로 forward/backward viewpoint 변화에 대한 SAM-6D pose projection 안정성을 본다.
4. 본 평가용으로 A-H 시나리오를 새로 수집한다.

## 11. ROS2 Jazzy 구현 기준 세부 설계

### 11.1 Topic/Message 초안

```text
/sam6d/object_observations_raw
  custom_msgs/ObjectObservationArray

/object_persistence/observations
  custom_msgs/ObjectObservationArray

/object_persistence/landmarks
  custom_msgs/ObjectLandmarkArray

/object_persistence/events
  custom_msgs/ObjectEventArray

/object_persistence/debug_association
  custom_msgs/AssociationDecisionArray
```

### 11.2 Node 분리

| Node | 역할 |
|---|---|
| `sam6d_bridge_node` | SAM-6D output을 ROS2 message로 표준화 |
| `object_observation_node` | TF lookup, `T_map_object` 계산, observation 저장 |
| `object_persistence_node` | landmark update, data association, lifecycle |
| `object_eval_logger_node` | prediction/GT alignment, metrics CSV 생성 |

### 11.3 DB 선택

MVP는 SQLite를 권장한다.

- `objects` table: object_uuid, class, pose, covariance, state
- `observations` table: observation_id, timestamp, object_uuid nullable, raw pose, projected pose
- `association_decisions` table: candidates, scores, selected landmark, reject reason
- `events` table: created, confirmed, lost, reacquired, moved, retired

JSONL도 가능하지만, 재현성과 query 편의성은 SQLite가 더 낫다.

### 11.4 Loop Closure Handling 최소안

SLAM backend를 직접 수정하지 않는 조건에서 가능한 단계:

1. 모든 observation에 `T_map_camera_at_observation`을 저장한다.
2. SLAM pose가 과거 timestamp에 대해 다시 조회 가능하면, loop closure 이후 landmark를 batch recompute한다.
3. 과거 pose 재조회가 불가능하면, loop closure event 전후의 `map` correction transform을 추정해 landmark에 적용한다.
4. 둘 다 어렵다면, loop closure 이후 association gate를 일시적으로 넓히고 `map_version`을 분리한다.

MVP에서는 4번으로 시작하고, 논문 확장에서는 2번 또는 3번을 구현한다.

## 12. MVP Top 10

가장 먼저 구현해야 하는 순서:

1. `ObjectObservation` message/schema 정의
2. SAM-6D output을 timestamped observation으로 저장
3. TF lookup으로 `T_map_object` 계산 및 로그 저장
4. SQLite 기반 ObjectObservation/ObjectLandmark DB
5. Pose-only nearest/gated data association baseline
6. Candidate/Confirmed/Lost/Reacquired lifecycle
7. Association decision debug log
8. Scenario A/B용 GT event CSV format
9. ID Switch/Re-ID Accuracy/ID Consistency evaluator
10. `SLAM_three_laps`와 새 Scenario A bag에서 dry run 비교

이 10개가 끝나면 "Object Persistence가 실제 구현 가능한가"에 대한 1차 답을 데이터로 낼 수 있다.

## 13. 3개월 석사 연구 계획안

### Month 1: MVP와 데이터 프로토콜

| Week | 목표 | 산출물 |
|---|---|---|
| 1 | ObjectObservation/DB/TF projection 구현 | observation JSONL/SQLite, projection sanity report |
| 2 | pose-only association + lifecycle 구현 | object_uuid 유지 baseline |
| 3 | Scenario A/B/C 소규모 수집 | 3개 scenario pilot bags |
| 4 | GT event format + evaluator 구현 | ID switch, Re-ID Accuracy, ID Consistency report |

Month 1 종료 기준: Scenario A/B에서 최소 1개 객체의 lost->reacquired 이벤트를 자동 검출하고 metric을 산출한다.

### Month 2: 난이도 높은 조건과 baseline 비교

| Week | 목표 | 산출물 |
|---|---|---|
| 5 | Scenario D/F/G 수집 | 동일 외형/가림/조명 pilot dataset |
| 6 | pose + geometry/appearance association 추가 | baseline 대비 ID switch 비교 |
| 7 | Scenario E/H 수집 | moved/long absence dataset |
| 8 | loop closure handling 최소안 구현 | Scenario C loop 전후 object pose/ID report |

Month 2 종료 기준: pose-only baseline과 개선 association의 정량 비교표를 만든다.

### Month 3: 논문화와 반복 실험

| Week | 목표 | 산출물 |
|---|---|---|
| 9 | A-H 반복 수집, GT 보강 | final dataset v1 |
| 10 | ablation study | pose-only vs pose+geometry vs pose+descriptor |
| 11 | 실패 사례 분석 | ID switch taxonomy, moved/occlusion failure report |
| 12 | 논문 초안 작성 | problem, method, dataset, results, limitations |

Month 3 종료 기준: 석사 논문/학회 short paper 수준의 정량 결과, 실패 분석, 재현 가능한 bag/evaluator를 확보한다.

## 14. 최종 결론

Object Persistence는 실제 구현 가능하다. 그러나 가능한 이유는 SLAM과 SAM-6D가 이미 충분해서가 아니라, 둘의 출력을 object-level memory system의 observation으로 사용할 수 있기 때문이다.

가장 중요한 기술적 경계는 다음이다.

- SAM-6D는 zero-shot 6D pose estimator이지 long-term identity tracker가 아니다.
- SLAM은 robot pose와 map consistency를 제공하지만 physical object ID를 제공하지 않는다.
- 따라서 Object Persistence의 본체는 `ObjectObservation -> DataAssociation -> ObjectLandmark -> Lifecycle -> Evaluation`이다.

석사 연구로 가장 현실적인 목표는 "SLAM + SAM-6D 기반 장기 Object Persistence의 MVP 구현과 정량 평가 프로토콜 제안"이다. 논문 가치를 만들려면 loop closure, 동일 외형 객체, 장시간 미관측, 객체 이동 중 최소 2개 이상을 정량 실험에 반드시 포함해야 한다.

## 15. Sources Checked

- SAM-6D paper: https://arxiv.org/abs/2311.15707
- SAM-6D implementation: https://github.com/JiehongLin/SAM-6D
- Semantic SLAM with Autonomous Object-Level Data Association: https://arxiv.org/abs/2011.10625
- EM-Fusion dynamic object-level SLAM: https://arxiv.org/abs/1904.11781
- Fusion++ volumetric object-level SLAM: https://arxiv.org/abs/1808.08378
- TrackEval / HOTA / ID metrics implementation: https://github.com/JonathonLuiten/TrackEval
- RTAB-Map loop closure overview: https://introlab.github.io/rtabmap/
- ROS2 Jazzy tf2 documentation: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html
- Nav2 transform setup documentation: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- Local workspace evidence: `performance_reports/orb_sparse_vs_orb_dense_vs_rtabmap_performance.md`
- Local workspace evidence: `rtabmap_ws/output/*/run_summary.md`
- Local workspace evidence: `sam6d_jitter_ws/output/sam6d_slam_fusion_shadow_eval/sam6d_slam_fusion_shadow_eval_report.md`
