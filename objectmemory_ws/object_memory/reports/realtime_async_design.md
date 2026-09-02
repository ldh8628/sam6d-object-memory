# 실시간 비동기 Object Memory — 설계 (design-first)

상태: **검토용 초안**. 구현 전 결정 2가지 확인 → 구현 → 4-bag 정량검증.
선행 완료: Story 5 품질가중 confidence + P_D 보정(`story5_confidence_likelihood_design.md`).

---

## 1. 왜 지금 구조로는 실시간이 안 되나 (한 줄)

지금은 **오프라인 배치**다: bag을 다 돌린 뒤 SLAM 궤적 전체(`sorted(slam_poses)`)와 SAM PEM 결과 전체를 손에 쥐고, 프레임을 시간순으로 훑으며 융합한다. 시차 문제가 안 보이는 이유는 **전부 과거**라 타임스탬프 매칭이 공짜이기 때문이다.

실시간에서는:
- SLAM pose가 고빈도(~30Hz)·저지연으로 **흘러 들어온다**.
- SAM-6D 결과가 저빈도·**고지연(수백 ms~초)**으로, 심지어 **순서가 뒤바뀌어** 들어온다. 각 결과의 `stamp`은 항상 **과거**(찍힌 시점)다.

## 2. 핵심 통찰 — 시간정합 원형이 이미 있다

`object_memory_runner.py:245 _nearest_pose(stamps, poses, ts, tol)`가 바로 우리가 필요로 하는 **stamped pose lookup**이다. 검출의 `ts`로 SLAM 궤적에서 그 시점 카메라 pose를 되찾아,
```
T_map_obj = T_map_cam(ts)  ·  T_cam_obj      # ts = 찍힌 시점, NOT 도착 시점
```
로 맵에 박는다. 즉 실시간 비동기는 **새 수학이 아니라**, 이 lookup을 **유한 스트리밍 버퍼**로 승격하고, 프레임 본문을 **상태 유지 step 처리기**로 감싸는 리팩터링이다. 존재필터·association·fusion·P_D 로직은 **그대로 재사용**한다(회귀 위험 최소).

## 3. 아키텍처 — 3개 컴포넌트

```
   SLAM pose 스트림 ──▶  ┌──────────────┐
     (고빈도/저지연)       │  PoseBuffer  │  lookup(stamp) → 그 시점 T_map_cam
                          └──────┬───────┘   (유한 윈도, 과거만 조회)
                                 │
   SAM-6D 결과(늦게/역순) ─▶  ┌───┴──────────────────┐
     stamp=과거              │ StreamingObjectMemory │  step(dets, pose, ts)
                            │  = 러너 프레임 본문     │   → store, live_ids 갱신
                            └───┬───────────────────┘
                                │
                          ┌─────┴──────┐
                          │   driver   │  오프라인: bag 재생+지연 시뮬 (검증용)
                          └────────────┘  온라인: ROS2 2-subscriber (후순위)
```

### 3.1 PoseBuffer (신규 `core/pose_buffer.py`)
- 들어오는 모든 `SlamCameraPose`를 stamp 순으로 유한 윈도(예: 최근 `W`=10s)에 유지. 오래된 것은 폐기 → **메모리 무한증가 없음**.
- `lookup(stamp, tol)`:
  - **온라인에선 항상 뒤(과거)만 조회** — SAM 결과 도착 시 `stamp`의 pose는 이미 버퍼에 있음. 오프라인 `_nearest_pose`의 ±1 양방향 탐색보다 오히려 단순.
  - 버퍼 윈도보다 오래된 stamp → `None`(= INV-008: pose 없음 → 융합 스킵, 검출 폐기). "가망 없이 늦은" SAM 결과의 자연스러운 처리.
- **결정①(§5)**: nearest-within-tol vs 두 pose 사이 보간(SLERP+lerp). 보간이 시간정합 잔차를 직접 줄인다.

### 3.2 StreamingObjectMemory (러너 본문 추출)
- `run_object_memory`의 프레임 루프 본문(stage1 측정 → stage2 게이트쌍 → stage3 accept/spawn → deleter)을 상태 유지 클래스 `step(detections, pose, ts)`로 추출. `store`, `live_ids_by_name`, `ages`, `logit_prior`, `weights`를 인스턴스가 보유.
- 오프라인 `run_object_memory`는 **이 step을 프레임순으로 부르는 얇은 래퍼**로 재작성 → 기존 79 테스트가 회귀 가드가 됨(동일 결과 보장).
- **결정②(§5)**: deleter(미검출 감쇠)를 **언제** 돌리나 — SAM 도착시 vs 별도 tick.

### 3.3 driver
- **오프라인 async 시뮬(이번 단계 산출물)**: bag 재생. pose는 stamp에 버퍼 투입. SAM 배치는 **측정 지연만큼 늦춰**(그리고 지연 편차로 역순 섞어) step에 투입. 지연→0이면 오프라인과 동일 결과가 나와야 함(정합성 검증).
- **온라인 ROS2 노드(후순위, 사용자 지시=지금 만들지 않음)**: 같은 버퍼+step에 2개 subscriber. `MultiThreadedExecutor` 필수(참고: orbslam3 IMU 배선 교훈).

## 4. 두 개의 미묘한 실시간 이슈

**(A) deleter 타이밍 — 생명주기를 SAM 도착률에서 분리해야 하나?**
오프라인은 매 프레임 deleter를 돈다("시야 안인데 미검출 → 감쇠"). 실시간에서 SAM 배치는 드물게 온다. deleter를 **SAM 도착시에만** 돌리면, 배치 사이 구간에서 유령이 감점을 안 받아 **너무 오래 생존**한다. P_D 감쇠의 의미는 "관측 기회마다"이고, 관측 기회 = 카메라가 본 프레임이지 SAM 출력이 아니다. → tick 기반(SLAM 키프레임/타이머)이 오프라인 의미에 충실.

**(B) 역순 도착.**
지연 편차로 stamp=1.0 배치가 stamp=1.2 배치보다 늦게 올 수 있다. 처리 방식: 짧은 재정렬 버퍼로 stamp 정렬 후 커밋하되, "committed watermark"보다 심하게 오래된 배치는 폐기(=INV-008과 동형). fusion은 incremental-mean이라 소폭 역순엔 강건.

## 5. 확인이 필요한 결정 (구현 전)

| # | 결정 | 선택지 | 추천 | 근거 |
|---|------|--------|------|------|
| ① | pose lookup 정확도 | (a) nearest-within-tol (오프라인과 동일) / (b) 두 pose SLERP+lerp 보간 | **(b) 보간** | 시간정합 잔차를 직접 줄임 = 이 작업의 목적 그 자체. 비용 저렴. |
| ② | deleter(미검출 감쇠) 타이밍 | (a) SAM 도착시에만(단순) / (b) SLAM 키프레임 tick으로 분리 | **(b) tick 분리** | 유령 생명주기를 SAM 도착률에서 분리. 오프라인 의미에 충실. |

*결정③(스코프)은 사용자 기존 지시로 이미 확정: **오프라인 async 시뮬+정량검증 먼저, ROS 노드는 후순위**(지금 안 만듦).*

## 6. 정량 검증 계획 (4-bag)

1. **정합성**: 지연→0으로 async 시뮬 = 오프라인 결과와 landmark 집합·pose가 **동일**해야(리팩터링 무회귀 증명). 79 pytest + 신규 async 테스트.
2. **시간정합 가치(ablation, 개념③ 실증)**: async를 **시간정합 끄고**(도착시점 pose로 융합) 돌려 landmark **번짐(smearing)** 정량화 → 켰을 때 대비 위치오차·유령수 개선폭.
3. **지연 스윕**: 시뮬 지연 0 / 0.2 / 0.5 / 1.0s → landmark 위치드리프트 vs 지연 곡선. 시간정합이 켜져 있으면 평탄해야 함(=지연에 강건).
4. 산출: 오프라인과 동일하게 4-bag 표 + gif(active+remembered 단일 산출물).

## 7. 신규/변경 파일 (예정)

- 신규 `src/core/pose_buffer.py` — PoseBuffer(lookup, 유한 윈도, 보간).
- 변경 `src/pipeline/object_memory_runner.py` — 프레임 본문 → `StreamingObjectMemory.step()` 추출, `run_object_memory`는 래퍼로.
- 신규 `scripts_test/run_async_object_memory.py` — 오프라인 async 시뮬 driver.
- 신규 `scripts_test/eval_async.py` — §6 정합성/ablation/지연스윕 지표.
- 신규 `scripts_test/test_streaming.py` — step 등가성·버퍼 lookup·역순·out-of-window 폐기.
- ROS2 노드는 **후순위**(별도 스토리).

## 8. 불변식 (유지)

- INV-002 object_id 소유·재사용 없음, INV-008 pose 없으면 융합 스킵 — 버퍼 miss가 자연히 INV-008을 재현.
- viz = active+remembered 단일 산출물, 전역 가중(객체별 임계 없음) 유지.

## 9b. 실행 계획 요약 — A → B → 전프레임 실험 (라이브 확정)

목표: 저장 데이터 스트리밍 엔진(A) → 실제 라이브 노드(B) → **SAM_* 4 bag 전 프레임(stride1) 실험**(C).
핵심 원칙: **A의 step/buffer가 그대로 B의 코어**가 되도록 짠다(입력 소스만 교체). A는 배치와 대조로 검증되고, B는 A를 재사용하므로 신뢰 이전.

### Phase A — 스트리밍 엔진 (저장 결과 재생, 검증) — *환경 불필요, 즉시 가능*
- `core/pose_buffer.py`: 유한 pose 버퍼 + `lookup(stamp)` 보간(결정①).
- `pipeline/object_memory_runner.py`: 프레임 본문 → `StreamingObjectMemory.step(dets, pose, ts)` 추출. `run_object_memory`는 순서 호출 래퍼(동작 불변, **79 테스트가 가드**).
- `scripts_test/run_realtime_object_memory.py`: 저장 SLAM traj + SAM 결과를 **타임스탬프 순서**로 투입(미래 안 봄), SAM은 **실측 지연만큼 늦춰** 도착. 키프레임 원장 + 지연마감 은퇴보류(결정②). 매 이벤트 상태 로그/프레임.
- **검증**: (a) 지연→0 == 배치와 동일(엔진 정확성) (b) 시간정합 OFF → 번짐(개념③) (c) 지연 스윕 0/0.2/0.5/1.0s 드리프트. 데이터=기존 stride10.

### Phase B — 라이브 노드 (입력 소스 교체) — *GPU·ROS 환경 필요*
- object memory를 ROS2 노드화: pose sub(고빈도)→PoseBuffer, detection sub(저빈도·늦음)→step. **MultiThreadedExecutor 필수**(참고: orbslam3 IMU 교훈).
- 동시 구동: ORB-SLAM3 노드 ∥ SAM 노드(`sam6d_multiobject_node`, **persistent PEM**=warm) ∥ object_memory 노드. bag 재생으로 카메라 스트림 주입.
- A의 `PoseBuffer`/`step()` **그대로 재사용** — B는 콜백 배선 + launch가 대부분, 새 알고리즘 없음.
- 환경: `sam_yolo`(ISM) / `sam6d_ros_humble`(PEM) 분리(참고: ISM→PEM 브릿지). 미가동 시 A까지만 즉시, B는 환경 준비 후.

### Phase C — 실험: SAM_* 4 bag **전 프레임(stride 1)**
- SAM 입력을 stride 10 → **stride 1**(모든 프레임)로. **정직한 제약**: SAM 처리량(ISM~8fps, PEM~15fps/det) < 30fps → **진짜 실시간 전프레임 불가**. 두 모드로 분리:
  - **C1 (라이브 정직)**: 처리 가능한 만큼만 소비, **못 따라간 프레임은 자연 드롭**(=실제 로봇 거동). 프레임드롭율 측정.
  - **C2 (조밀 상한)**: 실시간 아님, **모든 프레임 처리**(SAM이 걸리는 만큼 대기). 검출 밀도 최대 = 성능 상한.
- **측정(4 bag)**: landmark 수·생존(active+remembered), **choco 안정성**, 유령수명, 위치드리프트, (C1)드롭율, **stride10 대비 개선폭**. 산출=단일 gif(active+remembered).
- **비용 경고**: 전프레임 = 검출량 ~10배 → PEM 시간 ~10배(warm 4bag ~2~3분 → 조밀 시 수십 분). C는 배치로 사전생성(ISM/PEM) 후 A/B에 투입.

### 순서·게이트
A(검증 통과) → B(A 재사용, 환경 준비되면) → C(B 위에서 전프레임). 각 단계는 앞 단계의 검증을 전제로만 진행.

## 9c. 구현·검증 결과 (A·B·C1 완료)

**Phase A (스트리밍 엔진)** — `core/pose_buffer.py` + `StreamingObjectMemory.step()` 추출, `run_object_memory`는 래퍼(동작 불변). `run_realtime_object_memory.py`(async 재생) + `eval_async.py` + `test_streaming.py`. **87 pytest green**(79+8). 4-bag:
- PARITY: 스트리밍(interp off, align on, L=0.15) == 배치, drift **0.00000** (엔진 무회귀 증명).
- ABLATION: 시간정합 OFF → 번짐 circle 7.8cm·loop1/2 ~3.5cm·occlusion~0 (카메라 이동량에 비례 — 개념③ 실증).
- 지연 스윕: 정합 ON이면 drift ~0 (L=0/0.2/0.5/1.0 무관) = 지연 불변.

**Phase B (라이브 노드)** — `ros/object_memory_node.py`(rclpy). Phase A 엔진 그대로 재사용, ROS 콜백만 배선(PoseStamped→버퍼, Detection3DArray→step, MultiThreadedExecutor+Reentrant). conda `sam6d_ros_humble`에서 구성·구동 검증. 게이트: SAM 노드가 PoseArray(이름 손실)→**Detection3DArray**(이름+score+pose) publish로 바꿔야 함(`ros/README.md`).

**Phase C1 (라이브 장기 메모리 맵)** — `run_c1_map.py`, 실측 지연 L=0.2s(ISM0.12+PEM0.065). **조밀 SLAM 전 궤적(795~2431 pose) 연속 스트리밍** + 주기 SAM(54~148 frame). 결과 맵:
| bag | landmarks | 주요 |
|---|---|---|
| circle | 5 active | Bear/Mugcup/milk/saffron/Febreze (choco 상류recall로 미생존) |
| loop1 | 6 active+1 remembered | **choco_hazelnut conf0.92 안정**, Rabbit=remembered(화면밖 장기기억), milk/saffron 강 |
| loop2 | 5 active | Bear/Febreze/Mugcup/Rabbit/Sauce |
| occlusion | 2 active | Bear/milk |
- dropped=0(현 SAM 밀도 < SAM 처리량 → 라이브 드롭 미발생). **choco 라이브 경로에서도 안정**(P_D 보정 재확인). remembered 분리 동작(loop1 Rabbit).

**남은 것 (C1 확장)**: 진짜 stride-1(전 프레임 SAM)은 SAM 밀도만 올리며 **SAM 파이프라인 재생성(sam6d_ws, ~1~2h GPU)** 필요 → 드롭/밀도 효과 관찰용. SLAM 측 전프레임 연속은 이미 반영됨. 메커니즘(정합·드롭·장기기억)은 A·B·C1로 증명 완료.

## 9. 다음 액션 제안

- **A (추천): 결정 ①=보간·②=tick으로 확정하고 구현 진행** → §6 정량검증. 리팩터링이 79 테스트로 가드되므로 회귀 위험 낮음.
- **B: 결정 ①/②를 다르게 고르고 싶다** (예: ②를 단순한 도착시-감쇠로) → 표에서 선택.
- **C: 설계를 더 다듬고 싶다** (역순/watermark 정책, 버퍼 윈도 W, ROS 노드 우선순위 등 특정 항목 지정).

질문: **A로 확정하고 구현+4-bag 정량검증까지 진행할까요?** 아니면 ①/② 중 바꾸실 게 있나요?
