---
title: '실시간 지향 Raw/ObjectMemory 포즈 비교'
type: 'feature'
created: '2026-08-27'
status: 'done'
baseline_commit: 'NO_VCS'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 현재 ObjectMemory Explorer는 SAM-6D 원시 pose만 영상에 표시해, 같은 검출에 대한 ObjectMemory fused pose의 보정 효과를 동일 시각에서 직접 비교할 수 없다. 지도와 수렴 그래프도 재생 프레임마다 정적 요소까지 다시 그린다.

**Approach:** 기존 ObjectMemory 결과의 active 연관 검출에만 `inverse(T_map_cam) * T_map_obj_fused`를 계산해 타임라인에 저장하고, 하나의 디코딩 비디오 프레임을 raw 영상 패널과 fused Canvas 패널에서 함께 사용한다. Dense map과 그래프 곡선은 로드·선택·resize 때 사전 렌더링하고 재생 중에는 동적 요소만 갱신한다.

## Boundaries & Constraints

**Always:** ObjectMemory가 사용한 동일 프레임의 camera map pose와 fused object map pose를 기존 SE(3) 전용 함수로 변환한다. 정상 연관 후 상태가 `active`인 검출만 `corrected_pose`를 가지며 다른 검출은 명시적으로 `null`이다. 오른쪽 패널도 bbox와 label은 항상 표시한다. 비디오는 한 요소·한 소스만 사용하고 실제 디코딩 프레임당 렌더링은 최대 한 번이다. 각 `/api/frame` 응답은 독립적으로 렌더링 가능해야 한다.

**Ask First:** 출력 schema를 비호환 변경하거나, 브라우저·서버에 새 의존성을 추가하거나, 현재 생성 결과 외의 추론을 실행해야 하는 경우.

**Never:** 두 번째 `<video>`, 비교 MP4, 새 endpoint, WebSocket, WebGL, Worker, 바이너리 프로토콜, SAM-6D 재추론 또는 브라우저 pose 행렬 계산을 추가하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| active 정상 연관 | match 결정 + active snapshot + SLAM pose | raw `R/t_mm`과 fused `corrected_pose.R/t_mm`를 같은 프레임에 표시 | 유한 SE(3) 값을 생성기에 저장 |
| tentative/rejected/미연관 | 검출은 있으나 보정 대상 아님 | `corrected_pose: null`, 오른쪽 bbox/label만 표시 | pose를 추정하거나 대체하지 않음 |
| SLAM 누락 | ObjectMemory가 `step_no_pose` 처리 | `corrected_pose: null` | raw 검출은 그대로 유지 |
| rVFC 미지원 | 네이티브 callback 없음 | RAF로 재생하되 같은 frame sequence 중복 렌더 생략 | pause/seek 때 callback 상태 초기화 |
| resize/객체 선택 | Canvas 크기 또는 그래프 대상 변경 | map/graph 정적 배경만 재생성 | 현재 프레임 동적 요소 즉시 재합성 |

</frozen-after-approval>

## Code Map

- `integration/run_260826_object_memory.py` -- ObjectMemory 결과와 원시 검출을 frame timeline으로 결합하고 explorer metadata를 생성한다.
- `objectmemory_ws/object_memory/src/core/transforms.py` -- 강체 역변환과 `predict_current_camera_object_pose()`의 기존 구현이다.
- `integration/serve_object_memory_explorer.py` -- timeline 행을 `/api/frame`에서 그대로 제공하고 서버 self-test를 수행한다.
- `integration/object_memory_explorer/index.html` -- 단일 video와 비교·지도·그래프 패널 구조다.
- `integration/object_memory_explorer/app.js` -- 프레임 동기화, overlay, map, graph Canvas 렌더링을 담당한다.
- `integration/object_memory_explorer/app.css` -- 2×2 비교 레이아웃과 반응형 표시를 담당한다.

## Tasks & Acceptance

**Execution:**
- [x] `integration/run_260826_object_memory.py` -- active 정상 연관 검출에 기존 SE(3) 예측 함수를 적용해 `corrected_pose`를 한 번 직렬화하고 공식·최소 상태 metadata를 기록한다.
- [x] `integration/object_memory_explorer/index.html`, `app.css`, `app.js` -- raw/fused 첫 행, map/graph 둘째 행, 단일 decoder, rVFC/중복 방지 fallback, map·graph 사전 렌더링을 구현한다.
- [x] `integration/serve_object_memory_explorer.py` -- additive `corrected_pose`가 `/api/frame`에서 보존됨을 self-test로 고정한다.
- [x] 실제 longcircle dark 산출물을 재생성하고 자동·브라우저 검증 및 성능 측정을 수행한다.

**Acceptance Criteria:**
- Given frame 235 이후 active 연관 검출이 있을 때, when 재생하면, then 두 패널의 영상 시각과 bbox는 일치하고 raw/fused 축은 저장된 각 pose에 따라 독립적으로 보인다.
- Given tentative, rejected, SLAM 누락 또는 미연관 검출일 때, when fused 패널을 그리면, then bbox/label만 있고 축은 없다.
- Given 30 fps 재생일 때, when 브라우저 callback 수를 측정하면, then frame render 수는 실제 제시된 decoded frame 수보다 많지 않다.

## Spec Change Log

## Design Notes

`FrameResult.camera_T_map_cam`은 ObjectMemory가 실제 사용한 pose이며 snapshot의 `T_map_obj`가 fused authoritative pose다. 생성 시 이 둘을 tuple 기반 `predict_current_camera_object_pose()`에 직접 넣어 일반 역행렬과 브라우저 변환을 피한다. Canvas 정적 배경은 DPR 크기의 offscreen canvas로 보존하고 동적 pass에서 `drawImage`로 복사한다.

## Verification

**Commands:**
- `python3 -m py_compile integration/run_260826_object_memory.py integration/serve_object_memory_explorer.py` 및 `node --check integration/object_memory_explorer/app.js` -- 문법 통과.
- `python3 integration/run_260826_object_memory.py --self-test` 및 `python3 integration/serve_object_memory_explorer.py --self-test` -- 생성기·서버 계약 통과.
- `cd objectmemory_ws/object_memory && pytest scripts_test -q` -- ObjectMemory 전체 회귀 통과.
- `python3 integration/run_260826_object_memory.py --dataset 260826_etri_longcircle_dark` -- 실제 timeline/map 재생성.
- 산출물 invariant 검사와 localhost 브라우저 측정 -- 유한 회전행렬, active 제한, frame 235 이후 비교, bbox-only, 단일 video, render/callback 비율 확인.

**Manual checks (if no CLI):**
- frame 235 이후 raw/fused 영상 일치와 축 차이, tentative 프레임의 bbox-only 동작을 확인한다.

## Verification Results

- 2026-08-28, headless Chrome rVFC 2.5초 재생: decoded callback 75, fused Canvas render 75, 초과 렌더 0.
- 같은 구간의 강제 RAF fallback: decoded frame 76, fused Canvas render 76, 초과 렌더 0.
- frame 235: video sequence 235, fused 배경과 동일 video frame의 동일 pixel 98.92% (나머지는 bbox/축 overlay), raw/fused pose 차이 3.13° / 5.823 mm.
- frame 214 tentative: raw 축 3개, fused 축 0개이며 bbox/label은 유지됐다.
- 4,101-frame 실제 재생성: 7,440 detections 중 corrected 7,407, tentative 28, inactive match 1, rejected/no-SLAM 4. 보정 누적 113.584 ms, 전체 생성 13.197 s (0.861%).

## Suggested Review Order

**보정 pose 데이터 계약**

- 기존 강체 변환으로 active 연관만 브라우저용 pose로 직렬화한다.
  [`run_260826_object_memory.py:184`](../../integration/run_260826_object_memory.py#L184)

- 모든 검출은 독립 렌더 가능한 nullable 필드를 한 번 저장한다.
  [`run_260826_object_memory.py:289`](../../integration/run_260826_object_memory.py#L289)

- 산출물에 계산 공식과 최소 상태를 명시한다.
  [`run_260826_object_memory.py:483`](../../integration/run_260826_object_memory.py#L483)

- 기존 API가 additive metadata와 frame payload를 그대로 전달한다.
  [`serve_object_memory_explorer.py:119`](../../integration/serve_object_memory_explorer.py#L119)

**단일 decoder 비교 재생**

- 하나의 video와 fused Canvas가 첫 행의 비교 구조를 만든다.
  [`index.html:24`](../../integration/object_memory_explorer/index.html#L24)

- 같은 decoded frame 위에 raw와 corrected pose를 각각 그린다.
  [`app.js:31`](../../integration/object_memory_explorer/app.js#L31)

- rVFC timestamp를 정확한 sequence로 매핑하고 RAF도 중복 제거한다.
  [`app.js:86`](../../integration/object_memory_explorer/app.js#L86)

**정적 Canvas 캐시**

- dense points는 resize 때만 offscreen map으로 만든다.
  [`app.js:38`](../../integration/object_memory_explorer/app.js#L38)

- 그래프 곡선은 선택·resize 때만 offscreen base로 만든다.
  [`app.js:57`](../../integration/object_memory_explorer/app.js#L57)

**회귀 검증**

- 생성기 self-test가 active 성공과 tentative null을 고정한다.
  [`run_260826_object_memory.py:566`](../../integration/run_260826_object_memory.py#L566)

- 서버 self-test가 `/api/frame`의 corrected pose 보존을 확인한다.
  [`serve_object_memory_explorer.py:304`](../../integration/serve_object_memory_explorer.py#L304)
