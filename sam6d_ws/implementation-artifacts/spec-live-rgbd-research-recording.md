---
title: 'RealSense RGBD 라이브 입력과 연구용 비차단 기록·재생'
type: 'feature'
created: '2026-08-27'
status: 'done'
baseline_commit: 'eed879d3196cf1f94c91d9595ca6a929c4d657a7'
context:
  - 'implementation-artifacts/spec-minimal-pem-explorer-v2.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** RealSense의 개별 RGB·Depth 토픽을 Python에서 다시 동기화·복사하면 정상 30 Hz 센서 스트림이 receiver에서 크게 감소하고, 기존 오버레이 영상은 연구에 필요한 원본 RGB-D·검증·거절 정보를 보존하지 못한다.

**Approach:** 라이브 입력은 공식 `/camera/camera/rgbd` 결합 메시지 하나로 받고, 최신 프레임 공유메모리를 읽는 별도 viewer/recorder 프로세스로 추론에 역압 없이 원본 RGB-D와 pose 타임라인을 독립 저장한 뒤 기존 Explorer에서 합성 재생한다. Bag 입력은 기존 분리 토픽 동기화 경로를 유지한다.

## Boundaries & Constraints

**Always:** 기본 기록은 30 Hz RGB와 실제 추론 프레임의 uint16 Depth이며, full-depth는 명시적으로만 켠다. reader 종료가 writer 공유메모리를 unlink하지 않는다. recorder 실패·지연은 추론을 중단하거나 늦추지 않고 누락 수를 manifest에 남긴다. pose·검증·거절·mask 원본은 수치/별도 자산으로 보존한다. 기존 Explorer v2와 300 후보 분석은 그대로 유지한다.

**Ask First:** ROS bag 전체 또는 pose를 구운 중복 MP4를 추가하거나, 기존 출력 run을 교체·삭제하거나, NVENC 실패 시 CPU RGB 인코딩으로 자동 전환해야 하는 경우.

**Never:** recorder가 receiver/infer에 역압을 걸거나, full-depth 누락을 숨기거나, incomplete run을 완료로 표시하거나, output root 밖 파일을 웹으로 제공하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 라이브 추론 | 기본 실행 | RGBD 한 토픽을 수신하고 추론만 수행 | 침묵/QoS 상태를 receiver 통계로 노출 |
| 저부하 기록 | `--record` | 모든 관찰 RGB와 추론된 Depth·진단·mask 저장 | encoder 실패는 manifest incomplete, SAM은 계속 |
| 전체 Depth | `--record-full-depth` | recorder가 관찰한 모든 Depth 저장 | 밀리면 Depth만 drop하고 수치 기록 |
| viewer 조합 | `--view --record` | viewer/recorder가 독립 reader로 동시 실행 | 어느 reader 종료도 writer에 영향 없음 |
| live replay | 완료·중단 live run | 영상/pose/status/depth와 stale pose 시간 표시 | 불완전 상태 명시, 안전한 범위 요청만 제공 |

</frozen-after-approval>

## Code Map

- `run/realsense.sh`, `run/sam6d.sh` -- 카메라 옵션과 사용자 실행 인터페이스.
- `realtime/sam6d_receiver_node.py`, `realtime/shm_channel.py` -- live RGBD/legacy pair 분기와 최신 프레임 계약.
- `realtime/launch/sam6d_split.launch.py`, `realtime/sam6d_live_recorder.py` -- 독립 viewer/recorder 수명과 비차단 FFmpeg 기록.
- `realtime/sam6d_infer.py` -- pose·진단·mask 산출물 계약.
- `tools/serve_pem_explorer.py`, `tools/pem_explorer_live/` -- live run 검증·Range 영상·타임라인 UI.

## Tasks & Acceptance

**Execution:**
- [x] RGBD 단일 구독과 legacy split-topic 회귀 경로를 구현한다.
- [x] reader resource tracker와 frame sequence 메타데이터를 고정한다.
- [x] NVENC RGB/FFV1 Depth recorder, manifest, CLI/launch 종료 처리를 구현한다.
- [x] `live_replay` discovery/API/UI와 incomplete·depth 표시를 구현한다.
- [x] 최소 계약·보안·CLI·회귀 테스트를 추가한다.

**Acceptance Criteria:**
- Given live RGBD 메시지, when receiver가 처리하면, then RGB/depth stamp와 color CameraInfo가 한 callback에서 동일 shared frame으로 기록된다.
- Given recorder/viewer 조합, when reader 하나가 종료하면, then writer와 다른 reader는 계속 동작한다.
- Given recorder encoder failure or lag, when SAM 추론이 계속되면, then 기록만 incomplete/drop 처리되고 manifest에 원인이 남는다.
- Given live run, when Explorer에서 seek하면, then 대응 pose·상태·검증·거절·Depth와 pose held 경과가 표시된다.

## Spec Change Log

## Design Notes

RGB와 Depth 파일은 프레임 index→ROS timestamp JSONL로 결합한다. 브라우저는 H.264를 직접 Range 재생하고, FFV1 Depth는 요청한 index 한 장만 서버가 컬러맵 PNG로 변환한다.

## Verification

**Commands:**
- `python -m pytest -q tests/test_live_recording.py tests/test_shm_slam_context.py tests/test_pem_explorer_live.py tests/test_pem_explorer_v2.py` -- 신규 계약과 기존 Explorer 회귀.
- `bash -n run/realsense.sh run/sam6d.sh && python -m py_compile realtime/*.py tools/serve_pem_explorer.py` -- 실행 경로 문법.
- 실제 D455f 60초와 기록 조합 실행 -- 29.5 Hz 이상, timestamp/frame 정렬, Ctrl-C finalize 및 추론 성능 감소 5% 미만.

**Results:**
- 집중 계약/Explorer 회귀 40개 통과, 전체 회귀 235개 통과(2개는 저장소에 없는 기존 `output/` fixture 때문에 실패).
- RTX 5090에서 실제 640×480 3프레임 H.264 NVENC(yuv420p)·FFV1(gray16le) 생성과 Depth PNG 복원 통과.
- 공유메모리→recorder→SIGINT 합성 E2E에서 RGB 30/30, Depth 3/3, 누락 0, manifest 완료 처리를 확인.
- D455F는 연결 확인 후 드라이버 기동 중 `Depth stream start failure` 하드웨어 오류가 발생해 60초 rate/성능 측정은 수행하지 못함.

## Suggested Review Order

**실행 진입점**

- 사용자 옵션을 launch 구성과 독립 기록 폴더로 변환한다.
  [`sam6d.sh:20`](../run/sam6d.sh#L20)

- RealSense가 동기화된 공식 RGBD 메시지를 발행한다.
  [`realsense.sh:12`](../run/realsense.sh#L12)

**라이브 데이터 경로**

- 결합 메시지를 단일 callback에서 원자적 공유 프레임으로 만든다.
  [`sam6d_receiver_node.py:163`](../realtime/sam6d_receiver_node.py#L163)

- reader 종료가 writer 공유메모리를 제거하지 않게 소유권을 분리한다.
  [`shm_channel.py:76`](../realtime/shm_channel.py#L76)

- 라이브 mask를 겹침 없는 무손실 bitset으로 보존한다.
  [`sam6d_core.py:201`](../realtime/sam6d_core.py#L201)

**비차단 기록**

- 최신 프레임만 큐잉해 NVENC·FFV1 지연을 추론과 격리한다.
  [`sam6d_live_recorder.py:128`](../realtime/sam6d_live_recorder.py#L128)

- viewer와 recorder를 독립 프로세스로 기동하고 실패를 격리한다.
  [`sam6d_split.launch.py:80`](../realtime/launch/sam6d_split.launch.py#L80)

**재생과 검증**

- manifest·index를 검증한 뒤 timestamp 기반 replay 모델을 만든다.
  [`serve_pem_explorer.py:550`](../tools/serve_pem_explorer.py#L550)

- pose held 시간과 현재 검증값을 분리해 렌더링한다.
  [`app.js:394`](../tools/pem_explorer_live/app.js#L394)

- recorder 계약과 완료 manifest를 작은 회귀 테스트로 고정한다.
  [`test_live_recording.py:64`](../tests/test_live_recording.py#L64)
