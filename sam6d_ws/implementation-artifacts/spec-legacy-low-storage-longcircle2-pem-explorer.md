---
title: 'Legacy UI 동일형 저용량 longcircle2 PEM Explorer'
type: 'feature'
created: '2026-08-24'
status: 'done'
baseline_commit: 'NO_VCS'
context:
  - 'implementation-artifacts/spec-minimal-pem-explorer-v2.md'
  - 'implementation-artifacts/spec-longcircle2-sam-camera-explorer.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 현재 localhost Explorer는 처리된 PEM 시도 목록만 제공하고 legacy SAM report의 전체 프레임·9객체·후보 근거 탐색 경험을 복원하지 못한다. 실시간 latest-frame 실행은 bag 프레임을 드롭하므로 완전한 시각화 자료와 순수 실시간 계측도 구분되지 않는다.

**Approach:** trusted RGB-D bag을 순차 처리하는 저용량 offline 수집 profile과 추가 GPU shadow 계산을 금지하는 realtime profile을 같은 compact 계약에 추가한다. 서버는 bag RGB와 후보 근거를 RAM에서 동적으로 만들고 legacy DOM/CSS 기반 landing+iframe UI에 전체 프레임 상태, 실제 생산 선택, geometry Top-1 및 legacy shadow/GT 비교를 표시한다.

## Boundaries & Constraints

**Always:** exhaustive 실행은 검증된 2,130 bag frame을 정확히 한 번씩 순서대로 처리하고 `output/longcircle2_visualization`을 새로 생성한다. `frames.jsonl`은 검출 없는 프레임까지 기록하고, `detections.jsonl` 및 ROS 계약에는 생산 검증을 통과한 pose만 남긴다. full profile은 생산 선택 후 300 texture를 계측하고 realtime profile은 생산 중 이미 계산된 texture만 기록한다. 실제 선택 index/proposal/R/t/통과 여부는 계측 OFF와 같아야 한다. RGB JPEG·후보 projection·heatmap·feature tensor는 디스크에 쓰지 않으며 시도당 binary는 40 KiB 이하이다. 기존 대용량 static report와 `longcircle2_manual`은 읽기 호환·불변으로 보존한다.

**Ask First:** 기존 완료 run/report를 삭제·교체해야 하는 경우, shadow 결과를 생산 선택에 반영하는 경우, 후보별 이미지/feature를 영구 저장해야 하는 경우.

**Never:** realtime profile에서 미측정 후보 texture를 선계산하지 않는다. 미처리 bag frame을 검출 없음으로 표기하지 않는다. pseudo-GT를 외부 GT로 표기하지 않는다. symlink·절대경로·traversal·미완료/불일치 schema run을 정상 자산처럼 제공하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|---|---|---|---|
| exhaustive capture | trusted 2,130-frame bag, 새 output 경로 | 2,130 처리 frame, compact PEM 시도, 완료 manifest | frame count/stamp 불일치 또는 기존 output이면 실패 |
| realtime capture | latest-frame compact run | 처리 frame만 결과, 나머지는 `unprocessed_realtime_drop`, 미측정 texture flag | 동적 후보 클릭 때만 한 건 재계산 |
| no detection / rejected | 처리 frame에 PEM 없음/탈락 | 각각 `not_detected`/`pem_rejected`, 탈락 카드 클릭 가능 | 미수집 값을 실패로 위조하지 않음 |
| legacy/partial/hostile | 기존 static, partial, path 공격 | static 직접 열기/명시적 제한/통제된 거부 | output root 밖 파일은 읽지 않음 |

</frozen-after-approval>

## Code Map

- `realtime/pem_explorer_record.py` -- profile/프레임/선택·stage metadata와 하위 호환 compact binary 계약.
- `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py` -- 생산 texture와 exhaustive-only shadow texture의 연산 경계.
- `tools/capture_pem_explorer.py`, `realtime/run_longcircle2_sam_pem_explorer_full.yaml` -- trusted bag 순차 offline 수집 진입점.
- `tools/serve_pem_explorer.py`, `tools/pem_explorer_live/*` -- 전체-frame report/frame/candidate API, RAM LRU, legacy 동일형 UI.
- `tests/test_pem_explorer_v2.py`, `tests/test_pem_explorer_live.py` -- profile·저장·상태·API·보안·DOM 회귀.

## Tasks & Acceptance

**Execution:**
- [x] profile-aware 후보 계측 및 recorder manifest/index/frame 계약을 추가한다.
- [x] 검증된 bag을 순차 복원해 `Sam6DCore`로 전달하는 새 offline CLI와 full config를 구현한다.
- [x] report/frame/candidates/analysis API에 전체 frame, 실제/geometry/shadow/pseudo-GT/measurement metadata를 추가한다.
- [x] live 자산을 legacy landing+iframe, slider, 9 cards, detail/candidate/evidence Canvas 구조로 교체한다.
- [x] fixture/HTTP/headless Chrome 및 기존 Explorer 회귀를 자동 검증한다.

**Acceptance Criteria:**
- Given full profile, when capture completes, then bag/processed count와 첫·마지막 stamp가 모두 일치하고 300 texture가 measured이며 후보 자산 파일은 생성되지 않는다.
- Given realtime profile, when 생산 mask filter가 일부 texture만 계산하면, then 나머지는 NaN+`texture_measured=false`이고 shadow texture 함수 호출 수가 늘지 않는다.
- Given localhost report, when frame/object/candidate를 선택하면, then 전체 bag 상태와 실제 pose/geometry Top-1/300 후보 및 네 종류 Canvas 근거가 legacy 구조에서 갱신된다.
- Given 기존 완료/partial/legacy/hostile run, when discovery/API를 요청하면, then 하위 호환 표시 또는 fail-closed 거부가 유지된다.

## Spec Change Log

## Design Notes

binary record 폭은 80 bytes를 유지하고 texture 측정 여부는 후보 flag bit로 표현해 기존 v2를 그대로 읽는다. 후보 GT와 legacy shadow 결과는 저장 R/t와 frame reference pose로 서버가 계산한다. full frame 목록은 trusted SQLite timestamp와 `frames.jsonl`을 합쳐 profile에 따라 미처리와 검출 없음을 분리한다.

## Verification

**Commands:**
- `python3 -m pytest -q tests/test_pem_explorer_v2.py tests/test_pem_explorer_live.py tests/test_pem_explorer.py` -- compact/profile/API/UI/static 회귀 통과.
- `python3 tools/capture_pem_explorer.py --config realtime/run_longcircle2_sam_pem_explorer_full.yaml` -- 실제 bag 2,130/2,130 및 완료 manifest.
- `google-chrome --headless --disable-gpu --screenshot ...` -- landing/iframe/slider/9 cards/table/panels/Canvas 렌더 확인.

**Results:**
- 전체 테스트 `223 passed`, 알려진 `pynvml` deprecation warning 1건.
- exhaustive 산출물 `2,130/2,130`, 첫/끝 stamp 일치, 실제 통과 pose 1,783건.
- PEM 시도 2,994건 중 300후보 시도 2,979건 모두 texture 300개 측정.
- 시도당 최대 binary 34,657 bytes, 전체 binary 103,243,203 bytes, 후보 자산 파일 0개.
- 동적 재계산 최대 절대오차 0.000237, 분석 전후 산출물 SHA-256 목록 동일.
- headless Chrome에서 9 cards, 300 rows, XYZ pose 축과 4 Canvas의 nonzero pixel 검증.

## Suggested Review Order

**수집 profile과 선택 불변성**

- 드롭 없는 trusted bag 순차 수집과 고정 output 경계를 시작점으로 본다.
  [`capture_pem_explorer.py:298`](../tools/capture_pem_explorer.py#L298)

- 생산 선택 후 exhaustive만 texture shadow를 채우고 realtime은 측정 상한을 지킨다.
  [`model_utils.py:830`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L830)

- 80-byte 후보와 frame/stage/actual metadata를 append-only로 저장한다.
  [`pem_explorer_record.py:204`](../realtime/pem_explorer_record.py#L204)

**완료·보안·비교 정책**

- 완료 run의 index/binary/frame/stamp/texture 계약을 제공 전에 검증한다.
  [`serve_pem_explorer.py:161`](../tools/serve_pem_explorer.py#L161)

- 전체 bag frame을 profile별 처리·드롭 상태와 9객체 slot으로 결합한다.
  [`serve_pem_explorer.py:498`](../tools/serve_pem_explorer.py#L498)

- 대칭-aware pseudo-GT와 legacy shadow fallback을 생산 결과와 분리한다.
  [`serve_pem_explorer.py:395`](../tools/serve_pem_explorer.py#L395)

- 검증된 bag/model provenance로 클릭한 후보 한 건만 RAM에서 재계산한다.
  [`analyzer.py:249`](../tools/pem_explorer_live/analyzer.py#L249)

**Legacy UI 상호작용**

- 생산 pose XYZ 축과 선택 후보 투영을 같은 RGB overlay에 합성한다.
  [`app.js:43`](../tools/pem_explorer_live/app.js#L43)

- 탈락·0후보·300후보를 실제/Geometry Top-1/pseudo-GT 상세로 렌더링한다.
  [`app.js:148`](../tools/pem_explorer_live/app.js#L148)

- 후보별 비동기 재계산을 request token으로 격리해 stale Canvas를 막는다.
  [`app.js:214`](../tools/pem_explorer_live/app.js#L214)

**회귀와 실행 설정**

- capture ON/OFF 선택 동일성과 profile별 texture 호출 수를 고정한다.
  [`test_pem_explorer_profiles.py:86`](../tests/test_pem_explorer_profiles.py#L86)

- header stamp/drop/frame JPEG와 hostile 경계를 fixture로 검증한다.
  [`test_pem_explorer_live.py:122`](../tests/test_pem_explorer_live.py#L122)

- exhaustive exact completion과 texture completeness를 고정한다.
  [`test_pem_explorer_v2.py:195`](../tests/test_pem_explorer_v2.py#L195)

- full/realtime config가 서로 다른 계측 profile을 명시한다.
  [`run_longcircle2_sam_pem_explorer_full.yaml:28`](../realtime/run_longcircle2_sam_pem_explorer_full.yaml#L28)
