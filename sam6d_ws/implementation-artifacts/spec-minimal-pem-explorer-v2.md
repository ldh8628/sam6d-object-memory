---
title: '최소 저장형 PEM Explorer v2 및 longcircle2_manual 연동'
type: 'feature'
created: '2026-08-24'
status: 'done'
baseline_commit: '03b744f6fce30eea9731236122b9be2692562b90'
context:
  - 'implementation-artifacts/spec-reusable-pem-pose-explorer.md'
  - 'implementation-artifacts/spec-longcircle2-sam-camera-explorer.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 현재 정적 Explorer는 후보별 근거를 거대한 JSON/이미지로 미리 생성하며, 일반 split 실행은 통과 pose만 남겨 Mask·Texture·수렴·Fine 단계에서 탈락한 PEM 시도와 300개 후보를 사후 조사할 수 없다.

**Approach:** 실시간 선택 수학을 바꾸지 않는 opt-in 계측으로 모든 PEM 시도와 300개 후보의 최소 수치·재생 index를 고정폭 binary에 기록하고, localhost 서버가 선택 후보 한 건의 bag RGB-D·PEM feature·CAD 비교를 즉석 재계산하는 Explorer v2를 제공한다.

## Boundaries & Constraints

**Always:** `detections.jsonl`은 통과 결과만 기록하는 기존 계약을 유지한다. 6,000 생성과 residual 상위 300 정렬·선택 값 및 proposal ID는 계측 OFF 기준과 동일해야 한다. 후보 R/t·원점수는 float32 bit를 보존하며 PEM 시도당 40 KiB 이하를 목표로 한다. 탈락 객체도 index에 남기고 PEM 입력이 존재한 프레임의 label PNG는 통과 여부와 무관하게 한 번만 쓴다. manifest는 bag/config/checkpoint/template/CAD provenance와 완료 상태를 검증 가능하게 기록한다. 기존 정적 builder/report와 비추적 manifest/checksum을 보존한다.

**Ask First:** 기존 대용량 보고서 또는 run을 교체·삭제해야 하는 경우, 생산 후보 선택에 shadow texture를 반영하는 경우, 후보별 이미지/feature tensor를 디스크에 저장해야 하는 경우.

**Never:** symlink·절대경로·`..` traversal을 통해 `output/` 밖을 제공하지 않는다. 미완료 Explorer run을 완료 run처럼 표시하지 않는다. 재계산 mismatch를 숨기거나 partial split에 300개 후보가 있다고 표시하지 않는다. 동적 분석 중 후보별 PNG/feature 파일을 생성하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Explorer v2 | 완료된 `output/<run>` | frame/object/300 후보 탐색과 후보 클릭 동적 비교 | 저장 점수와 재계산 차이는 provenance 경고 |
| Partial split | `detections.jsonl`만 있는 일반 run | 최종 pose만 열고 후보 미수집 사유 표시 | 분석 endpoint 비활성화 |
| Legacy report | 정적 `index.html`/완료 marker | iframe/새 창으로 기존 report 유지 | `file://`에서는 localhost 안내 |
| Missing/incomplete | run 부재 또는 `completed:false` | 명확한 상태·재개/재실행 안내 | 동적 분석 거부 |
| Hostile path | symlink/absolute/traversal | 어떤 파일도 읽거나 제공하지 않음 | 400/403/404로 통제된 거부 |

</frozen-after-approval>

## Code Map

- `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py` -- 6,000→300, Mask/Texture/수렴 후보 계측의 유일한 선택 경계.
- `sam6d_master/SAM-6D/Pose_Estimation_Model/run_inference_custom.py` -- 동일 관측점과 CAD 표본 index를 얻는 입력 샘플링 경계.
- `realtime/sam6d_core.py`, `realtime/sam6d_infer.py` -- 통과/탈락 PEM 결과와 frame mask를 recorder에 전달하는 실행 경계.
- `realtime/pem_explorer_record.py` -- v2 manifest/index/fixed-width binary writer·reader 계약.
- `tools/serve_pem_explorer.py`, `tools/pem_explorer_live/` -- 안전한 run discovery, bag replay 분석 API, Worker/Canvas UI.
- `realtime/launch/sam6d_split.launch.py` -- 외부 bag 재생에서도 inference idle 종료를 launch 전체 종료로 전파.

## Tasks & Acceptance

**Execution:**
- [x] PEM 입력·후보 계측 -- source pixel/FPS/CAD index와 raw 300 후보 수치, selection 이후 shadow texture를 opt-in으로 노출한다.
- [x] recorder와 `realtime/run_longcircle2_sam_pem_explorer.yaml` -- manifest/index/binary/mask 및 정상 완료 처리를 구현한다.
- [x] server/UI -- 안전한 run 분류와 v2/partial/legacy 화면, 선택 후보의 수치 배열·bitmask 동적 분석, RAM LRU·timing/mismatch 표시를 구현한다.
- [x] split launch -- `bag.play:false`에서도 infer 종료 시 전체 launch를 정상 종료한다.
- [x] 자동 테스트 -- binary bit round-trip·용량·탈락 보존·path 공격·run 분류·partial/legacy·launch handler를 고정한다.

**Acceptance Criteria:**
- Given 동일 seed/input에서 계측을 켜고 끈 실행, when 후보 생성을 비교하면, then 6,000 residual 상위 300의 값·proposal ID·선택 결과가 일치한다.
- Given Mask/Texture/수렴/Fine 탈락 객체, when run을 종료하면, then Explorer index에는 보이지만 `detections.jsonl`과 ROS 발행에는 포함되지 않는다.
- Given 실제 bag 후보를 클릭하면, when 서버가 replay index로 비교를 계산할 때, then geometry/texture/mask 수치 배열만 반환하고 브라우저 Canvas가 표시하며 디스크 후보 자산은 늘지 않는다.

## Spec Change Log

## Design Notes

후보 record는 float32 R9+t3+geometry/Mask/texture/coverage/size와 정수 proposal/index/rank/flags를 고정폭으로 둔다. replay record는 원본 관측 pixel index, 196 FPS index, 1,024 CAD sample index만 보존하고 bbox/K/mask 위치는 JSON index가 참조한다. 서버는 공용 checkpoint/template/CAD를 시작 시 적재하고 객체 feature만 RAM LRU에 유지한다.

## Verification

**Commands:**
- `python -m pytest -q tests/test_pem_explorer_v2.py tests/test_pem_explorer.py tests/test_pem_candidate_generation_regression.py` -- 신규 계약과 기존 정적 report 회귀 통과.
- `python tools/serve_pem_explorer.py --root output --host 127.0.0.1 --port 8765 --no-model` -- run 분류/UI/API smoke 통과.
- 실제 `longcircle2_sam` 연구 실행 -- binary round-trip, 탈락 보존, 환산 용량 및 가능한 장비에서 p50/p90 측정.

**Results:**
- 집중 테스트 79개와 전체 테스트 199개 통과.
- 실제 run 196회 시도/58,500후보, 통과 114/탈락 82, 전체 8.0 MiB.
- full attempt 34,657 bytes, 모든 binary record byte round-trip 일치.
- 동적 분석 cold p50 14.0 ms/p90 24.1 ms, warm p50 3.5 ms/p90 9.0 ms.

## Suggested Review Order

**후보 선택 불변성과 계측 경계**

- 선택 이후 shadow 점수를 채워 생산 300후보 결정을 보존한다.
  [`model_utils.py:824`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L824)

- 고정폭 저장 payload를 raw float32와 replay index로 조립한다.
  [`model_utils.py:761`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L761)

- coarse 결과를 모델 출력까지 명시적으로 전달한다.
  [`coarse_point_matching.py:85`](../sam6d_master/SAM-6D/Pose_Estimation_Model/model/coarse_point_matching.py#L85)

- crop 표본을 원본 frame pixel index로 무손실 변환한다.
  [`run_inference_custom.py:200`](../sam6d_master/SAM-6D/Pose_Estimation_Model/run_inference_custom.py#L200)

- 통과와 무관하게 PEM 시도를 recorder payload에 연결한다.
  [`sam6d_core.py:298`](../realtime/sam6d_core.py#L298)

**저장 계약과 실행 완료**

- append-only manifest/index/binary와 frame bitmask를 원자적으로 기록한다.
  [`pem_explorer_record.py:139`](../realtime/pem_explorer_record.py#L139)

- binary offset·길이를 검증하며 선택 시도 하나만 복원한다.
  [`pem_explorer_record.py:269`](../realtime/pem_explorer_record.py#L269)

- 신뢰 bag tail과 실시간 drop window로 완료 상태를 판정한다.
  [`sam6d_infer.py:66`](../realtime/sam6d_infer.py#L66)

- dual RGB/depth stamp를 shared-memory seqlock에 함께 전달한다.
  [`shm_channel.py:27`](../realtime/shm_channel.py#L27)

- infer 종료를 외부 bag 모드의 launch 전체 종료로 전파한다.
  [`sam6d_split.launch.py:35`](../realtime/launch/sam6d_split.launch.py#L35)

**localhost 분석과 UI**

- run 종류·schema·binary 크기·경로를 fail-closed로 분류한다.
  [`serve_pem_explorer.py:50`](../tools/serve_pem_explorer.py#L50)

- READY 전에 실제 후보로 전체 분석 kernel을 워밍업한다.
  [`serve_pem_explorer.py:142`](../tools/serve_pem_explorer.py#L142)

- bag RGB-D와 저장 index로 후보 하나의 수치 근거만 재계산한다.
  [`analyzer.py:217`](../tools/pem_explorer_live/analyzer.py#L217)

- v2·partial·legacy 상태와 비동기 선택을 안전하게 표시한다.
  [`app.js:8`](../tools/pem_explorer_live/app.js#L8)

**검증과 연구 설정**

- bit 정확성·보안 경계·실제 propagation 회귀를 고정한다.
  [`test_pem_explorer_v2.py:41`](../tests/test_pem_explorer_v2.py#L41)

- `longcircle2_manual` 연구 실행을 opt-in 설정으로 격리한다.
  [`run_longcircle2_sam_pem_explorer.yaml:35`](../realtime/run_longcircle2_sam_pem_explorer.yaml#L35)
