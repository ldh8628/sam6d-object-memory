# Deferred Work

## Pre-existing realtime rejection diagnostics

- `realtime/sam6d_core.py` computes mask/depth radius rejection diagnostics for every ISM hit even when the new PEM explorer diagnostic option is disabled.
- This behavior predates the PEM explorer work and supported the earlier rejection-recovery experiment, so it was not changed in this story.
- A focused follow-up should benchmark it, define whether `frames.jsonl` must retain these fields in normal operation, and gate or streamline the calculations without losing the existing recovery diagnostics.

## Pre-existing PEM diagnostic parity coverage

- The prior reusable explorer story requires diagnostic-off numerical and performance parity, but the current test suite has no retained seeded baseline test that compares PEM R/t/score and overhead with diagnostics disabled.
- This was not introduced by the SAM-camera dataset switch. A focused follow-up should restore a small deterministic parity fixture and define an explicit overhead budget.

## Pre-existing explorer bundle scalability and HTML hardening

- `tools/build_pem_explorer.py` serializes every frame and candidate into one executable `report-data.js`; after repeated-field compaction the full top-100 SAM report is still about 443 MB of JavaScript and 34 GiB including evidence assets.
- The evidence HTML path interpolation and same-origin unsandboxed landing iframe also deserve a focused security pass even though generated asset names are currently builder-controlled.
- A follow-up should introduce frame-level lazy loading/non-executable JSON, bounded bundle tests, DOM-based image construction, and an iframe sandbox compatible with local `file://` use.

## Candidate-analysis provenance hardening

- The Shadow builder now rejects mismatched detection keys, duplicate/implicit ranks, and a geometry-winner proposal that differs from the detection source. The generated policy is therefore bound to the selected proposal and report fingerprint.
- A malicious or accidentally stale full-stage300 file could still preserve every detection key and winner proposal while changing only nonwinning alternatives. This requires a separate versioned analysis manifest or canonical candidate-analysis digest carried through the diagnostic provenance, policy, and report rather than another local field check.

## Multi-file diagnostic transaction

- `temp/verify_eval.py` fsyncs each JSONL and the containing directory, but detections, score analysis, frames, and provenance are still published as multiple renames.
- A process failure between those renames can leave a mixed generation when reusing an output directory. A focused change should publish a complete run directory with an atomic version pointer/directory exchange and make all consumers require the completion provenance.

## Pre-existing nonfinite geometry winner policy

- `independent_candidate_verify` and `sequential_candidate_select` inherit the production geometry ordering behavior for nonfinite scores; a synthetic NaN/Inf geometry value can become the raw argmax/Geometry Top-1 before later validity gates reject it.
- The compact Explorer deliberately preserves the production index/proposal/R/t selection contract and the captured longcircle2 candidates are finite, so this story did not change production ordering semantics. The live GT/shadow comparison now treats malformed poses/evidence as unavailable rather than correct.
- A focused follow-up should define an authoritative nonfinite geometry policy, update both production selection paths together, and version the behavior with diagnostic-OFF parity fixtures before changing winners.

## Pre-existing ObjectMemory Explorer media error and playback accessibility

- `integration/object_memory_explorer/app.js`의 `seekVideo()`는 기존부터 `seeked`만 기다려 preview decode 오류가 발생하면 대기가 끝나지 않을 수 있다.
- 재생 중에도 기존 `#play` 버튼의 `aria-label`은 `재생`으로 유지되어 화면 읽기 프로그램에 실제 동작과 반대 액션을 알릴 수 있다.
- 이번 raw/fused 비교 변경에서 새로 생긴 문제는 아니므로, 별도 접근성·media error 처리 작업에서 오류 이벤트와 동적 label을 함께 보완한다.
