# PRD Rubric Review — SAM-6D Pose Estimation 신뢰성 개선

- **Target:** `_bmad-output/planning-artifacts/prds/prd-sam6d_ws-2026-06-08/prd.md`
- **Reviewer:** PRD quality reviewer (read-only)
- **Date:** 2026-06-08
- **Context:** Technical/engineering PRD — two bug fixes (false-positive detection with no object present; pose jitter on static scenes). Grounded in RCA doc `docs/sam6d-pose-rca-analysis-2026-06-08.md` (verified present). Single operator, internal tooling.

## Overall Gate: CONDITIONAL PASS

The PRD is unusually well-grounded (every FR/problem carries concrete code anchors) and scope is coherent. It is **not** blocked, but it has **two High-severity gaps that will cause friction at the architecture/implementation step**: (a) the central success metric "FP 감소율 ≥ 90%" is unfalsifiable because no baseline FP rate is yet measured and the threshold-tuning step that sets the floor is the same step expected to prove the floor works (circularity), and (b) NFR-1 "true positive recall 저하 없음" has no defined dataset, ground-truth labels, or numeric tolerance, so AT-2 cannot actually be executed. Resolve these before passing to architecture.

---

## Dimension Verdicts

### 1. Completeness — PASS
All standard PRD sections are present and substantive: Background, Problem Statement (split A/B), Functional Requirements (9 FRs across 3 features), NFRs (4), Success Criteria, Acceptance Tests (7, tabular), Implementation Scope (Must/Should/Nice), Risk (6 items with mitigations), Deliverables, and a Resolved Decisions log. No section is a placeholder. Minor: there is no explicit "Out of Scope / Non-Goals" section distinct from Nice-to-Have, and no rollback/feature-flag-off plan beyond "기본값 off" scattered in NFR-3.

### 2. Testability — WEAK
Most ATs are measurable and reference the FR-9 CSV/JSON as the measurement source, which is good. But several success conditions are non-falsifiable as written:
- AT-2 ("true positive recall 저하 없음") has no labeled dataset, no recall tolerance, and "정답 라벨" is assumed to exist but is never confirmed — the `output/sam6d_milk_verify/` baseline is not stated to carry ground-truth pose labels.
- AT-3/AT-4 success = "기준선 대비 대폭 감소 (목표 충족)" — "대폭"/"충족" defer to numbers that do not yet exist; the concrete `[ASSUMPTION]` targets (σ_t ≤ 1 mm, σ_R ≤ 0.5°) are flagged as examples, not commitments.
- AT-5 success = "출력 동일성↑ ... 충분히 안정이면 합격" — "충분히 안정" has no threshold; this is a subjective pass condition.
- AT-1's "≥ 90% 감소" depends on a baseline FP rate that is explicitly still unmeasured.

### 3. Scope clarity — PASS
Must/Should/Nice is coherent and consistent with the FRs and Risk. The split is defensible: detection/floor fixes (FR-1–4) + seed restoration (FR-8) + debug (FR-9) as Must directly address the two reported bugs' primary causes; temporal smoothing/outlier (FR-5–7) as Should; multi-instance association and full per-frame determinism as Nice. The Resolved Decisions log explicitly reconciles Scope with Risk (e.g., R-4/R-6 deferred to Nice). One inconsistency: FR-5 ("연속 프레임 pose 안정화") is a *Success-Criteria-level outcome* but is filed under Should, while FR-8 (a contributing cause to the same outcome) is Must — meaning the FR-5 outcome can only be partially met in the Must scope. Acceptable but should be called out.

### 4. Traceability — PASS
Strongest dimension. Every problem bullet and nearly every FR carries a file:line code anchor (e.g., `detector.py:286-288`, `model_utils.py:221`, `run_batch_inference_fast.py:680-689`), the basis RCA doc exists and is referenced, and the target source files exist on disk. FRs stay capability-level ("apply a minimum floor", "use pred_pose_score as a filter") while pointing at *where* without dictating *how*. The `[ASSUMPTION]` convention cleanly separates load-bearing requirements from to-be-tuned numbers.

### 5. Internal consistency — WEAK
No hard contradictions, but a circular dependency undermines the core claim. AT-1/Success Criteria assert "FP 감소율 ≥ 90%", yet FR-2's floor values are to be *derived from* the same object-absent dataset distribution used to *measure* that 90% (FR-9 → floor → AT-1). The floor is fit to the data it is then validated on, so the 90% target is self-confirming rather than an independent acceptance bar. Secondary: FR-2 says it "보완"s (supplements) the 0.2 semantic gate while Problem A frames 0.2 as the root defect — it is left implicit whether 0.2 is kept, raised, or bypassed. Also, "pose 발행 건수가 0에 수렴" (Success Criteria) vs AT-1 "발행 건수 ≈ 0" vs FR-1 "어떤 pose도 발행하지 않아야" — the absolute (zero) and asymptotic (≈0/수렴) phrasings coexist without a tie-break rule for residual false positives.

### 6. Requirement quality — PASS
FRs are predominantly capability-focused (what, not how): FR-1 judges no-object, FR-3 uses an existing score as a filter, FR-8 ensures determinism. Ambiguity is contained to `[ASSUMPTION]`-tagged numbers. Minor duplication/overlap: FR-1, FR-3, and FR-4 all converge on "drop low-confidence detections / don't publish" — FR-4 largely restates the consequence of FR-1+FR-3 and could be a clause rather than a peer FR. FR-6 leaks a mild implementation hint ("EMA 또는 저차 필터") but frames it as an example, which is acceptable for a technical PRD.

### 7. Risk coverage — PASS
All four named side effects are covered with mitigations: threshold over-rejection (R-1 → distribution-separation floor + AT-2 regression), smoothing lag (R-2 → outlier bypass + configurable coefficient), determinism perf (R-3 → opt-in + NFR-2 measurement), API regression (R-5 → default-preserving NFR-3 + AT-7). Plus R-4 (single-seed insufficiency) and R-6 (multi-instance mixing) extend coverage. Gap: R-1's mitigation leans on AT-2, which (per Dimension 2) is itself not executable yet — so the headline risk's primary guard is currently unverifiable. No mitigation owner or trigger/decision threshold is attached to any risk.

---

## Top Findings (severity-ordered)

| # | Severity | Finding | Section |
|---|----------|---------|---------|
| 1 | **High** | Core success metric "FP 감소율 ≥ 90%" is self-confirming: the confidence floor is fit from the same object-absent dataset it is then validated against — no independent acceptance bar and no measured baseline FP rate yet. | Success Criteria / AT-1 / FR-2 |
| 2 | **High** | NFR-1 "true positive recall 저하 없음" and AT-2 are not executable: no labeled positive dataset, no confirmation `sam6d_milk_verify` carries ground-truth pose labels, no recall tolerance. R-1's primary mitigation depends on this unexecutable test. | NFR-1 / AT-2 / Risk R-1 |
| 3 | **Medium** | Stability acceptance conditions are subjective: AT-3/AT-4 "대폭 감소", AT-5 "충분히 안정", with σ_t/σ_R targets flagged as examples, not commitments. Downstream cannot tell pass from fail without a number. | Acceptance Test AT-3/4/5 |
| 4 | **Medium** | Publish-count target oscillates between absolute ("어떤 pose도 발행하지 않아야", "= 0") and asymptotic ("≈ 0", "0에 수렴") with no tie-break rule for residual FPs. | FR-1 / Success Criteria / AT-1 |
| 5 | **Low** | FR-4 largely restates the consequence of FR-1+FR-3 (overlap); FR-5 outcome is Should while its contributing cause FR-8 is Must, so FR-5 is only partially attainable in Must scope. | FR-4 / FR-5 / Scope |

## Recommended pre-architecture actions
1. Measure and record the **baseline FP rate and baseline σ_t/σ_R on `sam6d_milk_verify`** first, then commit them as fixed acceptance numbers in this PRD (converts AT-1/3/4/5 from relative to absolute). Resolve the floor-fitting circularity by reserving a held-out object-absent slice for AT-1 validation.
2. Confirm whether a **labeled positive dataset with ground-truth poses** exists; if not, AT-2/NFR-1 must be redefined (e.g., agreement-with-prior-build rather than recall-vs-truth) before R-1 can be considered mitigated.
3. Pin σ_t/σ_R/jump thresholds and the AT-5 "충분히 안정" definition to concrete values (even provisional), and clarify FR-2's relationship to the existing 0.2 semantic gate (keep / raise / bypass).
