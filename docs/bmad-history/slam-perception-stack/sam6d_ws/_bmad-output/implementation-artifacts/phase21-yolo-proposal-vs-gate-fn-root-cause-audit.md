# Phase 2.1 FN Root-Cause Audit
## YOLO-World Proposal 품질 vs 후단 Gate 과제거 — 인과 분리 검증

- 작성일 2026-07-22 · Ldh9501 · `/bmad-quick-dev` · **원인규명 전용, 운영 무변경, git 무조작**
- 산출물 `outputs/phase21_fn_root_cause_audit/` · 명세 `spec-phase21-yolo-proposal-vs-gate-fn-root-cause-audit.md`
- 데이터: 사람 GT=frame-level visibility 339프레임(935 positive cell). **GT bbox/mask 부재** → 사용자 결정(Option 1)에 따라 GT-free 인과분석 + best-ROI oracle 수행, IoU/CF-1·2·3는 `not_available`(annotation CSV 발행).

---

## 첫 페이지 — 15문답 (필수)

| # | 질문 | 답 |
|---|---|---|
| 1 | Per-prompt recall 0.959는 단순 BBox 존재율인가? | **그렇다.** `eval_yolo_bbox_recovery.recall_fp` — (frame,object) 셀에 conf≥0.02 box ≥1개면 hit. **IoU 미사용, frame-object level BBox-existence recall.** |
| 2 | IoU 0.5 기준 proposal recall은? | **not_available** (GT bbox 부재, deferred). |
| 3 | IoU 0.7 기준 proposal recall은? | **not_available**. |
| 4 | FN 313 중 YOLO raw miss(후보 자체 없음)는? | **38개 (12.1%)** = all-gate bypass 후에도 남는 FN floor. |
| 5 | 좋은 raw proposal이 confidence에서 제거된 FN은? | conf 하한 0.02 아래 목표박스 존재(Phase2 F1급) 소수 — cur(per-prompt)에선 이미 흡수, 잔여 proposal-miss 38에 포함(별도 IoU 확인은 deferred). |
| 6 | 좋은 proposal이 NMS에서 제거된 FN은? | cur=per-prompt라 shared-NMS 억제는 해소됨. 잔여 38 중 NMS-특정 분해는 GT bbox 필요(deferred). |
| 7 | final BBox 있으나 IoU 낮은 FN은? | GT bbox 부재로 IoU 직접판정 불가. **간접**: best-ROI oracle에서 "어떤 box로도 gate 통과 못함"=224 (아래 15번). |
| 8 | 좋은 BBox 있으나 mask/ROI 나쁜 FN은? | GT mask 부재로 U4/U5 `not_available`. 간접: oracle에 MobileSAM mask 포함되어 gate 통과 여부에 반영됨. |
| 9 | 유효 proposal·ROI 있으나 **semantic**에서 제거된 FN은? | **141 (RC8, 45.0%)** — 단, oracle 재분류로 일부는 selection. |
| 10 | **appearance**에서 제거된 FN은? | **106 (RC9, 33.9%)**. |
| 11 | **HSV**에서 제거된 FN은? | **23 (RC10, 7.3%)**. |
| 12 | GT BBox를 넣으면 TP가 몇 개 증가? | **측정불가(deferred)** — GT bbox 필요. 대체지표: all-gate bypass는 +275 TP, best-ROI oracle은 +51만 복구(나머지 224는 어떤 box로도 gate 미통과). |
| 13 | 모든 gate 우회 시 TP·FP 증가? | **TP +275 (622→897), FP +1158 (86→1244)**, F1 0.7572→0.5832. gate 제거는 recall을 사지만 precision을 파괴. |
| 14 | GT BBox + all-gate bypass 후 남는 FN은? | GT bbox 없이 정확산출 불가. 상한 근사: all-gate bypass 잔여 FN = **38**(proposal-miss floor). |
| 15 | **최종 병목은 proposal vs gate?** | **후단 GATE (판정 B).** 최종 인과분할 **proposal 38(12.1%) / selection 53(16.9%) / gate 222(70.9%)**. class-agnostic best-ROI oracle로도 275 gate-FN 중 **224(81.5%)는 어떤 proposal로도 gate 미통과** → proposal 개선으로 복구 불가. |

**한 줄 결론:** FN의 주 병목은 YOLO proposal이 아니라(12%) **후단 gate(71%)** 이며, 더 좋은 proposal을 줘도(oracle) 224개는 gate가 막는다. 단 gate를 풀면 FP가 13배 폭증 → 본질은 **gate에서의 precision-recall 트레이드오프**이지 gate 버그가 아니다.

---

## 1. 전체 판정 · Phase 1C 재현

- **PASS** (핵심 인과질문 확정). IoU/CF-1·2·3는 GT bbox 부재로 **PARTIAL(deferred)** — 사용자 승인(Option 1).
- Phase 1C 정본 재현(cur pairs + cur/{ds}_hsv.npz 슬라이스): **TP 622 / FP 86 / FN 313 / F1 0.7572** 정확 일치(assert 통과). 이 313이 원인분석 권위 집합.

## 2. AC-01 — "0.959"의 정확한 정의

`_ism_research_2026_07/phase2_bottleneck/eval_yolo_bbox_recovery.py:recall_fp` — 셀당 conf≥0.02 box ≥1개면 hit. **BBox-existence recall(IoU 미사용, frame-object level, 분모=935 visible cell)**. 3개념 중 #1(존재)만 측정, #2(IoU) 미측정, #3(usable)은 oracle로 간접.

| proposal mode | BBox-existence recall | IoU@0.3/0.5/0.7 |
|---|---|---|
| shared960 (운영 main) | 0.8257 | not_available |
| perprompt960 (cur/평가·PEM) | **0.9594** | not_available |
| perprompt1280 | 0.9711 | not_available |

## 3. FN 313 배타적 원인 (결정트리, Σ=313, unknown 0%)

파이프라인 gate 순서 우선순위 RC-1→RC-8→RC-9→RC-10→RC-11.

| primary_root_cause | count | % | group |
|---|---|---|---|
| RC1 no_candidate | 38 | 12.1 | proposal |
| RC8 semantic | 141 | 45.0 | gate |
| RC9 appearance | 106 | 33.9 | gate |
| RC10 hsv | 23 | 7.3 | gate |
| RC11 selection | 5 | 1.6 | selection |
| RC12 unknown | 0 | 0.0 | — |

**AC-03 충족**: 전수 분류, Σ=313, unknown 0%(≤5%).

## 4. best-ROI oracle — gate-strictness vs selection 분리 (GT-free 핵심)

RC8/9/10+RC11 = 275 gate/selection FN의 프레임에서 **모든 per-prompt proposal(class-agnostic union, conf≥0.02)**을 target object 템플릿으로 ISM(semantic+appe+HSV) 스코어 → gate 통과 ROI 존재 여부:

- **usable ROI 존재(selection-causal): 51 / 275 (18.5%)** — 좋은 box가 프레임에 있었으나 운영 selection/routing이 놓침. proposal이 아니라 **선택** 문제(gate·GT 불필요로 개선 가능).
- **gate/feature-limited: 224 / 275 (81.5%)** — 프레임의 **어떤 box로도** 현재 gate를 통과 못함. localization 문제 아님(더 좋은 box도 소용없음) → **gate 임계 또는 feature 변별력**이 binding.

## 5. 최종 인과분할 (oracle 병합, Σ=313)

| group | count | % | 의미 |
|---|---|---|---|
| **gate** | **222** | **70.9** | 어떤 proposal로도 gate 미통과(feature/threshold binding) |
| selection | 53 | 16.9 | usable box 존재하나 미선택(RC11 5 + oracle 48) |
| proposal | 38 | 12.1 | usable proposal 자체 없음 |

## 6. 반사실(CF-0/CF-4) — gate의 인과효과

| 구성 | TP | FP | FN | F1 | FN복구 | FP재유입 |
|---|---|---|---|---|---|---|
| CF-0 actual | 622 | 86 | 313 | .7572 | 0 | 0 |
| CF-4 semantic bypass | 650 | 98 | 285 | **.7724** | 28 | 12 |
| CF-4 appearance bypass | 713 | 242 | 222 | .7545 | 91 | 156 |
| CF-4 hsv bypass | 648 | 334 | 287 | .6761 | 26 | 248 |
| CF-4 sem+appe bypass | 822 | 493 | 113 | .7307 | 200 | 407 |
| CF-4 appe+hsv bypass | 756 | 630 | 179 | .6514 | 134 | 544 |
| CF-4 **all gates bypass** | 897 | 1244 | **38** | .5832 | **275** | 1158 |

**해석**:
- all-gate bypass FN=**38** = proposal-miss floor(4번 재확인). 나머지 275는 gate가 만든 FN.
- gate별 TP-복구 대비 FP-비용: semantic 28:12(유일하게 F1↑, 다소 과엄격), appearance 91:156, **HSV 26:248**(FP killer, 설계대로). gate를 풀면 recall↑지만 FP 13배 → gate는 **버그가 아니라 PR 운영점**.

## 7. class별 원인 (final partition)

| class | FN | proposal | selection | gate |
|---|---|---|---|---|
| Sauce_high | 51 | 3 | 12 | 36 |
| Dinosaur | 49 | 2 | 5 | **42** |
| Rabbit | 38 | 8 | 8 | 22 |
| Bear | 37 | 2 | 6 | 29 |
| choco_hazelnut | 33 | **11** | 4 | 18 |
| milk | 32 | **10** | 3 | 19 |
| Mugcup_high | 26 | 1 | 3 | 22 |
| Sikhye_high | 18 | 0 | 5 | 13 |
| Febreze_high | 15 | 1 | 4 | 10 |
| saffron | 14 | 0 | 3 | 11 |

- **gate가 전 클래스 지배**. dolls(Dinosaur 42/49, Bear 29/37)·rigid(Sauce 36, Mugcup 22)에서 특히 gate-limited(=feature 변별력 부족, 도메인갭).
- **proposal은 choco(11)/milk(10)/Rabbit(8)** 에서만 유의(작은/원거리/가려진 포장) — 그래도 소수.
- bag별 표는 `csv/fn_root_cause_by_bag.csv`.

## 8. AC-06 최종 병목 판정

**B. 후단 gate가 주요 FN 병목** (class 편중 있으나 전 클래스 gate 우세이므로 순수 C 아님). proposal은 최소(12%), 그중 대부분은 특정 포장류. gate 71% + selection 17% = 후단 88%.

## 9. AC-07 이전 두 결론이 달라진 이유 (정정)

- **결론 A("YOLO proposal이 주원인")는 정정된다.** 원인은 (i) **proposal 성공 정의 차이** — A는 shared-pass BBox-existence(.826) 또는 IoU-엄격/초기 파이프라인을, B/본 감사는 per-prompt BBox-existence(.959)를 사용. (ii) **shared vs per-prompt 경로 차이** — shared 단일패스는 NMS로 recall .826, per-prompt는 .959. (iii) **object-level 분모 확정** — 본 감사는 935 visible cell + 정본 313 FN에 결정트리+oracle 적용.
- **결론 B("gate 과제거가 주원인")는 확정·정밀화된다.** 단 "gate가 좋은 box를 잘못 제거"라는 순진한 해석은 부분정정: oracle상 275 중 224는 **어떤 box로도 통과 못하는 feature/threshold 한계**(단순 localization/selection 아님), 51만 selection. 그리고 gate를 풀면 FP 13배 → gate 자체는 정상 PR 운영점.
- 평가코드 오류·GT 버전 문제·실제 병목 변화는 아님(정본 313 정확 재현).

## 10. AC-04/deferred — GT bbox 필요 항목

CF-1(GT bbox oracle)/CF-2(GT mask)/CF-3(best-IoU oracle)/IoU recall = **not_available**. FN 313 전수 + TP/FP control 373행을 `csv/yolo_fn_root_cause_human_bbox_gt.csv`(gt_bbox 빈칸)로 발행. 사람 bbox 입력 후 재실행하면 이 항목들이 채워진다. **자동 GT/pseudo-GT는 사용 안 함(fabricate 금지).**

## 11. 변경 파일 · rollback

신규(연구 경로만): `_ism_research_2026_07/phase21_fn_audit/{phase21_common,audit_recall_definition,build_fn_object_index,run_fn_counterfactuals,emit_annotation_targets,make_phase21_figures}.py`, `tests/test_phase21_rootcause.py`, `outputs/phase21_fn_root_cause_audit/**`, 본 보고서/spec.
**운영 변경 없음**(config·`yolo_ism*.py`·`ism_hsv.py` 판정 diff 0). rollback=산출물 폴더 삭제(운영 영향 0).

## 12. 남은 위험

- IoU/CF-1·2·3는 GT bbox 없이는 미측정(현재 deferred). "gate-limited 224" 중 "gate 과엄격" vs "feature 근본부족"의 최종 분리는 GT bbox + 임계 sweep 필요.
- best-ROI oracle은 현재 gate 임계 기준 → "gate 임계를 낮추면 통과할 box"는 gate-limited로 분류됨(설계상 gate-causal 맞음).
- GT는 관대한 visibility 기준 → recall 하한 성격. dolls 소표본.

## 13. 최종 구현 권장 (원인규명 → 다음 단계)

1. **최우선 = gate feature 변별력**(222 gate-FN): 색/texture 조정이 아니라 **더 강한 appearance/semantic 특징**(예: block별 class-wise, 다중뷰 verification, calibrated head)으로 참객체가 FP 없이 gate를 넘게 함. 단순 임계 완화는 FP 13배(§6)로 불가.
2. **차선 = selection/routing 개선**(53): usable box가 있으나 놓친 케이스 — per-prompt 유지 + best-by-sem 대신 gate-aware 재선택.
3. **proposal(38)은 최하위**: choco/milk/Rabbit 소형·가림 한정. YOLO 튜닝 이득 상한 12%.
4. **YOLO proposal 대규모 투자 지양** — oracle이 224를 복구 못함을 실증.

---

## 부록 산출물
- csv: `gt_object_index.csv`(3390), `fn_root_cause_{per_object(313),summary,by_class,by_bag}.csv`, `counterfactual_results.csv`, `fn_oracle_reclass.csv`, `proposal_definition_recall.csv`, `yolo_fn_root_cause_human_bbox_gt.csv`(373, deferred)
- metrics: `root_cause_metrics.json`, `counterfactual_metrics.json`, `proposal_metrics.json`, `final_causal_partition.json`
- figures: `outputs/phase21_fn_root_cause_audit/figures/`(funnel/원인bar/final partition/CF/oracle/recall정의/class stacked/초보자요약)
- 재현: `~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase21_fn_audit/{build_fn_object_index, run_fn_counterfactuals --with-gpu, audit_recall_definition, emit_annotation_targets, make_phase21_figures}.py`
