---
title: 'Phase 2.1 FN Root-Cause Audit — YOLO Proposal vs 후단 Gate 인과 분리'
type: 'chore'
created: '2026-07-22'
status: 'done'
context: []
baseline_commit: '10ec30f1ad027a5c0d05bce9cf5f0f663dfce429'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Phase 1C FN 313의 주원인이 "YOLO proposal 부재/부정확"(이전 결론 A)인지 "semantic·appearance·HSV gate의 참후보 과제거"(최근 결론 B)인지 확정되지 않았다. 두 결론은 서로 다른 proposal 성공 정의(BBox 존재 vs IoU vs downstream-usable)에서 비롯됐을 수 있어, FN 313 각각에 배타적 원인을 부여하고 반사실로 인과를 검증해야 한다.

**Approach:** Phase 1C를 정확히 재현(622/86/313)한 뒤, FN 313 전수에 우선순위 결정트리로 단일 primary_root_cause를 부여한다. 사용자 결정(Option 1)에 따라 **GT bbox 없이 가능한 전부를 수행**: (1) Phase 2의 0.959 정의를 코드로 감사(=frame-object BBox-존재 recall, IoU 아님 확인), (2) CF-4 gate bypass(semantic/appe/HSV/조합)로 각 gate의 TP-복구·FP-재유입 인과효과 측정, (3) 모든 per-prompt YOLO proposal에 ISM을 돌려 "gate를 통과 가능한 ROI가 프레임에 존재하나"를 보는 **class-agnostic best-ROI oracle**로 gate-strictness와 selection을 분리(GT 불필요). IoU recall·CF-1/2/3(GT bbox oracle)은 GT bbox가 없어 fabricate 금지 → FN 전수 annotation-target CSV만 생성하고 **deferred**로 명시. 최종 병목을 A/B/C/D/E로 판정하고 이전 두 결론을 정정한다.

## Boundaries & Constraints

**Always:**
- 운영 무변경: YOLO conf/NMS/per-prompt·shared 경로·semantic 공식·appe block·appe/HSV threshold·Dinosaur hue·texture·최종 accept 로직 전부 불변. diagnostic/eval 스크립트만 추가.
- Phase 1C 정확 재현: **cur(per-prompt) 후보 + cur/{ds}_hsv.npz 슬라이스 HSV**(eval_phase1c 경로)로 TP 622 / FP 86 / FN **313** / F1 0.7572. 이 313이 원인분석의 권위 FN 집합. (phase2 hsv_score는 ±1 차이라 원인분석엔 cur-hsv 정본 사용.)
- 배타적 분류: FN 313 각각 정확히 1개 primary_root_cause. 결정트리 우선순위 RC-1→RC-8→RC-9→RC-10→RC-11→(oracle 재분류)→RC-0/RC-12. secondary는 별도 컬럼.
- 일관성: 모든 primary_root_cause count 합 = **313** (검증 테스트로 강제). unknown(RC-12) ≤ 5%.
- 3개념 분리 유지: BBox-existence / geometrically-valid(IoU) / downstream-usable. IoU 필요 지표는 GT bbox 없이는 산출 금지.
- 데이터: 사람 GT=frame-level visibility(user_visibility_gt.csv 339프레임, 935 positive cell), 후보=cur/{ds}_pairs.csv, 재사용 augmented=phase2_candidates.csv, per-prompt raw=raw/yolo_raw/{perprompt960,perprompt1280}. 6셋(sam_105018/105314/105652/110104/110532/110633).

**Ask First:**
- IoU/CF-1/2/3를 실제로 채우기 위해 GT bbox가 필요해지면 → 이미 Option 1로 deferred 결정됨. 추가로 pseudo-GT를 만들거나 사람 annotation을 강제해야 할 상황이면 HALT.
- best-ROI oracle GPU 재생이 불가(GPU 없음/OOM)하면 HALT하고 CF-4(비-GPU)만으로 축소할지 확인.

**Never:**
- GT bbox/mask fabricate 또는 자동 pseudo-GT를 사람 GT처럼 사용. surrogate mask를 GT로 사용.
- 기존 GT·Phase1C·Phase2 output 수정/덮어쓰기. git add/commit/push. 결론을 기존 주장에 억지로 맞추기(측정대로 정정).
- 중복 원인 집계. candidate row 수를 recall 분모로 사용(분모는 object-level visible cell).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| FN 원인분류 | visible cell, 미accept | RC-1/8/9/10/11 중 정확히 1개(파이프라인 gate 순서) | 후보 없으면 RC-1, 전gate 통과인데 FN이면 RC-11 |
| best-ROI oracle | FN cell의 전 per-prompt proposal | max semantic ROI가 gate 통과? → 통과 시 selection-causal 재분류, 실패 시 gate/feature-limited | proposal 0이면 RC-1 유지 |
| CF-4 gate bypass | 전 935 cell + non-visible | gate pass 가정 시 TP복구+FP재유입 | 후보 없는 cell은 bypass해도 FN(=RC-1 floor) |
| IoU recall/CF-1/2/3 | GT bbox 필요 | **not_available** 기록 + annotation CSV 생성 | 절대 0/추정치로 채우지 않음 |
| 일관성 | 313 분류 | Σ primary_root_cause == 313 | 불일치 시 테스트 FAIL |

</frozen-after-approval>

## Code Map

- `_ism_research_2026_07/dinosaur_color_and_texture_research/research_a_dinosaur_color/evaluation/eval_phase1c.py` -- Phase 1C 정본(선택 규칙·cur-hsv 슬라이스·grid TP/FP/FN). P0 재현·FN 추출 기준.
- `_ism_research_2026_07/phase2_bottleneck/phase2_common.py` -- load_gt/load_candidates/select_best/grid_metrics/lodo (재사용).
- `_ism_research_2026_07/phase2_bottleneck/eval_phase2_integrated.py:build_p0` -- cur-hsv 정확 HSV(622/86/313) 로직 (재사용).
- `outputs/phase2_appearance_semantic_yolo_bbox/csv/phase2_candidates.csv` -- per-candidate sem/appe/hsv/gate (FN 분류 입력).
- `outputs/phase2_appearance_semantic_yolo_bbox/raw/yolo_raw/{perprompt960,perprompt1280}/{ds}.csv` -- best-ROI oracle용 전 proposal.
- `yolo_ism.py`/`yolo_ism_object_n.py` -- crop_resize_pad/dinov2_blocks_forward/segment_boxes/masked_appe/semantic (oracle GPU 스코어링 재사용, 읽기전용).
- `_ism_research_2026_07/gt_input/user_visibility_gt.csv` -- 권위 GT(visibility). 무수정.
- `_ism_research_2026_07/yolo_localization_research/pipeline/cur/{ds}_{pairs.csv,hsv.npz}` -- Phase 1C 후보·HSV 정본.

## Tasks & Acceptance

**Execution:** (전부 완료 [x])
- [x] `outputs/phase21_fn_root_cause_audit/` -- 디렉토리 트리(provenance/config_snapshot/raw/csv/metrics/overlays/figures/logs/report) + provenance(git-hash·seed·정의) 기록.
- [x] `_ism_research_2026_07/phase21_fn_audit/audit_recall_definition.py` -- Phase 2 "0.959" 계산 코드 추적, hit 조건이 frame-object BBox-존재(routed≥1, IoU 미사용)임을 코드 근거로 명시 → `metrics/proposal_metrics.json`(recall 정의 표). BBox-존재 recall(shared/perprompt960/1280) 재보고, IoU recall은 `not_available`.
- [x] `_ism_research_2026_07/phase21_fn_audit/build_fn_object_index.py` -- eval_phase1c 정본으로 935 cell TP/FP/FN 재현(622/86/313 assert), FN 313에 결정트리 primary_root_cause(RC-1/8/9/10/11) + per-gate pass flag → `csv/{gt_object_index,fn_root_cause_per_object,fn_root_cause_summary,fn_root_cause_by_class,fn_root_cause_by_bag}.csv`.
- [x] `_ism_research_2026_07/phase21_fn_audit/run_fn_counterfactuals.py` -- CF-0 재현 + CF-4(semantic/appe/HSV/pairwise/all bypass) TP복구·FP재유입·F1 + **best-ROI oracle**(전 per-prompt proposal GPU 스코어링→gate 통과 ROI 존재 여부, selection vs gate-strict 분리) → `csv/counterfactual_results.csv`,`metrics/counterfactual_metrics.json`, oracle 재분류 반영.
- [x] `_ism_research_2026_07/phase21_fn_audit/emit_annotation_targets.py` -- FN 313 + TP/FP control sample을 `csv/yolo_fn_root_cause_human_bbox_gt.csv`(bag,frame,object,routed_bbox,image_path, gt_bbox=빈칸)로 생성. IoU/CF-1/2/3는 **deferred(not_available)** 명시.
- [x] `_ism_research_2026_07/phase21_fn_audit/make_phase21_figures.py` -- FN funnel(935→후보→gate단계→TP/FN), 원인 분포 bar, CF-4 TP복구·FP재유입 bar, class별 stacked, 초보자 1장 요약. Agg·Korean·PNG+SVG.
- [x] `tests/test_phase21_rootcause.py` -- IoU/coverage/purity 계산, greedy one-to-one matching(동일class 다중instance), 결정트리 우선순위, 분류 합==313 일관성, RC-1/gate/selection 분기.
- [x] `_bmad-output/implementation-artifacts/phase21-yolo-proposal-vs-gate-fn-root-cause-audit.md` -- 첫 페이지 15문답 표 + 본문 + 30항목 터미널 출력.

**Acceptance Criteria:**
- Given eval_phase1c 정본, when P0 재현, then TP 622 / FP 86 / FN 313 / F1 0.7572 정확 일치(불일치 시 원인분석 중단).
- Given FN 313, when 결정트리 분류, then 각 FN에 1개 primary_root_cause, Σ==313, unknown ≤ 5%.
- Given CF-4 all-gate bypass, when 실행, then TP 복구수·FP 재유입수·잔여 FN(=proposal-miss floor)을 수치로 산출.
- Given best-ROI oracle, when 실행, then gate-strict-causal(어떤 ROI도 통과 못함) vs selection-causal(통과 ROI 존재하나 미선택)로 분리.
- Given IoU/CF-1/2/3, when GT bbox 부재, then 결과를 fabricate하지 않고 `not_available`+annotation CSV로 산출.
- Given 최종 판정, when 보고, then A/B/C/D/E 중 하나 + class별 근거 + 이전 두 결론이 왜 달라졌는지(정의/경로/분모/IoU미사용) 정정.
- Given 종료, when git status, then staged/commit 없음, 운영 파일 판정 diff 없음, 기존 GT/output 무수정.

## Design Notes

- **정본 FN 집합**: phase2 hsv_score는 Phase 1C와 ±1 차이 → 원인분석은 반드시 `build_p0`(cur pairs + cur/{ds}_hsv.npz 슬라이스 [0:224][96:224]) 정본으로 313 확정.
- **결정트리(파이프라인 gate 순서)**: RC-1(routed 후보 conf≥0.02 없음) → RC-8(sem_top5<sim_thr) → RC-9(appe11<appe_gate) → RC-10(hsv<0.1214) → RC-11(전 gate 통과인데 FN=selection/dedup) → RC-12(unknown). RC-0(GT오류)는 명백한 경우만.
- **best-ROI oracle(GT-free, 핵심)**: RC-8/9/10 cell에 대해 그 프레임의 전 per-prompt proposal(conf≥0.02, class-agnostic union)을 ISM(semantic+appe+hsv)로 스코어→ target object 기준 max. (a) 통과 ROI 존재+운영이 미선택 → **selection-causal**로 재분류, (b) 어떤 ROI도 미통과 → **gate/feature-limited**(gate-causal 강). GT bbox 없이 localization-vs-gate를 근사 분리.
- **CF-4 FP 재유입**: 비-visible cell에서 gate bypass 시 통과 후보 = FP. gate의 TP-복구 대비 FP-비용 정량화(운영 채택은 별개, 원인 규명 우선).
- **CF-1/2/3·IoU**: GT bbox 필요 → Option 1로 deferred. annotation CSV에 운영 routed bbox를 예시로 채워 사람 IoU 판정 준비만.
- AUROC/matching: greedy IoU one-to-one(동일 class 다중 instance), tie-break=conf 내림차순, 규칙 기록.

## Verification

**Commands:**
- `~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase21_fn_audit/build_fn_object_index.py` -- expected: P0 재현 assert 통과, FN 313 분류 Σ==313.
- `~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase21_fn_audit/run_fn_counterfactuals.py --with-gpu` -- expected: CF-0=622/86/313, CF-4/oracle 산출.
- `~/miniconda3/envs/sam_yolo/bin/python tests/test_phase21_rootcause.py` -- expected: 신규 테스트 pass(합==313 포함).
- `~/miniconda3/envs/sam_yolo/bin/python tests/test_uniform_threshold.py; ...test_hsv_shadow.py; ...test_phase1c.py; ...test_phase2_eval.py` -- expected: 회귀 전부 pass.
- `~/miniconda3/envs/sam_yolo/bin/python -m py_compile _ism_research_2026_07/phase21_fn_audit/*.py tests/test_phase21_rootcause.py` -- expected: OK.
- `git status --porcelain` -- expected: 운영 판정 diff 없음, staged 없음.

**Manual checks:**
- `outputs/phase21_fn_root_cause_audit/report/` + figures(funnel/원인bar/CF/초보자요약) 존재, 첫 페이지 15문답 표에 최종 병목 A/B/C/D/E 판정 + 이전 결론 정정 명시.
