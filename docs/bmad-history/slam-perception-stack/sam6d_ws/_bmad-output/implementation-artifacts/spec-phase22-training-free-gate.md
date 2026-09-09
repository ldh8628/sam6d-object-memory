---
title: 'Phase 2.2 — Training-Free Gate Decision Improvement (재학습 없는 결합·선택 개선)'
type: 'feature'
created: '2026-07-22'
status: 'done'
baseline_commit: '10ec30f1ad027a5c0d05bce9cf5f0f663dfce429'
context: []
supersedes_note: '이전 draft의 supervised Calibrated Verification Head 방향은 INV-01 위반으로 폐기. logistic OOF(TP680/F1.8019)는 연구 참고값으로만 보존.'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Phase 2.1은 FN 313의 71%(222)가 gate-causal, 17%(53)가 selection-causal임을 밝혔다. gate 단순 우회는 FP를 13배(all-bypass +1158) 재유입시켜 불가하다. 기존 semantic·appearance·HSV score에는 정보가 있으나 현재 순차 hard-AND 구조가 일부 TP를 과제거한다. 단, **새 객체 추가 시 재학습이 없어야 한다(INV-01)** — 따라서 supervised head/logistic/calibration 계열은 전면 금지한다.

**Approach:** 학습·GT·객체별 파라미터 없이, **고정된 결정 규칙**으로만 gate 결합과 candidate 선택을 개선하는 training-free 방법군(A reranking, B deterministic rescue, C relative margin/rank, D multi-block consensus, E multi-template agreement, F template self-normalization, G Pareto selection, H temporal consistency, I mask/geometry routing)을 사전 고정 규칙으로 검증한다. 규칙 파라미터는 오직 {기존 운영 threshold, config 승인 margin, score 수학범위 기반 고정 비율, template self-similarity 통계, candidate 내부 상대 margin}에서만 유도한다. 검증은 dev/holdout 분리 + **Leave-One-Object-Out** + **신규객체 시뮬레이션**으로 누수·일반화를 검사한다. 채택기준(동일 FP≤86서 TP↑ 또는 F1↑, 다수 class/bag, 신규객체 시뮬 안정, 학습 artifact 0)을 충족한 방법만 **config-gated 기본 OFF**로 구현한다. 전부 실패하면 "고정 feature+무학습 규칙으로는 개선 근거 부족"으로 결론.

## Boundaries & Constraints

**Always (불변조건 INV-01~05):**
- **재학습·학습 artifact 절대 금지**: logistic/MLP/SVM/tree/linear-probe/calibration(Platt/isotonic/temperature)/객체별 coefficient·threshold·bias·자기학습·online/continual 전부 금지. 운영 결정은 코드+config의 고정 규칙만으로 재현. learned weight/checkpoint 파일 없음.
- **객체 비식별 공통 규칙**: object_name/class_id one-hot, 객체별 learned threshold, bag/sequence ID, GT 기반 난이도를 decision feature로 사용 금지. onboarding 시 생성되는 reference/template score·통계는 사용 가능.
- **사람 GT는 평가 전용**: 규칙/가중치/threshold/객체예외/normalization 생성에 사용 금지. TP/FP/FN 계산·방법 비교·보고에만.
- **Phase 1C 보존**: 기본 OFF, OFF에서 TP622/FP86/FN313/F1.7572 정확 재현. HSV t0.1214·Dino+9·texture off·sem0.35·appe0.55·appe_blocks[11] 불변. HSV 전체 완화 금지(bypass FP+248).
- **규칙 사전 고정**: 각 방법 수식·파라미터 후보를 평가 전 spec/config에 기록. holdout 결과를 보고 규칙/파라미터를 재수정하지 않음(수동학습 방지).
- **데이터**: 라벨 GT=user_visibility_gt.csv(935 cell). 후보 전수=`phase2_candidates.csv`(18060, 모든 routed 후보×객체: sem_top1/3/5/all_mean/median, appe2/9/11_clstop1, hsv_score, routed, yolo_conf, gt_visible) + `view_sims_phase2.npz`(uid‖object→42뷰). template 통계=`outputs/yolo_ism_object_n/template_features/*_cls.pt`·`*_appe_b*.pt`·`*_hsv.npz`. 대부분 오프라인(GPU 불요).

**Ask First:**
- 채택기준 충족으로 **기본 ON 배포**하려면 → HALT(활성화는 별도 승인, 이번은 기본 OFF 구현까지).
- 모든 방법이 채택기준 미달이면 → 운영 코드에 규칙 게이트를 넣지 말고 보고만 할지 HALT.
- 방법 I(mask/geometry)나 H(temporal)에 GPU 마스크 재계산이 필요하면 → 비용 확인 HALT(우선 오프라인 가능분부터).

**Never:**
- supervised/fine-tuning/calibration 도입. 학습 artifact 로드. holdout 보고 규칙 재조정. 객체별 예외 증식. GT로 파라미터 fit. git add/commit/push. 기존 GT·Phase1C/2/2.1 output 수정. OFF에서 판정 변화.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 기능 OFF(기본) | enabled=false, method=phase1c | 완전 no-op, Phase 1C 재현 | 필드 미기록 |
| gate-aware rerank | Top-K 후보, Top-1 gate 실패 | 전 gate 통과 차순위 후보 선택, 없으면 reject | K≤routed 수 |
| one-borderline rescue | 1 gate만 경계실패 & 나머지 strong | 고정규칙으로 제한 rescue | severe-fail(예:HSV 명확 불일치)면 rescue 금지 |
| self-normalization | template self-sim | median/MAD 정규화 | **MAD≈0/template<N → fallback(raw score)** |
| margin/rank | 후보<2 | margin 정의불가 → 규칙 미적용(기존 판정) | 0 대체 금지 |
| temporal | 인접 frame 없음(online 첫프레임) | single-frame 판정으로 fallback | 미래 frame 사용 금지 |
| LOOO/신규객체 시뮬 | 1객체 숨김 | 규칙 불변 적용, 규칙 변경 시 FAIL | fold<2 경고 |

</frozen-after-approval>

## Code Map

- `_ism_research_2026_07/phase2_bottleneck/phase2_common.py` -- load_gt/grid_metrics/auroc/lodo (재사용, LOOO 추가).
- `outputs/phase2_appearance_semantic_yolo_bbox/csv/phase2_candidates.csv` -- 후보 전수 feature(모든 방법 입력).
- `outputs/phase2_appearance_semantic_yolo_bbox/raw/view_sims_phase2.npz` -- 42뷰 배열(multi-template agreement·margin).
- `_ism_research_2026_07/phase21_fn_audit/phase21_common.py` -- 정본 313 FN·RC 라벨(복구 FN이 어느 RC인지 대조).
- `outputs/yolo_ism_object_n/template_features/{*_cls.pt,*_appe_b*.pt,*_hsv.npz}` -- template self-similarity 통계(F, onboarding 파생).
- `yolo_ism_object_n.py` -- `recognize_frame`(:499 accept·top_k 선택), `masked_appe_blocks`(block2/9/11), `_apply_hsv_gate`(gate 삽입 패턴), `_OVERRIDABLE`. 채택 시 규칙 게이트 삽입 지점.
- `ism_hsv.py` -- authority 모듈 패턴(고정규칙·fail-open) 참고.
- `configs/yolo_ism_objects.yaml` -- `training_free_gate_improvement:{enabled:false,method:phase1c,...}` 키(기본 OFF).

## Tasks & Acceptance

**Execution:**
- [x] `outputs/phase22_training_free_gate/` -- 트리(provenance/config_snapshots/csv/metrics/figures/representative_cases/report) + provenance(git-hash·seed·사전고정 규칙표).
- [x] `_ism_research_2026_07/phase22_training_free/build_candidate_pool.py` -- 모든 routed 후보/(ds,frame,object)를 phase2_candidates+view_sims로 조립, per-gate pass flag·rank·margin·42뷰통계·Phase1C 정본(622/86/313) 재현 assert → `csv/candidate_pool.csv`.
- [x] `_ism_research_2026_07/phase22_training_free/rules.py` -- 방법 A~G(+H) 결정규칙을 **사전 고정 수식**으로 구현(파라미터는 threshold/고정비율/template통계/상대margin에서만). GT fit 없음. 단위 검증 가능한 순수함수.
- [x] `_ism_research_2026_07/phase22_training_free/eval_training_free_methods.py` -- P0~P10 평가(A rerank Top2/3/5, B rescue, C margin/rank, D multi-block consensus, E multi-template agreement, F self-norm, G Pareto), 목적함수(FP≤86 max TP / F1), class·bag·bbox크기·RC별 복구, **LOOO + 신규객체 시뮬** → `csv/{method_comparison,method_by_class,method_by_bag,recovered_fn_cases,readmitted_fp_cases,candidate_reranking_trace,margin_rank_analysis,multiblock_consensus,template_agreement,self_normalization}.csv`,`metrics/*.json`.
- [x] `_ism_research_2026_07/phase22_training_free/eval_temporal.py` -- 방법 H(online 인접 frame 일관성, 미래 frame 미사용) H0~H3 → `csv/temporal_consistency.csv`. (I mask/geometry는 오프라인 가능분만; GPU 필요시 Ask First.)
- [x] `_ism_research_2026_07/phase22_training_free/make_phase22_figures.py` -- 방법별 TP/FP/F1·TP복구vsFP재유입·class별·RC복구·rerank 전후·rank agreement·block consensus·Pareto/temporal 대표·초보자 1장. Agg·Korean·PNG+SVG.
- [x] `tests/test_training_free_gate.py` -- Top-K rerank·동점·rank·margin·multi-block vote·template agreement·self-norm(MAD=0 fallback)·Pareto dominance·severe veto·borderline rescue·temporal online·feature 결측 fallback + OFF no-op(Phase1C 재현) + LOOO 규칙불변.
- [x] (채택 조건부) `ism_training_free_gate.py` + `yolo_ism_object_n.py`(config-gated `_apply_training_free_gate`, 기본 OFF) + `configs/yolo_ism_objects.yaml`(키 추가, 기본 OFF) -- **채택기준 충족 시에만**. 미충족 시 미구현·보고만.
- [x] `_bmad-output/implementation-artifacts/phase22-training-free-gate-decision-improvement.md` -- 12문답 + 방법별 결과·신규객체 시뮬·채택/기각·운영여부 + 26항목 터미널 출력.

**Acceptance Criteria:**
- Given 기능 OFF(또는 미구현), when 파이프라인/회귀 실행, then TP622/FP86/FN313/F1.7572 재현 + 기존 테스트(1A/1B/1C/P2/P2.1) 전부 pass.
- Given 각 방법, when 사전고정 규칙으로 평가, then 학습·GT-fit·객체별 파라미터 없이 홀드아웃/LOOO 성능·class/bag/RC별 복구를 수치 보고.
- Given 신규객체 시뮬(1객체 결과 숨김·prompt/template만 onboarding), when 동일 규칙 적용, then 규칙 변경 없이 동작함을 확인(변경 필요 시 해당 방법 기각).
- Given 채택 판정, when 기준(FP≤86서 TP↑ 또는 F1↑ & 다수 class/bag & 신규객체 안정 & 학습 artifact 0) 충족, then config-gated 기본 OFF 구현; 미충족 시 미구현·보고(Ask First). 전 방법 미달이면 "무학습 개선 근거 부족"으로 결론.
- Given 종료, when git status, then staged/commit 없음; 기능 OFF 기본이라 판정 무변경, learned artifact 0.

## Design Notes

- **사전 고정 파라미터 예시**(GT-free): borderline = 0.9×threshold(고정비율), strong = template self-sim median 이상, margin 최소 = top1/top2 비율 고정값, multi-block vote = 2/3(동일가중 1/3), template agreement = Top-K 중 N개 통과. 값 후보는 평가 전 `rules.py`/config에 기록, holdout 후 재조정 금지.
- **HSV veto 유지**: bypass FP+248이므로 HSV 명확 불일치는 hard veto, 경계일 때만 sem·appe 모두 strong시 제한 rescue(B2).
- **selection FN 53 우선**: A(gate-aware rerank)는 threshold 불변·차순위 후보 재검증으로 selection-causal 우선 복구 — 가장 안전(FP 위험 낮음).
- **누수 방지**: 단일 방법 먼저, 독립 이득 확인분만 최대 2개 결합(P10). 전수 조합탐색 금지.
- **self-norm fallback**: template<N 또는 MAD≈0 → raw score(불안정 회피).
- **참고 상한(운영후보 아님)**: supervised logistic OOF TP680/F1.8019 — 재학습 필요성으로 기각, training-free가 이에 얼마나 근접하는지 비교지표로만.

## Verification

**Commands:**
- `~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase22_training_free/build_candidate_pool.py` -- expected: Phase1C 622/86/313 재현 assert.
- `~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase22_training_free/eval_training_free_methods.py` -- expected: P0~P10 + LOOO + 신규객체 시뮬 결과.
- `~/miniconda3/envs/sam_yolo/bin/python tests/test_training_free_gate.py` -- expected: pass(OFF no-op·fallback·LOOO 규칙불변 포함).
- `~/miniconda3/envs/sam_yolo/bin/python tests/test_uniform_threshold.py; ...test_hsv_shadow.py; ...test_phase1c.py; ...test_phase2_eval.py; ...test_phase21_rootcause.py` -- expected: 회귀 전부 pass.
- `~/miniconda3/envs/sam_yolo/bin/python -m py_compile _ism_research_2026_07/phase22_training_free/*.py tests/test_training_free_gate.py` -- expected: OK.
- `git status --porcelain` -- expected: staged 없음; learned artifact 0; OFF 기본 판정 무변경.

**Manual checks:**
- 채택 시 head_gate 아닌 `training_free_gate_improvement.enabled=false` 기본, OFF에서 1 bag 실제 실행 결정 불변.
- `outputs/phase22_training_free_gate/report/` + figures + 대표사례(현재/대체 후보 rank·score·판정이유) 존재, 신규객체 시뮬 결과·활성화 권고 명시.
