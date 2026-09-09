# Phase 2.2 — Training-Free Gate Decision Improvement
## 재학습 없는 Semantic·Appearance·HSV 결합 및 Candidate Selection 개선

- 작성일 2026-07-22 · Ldh9501 · `/bmad-quick-dev` · **검증 + config-gated 기본 OFF 구현**
- 절대조건: **학습·GT-fit·객체별 파라미터·learned artifact 0** (INV-01~05). GT는 평가 전용. OFF=Phase 1C 정확 재현.
- supersedes: supervised Calibrated Verification Head(폐기, INV-01 위반). logistic OOF(TP680/F1.8019)는 참고 상한.
- 산출: `outputs/phase22_training_free_gate/` (분석 + **실제 프레임 시각화 581장**)

---

## 12문답 (필수)

| # | 질문 | 답 |
|---|---|---|
| 1 | 학습 없이 가장 많은 TP 복구 방법? | `rank_consensus`(+78) / `multiblock_consensus`(+73) — **단 FP +143/+133로 사용불가**. FP 통제하 최선=`one_borderline_rescue`(+28). |
| 2 | 동일 FP≤86서 TP 증가? | **없음.** rerank_top3/pareto가 +5 TP지만 +6 FP(FP 92). 엄격 기준(FP≤86 & TP↑) 충족 방법 0. |
| 3 | rerank로 selection FN 복구? | **5개**(RC11 전수) 복구, +6 FP(대부분 choco). threshold 불변. |
| 4 | relative rank/margin이 절대 threshold보다 나았나? | 부분적. rank_consensus는 appe FN 78 복구하나 FP 143 동반 → 절대임계 제거는 FP 폭증. |
| 5 | multi-block consensus > block9 단독? | 아니오(FP 관점). block rank 합의는 +73 TP/+133 FP — Phase 2 block9 게이트(FP↓)와 반대 방향. |
| 6 | multi-template agreement가 TP/FP 구분? | `view_stability`는 조건 충족 사례 0(복구 0) — 42뷰 일관성만으로는 borderline semantic 미복구. |
| 7 | template self-normalization이 신규객체 score차 완화? | 이번 오프라인셋에선 유의미 복구 미확인(별도 GPU 통계 필요, 우선순위 낮음). |
| 8 | Pareto가 잘못된 Top-1 선택 감소? | 소폭. pareto=+5 TP/+6 FP(=rerank와 동일, RC11 5 복구). |
| 9 | temporal consistency TP복구/FP비용? | online 인접프레임 규칙 별도(eval_temporal); 본셋은 프레임 희소(strided)라 이득 제한. |
| 10 | 신규객체 시뮬서 규칙 변경 없이 동작? | **예.** rules.py는 object identity 미사용(INV-03) → 구조적으로 모든 객체 동일 규칙. LOOO도 규칙 불변. |
| 11 | 객체별 예외 없이 공통 적용? | **예.** 어떤 규칙도 객체명/class_id/객체별 threshold 미사용. |
| 12 | 최종 운영 구현 후보 존재? | **조건부.** `one_borderline_rescue`가 F1 기준(.7572→.7715, 8 class↑) 충족·엄격 FP기준 미충족 → **config-gated 기본 OFF**로 구현, 활성화는 별도 승인. rerank는 near-neutral 안전옵션. |

**한 줄 결론:** training-free 규칙만으로 **FP≤86 유지하며 TP 늘리는 방법은 없다**. 최선은 결정론적 `one_borderline_rescue`(+28 TP/+14 FP, F1 +0.014, 8 class 개선, FP 대부분 choco 갈색박스). 큰 복구는 FP 폭증(Phase 2.1의 "222 feature-limited FN"이 training-free로 넘을 수 없음을 재확인) → 근본 해법은 **더 강한 feature**(범위 밖).

---

## 1~3. 결과 · Phase 1C 회귀 · supervised 폐기

- **PASS**(검증 완료 + config-gated OFF 구현). 엄격 FP≤86 기준은 미충족(전 방법) → 운영 **기본 OFF**.
- Phase 1C 회귀: candidate_pool 재현 **622/86/313/.7572** assert 통과. 기존 테스트 1A5·1B13·1C12·P2 8·P2.1 12 전부 pass. **OFF에서 판정 무변경**(tf-gate 12 test 포함).
- supervised head 폐기 확인: logistic/MLP/calibration 전부 미사용. learned artifact 0. GT 평가 전용.

## 4. 평가한 training-free 방법 (candidate_pool 3712후보/2141셀, 학습 0)

| 방법 | TP | FP | FN | F1 | ΔTP | ΔFP | 판정 |
|---|---|---|---|---|---|---|---|
| P0 Phase1C | 622 | 86 | 313 | .7572 | 0 | 0 | 기준 |
| P1 rerank_top2 | 601 | 87 | 334 | .7406 | −21 | +1 | 기각(top3 sem선택보다 좁음) |
| **P1 rerank_top3** | 627 | 92 | 308 | .7582 | +5 | +6 | selection 5 복구, near-neutral |
| P2 rank_consensus | 700 | 229 | 235 | .7511 | +78 | +143 | 기각(FP폭증) |
| P3 multiblock_consensus | 695 | 219 | 240 | .7518 | +73 | +133 | 기각(FP폭증) |
| P4 view_stability | 622 | 86 | 313 | .7572 | 0 | 0 | 무효과 |
| P6 pareto_select | 627 | 92 | 308 | .7582 | +5 | +6 | =rerank |
| **P7 one_borderline_rescue** | 650 | 100 | 285 | **.7715** | +28 | +14 | **최선(F1↑, 8 class)** |

## 5. Gate-aware reranking 결과

- rerank_top3: **RC11 selection FN 5개 전수 복구**(threshold 불변), +6 FP(choco 5, Sauce 1). F1 .7582.
- Phase 2.1 oracle의 "selection-causal 53"은 대부분 운영 conf-top3 **밖** 후보 → 운영 후보집합 내 rerank로 복구 가능한 건 **5개**(RC11)뿐. 나머지는 후보 확장(proposal) 필요.
- rerank_top2가 −21 TP인 이유: Phase 1C가 conf-top3 중 best-by-sem을 고르는데 top2로 좁히면 sem-우수 후보 탈락.

## 6. Relative margin/rank · 7. Multi-block · 8. Multi-template · 9. Self-norm · 10. Pareto · 11. Rescue · 12. Temporal

- rank_consensus(C)/multiblock(D): appe FN을 크게 복구(78/73)하나 **FP 143/133 동반** → appe 절대임계를 상대순위/블록합의로 바꾸면 FP가 같이 들어옴(feature 중첩 재확인).
- view_stability(E): 복구 0(42뷰 일관성만으로 borderline semantic 미복구).
- self-norm(F): 본셋 유의 복구 미확인.
- pareto(G): rerank와 동등(+5/+6).
- **one_borderline_rescue(B)**: 유일하게 F1 순증(.7715). HSV hard-veto 유지(severe HSV는 rescue 금지)로 FP 폭증 회피.
- temporal(H): 프레임 strided라 이득 제한(별도 eval).

## 13. Mask/geometry routing

이번 범위에서 별도 GPU 마스크 재계산 필요 → 우선순위 낮춤(오프라인 미수행). 시각화 overlay에는 MobileSAM contour 포함.

## 14. 동일 FP 예산(≤86) 비교

**FP≤86 유지하며 TP 증가하는 방법 없음.** 가장 근접: rerank/pareto(FP 92, +5 TP). rescue는 FP 100. 큰 복구는 전부 FP≥219. → Phase 2.1 결론(222 feature-limited) 재확인.

## 15~16. class·bag별

- **P7 복구 8 class**: Rabbit+6·choco+6·milk+4·Sauce+4·Bear+3·Sikhye+3·Dino+1·Febreze+1. FP 증가 3 class, **14 FP 중 12가 choco_hazelnut**(갈색박스 흡착, 기존 known issue).
- **rerank 복구**: Bear+2·Dino+2·milk+1(3 class), FP choco+5·Sauce+1.
- bag별 `csv/method_by_bag.csv`.

## 17. 신규객체 시뮬레이션

rules.py는 **object identity/GT/학습 미사용** → 신규객체에 동일 규칙 자동 적용(구조적 보장, INV-03). LOOO(객체 1개 숨김·규칙 불변)에서 규칙 변경 0. 새 객체 onboarding = prompt+template+cache만(재학습 없음).

## 18~20. 채택/기각/운영

- **채택(조건부)**: `one_borderline_rescue` — F1 기준·다수 class·신규객체 안정·학습 0 충족. 단 **엄격 FP≤86 미충족**(FP 100). → **config-gated 기본 OFF 구현**, 활성화는 사용자 승인.
- **안전 옵션**: rerank_top3(selection 5 복구, near-neutral).
- **기각**: rank_consensus·multiblock(FP폭증)·view_stability(무효)·rerank_top2(TP손실).
- **운영 구현**: `training_free_gate_enabled: false`(기본), `method: phase1c`(기본). 모듈 `ism_training_free_gate.py`(학습 0). ⚠ **운영 recognize_frame은 best-by-sem 후보만 후단 score 계산** → rescue의 appe/sem-borderline 복구를 운영에서 완전 재현하려면 per-candidate 전-gate 계산 리팩터가 필요(별도 스토리). 현 hook은 도달가능 범위만 안전 적용(OFF 기본이라 무영향).

## 21. 변경 파일

- 신규: `ism_training_free_gate.py`(학습0 규칙 모듈), `_ism_research_2026_07/phase22_training_free/{build_candidate_pool,rules,eval_training_free_methods,make_visual_cases,make_phase22_figures}.py`, `tests/test_training_free_gate.py`, `outputs/phase22_training_free_gate/**`.
- 수정: `yolo_ism_object_n.py`(+import, +`_apply_tf_gate` hook, 4곳 호출 — **전부 기본 OFF no-op**), `configs/yolo_ism_objects.yaml`(tf 키 2개, 기본 OFF).
- rollback: config 키 제거 또는 enabled:false 유지(=현 상태). learned artifact 0.

## 22. 실행 명령
```
PY=~/miniconda3/envs/sam_yolo/bin/python
$PY _ism_research_2026_07/phase22_training_free/build_candidate_pool.py
$PY _ism_research_2026_07/phase22_training_free/eval_training_free_methods.py
$PY _ism_research_2026_07/phase22_training_free/make_visual_cases.py --method one_borderline_rescue
$PY _ism_research_2026_07/phase22_training_free/make_visual_cases.py --method P1_rerank_top3
$PY _ism_research_2026_07/phase22_training_free/make_phase22_figures.py
$PY tests/test_training_free_gate.py
```

## 23. 테스트
tf-gate 12/12 · 회귀 1A5/1B13/1C12/P2 8/P2.1 12 전부 pass · py_compile OK · **OFF=Phase1C 보존**.

## 24. 시각화와 보고서 (실제 프레임 기반, 사용자 요구)

`outputs/phase22_training_free_gate/figures_visual/` — **총 581 PNG**:
- single_overlay 105·before_after 43·candidate_panels 28·by_class 105·by_bag 105·root_cause 105·contact_sheets 15.
- 각 overlay: 원본 gt_input 프레임 + candidate bbox(선택=빨강/대체=파랑/복구=주황) + MobileSAM mask contour + sem/appe/hsv/rank/gate(SOAOHX) + Phase1C vs 새방법 판정 + root-cause + 한국어 해설. **GT bbox는 없음(frame-level visibility GT)** 명시.
- 인덱스: `figures_visual/report/visual_case_index.{md,html}`, manifests `visual_cases_manifest.csv`(120) + recovered/false_positive/hard_fn/selection_fix/class_summary CSV.
- 분석 figure: `figures/`(method 비교·TP복구vsFP·RC분포·class Δ·초보자요약).

## 25~26. Git · 다음 권장

- Git: staged/commit/push 0. 운영 파일은 기본 OFF(판정 무변경). learned artifact 0.
- **다음 권장**: (1) `one_borderline_rescue` 활성화는 choco FP(+12) 감내 여부로 사용자 판단. (2) 진짜 병목(222 feature-limited)은 training-free 한계 밖 → **더 강한 feature**(다중뷰 verification·block별 texture, 단 재학습 없이) 연구. (3) selection FN 확대 복구는 후보집합 확장(per-prompt 유지 + top_k↑) 필요.
