# Quick-Dev 최종 보고서 — Template Selection Instrumentation & Failure Verdict

**날짜:** 2026-06-10 · **스펙:** `spec-template-selection-instrumentation.md` · **baseline:** `503e871`

---

## Implementation Summary

**수정 파일 (계측 전용 — score/ranking/threshold 무수정):**
| 파일 | 변경 |
|---|---|
| `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:293` | `compute_semantic_score`에 side-channel `self._last_template_scores`(=filtered per-template scores, idx_selected 순서) + `_last_pred_idx_objects` 노출. **반환 시그니처·argmax·점수 불변.** |
| `sam6d_master/SAM-6D/run_batch_inference_fast.py:446` | `run_ism_single`의 scores 리스트에 `best_template_id/score/top5_template_ids/top5_template_scores` 추가. side-channel에서 후보별 top-k 유도. `best_template[i]`와 `_sem_np[i]` 동일 idx_selected 순서라 정합 보장. |
| `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py` | (a) `_filter_and_decide`에서 breakdown(ism_score 조인)으로 4필드 추출→records; (b) `csv_fields`에 4컬럼 **append**; (c) writerow에 기록(list는 JSON 문자열). |

**추가 파일:**
| 파일 | 역할 |
|---|---|
| `tools/validation/analyze_template_selection.py` | stdlib 분석: usage 히스토그램 / TP·FP usage / per-template precision / top1-top2·top5 margin / verdict |

**변경한 컬럼:** debug CSV 끝에 `best_template_id, best_template_score, top5_template_ids, top5_template_scores` 4개 append (기존 51컬럼 순서·의미 불변).

**변경하지 않은 것:** final_score 공식, avg_5, confidence_thresh 0.2, nms 0.25, appearance/geometric/visible_ratio, ism_rerank(OFF 유지), pose/ism floor 0.3, workspace/depth gate, seed=1.

---

## Verification Summary

- **Build:** `colcon build --packages-select sam6d_ros` 성공, install에 계측 반영 확인.
- **계측 AC (345 is_best 행 전수):** `best_template_id ∈ [0,41]` **345/345** ✓ · `best_template_score == top5_scores[0]` 및 `id == top5_ids[0]` **345/345** ✓ · `top5_scores` 내림차순 **345/345** ✓ · valid id **345/345** ✓.
- **Validation re-run:** conda `sam6d_ros_humble`, `no_cli_milk_nomilk.yaml`, ism_rerank OFF, seed=1. (재현 보정: bag rate 1.0→0.25로 프레임 드롭 방지, **추론 결과는 프레임당 불변**, 분석 후 1.0 원복.)
- **신규 컬럼 확인:** debug CSV 헤더에 4컬럼 정상 기록.

### ⚠️ Confusion 재현 실패 (Task 6 HALT 보고 사항)
| | baseline | 재실행 |
|---|---|---|
| 처리 프레임 | 315 | 345 |
| **GT 라벨 매핑** | 315 | **56** (TP22/FP27/TN7, UNKNOWN 289) |

**원인(실측):** `frame_id`는 카메라/bag 프레임 번호이고, 라이브 bag 재생은 color/depth **sync 타이밍에 따라 매 실행 다른 프레임 집합**을 처리한다(예 baseline 2번째=000022 vs 재실행=000013). GT 라벨(`visibility_labels.csv`)은 **baseline이 처리한 특정 315프레임에만** 존재 → 라이브 재실행과는 56프레임만 겹친다. **시스템 변화가 아니라 프레임 집합 비재현성** 때문. baseline confusion(TP112/FP120) 직접 재현은 라이브 경로로는 구조적으로 불가.

**영향 범위:** GT-**독립** 신호(usage 히스토그램·margin)는 345프레임 **전수 풀파워**. GT-**의존** 신호(TP/FP usage·precision)만 56프레임 **축소파워**.

---

## Template Usage Histogram (345프레임, GT-독립)
- 사용 distinct template = **30/42**. 분산되어 있음(특정 소수 collapse 아님).
- 최다 **template_07 = 85회(24.6%)**, 이어서 t09(32) t13(31) t19(30) t18(27) t16(23) t33(22) …
- (전체 표: `outputs/validation/SLAM_with_milk_nomilk/template_selection_analysis.txt`)

## TP / FP Template Usage (56 GT프레임)
- TP=22, FP=27.
- **FP 분산**: t13(5) t16(4) t09(3) t18(2) t19(2) t20(2) t29(2) t33(2) t07(2) … — **최다 t13도 18.5%**. 단일 오검출 template로의 collapse **없음**.
- **TP 최다 = t07(12)**, FP 최다 = t13 — 다르지만 FP가 분산이라 "오선택 collapse" 패턴 아님.

## Per-template Precision (56 GT프레임)
- **지배 template t07: TP12/FP2 → precision 0.857** (milk 정답뷰를 올바르게 선택, 고품질).
- 저-precision template(t09=0.00/sup3, t13=0.167/sup6, t16=0.00/sup4)은 **support가 작고 FP도 소수**(최다 5). **FP≥10 & precision≤0.20인 "오검출 전담" template 없음.**
- support≥5 precision std = 0.285 (편차 제한적, 표본 3개).

## Top1-Top5 Margin Analysis (345프레임, GT-독립)
- **top1-top2**: mean 0.0226, **median 0.0180**, min 0.0, max 0.141 → median이 near-tie 임계(0.005) 위. argmax 대체로 분리되나 일부 프레임 0.0 동률 존재(흰물체 모호성 흔적).
- **top1-top5**: mean 0.0713, median 0.0656 → 상위 5뷰가 좁은 대역(0.07)에 몰림 = 흰 저텍스처에서 뷰 변별이 약함을 시사(단, best는 분리됨).

---

## Selection Failure Verdict

### **Selection Failure = Rejected**

**근거(판정기준 대비):**
1. FP collapse(≥50%) **불충족** — FP 최다 점유율 18.5%, 12+ template에 분산.
2. 저-precision 고-FP template(FP≥10 & prec≤0.2) **부재** — 단일 template 최대 FP=5.
3. argmax near-tie(median margin<0.005) **불충족** — median 0.018.
4. TP최다≠FP최다 & FP집중≥40% **불충족** — FP 집중 18.5%.

**해석:** 모델은 milk가 **있을 때 정답뷰(t07)를 0.857 precision으로 정확히 선택**하고, milk가 **없을 때(FP)는 점수가 고만고만한 여러 뷰로 분산** 선택한다. 이는 "정답 template가 있는데 다른 걸 고르는 Selection Failure"가 아니라, **흰 저텍스처 객체에서 외관 점수 자체가 약해 배경에도 0.4~0.5대를 주는 Appearance Ambiguity**의 발현이다. (top1-top5가 0.07 대역에 몰리는 것도 같은 신호.)

**신뢰도 단서:** Rejected의 핵심 근거(FP 분산·t07 고precision)는 명확하나, GT-의존 부분은 56프레임 축소파워다. 단 Confirmed 기준 4개가 **하나도 근접조차 안 함**이라, 표본을 315로 늘려도 결론이 뒤집힐 정황은 없다. (원하면 frames/ 폴더 풀-GT 재실행으로 신뢰도 보강 가능 — 아래 옵션.)

---

## Root Cause Tree Update

| 항목 | 이전 | 갱신 |
|---|---|---|
| Proposal Failure | Rejected | **Rejected** (불변) |
| Template Coverage Failure | Rejected | **Rejected** (불변, 풀-스피어 42뷰) |
| PEM Template Information Loss | Rejected | **Rejected** (불변, 설계상 비전달) |
| Implementation Mismatch | Rejected | **Rejected** (불변, 코어 100% 충실) |
| **Template Selection Failure** | **Unknown** | **→ Rejected** (본 계측·분석으로 확정) |
| **Appearance Ambiguity** | Confirmed | **Confirmed (강화)** — 유일하게 남은 주원인 |

→ Root Cause Tree에서 **모든 후보가 Rejected, Appearance Ambiguity만 Confirmed로 단독 잔존.**

---

## Recommended Next Story

Selection Failure가 **Rejected**이므로 보고서 기준 경로 = **QD-A**:

- **QD-A (P0)**: `[semantic, appearance, geometric]` 결합 + 보정(temperature/로지스틱) + hold-out 분리로, 단일 pose-floor(0.3) 대비 동일 R에서 FP 감소 입증. 흰물체 Appearance Ambiguity를 다중점수 결합으로 보완.
- **(병행 권장) QD-V1**: DINOv2 백본 `vits14`(384-dim) → 논문급(`vitl14`, 1024-dim) 교체 후 FP 변화 측정. Appearance Ambiguity의 **구현측 원인**(약한 백본) 직접 검증 — 3자 감사에서 도출된 최저비용·최대영향 항목.
- **(선택) 신뢰도 보강**: frames/ 폴더(라벨 stem과 동일 파일명)로 풀-315 GT 재실행이 필요하면 별도 소형 story로 분리.

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 보고서 표준 — 이 절로 마무리.

**추천: Rejected를 채택하고 QD-A(+QD-V1)로 진행.** Selection Failure는 4개 Confirmed 기준에 근접조차 못 했고, 남은 단독 주원인은 Appearance Ambiguity다.

선택지:
- **(A) Rejected 채택 → QD-A(결합 보정)로 진행** *(추천)*
- **(B) QD-V1(DINOv2 vitl14 교체) 먼저** — Appearance Ambiguity의 구현측 원인부터 제거
- **(C) frames/ 폴더 풀-315 GT 재실행으로 verdict 신뢰도 보강 후 결정** — 축소파워(56프레임)가 마음에 걸릴 경우

→ **어느 쪽으로 진행할까요?** (추천: A 또는 B)
