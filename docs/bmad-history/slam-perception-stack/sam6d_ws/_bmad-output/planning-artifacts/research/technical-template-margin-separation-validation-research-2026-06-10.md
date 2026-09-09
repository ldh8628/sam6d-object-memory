---
stepsCompleted: [1]
workflowType: 'research'
research_type: 'technical'
research_topic: 'Template Margin Separation Validation (Selection Failure Rejected 재검증)'
research_goals: 'TP/FP/TN/FN 그룹별 margin·entropy 분포로 Selection Failure=Rejected 판정의 타당성 검증'
user_name: 'Ldh9501'
date: '2026-06-10'
source_verification: true
---

# Technical Research — Template Margin Separation Validation

> **데이터 소스(기존 계측만, 코드 무수정)**: `outputs/validation/SLAM_with_milk_nomilk/_run/debug/sam6d_debug.csv`(top5_template_scores) × `frame_results.csv`(decision_type). 분석 `/tmp/margin_sep.py`(stdlib).
> **표본(GT-매핑 56프레임)**: TP=22, FP=27, TN=7, **FN=0**. ⚠️ 직전 Quick-Dev의 라이브-bag 프레임 비재현성으로 GT는 56프레임 축소표본(전체 315 아님). FN 0개라 TP-vs-FN 검정 불가.

---

## Executive Summary

핵심 질문 — *"Selection Failure는 전체적으론 없지만 FP 집단에서만 발생하는가?"* — 에 대한 답:

**부분적으로 그렇다 — FP의 top1-top2 margin은 TP보다 통계적으로 유의하게 작다(AUC 0.675, Cliff's δ=0.35 medium, p≈0.04). 그러나 이것은 Selection Failure가 아니라 Appearance Ambiguity의 FP-집단 발현이다.**

근거 3가지:
1. **개념적 결정타**: Selection Failure = "정답 template가 존재하는데 다른 것이 선택됨"은 **객체가 present일 때만 정의 가능**하다. FP 프레임은 milk가 **없으므로 '정답 template' 자체가 없다** → 작은 FP margin은 "오선택"이 아니라 "배경/잡음이 여러 뷰에 고만고만하게 매칭"(=Appearance Ambiguity)을 의미한다.
2. **방향이 반대**: milk가 present인 TP 프레임에서 margin은 **오히려 더 크다**(median 0.033 vs FP 0.014). 즉 객체가 있으면 best 뷰가 더 분명히 이긴다 — selection이 **작동**한다는 증거.
3. **Entropy 무차이**: TP/FP entropy·top1-dominance 사실상 동일(AUC 0.577, δ 0.155). margin 차이는 점수 **크기(level)** 차이(TP top1≈0.55–0.69 vs FP≈0.45–0.50)에서 파생된 것이지, 분포 형태(집중도)의 차이가 아니다.

**Verdict: Weak Evidence** (Rejected 유지하되 FP-margin 축소를 Appearance Ambiguity 추가증거로 주석). Selection Failure 재수사(Re-open) 기준에는 미달.

---

## TP/FP/TN/FN Margin Distribution (연구과제 1)

**top1-top2 margin** (실측):
| group | n | mean | median | std | p10 | p25 | p75 | p90 |
|---|---|---|---|---|---|---|---|---|
| TP | 22 | 0.0316 | **0.0331** | 0.0201 | 0.0012 | 0.0150 | 0.0461 | 0.0538 |
| FP | 27 | 0.0215 | **0.0143** | 0.0279 | 0.0012 | 0.0042 | 0.0272 | 0.0401 |
| TN | 7 | 0.0164 | 0.0158 | 0.0085 | 0.0077 | 0.0087 | 0.0211 | 0.0268 |
| FN | 0 | — | — | — | — | — | — | — |

**top1-top5 margin**:
| group | n | mean | median | p25 | p75 |
|---|---|---|---|---|---|
| TP | 22 | 0.0793 | 0.0852 | 0.0586 | 0.1039 |
| FP | 27 | 0.0664 | 0.0622 | 0.0460 | 0.0800 |
| TN | 7 | 0.0484 | 0.0443 | 0.0377 | 0.0503 |

**관찰**: TP > FP > TN 순으로 margin 감소. milk가 명확할수록(TP) best 뷰가 더 분리되고, 객체가 없을수록(FP/TN) 상위 뷰들이 근접. 단 FP p10=0.0012로 TP p10과 같아 분포가 크게 겹친다(꼬리 공유).

---

## Statistical Tests (연구과제 2)

top1-top2 margin, Mann-Whitney U(=AUC) + Cliff's δ:
| 비교 | n | AUC = P[A>B] | Cliff's δ | 효과크기 | 근사 p(2-tailed) |
|---|---|---|---|---|---|
| TP vs FP | 22/27 | **0.675** | **0.35** | medium | **≈0.04** (z≈2.09) |
| TP vs TN | 22/7 | 0.727 | 0.455 | large | ≈0.06 (n작음) |
| TP vs FN | 22/0 | — | — | — | 검정불가(FN=0) |

**답 — TP와 FP는 같은 margin 분포인가?**
- **아니다(약하게).** TP margin이 FP보다 유의하게 크다(p≈0.04, medium effect). H0(동일분포) 약하게 기각.
- 단 **medium effect + 작은 표본(22/27) + 분포 큰 겹침**(FP p90 0.040 < TP median 아님; IQR 중첩)이라 강한 분리는 아니다. AUC 0.675는 "약간 더 크다" 수준(0.5=무분리, 0.71+=실질).

---

## Entropy Analysis (연구과제 4)

top5_template_scores를 softmax 정규화 후 entropy / top1-dominance:
| group | entropy mean | top1_dom mean |
|---|---|---|
| TP | 1.6077 | 0.2160 |
| FP | 1.6074 | 0.2175 |
| TN | 1.6085 | 0.2113 |

- **entropy TP vs FP: AUC 0.577, Cliff's δ 0.155 (negligible)** — 사실상 무차이.
- **계측적 주의(중요)**: 5-class 최대 entropy = ln5 = 1.6094. 관측치 1.607x는 **거의 균일(포화)**. 이유 = top5 점수가 0.07 좁은 대역(예 [0.69,0.66,0.65,0.58,0.57])이라 softmax가 거의 평탄. 따라서 **entropy는 본 데이터에서 변별력 없는(포화된) 지표**이며, raw margin이 더 적절한 렌즈다.
- **답 — FP는 여러 template가 비슷한 점수? TP는 특정 template 지배?** 두 그룹 **모두** top5가 비슷한 점수(흰물체 공통 특성). TP가 "지배적"이라기보다 top1-top2 gap만 약간 더 클 뿐, dominance ratio는 동일(0.216 vs 0.218). → 흰 저텍스처에서 **모든 프레임이 약한 외관변별**을 공유.

---

## Verdict Review (연구과제 5)

### **판정: Weak Evidence**

| 기준 | 해당? |
|---|---|
| Rejected 유지 (차이 미미·effect 작음·entropy 미미) | △ entropy는 미미하나 **margin은 medium effect로 "미미" 초과** |
| **Weak Evidence (차이는 있으나 FP를 설명할 정도 아님)** | **✅ 적중** |
| Re-open (FP margin 현저히 작음·entropy 차이 큼) | ✗ effect는 medium(현저 아님), entropy 무차이 |

**왜 Re-open이 아닌가 (3가지 결정 근거):**
1. **Selection Failure는 FP에서 정의 불가** — 객체 부재 프레임엔 '정답 template'가 없어 "오선택"을 말할 수 없다. 작은 FP margin = **Appearance Ambiguity**(무객체→근접 매칭)이지 Selection Failure 아님.
2. **TP에서 margin이 더 큼** — 객체 present 시 best 뷰가 더 분명히 이긴다 → selection은 작동. Selection Failure의 반증.
3. **차이의 출처가 score level** — entropy/dominance 무차이 + TP top1 점수가 본래 높음 → margin 차이는 점수 크기 파생물, 선택 메커니즘 결함 아님.

**왜 단순 "Rejected 유지"도 아닌가:** TP-vs-FP margin이 medium effect(δ0.35, p≈0.04)로 **무시할 수 없는 신호**다. 다만 그 신호는 Selection Failure가 아니라 **Appearance Ambiguity를 FP 집단에서 재확인**해 준다. 따라서 Rejected는 유지하되 이 관찰을 명시적 주석으로 남기는 Weak Evidence가 정확하다.

**신뢰도 한계(명시):** 56프레임 축소표본, FN=0(TP-vs-FN 미검정), 효과 medium·표본 소 → 결론은 "FP margin이 약간 작다(Appearance Ambiguity 일관)"까지만 확정. 전체 315 GT로도 방향은 바뀌지 않을 것이나(개념적 근거가 데이터와 독립적으로 성립), 효과크기 정밀화엔 frames/ 폴더 풀-GT가 필요.

---

## Recommended Next Step

**Weak Evidence → QD-A + 참고사항.**

- **QD-A 진행**: `[semantic, appearance, geometric]` 결합 + 보정(temperature/로지스틱) + hold-out으로 단일 pose-floor(0.3) 대비 FP 감소 입증. **본 연구의 함의**: FP의 약한 top1-top2 margin을 결합 피처로 추가하면 보조 변별에 쓸 여지가 있으나(δ0.35), 단독으론 약하므로 **주력은 sem/appe/geo 결합·보정**.
- **참고사항(QD-A에 반영)**: ① margin은 score level과 상관 → 결합 시 정규화 필요. ② entropy는 포화로 무용 → 피처 제외. ③ Appearance Ambiguity가 단독 주원인으로 재확인되므로, 병행 **QD-V1(DINOv2 vits14→vitl14)**이 근본 개선 후보.
- **Re-open 불필요**: Selection Failure 추가 수사는 권장하지 않음(개념적·데이터적으로 FP-margin은 Appearance Ambiguity로 귀속).

---

## 부록 — 재현 & 한계

**재현**: `python3 /tmp/margin_sep.py`. AUC=Mann-Whitney, Cliff's δ=(#TP>FP − #TP<FP)/(n·m), entropy=top5 softmax.

**한계(추측 금지)**: GT 56프레임(TP22/FP27/TN7/FN0). FN 0개로 TP-vs-FN 미검정. 효과 medium·표본 소 → 정밀 효과크기는 풀-315 GT(frames/ 폴더 처리) 필요. entropy 포화로 변별 불가(데이터 특성).

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 research .md 표준 — 이 절로 마무리.

**추천: Weak Evidence를 수용하고 QD-A로 진행**(Selection Failure는 Rejected 유지, FP-margin 관찰은 Appearance Ambiguity 주석으로 흡수).

선택지:
- **(A) QD-A 진행 + 본 연구 참고사항 반영** *(추천)* — 결합·보정으로 FP 감소, margin은 보조피처로만.
- **(B) QD-V1(DINOv2 vitl14) 먼저** — Appearance Ambiguity 근본(약한 백본) 제거 후 QD-A.
- **(C) frames/ 폴더 풀-315 GT 재실행**으로 본 검정 효과크기 정밀화 후 결정 — 56→315 표본 보강.

→ **어느 쪽으로 진행할까요?** (추천: A 또는 B; C는 효과크기 정밀화가 필요할 때만)
