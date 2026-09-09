---
stepsCompleted: [1]
workflowType: 'research'
research_type: 'technical'
research_topic: 'RGB Texture Discriminability Check (구조변경 전 마지막 검증)'
research_goals: 'milk 색/텍스처(빨강·파랑·로고) 신호로 흰물체 FP를 가를 수 있는지, DINOv2 appearance가 이를 놓치는지 코드 무수정 검증'
user_name: 'Ldh9501'
date: '2026-06-10'
source_verification: true
---

# Technical Research — RGB Texture Discriminability Check

> **데이터(읽기전용·코드 무수정)**: baseline 자기정합 311프레임 — `frames/<fid>.png` + `_run/<fid>/detection_pem.json`(bbox+RLE mask+score, pycocotools 디코드) + `frame_results`(decision_type). 42 템플릿 `rgb_*/mask_*`. 분석 `tools/validation/texture_discriminability.py`(cv2/pycocotools/numpy). 시각자료 `outputs/validation/texture_audit_vis/`.
> 표본: TP=112, FP=120, TN=73, FN=6 (311 crop). best=det[0](최고 score) candidate의 mask 영역 분석.

---

## Executive Summary — 결론 5개

1. **템플릿엔 색 신호가 실재한다 — 단 "파랑"이 주신호, "빨강"은 거의 없음.** 42 템플릿 평균 blue_ratio 0.129(최대 0.293), red_ratio 0.008(미미·작은 글씨), colorful 76.5, edge 0.066. milk의 변별 색은 **파란 디자인**이지 빨간 글씨가 아니다.
2. **색/텍스처 score는 실제로 TP/FP를 가르나 기존 score보다 약하다.** present-vs-absent AUC: **blue 0.807** > colorful 0.737 > edge 0.699 > sat 0.703 > rb 0.753(TPvsFP) ≫ red 0.514·hist_sim 0.580. 그러나 모두 **appe(DINOv2) 0.871 / geo 0.885 보다 낮다.**
3. **blue는 appearance와 상보적이다(가장 중요).** corr(appe, blue)=0.38(중간). **appe+blue AUC 0.910**(appe 단독 0.871 → **+0.039**), **[sem,appe,geo]+blue 0.922**(0.908 → **+0.014, in-sample**). DINOv2 appe가 놓치는 색 신호를 일부 보완.
4. **DINOv2 appearance는 색을 사실상 무시한다(시각 확인).** 高-appe FP 상위 6개(appe 0.66–0.74)는 전부 **blue≈0.00, colorful 11–18**(흰 박스/벽). 사람 눈엔 milk 색 전무인데 appe는 높게 부여 → appe의 명백한 맹점.
5. **그러나 단독 해결 불가 + hold-out·조명 리스크.** blue 0.807은 TP에도 큰 겹침(blue 안 보이는 milk 뷰 존재: TP 평균 rb 0.054 vs FP 0.015 — 차이 작고 분포 중첩). ceiling gain +0.014는 in-sample(QD-A 교훈상 hold-out서 축소), 색은 조명/뷰포인트에 취약. → **보조 신호로만 가치.**

---

## Template RGB Texture Audit (Task 1)
| feature | mean | std | range |
|---|---|---|---|
| red_ratio | 0.008 | 0.008 | 0–0.033 |
| **blue_ratio** | **0.129** | 0.075 | 0–0.293 |
| sat_mean | 0.142 | 0.083 | 0–0.322 |
| colorful(H-S) | 76.5 | 31.2 | 0.3–121.9 |
| edge_density | 0.066 | 0.028 | 0–0.114 |

→ 템플릿에 **파랑·채도·엣지 판별 신호 존재**(빨강은 미미). 단 일부 뷰(min=0)는 색이 거의 안 보임 → 뷰 의존성 내재.

## Real Crop vs Template Texture Comparison (Task 2)
- TP 평균 vs FP 평균: **rb 0.054 vs 0.015**, **colorful 21.8 vs 12.2**, edge 0.143 vs 0.097, hist_sim(템플릿셋 HSV 상관) 0.129 vs 0.118.
- TP crop이 더 파랗고/colorful/edge 많음 = 방향 맞음. 단 절대값 작고(실제 milk도 colorful 21.8로 낮음 — 거리/조명/해상도로 색 바램) 겹침 큼 → 분리 AUC가 0.7–0.8대에 머무는 이유.

## Texture Score Separation (Task 3)
| score | present-vs-absent AUC | TP-vs-FP AUC |
|---|---|---|
| **blue** | **0.807** | **0.812** |
| rb(red+blue) | 0.714 | 0.753 |
| colorful | 0.698 | 0.737 |
| sat | 0.684 | 0.703 |
| edge | 0.697 | 0.699 |
| hist_sim | 0.580 | 0.573 |
| red | 0.514 | 0.532 |
| — appe(기존) | 0.871 | 0.896 |
| — geo(기존) | 0.885 | 0.888 |

→ 최강 texture(blue)도 기존 appe/geo보다 **0.06–0.08 낮다.** 단독 게이트로 P/R 동시확보 불가.

## Comparison with SAM-6D Appearance Score (Task 4)
- corr(appe, blue)=0.38 / colorful 0.24 / edge 0.36 / hist_sim 0.02 → **중복 아님(상보적)**.
- appe-only **0.871** → appe+blue **0.910 (+0.039)**.
- [sem,appe,geo] **0.908** → +blue **0.922 (+0.014, in-sample)**.
- → blue는 기존 결합이 못 잡는 신호를 일부 추가하나, **전체 ceiling을 크게 못 올림**(in-sample 0.92, hold-out시 더 작을 것 — QD-A 교훈).

## FP Visual Audit (Task 5)
`outputs/validation/texture_audit_vis/fp_*.png` 6개 저장(frame|mask overlay|crop|template).
| FP frame | appe | blue/rb | colorful | hist_sim |
|---|---|---|---|---|
| 001372 | 0.736 | 0.000 | 12.2 | -0.008 |
| 001363 | 0.724 | 0.031 | 15.8 | 0.040 |
| 001368 | 0.668 | 0.000 | 11.1 | 0.012 |
| 001365 | 0.665 | 0.002 | 12.3 | 0.017 |
| 001564 | 0.660 | 0.000 | 12.9 | 0.042 |

→ **존재한다: 사람 눈엔 milk 색 전무(blue≈0)인데 DINOv2 appe가 0.66–0.74로 높게 준 FP.** appe가 색·로고를 안 보고 형태/패치 통계로 점수를 줌을 시각적으로 확인. = texture의 상보성 근거이자 appe 맹점 증거.

---

## Texture Check Verdict

### **Weak**

| 기준 | 판정 |
|---|---|
| texture가 FP를 잘 낮춤 | △ blue로 일부(색없는 흰물체) 가능하나 단독 AUC 0.807 < 기존 |
| TP recall 손실 작음 | ✗ 미검증·리스크 — 색 안 보이는 milk 뷰(blue≈0 TP) 존재 → 색 게이트 시 recall 손실 위험 |
| appearance와 상보적 | ✅ corr 0.38, appe+blue +0.039, 高-appe-無색 FP 포착 |
| 구현 비용 낮음 | ✅ HSV blue/colorful = 수 줄 |

**종합:** 실재하는 **상보적 보조 신호**(특히 흰물체 FP에 대한 DINOv2 맹점 보완)이나, ① 단독 분리력이 기존 appe/geo보다 낮고 ② 전체 결합 ceiling을 in-sample +0.014밖에 못 올리며(hold-out시 축소 예상, QD-A 교훈) ③ 색 무뎌지는 milk 뷰에서 recall 손실·조명 취약 리스크 → **단독 해결 불가, 보조 피처로만 가치 = Weak.**

---

## Recommended Next Step

**Weak → 구조 변경 연구로 이동하되 blue/color consistency를 보조 신호로 보유.**

1. **주력: 구조 변경 연구 진행**(Grounded-SAM/YOLO-World detect-then-segment). texture로는 ceiling(≈0.91–0.92)을 못 깨므로 근본 해결은 앞단 의미검출.
2. **저비용 병행 옵션(권장 가치 있음)**: `blue_ratio`(또는 colorful) 보조 신호를 **shadow-mode**로만 로깅/시뮬레이션해, "高-appe & blue≈0" FP를 얼마나 안전히(특정 recall 유지) 걸러낼 수 있는지 hold-out 정량화. 효과·recall 손실이 확인되면 보조 게이트로 채택, 아니면 폐기. (단독 Story로 분리 가능 — 코드 무수정 원칙은 검증까지, 채택 시 별도 Quick-Dev.)
3. blue는 milk(파란 디자인) 특화 신호 → **일반화엔 부적합**, milk 한정 보조로만.

---

## 부록 — 재현 & 한계
**재현**: `python3 tools/validation/texture_discriminability.py`. (주의: 스크립트의 best_tex 자동선정이 후보에서 blue를 누락해 rb로 결합 보고함 → 본 보고서의 결합 수치는 blue 포함 별도 재계산값 사용: appe+blue 0.910, [sem,appe,geo]+blue 0.922.)
**한계(추측 금지)**: ① 결합 AUC는 **in-sample**(hold-out 미적용) → QD-A상 일반화시 축소. ② det[0] crop만 분석(프레임당 1후보). ③ 색은 조명/화이트밸런스/해상도 의존 — 다른 환경 일반화 미검증. ④ red 신호는 실crop에서 거의 소실(작은 글씨). ⑤ baseline 311프레임(FN6 포함, 시각·색분석엔 GT 충분).

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 research .md 표준 — 이 절로 마무리.

**판정: Weak.** texture(blue)는 DINOv2 맹점을 보완하는 실재 보조신호지만 단독 FP 해결은 불가. **주력은 구조 변경**, blue는 저비용 보조로만.

선택지:
- **(A) 구조 변경 연구(Grounded-SAM/YOLO-World detect-then-segment) 시작** *(추천)* — FP 근본 해결.
- **(B) blue/color shadow-mode 보조게이트 hold-out 검증 Quick-Dev** 먼저 — 코드 무수정 검증으로 "高-appe·無색 FP"를 recall 손실 없이 거를 수 있는지 정량화(저비용, 효과 시 채택).
- **(C) A + B 병행** — 구조 연구 본류 + blue 보조 검증 동시.

→ **어느 쪽으로 진행할까요?** (추천: A, 또는 저비용이라 C)
