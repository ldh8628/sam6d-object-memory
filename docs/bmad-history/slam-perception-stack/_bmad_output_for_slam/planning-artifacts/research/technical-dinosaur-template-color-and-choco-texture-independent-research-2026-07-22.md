---
title: '독립 연구 — Dinosaur 템플릿 색상 보정(A) & 갈색박스 vs 초코하임 texture 검증(B)'
type: 'technical-research'
date: '2026-07-22'
scope: '오프라인 연구만. 운영 코드/config/템플릿/cache 무수정, git 무조작.'
workdir: 'sam6d_ws/_ism_research_2026_07/dinosaur_color_and_texture_research/'
authority: 'ism_hsv.py; DINOv2 patch(block2/11); pipeline/cur; box_labels_merged(provisional); shadow_decisions'
---

# 1. Executive Summary

두 연구를 독립 baseline·지표·판정으로 수행했다.

- **연구 A(색상)**: Dinosaur 템플릿 42장에만 단일 hue 보정(+9° OpenCV, LODO 6폴드 안정)을 적용하면
  **Dinosaur TP 33→39(+6), 새 FP 0, 다른 9객체 불변, 전체 F1 .7526→.7564**. → **채택**(깨끗하나 폭 제한:
  단일 재중심은 넓은 실색 분포를 다 못 덮어 6건만 복구).
- **연구 B(texture)**: DINOv2 block11 patch를 42템플릿 매칭(P2)하면 초코 vs 갈색박스 provisional
  AUROC 0.92. **⚠ §18 사람 GT 재검증에서 하향: AUROC 0.834, 초코 TP 손실 동반(clean gate 아님, modest).**
- **⚠ 중요 정정(§18)**: 초기 "동일 임계에서 Dino 19/22 rescue" 낙관은 **FP 재유입 검증에서 무너짐** —
  rescue-verifier **AUROC 0.508(우연)**, thr 0.62서 TP 29 복구에 **FP 194/248 재유입**. HSV가 지운 FP는
  애초 texture가 유사해 통과한 것(색만 분리)이라 texture로 재분리 불가. **rescue 기각.**
- **수정된 관계**: 연구 A(색 +6, FP 0)만이 Dinosaur의 **안전한** 복구. texture rescue는 사망. texture
  negative verifier는 초코 FP를 modest하게 줄임(선택적).

---

# Part A — Dinosaur Template Color Correction

## 2. 연구 A 목적
렌더 템플릿(저채도 황록 H78°)이 실제 Dinosaur의 넓은 색 분포를 못 덮어 22 TP가 HSV에서 오탈락.
**Dinosaur 등록 템플릿 42장에만** 단일 색 변환을 가해 복구되는지 검증(런타임 HSV 알고리즘·threshold
불변, 다른 9객체 reference 불변 → 다른 객체 결과는 정의상 불변).

## 3. 실제·렌더 Dinosaur 색 분포
`results/dinosaur_real_hsv_statistics.csv`: 실제 Dino(가시 TP) OpenCV Hue mean 34 / median 39 / p10 7.7 /
p90 49.7 (넓음). 렌더(−3 보정 후) ≈ 36. 오탈락군은 더 초록(H~50 ocv = ~100° in 0-360). 즉 Dino는
**+green 방향** 보정이 필요(전역 −6°와 반대) — 그래서 전역보정으론 불가, Dino-only만 가능.

## 4. 보정 방식 (`dinosaur_template_correction_sweep.csv`, `dinosaur_color_final_comparison.csv`)
LODO(5데이터로 보정 선택 → held-out 평가), t=0.12140 고정, 42-view max 유지.

| 변형 | Dino TP | Dino FP | 전체 F1 | LODO pick |
|---|---:|---:|---:|---|
| A0 현재 | 33 | 0 | .7526 | — |
| **A1 단일 hue shift** | **38** | **0** | **.7564** | **전 폴드 +9 ocv** |
| A2 hue+sat | 38 | 0 | .7564 | +10, sat0.9 (추가이득 없음) |
| A4 hist-match | 9 | 0 | .7340 | 음수 shift(집계 hue 왜곡에 오도)→기각 |

전수 스윕: +9에서 Dino 39 peak, **+12 이상은 overshoot(→25)** — 단일 재중심의 한계(넓은 분포 다 못 덮음).

## 5. Dinosaur TP 복구 결과
- **보정 전 33 / 후 39 (+6)**, 오탈락 22 중 **6 복구**, **새 FP 0**, 전체 FP 86 불변, F1 .7526→.7564.
- 다른 9객체: **완전 불변**(reference 미변경). LODO +9 전 폴드 동일 → **과적합 아님**.

## 6. 연구 A 독립 판정
**채택(A1: Dinosaur 템플릿 42장에 +9° OpenCV 단일 hue shift).** 성공기준 충족: Dino 유의미 복구(소폭),
FP 무증가, 타객체 불변, 공통 threshold 0.1214 유지, 단일 보정, 런타임 분기 없음, LODO 안정. 한계: 넓은
실색 분포로 22 중 6만 복구(단일 재중심의 원리적 상한).

---

# Part B — Choco vs Brown Box Texture Verification

## 7. 연구 B 목적
HSV로 못 거르는 **갈색 택배박스→choco 오수락**(색 동일) 감소가 주목적. DINOv2 texture/patch로 분리.
Dinosaur rescue는 부가.

## 8. Texture 특징 비교 (`texture_method_comparison.csv`, 58 candidates, 12 choco/46 non, provisional)
| 방식 | AUROC | choco mean | non mean |
|---|---:|---:|---:|
| P0 block11 clstop1 (현행) | 0.875 | 0.615 | 0.541 |
| **P2 block11 42템플릿 max** | **0.92** | 0.683 | 0.584 |
| P3 block2+block11 | 0.853 | 0.725 | 0.679 |
| P5 b2+b11 semTop5 | 0.871 | — | — |
| P8 b2+b11 top-pct | 0.817 | — | — |

**핵심: 현행(P0, 단일 clstop1 템플릿)보다 42템플릿 전체 매칭(P2)이 우수(.875→.92).** 이전 "block2+block11
최고"는 **본 라벨셋에선 재현 안 됨**(P3 .853 < P2 .92) — 사용자 지시대로 재검증하여 정정. 미실행:
P1(block2단독)·P6(vectorized)·P7(mutual-NN)·P9(spatial) — P2가 이미 충분히 우수해 우선순위 낮춤(한계 명시).

## 9. 갈색 박스와 초코하임 구분 (`texture_negative_verifier_results.csv`)
P2 음성검증기 임계 스윕(초코 12 / 갈색등 46):
| thr | 초코 유지 | FP 제거 |
|---:|---:|---:|
| 0.50 (전초코유지) | 12/12 | 2/46 |
| 0.575 | 11/12 | 19/46 |
| **0.625** | **11/12** | **43/46** |
| 0.649 | 11/12 | 46/46 |
| 0.65 | 10/12 | 46/46 |

"전 초코 유지" 지점은 초코 1건 outlier 때문에 약함(2/46)이나, **초코 1건 손실을 허용하면 thr 0.625에서
갈색박스 FP를 거의 전부(43~46) 제거.** 시각화(`research_b_choco_vs_brownbox_texture.png`): 실초코 tex
0.67~0.71(인쇄 로고·포장) vs 갈색박스 0.60~0.63(민무늬 골판지) — 분리되나 마진 좁음.

## 10. Texture negative verifier
**판정: 채택(사람 GT 검증 조건부).** 인쇄 무늬가 실제 판별 기여(로고 있는 초코 vs 민무늬 박스). 런타임
**거의 무료**(block11 patch는 파이프라인이 이미 계산; +42템플릿 max ≈ +0.026ms/box). 객체별 알고리즘·
block·threshold 없음, 신규 객체도 42-view 자동 캐시, 실사 reference 불필요. ⚠ N 작음(초코 12)·provisional
라벨 → 사람 GT로 임계·마진 재확정 필요.

## 11. Texture rescue verifier (`texture_rescue_results.csv`)
HSV 오탈락 Dino(22)에 **동일 P2·동일 알고리즘** 적용:
- HSV-통과 Dino texture mean **0.666** vs HSV-오탈락 Dino **0.655** — **거의 동일**(오탈락도 진짜 Dino임을
  texture가 확증).
- **동일 임계 0.62에서 19/22 rescue**, Dino p10(0.621)≈choco 임계 → **Dino 전용 파라미터 불필요**.
- **판정: 유망하나 추가 검증** — HSV가 올바로 제거한 248 FP에 대한 **rescue의 FP 재유입 비용은 미측정**
  (핵심 미결). 무차별 rescue가 cross-fire FP를 되살리면 안 됨.

## 12. Dinosaur 부가 결과
복구 19/22, 새 파라미터 불필요, 초코용 texture 설정 그대로 Dino 적용 가능 → **일반화 성공**(Dino 전용
block/threshold 불요). 색상(A, +6)보다 texture rescue(+19)가 Dino를 훨씬 더 복구.

## 13. 연구 B 독립 판정
- **negative verifier: 채택(사람 GT 검증 필요).** 갈색박스 FP 대폭 제거, 런타임 무료, 객체무관.
- **rescue verifier: 추가 검증.** Dino 19/22 유망하나 FP 재유입 미측정.

---

# 14. 독립 결과 비교 (`combined_summary/results/independent_research_comparison.csv`)
| 연구 | 성공 | 핵심 | 독립 채택 |
|---|---|---|---|
| A 색상 | ✔(소폭) | Dino +6, FP 0, 타객체 불변 | 가능 |
| B neg-verifier | ✔ | 초코-갈색 AUROC .92, FP 43~46/46 제거 | 가능(GT 검증) |
| B rescue | 유망 | Dino 19/22, 무료·객체무관 | FP 재유입 검증 후 |

두 연구는 **서로 독립적으로 채택 가능**. Dinosaur 회복은 **texture rescue가 color를 압도**(19 vs 6).

# 15. 참고용 결합 결과 (`optional_combined_result.csv`)
C0 = A(+9 Dino 색) + B(texture neg+rescue): Dino 관점에서 texture rescue가 color의 상위집합이므로 결합
이득은 texture 주도. 갈색박스 FP는 B만 해결. **참고용이며 A/B 독립 판정을 대체하지 않음.**

# 16. 운영 반영 우선순위
1. **연구 B negative verifier(초코 갈색박스)** — HSV가 못 푸는 최대 FP원, 런타임 무료. 단 사람 GT로 임계
   재확정 후 Phase 1C에 반영.
2. **연구 B rescue verifier** — Dino 대량 복구의 정공법. FP 재유입(248 HSV-제거분) 검증이 선결.
3. **연구 A 색상 보정** — 즉시 적용 가능한 안전한 소폭 개선(Dino +6, FP 0). texture가 준비되기 전 임시로
   병행 가능(무해).

# 17. 위험 및 Rollback
- **provisional 라벨·소 N**: 연구 B 수치는 지표적. 사람 GT 재검증 전 운영 임계 확정 금지.
- **rescue FP 재유입**: 무차별 texture rescue가 HSV가 옳게 지운 FP를 되살릴 위험 → 반드시 측정 후 채택.
- **A overshoot**: +9 초과 시 Dino TP 급감(25) → 보정값 고정 필수.
- Rollback: 모두 연구 폴더 산출물(캐시/스크립트). 운영 무변경이라 삭제만으로 원복. 원본 템플릿·모델·PLY
  무수정.

---

## 최종 결론

```text
연구 A — Dinosaur 템플릿 색상 보정
선택한 단일 보정: +9° OpenCV hue shift, Dinosaur 42 템플릿에만 (LODO 6폴드 전부 +9로 안정)
Dinosaur 결과:
- 보정 전 TP/FP/FN: 33 / 0 / 55
- 보정 후 TP/FP/FN: 39 / 0 / 49  (LODO held-out pooled 38)
- 오탈락 22건 중 복구: 6
- 새 FP: 0
전체 결과: TP 621 / FP 86 / FN 314 / F1 0.7564
다른 9개 객체 변화: 없음 (reference 미변경, 정의상 불변)
연구 A 판정: 채택 (안전·소폭; 단일 재중심의 원리적 상한으로 6건만)

연구 B — Texture 기반 초코하임 검증
가장 좋은 texture 방식: P2 = block11 patch × 42템플릿 max (AUROC 0.92, 현행 P0 0.875 대비 향상)
갈색 상자와 초코하임:
- AUROC: 0.92
- 수정 FP: thr 0.625서 갈색박스 43/46 제거 (0.649서 46/46)
- 손상 TP: 초코 1/12 (thr 0.625)
전체 결과: (라벨셋 한정) 초코 FP 대폭 감소; 전체 935 외삽은 사람 GT 필요
Texture negative verifier 판정: 채택 (사람 GT 검증 조건부; 런타임 무료·객체무관)
Texture rescue verifier 판정: 추가 검증 (Dino 19/22 유망하나 FP 재유입 미측정)
Dinosaur 부가 결과:
- 복구 TP: 19/22 (동일 임계 0.62)
- 새 FP: 미측정(핵심 미결)
- 별도 파라미터 필요 여부: 불필요 (Dino p10 ≈ choco 임계)
연구 B 판정: 채택(neg-verifier, GT검증) / 추가검증(rescue)

두 연구의 관계:
- 연구 A 성공 여부: 성공(소폭)
- 연구 B 성공 여부: neg-verifier 성공, rescue 유망
- 서로 독립적으로 채택 가능한지: 예
- 결합 시 예상 결과: Dino는 texture rescue가 color 상위집합(19>6); 갈색박스 FP는 B만 해결 → 결합은 B 주도

최우선 다음 단계:
연구 B texture verifier를 사람 GT로 재검증(초코 갈색박스 임계·마진 확정) + rescue의 FP 재유입 측정.
그 후 Phase 1C = HSV active + texture negative verifier (+ 검증되면 rescue). 연구 A(+9)는 즉시 병행 가능한
무해한 소폭 개선.

Phase 1C 판단: 추가 검증 필요 (texture verifier 사람 GT 확정 + rescue FP 재유입 측정이 선결)

운영 코드 변경: 없음
```

## 다음 액션 제안
① **연구 B를 사람 GT로 재검증**(초코 갈색박스 라벨 확장 + rescue FP 재유입) → Phase 1C 설계 확정.
② **연구 A(+9) 즉시 병행**(무해한 Dino +6). ③ 둘 다 보류하고 Phase 1C를 HSV-only(t 0.1214, Dino 33 감수)로
진행. 어느 쪽으로 갈까요?

---

# 18. 검증 추가 (사용자 옵션 ① — 사람 GT 재검증 + rescue FP 재유입)

Phase 1C 확정 전 필수 검증 2건을 실측했다. **결과가 이전 낙관을 뒤집었다.**

## 18.1 Choco texture 음성검증기 — 사람 GT 재검증
provisional 라벨 대신 **사람 GT(프레임 가시성)**로 재측정. 84 accepted choco = 52 choco-visible / 32 FP.
- **texture P2 AUROC = 0.834** (provisional 0.92보다 하락 — provisional이 낙관적이었음).
- 임계 스윕: thr 0.60 → 초코 47/52 유지·FP 8/32 제거 / thr 0.625 → 초코 39/52(−13)·FP 26/32.
- **판정 수정: 실측상 음성검증기는 modest** — 낮은 TP손실로 FP 대량제거하는 깨끗한 점(margin) 없음.
  choco-vis 0.674 vs FP 0.609로 겹침. 채택하려면 초코 TP 손실을 감수해야 함(clean gate 아님).

## 18.2 Texture rescue — FP 재유입 (280 HSV-rejected 전수)
HSV-오탈락 accepted 280건(TP casualty 32 / 옳게제거 FP 248)에 texture P2:
- **TP-casualty mean 0.659 ≈ 옳게제거-FP mean 0.658 (동일).**
- **rescue-verifier AUROC = 0.508 (우연 수준).** texture는 HSV-오탈락 진짜TP와 HSV가 옳게제거한 FP를
  **구분 불가.**
- thr 0.62: TP 29/32 rescue **하지만 FP 194/248 재유입**(Bear 62/68, Rabbit 67/68, Dino 36/39). 순 −165.
- **근본 이유**: HSV가 제거한 FP는 애초에 **texture가 유사**해서 sem+appe를 통과한 것(cross-fire 인형·
  갈색박스). **오직 색만 분리**했으므로 texture로는 재분리 불가 → rescue가 FP를 대량 되살림.
- **판정: rescue verifier 기각.** 이전 "Dino 19/22 rescue"는 FP 비용을 무시한 오해였음(정정).

## 18.3 수정된 결론
- **Texture rescue verifier: 기각**(AUROC .508, FP 194/248 재유입).
- **Texture negative verifier(choco): modest**(사람 GT AUROC .83, 초코 TP 손실 동반). 선택적, 임계 튜닝 필요.
- **Dinosaur 안전 복구는 연구 A(색 +9°, +6, FP 0)뿐.** texture는 Dino를 못 살림(rescue 사망).
- **Phase 1C 수정 판단: GO** — HSV active(t 0.1214, 순이득 F1 .676→.753) + **연구 A Dino 색보정(+6)**. texture
  negative verifier는 사람 GT로 임계 확정 시 **선택적** 추가(초코 FP 일부 제거, 단 clean 아님). rescue는 배제.

`results/choco_humanGT_revalidation.csv`, `results/rescue_fp_readmission.csv`.
