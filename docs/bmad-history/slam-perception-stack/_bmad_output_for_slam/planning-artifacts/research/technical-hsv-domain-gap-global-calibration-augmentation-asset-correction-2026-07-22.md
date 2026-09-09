---
title: 'HSV domain-gap 해결 연구 — Dinosaur TP 손실 원인 분석 및 일반화 색상 처리 결정'
type: 'technical-research'
date: '2026-07-22'
scope: '오프라인 연구만. 운영 코드/config/템플릿 무수정, git 무조작.'
authority: 'ism_hsv.py (Phase 1B); pipeline/cur 덤프; 사람 GT 935; prototype_colors.npz'
workdir: 'sam6d_ws/_ism_research_2026_07/hsv_domain_gap_resolution/'
---

# HSV domain-gap 해결 — Dinosaur TP 손실 원인과 일반화 색상 처리

## 요약 (핵심 3줄)

1. **Dinosaur 자산은 잘못되지 않았다.** 운영 PLY·대체 PLY 3종·텍스처·ply_view가 전부 동일한
   황록색(Hue ~83°)이고, 더 초록인 "올바른" 대체 자산은 **존재하지 않는다** → **C3 기각**.
2. **진짜 원인은 C+F**: 렌더는 저채도 황록(H78°)인데 **실제 Dinosaur의 색은 H68~108°로 넓게
   분포**(시점·조명). 오탈락된 22건은 갈색이 아니라 **더 파란-초록(H~102°)인 진짜 Dinosaur**로,
   단일 reference의 hue 커버리지를 벗어난다(육안 crop으로 확정).
3. **C1(전역보정)·C2(augmentation)·보수적 threshold 모두 성공기준 미달** — 색만으로는 Dinosaur
   복구와 FP≤86을 **동시 달성 불가**(greener dino view가 FP와 색공간에서 겹침). → 색 방법 채택
   안 함. **Dinosaur TP 손실은 색이 아니라 block2+block11 texture 검증기(Story 2)로 보완**.

---

## 1. 근본 원인 분석 (asset_audit)

### 1.1 자산 색상 감사 (`results/dinosaur_asset_audit.csv`)

OpenCV Hue(0-180)와 0-360 환산:

| 색원 | Hue(0-360) | S | n |
|---|---:|---:|---:|
| 운영 PLY vertex color | **82.6°** | 128 | 591,129 |
| 대체 PLY high / middle / low | 82.6 / 84.1 / 84.0 | ~130 | — |
| 텍스처 PNG (Dinosaur_color.png) | 82.9° | 132 | 16.8M |
| ply_view42 (가시 vertex) | 82.3° | 126 | — |
| **renderer 출력 (render42)** | **77.5°** | **92** | 840,000 |
| 실제 Dino PASS(TP) | 86° | — | 33 |
| 실제 Dino REJECT (육안 확인) | **~102°** | — | 22 |

→ **모든 Dinosaur 자산이 82~84°로 완전 일치.** renderer만 채도를 128→92로 낮추고 hue를 83→77.5°로
살짝 이동(정상 셰이딩 범위). 실제 Dino는 PASS 86° / REJECT ~102°.

### 1.2 오탈락 crop 육안 확인 (`visualizations/dinosaur_reject_vs_pass_crops.png`)

**초기 집계 통계(rejected median 9.5°=갈색)는 히스토그램 marginal 계산 artifact였다.** 실제 bag에서
추출한 22개 오탈락 crop은 **전부 명백한 초록 Dinosaur 인형**(H94~108), 마스크·위치 정상. PASS crop은
H68~94(렌더에 가까움). 즉 **오탈락은 "더 파란-초록으로 보이는 시점"의 진짜 Dinosaur**다.

### 1.3 원인 판정 (A~G)

| 후보 | 판정 |
|---|---|
| A. PLY vertex color가 실물과 다름 | **기각** (PLY=텍스처=대체 모두 83°, 일관) |
| B. renderer가 연두로 왜곡 | 부분(채도↓·hue −5°) 이나 주원인 아님 |
| **C. 공통 Hue 보정이 Dinosaur에 안 맞음** | **성립** (−6° 전역보정은 대다수엔 맞으나 Dino는 +green 필요) |
| D. 카메라 WB/조명 | 부분 기여(실 Dino가 시점별로 greener) |
| E. mask가 배경 포함 | **기각** (crop 육안 마스크 정상) |
| **F. 실제 Dino가 시점·그림자로 여러 색 분포** | **성립** (실 Dino H68~108, 렌더 H78) |
| G. 판정 불가 | 해당 없음 |

**결론: C + F.** 단일 hue-보정 렌더 reference가 실제 Dinosaur의 넓은 hue 분포를 못 덮음. **자산 오류
아님.**

---

## 2. 세 방법 비교 (`results/method_comparison.csv`)

동일 authority(H+S 16×8, L1, Bhattacharyya max, 공통 t=0.12140, 저채도예외 없음), reference bank만 변경.
사람 GT 935, B2-shadow-sim.

| 방법 | TP | FP | FN | F1 | Dino TP | 판정 |
|---|---:|---:|---:|---:|---:|---|
| **B0** (현재 hue-보정 렌더) | 616 | **86** | 319 | **0.7526** | 33 | 기준 |
| C1gw (gray-world WB, 전객체) | 540 | 190 | 395 | 0.6486 | 34 | **기각**: Bear 49→8·Sikhye 89→54 붕괴 |
| C1h (LOO 전역 hue offset) | 616 | 87 | 319 | 0.7521 | 33 | **기각**: 최적 offset=0(현재가 이미 전역최적) |
| C2a (hue-aug ±5,±10 ×5) | 623 | **124** | 312 | 0.7408 | **39** | **기각**: Dino +6이나 FP +38, F1↓ |
| C2b (hue-aug ±3,±6 ×5) | 617 | 124 | 318 | 0.7363 | 33 | **기각**: Dino 무복구, FP↑ |

### 보수적 threshold 재설계 (fallback 1)

| t | TP | FP | F1 | Dino TP |
|---:|---:|---:|---:|---:|
| 0.05 | 623 | 143 | 0.7325 | **38** |
| 0.06 | 623 | 129 | 0.7386 | 38 |
| 0.07 | 622 | 116 | 0.7436 | 37 |
| **0.1214** | 616 | **86** | 0.7526 | 33 |

→ **22개 Dino 오탈락은 전부 hsv ∈ [0.06, 0.1214].** t≤0.06이면 Dino 완전 보존(38)이나 FP 129~143.
**FP≤86과 Dino=38을 동시에 주는 t는 없다**(coupled trade-off, `threshold_tradeoff_and_hue_spread.png`).
근본 이유: greener dino view(H102)가 cross-fire/배경 FP와 색공간에서 겹침 → 색만으로 분리 불가.

---

## 3. 성공기준 대조

| 기준 | C1 | C2 | C3 | 보수적 t |
|---|---|---|---|---|
| 1. Dino 22 복구 | ✗ | 부분(+6) | — | 부분(≤0.06서 +5) |
| 2. 타 9객체 TP 무손상 | ✗(Bear/Sikhye) | ✓ | — | ✓ |
| 3. FP ≤ 86 | ✗ | ✗(124) | — | ✗(129~143) |
| 4. 단일 공통 threshold | ✓ | ✓ | — | ✓ |
| 5. 런타임 분기 없음 | ✓ | ✓ | — | ✓ |
| 7. authority/cache 재사용 | ✓ | ✓ | — | ✓ |

**세 방법 모두 (1)+(2)+(3) 동시 충족 실패** → 사용자 §8에 따라 **HSV active를 색상 방법으로 강행하지
않는다.**

---

## 4. 반드시 판단할 7문항 답

1. **Dino PLY/템플릿이 잘못된 자산인가?** — **아니오.** 전 자산 83° 일치, 대체 자산 동일.
2. **연두 vs 초록 차이는 자산 오류인가 domain gap인가?** — **domain gap(C+F).** 실 Dino가 H68~108로
   넓고, 렌더(H78)는 그 중 황록쪽만 대표.
3. **전역 calibration으로 복구되는가?** — **아니오.** gray-world는 타객체 붕괴, 전역 hue offset 최적=0.
4. **공통 augmentation으로 FP 유지하며 복구되는가?** — **아니오.** Dino +6에 FP +38(>86).
5. **Dinosaur 자산만 수정이 정당한가?** — **아니오.** 독립 근거 없음(자산 오류 미확인). 채택 금지.
6. **자산 수정 후 알고리즘이 전객체 동일한가?** — 자산 수정 미채택이므로 무관(런타임 이미 객체무관).
7. **Phase 1C reference는?** — **B0(현재 hue-보정 렌더) 그대로, threshold 0.12140 그대로.** 색 변경 없음.

---

## 5. 시각화

### `dinosaur_reject_vs_pass_crops.png`
![crops](../../../sam6d_ws/_bmad-output/implementation-artifacts/hsv-domain-gap-visualizations/dinosaur_reject_vs_pass_crops.png)

렌더 템플릿(H78 밝은 황록) vs 실제 PASS Dino(H68~94 초록) vs 실제 REJECT Dino(H94~108 더 파란 초록).
**오탈락은 갈색이 아니라 더 초록인 진짜 Dinosaur** — 자산 오류가 아니라 hue 분포 폭 문제임을 육안 확정.

### `threshold_tradeoff_and_hue_spread.png`
![tradeoff](../../../sam6d_ws/_bmad-output/implementation-artifacts/hsv-domain-gap-visualizations/threshold_tradeoff_and_hue_spread.png)

좌: t를 낮추면 Dino 복구(→38)되나 FP 급증(→143), FP≤86은 t=0.1214에서만(Dino=33) — **동시 불가**.
우: 실제 Dino hue가 렌더보다 넓게 분포(reject 뷰가 greener).

---

## 6. 최종 결론

```text
Dinosaur TP 손실의 근본 원인:
렌더 reference(저채도 황록 H78)가 실제 Dinosaur의 넓은 hue 분포(H68~108, 특히 greener 시점 H~102)를
덮지 못함. 원인 유형 C(공통 hue보정 부적합) + F(실물 색 분포 넓음). 자산 오류 아님.

Dinosaur 원본 자산 오류 여부:
기각 — 운영 PLY 82.6°, 대체 PLY 3종 82.6~84°, 텍스처 82.9° 전부 일치. 더 초록인 대체 자산 없음.

현재 PLY와 템플릿의 문제:
문제 없음(자산은 실물 황록을 충실히 표현). renderer가 채도만 128→92로 낮춤(정상 범위).

후보 1 — 공통 color calibration:
- Dinosaur TP: 33~34 (복구 실패)
- 전체 TP/FP/FN: gray-world 540/190/395, 전역hue 616/87/319
- 다른 객체 손상: gray-world에서 Bear 49→8, Sikhye 89→54 (심각)
- 판정: 기각

후보 2 — 공통 augmentation:
- Dinosaur TP: 39 (+6)
- 전체 TP/FP/FN: 623/124/312
- 다른 객체 손상: 없음
- 판정: 기각 (FP 86→124로 성공기준 3 위반)

후보 3 — Dinosaur 자산 교정:
- 교정 근거: 없음 (자산 오류 미확인)
- 수정한 자산: 없음
- Dinosaur TP: N/A
- 전체 TP/FP/FN: N/A
- 런타임 특별 처리 여부: 없음
- 판정: 기각 (자산 오류 독립 근거 부재 → 규칙상 채택 금지)

최우선 권장안:
색상 방법 채택하지 않음. HSV reference·threshold를 B0/0.12140 그대로 유지하고, Dinosaur(및 Bear/Rabbit
소수)의 HSV 오탈락은 **block2+block11 texture 조건부 검증기(Story 2)로 rescue**한다. 즉 "색 OR 텍스처"
중 하나라도 확증하면 TP 유지 — 객체무관 알고리즘이라 런타임 Dinosaur 분기 불필요.
(즉시 active가 필요하면 임시로 보수적 t≈0.07: Dino 37/FP 116/F1 .744, 단 FP>86 감수.)

권장 이유:
1) 색만으로는 Dino 복구와 FP≤86이 원리적으로 양립 불가(greener dino가 FP와 색공간에서 겹침).
2) 자산 오류가 아니므로 자산 교정은 데이터 맞춤이 되어 금지 규칙 위반.
3) texture 검증기는 이미 연구에서 강함(block2+block11 AUROC 0.94)이고 색과 직교 신호라 FP 재유입 없이
   TP를 살린다. 전객체 동일 적용 → 신규 객체도 추가 분석 불필요.

Phase 1C 판단:
추가 검증 필요 — HSV active(t=0.1214) 자체는 순이득(F1 .676→.753, FP 334→86)이라 진행 가치가 크나,
Dinosaur TP 22건 회복은 색이 아니라 Story 2(texture 검증기) 완료 후 확정. 색 방법으로 Dinosaur를
강제 복구하지 말 것.

런타임 객체별 예외:
없음

신규 객체 적용 방식:
표준 42-view 렌더 → 공통 hue보정 HSV 캐시 → 공통 threshold 0.12140. (색 방법 변경 없음.) TP 손실
보완이 필요하면 동일한 객체무관 texture 검증기가 자동 적용.

운영 코드 변경:
없음
```

## 7. 다음 액션 제안

- **(추천) Story 2 착수**: block2+block11 조건부 texture 검증기를 "HSV-rejected 후보 rescue" 용도로
  오프라인 설계·검증 → Dinosaur 22건 중 texture로 확증되는 비율 측정. 성공 시 Phase 1C = active HSV +
  texture-rescue (전객체 동일).
- **(대안) 즉시 Phase 1C**: HSV active를 t=0.1214 그대로 켜고 Dinosaur 33 손실을 감수(순이득 F1 .753).
  또는 보수적 t=0.07(Dino 37, FP 116).
- 어느 쪽이든 **색 기반 Dinosaur 보정·자산 수정은 채택하지 않음**(근거 없음/양립 불가).

진행 방향을 선택해 주세요: ① Story 2(texture rescue) 먼저 ② Phase 1C 즉시(t=0.1214 감수) ③ Phase 1C
보수적 t=0.07.
