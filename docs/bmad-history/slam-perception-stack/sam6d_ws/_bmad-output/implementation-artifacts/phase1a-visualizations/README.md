# Phase 1A 시각화 — 객체별 threshold guard 제거로 Rabbit 복구

**변경 한 줄 요약:** Rabbit/Bear/Dinosaur에 걸려 있던 높은 YOLO `score_threshold` guard
(0.30/0.35)를 없애고 전 객체를 `0.02`로 통일. → guard가 죽이던 **진짜 Rabbit 후보가 살아나
후단 ISM까지 전달**되어 정상 인식(TP)됨.

두 이미지 모두 **실제 `SAM_*` bag 프레임 + 실제 B0/B1 판정**이며, 합성·재구성하지 않았다.
초록 상자 = 검출(TP), 주황 점선 = threshold에서 제거된 후보(FN). 색만이 아니라 `TP`/`FN`/
`REMOVED by threshold` 텍스트도 함께 표시했다. (OpenCV가 한글을 못 그려 이미지 안 라벨은 영어)

---

## 1. `before_after_rabbit_recovered.png` (대표 사례)

- **dataset / frame:** `sam_110633` / frame 72
- **GT:** Rabbit visible (사람 라벨), box (114, 258, 213, 350) — 흰 토끼 인형이 테이블 위에 **크고 뚜렷하게** 보임
- **Before (B0, 객체별 guard):** Rabbit 후보 conf **0.2995** < guard **0.30** → **후보 단계에서 제거** → Rabbit **미검출(FN)**
- **After (B1, 통일 0.02):** 동일 후보 conf 0.2995 ≥ 0.02 → 살아남음 → semantic **0.547 ≥ 0.35**, appearance **0.733 ≥ 0.55** 통과 → **Rabbit 정상 검출(TP)**

conf 0.2995가 guard 0.30에 **0.0005 차이로 걸려 탈락**하던 전형적 사례다. 낮은 conf가 곧 오검출을
뜻하지 않는다는 것(후단 sem/appe가 실제로 통과)을 한눈에 보여준다.

## 2. `before_after_additional_recovery.png` (교차 데이터 일반화)

- **dataset / frame:** `sam_110104` / frame 382 — **다른 bag**
- **GT:** Rabbit visible, box (15, 218, 65, 252) — 왼쪽 먼 테이블의 **작은/원거리** 인스턴스
- **Before:** conf **0.1941** < guard 0.30 → 제거 → FN
- **After:** conf 0.1941 ≥ 0.02 → sem **0.616 ≥ 0.35**, appe **0.743 ≥ 0.55** 통과 → TP

대표 사례와 **다른 데이터셋**에서, 그리고 **더 작고 먼** 인스턴스에서도 같은 복구가 일어남을 보인다.
Rabbit 복구는 특정 프레임 한 건이 아니라 **4개 데이터셋 23프레임**에서 관찰됐다(대표성 근거).

---

## 주의 (정량 지표로 판단할 것)

- 이 이미지는 **예시**다. 전체 성능은 아래 정량 지표로 판단해야 한다(출처
  `_ism_research_2026_07/phase1a_validation/b0b1_*.csv`, 사람 GT 935 가시 인스턴스, HSV OFF):
  - Rabbit: **TP 38 → 60, FN 56 → 34** (B0 → B1)
  - 전체: TP 623 → 648(**+25**), FN 312 → 287(**−25**), **FP 312 → 334(+22)**
- **중요:** 위 FP **+22**는 Phase 1A(HSV 미적용)의 정직한 결과다. guard가 막던 것은 Rabbit 진짜
  후보만이 아니라 **cross-fire 오검출**('brown bear doll' 프롬프트가 흰 토끼에 conf 0.17~0.24로
  반응하는 등)도 포함되는데, 이를 색으로 걸러내는 **HSV hard gate가 Phase 1B/1C(이번 범위 밖)**
  이기 때문이다. 연구에서 확인된 "FP ±0"은 threshold 통일을 **HSV와 함께** 적용했을 때의 결과다.
