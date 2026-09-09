# Phase 1B 시각화 — HSV shadow가 하는 일 (그리고 그 대가)

**HSV hard gate의 역할:** semantic/appearance가 형태·질감으로 통과시킨 후보를 **색으로 한 번 더**
검사한다. Phase 1B는 **shadow**라서 점수만 계산·기록하고 **실제 판정은 바꾸지 않는다**. 오른쪽 패널은
"활성화됐다면 이렇게 됐을 것"이라는 **시뮬레이션**이지 실제 출력이 아니다.

주황 점선 = "SHADOW: would reject"(색 불일치). 색만이 아니라 텍스트로도 표시했다.
(OpenCV 한글 미지원 → 이미지 라벨 영어)

---

## 1. `shadow_crossfire_fp_would_reject.png` — HSV가 잡아내는 이득

- **dataset / frame:** `sam_105652` / 175 — 상자 안 객체는 **갈색 곰 인형**
- **Phase 1A (HSV off):** 이 박스가 **"Rabbit"으로 오검출**됐다(FP). 'white rabbit doll' 프롬프트가
  갈색 곰에 반응한 **cross-fire**이고, sem 0.45·appe 0.68이 통과시켰다. GT: Rabbit **미가시**(정답은 곰).
- **Phase 1B shadow:** HSV 색 점수 **0.0108 < 0.1214** → 활성 시 **거절**. 갈색 곰의 색이 Rabbit의
  흰색 렌더 템플릿과 전혀 안 맞기 때문. **Phase 1A가 새로 만든 cross-fire FP를 HSV가 정확히 제거**한다.

이것이 Phase 1A의 FP +22를 상쇄하는 원리다: 색이 다른 오검출은 sem/appe로는 못 걸러도 HSV로는 걸린다.

## 2. `shadow_dinosaur_tp_casualty.png` — HSV의 대가(정직한 downside)

- **dataset / frame:** `sam_110633` / 225 — **진짜 Dinosaur**(연두색 인형), GT 가시
- **Phase 1A:** 정상 검출(TP).
- **Phase 1B shadow:** HSV 점수 **0.10 < 0.1214** → 활성 시 **거절**(오탈락). 렌더 템플릿의 초록과
  실제 카메라의 초록 사이 **domain gap** 때문에 진짜 Dinosaur까지 걸린다.

이런 오탈락이 **Dinosaur에서 22건**(TP 55→33) 발생한다. HSV의 이득(cross-fire FP 제거)에는 이 대가가
따른다 — **shadow mode의 존재 이유가 바로 이것을 활성화 전에 드러내는 것**이다(AC-7).

---

## 정량 지표 (예시가 아닌 판단 근거)

출처 `_ism_research_2026_07/phase1b_validation/b1b2_*.csv` (사람 GT 935, 동결 t=0.1214):

- 전체: B1 TP648/FP334/FN287 → **B2-sim TP616/FP86/FN319**(ΔFP **−248**, ΔTP −32, F1 .676→**.753**)
- HSV would-reject 내역: **제거한 FP 248 : 손상한 TP 32** (약 7.8:1)
- **손상 TP 32건 중 22건이 Dinosaur** — 유일한 심각 단일객체 붕괴. Phase 1C는 이 처리(Dinosaur 예외
  또는 색보정) 확정 후 활성화 권장.
- 오른쪽 패널은 **shadow simulation**이며 현재 운영 판정은 **바뀌지 않았다**.
