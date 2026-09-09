---
stepsCompleted: [1]
workflowType: 'research'
research_type: 'technical'
research_topic: 'Real-Time Safe Detect-then-Segment Alternatives for SAM-6D FP Reduction'
research_goals: '시간 성능 악화 없이 앞단 후보생성/no-object rejection을 개선해 FP를 줄일 구조가 있는지 판정'
user_name: 'Ldh9501'
date: '2026-06-10'
source_verification: true
---

# Technical Research — Real-Time Safe Detect-then-Segment for SAM-6D FP Reduction

> **로컬 baseline(실측)**: rate-0.25 검증 run 로그 345프레임 파싱(`/tmp/rerun_logs/launch2.log`). 후보 런타임은 웹조사(YOLO-World/GroundingDINO/SAM2/OWLv2/RT-DETR 논문·repo). HW = **RTX PRO 6000 Blackwell 96GB**(매우 강력 → 절대 latency는 인용치보다 빠름, VRAM 비제약). 코드 무수정.

---

## Executive Summary — 결론 5개

1. **병목은 앞단(proposal)이 아니라 SAM-6D 본체(ISM-DINOv2 + PEM)다.** 프레임당 총 **0.878s(~1.14 FPS)**, 그중 **GPU추론 0.694s(79%)**. FastSAM은 경량 **FastSAM-s**라 비중 작음. → **앞단에 가벼운 detector를 더할 여유가 충분하다**(상대비용 수 %).
2. **앞단 open-vocab 게이트는 latency를 오히려 줄일 수 있다(핵심).** 검증셋의 **62%(197/315)가 no-object** 프레임이다. 앞단에서 "milk 없음"이면 0.7s짜리 ISM/PEM을 **건너뛰면**, 새 모델 비용(수~수십 ms)을 압도하는 평균 latency **감소**가 가능 → 제약("유지 또는 더 빠름") 충족.
3. **유일한 real-time-safe open-vocab 후보 = YOLO-World(재파라미터화·캐시 임베딩).** "milk carton" 프롬프트를 오프라인 vocab으로 고정하면 text encoder가 제거되어 **YOLOv8 수준 속도**(S: 13.5ms/74FPS@V100, Blackwell+TRT 한자릿수 ms). 캐시가 설계상 내장. GroundingDINO(106ms PT)·**Grounded-SAM(800ms+)·OWLv2(300ms)는 Reject**.
4. **SAM2(D)는 텍스트 detector가 아니라 부적합·과잉.** ~22ms/frame이지만 point/box 프롬프트만 — "milk 유무" 판정 불가. transient FP 억제는 **SAM2 없이 노드 내 temporal voting(거의 0비용)**으로 충분 → SAM2 도입 가치 낮음.
5. **DINOv2-feature 재사용 presence classifier(E)는 가장 싸나 천장에 막힌다.** 이미 추출된 ISM cls-token 재사용=추가비용≈0이지만, 이는 QD-A가 검증한 [sem,appe,geo] 결합과 동일 정보원 → **AUC≈0.91 ceiling**(Appearance Ambiguity)을 못 넘음. 독립 신호인 YOLO-World가 필요한 이유.

---

## Current Runtime Bottleneck (Task 1)

| 단계 | mean (s/frame) | 비중 |
|---|---|---|
| **GPU추론(ISM DINOv2 + PEM)** | **0.694** | **79%** |
| 렌더링(geometric 투영) | 0.072 | 8% |
| 전처리(PEM point cloud 등) | 0.060 | 7% |
| 기타(노드 오버헤드) | 0.038 | 4% |
| 파일저장/로드 | 0.01 | 1% |
| **총** | **0.878 (~1.14 FPS)** | |

- segmentor = **FastSAM-s**(경량, `fast_sam.yaml` checkpoint FastSAM-s.pt) → proposal 생성은 병목 아님.
- GPU 0.694s 내 ISM/PEM 정확 분할은 INFO 로그에 없음(debug-level 미기록) → [추정] SAM-6D 특성상 **PEM(coarse+fine point matching, 2048pts) + 후보별 DINOv2가 지배**, FastSAM-s는 minor. (정밀 분할은 debug timing 계측 필요.)
- **여유 판정**: 앞단에 ~10–30ms detector 추가는 +1~4%. **no-object 프레임에서 ISM/PEM 스킵 시 −0.7s/frame** → 평균적으로 순감(net faster).

---

## Candidate Architecture Comparison + Real-Time Safety (Task 2·3)

| Candidate | FP 감소 | Recall 위험 | Latency 영향 | VRAM | ROS2 통합비용 | Real-time? |
|---|---|---|---|---|---|---|
| **A. YOLO-World→SAM-6D** | **높음**(독립 open-vocab 의미게이트) | 중(milk 검출 신뢰도·domain gap) | **−(absent서 ISM/PEM 스킵)~+한자릿수ms** | 작음(Blackwell 비제약) | 중 | **Real-time Safe ✅** |
| B. GroundingDINO/Grounded-SAM | 높음 | 중 | **+106ms(GD)~800ms+(G-SAM)** | 큼 | 높음 | **Reject ✗** |
| C. Open-vocab(=YOLO-World) | =A | =A | =A | — | — | **=A(YOLO-World만 Safe)** ; OWLv2 300ms✗, RT-DETR 빠르나 **closed-set✗**(open-vocab 아님) |
| D. SAM2 temporal | 간접(transient만) | 낮음 | +~22ms/frame | 중 | 높음 | **Risky/과잉** — 텍스트게이트 아님, 노드 temporal voting로 대체 가능 |
| E. DINOv2-reuse presence cls | 낮음(천장 0.91) | 낮음 | **≈0(feature 재사용)** | ≈0 | 낮음 | Real-time Safe이나 **정확도 천장**(=QD-A 결과) |

**판정**: A = Real-time Safe(유일), C는 A로 수렴(RT-DETR은 closed-set이라 별도 custom-detector로만 의미), B/OWLv2 = Reject, D = Risky/과잉, E = Safe-but-capped.

---

## Accuracy Risk Analysis (Task 4)

| 후보 | no-object FP | 흰박스/모니터 FP | milk recall 유지 | custom object 적응 | domain gap |
|---|---|---|---|---|---|
| **A YOLO-World** | **강함**(box 없으면 no-object) | **강함**(milk-carton 의미로 흰물체 배제) | **검증 필요**(YOLO-World가 실제 우유팩·시점·조명서 검출하나?) | 프롬프트/소량 finetune | open-vocab이라 작음~중(특정 팩 외형은 확인 필요) |
| B Grounded-SAM | 강함 | 강함 | 유사 | text prompt | 작음 (단 너무 느림) |
| D SAM2 | 약함(유무판정 못함) | 약함 | 높음(연속성) | n/a | n/a |
| E presence cls | 약함(천장) | 약함 | 높음 | 쉬움 | 큼(milk 특화) |

**핵심 리스크 = A의 recall(domain gap).** YOLO-World가 이 우유팩을 실제 프레임에서 안정적으로 잡는지가 GO의 전제 — **shadow 실험으로 먼저 측정해야** 한다(아래).

---

## Integration Strategy (Task 5, 시간악화 없는 방식만)

1. **YOLO-World front gate + ISM/PEM short-circuit** (권장): 매 프레임 YOLO-World("milk carton/bottle" 캐시 임베딩) → box 0개면 즉시 **no-object 발행, ISM/PEM 스킵**. → absent 62% 프레임에서 0.7s 절감, 평균 FPS↑.
2. **box로 FastSAM/ISM 후보 제한**: box 있으면 그 ROI에만 FastSAM/ISM 적용 → 후보수↓로 PEM 비용도 상쇄.
3. **keyframe + temporal cache**: YOLO-World를 매 프레임이 부담되면 keyframe만 실행하고 사이는 tracking/cache(단 Blackwell선 매 프레임도 가능).
4. **저비용 보조(병행)**: 직전 연구의 `blue/color consistency`를 YOLO-World 게이트의 보조 tie-break로(코드 무수정 shadow 검증 후).
5. **금지 회피**: GroundingDINO/SAM2/OWLv2는 매 프레임 추가 금지(무거움) — 통합전략에서 제외.

---

## Recommended Architecture

**YOLO-World-S(v2, 재파라미터화·"milk carton" 캐시) → front gate → (box 있으면) 기존 FastSAM-s+ISM+PEM(ROI 제한).**
- 시간: absent 프레임 ISM/PEM 스킵으로 **평균 latency 감소**(net faster), present 프레임 +한자릿수 ms.
- FP: DINOv2-template ambiguity와 **독립적인** open-vocab 의미 게이트라 흰박스/모니터/벽 FP를 앞단 차단(E/QD-A의 천장을 우회).
- 단 전제 = YOLO-World의 milk recall이 충분해야 함 → shadow 검증 필수.

---

## Go / No-Go Recommendation

### **GO-A** (조건부)
- 구조 변경 후보 **존재**(YOLO-World front gate), FP 감소 가능성 **높음**, 시간성능 **악화 없음~오히려 빨라질 가능성**(absent 스킵). → GO-A 충족.
- **단 조건**: YOLO-World의 이 우유팩 recall·domain gap 미검증 → 실측 전 full 통합 금지. recall 부족 시 GO-B(shadow-only)로 강등.

---

## Suggested Next BMAD Step

### **Quick-Dev shadow experiment** (PRD 아님, 추가 Research 아님)

근거: 후보·런타임 분석 완료, 남은 단일 미지수 = "YOLO-World가 이 데이터에서 milk를 잘 잡고 흰물체를 거르나"(recall/FP). 이는 **오프라인 shadow로 즉시 측정 가능**, 코드/ROS 무수정.

**Shadow 실험 설계(QD)**:
1. YOLO-World-S(ultralytics, 캐시 프롬프트 "milk carton","carton","milk bottle")를 `outputs/validation/.../frames` 311프레임에 오프라인 실행(별도 env/스크립트, ROS 무관).
2. frame별 box 유무·conf를 GT(visibility_labels)와 조인 → **present-vs-absent recall/precision, FP 감소율, milk recall 손실** 측정. 기존 baseline(P0.48/R0.95)·QD-A 천장(AUC0.91)과 비교.
3. 프레임당 YOLO-World latency 실측(Blackwell) → 평균 파이프라인 latency 영향 정량(absent 스킵 시뮬 포함).
4. 판정: recall 유지 & FP 큰 감소 & latency 비악화 → **PRD(실제 ROS detect-then-segment 통합)로 승격**. recall 부족/domain gap 크면 → 소량 finetune 또는 다른 프롬프트/모델(GO-B).

> PRD는 shadow가 GO를 확증한 *후*에. 지금 PRD는 시기상조(recall 미검증).

---

## 부록 — 재현 & 한계
**재현**: 로컬 timing = `/tmp/rerun_logs/launch2.log` 파싱(345프레임). 후보 런타임 = 웹 인용(아래). **한계**: ① GPU 0.694s의 ISM/PEM 분할은 debug-timing 미기록 → [추정](PEM 지배). ② 후보 latency는 인용 GPU(V100/A100/T4) 기준, Blackwell선 더 빠름(상대순서만 유효). ③ YOLO-World milk recall은 **미측정**(shadow 실험의 핵심). ④ FastSAM-s 확인(코드), 단 일부 자산엔 FastSAM-x.pt도 존재(config는 -s 사용).

**출처**: YOLO-World arXiv:2401.17270(재파라미터화 13.5ms/74FPS@V100) · GroundingDINO arXiv:2303.05499(9.4FPS PT@A100) · Grounded-SAM arXiv:2401.14159(800ms+) · SAM2 arXiv:2408.00714(43.8FPS@A100) · OWLv2 arXiv:2306.09683(~300ms) · RT-DETR arXiv:2304.08069(108FPS@T4, closed-set).

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 research .md 표준 — 이 절로 마무리.

**판정: GO-A(조건부).** YOLO-World front gate가 유일한 real-time-safe 후보이고 absent 스킵으로 오히려 빨라질 수 있음. 단 milk recall 미검증 → **Quick-Dev shadow 실험 먼저**.

선택지:
- **(A) YOLO-World shadow 실험 Quick-Dev** *(추천)* — 311프레임 오프라인 검출로 recall/FP/latency 측정, ROS 무수정. GO 확증 시 PRD.
- **(B) 바로 PRD(detect-then-segment 통합 설계)** — recall 자신 있으면; 단 미검증이라 위험.
- **(C) shadow에 blue/color 보조신호 + temporal voting 동시 평가** — 한 번에 보조전략까지.

→ **어느 쪽으로 진행할까요?** (추천: A — 저비용·무수정으로 GO 전제 확증)
