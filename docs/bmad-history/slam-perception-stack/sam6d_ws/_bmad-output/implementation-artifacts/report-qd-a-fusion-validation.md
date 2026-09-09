# QD-A 최종 보고서 — SAM-6D Eq.(4)를 Milk 데이터에서 능가할 수 있는가?

**날짜:** 2026-06-10 · **유형:** 오프라인 가설검증(프로덕션 코드 0줄 수정) · **스크립트:** `tools/validation/qd_a_fusion_benchmark.py` · **출력:** `outputs/validation/qd_a_fusion_benchmark_output.txt`
**데이터:** baseline 315 풀-GT(present 118 / absent 197), Eq.4 score = detection_ism.json best (커버리지 **315/315**).

---

## Executive Summary — 결론 5개

1. **Eq.(4)는 이미 최적에 가깝다.** present-vs-absent ROC-AUC: **Eq.4=0.910** > pose 0.894 > geo 0.886 > appe 0.873 > sem 0.858. 결합이 단일점수를 모두 능가 → 결합 자체는 정상 작동. **in-sample 로지스틱 회귀(0.909) ≈ Eq.4(0.910)** → 학습으로도 Eq.4를 못 넘는다(이미 선형결합 최적점).
2. **학습 fusion은 hold-out에서 Eq.4를 능가하지 못한다.** 5-fold CV ROC-AUC: C(logreg)=0.908±0.033 vs A(Eq.4)=0.911±0.018 → **ΔAUC=−0.003(개선 없음)**.
3. **유일하게 보였던 operating-point 이득이 일반화되지 않는다.** P@R0.90은 random CV에서 +0.064지만 **temporal split에서 −0.093으로 부호가 뒤집힌다.** R0.95에선 −0.058. → 견고한 개선 아님(운영점·분할 의존).
4. **Calibration 문제 아님.** Temperature/Platt는 단조변환이라 AUC·분리력 불변. ECE도 raw 0.160 → Platt 0.186(개선 없음). FP는 보정으로 줄지 않는다.
5. **Appearance Ambiguity가 근본 ceiling.** Ablation: geo 제거 −0.039(핵심), **appe 제거 +0.004(외관 feature는 기여 0/오히려 미세 유해)**. 결합의 분리력은 전부 geometric(bbox IoU)에서 나오고 appearance는 정보가 없다 → feature 정보량이 ROC-AUC≈0.91, PR-AUC≈0.88에서 상한.

---

## Eq.(4) Audit (Task 1)
- 논문 Eq.(4) `(s_sem+s_appe+r·s_geo)/(2+r)` = 코드 `detector.py:384` / `run_batch_inference_fast.py:416` **verbatim**(3자 감사 기확인).
- 수치 재현: detection_ism.json best-candidate `score`(=Eq.4 출력) 315/315 프레임 로드. **현재 프로젝트는 실제로 Eq.4를 사용**.
- 한계(명시): visible_ratio(r)는 저장 안 됨 → sem/appe/geo만으로 Eq.4를 독립 재계산은 불가. 대신 **시스템이 저장한 Eq.4 출력값**을 직접 사용(더 충실).

## Baseline Benchmark (Task 2)
| score | ROC-AUC | PR-AUC |
|---|---|---|
| **Eq.4 (A, baseline)** | **0.910** | **0.875** |
| pose(PEM) | 0.894 | 0.814 |
| geometric | 0.886 | 0.860 |
| appearance | 0.873 | 0.822 |
| semantic | 0.858 | 0.748 |

운영점(5-fold CV): Eq.4 P@R0.90=**0.669**, P@R0.95=0.588. (전체 baseline confusion: P=0.483/R=0.949 — 단 이는 노드 pose-floor 0.30 발행 기준이고, 위는 present-vs-absent 분리력 상한.)

## Alternative Fusion Benchmark (Task 3) — Eq.4 vs B/C/D
| Model | 정의 | CV ROC-AUC | 비고 |
|---|---|---|---|
| **A** Eq.4 | 고정 결합(baseline) | **0.911±0.018** | 기준 |
| **B** 가중합 | a·sem+b·appe+c·geo | ≈C (단조 동치) | 별도 이득 없음 |
| **C** logreg[sem,appe,geo] | 학습 결합 | 0.908±0.033 | **ΔAUC −0.003** |
| **D** logreg[+margin] | +top1-top2 margin | 0.897±0.104 (n=56) | **저파워**, Δ+0.029 неreliable |

→ B/C/D 어느 것도 Eq.4를 ROC-AUC에서 유의하게 능가하지 못함. (Model D의 margin 이득은 56프레임·std±0.10로 신뢰불가.)

## Hold-Out Results (Task 4)
| 검증 | A Eq.4 | C logreg | 판정 |
|---|---|---|---|
| 5-fold CV ROC-AUC | 0.911 | 0.908 | 동급(C 미세↓) |
| 5-fold P@R0.90 | 0.669 | 0.733 (+0.064) | CV에선 C 우세 |
| 5-fold P@R0.95 | 0.588 | 0.530 (−0.058) | C 열세 |
| **Temporal P@R0.90** | 0.616 | **0.523 (−0.093)** | **C 열세(역전)** |
| train-holdout gap | — | +0.001 | 과적합 없음(단, 이득도 없음) |

**핵심:** 과적합은 없으나 **개선도 없다.** CV의 +0.064는 temporal split에서 −0.093으로 뒤집혀 **견고하지 않다**(운영점/분할 우연).

## Calibration Analysis (Task 5)
- Temperature/Platt = 단조변환 → ROC/PR-AUC·분리력 **불변**. ECE raw 0.160 → Platt 0.186(미개선, 소표본).
- **FP는 calibration 문제가 아니다.** 보정은 임계해석만 바꿀 뿐 present/absent 분리를 못 늘린다.

## Feature Attribution (Task 6)
- 표준화 logreg 계수: **geo=+1.275**(지배), sem=+0.683, appe=+0.670, intercept −0.827.
- Ablation(CV ROC-AUC 0.908 기준): drop **geo → 0.869(−0.039)**, drop sem → 0.904(−0.004), **drop appe → 0.911(+0.004)**.
- → **분리력은 geometric(bbox IoU)이 전담, appearance는 기여 0(미세 유해).** 흰 저텍스처에서 외관 디스크립터가 무정보임을 정량 확인.

## Appearance Ambiguity Ceiling (Task 7)
- 최고 모델(C/Eq.4) ROC-AUC≈**0.91**, PR-AUC≈**0.88**이 현재 feature 정보량의 **천장**. 학습·보정·margin 추가 모두 이를 못 넘김.
- 천장의 원인 = appearance 무정보(ablation) + geometric만으로 부분 분리. **Appearance Ambiguity가 근본 ceiling으로 확정.**

## Generalization Risk Assessment (Task 8)
- C의 미세 이득은 (a) 특정 recall 운영점에서만, (b) temporal split에서 역전 → **milk/run 특화 과적합 위험**. 견고한 일반 개선 아님.
- 이득의 출처가 **geometric(depth/template-pose 투영)**이지 appearance가 아니므로, 외관 기반 FP를 본질적으로 못 줄임. 결합 재학습은 일반화 가치 낮음.

---

## GO / NO-GO Verdict

### **NO-GO**

근거(요청 기준 대조):
- Hold-out 개선 **없음**(ΔROC-AUC −0.003).
- Precision 개선 **미미·비일관**(CV +0.064 → temporal −0.093 역전, R0.95 −0.058).
- 과적합은 없으나 **이득 자체가 부재**.
- Appearance Ambiguity **ceiling 확인**(appe 기여 0, 상한 AUC 0.91).
- 미세 이득은 **milk 특화·운영점 의존** 위험.

→ **score fusion 방향 연구 종료.** Eq.(4)는 이미 이 feature들의 선형결합 최적이며, FP의 근본은 결합·보정이 아니라 **외관 feature 정보량의 한계**다.

---

## Recommended Next Story — 구조 변경 연구

score fusion이 천장에 막혔으므로 **feature/구조를 바꾸는** 방향으로 이동:
1. **Detect-then-segment 프론트엔드**: Grounded-SAM / YOLO-World로 open-vocab "milk carton" 검출 → 후보 자체의 의미 게이팅(현 ISM의 무차별 proposal 대체). FP의 근원(배경을 객체로 제안)을 앞단에서 차단.
2. **중간층 patch feature(FoundPose식)**: 마지막층 DINOv2 대신 중간층 패치 매칭 — 흰/저텍스처 변별력 보강(appearance ceiling 직접 공략). *(DINOv2 교체 금지 제약과 별개 연구로 분리)*
3. **SAM2 + temporal voting**: 영상 시퀀스 연속성으로 단발 FP 억제.
4. **(병행 저비용)** 3자 감사에서 도출된 DINOv2 백본 vits14→vitl14는 ceiling을 약간 올릴 여지(별도 검증).

→ 권장: **Grounded-SAM/YOLO-World detect-then-segment 타당성 Technical Research**부터.

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 보고서 표준 — 이 절로 마무리.

**판정: NO-GO.** score fusion으로는 Eq.4를 milk에서 능가 불가(천장=Appearance Ambiguity). **구조 변경 연구로 이동 권장.**

선택지:
- **(A) Detect-then-segment(Grounded-SAM/YOLO-World) Technical Research 시작** *(추천)* — FP 근원을 앞단 의미검출로 차단.
- **(B) 중간층 patch feature / SAM2+temporal 등 대안 구조 비교 연구** — 어느 구조부터 갈지 먼저 스코핑.
- **(C) score fusion 종료만 확정하고 일시 중단** — 다른 우선순위로.

→ **어느 쪽으로 진행할까요?** (추천: A)
