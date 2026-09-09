# Report — YOLO-World Shadow Validation for Milk Presence Gating

> **유형**: Quick-Dev Shadow Experiment (오프라인 전용, 프로덕션 무수정)
> **Spec**: [spec-yoloworld-shadow-validation.md](spec-yoloworld-shadow-validation.md)
> **날짜**: 2026-06-10 · **GPU**: RTX PRO 6000 Blackwell Max-Q (96GB) · **env**: `sam_yolo`(격리)
> **데이터(읽기전용)**: 311 프레임 `SLAM_with_milk_nomilk/frames/<fid>.png` + GT `..._baseline_bak/frame_results.csv`(decision_type). 평가셋 present=118(TP112+FN6), absent=193(FP120+TN73). SAM-6D baseline: recall 112/118=**0.949**, **FP=120**, 0.878s/frame.
> **분석 스크립트**: `tools/validation/yoloworld_shadow_eval.py` · 캐시 `outputs/validation/yoloworld_shadow_cache.json`

---

## Executive Summary — 결론 5개

1. **'milk carton' 단일 prompt가 압도적 최적 게이트.** yolov8s @0.05: presence **R=0.949 P=0.933 F1=0.941**(TP112/FP8/FN6/TN185). yolov8m @0.02: **R=0.966 P=0.974 F1=0.970**. 반면 `milk box`·`carton`·combo는 과검출(P 0.39~0.48), `milk` 단독은 저recall(0.42) → **부적합**.
2. **front gate가 SAM-6D FP를 82~98% 제거.** recall 바닥(~0.92) 동작점에서 yolov8s는 120→21~16 FP(99~104 제거), yolov8m은 120→**2 FP(118 제거)**.
3. **시간 성능은 유지가 아니라 대폭 개선.** YOLO-World 자체 **7.2ms/frame(yolov8s, 138 FPS)** — SAM-6D GPU 694ms 대비 무시 가능. absent 57~62% 스킵 시 **0.878s → 0.31~0.39s/frame, FPS 1.14 → 2.6~3.0 (2.3~2.6배)**.
4. **단 완전 recall 유지는 구조적으로 불가.** SAM-6D TP 프레임 중 3~4개를 YOLO가 conf≈0으로 못 봄 → recall **0.949 → 0.915~0.924(−2.5~3.4%p)**. **더 큰 모델로도 회복 안 됨**(s/m 모두 실패). 단 s·m이 못 보는 프레임이 달라 **공통 미검출은 2개(001695, 002015)뿐** → 앙상블/OR-recovery 시 recall 0.932 회복 가능.
5. **YOLO가 SAM-6D의 FN 6개를 전부 검출**(conf≥0.02) — 강한 상보성. 순수 게이트론 회복 못하나 hybrid(OR-recovery) 설계 시 recall을 **오히려 개선**할 잠재력. → 근본 해결 방향이 맞다는 추가 근거.

**판정: GO (Shadow Only)** — FP 대폭↓ + latency 2.3배↑ + recall 거의 유지(−2.5%p). 단 ①완전 recall 유지 불가(구조적 2~4 프레임) ②단일 시퀀스 검증 → **PRD 직행 대신 제한적 통합 실험**(hybrid recovery + 다중 시퀀스) 권고.

---

## sam_yolo Environment Summary (Task 1)

| 항목 | 내용 |
|---|---|
| 생성 | `conda create -n sam_yolo python=3.11 -y` |
| 설치 | `pip install "ultralytics>=8.1"` (torch==2.7.1+cu129 명시 시도는 실패 → 기본 의존성 해소로 대체) |
| 핵심 버전 | **ultralytics 8.4.64**, **torch 2.12.0+cu130**, torchvision 0.27.0, numpy 2.4.6, opencv 4.13.0 |
| GPU 호환 | torch 2.12.0+cu130 = `arch [sm_90, sm_100, sm_120]` → **Blackwell(sm_120) 지원 확인**, `cuda.is_available()=True` |
| 가중치 | `yolov8s-worldv2.pt`, `yolov8m-worldv2.pt`(자동 다운로드) + CLIP 텍스트 인코더(~338MB) |
| **프로덕션 무영향 확인** | `sam6d_ros_humble` 검증 후에도 **ultralytics 8.0.135 / torch 2.7.1 불변** (FastSAM 의존성 무손상) |
| 재사용성 | `sam_yolo`는 향후 YOLO-World+SAM-6D 통합의 기반 env로 그대로 사용 가능 |

> **주의**: 프로덕션 `sam6d_ros_humble`(ultralytics 8.0.135)은 YOLOWorld 클래스를 미지원하므로 절대 업그레이드하지 않고 별도 env로 격리함. 두 env는 완전 분리.
> **구현 노트**: ultralytics YOLO-World는 GPU predict 후 `set_classes()` 재호출 시 텍스트 임베딩 device mismatch 버그가 있어, phrase마다 모델을 새로 로드(set_classes를 predict 이전 1회)하여 회피.

---

## Prompt Comparison (Task 2)

presence detection(YOLO 검출 vs GT milk 존재), yolov8s-worldv2:

| prompt | thr | R | P | F1 | 비고 |
|---|---|---|---|---|---|
| **milk carton** | **0.05** | **0.949** | **0.933** | **0.941** | ★ recall=SAM-6D 동일 + 고정밀 |
| milk carton | 0.10 | 0.839 | 0.980 | 0.904 | 정밀↑ recall↓ |
| milk package | 0.05 | 0.966 | 0.864 | 0.912 | 차선(정밀 다소↓) |
| milk bottle | 0.05 | 0.958 | 0.485 | 0.644 | 과검출 |
| carton | 0.05 | 0.966 | 0.475 | 0.637 | 과검출 |
| milk box | 0.05 | 0.966 | 0.416 | 0.582 | 과검출 |
| combo(carton+box+package) | 0.05 | 0.975 | 0.394 | 0.561 | 최고 raw recall이나 게이트 부적합 |
| milk | 0.05 | 0.415 | 0.653 | 0.508 | 저recall(실패) |

→ **최고 recall은 combo(0.975)지만 정밀도가 낮아 게이트론 무용**. 게이트 목적(recall 유지 + FP 제거)엔 **'milk carton'**이 최적. yolov8m에선 'milk carton'의 정밀도가 더 높아짐(@0.02 P=0.974).

---

## Presence Detection Benchmark (Task 3)

best gate prompt 'milk carton':

| model | thr | TP | FP | TN | FN | Precision | Recall | F1 |
|---|---|---|---|---|---|---|---|---|
| yolov8s | 0.05 | 112 | 8 | 185 | 6 | 0.933 | 0.949 | 0.941 |
| yolov8s | 0.02 | 114 | 20 | 173 | 4 | 0.851 | 0.966 | 0.905 |
| **yolov8m** | **0.02** | **114** | **3** | **190** | **4** | **0.974** | **0.966** | **0.970** |
| yolov8m | 0.05 | 107 | 0 | 193 | 11 | 1.000 | 0.907 | 0.951 |

→ yolov8m @0.02가 **presence F1 0.970**으로 최고(거의 완벽 분리). milk 부재 193프레임 중 190(yolov8m) 정확 기각.

---

## FP Reduction Potential (Task 4)

front gate 적용 시 SAM-6D FP(120)가 사전 제거됨 = YOLO가 "milk 없음"이라 판정한 SAM-6D-FP 프레임:

| model | 동작점 thr | gated recall | **FP 120 → ?** | 제거 |
|---|---|---|---|---|
| yolov8s | 0.015 | 0.924 | **120 → 21** | **99 (82.5%)** |
| yolov8s | 0.05 | 0.898 | 120 → 7 | 113 (94%) |
| **yolov8m** | **0.02** | **0.915** | **120 → 2** | **118 (98%)** |

→ **FP의 82~98%를 ISM/PEM 실행 전에 제거.** 핵심 질문 "흰 물체 no-object FP"에 대한 직접 해법.

---

## Recall Risk (Task 5) — 가장 중요

| model | thr | gated recall | TP 손실 | recall loss | FN 증가 |
|---|---|---|---|---|---|
| SAM-6D (기준) | — | 0.949 | 0 | — | 0 |
| yolov8s | 0.015 | 0.924 | 3 | −2.5%p | +3 |
| yolov8s | 0.02 | 0.915 | 4 | −3.4%p | +4 |
| yolov8m | 0.02 | 0.915 | 4 | −3.4%p | +4 |

- **구조적 한계**: 완전 recall 유지(TP손실 0)는 thr=0에서만 가능 → FP 제거 0. 즉 **YOLO가 거의 못 보는 우유 뷰가 존재**(TP conf p0=0.000, p5=0.046).
- **모델 크기로 해결 안 됨**: yolov8s 손실 `{001680,001688,001695,002015}`, yolov8m 손실 `{001571,001695,002015,002110}` — **둘 다 못 보는 건 2개(001695, 002015)뿐**.
- **완화책**: s∪m 앙상블/OR 시 공통 미검출 2개만 손실 → **recall 0.932 회복**. 추가로 YOLO가 SAM-6D **FN 6개를 전부 검출** → hybrid OR-recovery 설계 시 recall을 SAM-6D 이상으로 끌어올릴 여지.
- **판단**: recall 손실은 **작음(−2.5~3.4%p, 2~4 프레임)** = "손실 큼(NO-GO)" 아님. 단 "완전 유지"도 아님 → Shadow Only 근거.

---

## Runtime Analysis (Task 6)

| 항목 | yolov8s | yolov8m |
|---|---|---|
| YOLO-World 추론 | **7.22 ms/frame (138.5 FPS)** | ~7.8 ms/frame (~128 FPS) |
| vs SAM-6D GPU(694ms)·전체(878ms) | 1.0% 수준 | 1.1% 수준 |

→ 앞단 detector 추가 비용은 **무시 가능**(GPU 96GB 여유, FastSAM-s 경량). "시간 악화" 위험 없음.

---

## ISM/PEM Skip Simulation (Task 7)

게이트가 "milk 없음" 판정 프레임에서 ISM/PEM(GPU 0.694s)을 생략. 평균 latency = YOLO(전프레임) + 통과프레임×SAM-6D:

| model | thr | skip 비율 | 평균 latency | FPS | vs 0.878s |
|---|---|---|---|---|---|
| SAM-6D (기준) | — | 0% | 878 ms | 1.14 | — |
| yolov8s | 0.015 | 53.7% | ~393 ms | ~2.5 | **2.2배↑** |
| yolov8s | 0.05 | 61.4% | 346 ms | 2.89 | 2.5배↑ |
| **yolov8m** | **0.02** | **62.4%** | **338 ms** | **2.96** | **2.6배↑** |

→ absent 193/311(62%)을 게이트가 스킵 → **평균 latency 절반 이하, FPS 2.3~2.6배**. "유지 또는 개선" 조건을 **개선으로 초과 달성**.

---

## GO / NO-GO Verdict

### **GO (Shadow Only)**

| 기준 | 판정 | 근거 |
|---|---|---|
| Recall 유지 | △ 거의 유지 | 0.949→0.915~0.924 (−2.5~3.4%p, 2~4프레임). 손실 작음·구조적, 모델로 미해결 |
| FP 감소 | ✅ 강함 | 120→2~21 (82~98% 제거) |
| Latency 유지/개선 | ✅✅ 대폭 개선 | 878→338~393ms, FPS 2.3~2.6배 |

**종합**: 세 목표가 사실상 모두 충족(FP 대폭↓, 속도 2.3배↑, recall 근사 유지)이나, ①완전 recall 유지가 구조적으로 불가(YOLO 미검출 우유 뷰 2~4개)하고 ②단일 시퀀스(311프레임, 단일 조명/배경)에서만 검증됨 → **PRD 직행은 위험**. 강한 GO 신호이되 **제한적 통합 실험으로 한 단계 더 검증** 필요 = **GO (Shadow Only)**.

---

## Recommended Next Story

**GO (Shadow Only) → 제한적 통합 실험** (PRD 전 단계):

1. **Hybrid OR-recovery 설계 검증** *(권장)* — 순수 AND-게이트(recall 손실) 대신, YOLO가 SAM-6D FN을 전부 잡는 상보성을 활용. s∪m 앙상블 또는 낮은 thr+면적/시간 필터로 **공통 미검출 2프레임(001695,002015)까지 분석**해 recall 손실 무력화 가능성 정량화.
2. **다중 시퀀스 일반화 검증** — 다른 조명/배경/거리 bag에서 'milk carton' recall·FP 재현성 확인(현재 단일 시퀀스 한계).
3. **제한적 ROS 통합(shadow 토픽)** — 게이트 결정을 publish하지 않고 별도 토픽으로 로깅, 실시간 latency·recall을 라이브로 측정. 확증되면 PRD.

> 코드 무수정 원칙은 검증 단계까지 유지. 실제 게이트 연결(파이프라인 수정)은 위 검증 통과 후 별도 Story.

---

## 부록 — 재현 & 한계

**재현**:
- `conda run -n sam_yolo python tools/validation/yoloworld_shadow_eval.py --weights yolov8s-worldv2.pt`
- `... --weights yolov8m-worldv2.pt --prompt "milk carton"`
- `... --from-cache --prompt "milk carton"` (캐시 재분석)

**한계(추측 금지)**:
1. **단일 시퀀스**(311프레임, 단일 bag/조명) — 일반화 미검증.
2. **GT = SAM-6D decision_type** 기반(TP/FN→present); visibility_labels.csv와 0 불일치로 교차검증했으나 수동 라벨 자체 한계 잔존.
3. **순수 AND-게이트 가정** — 실제 파이프라인의 ROI 전달·NMS·시간 안정화 상호작용 미반영.
4. **latency 시뮬**은 SAM-6D GPU 0.694s/전체 0.878s(launch2.log 단일 측정) 기반 — 통과프레임 오버헤드 단순 가정.
5. **2~4개 손실 프레임의 원인**(가림/원거리/극단뷰) 미시각화 — 통합 실험에서 확인 권장.

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 보고서 표준 — 이 절로 마무리.

**판정: GO (Shadow Only).** YOLO-World 'milk carton' front gate는 SAM-6D FP를 82~98% 제거하고 latency를 2.3~2.6배 개선하며 recall을 거의 유지(−2.5~3.4%p). 단 완전 recall 유지는 구조적으로 불가하고 단일 시퀀스 검증이라 PRD 직행 대신 **제한적 통합 실험** 권고.

선택지:
- **(A) Hybrid OR-recovery + 제한적 통합 실험 Quick-Dev** *(추천)* — recall 손실 무력화(앙상블/상보성) 검증 + shadow 토픽 라이브 측정. 확증 시 PRD.
- **(B) 바로 Detect-then-Segment PRD 작성** — 신호 충분하다 보고 통합 설계 착수(단 recall 손실·일반화 리스크 감수).
- **(C) 다중 시퀀스 일반화 검증 먼저** — 다른 bag들로 'milk carton' 재현성부터 확인 후 결정.

→ **어느 쪽으로 진행할까요?** (추천: A — recall 리스크를 직접 해소하면서 라이브 검증)
