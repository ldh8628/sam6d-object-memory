# SAM-6D ISM: semantic + appearance 에 HSV 색 특징을 더했을 때의 FP/FN 오프라인 검증

- **작성일**: 2026-07-21
- **작성**: BMAD Technical Research (Ldh9501)
- **분석 폴더**: `sam6d_ws/_ism_research_2026_07/ism_color_validation/`
- **운영 코드·config·threshold 변경**: 없음
- **git 조작**: 없음
- **신규 대규모 라벨링**: 없음 (최소 검수 큐 130건만 생성)

---

## 0. Executive Summary

| 항목 | 결론 |
|---|---|
| **최종 판정** | **A. HSV hard gate 채택** (조건부 — 최소 검수 130건 후 확정) |
| 가장 좋은 reference | **H2 = Hue 보정 렌더 템플릿 42장** (H+S 히스토그램, 저채도 예외 없음) |
| 가장 좋은 결합 | **hard gate** — 점수로 더하는 방식(C/D/E)은 특정 객체를 심각히 훼손 |
| 효과 | **FP 228 → 89 (−61%)**, FN 334 → 345 (+11, +3.3%), Precision 0.725 → **0.869** |
| 오류 교환비 | 수정 **139건** : 새로 깨짐 **11건** = **12.6 : 1** |
| PLY 필요 여부 | **불필요** — H5(렌더+PLY)는 H2와 사실상 동일(FP 88 vs 89) |
| 실사 prototype | **불필요** — 렌더 템플릿만으로 달성 |

**가장 중요한 발견**: **HSV 를 점수로 더하면 오히려 해롭고, 게이트로 쓰면 안전하다.**
초코하임과 갈색 택배박스는 **색이 같기 때문**에, 색 점수를 더하면 오답을 밀어 올린다
(choco FP 33 → 100~143). 게이트는 제거만 하므로 이 위험이 원리적으로 없다.

---

## 1. 평가에 사용한 기존 GT와 provenance

새로 만든 라벨은 없다. 아래를 **읽기 전용**으로 재사용했다.

| 자산 | 경로 | 성격 | 이번 용도 |
|---|---|---|---|
| 사람 가시성 GT 339 프레임 | `gt_input/user_visibility_gt.csv` | **사람 GT** | TP/FP/FN 판정 |
| 사람 triage 334건 | `gt_input/triage_answers.csv` | **사람 GT** | "검출 가능" 분모 보정 |
| 백업 | `~/gt_backup/` | **사람 GT** | 무결성 대조 |
| 박스 라벨 504건 | `ism_accuracy_observation/labels/box_labels_merged.csv` | **provisional (Claude Vision)** | reference AUROC 비교 전용 |
| 검수 대기 79건 | `…/labels/human_review_queue.csv` | provisional 저신뢰 | 검수 큐 P6 |
| 후보 BBox·mask·sem·appe | `ism_accuracy_observation/frame_dumps/*_pairs.csv` | 운영 덤프 | **고정** |
| 박스 HSV 히스토그램 | `…/hsv_features/<ds>_hsv.npz` | 운영 덤프 | 질의 특징 |
| 렌더 템플릿 42장 | `sam6d_ws/template/<obj>/templates/` | 운영 자산 | H1/H2 reference |
| PLY vertex color | `ism_fusion_research/ply_hsv/prototype_colors.npz` | 파생 자산 | H3/H4/H5 reference |
| Hue 보정 실험 | `ism_fusion_research/results/hsv_global_calibration.csv` | 이전 연구 | 보정값(−6°, ×1.3) 근거 |
| 혼동 사례 메모 115건 | `gt_input/triage_answers.csv` (note 열) | **사람 GT** | 해석 |

> **provisional 과 사람 GT 구분**: 결정 단위 TP/FP/FN 은 전부 **사람 GT** 기반이다.
> **provisional 라벨은 §4 의 reference AUROC 비교에만** 쓰였고, 표에 명시했다.

### 고정한 것 (색 효과만 분리)

지시대로 아래는 전부 운영 덤프 값을 **그대로** 사용하고 재계산하지 않았다.

```
YOLO raw prediction · confidence/NMS 결과 · 후보 BBox · top_k=3
MobileSAM mask · semantic(sem_top5) · appearance(appe11_clstop1)
```

직전 연구에서 만든 "가드 제거 재덤프"는 **후보 집합이 달라지므로 이번 비교에서 제외**했다.
같은 후보에 대해 HSV 사용 전후의 판정만 비교한다.

### 범위에서 제외한 것

- 실제 객체 위치에 BBox 가 아예 없는 FN **109건** — HSV 로는 **원리적으로 복구 불가**.
  모든 표에서 `FN_no_candidate` 로 분리 표기했고, 전 방식에서 **109 로 고정**이다.
- YOLO prompt 개선 · BBox localization · proposal recall 재측정 — 별도 후속 연구.

---

## 2. 평가 설계

```
TP = 프레임에 보이는 객체를 수락
FP = 보이지 않는 객체를 수락
FN = 보이는 객체를 미수락
LODO 6-fold — threshold·가중치는 held-out 을 제외한 5개에서만 보정
```

**한계 2가지를 먼저 명시한다.**

1. 프레임 단위 GT 라 **수락된 박스가 실제로 그 객체 위에 있었는지 확인하지 않는다.**
   → 이것이 §7 최소 검수 큐의 존재 이유다.
2. 가시성 GT 작성자 진술상 기준이 관대하므로 **recall 은 하한, FP 는 상한**이다.
   triage 로 보정한 `recall_detectable` 을 함께 보고한다.

---

## 3. HSV 는 어떤 색 정보인가 (용어)

```
Hue        색의 종류 (빨강·노랑·초록·파랑). OpenCV 는 0~179 로 저장 — 실제 각도의 절반
Saturation 색이 선명한 정도. 0 이면 회색
Value      밝기
```

조명이 바뀌면 Value 가 가장 흔들리고 Hue 가 가장 덜 흔들린다. 그래서 **H+S** 를 기본으로 쓴다.
이전 연구에서 렌더러가 실사 대비 Hue 를 오른쪽으로 밀고 채도를 26~38% 낮추는
**계통 편향**이 확인됐고(LODO 6/6 fold 에서 동일 선택), 보정은
`(H, S, V) → ((H − 6°) mod 360°, S × 1.3, V)` 이다. Hue 는 색상환이므로 순환 이동이다.

---

## 4. HSV reference 비교 (H0~H5)

실사 crop HSV 는 **평가 기준으로만** 사용했고 reference 로는 쓰지 않았다.

### 4.1 순위 지표 (provisional 박스라벨 기준, LODO)

| reference | 특징공간 | 저채도 처리 | 평균 AUROC | 최저 객체 AUROC | 평균 PR-AUC |
|---|---|---|---:|---:|---:|
| **H5** 렌더+PLY | hs | svfall | **0.9673** | 0.9222 (Sauce) | 0.8095 |
| **H2** Hue보정 렌더 | hs | svfall | **0.9661** | 0.8944 (milk) | 0.8042 |
| H5 | hs | none | 0.9616 | 0.9222 | 0.7829 |
| H2 | hs | none | 0.9590 | 0.8944 | 0.7686 |
| H2 | hsv | svfall | 0.9268 | 0.7866 (choco) | 0.6393 |
| H5 | h (색상만) | none | 0.8197 | **0.2566 (Rabbit)** | 0.5841 |
| H5 | sv | none | 0.6956 | 0.3026 (choco) | 0.3269 |

- **특징공간은 `hs`(H16×S8)가 압도적**이다. `h` 단독은 Rabbit 이 0.257 로 붕괴하고,
  `sv` 단독은 전반적으로 열등하다.
- **H1(보정 없는 렌더)은 상위권에 없다** — Hue 보정이 실질적 기여를 한다.
- H3(PLY 전체 1개)·H4(PLY 시점별 42개)는 단독으로는 렌더보다 나쁘다.

### 4.2 저채도 처리 — 두 가지 정정

저채도 객체는 **Rabbit 하나**뿐이다(평균 채도 2.4/255, 다른 객체는 36~148).

| 처리 | 정의 | 결과 |
|---|---|---|
| `none` | 예외 없음 | 기준 |
| `skip` | 저채도 객체는 Hue 보정 미적용 | AUROC +0.002 (거의 무효) |
| `satw` | 채도에 비례해 색 점수를 중립(0.5)으로 당김 | **AUROC 변화 0** |
| `svfall` | 저채도 객체는 H 대신 S/V 특징 사용 | AUROC +0.007 |

> **정정 1** — `satw` 는 **객체별 AUROC 를 정의상 바꿀 수 없다.** 객체 안에서 단조 아핀
> 변환이라 순위가 보존되기 때문이다. 전역 threshold 를 쓸 때만 의미가 있어 결정 단위에서
> 다시 확인했고, 거기서도 변화가 없었다(§5).
>
> **정정 2 — 순위 지표와 결정 지표가 역전한다.** `svfall` 은 AUROC 를 +0.007 올리지만
> 게이트에서는 **FP 89 → 95 로 나빠진다**. `hs` 와 `sv` 는 점수 스케일이 달라서
> **전역 threshold 하나로 두 공간을 재면 손해**이기 때문이다.
> → **AUROC 만 보고 채택했다면 잘못된 선택을 했을 것이다.**

### 4.3 reference 결정 (결정 단위, hard gate 기준)

| reference / 저채도 | TP | FP | FN |
|---|---:|---:|---:|
| **H2 / none** | **590** | **89** | **345** |
| H2 / skip | 590 | 89 | 345 |
| H2 / satw | 590 | 89 | 345 |
| H2 / svfall | 588 | 95 | 347 |
| H5 / none | 588 | 88 | 347 |
| H5 / svfall | 587 | 94 | 348 |

**채택: H2 (Hue 보정 렌더 42장) + H+S + 저채도 예외 없음.**
H5 는 FP 가 1건 적을 뿐인데 **PLY 파생 자산을 추가로 관리**해야 하므로 이득이 비용을 넘지 않는다.

`results/hsv_reference_comparison.csv`

---

## 5. 결합 방식 A~G 비교 (LODO held-out 합산)

사람 GT 935 가시 / 2455 비가시. 후보·mask·sem·appe 고정.

| | 방식 | TP | FP | FN | (후보無 FN) | Precision | Recall | F1 | Recall(검출가능) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| **A** | baseline (sem+appe) | 601 | 228 | 334 | 109 | 0.7250 | 0.6428 | 0.6814 | 0.7356 |
| **B** | **HSV hard gate** | 590 | **89** | 345 | 109 | **0.8689** | 0.6310 | 0.7311 | 0.7222 |
| C | 단순 평균 (1/3씩) | 666 | 236 | 269 | 109 | 0.7384 | 0.7123 | 0.7251 | 0.7947 |
| D | 공통 가중 평균 | 719 | 274 | 216 | 109 | 0.7241 | 0.7690 | 0.7459 | 0.8499 |
| E | rank fusion | 652 | 175 | 283 | 109 | 0.7884 | 0.6973 | 0.7401 | 0.7799 |
| F | uncertainty-only | 592 | 95 | 343 | 109 | 0.8617 | 0.6332 | 0.7300 | 0.7246 |
| G | negative rejection | 593 | 113 | 342 | 109 | 0.8399 | 0.6342 | 0.7227 | 0.7258 |

`후보無 FN` 이 전 방식에서 **109 로 고정**인 점을 확인하라 — HSV 는 없는 BBox 를 만들 수 없다.

### 5.1 객체별 TP/FP/FN — C·D·E 가 탈락하는 이유

| 객체 | A (baseline) | **B (hard gate)** | C (평균) | D (가중) | E (rank) |
|---|---|---|---|---|---|
| Bear | 51/40/35 | 49/**4**/37 | 70/26/16 | 70/8/16 | 67/8/19 |
| Rabbit | 25/26/69 | 25/**0**/69 | 28/14/66 | 33/1/61 | 28/0/66 |
| Dinosaur | 41/7/47 | **35**/0/**53** | 40/3/48 | 64/1/24 | 35/1/53 |
| milk | 60/18/30 | 60/**2**/30 | 64/1/26 | 69/4/21 | 65/0/25 |
| **choco_hazelnut_high** | 54/**33**/31 | 54/**30**/31 | 71/**109**/14 | 74/**143**/11 | 67/**100**/18 |
| Febreze_high | 65/58/14 | 65/**29**/14 | 74/55/5 | 76/57/3 | 73/40/6 |
| Mugcup_high | 45/7/26 | **42**/6/**29** | 43/7/28 | 47/7/24 | 43/6/28 |
| saffron | 110/30/12 | 110/**12**/12 | 120/18/2 | 122/46/0 | 118/17/4 |
| Sauce_high | 61/6/52 | 61/4/52 | 65/2/48 | 70/6/43 | 65/2/48 |
| Sikhye_high | 89/3/18 | 89/2/18 | 91/1/16 | 94/1/13 | 91/1/16 |

**C·D·E 는 choco_hazelnut_high 의 FP 를 33 → 100~143 으로 3~4배 늘린다.**
판정 기준의 *"주요 객체 중 한 객체의 성능을 심각하게 훼손하지 않음"* 에 정면으로 걸린다.

**원인은 명확하다.** 초코하임(마룬색 과자 상자)과 갈색 택배박스는 **색이 같다.**
색을 *더하는* 방식은 갈색 택배박스의 최종 점수를 함께 올려 게이트를 통과시킨다.
반대로 게이트는 **제거만 하므로** 이 위험이 원리적으로 없다 — 색이 같으면 그냥
게이트를 통과할 뿐, 오답이 새로 만들어지지 않는다.

D 가 고른 가중치도 경고 신호다: 6 fold 중 5 fold 에서 **`w_sem = 0.0`** 을 선택했다.
F1 을 최대화하려고 semantic 을 통째로 버린 것으로, 운영에 넣을 수 없는 해다.

### 5.2 F(uncertainty-only)와 G(negative rejection)

- **F**: 확신 마진 `m` 이 6/6 fold 에서 **0.25**(탐색 범위 최대값)로 고정 선택됐다.
  즉 "애매할 때만"이 실질적으로 "거의 항상"이 되어 **B 로 수렴**한다.
  결과도 B 와 사실상 동일(TP +2, FP +6). **파라미터만 하나 늘고 이득은 없다.**
- **G**: TP 손실 ≤1% 제약을 걸어 보수적으로 튜닝한 게이트. TP 를 8건 더 지키는 대신
  FP 를 24건 더 남긴다(113 vs 89). **B 의 보수적 변종**이며 별도 방식이 아니다.

### 5.3 파라미터 안정성 (공통 규칙인지 확인)

```
B  t_hsv = [0.1370, 0.1357, 0.1179, 0.1227, 0.1198, 0.1239]   ← 6 fold 편차 ±0.01
G  t_hsv = [0.0776, 0.0694, 0.0679, 0.0721, 0.1198, 0.1107]
E  t     = [0.685, 0.663, 0.667, 0.640, 0.685, 0.662]
D  w     = [(0,.4,.6), (.1,.3,.6), (0,.4,.6), (0,.4,.6), (0,.4,.6), (0,.3,.7)]  ← w_sem≈0
```

**B 의 임계는 6 fold 에서 0.118~0.137 로 매우 안정적**이다. 객체별 규칙이 아니라
**전 객체 공통 스칼라 하나**로 동작한다.

### 5.4 데이터셋별 일관성

| 데이터셋 | A: TP/FP/FN | B: TP/FP/FN | ΔFP | ΔFN |
|---|---|---|---:|---:|
| sam_105018 | 80/26/42 | 79/11/43 | **−15** | +1 |
| sam_105314 | 132/38/71 | 131/9/72 | **−29** | +1 |
| sam_105652 | 119/54/33 | 118/12/34 | **−42** | +1 |
| sam_110104 | 118/56/70 | 118/29/70 | **−27** | 0 |
| sam_110532 | 79/23/65 | 75/12/69 | **−11** | +4 |
| sam_110633 | 73/31/53 | 69/16/57 | **−15** | +4 |

**6개 데이터 전부에서 FP 감소, FN 증가는 0~4건.** 방향이 완전히 일관된다.

`results/baseline_vs_hsv_{all,by_object,by_dataset}.csv`, `results/fold_params.csv`

---

## 6. 오류 변화 상세

### 6.1 수정 vs 파손

```
판정이 바뀐 사례 150건
  수정된 오류 (fixed)   139  ← 전부 FP → TN
  새로 깨진 것 (broken)  11  ← 전부 TP → FN
  교환비 12.6 : 1
```

| 객체 | 수정 | 파손 |
|---|---:|---:|
| Bear | **36** | 2 |
| Febreze_high | **29** | 0 |
| Rabbit | **26** | 0 |
| saffron | 18 | 0 |
| milk | 16 | 0 |
| Dinosaur | 7 | **6** |
| choco_hazelnut_high | 3 | 0 |
| Sauce_high | 2 | 0 |
| Mugcup_high | 1 | **3** |
| Sikhye_high | 1 | 0 |

**파손 11건은 Dinosaur 6 · Mugcup 3 · Bear 2 에 집중**된다. 이것이 §7 검수 큐의 우선순위 1이다.

### 6.2 혼동쌍 변화 (FP 기준)

| 수락된 객체 | 실제로 보인 것 | A | B | Δ |
|---|---|---:|---:|---:|
| **Febreze_high** | **saffron** | 47 | 18 | **−29** |
| **Bear** | **Dinosaur** | 28 | 3 | **−25** |
| Febreze_high | Sikhye_high | 33 | 14 | −19 |
| Febreze_high | Dinosaur | 24 | 5 | −19 |
| saffron | Febreze_high | 19 | 2 | **−17** |
| Rabbit | Dinosaur | 17 | 0 | **−17** |
| Bear | saffron | 20 | 3 | −17 |
| Bear | Sauce_high | 18 | 3 | −15 |
| Febreze_high | Bear | 19 | 4 | −15 |
| Rabbit | Bear | 14 | 0 | **−14** |
| Bear | Rabbit | 15 | 4 | −11 |
| Dinosaur | Rabbit | 5 | 0 | −5 |
| **choco_hazelnut_high** | **Bear** | 18 | **17** | **−1** |
| **choco_hazelnut_high** | **milk** | 16 | **13** | **−3** |
| Sauce_high | Sikhye_high | 2 | 2 | 0 |
| Sikhye_high | Sauce_high | 1 | 1 | 0 |

**해결된 것**: 최대 FP 원인이던 **saffron ↔ Febreze**(47+19=66 → 18+2=20, −70%),
**인형 3종 상호 혼동**(Bear↔Dinosaur↔Rabbit 79 → 7, −91%).

**해결되지 않은 것**: **초코하임 ↔ 갈색 물체**(Bear 18→17, milk 16→13).
색이 같아서 색으로는 못 가른다. 이것은 색 특징의 **원리적 한계**이며,
§5.1 에서 본 "점수로 더하면 오히려 악화"와 같은 뿌리다.

**새로 나타난 혼동**: 없음. 모든 쌍에서 FP 가 감소하거나 동일하다.

`results/changed_decision_cases.csv`, `fixed_errors.csv`, `newly_broken_cases.csv`,
`confusion_pair_changes.csv`

### 6.3 객체별 AUROC / PR-AUC (provisional 박스라벨 — **사람 GT 아님**)

| 객체 | pos | neg | sem | appe | **hsv** | 세 점수 평균 |
|---|---:|---:|---:|---:|---:|---:|
| Bear | 28 | 19 | 0.691 | 0.579 | **1.000** | 1.000 |
| Rabbit | 21 | 8 | 0.827 | 0.548 | **1.000** | 0.988 |
| Dinosaur | 25 | 5 | 0.544 | 0.472 | **1.000** | 0.936 |
| milk | 23 | 77 | 0.971 | 0.974 | 0.924 | 0.994 |
| choco_hazelnut_high | 18 | 61 | 0.936 | 0.916 | 0.940 | 0.973 |
| Febreze_high | 22 | 85 | 0.970 | 0.923 | **0.999** | 1.000 |
| Mugcup_high | 19 | 21 | 1.000 | 1.000 | 1.000 | 1.000 |
| saffron | 51 | 86 | 0.921 | 0.997 | 0.973 | 0.988 |
| Sauce_high | 40 | 73 | 0.998 | 0.999 | 0.983 | 1.000 |
| Sikhye_high | 47 | 41 | 0.998 | 0.997 | 0.985 | 1.000 |

**인형 3종에서 sem·appe 는 무작위에 가깝고(0.47~0.83) HSV 가 1.000 이다.**
색과 형태가 **완전히 상보적**이라는 직접 증거다.

> 표본이 작다(Dinosaur pos 25 / neg 5). AUROC 1.000 을 과신하면 안 된다.
> 또한 이 표만 **provisional 라벨** 기반이며, §5·§6.1·§6.2 는 전부 사람 GT 기반이다.

---

## 7. 최소 사람 검수 큐 (130건)

### 7.1 왜 필요한가

프레임 단위 GT 로는 **수락된 박스가 그 객체 위에 있었는지** 알 수 없다.
"HSV 가 FP 139건을 제거했다"를 확정하려면 그 박스 안에 무엇이 있었는지만 알면 된다.

**BBox 를 새로 그리게 하지 않는다.** 박스 하나당 선택지 하나다.

```
객체 이름 10개 중 하나 / hard_negative / wrong_location / unsure
```

`hard_negative`(다른 물건)와 `wrong_location`(배경만)의 구분이 핵심이다 —
색 게이트는 전자를 거를 수 있지만 후자를 거를 이유가 없기 때문이다.

### 7.2 규모와 선정

| 항목 | 값 |
|---|---:|
| **검수 대상** | **130 박스** |
| 전체 후보(22,436) 대비 | **0.6%** |
| 우선순위 1 (필수) | **11건** |
| 예상 소요 | 전체 **20~30분** (건당 10~15초 **가정**, 측정값 아님) |

| 선정 이유 | 건수 | 근거 |
|---|---:|---|
| `P1_newly_broken` | **11** | HSV 가 정답을 거절 — 채택 여부를 가르는 유일한 비용 |
| `P2_fixed_error` | 31 | 수정된 오류의 객체별 층화 표본 |
| `P3_mode_disagree` | 30 | B/E/G 결론 불일치 |
| `P4_near_threshold` | 29 | HSV 점수가 게이트 임계(≈0.123) 근처 |
| `P5_key_confusion` | 20 | saffron↔Febreze, 인형 3종, choco↔갈색 |
| `P6_lowconf_label` | 18 | 기존 provisional 저신뢰 라벨 |

중복 제거 후 130건. 한 박스가 여러 이유로 뽑히면 `+` 로 병기했다.

### 7.3 산출물

```
ism_color_validation/human_review/
  color_review_queue.csv     130행 — 채울 칸은 review_label, reviewed 둘뿐
  USER_REVIEW_GUIDE.md       선택지 정의·우선순위·편향 주의
  crops/C0001.png … C0130.png  대상 박스를 주황 상자로 표시한 프레임
  contact_sheets/            17장 (9장씩, 420px)
  serve_review.py            숫자키 선택 뷰어 (표준 라이브러리만, 즉시 저장)
```

CSV 컬럼: `case_id, priority, dataset_name, frame_id, target_object, uid, bbox, image_path,
baseline_result, hsv_result, hsv_score, reasons, provisional_label, gt_visible_in_frame,
review_label, reviewed, note`

> 편향 방지: `baseline_result`·`hsv_result`·`provisional_label` 은 뷰어에서 **기본 숨김**이다.

---

## 8. 핵심 질문 7개에 대한 답

**1. semantic+appearance 보다 HSV 를 추가하면 FP 가 감소하는가?**
**예. FP 228 → 89 (−61%).** 6개 데이터 전부에서 감소하며 감소폭은 −11 ~ −42.
Precision 0.725 → 0.869.

**2. HSV 로 기존 TP 가 손상되거나 FN 이 증가하는가?**
**소폭 증가한다.** TP 601 → 590 (−11), FN 334 → 345 (+11, +3.3%).
파손 11건 전부가 TP→FN 이고, 수정 139건과의 비는 **12.6 : 1**.
단, 파손 11건이 정말 손해인지는 §7 검수 전까지 **확정되지 않았다.**

**3. HSV 는 어떤 객체에서 돕고 어떤 객체에서 악화되는가?**

| | 객체 | 근거 |
|---|---|---|
| **크게 도움** | Bear(FP 40→4), Rabbit(26→0), Febreze(58→29), saffron(30→12), milk(18→2) | 색이 뚜렷이 다름 |
| **거의 무효** | choco_hazelnut_high(33→30) | 갈색 택배박스와 **색이 같음** |
| **악화** | Dinosaur(TP 41→35), Mugcup(45→42) | 검수 큐 P1 대상 |

**4. 모든 객체에 적용 가능한 공통 결합 방식이 존재하는가?**
**예 — 단 게이트에 한해서.** 전 객체 공통 스칼라 임계 하나(`t_hsv ≈ 0.12`)로 동작하고
6 fold 편차가 ±0.01 이다. 반면 **공통 가중 평균(D)은 실패**한다 —
choco FP 가 4배로 늘고, 학습된 가중치가 `w_sem = 0` 이라 운영 불가다.

**5. hard gate 와 weighted score 중 어느 쪽이 나은가?**
**hard gate 가 명확히 낫다.** 이유는 성능 수치가 아니라 **구조**다.
weighted score 는 오답의 점수를 **올릴 수 있어** 새 FP 를 만든다(choco 33→109).
hard gate 는 **제거만 하므로** 새 FP 를 만들 수 없다.
색이 같은 hard negative 가 존재하는 한 이 비대칭은 데이터가 바뀌어도 유지된다.

**6. 불확실한 후보에서만 HSV 를 쓰는 방식이 더 안전한가?**
**아니다 — 더 안전하지도, 더 좋지도 않다.** 마진 파라미터가 6/6 fold 에서 탐색 범위
최대값(0.25)으로 선택되어 사실상 항상 HSV 를 적용하게 되고, 결과도 B 와 동일하다
(TP +2, FP +6). **파라미터만 하나 늘어난다.**

**7. 실사 prototype 없이 렌더 또는 PLY 색만으로 운영 가능한가?**
**예. 렌더 템플릿만으로 충분하다.** H2(Hue 보정 렌더) 로 FP −61% 를 달성했고,
PLY 를 더한 H5 는 FP 가 1건 적을 뿐이라 추가 자산을 관리할 이유가 없다.
실사 crop 은 평가에만 썼고 운영 자산으로 쓰지 않았다.

---

## 9. 판정 기준 대조

| 조건 | B (hard gate) | 판정 |
|---|---|:---:|
| 기존 대비 FP 감소 | 228 → 89 (−61%) | ✅ |
| FN 증가 없음 또는 충분히 작음 | +11 (+3.3%) | ⚠️ 작지만 존재 |
| 한 객체를 심각히 훼손하지 않음 | Dinosaur TP −6(−15%), Mugcup −3(−7%) | ⚠️ 경미 |
| 6개 데이터 방향 일관 | 6/6 에서 FP 감소 | ✅ |
| 신규 실사 prototype 불필요 | 렌더 템플릿만 사용 | ✅ |
| 공통 결합 규칙 | 전 객체 공통 스칼라 1개 | ✅ |
| 기존 후보·mask 만으로 계산 | 재추론 없음 | ✅ |

⚠️ 두 항목이 **검수 11건으로 확정 가능**하다. Dinosaur 6건이 정말 Dinosaur 였다면 실제 손해이고,
`hard_negative`/`wrong_location` 이었다면 오히려 **추가 이득**이다.

---

## 10. 산출 파일

```
ism_color_validation/
  probes/
    build_hsv_refs.py        H0~H5 × 4 특징공간 × 4 저채도처리
    eval_fusion_modes.py     A~G LODO (REF/LOWSAT 환경변수로 변형)
    analyze_changes.py       판정 변화·혼동쌍·객체별 AUROC
    build_review_queue.py    최소 검수 큐
  results/
    baseline_vs_hsv_all.csv        ★ A~G 전체 지표
    baseline_vs_hsv_by_object.csv  ★ 객체별 TP/FP/FN
    baseline_vs_hsv_by_dataset.csv ★ 데이터셋별
    hsv_reference_comparison.csv   ★ H1~H5 × 공간 × 저채도 (36행)
    changed_decision_cases.csv     ★ 판정 변화 150건
    fixed_errors.csv               ★ 수정 139건
    newly_broken_cases.csv         ★ 파손 11건
    confusion_pair_changes.csv     ★ 혼동쌍 변화
    score_auroc_by_object.csv      sem/appe/hsv/fused AUROC (provisional)
    fold_params.csv                fold별 선택 파라미터
    hsv_reference_best.json        채도 통계·최적 설정
  human_review/
    color_review_queue.csv  USER_REVIEW_GUIDE.md  serve_review.py
    crops/(130)  contact_sheets/(17)
```

---

## 11. 한계

1. **프레임 단위 GT** — 수락 박스의 위치 정확성 미확인. → 검수 큐 130건이 이를 겨냥.
2. **가시 기준이 관대** — recall 은 하한, FP 는 상한(양쪽 모두 모델을 나쁘게 보이게 함).
3. **조명 일반화 미검증** — 6개 데이터가 같은 날 같은 사무실. HSV 는 조명 의존적이고,
   Hue 보정 스칼라 2개(−6°, ×1.3)는 **이 조명에 적합된 값**이다.
4. **§6.3 AUROC 표만 provisional 라벨** 기반이며 표본이 작다(Dinosaur neg 5).
5. **후보 없음 FN 109건은 색으로 복구 불가** — 별도 연구.
6. 운영 코드·config·threshold 무변경. **오프라인 검증만.**

---

## 12. 결론

```text
평가에 사용한 기존 GT:
사람이 직접 작성한 가시성 GT 339 프레임(가시 935 frame×object)과 FN 334건 triage 결과를
판정 기준으로 사용했다. 후보 BBox·MobileSAM mask·semantic·appearance 는 운영 덤프
(ism_accuracy_observation)를 그대로 고정해 재계산하지 않았고, 렌더 템플릿 42장과
PLY vertex color, 이전 Hue 보정 실험 결과를 reference 로 재사용했다.
provisional 박스라벨 504건은 사람 GT 와 구분해 reference AUROC 비교(§6.3)에만 사용했다.

추가로 수행한 사람 라벨링:
없음 — 최소 검수 큐만 생성함.

가장 좋은 HSV reference:
H2 = Hue 보정 렌더 템플릿 42장 (H+S 16x8 히스토그램, Bhattacharyya, 42개 중 max,
공통 보정 Hue -6도 + 채도 x1.3, 저채도 예외 없음).
H5(렌더+PLY)가 FP 1건 적지만 PLY 파생 자산을 추가 관리해야 하므로 채택하지 않는다.
주의: 순위 지표(AUROC)는 svfall 저채도 처리를 선호했으나 결정 단위에서는 FP 89->95 로
역전했다 — 특징 공간이 섞이면 전역 threshold 하나로 잴 수 없기 때문이며,
AUROC 만 보고 채택했다면 잘못된 선택이 됐을 것이다.

가장 좋은 결합 방식:
HSV hard gate. 전 객체 공통 스칼라 임계 하나(t_hsv 약 0.12, 6 fold 편차 ±0.01)로 동작한다.
점수로 더하는 방식(단순 평균·공통 가중 평균·rank fusion)은 전부 탈락한다 —
초코하임과 갈색 택배박스는 색이 같아서 색 점수가 오답을 밀어올리고,
choco FP 가 33 -> 100~143 으로 3~4배 늘기 때문이다. 게이트는 제거만 하므로
새 FP 를 만들 수 없다는 구조적 비대칭이 결정적이다.
uncertainty-only 는 마진이 6/6 fold 에서 탐색 최대값으로 선택되어 사실상 hard gate 로
수렴하며 이득 없이 파라미터만 늘고, negative rejection 은 hard gate 의 보수적 변종이다.

Baseline:
- TP: 601
- FP: 228
- FN: 334  (이 중 후보가 아예 없는 구조적 FN 109 포함)
- Precision: 0.7250
- Recall: 0.6428
- F1: 0.6814

HSV 적용 결과:
- TP: 590
- FP: 89
- FN: 345  (이 중 후보가 아예 없는 구조적 FN 109 포함 — baseline 과 동일)
- Precision: 0.8689
- Recall: 0.6310
- F1: 0.7311

수정된 기존 오류:
- 139건, 전부 FP -> TN (FN 이 복구된 것은 0건 — 게이트는 제거만 하므로 당연하다)
- 객체별: Bear 36 · Febreze 29 · Rabbit 26 · saffron 18 · milk 16 · Dinosaur 7 ·
  choco 3 · Sauce 2 · Mugcup 1 · Sikhye 1
- 혼동쌍: saffron<->Febreze 66 -> 20 (-70%), 인형 3종 상호혼동 79 -> 7 (-91%)

새로 발생한 오류:
- 11건, 전부 TP -> FN (새로 생긴 FP 는 0건)
- 객체별: Dinosaur 6 · Mugcup 3 · Bear 2
- 새로 나타난 혼동쌍 없음 — 모든 쌍에서 FP 가 감소하거나 동일하다
- 수정 대 파손 = 12.6 : 1

객체별 심각한 악화 여부:
심각한 악화는 없으나 두 객체에 경미한 손실이 있다. Dinosaur TP 41 -> 35 (-15%),
Mugcup_high TP 45 -> 42 (-7%). 나머지 8개 객체는 TP 유지 또는 -2 이내이며 FP 는 전부 감소한다.
초코하임은 개선도 악화도 거의 없다(FP 33 -> 30) — 갈색 택배박스와 색이 같아
색 특징으로는 원리적으로 구분할 수 없기 때문이며, 이는 별도 수단이 필요한 영역이다.

최소 사람 검수 필요 건수:
130건 (전체 후보 22,436 의 0.6%). 그중 우선순위 1 은 11건이며 이것만으로도
채택 여부의 핵심 판단이 가능하다. 예상 소요는 전체 20~30분이다(건당 10~15초 가정,
측정값이 아님). BBox 를 새로 그리지 않고 박스당 선택지 하나만 고른다.

최종 판정:
A (HSV hard gate 채택)

운영 적용 권장 여부:
조건부 권장. 오프라인 근거는 충분하다 — FP 61% 감소, 6개 데이터 방향 일관,
공통 스칼라 하나로 적용 가능, 신규 실사 prototype 불필요, 기존 후보·mask 만으로 계산 가능.
다만 지금 바로 운영에 넣는 것은 권하지 않는다. 두 가지가 선결되어야 한다.
(1) 검수 11건 — HSV 가 거절한 것이 정말 정답이었는지. hard_negative 나 wrong_location
    으로 밝혀지면 손실이 아니라 추가 이득이 되어 판정이 더 강해진다.
(2) 조명이 다른 세션 최소 1개 — 6개 데이터가 같은 날 같은 사무실이고 Hue 보정 스칼라
    2개가 이 조명에 적합된 값이라, 조명 일반화는 현재 어떤 방법으로도 검증되지 않았다.

다음 단계:
최소 검수 후 확정.
```

---

## 다음 액션 제안

운영 코드·config·threshold 는 전혀 변경하지 않았습니다. 다음 중 선택해 주십시오.

1. **검수 11건만 먼저 (권장, 약 3분)** — `human_review/serve_review.py` 실행 후
   우선순위 1 필터. HSV 가 거절한 11건 안에 무엇이 있었는지만 확인하면 판정이 확정됩니다.
   `hard_negative`/`wrong_location` 으로 나오면 손실이 아니라 **이득**이 되어 근거가 더 강해집니다.
2. **130건 전체 검수 (약 20~30분)** — 결론의 견고성까지 확인.
3. **조명이 다른 세션 수집** — 한계 3을 닫는 유일한 방법. 색 게이트를 운영에 넣기 전
   가장 보수적인 선택입니다.
4. **운영 적용 명세 작성** — 검수 없이 진행할 경우. `yolo_ism_object_n.py` 에 세 번째 게이트를
   추가하는 변경 명세와 롤백 절차를 문서로 만듭니다(코드는 수정하지 않음).

**질문**: 1번(11건, 3분)부터 하시겠습니까? 그 결과에 따라 판정이 "조건부 권장"에서
"권장" 또는 "보류"로 바뀝니다. 뷰어는 이미 띄워 두었습니다 — `http://127.0.0.1:8765`
(안 되면 `http://10.254.176.41:8765`).
