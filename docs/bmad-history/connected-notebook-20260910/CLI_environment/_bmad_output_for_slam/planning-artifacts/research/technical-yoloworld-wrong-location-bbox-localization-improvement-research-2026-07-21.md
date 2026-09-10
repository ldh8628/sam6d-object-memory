# YOLO-World wrong-location BBox / localization 실패 개선 연구

- **작성일**: 2026-07-21
- **작성**: BMAD Technical Research (Ldh9501)
- **분석 폴더**: `sam6d_ws/_ism_research_2026_07/yolo_localization_research/` (1.1 GB)
- **운영 코드·config·threshold 변경**: 없음 · **git 조작**: 없음 · **신규 전수 BBox 라벨링**: 없음

---

## 1. Executive Summary

| 항목 | 결론 |
|---|---|
| **wrong-location 의 정체** | **증상이지 원인이 아니다.** 오답 후보의 YOLO conf 중앙값이 **0.087**(TP 는 0.455) — YOLO-World 는 이미 "확신 없음"을 표현하고 있고, 후단이 그것을 걸러낸다. 진짜 문제는 **correct-location proposal 부재** |
| **최대 유형** | L6 프롬프트가 잘못된 위치에만 반응 **34건(35.4%)** · L2 비슷한 형태의 다른 객체 **23건(24.0%)** · **L7 슬롯 경쟁은 3건(3.1%)뿐** |
| **슬롯 정책** | **기각.** L7 이 3건뿐이고 top_k 3→5 가 +4 였던 것과 일치 |
| **프롬프트 개선** | **부분 유효.** 확장 프롬프트 단독으로 미검출 복구율 0.175 → **0.338** (1.9배) |
| **해상도 증가** | **기각.** 1280/1920 에서 현행 위치 재현율이 오히려 **감소**(1.000→0.979→0.959) |
| **타일 검출** | 유효하나 **비싸다** — 복구 0.325, 박스/프레임 31.3, 82 s |
| **multi-label 단일 pass** | **라우팅 손실 0.175 확정.** any=1.000 인데 own=0.825 — Dinosaur 0.182 / Rabbit 0.500 붕괴 원인이 localization 이 아니라 **NMS 가 박스당 argmax 클래스 하나만 남기는 것**임을 실측 |
| **최우선 권장** | **H. 두 방법의 결합 — YOLOE 추가 + 클래스 라우팅 제거** |
| 근거 | `현행 + YOLOE` union: 복구율 own **0.362** / any **0.588** (현행 0.175 / 0.362) · 박스/프레임 21.7 · 추가 **5 초** |

**YOLOE 는 이미 설치된 ultralytics 8.4.64 안에 있다.** 신규 의존성이 0이고, 339 프레임 추론이
**5 초**로 현행 YOLO-World(38 초)의 **1/7.6** 이며, 박스는 오히려 **더 적게**(9.6 vs 12.1) 만든다.

---

## 2. ⚠ 방법론 정정 — 첫 평가 설계는 무효였다

처음 "정답 위치 proposal recall" 을 재려고 **provisional 박스 라벨 504건**을 위치 GT 로 썼다.
그 결과 현행(`cur`)의 recall 이 **정확히 1.0000** 이 나왔다.

**이것은 순환이다.** provisional 라벨은 **운영 후보 박스에 붙인 라벨**이므로, 그 위치는 정의상
현행 파이프라인이 만든 박스다. 현행이 자기 자신을 100% 맞추는 것은 당연하다.

> **이 프로젝트에는 현행과 독립적인 BBox 위치 GT 가 존재하지 않는다.**
> 모든 박스 라벨이 운영 후보에서 파생됐기 때문이다. 이것이 이번 연구의 근본 제약이다.

그래서 지표를 두 개로 분리했다. 둘 다 순환이 아니다.

| 지표 | 정의 | 무엇을 말하는가 |
|---|---|---|
| **재현율** | 현행이 찾은 위치를 다른 설정이 재현하는가 | 설정 교체 시 **잃는 것**. `cur`=1.0 은 제외하고 읽는다 |
| **라우팅 손실** | `own_prompt` 와 `any_prompt` 재현율의 격차 | 박스는 있는데 그 객체 후보로 안 가는 비율 |
| **미검출 복구율** | 직전 연구가 '정답 위치 후보 없음' 으로 판정한 사례에서, **인접 프레임(±5) 에 그 객체가 수락된 박스**를 기대 위치로 삼아 IoU≥0.3 인 박스를 만들었는가 | 설정 교체 시 **얻는 것**. 현행도 같은 기준으로 채점되므로 공정 |

미검출 복구율의 기대 위치는 카메라가 움직이므로 근사다. 그래서 복구된 사례 **254건**을
검수 큐(`recommended_cases.csv`)로 분리했다. 전수 BBox 라벨링은 하지 않았다.

---

## 3. 현재 YOLO-World proposal 구조 (코드·설정 확인)

```
입력 이미지(848×480 또는 640×480)
→ 프롬프트 10개를 각각 별도 forward (build_ism_inputs_imu.py:173-183)
→ predict(conf=min_score=0.02, imgsz=960)   ← iou/max_det/agnostic 은 미지정 = 기본값
→ NMS: iou=0.7, max_det=300, agnostic_nms=False, multi_label=False
→ 객체별 score_threshold (Bear .35 / Rabbit .30 / Dinosaur .30 / 나머지 .02)
→ top_k=3
→ DINOv2 CLS semantic selection
```

| 확인 항목 | 실제 값 |
|---|---|
| 프롬프트 10개 각각 별도 forward? | **예.** 고유 프롬프트 10개 → forward 10회 |
| 한 박스에 클래스 하나만? | **예.** `multi_label=False` 이므로 박스당 argmax 클래스 1개 |
| class-aware / agnostic NMS? | `agnostic_nms=False` 이나 **프롬프트당 클래스가 1개뿐이라 실질 class-agnostic** |
| 입력 해상도 / resize | imgsz **960**, ultralytics letterbox (종횡비 유지 + 패딩) |
| max_det / NMS IoU / conf | **300 / 0.7 / 0.02** |
| 프롬프트별 후보를 다른 객체가 재사용? | **불가.** `cand_of[bx][o["name"]]` 로 프롬프트→객체 고정 라우팅 |
| multi-label 출력 가능? | 가능하나 현행 경로에서 미사용 |
| objectness 분리? | **아니오.** YOLOv8-world 는 decoupled head 이나 별도 objectness 브랜치가 없다. conf = 텍스트-이미지 유사도 기반 클래스 점수 |
| 저장된 raw 후보 범위 | conf 0.005~0.97, 프레임당 12.1 박스(운영 조건) |

가중치는 `yolov8m-worldv2.pt` (57 MB).

---

## 4. Wrong-location 오류 유형 (L1~L10)

대상: 직전 연구의 F5 63건 + F12(unresolved) 33건 = **96건**.

> **부수 정정**: 직전 연구의 메모 파서가 "…잘못 선정되었지만, **실제 머그컵도 후보에 들어와있음**"
> 처럼 부정·긍정이 한 문장에 섞인 경우를 흘렸다. 절 단위로 쪼개 '실제/진짜' 가 든 절의
> 극성으로 판정하도록 고쳤고, **7건이 재판정**됐다(F1 4 · F5 2 · F12 1).
> `results/memo_parser_correction.csv`

| 유형 | 내용 | 건수 | 비율 |
|---|---|---:|---:|
| **L6** | 프롬프트가 잘못된 위치에만 반응 | **34** | **35.4%** |
| L10 | 판정 불가 | 30 | 31.2% |
| **L2** | 비슷한 형태의 다른 객체 | **23** | **24.0%** |
| L3 | 같은 범주 객체 혼동 | 6 | 6.2% |
| **L7** | **정답 후보도 있으나 오답이 슬롯 차지** | **3** | **3.1%** |
| L1/L4/L5/L8/L9 | — | 0 | 0% |

신뢰도: confirmed(사람 메모) 46 · probable 35 · unresolved 15.

**L7 이 3건뿐**인 것이 결정적이다. "정답 후보가 있는데 오답이 슬롯을 뺏는" 상황은 거의 없다.
따라서 **슬롯 정책 개선(top_k 증가, multi-class 유지)으로 얻을 것이 없다** — top_k 3→5 가
TP +4 에 그쳤던 직전 결과와 정확히 일치한다.

오답으로 지목된 객체: **Sikhye_high 11회** · carton 6 · Febreze 4 · saffron 3.

---

## 5. wrong-location 의 정체 — 증상이지 원인이 아니다

시각 자료(`visualizations/wrong_location_examples.jpg`)가 보여주는 것:

- `"orange bottle"`(Sauce) 프롬프트가 **금색 식혜 캔**과 **파란 페브리즈 분무기**에 반응 (conf 0.02~0.08)
- `"maroon box"`(choco) 프롬프트가 **거대한 갈색 택배박스 전체**에 반응 (conf 0.05)

**후보 최고 YOLO confidence 분포**

| 집단 | n | 최소 | p25 | **중앙** | p75 | 최대 |
|---|---:|---:|---:|---:|---:|---:|
| TP (정답 검출) | 590 | 0.020 | 0.156 | **0.455** | 0.800 | 0.972 |
| **F5 wrong-location** | 63 | 0.020 | 0.038 | **0.087** | 0.355 | 0.936 |
| F7/F9 게이트 탈락 | 66 | 0.035 | 0.208 | 0.485 | 0.743 | 0.893 |

| conf 임계 | TP 중 그 아래 | F5 중 그 아래 |
|---|---:|---:|
| < 0.05 | 3.7% | **33.3%** |
| < 0.10 | 16.1% | **52.4%** |
| < 0.20 | 33.2% | 65.1% |

**해석**: wrong-location 후보는 실제 객체가 프레임에 없거나 아주 작을 때 `conf=0.02` 하한을
간신히 넘는 **저신뢰 노이즈**다. YOLO-World 는 이미 "이건 orange bottle 이 아니다"를
낮은 confidence 로 말하고 있고, 후단(semantic/appearance/HSV)이 실제로 그것을 거절한다 —
그래서 **F5 는 FP 가 아니라 FN 으로 귀결된다.**

> **따라서 "wrong-location 을 줄이는 것"은 목표가 아니다.**
> 목표는 **correct-location proposal 을 만드는 것**이다. 이 재정의가 이번 연구의 핵심이다.

---

## 6. 객체별 localization 실패

| 객체 | 계 | L6 | L10 | L2 | L3 | L7 |
|---|---:|---:|---:|---:|---:|---:|
| **Sauce_high** | 28 | 11 | 2 | **9** | **5** | 1 |
| Dinosaur | 18 | 3 | **15** | 0 | 0 | 0 |
| choco_hazelnut_high | 12 | 5 | 1 | **6** | 0 | 0 |
| Bear | 10 | 4 | 6 | 0 | 0 | 0 |
| milk | 8 | 5 | 1 | 1 | 0 | 1 |
| Rabbit | 7 | 3 | 4 | 0 | 0 | 0 |
| Mugcup_high | 6 | 1 | 0 | 4 | 0 | 1 |
| Sikhye_high | 5 | 2 | 1 | 2 | 0 | 0 |
| Febreze_high | 2 | 0 | 0 | 1 | 1 | 0 |

- **choco ↔ 갈색 택배박스**: L2 6건. `"maroon box"` 가 택배박스 전체를 잡는다. 이미 `"brown box"` 에서
  `"maroon box"` 로 바꿔 완화를 시도한 이력이 config 주석에 남아 있다.
- **Sauce ↔ 식혜/Febreze**: L2 9 + L3 5 = 14건. `"orange bottle"` 이 금색 캔·파란 분무기에 반응.
- **Febreze ↔ saffron**: L2/L3 각 1건 — proposal 단계에서는 작은 문제(후단 FP 가 주 문제)
- **Bear/Rabbit/Dinosaur**: L2/L3 **0건**. 인형끼리는 proposal 이 아니라 **판별** 문제다.
- **작은 객체 프레임**: 96건 중 **가시 5~6개 프레임이 55.2%**, 3~4개가 39.6%.

---

## 7~9. 개선 후보 실측 결과

### 7.1 종합표 (339 GT 프레임, conf≥0.02)

| 설정 | 재현 own | 재현 any | 라우팅 손실 | 작은객체 | 신규위치 | 박스/f | 초 |
|---|---:|---:|---:|---:|---:|---:|---:|
| **cur** 현행 | 1.0000* | 1.0000* | 0.0000 | 1.0000* | 287 | 12.1 | 38 |
| cur_1280 | 0.9794 | 0.9794 | 0.0000 | 0.9688 | 597 | 13.5 | 29 |
| cur_1920 | 0.9588 | 0.9691 | 0.0103 | 0.9062 | 1051 | 14.5 | 55 |
| ml960 multi-label | 0.8247 | **1.0000** | **0.1753** | 0.9062 | 617 | 8.1 | 5 |
| ml960_agn | 0.7010 | 1.0000 | **0.2990** | 0.7188 | 581 | 6.4 | 3 |
| generic 8종 | — | 0.9794 | — | — | 2146 | 13.8 | 29 |
| ext 확장 프롬프트 | 0.9691 | 0.9897 | 0.0206 | 0.9375 | 1568 | 26.0 | 73 |
| tile2x2 | 0.9794 | 0.9897 | 0.0103 | 0.9688 | 2636 | 31.3 | 82 |
| **yoloe_txt** | 0.8351 | **1.0000** | 0.1649 | **0.9688** | 1460 | **9.6** | **5** |
| union_cur_ext | 1.0000 | 1.0000 | 0 | 1.0000 | 1855 | 38.1 | 111 |
| union_cur_tile | 1.0000 | 1.0000 | 0 | 1.0000 | 2923 | 43.4 | 120 |
| **union_cur_yoloe** | 1.0000 | 1.0000 | 0 | 1.0000 | 1747 | **21.7** | **43** |

\* 순환(§2). 비교에서 제외.

### 7.2 미검출 복구율 — **가장 중요한 지표**

인접 프레임 기대 위치 기준, IoU≥0.3, 대상 **80건**.

| 설정 | own 복구 | own율 | any 복구 | any율 |
|---|---:|---:|---:|---:|
| **union_cur_yoloe** | **29** | **0.362** | **47** | **0.588** |
| union_cur_ext | 28 | 0.350 | 43 | 0.537 |
| ext | 27 | 0.338 | 42 | 0.525 |
| **yoloe_txt** | 27 | **0.338** | 45 | 0.562 |
| tile2x2 | 26 | 0.325 | 45 | 0.562 |
| union_cur_tile | 26 | 0.325 | 46 | 0.575 |
| cur_1920 | 24 | 0.300 | 34 | 0.425 |
| cur_1280 | 22 | 0.275 | 35 | 0.438 |
| ml960 | 17 | 0.212 | 41 | 0.512 |
| **cur (현행)** | **14** | **0.175** | 29 | 0.362 |
| ml960_agn | 14 | 0.175 | 40 | 0.500 |
| generic | 0 | 0.000 | 39 | 0.487 |

**두 개의 독립된 손실이 동시에 보인다.**

```
현행 own 0.175  →  현행 any 0.362      격차 0.187 = 라우팅 손실
현행 any 0.362  →  최고 any 0.588      격차 0.226 = localization 한계
```

즉 **박스가 있어도 절반은 그 객체 후보로 가지 못하고**, 박스 자체가 없는 경우도 40% 남는다.
한 방법으로 둘 다 해결할 수 없다 — 이것이 결합(H)을 권하는 근거다.

### 7.3 후보별 판정

**후보 A — 프롬프트 개선: 부분 유효, 단 수동 튜닝 필요**
확장 프롬프트(객체당 +2, 제품명·외형 포함)로 복구율 0.175 → **0.338**. 다만
현행 위치 재현율은 0.9691 로 **약간 손실**되고(choco 0.800, Sauce 0.833),
프롬프트를 객체마다 손으로 만들어야 한다. 일반화 규칙은 발견하지 못했다.
비용도 크다(73 s, 26 박스/프레임).

**후보 B — prompt ensemble union: 유효, 비쌈**
`union_cur_ext` 복구 own 0.350 / any 0.537. 재현율 손실 없음(union 이므로).
그러나 **박스/프레임 38.1(현행의 3.2배)**, 시간 111 s. 후단 DINOv2/MobileSAM 부하가 3배가 된다.

**후보 C — generic / class-agnostic shared proposal: 부분 유효**
generic 프롬프트 8종(`object, box, bottle, can, toy, package, container, doll`)만으로
현행 위치의 **97.9%(any)** 를 커버하고 미검출 복구 any 0.487.
그러나 클래스 라벨이 없어 **ISM 이 전부 판정**해야 하며, `union_cur_generic` 의 복구 own 은
0.175 로 현행과 같다(라우팅이 없으니 own 개념이 성립하지 않는다).
→ **라우팅 제거의 근거는 되지만, 단독으로는 localization 을 개선하지 않는다.**

**후보 D — multi-label 단일 pass: Rabbit 감소 원인 규명 완료, 채택 불가**
`ml960` 은 any=1.000 인데 own=0.8247 → **라우팅 손실 0.1753**.
객체별: **Dinosaur 0.182 · Rabbit 0.500 · Sikhye 0.909 · saffron 0.955**.
즉 이전에 관찰된 "Rabbit 수락 −15" 의 원인은 **localization 도 슬롯 정책도 아니고,
NMS 가 박스당 argmax 클래스 하나만 남겨 인형 3종이 서로를 지우는 것**이다.
`agnostic_nms=True` 로 바꾸면 손실이 **0.2990 으로 악화**한다(박스가 더 합쳐지므로).
→ **multi-label 단일 pass 는 그 자체로 라우팅 문제를 만든다. 채택 불가.**

**후보 E — 해상도/타일: 해상도 기각, 타일은 비싸다**
imgsz 1280/1920 에서 현행 위치 재현율이 **1.000 → 0.979 → 0.959 로 감소**하고
작은 객체 재현율도 1.000 → 0.969 → 0.906 으로 **떨어진다**.
복구율은 0.275/0.300 으로 오르지만 재현 손실과 상쇄된다. **단순 업스케일은 기각.**
2×2 타일은 복구 0.325 로 의미 있으나 **82 s + 31.3 박스/프레임** 으로 가장 비싸다.

**후보 F — temporal 보완: 보류**
단순 BBox 복사는 인접 프레임 IoU 중앙 0.452(직전 실측)라 성립하지 않는다.
optical flow·tracker 는 **이번에 측정하지 않았다** — 구현은 가능하나 효과는
**현재 산출물로는 확인할 수 없음**. `results/temporal_depth_feasibility.csv`

**후보 H — depth 보완: 원리적으로 가능, 미측정**
bag 에 **`/camera/camera/aligned_depth_to_color/image_raw` 가 실재**함을 확인했다.
BBox 가 독립 객체를 가리키는지 검증하는 데는 쓸 수 있어 보이나, 책상 위 밀집 객체가
하나로 뭉칠 위험과 작은 객체·반사·depth hole 문제가 있다. **미측정.**

**후보 G — 다른 detector: YOLOE 를 실측했고 결과가 좋다**

| detector | 설치 | 크기 | 339프레임 추론 | 박스/f | 복구 own | 복구 any | 근거 |
|---|---|---|---:|---:|---:|---:|---|
| YOLO-World v2-m (현행) | 설치됨 | 57 MB | 38 s | 12.1 | 0.175 | 0.362 | **실측** |
| **YOLOE-11l-seg (text)** | **같은 ultralytics 8.4.64** | 71 MB (+MobileCLIP 572 MB 최초 1회) | **5 s** | **9.6** | **0.338** | **0.562** | **실측** |
| Grounding DINO | 별도 설치 | ~700 MB | 미측정 | — | — | — | 조사만 |
| OWLv2 | transformers | ~600 MB | 미측정 | — | — | — | 조사만 |
| SAM auto mask | 설치됨 | 41/375 MB | 미측정 | — | — | — | 조사만 |

**YOLOE 는 신규 의존성이 0이다.** `from ultralytics import YOLOE` 가 이미 동작하고,
추론이 현행의 **1/7.6** 이며 박스는 **더 적게** 만들면서 복구율은 **1.9배**다.
공개 벤치마크에서도 YOLOE26-S 가 LVIS 에서 YOLO-World-S 대비 +11.4 AP 로 보고된다.

---

## 10. 후보 비교표

| 후보 | 해결하는 오류 | 복구 own | 복구 any | 재현 손실 | 박스/f | 추가 시간 | 수동 튜닝 | 판정 |
|---|---|---:|---:|---:|---:|---:|---|---|
| A 프롬프트 개선 | localization 일부 | 0.338 | 0.525 | −0.031 | 26.0 | +35 s | **객체별 필요** | 조건부 |
| B prompt ensemble union | localization | 0.350 | 0.537 | 0 | 38.1 | +73 s | 객체별 필요 | 비쌈 |
| C generic shared | 라우팅 | 0.175 | 0.487 | 0 | 25.9 | +29 s | 불필요 | 보조 |
| D multi-label 단일 pass | — | 0.212 | 0.512 | **−0.175** | 8.1 | **−33 s** | 불필요 | **기각** |
| E-1 해상도 1280/1920 | — | 0.275/0.300 | 0.438/0.425 | **−0.021/−0.041** | 13.5/14.5 | −9/+17 s | 불필요 | **기각** |
| E-2 2×2 타일 | 작은 객체 | 0.325 | 0.562 | −0.021 | 31.3 | +44 s | 불필요 | 비쌈 |
| F temporal | — | 미측정 | 미측정 | — | — | — | — | **보류** |
| H depth | BBox 검증 | 미측정 | 미측정 | — | — | — | — | **보류** |
| **G YOLOE 단독** | localization | **0.338** | **0.562** | −0.165 | **9.6** | **−33 s** | 불필요 | 유력 |
| **H′ 현행+YOLOE union** | localization | **0.362** | **0.588** | **0** | 21.7 | **+5 s** | 불필요 | **최우선** |

---

## 11. 최우선 권장안 — H. YOLOE 추가 + 클래스 라우팅 제거

두 방법이 **서로 다른 오류**를 해결한다는 지시 조건을 만족한다.

```
① YOLOE 를 현행 YOLO-World 와 병렬로 추가 (union)
   → localization 해결.  복구 any 0.362 → 0.588
② 프롬프트→객체 고정 라우팅 제거 (모든 박스를 모든 객체의 후보로)
   → 라우팅 손실 해결.  복구 own 을 any 에 근접시킴 (0.175 → 0.588 이 상한)
```

**왜 이 조합인가**

1. **정답 위치 proposal recall 증가** — 복구율 own 0.175 → 0.362 (2.07배), 상한 0.588 (3.4배)
2. **wrong-location 감소** — 목표가 아니다(§5). 다만 YOLOE 는 박스를 **더 적게** 만든다(9.6 vs 12.1)
3. **작은 객체** — YOLOE 작은객체 재현 0.9688 로 타일과 동급, 해상도 증가보다 낫다
4. **후단 호환** — BBox 만 늘어나므로 HSV·DINOv2 게이트는 그대로 작동
5. **FP 증가** — 후보가 1.8배 늘지만 후단 게이트가 이미 FP 를 89건까지 줄여 놓았다
6. **수동 튜닝 불필요** — 프롬프트는 현행 그대로 재사용
7. **운영 코드 변경 범위** — `YOLOWorld` → `YOLOE` 인스턴스 추가 + 후보 병합 + 라우팅 제거
8. **처리시간** — **+5 초 / 339 프레임** (프레임당 +15 ms)

**권장 데이터 흐름**

```
입력 → [YOLO-World 프롬프트 10 pass + YOLOE text pass] → 박스 union → IoU 중복 제거
     → 클래스 라우팅 없이 모든 박스를 후보로 → DINOv2 semantic 선택 → MobileSAM
     → appearance 게이트 → HSV 게이트 → (ambiguity 시 block2+block11 patch 검증) → 수락
```

---

## 12. 단계별 구현 및 검증 계획

| 단계 | 내용 | 검증 지표 | 중단 기준 |
|---|---|---|---|
| **S1** | 검수 큐 254건 중 YOLOE 복구분 표본 검토 | 복구 박스가 실제 객체 위인가 | 정답률 < 50% 면 S2 중단 |
| **S2** | YOLOE union 을 오프라인 ISM 에 연결해 TP/FP/FN 측정 | 사람 GT 935 기준 | FP 가 89 → 150 초과 시 중단 |
| **S3** | 라우팅 제거(모든 박스 = 모든 객체 후보) 효과 측정 | own 이 any 에 얼마나 접근하는가 | FP 급증 시 top-N class 유지로 후퇴 |
| **S4** | latency·GPU 메모리 실측 후 운영 명세 작성 | 프레임당 시간 | 실시간 요구 초과 시 YOLOE 단독으로 전환 |

**예상 개선 (상한, 검수 전)**

```
정답 위치 proposal recall : 0.175 → 0.362 (union), 라우팅 제거 시 상한 0.588
wrong-location            : 감소가 목표 아님. 박스 수는 12.1 → 21.7
TP / FP / FN              : 미측정 — S2 에서 측정한다. 현 단계에서 추정치를 제시하지 않는다
latency                   : +5 s / 339 프레임 = 프레임당 +15 ms
```

---

## 13. 위험 요소와 rollback 기준

| 위험 | 근거 | rollback 기준 |
|---|---|---|
| 후보 1.8배 증가로 후단 부하 상승 | 12.1 → 21.7 박스/프레임 | 프레임당 처리시간이 현행 대비 +50% 초과 |
| 라우팅 제거로 FP 증가 | 모든 박스가 모든 객체 후보가 됨 | 사람 GT 기준 FP 89 → 150 초과 |
| YOLOE 재현 손실 | own 재현 0.8351 (단독 사용 시) | **union 으로 회피** — 단독 전환 금지 |
| MobileCLIP 572 MB 최초 다운로드 | 실측(11분 41초) | 오프라인 환경이면 사전 배포 필요 |
| 기대 위치 근사 오차 | 인접 프레임 ±5, IoU≥0.3 | 검수 254건으로 확인 |
| 조명·장면 일반화 | 6 데이터가 같은 날 같은 사무실 | 다른 세션 수집 전까지 운영 반영 보류 |

---

## 14. 산출물

```
yolo_localization_research/
  probes/  dump_proposals.py  dump_yoloe.py  wrong_location_taxonomy.py
           eval_proposal_recall.py(결함, 보존)  eval_honest.py  eval_recovery.py
           make_vis.py  make_feasibility.py
  candidate_dumps/  cur cur_1280 cur_1920 ml960 ml960_agn generic ext tile2x2 yoloe_txt
  results/
    wrong_location_taxonomy.csv(96)     current_proposal_recall.csv
    prompt_experiment.csv               prompt_union_experiment.csv
    shared_proposal_experiment.csv      multi_label_slot_experiment.csv
    resolution_tile_experiment.csv      temporal_depth_feasibility.csv(6)
    alternative_detector_comparison.csv(5)  method_comparison.csv(13)
    recommended_cases.csv(254)          recovery_experiment.csv
    routing_loss.csv  novel_location.csv  memo_parser_correction.csv(7)
  visualizations/  wrong_location_examples.jpg  recovered_by_ext_tile.jpg
```

---

## 15. 한계

1. **현행과 독립된 BBox 위치 GT 가 없다.** 첫 평가가 순환이었던 근본 원인이며(§2),
   "정답 위치 recall" 의 절대값은 이 프로젝트에서 측정 불가다.
2. **미검출 복구율의 기대 위치는 인접 프레임(±5) 근사**다. 카메라가 움직이므로 오차가 있고,
   기대 위치를 세울 수 있는 사례가 F2~F6 중 **80건**뿐이다.
3. **YOLOE 를 ISM 후단에 연결해 TP/FP/FN 을 측정하지 않았다.** 이번 범위는 proposal 까지다.
4. **temporal(F)·depth(H) 는 미측정**이다. 판정을 "보류" 로 남겼고 추정치를 내지 않았다.
5. Grounding DINO·OWLv2·SAM auto-mask 는 **설치·실행하지 않았다.** 조사 결과만 기록했다.
6. L10(판정 불가) 30건이 남아 있고 Dinosaur 15건이 집중돼 있다.

---

## 16. 결론

```text
현재 wrong-location 의 가장 큰 원인:
wrong-location 은 원인이 아니라 증상이다. 오답 후보의 YOLO confidence 중앙값이 0.087 로
TP(0.455)의 1/5 이며, conf<0.10 인 비율이 TP 16.1% 대 wrong-location 52.4% 다.
즉 YOLO-World 는 이미 낮은 confidence 로 '확신 없음' 을 표현하고 있고 후단이 그것을 거절한다.
실제 원인은 그 프레임에서 correct-location proposal 이 아예 만들어지지 않는 것이며,
유형으로는 L6(프롬프트가 잘못된 위치에만 반응) 34건 35.4% 와
L2(비슷한 형태의 다른 객체) 23건 24.0% 가 지배한다.

프롬프트 자체의 문제 비율:
L6 34건(35.4%) + L2/L3 29건(30.2%) = 65.6% 가 프롬프트-대상 대응이 어긋난 경우다.
다만 확장 프롬프트로 복구되는 것은 미검출 80건 중 27건(0.338)에 그치고, 현행 위치 재현율은
0.9691 로 오히려 손실이 생기며, 객체마다 손으로 만들어야 한다. 프롬프트 튜닝만으로는
문제의 절반도 해결하지 못하므로 단독 해법으로는 기각한다.

작은 객체 문제 비율:
wrong-location 96건 중 프레임당 가시 객체가 5~6개인 경우가 55.2%, 3~4개가 39.6% 로
작은 객체 상황에 집중돼 있다. 그러나 해상도 증가는 해법이 아니다 — imgsz 1280/1920 에서
작은 객체 재현율이 1.000 → 0.969 → 0.906 으로 오히려 떨어졌다. 타일 검출(0.9688)과
YOLOE(0.9688)만이 작은 객체 재현율을 지키면서 복구율을 올렸다.

라우팅 및 슬롯 정책 문제 비율:
슬롯 정책은 사실상 문제가 아니다 — L7(정답 후보가 있는데 오답이 슬롯 차지)이 96건 중 3건(3.1%)
뿐이고, top_k 3→5 가 TP +4 에 그쳤던 직전 결과와 일치한다.
반면 라우팅은 큰 문제다. 현행의 미검출 복구율이 own 0.175 인데 any 0.362 로,
격차 0.187 이 '박스는 있는데 그 객체 후보로 가지 못한' 비율이다.
multi-label 단일 pass 는 이 문제를 더 키운다(라우팅 손실 0.1753, agnostic NMS 시 0.2990).

Prompt ensemble 판정:
조건부 유효하나 비싸다. union_cur_ext 는 복구 own 0.350 / any 0.537 로 현행(0.175/0.362)보다
분명히 낫고 재현 손실도 없다. 그러나 박스/프레임이 12.1 → 38.1 로 3.2배가 되고 시간이 +73초이며,
프롬프트를 객체마다 수동으로 만들어야 하고 일반화 규칙을 찾지 못했다.
YOLOE union 이 더 적은 비용(21.7 박스, +5초)으로 더 나은 복구율(0.362/0.588)을 내므로 차선이다.

Class-agnostic shared proposal 판정:
보조 수단으로만 유효하다. generic 프롬프트 8종만으로 현행 위치의 97.9%(any)를 커버하고
미검출 복구 any 0.487 을 낸다. 즉 '위치를 찾는 능력' 은 클래스 프롬프트 없이도 상당하다.
그러나 클래스 라벨이 없어 ISM 이 전부 판정해야 하고, union_cur_generic 의 복구 own 은 0.175 로
현행과 같다. 라우팅 제거의 정당화 근거는 되지만 localization 자체를 개선하지는 않는다.

Multi-label proposal 판정:
기각. ml960 은 any=1.000 인데 own=0.8247 로 라우팅 손실이 0.1753 이고,
객체별로 Dinosaur 0.182 · Rabbit 0.500 으로 붕괴한다. 이전에 관찰된 'Rabbit 수락 -15' 의 원인이
localization 도 슬롯 정책도 아니고 NMS 가 박스당 argmax 클래스 하나만 남겨 인형 3종이 서로를
지우는 것임이 실측으로 확정됐다. agnostic NMS 로 바꾸면 0.2990 으로 더 악화한다.

해상도·tile 검출 판정:
해상도 증가는 기각한다. imgsz 1280/1920 에서 현행 위치 재현율이 1.000 → 0.979 → 0.959,
작은 객체 재현율이 1.000 → 0.969 → 0.906 으로 모두 떨어졌다. 단순 업스케일이 정보량을
늘리지 못한다는 예상이 실측으로 확인됐다.
2×2 타일은 유효하다(복구 0.325, 작은 객체 0.9688). 그러나 82초 + 31.3 박스/프레임으로
모든 후보 중 가장 비싸며, YOLOE union 이 절반 비용으로 더 나은 결과를 낸다.

Temporal 보완 판정:
보류. 단순 BBox 복사는 인접 프레임 IoU 중앙 0.452 라 성립하지 않는다.
optical flow·tracker·Kalman 은 구현 가능하나 이번에 측정하지 않았으므로 효과는
현재 산출물로는 확인할 수 없다. 추정치를 제시하지 않는다.

Depth 보완 판정:
보류하되 원리적으로 가능하다. bag 에 /camera/camera/aligned_depth_to_color/image_raw 가
실재함을 확인했다. BBox 가 독립 객체를 가리키는지 검증하는 용도로는 유망하나,
책상 위 밀집 객체가 하나로 뭉칠 위험과 작은 객체·반사·depth hole 문제가 있고 미측정이다.

다른 detector 필요 여부:
필요하다. 단 새 대형 모델을 들이는 것이 아니다. YOLOE 는 **이미 설치된 ultralytics 8.4.64 안에
포함**돼 있어 신규 의존성이 0이다. 실측 결과 339 프레임 추론이 5초로 현행 YOLO-World(38초)의
1/7.6 이고, 박스는 오히려 적게(9.6 vs 12.1) 만들면서 미검출 복구율은 0.338 로 현행 0.175 의
1.9배다. Grounding DINO·OWLv2·SAM auto-mask 는 조사만 했고 설치·실행하지 않았다.

최우선 권장안:
H. 두 방법의 단계적 결합 — YOLOE 를 현행 YOLO-World 와 병렬 추가(union)해 localization 을
해결하고, 프롬프트→객체 고정 라우팅을 제거해 라우팅 손실을 해결한다.
두 방법이 서로 다른 오류를 해결한다: union 은 '박스가 없는' 문제를, 라우팅 제거는
'박스는 있는데 그 객체 후보로 안 가는' 문제를 다룬다.
union_cur_yoloe 의 실측은 복구 own 0.362 / any 0.588 로 모든 설정 중 최고이며,
박스/프레임 21.7 로 타일(43.4)의 절반, 추가 시간 +5초로 prompt ensemble(+73초)의 1/15 이다.

권장 데이터 흐름:
입력 → [YOLO-World 10 pass + YOLOE 1 pass] → 박스 union → IoU 중복 제거 → 클래스 라우팅 없이
전 박스를 후보로 → DINOv2 semantic 선택 → MobileSAM → appearance → HSV → (모호 시 patch 검증) → 수락

예상 개선:
- 정답 위치 proposal recall: 미검출 복구율 own 0.175 → 0.362 (2.07배), 라우팅 제거 시 상한 0.588
- wrong-location: 감소가 목표가 아니다(§5). 박스/프레임은 12.1 → 21.7 로 증가한다
- TP: 미측정 — proposal 단계까지가 이번 범위다. S2 에서 측정한다
- FP: 미측정 — 후보 증가로 늘 수 있으나 후단 게이트가 이미 89건까지 줄여 두었다
- FN: 미측정 — 복구율 상한이 0.588 이므로 미검출 80건 중 최대 47건이 후보를 얻는다
- latency: +5 초 / 339 프레임 = 프레임당 +15 ms (실측)

첫 번째 구현 실험:
검수 큐 recommended_cases.csv 254건 중 YOLOE 가 복구한 사례를 표본 검토해
'복구된 박스가 실제 객체 위인가' 를 확인한다. 정답률이 50% 미만이면 인접 프레임 기대 위치가
부정확하다는 뜻이므로 다음 단계를 진행하지 않는다. 통과하면 YOLOE union 을 오프라인 ISM 에
연결해 사람 GT 935 기준 TP/FP/FN 을 측정한다.

운영 코드 변경:
없음

다음 권장 BMAD 명령:
YOLOE union + 라우팅 제거의 오프라인 구현 실험. 운영 코드를 수정하지 않고 복사본에서
후보 생성부를 교체해 사람 GT 935 기준 TP/FP/FN 변화를 측정하고, 후단 부하(프레임당 박스 수,
DINOv2/MobileSAM 시간)와 FP 증가폭을 함께 낸다. rollback 기준(FP 150 초과, 처리시간 +50% 초과)을
사전에 고정한 상태로 진행한다.
```

---

## 다음 액션 제안

1. **검수 큐 254건 표본 검토 (권장, 우선)** — YOLOE·확장 프롬프트가 "복구했다"는 박스가 실제로 그 객체 위인지 확인합니다. 인접 프레임 기대 위치가 근사이므로 이 확인 없이는 복구율 0.362 를 신뢰할 수 없습니다.
2. **YOLOE union 오프라인 구현 실험** — 후단까지 연결해 TP/FP/FN 을 실제로 측정합니다.
3. **라우팅 제거 단독 실험** — 후보 증가 없이 라우팅만 없애면 얼마나 얻는지(상한 0.362 → 0.588 중 라우팅 몫) 분리 측정합니다.
4. **temporal/depth 실측** — 이번에 "보류"로 남긴 두 후보를 실제로 재봅니다.

**질문**: 1번(검수)부터 하시겠습니까? 복구율이 이번 결론의 핵심 근거인데 기대 위치가 근사라, 표본 30건 정도만 확인해도 결론의 신뢰도가 크게 달라집니다. 아니면 2번(구현 실험)으로 바로 가시겠습니까?

**Sources**
- [YOLOE: Real-Time Seeing Anything | Ultralytics Docs](https://docs.ultralytics.com/models/yoloe)
- [YOLOE Tutorial: Real-Time Open-Vocabulary Object Detection](https://learnopencv.com/yoloe-tutorial-real-time-open-vocabulary-detection/)
- [What is YOLOE? | Ultralytics](https://www.ultralytics.com/blog/what-is-yoloe-taking-computer-vision-models-further)
- [YOLO-World Model | Ultralytics Docs](https://docs.ultralytics.com/models/yolo-world)
- [Best Object Detection Models 2026: RF-DETR, YOLOv12 & Beyond](https://blog.roboflow.com/best-object-detection-models/)
