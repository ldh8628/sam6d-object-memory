# SAM-6D ISM 후속 검증: Semantic aggregation 원인 · `*_high.ply` 템플릿 HSV · 최소 수동 GT 설계

- **작성일**: 2026-07-21
- **작성**: BMAD Technical Research (ldh)
- **격리 폴더**: `sam6d_ws/_ism_research_2026_07/ism_followup_research/`
- **운영 코드 변경**: 없음
- **git 조작**: 없음

> **폴더 위치 안내 (지시와 다른 점 1건)**
> 지시된 경로는 `sam6d_ws/ism_followup_research/` 였으나, 직전 요청("연구 폴더를 하나로 묶어
> 나중에 삭제하기 편하게")에 따라 `sam6d_ws/_ism_research_2026_07/ism_followup_research/` 에
> 생성했습니다. 하위 폴더 구성(`semantic_analysis/ high_ply_extraction/ high_ply_templates/
> hsv_analysis/ gt_seed_design/ results/ reports/`)은 지시대로입니다.
> `rm -rf sam6d_ws/_ism_research_2026_07` 만으로 원상복구됩니다.

> **대상 파일명 정정 (지시와 다른 점 2건)**
> 지시된 `sam6d_ws/data/ply_files_excluding_20260602.tar.gz` 는 존재하지 않습니다.
> 실제 파일은 **`ply_files_excluding_20260702.tar.gz`** (0602 → **0702**, 195.5 MB, 31개 PLY)
> 이며 이것으로 진행했습니다. 원본 압축 파일은 삭제하지 않았습니다.

---

## 1. Executive Summary

세 가지 검증 모두 **당초 가설과 다른 결론**이 나왔고, 그 과정에서 예상하지 못한 실행 가능한
발견이 두 건 나왔습니다.

| # | 검증 항목 | 결과 |
|---|---|---|
| 1 | `sem_top5` vs `sem_mean` 원인 | **기전 확정.** 두 점수의 우열은 오직 "꼬리(6~42위 view)가 상위 5개보다 변별력이 높은가" 로 결정됩니다 — 상관계수 **+1.000 (n=10)**. peakiness(뾰족함) 가설은 기각(−0.251) |
| 2 | `*_high.ply` 재렌더 | **개선 없음.** 10개 객체 중 **4개는 이미 `*_high.ply` 로 렌더 중**(색 100% 일치)이고, 실제로 재렌더 가능한 4개의 AUROC 개선량은 **−0.001 ~ +0.007**. Bear는 정점 4배 증가에도 이미지 차이 **0.234/255** |
| 3 | 최소 수동 GT | **방안 D 기각.** "불확실 프레임만 검수"는 2,596 중 **1,894(73%)** 로 전수와 다름없습니다. 권장은 **A-25 균등 + 불확실 200장** = 301 프레임(11.6%) |

**예상 못 한 발견 2건 (둘 다 신규 데이터 수집 0):**

- **(가) PLY 정점색을 렌더러 없이 직접 prototype 으로 쓰면 크게 개선됩니다.**
  객체별로 `render_old` / `ply_color` 중 나은 쪽을 학습 fold 에서만 골라 held-out 채점하면
  HSV AUROC **0.8467 → 0.9256**. 실사 prototype 상한(0.9881)까지의 격차 **56% 를 회수**합니다.
- **(나) 렌더러에 계통적 Hue 편향이 있습니다.**
  전 객체 공통 **Hue −5.6°** 보정 하나로 (LODO **6/6 fold 에서 동일하게 선택**)
  Hue-only AUROC 0.7881 → 0.8522, **Bear 는 0.5919 → 0.9865**.

- **saffron 은 aggregation 자체가 다릅니다.** 상위가 아니라 **하위 5개 view 평균(`bottom5`)**
  이 6/6 fold 에서 선택되며 held-out **0.982** (top5 0.928 / mean 0.947). "최선의 view 가
  얼마나 맞는가" 가 아니라 "최악의 view 도 얼마나 맞는가(일관성)" 가 더 강한 신호입니다.

---

## 2. 분석 환경과 데이터 provenance

### 2.1 격리 폴더

```
sam6d_ws/_ism_research_2026_07/ism_followup_research/
  semantic_analysis/     dump_view_sims.py  analyze_aggregation.py  analyze_tail.py
                         sweep_aggregation.py  make_figures.py  view_sims.npz
  high_ply_extraction/   inventory_ply.py  scan_ply_color.py  verify_identity.py
  high_ply_templates/    Bear_high/  saffron_high/  Febreze_high/  Mugcup_high/  (각 42 view)
  hsv_analysis/          triangulate_color.py  compare_prototypes.py
                         test_fusion.py  test_hue_calibration.py
  gt_seed_design/        measure_propagation.py  build_gt_plan.py
  results/               (아래 §9)
  figures/               (아래 §10)
  reports/
```

압축 해제본은 지시대로 전용 경로에 두었습니다:
`sam6d_ws/data/ply_files_excluding_20260702_extracted/` (1.5 GB, 31 PLY).
기존 `sam6d_ws/template/` 은 **읽기만** 했습니다.

### 2.2 재사용한 이전 산출물 (provenance)

| 자산 | 출처 | 이번 용도 |
|---|---|---|
| `box_labels_merged.csv` (504건) | `ism_accuracy_observation` (2026-07-20) | pos/neg 정의 |
| `<ds>_boxes.csv` / `<ds>_pairs.csv` | 동 | 박스 좌표, 운영 baseline 재현 |
| `<ds>_hsv.npz` (masked/bbox/background H32·S32·V32·H16×S8) | 동 | 실사 HSV |
| `template_render_hsv.npz` | 동 | 기존 렌더 HSV |
| `template/_logs/_batch_driver.log` | 운영 렌더 기록 (2026-06-16) | 어떤 PLY로 렌더했는지 확정 |

**라벨 신뢰도**: 504건 전부 `label_source = claude-vision (provisional)`, `review_status =
not_reviewed` 입니다. **사람 GT 가 아닙니다.** 이 보고서의 모든 AUROC 는 이 잠정 라벨 위에
서 있습니다.

### 2.3 신규 추론 (모델 재실행)

`semantic_view_similarity.csv` 생성을 위해 라벨된 504 박스에 대해서만 DINOv2 CLS 를 재계산했습니다
(YOLO·MobileSAM 은 실행하지 않음, 박스 좌표는 이전 덤프에서 재사용). 운영 모듈 `yolo_ism.py` ·
`yolo_ism_object_n.py` 는 **import 만** 했고 수정하지 않았습니다.

결과: **817 pair** (positive 294 / negative 523), 각 pair 마다 42-view 코사인 전체를 보존.

> **이전 보고서 수치와의 차이**
> 직전 대화에서 제시한 표(pos/neg 770 pair)와 본 보고서(817 pair)는 negative 집합 정의가
> 다릅니다. 이번에는 `unclear` 만 제외하고 `carton`·`other` 를 negative 로 포함했습니다.
> 그 결과 Sikhye 0.998→0.977, Febreze 0.970→0.973 등 소수 3째 자리가 이동했습니다.
> 결론(어느 객체에서 mean 이 이기는가)은 바뀌지 않았습니다.

---

## 3. 검증 1 — `sem_top5` vs `sem_mean` 의 객체별 원인

### 3.1 기각된 가설

첫 가설은 "곡선의 뾰족함(peakiness)" 이었습니다.
`peakiness = (top5평균 − 전체평균) / 전체표준편차` 로 정의하고
`Δpeak = peak(negative) − peak(positive)` 가 클수록 mean 이 유리할 것으로 봤습니다.

**결과: corr(Δpeakiness, ΔAUROC) = −0.251 (n=10) → 기각.**
정규화되지 않은 절대 격차(`Δgap`)로 바꿔도 +0.663 에 그쳤습니다.
(`results/aggregation_mechanism.json`)

### 3.2 확정된 기전

정의상 `sem_mean = (5/42)·sem_top5 + (37/42)·sem_tail` 이므로, mean 의 우열은 오직 꼬리가
결정합니다. 이를 직접 측정했습니다.

**corr( AUROC(꼬리) − AUROC(top5) , AUROC(mean) − AUROC(top5) ) = +1.000 (n=10)**

| 객체 | AUROC@1 | AUROC top5 | **AUROC 꼬리(6~42)** | AUROC mean | mean−top5 | corr(top5,꼬리) |
|---|---:|---:|---:|---:|---:|---:|
| Dinosaur | 0.536 | 0.544 | **0.904** | 0.848 | **+0.304** | 0.44 |
| Rabbit | 0.839 | 0.827 | 0.887 | 0.881 | +0.054 | 0.83 |
| choco_hazelnut_high | 0.927 | 0.937 | **0.983** | 0.983 | +0.046 | 0.80 |
| saffron | 0.919 | 0.923 | 0.954 | 0.951 | +0.028 | 0.98 |
| milk | 0.964 | 0.972 | **0.994** | 0.994 | +0.022 | 0.97 |
| Sikhye_high | 0.977 | 0.977 | 0.978 | 0.978 | +0.001 | 0.99 |
| Mugcup_high | 0.975 | 0.994 | 0.994 | 0.994 | 0.000 | 0.97 |
| Sauce_high | **0.999** | 0.997 | 0.976 | 0.981 | −0.016 | 0.96 |
| Bear | 0.696 | 0.690 | 0.632 | 0.650 | −0.040 | 0.80 |
| Febreze_high | 0.966 | 0.973 | 0.919 | 0.931 | −0.042 | 0.98 |

`results/semantic_tail_decomposition.csv`

### 3.3 순위별 AUROC 곡선 — 신호가 어디에 사는가

![순위별 AUROC](../../../sam6d_ws/_ism_research_2026_07/ism_followup_research/figures/semantic_rank_curves.png)

| 객체 | r1 | r3 | r5 | r10 | r20 | r30 | r42 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Dinosaur | 0.54 | 0.56 | 0.54 | 0.70 | 0.88 | 0.87 | **1.00** |
| choco_hazelnut_high | 0.93 | 0.92 | 0.95 | 0.95 | 0.98 | 0.98 | **0.99** |
| saffron | 0.92 | 0.92 | 0.93 | 0.93 | 0.95 | 0.96 | **0.95** |
| milk | 0.96 | 0.97 | 0.98 | 0.99 | 0.99 | **1.00** | 0.75 |
| Rabbit | 0.84 | 0.82 | 0.82 | 0.92 | 0.88 | 0.88 | 0.86 |
| Sikhye_high | 0.98 | 0.98 | 0.98 | 0.98 | 0.98 | 0.98 | 0.52 |
| Mugcup_high | 0.97 | 1.00 | 1.00 | 1.00 | 1.00 | 0.99 | 0.69 |
| Febreze_high | 0.97 | 0.98 | 0.97 | 0.96 | 0.91 | 0.90 | **0.38** |
| Sauce_high | **1.00** | 0.99 | 0.99 | 0.98 | 0.96 | 0.98 | **0.28** |
| Bear | 0.70 | 0.69 | 0.69 | 0.69 | 0.62 | 0.60 | 0.49 |

**두 가지 서로 다른 악화 기전이 있습니다.**

- **포화형 (Sauce, Febreze)** — r1 이 이미 0.97~1.00. 더 섞을 여지가 없고 37개 저유사 view 는
  희석만 합니다. 최하위 순위(r42)는 **역변별**(0.28 / 0.38)로 적극적으로 해롭습니다.
- **전면 약화형 (Bear)** — 어느 순위에서도 0.49~0.70. 포화가 아니라 **CLS 자체가 Bear 를
  구분하지 못합니다.** aggregation 을 바꿔도 해결되지 않습니다.

**개선형의 공통 성질**: 상위 view 는 혼동 객체와 **공유**되고(인형 실루엣, 갈색 상자),
구분 정보는 "잘 안 맞는 쪽" 에 있습니다. Dinosaur 가 극단으로, r1=0.54(우연)에서
r42=1.00 까지 단조 상승합니다.

### 3.4 42-view 유사도 곡선 (대표 6객체)

![42-view 곡선](../../../sam6d_ws/_ism_research_2026_07/ism_followup_research/figures/semantic_view_curves.png)

positive(파랑)/negative(빨강) 평균과 IQR. Dinosaur 는 상위 구간에서 두 곡선이 겹치고
하위로 갈수록 벌어지는 반면, Sauce/Febreze 는 상위에서 벌어지고 하위에서 붙습니다.

### 3.5 대칭성·단색성·형태와의 관계

지시하신 "대칭성, 단색성, 직사각형, 텍스처 부족" 과의 대응은 **부분적으로만 성립**합니다.

| 성질 | 해당 객체 | mean 우열 | 일관성 |
|---|---|---|---|
| 텍스처 풍부·강체 병 | Sauce, Febreze | **악화** | 일관 (2/2) |
| 인형(비강체·유사 실루엣) | Dinosaur, Rabbit / **Bear** | 개선 / **악화** | **불일치** |
| 직사각형 포장 | choco, milk | 개선 | 일관 (2/2) |
| 단색 원통 | saffron, Sikhye, Mugcup | 개선 / 무차별 / 무차별 | 약함 |

Bear 가 반례입니다. Bear 는 인형인데도 mean 이 악화하는데, 원인은 §3.3 의 "전면 약화형"
이지 aggregation 성질이 아닙니다. **따라서 기하 성질만으로 aggregation 을 정하면 안 됩니다.**

### 3.6 불량 template view 검증 (판정 D 해당 여부) — **기각**

고정 view 번호별 AUROC 를 재서 공통 불량 view 를 찾았습니다.

- **view 0** 이 4개 객체에서 AUROC<0.5, **view 41** 이 3개 객체에서 AUROC<0.5
- 그러나 두 view 를 제거해도 결과가 바뀌지 않습니다:
  top5 평균 0.883 → **0.882**, mean 평균 0.919 → **0.919**

불량 view 는 존재하지만 42개 평균 안에서 이미 희석되어 있어, 제거는 무효입니다.
**판정 D 는 어느 객체에도 해당하지 않습니다.**
(`results/semantic_view_profile.csv`)

### 3.7 aggregation 변형 스윕과 LODO 검증

전체 데이터로 최적 변형을 고르고 같은 데이터로 성능을 보고하면 과적합이므로,
**6-fold LODO** 로 학습 fold 에서만 변형을 고르고 held-out 에서만 채점했습니다.

| 객체 | fold수 | 선택형(held-out) | top5 | mean | trim0_1 | 최빈 선택 |
|---|---:|---:|---:|---:|---:|---|
| Bear | 3 | 0.628 | **0.680** | 0.652 | 0.652 | top2 (2/3) |
| Dinosaur | 1 | 1.000 | 0.667 | **1.000** | 1.000 | bottom1 (1/1) |
| Febreze_high | 4 | 0.949 | **0.960** | 0.923 | 0.925 | top5 (3/4) |
| Mugcup_high | 3 | 0.995 | 0.995 | **1.000** | 1.000 | top3 (1/3) |
| Rabbit | 2 | 0.833 | 0.800 | **0.900** | 0.900 | top1 (1/2) |
| Sauce_high | 6 | 0.997 | **0.997** | 0.982 | 0.982 | top2 (4/6) |
| Sikhye_high | 5 | 0.991 | 0.987 | 0.991 | 0.991 | trim10_10 (3/5) |
| choco_hazelnut_high | 4 | 0.905 | 0.901 | **0.964** | 0.964 | bottom10 (2/4) |
| milk | 5 | 0.995 | 0.998 | **1.000** | 1.000 | top42 (4/5) |
| **saffron** | 6 | **0.982** | 0.928 | 0.947 | 0.945 | **bottom5 (6/6)** |
| **전체평균** | 39 | 0.943 | 0.929 | **0.944** | 0.944 | |

**중요**: "객체별 최적 선택"(0.943)이 **고정 mean(0.944)을 이기지 못합니다.** 대부분의 객체에서
선택이 과적합입니다. 예외는 **saffron** 으로, `bottom5` 가 **6/6 fold 전부에서** 선택되고
held-out 에서도 +0.054 를 유지합니다. 이것은 과적합이 아니라 실재하는 성질입니다.

`bottom-k` 의 의미: 42개 view 중 **가장 안 맞는** view 들의 유사도. 진짜 saffron 은 최악의
각도에서도 어느 정도 닮지만, 유사 객체는 반드시 크게 어긋나는 view 가 있습니다.
즉 **최선(best-case) 정합이 아니라 최악(worst-case) 정합, 곧 일관성** 을 재는 지표입니다.
현재 시스템에는 이 축이 전혀 없습니다.

### 3.8 객체별 판정 (A~E)

판정 기준: held-out fold 가 3개 이상이고 차이가 재현될 때만 A/B/C, 아니면 E.

| 객체 | 판정 | 근거 |
|---|---|---|
| **choco_hazelnut_high** | **A** 전체 평균이 구조적으로 적합 | 꼬리 0.983 > top5 0.937, held-out mean 0.964 > top5 0.901 (4 fold) |
| **milk** | **A** | 꼬리 0.994 > top5 0.972, held-out 1.000 vs 0.998 (5 fold) |
| **Sikhye_high** | **A** (실질 무차별) | held-out 0.991 vs 0.987, 차이 0.004 — 어느 쪽이든 무해 (5 fold) |
| **Mugcup_high** | **A** (실질 무차별) | held-out 1.000 vs 0.995, 두 방식 모두 포화 (3 fold) |
| **Sauce_high** | **B** Top-5가 구조적으로 적합 | r1=0.999 포화, r42=0.28 역변별. held-out top5 0.997 > mean 0.982 (6 fold, 최다) |
| **Febreze_high** | **B** | r1=0.966 포화, r42=0.38 역변별. held-out top5 0.960 > mean 0.923 (4 fold) |
| **saffron** | **C** adaptive aggregation 필요 | `bottom5` 가 6/6 fold 선택, held-out 0.982 로 top5·mean 모두 상회. 상위/하위 어느 한쪽 고정으로는 최적이 아님 |
| **Bear** | **E** 현재 라벨 수로 판단 불가 | 전 순위 AUROC 0.49~0.70 (우연 근처). pos 28 / neg 19, held-out fold 3개. aggregation 이 아니라 **CLS 변별력 자체의 문제** |
| **Dinosaur** | **E** | held-out fold **1개** (pos 25 중 대부분이 sam_105314 에 집중). 전체 데이터로는 mean 이 압도(+0.304)하나 held-out 재현 불가 |
| **Rabbit** | **E** | held-out fold 2개, pos 21 / neg 8. 방향은 mean 우세이나 표본 부족 |

**판정 D 는 0건** (§3.6에서 실측 기각).

`results/semantic_aggregation_by_object.csv`, `results/semantic_lodo_aggregation.csv`

---

## 4. 검증 2 — `*_high.ply` 추출과 신규 템플릿

### 4.1 압축본 인벤토리

`ply_files_excluding_20260702.tar.gz` (195.5 MB) → 31개 PLY, 11개 객체 × {low, middle, high}
(+ `Husky_ani`). 손상 파일 없음. 전부 vertex color·normal·UV 보유.

**`*_high.ply` 10종**: Bear, Dinosaur, Febreze, Mugcup, Rabbit, Rack, saffron, Sauce,
Sikhye, choco_hazelnut. **`milk_high.ply` 는 없습니다.**

| 파일 | 크기 | vertex | face | 색 상태 |
|---|---:|---:|---:|---|
| Sauce_high | 423.7 MB | 3,801,600 | 1,267,200 | OK (uniq 6,322) |
| saffron_high | 281.6 MB | 2,502,840 | 834,280 | OK (uniq 2,084) |
| Sikhye_high | 269.3 MB | 2,409,216 | 803,072 | OK (uniq 1,388) |
| Bear_high | 72.9 MB | 651,312 | 217,104 | OK (uniq 4,990) |
| Rabbit_high | 67.0 MB | 599,034 | 199,678 | OK (uniq 2,377) |
| Dinosaur_high | 66.2 MB | 591,129 | 197,043 | OK (uniq 3,207) |
| Febreze_high | 28.5 MB | 256,992 | 85,664 | OK (uniq 579) |
| Rack_high | 19.5 MB | 176,256 | 58,752 | **NO_COLOR** (전 정점 186,186,186) |
| choco_hazelnut_high | 16.1 MB | 147,456 | 49,152 | OK (uniq 6,617) |
| Mugcup_high | 2.1 MB | 74,874 | 24,958 | WEAK (uniq 116, binary PLY) |

`results/high_ply_inventory.csv`, `results/ply_color_scan.csv`

> **방법 주의 1건**: 최초 인벤토리는 파일 **선두 20,000 정점**만 봤고 그 결과
> Febreze/Sikhye 가 `unique=1`(단색)로 보였습니다. 파일 **전체를 균등 표본**으로 재스캔하니
> Febreze uniq 579, Sikhye uniq 1,388 로 정상이었습니다. 진짜 무색은 **Rack 뿐**입니다.
> (Rack 은 ISM 운영 객체 목록에 없습니다.)

### 4.2 운영 PLY 와의 대응 — **핵심 발견**

정점 수만으로는 동일 여부를 알 수 없어, 좌표를 **중심화 + 최대반경 정규화**(스케일 불변)한 뒤
최대 절대오차와 색 완전일치율로 판정했습니다.

| ISM 객체 | 운영 PLY vertex | 대응 압축본 | 좌표 오차 | 색 일치 | 스케일 | high 업그레이드 여지 |
|---|---:|---|---:|---:|---:|---|
| **Dinosaur** | 591,129 | **Dinosaur_high** | 4.5e-05 | **1.0000** | 1000× | **없음 — 이미 high** |
| **Rabbit** | 599,034 | **Rabbit_high** | 3.7e-07 | **1.0000** | 1000× | **없음 — 이미 high** |
| **Sauce_high** | 3,801,600 | **Sauce_high** | 2.3e-07 | **1.0000** | 1198.5× | **없음 — 이미 high** |
| **Sikhye_high** | 2,409,216 | **Sikhye_high** | 2.7e-07 | **1.0000** | 1521.0× | **없음 — 이미 high** |
| Bear | 162,828 | **Bear_middle** | 2.1e-06 | 1.0000 | 1000× | 있음 (→ 651,312, 4×) |
| saffron | 600,228 | **saffron_middle** | 1.9e-07 | 1.0000 | 1000× | 있음 (→ 2,502,840, 4.2×) |
| Febreze_high | 230,208 | 일치 없음 (high=256,992) | — | — | — | 다른 스캔 |
| Mugcup_high | 53,640 | 일치 없음 (high=74,874) | — | — | — | 다른 스캔 |
| choco_hazelnut_high | **589,824** | 일치 없음 (high=**147,456**) | — | — | — | **운영이 4× 더 큼** |
| milk | 16,302 | 압축본에 없음 | — | — | — | 해당 없음 |

**10개 중 4개(Dinosaur·Rabbit·Sauce·Sikhye)는 이미 `*_high.ply` 로 렌더되어 있습니다.**
색이 100.00% 일치하므로 재렌더는 정의상 같은 이미지를 만듭니다 — 렌더하지 않았습니다.
choco 는 운영본이 압축본 high 보다 4배 조밀하므로 교체는 **후퇴**입니다.

`results/ply_identity.csv`

### 4.3 신규 렌더 (4객체 × 42 view)

렌더러·스크립트·인자를 운영과 **완전히 동일**하게 두고 입력 PLY 만 바꿨습니다.

| 항목 | 값 |
|---|---|
| 스크립트 | `sam6d_master/SAM-6D/Render/render_custom_templates_cpu.py` (운영과 동일) |
| 렌더러 | blenderproc + blender-3.3.1 (CPU 강제) |
| 카메라 | `cam_poses_level0.npy` 42 view |
| 조명 | POINT, energy=1000, scale=2.5 (view 마다 재배치) |
| 샘플 | `set_max_amount_of_samples(50)` |
| 정규화 | `get_norm_info` — 스케일 차이는 결과에 영향 없음 |

출력: `high_ply_templates/{Bear_high, saffron_high, Febreze_high, Mugcup_high}/templates/`
각 42 × {rgb, mask, xyz}. **기존 `template/` 은 건드리지 않았습니다.**

**Bear 직접 비교 (정점 162,828 → 651,312, 4배):**

| 지표 | 기존 | high PLY | 차이 |
|---|---:|---:|---:|
| 42뷰 평균 BGR | (132.10, 157.22, 188.15) | (132.05, 157.16, 188.08) | **0.07** |
| 마스크 픽셀 평균 | 32,658 | 32,382 | −0.8% |
| 이미지 평균 절대차 | — | — | **0.234 / 255** |

정점 4배 증가가 렌더 이미지를 **사실상 바꾸지 않습니다.** 이유는 §4.1 의 색 통계로 이미
예측 가능했습니다 — Bear_middle 과 Bear_high 의 정점색 평균이 **(226.7, 167.5, 131.5) 로
소수 첫째 자리까지 동일**합니다. 즉 tier 차이는 **메시 밀도**이지 색이 아닙니다.

![Bear 렌더 비교](../../../sam6d_ws/_ism_research_2026_07/ism_followup_research/figures/render_old_vs_high_Bear.png)

---

## 5. 검증 2b — 기존 렌더 vs `*_high.ply` 렌더 vs 실사 crop HSV

### 5.1 평가 설계

거리(Bhattacharyya) 절대값은 해석 척도가 없어(같은 객체라도 데이터셋이 다르면 벌어짐)
**운영상 의미 있는 질문**으로 바꿨습니다: *"이 prototype 으로 채점하면 해당 객체 박스를
다른 객체 박스와 얼마나 잘 구분하는가"* → 객체별 AUROC.

- 특징: **masked H16×S8** 히스토그램 (이전 조사에서 최적으로 확인된 변형)
- 점수: `max_{proto} (1 − Bhattacharyya)`
- 분할: **6-fold LODO**. 모든 prototype 을 **같은 held-out 집합**에서 채점
- 평가 박스 474개 (`unclear` 제외), 12개 라벨 클래스

### 5.2 결과표

| 객체 | 기존 렌더 AUROC | **high PLY 렌더 AUROC** | PLY 정점색 AUROC | 실사 기준 AUROC | **high PLY 개선량** | 판정 |
|---|---:|---:|---:|---:|---:|---|
| Bear | 0.5191 | 0.5182 | **0.9547** | 0.9991 | **−0.0009** | 개선 없음 |
| Febreze_high | 0.9009 | 0.9079 | 0.7455 | 0.9917 | **+0.0070** | 개선 없음(오차 수준) |
| Mugcup_high | 1.0000 | 1.0000 | 1.0000 | 1.0000 | **0.0000** | 포화 — 판별 불가 |
| saffron | 0.8351 | 0.8377 | 0.8866 | 0.9621 | **+0.0026** | 개선 없음 |
| Dinosaur | 1.0000 | (재렌더 불필요 — 이미 high) | 0.9990 | 1.0000 | — | 해당 없음 |
| Rabbit | 0.9335 | (재렌더 불필요) | 0.8939 | 0.9977 | — | 해당 없음 |
| Sauce_high | 0.5879 | (재렌더 불필요) | 0.8101 | 0.9988 | — | 해당 없음 |
| Sikhye_high | 0.8470 | (재렌더 불필요) | 0.4818 | 0.9810 | — | 해당 없음 |
| choco_hazelnut_high | 0.9743 | (운영이 더 조밀) | 0.8479 | 0.9882 | — | 해당 없음 |
| milk | 0.8687 | (압축본 없음) | 0.9726 | 0.9619 | — | 해당 없음 |
| **평균** | **0.8467** | **0.8159** (4객체) | **0.8592** | **0.9881** | **+0.0022** (4객체) | |

`results/old_vs_high_ply_hsv.csv`, `results/rendered_vs_real_hsv.csv`

### 5.3 Hue 분포 시각 확인

![HSV 비교](../../../sam6d_ws/_ism_research_2026_07/ism_followup_research/figures/hsv_render_vs_real.png)

`*_high.ply` 렌더(빨간 점선)가 기존 렌더(파랑)와 **완전히 포개집니다** — 수치 결과를
시각적으로 확증합니다. 동시에 **렌더가 실사(초록)보다 Hue 를 일관되게 오른쪽으로 밀고 있는**
것이 보입니다 (Bear 13 vs 7, saffron 23 vs 12, Sikhye 23 vs 17).

### 5.4 지시하신 6개 질문에 대한 답

**1. `*_high.ply` 가 기존 PLY보다 실제 색상을 더 잘 보존하는가?**
**아니오.** 애초에 10개 중 4개는 **이미 `*_high.ply`** 이고(색 100% 일치), 실제로 tier 가
낮았던 Bear·saffron 도 정점색 평균이 high tier 와 소수 첫째 자리까지 같습니다. tier 는
메시 밀도이지 색이 아닙니다. Febreze 는 압축본 high 의 색 다양성이 오히려 **더 낮습니다**
(uniq 579 vs 운영 2,379).

**2. 신규 렌더 HSV 가 실사 crop HSV 에 더 가까워지는가?**
**아니오.** 개선량 −0.0009 ~ +0.0070, 평균 **+0.0022**. Hue 곡선이 기존과 겹칩니다.

**3. 개선이 모든 객체에서 나타나는가?**
**어느 객체에서도 나타나지 않습니다.** Bear 는 오히려 −0.0009.

**4. 개선되지 않는 객체는 무엇이 원인인가?**
원인 분리를 위해 **PLY 정점색 자체**(렌더러 미경유)를 세 번째 지점으로 넣어 삼각측량했습니다.

| 객체 | 렌더 AUROC | PLY 정점색 AUROC | 차이 | 귀속 |
|---|---:|---:|---:|---|
| **Bear** | 0.519 | **0.955** | **+0.436** | **렌더러/조명** — 색은 PLY에 있는데 렌더가 잃음 |
| **Sauce_high** | 0.588 | **0.810** | **+0.222** | **렌더러/조명** |
| milk | 0.869 | 0.973 | +0.104 | 렌더러/조명 |
| saffron | 0.835 | 0.887 | +0.051 | 렌더러/조명 (경미) |
| Rabbit | 0.934 | 0.894 | −0.040 | 렌더러 무해 |
| choco | 0.974 | 0.848 | −0.126 | 렌더러가 오히려 도움 |
| Febreze_high | 0.901 | 0.746 | −0.155 | 렌더러가 오히려 도움 |
| **Sikhye_high** | 0.847 | **0.482** | **−0.365** | **PLY 색 문제** (정점색이 회색 편중, gray 비율 0.88) |

즉 **PLY 색상 문제 / renderer·조명 문제 / material 문제가 객체마다 다릅니다.**
단일 원인이 아닙니다. `*_high.ply` 교체는 이 중 **어느 것도** 건드리지 않으므로 무효입니다.

**5. `*_high.ply` 만으로 실사 prototype 없이 쓸 수준의 HSV 분별력을 얻는가?**
**아니오.** high 렌더 0.8159 vs 실사 LODO **0.9881**. 격차 0.17 이 그대로 남습니다.

**6. 렌더링 단계에서 color calibration 이 추가로 필요한가?**
**예. 그리고 효과가 큽니다** — §6 참조.

### 5.5 삼각측량 거리 (보조 진단)

| 객체 | d(PLY, 실사) | d(렌더, 실사) | d(PLY, 렌더) |
|---|---:|---:|---:|
| Bear | **0.406** | 0.780 | 0.795 |
| Mugcup_high | **0.423** | 0.586 | 0.769 |
| saffron | **0.546** | 0.688 | 0.851 |
| choco_hazelnut_high | 0.648 | 0.422 | 0.469 |
| Rabbit | 0.658 | 0.675 | 0.393 |
| Sauce_high | 0.774 | 0.697 | 0.559 |
| milk | 0.782 | 0.653 | 0.872 |
| Febreze_high | 0.844 | 0.618 | 0.655 |
| Dinosaur | 0.881 | 0.561 | 0.579 |
| Sikhye_high | 0.925 | 0.806 | 0.579 |

**한계 명시**: 이 거리들은 잡음 바닥(같은 객체의 데이터셋 간 거리)을 기준선으로 잡지
않았으므로 절대값 해석은 불가합니다. 객체 간 상대 비교와 §5.4 표의 AUROC 로만 판단했습니다.
`results/color_triangulation.csv`

---

## 6. 예상 못 한 발견 — 신규 촬영 없이 회복 가능한 것

지시하신 대로 "새 실사 이미지 사전 촬영"은 제외한 채, 이미 있는 자산만으로
어디까지 회복되는지 LODO 로 검증했습니다.

### 6.1 (가) PLY 정점색 prototype — 객체별 선택

| 방식 | 설명 | held-out 평균 AUROC |
|---|---|---:|
| S1 `render_old` | 현행 | 0.8467 |
| S2 `ply_color` | PLY 정점색만 | 0.8592 |
| S3 `render_plus_ply` | 두 prototype 합집합에 max | 0.8783 |
| S4 `render_satnorm` | 렌더 S·V 정규화 | 0.7918 ← **불안정, 기각** |
| **S5 `per_object_best`** | **객체별로 S1/S2 중 학습 fold 에서 선택** | **0.9256** |
| S6 `real_lodo` | 실사 prototype (상한 참고) | 0.9881 |

**선택 안정성** (fold 별로 독립 선택했는데 결과가 일치하는가):

| 객체 | 선택 | 안정성 | | 객체 | 선택 | 안정성 |
|---|---|---|---|---|---|---|
| Bear | `ply_color` | **4/4** | | Sauce_high | `ply_color` | **6/6** |
| milk | `ply_color` | **6/6** | | saffron | `ply_color` | 5/6 |
| Dinosaur | `render_old` | **6/6** | | Febreze_high | `render_old` | **5/5** |
| Rabbit | `render_old` | **5/5** | | Sikhye_high | `render_old` | **5/5** |
| choco | `render_old` | **4/4** | | Mugcup_high | `render_old` | **3/3** |

10개 중 9개가 만장일치입니다. 실사 상한까지의 격차 0.1414 중 **0.0789(56%)** 를
**추가 데이터 수집 0** 으로 회수합니다. S4(채도 정규화)는 Rabbit 이 0.934 → 0.281 로
붕괴하므로 기각했습니다.

`results/hsv_fusion_lodo.csv`

### 6.2 (나) 렌더러의 계통적 Hue 편향

§5.3 의 hue 이동이 우연인지 계통 편향인지 확인하기 위해, Hue 히스토그램을 순환 이동시키고
**이동량을 학습 fold 에서만 추정**했습니다 (H32 기준, 1 bin = 5.625°).

| 객체 | 현행 | 객체별 보정 | 공통 보정 | 객체별 이동량 |
|---|---:|---:|---:|---|
| **Bear** | 0.5919 | **0.9865** | **0.9865** | −5.6° |
| Febreze_high | 0.9312 | 0.9778 | 0.9778 | −5.6° |
| Sauce_high | 0.6110 | 0.8851 | 0.8851 | −5.6° |
| Sikhye_high | 0.8524 | 0.9424 | 0.9424 | −5.6° |
| milk | 0.8370 | 0.9286 | 0.8833 | −16.9 ~ −11.2° |
| Dinosaur | 0.9983 | 0.9983 | 0.9683 | 0.0 ~ +5.6° |
| Mugcup_high | 0.9984 | 0.9984 | 0.9212 | 0.0 ~ +5.6° |
| choco | 0.9472 | 0.9294 | 0.9341 | −5.6 ~ 0.0° |
| saffron | 0.7661 | 0.7868 | 0.7983 | −5.6 ~ 0.0° |
| **Rabbit** | 0.3480 | 0.8518 | **0.2252** | +45.0° ← **불안정** |
| **평균** | **0.7881** | **0.9285** | **0.8522** | |

**공통 이동량은 LODO 6/6 fold 전부에서 −1 bin (−5.6°) 로 동일하게 선택**됐습니다.
이는 객체별 우연이 아니라 **렌더러(blenderproc/blender 3.3.1 조명·셰이딩)의 계통 편향**
이라는 강한 증거입니다.

**주의 3가지**:
1. 이 표는 **Hue 32bin 단독** prototype 기준입니다(§5.2 는 H+S). 그래서 현행 기준값이
   0.7881 로 §5.2 의 0.8467 과 다릅니다. 두 표를 직접 비교하면 안 됩니다.
2. 최적 이동량 −5.6° 는 **1 bin = 최소 이동 단위**입니다. 실제 편향이 −5.6° 인지
   −1° ~ −8° 사이 어딘가인지는 이 해상도로 구분할 수 없습니다.
3. **Rabbit 은 무채색**(흰 토끼)이라 Hue 가 의미 없고, 공통 보정이 오히려 해롭습니다
   (0.348 → 0.225). 채도가 낮은 객체는 Hue 보정에서 제외해야 합니다.

`results/hue_calibration_lodo.csv`

---

## 7. 검증 3 — 최소 수동 GT 입력 설계

### 7.1 데이터의 시간적 성질 (실측)

| 데이터셋 | 프레임 | 길이 | 유효 fps |
|---|---:|---:|---:|
| sam_105018 | 401 | 133.5 s | 2.6 |
| sam_105314 | 594 | 99.0 s | 5.9 |
| sam_105652 | 549 | 183.3 s | 3.0 |
| sam_110104 | 547 | 182.3 s | 3.0 |
| sam_110532 | 152 | 51.3 s | 2.9 |
| sam_110633 | 353 | 118.0 s | 3.0 |
| **합계** | **2,596** | **767 s** | ~3.4 |

- 프레임당 운영 수락 객체 수: 평균 **2.61**, 중앙값 3, 최대 8, 0개 프레임 398개
- 프레임당 후보 보유 객체 수: 평균 6.37

**인접 프레임 BBox 전파 가능성 — 낮습니다.**

| 지표 | 값 |
|---|---:|
| 연속 프레임 수락 박스 IoU 중앙값 | **0.452** |
| IoU > 0.5 비율 | 53.8% |
| IoU > 0.7 비율 | **32.3%** |

3 fps 사무실 스윕이라 프레임 간 카메라 이동이 커서, **단순 BBox 복사 전파는 성립하지 않습니다.**
실제 tracker(광류·시각 추적)가 필요하며, 그 정확도는 이번 조사에서 측정하지 않았습니다.

`results/gt_propagation_stats.csv`

### 7.2 방안별 실제 비용 (6개 데이터 전체)

![GT 비용](../../../sam6d_ws/_ism_research_2026_07/ism_followup_research/figures/gt_seed_propagation.png)

| 방안 | 사용자 입력 항목 | 프레임 수 | 전체 대비 | 예상 시간* | 검출 비의존 | proposal miss 측정 | semantic/HSV 검증 |
|---|---|---:|---:|---:|:---:|:---:|:---:|
| A-10 | 보이는 객체 이름 목록 | 257 | 9.9% | 2.1 h | **O** | **O** | X |
| **A-25** | 보이는 객체 이름 목록 | **101** | **3.9%** | **0.8 h** | **O** | **O** | X |
| A-50 | 보이는 객체 이름 목록 | 49 | 1.9% | 0.4 h | **O** | **O** | X |
| B 상태변화 | 등장/소멸 프레임 | 1,614 | 62.2% | 13.4 h | X | X | X |
| C 트랙시작 | 객체별 최초 BBox | 807 | 31.1% | 6.7 h | X | X | O |
| D 불확실 | Top-3 중 선택 | **1,894** | **73.0%** | 15.8 h | X | X | O |
| **A-25 + D층 200** | 둘 다 | **301** | **11.6%** | **2.5 h** | **O** | **O** | **O** |

\* 프레임당 0.5분 가정. 실제 속도는 미측정입니다.

**방안 D 가 가장 비쌉니다.** "불확실"을 (Top1−Top2 gap<0.05) ∪ (후보 0) ∪ (클래스 전환) 으로
정의하면 2,596 중 **1,894 프레임(73%)** 이 걸립니다. 사실상 전수 라벨과 다르지 않습니다.

**방안 B·C 도 A 보다 비쌉니다.** ISM 수락 구간으로 센 "트랙"이 807개인데, 트랙 길이 중앙값이
대부분 **1~6 프레임**으로 잘게 쪼개져 있기 때문입니다. 이 파편화는 객체가 실제로 사라져서가
아니라 **검출이 끊겨서** 생깁니다.

> **중요한 한계**: 따라서 807 은 진짜 등장 횟수가 아니라 **상한**입니다. 진짜 등장/소멸
> 횟수는 가시성 GT 없이는 알 수 없습니다. 방안 B·C 의 실제 비용은 807 보다 낮을 수 있지만
> **얼마나 낮은지는 현재 산출물로 확인할 수 없습니다.**

### 7.3 결정적 논점 — B·C·D 는 proposal miss 를 잴 수 없습니다

방안 B(상태 변화)·C(트랙 시작)·D(불확실 프레임)는 **모두 검출 결과에서 출발**합니다.
그런데 proposal miss 는 정의상 *"객체가 보이는데 후보가 하나도 없는"* 사건이므로,
검출에서 출발하면 **원리적으로 관측할 수 없습니다**(순환 논리).

이전 6데이터 조사가 프레임 단위 가시성 GT 를 만들지 못해 proposal recall 의 분모를
확정하지 못한 것이 정확히 이 문제였습니다. **검출과 무관하게 뽑은 균등 표본(방안 A)만이
이 분모를 만듭니다.**

### 7.4 표본 크기와 추정 정밀도

| 방안 | 프레임 | 객체-프레임 관측* | 객체당 | 객체별 recall 95% CI 반폭 | 전체 합산 |
|---|---:|---:|---:|---:|---:|
| A-10 | 257 | ~671 | ~67 | ±11.0 %p | ±3.5 %p |
| A-25 | 101 | ~264 | ~26 | ±17.5 %p | ±5.5 %p |
| A-50 | 49 | ~128 | ~13 | ±25.1 %p | ±7.9 %p |

\* 프레임당 수락 객체 2.61 을 하한으로 사용(가시 객체는 이보다 많으므로 실제 관측 수는 더 큼).

**해석**: A-25 는 **전체 합산 proposal recall** 은 ±5.5 %p 로 충분히 잽니다. 그러나
**객체별** 비교(예: "Rabbit 이 Bear 보다 나쁜가")에는 ±17.5 %p 라 부족합니다.
객체별 결론까지 원하면 **A-10 (257 프레임, 약 2시간)** 이 필요합니다.

### 7.5 자동/반자동 확장 방법 비교

| 방법 | 사용자 최소 입력 | 자동 전파 가능 구간 | 사람 재확인 필요 | GT 신뢰도 | 오전파 방지 조건 |
|---|---|---|---|---|---|
| BBox 인접 복사 | 시작 BBox | **거의 없음** (IoU 중앙 0.452) | 대부분 | 낮음 | 사용 비권장 |
| Object tracking (CSRT/KCF 등) | 시작 BBox | 미측정 | tracker 끊김 지점 | 미검증 | IoU/신뢰도 급락 시 중단 |
| Optical flow | 시작 BBox | 미측정 | 급회전·모션블러 구간 | 미검증 | flow 잔차 임계 |
| MobileSAM mask propagation | 시작 점/박스 | 미측정 | 가림 시작/해제 | 미검증 | mask 면적 급변 시 중단 |
| **기존 ISM 고신뢰 결과 활용** | 없음 | **appe≥0.7 & gap≥0.19 구간** | 나머지 전부 | 중간 | **동일 라벨이 원인·결과가 되지 않도록 평가에서 제외** |
| **Temporal consistency** | 없음 | 동일 라벨 3연속 구간 | 전환 지점 | 중간 | 3프레임 미만 구간 불채택 |
| Active learning | 반복 검수 | — | 매 라운드 | — | 라운드마다 무편향 검증셋 유지 |
| **Contact sheet 일괄 검수** | 시트당 12장 판정 | — | — | 중간 | 150px 이상 썸네일 필수 |

> **직전 조사의 실패 사례**: 300 px 썸네일로 848×480 프레임의 가시성을 판정하려다
> 작거나 일부 가려진 객체를 판별할 수 없어 라벨 생성을 **포기**했습니다. contact sheet 를
> 쓴다면 프레임당 **최소 150 px 이상**, 가급적 개별 프레임 전체 보기가 필요합니다.

### 7.6 권장 GT 스키마

```
dataset_name        sam_105018 ...
frame_id            정수 (color 메시지 인덱스)
timestamp           ns
object_class        10개 ISM 객체명 + carton/other
visible             1 / 0
visibility_level    full / partial / heavily_occluded
bbox                x1,y1,x2,y2   (선택 — §7.7 참조)
occlusion_level     0.0~1.0 또는 등급
label_source        human / claude-vision / propagated
confidence          high / medium / low
propagated_from     원본 frame_id (전파본일 때만)
review_status       pending / reviewed / rejected
reviewer            이름
```

### 7.7 지시하신 6개 질문에 대한 답

**1. 사용자가 최소한 어떤 프레임을 직접 라벨링해야 하는가?**
**25프레임마다 1장, 6개 데이터 합계 101 프레임** (전체의 3.9%). 여기에 오차분석용
불확실 프레임 200장을 더한 **301 프레임(11.6%)** 을 권장합니다. 객체별 결론까지 필요하면
10프레임마다(257장)로 올리십시오.

**2. 객체 존재 여부만 필요한가, BBox 도 필요한가?**
**균등 표본(A층)은 존재 여부만으로 충분합니다.** proposal recall 의 분모는 "그 프레임에
그 객체가 보였는가" 이고, 후보 박스와의 대응은 이미 저장된 YOLO 박스로 자동 판정할 수
있습니다. **단 raw YOLO 패스의 박스 좌표는 현재 저장돼 있지 않아**(개수·최대 conf 만 저장)
"raw 에 후보가 있었다"가 정답 위치였는지는 확인 불가입니다 — 이 부분은 프로브 재실행이
필요하며 사용자 라벨과 무관합니다.
**BBox 가 필요한 곳은 D층뿐**이며, 그것도 "제시된 Top-3 박스 중 어느 것이 맞는가" 선택으로
대체 가능합니다.

**3. proposal miss 를 검증하려면 최소 어떤 GT 가 필요한가?**
**검출과 무관하게 뽑은 균등 표본 프레임의 가시 객체 목록**입니다. 이것 없이는 분모가
존재하지 않습니다. B·C·D 방안은 모두 검출 조건부라 **원리적으로 불가**합니다(§7.3).
정밀도: A-25 → 전체 ±5.5 %p, A-10 → 객체별 ±11 %p.

**4. Semantic/HSV 성능 검증에는 어떤 GT 가 필요한가?**
**박스 단위 클래스 라벨**입니다. 이미 **504건** 보유하고 있고 이 보고서의 모든 AUROC 가
그 위에 있습니다. 추가로 필요한 것은 **양이 아니라 질** — 504건 전부가 Claude Vision
잠정 라벨이므로 **사람 검수(우선 대기열 79건)** 가 선행되어야 합니다.
또한 6개 데이터가 **같은 날 같은 사무실**이라 조명 일반화는 어떤 라벨로도 검증할 수 없습니다.

**5. seed GT 로부터 나머지 프레임을 어느 정도 자동 생성할 수 있는가?**
**단순 BBox 전파로는 거의 불가능합니다** — 인접 프레임 IoU 중앙값 0.452, IoU>0.7 은 32.3%.
tracker/optical flow 의 실제 성능은 **이번 조사에서 측정하지 않았습니다**. 따라서
"몇 % 자동 생성 가능" 이라는 수치는 **현재 산출물로는 확인할 수 없습니다.**
확실히 말할 수 있는 것은, 자동 전파를 **평가용 GT 로 쓰면 안 된다**는 것입니다 —
전파의 근거가 검출이므로 검출 성능 평가가 순환합니다.

**6. 자동 생성 GT 를 최종 GT 로 쓰기 전에 어떤 프레임만 사람이 검수하면 되는가?**
- 동일 라벨 **3연속 미만** 구간 전부
- **클래스 전환** 프레임 (t−1 과 t 의 수락 클래스 집합이 다른 지점)
- **Top1−Top2 gap < 0.05** 프레임 (직전 조사: 정답Top1 중앙 0.188 vs 오답Top1 0.0217)
- 후보 **0개** 프레임 (전파가 근거를 잃는 지점)
- tracker IoU 또는 신뢰도 **급락** 지점

단 이 조건들의 합집합이 곧 방안 D(1,894 프레임, 73%)입니다. 즉 **자동 전파 + 전수 검수는
균등 표본보다 비쌉니다.** 자동 전파는 평가용 GT 가 아니라 **탐색·오차분석 보조**로만
쓰는 것이 합리적입니다.

### 7.8 실제 라벨 요청 목록

`results/human_label_request.csv` — **303 프레임** (균등 107 + 불확실층 196, 전체의 11.7%).

| 열 | 내용 |
|---|---|
| `dataset`, `frame_id` | 대상 프레임 |
| `stratum` | `uniform_every25` (우선순위 1) / `uncertain_top200` (우선순위 2) |
| `reason` | 왜 이 프레임인지 |
| `current_ism_accepted`, `n_current_accepted` | 현재 운영 판정 (참고용) |
| `visible_objects`, `occluded_objects` | **사용자가 채울 칸** |
| `label_source`, `confidence`, `review_status`, `reviewer`, `notes` | 메타 |

> **편향 주의**: `current_ism_accepted` 를 먼저 보고 "맞다/틀리다" 를 판정하면 확증 편향이
> 생깁니다. 균등 표본(우선순위 1)은 **현재 판정을 가린 채** 보이는 객체를 독립적으로
> 적어야 무편향 분모가 됩니다.

---

## 8. 종합 판단과 후속 우선순위

| 후속 | 내용 | 근거 | 위험 | 순서 |
|---|---|---|---|---|
| **F1** | **Hue −5.6° 공통 보정 + 무채색 객체 예외** | 6/6 fold 동일 선택, Bear 0.592→0.987 | Rabbit 등 저채도 객체 악화 (0.348→0.225) | **1** |
| **F2** | **객체별 `render_old`/`ply_color` prototype 선택** | 10객체 중 9개 만장일치, +0.079 | 잠정 라벨 기반, 조명 일반화 미검증 | **2** |
| **F3** | **균등 GT 101~257 프레임 수집** | proposal recall 분모 부재 | 사용자 작업 0.8~2.1 h | **3** |
| **F4** | saffron `bottom-k` aggregation 실험 | 6/6 fold, held-out +0.054 | 단일 객체 전용 규칙 위험 | 4 |
| **F5** | Sauce/Febreze `top5` 유지 확인 | held-out 6/4 fold 재현 | 현행 유지이므로 위험 없음 | 4 |
| **F6** | Bear CLS 변별력 문제 별도 조사 | 전 순위 AUROC 0.49~0.70 | aggregation 으로 해결 불가 | 5 |
| — | ~~`*_high.ply` 템플릿 교체~~ | **개선량 +0.002 → 기각** | — | — |
| — | ~~불량 template view 제거~~ | **효과 0.000 → 기각** | — | — |
| — | ~~방안 D 단독 GT~~ | **73% 비용 → 기각** | — | — |

### 검증 지표 (F1·F2 진행 시)

절대 AUROC 만 보지 말고 다음을 함께 보십시오.
- 객체별 AUROC **와 최저 객체** (평균에 회귀 숨기지 않기)
- 결정 단위 TP/FP/FN — 단, **임계 재보정을 반드시 동반** (점수 스케일이 바뀜)
- LODO 6 fold **전부**의 fold 별 값 (평균만 보면 1개 fold 붕괴를 놓침)
- 저채도 객체(Rabbit, Mugcup) 회귀 여부

---

## 9. 산출 파일

```
_ism_research_2026_07/ism_followup_research/results/
  semantic_aggregation_by_object.csv    ★지시 산출물  객체별 top5/mean AUROC·PR·분리도
  semantic_view_similarity.csv          ★지시 산출물  817 pair × 42-view 유사도 전체
  high_ply_inventory.csv                ★지시 산출물  압축본 31 + 운영 16 PLY 인벤토리
  old_vs_high_ply_hsv.csv               ★지시 산출물  기존/high/PLY색/실사 AUROC 비교
  rendered_vs_real_hsv.csv              ★지시 산출물  위의 fold 단위 원자료
  minimum_gt_plan.csv                   ★지시 산출물  방안 A~D + 권장안 비용/획득물
  human_label_request.csv               ★지시 산출물  303 프레임 라벨 요청 목록

  semantic_tail_decomposition.csv       꼬리 분해 (기전 확정 근거)
  semantic_rank_auroc.csv               객체 × 순위 k AUROC
  semantic_view_profile.csv             객체 × 고정 view 번호 AUROC (불량 view 탐지)
  semantic_aggregation_sweep.csv        22개 aggregation 변형 전체 스윕
  semantic_lodo_aggregation.csv         LODO 검증 (fold 단위)
  aggregation_mechanism.json            기각된 가설 포함 상관계수
  ply_color_scan.csv                    PLY 전체 표본 색 통계
  ply_identity.csv                      운영 PLY ↔ 압축본 동일성 판정
  color_triangulation.csv               PLY/렌더/실사 거리 삼각측량
  hsv_fusion_lodo.csv                   prototype 6방식 LODO
  hue_calibration_lodo.csv              Hue 보정 LODO
  gt_propagation_stats.csv              (데이터셋, 객체) 별 트랙·IoU 통계
  gt_cost_by_plan.csv                   데이터셋별 방안 비용
```

---

## 10. 시각 자료

```
figures/
  semantic_rank_curves.png            순위별 AUROC — mean 유리군 vs 불리군 대비
  semantic_view_curves.png            대표 6객체 42-view 곡선 (pos/neg, IQR)
  render_old_vs_high_Bear.png         기존 vs high PLY 렌더 + 차분 (6 view)
  render_old_vs_high_saffron.png      동
  render_old_vs_high_Febreze_high.png 동
  render_old_vs_high_Mugcup_high.png  동
  hsv_render_vs_real.png              실사/기존렌더/high렌더 Hue 분포 (6객체)
  gt_seed_propagation.png             인접 IoU 분포 + 방안별 수동 비용
```

---

## 11. 한계와 재현 조건

1. **모든 라벨이 잠정입니다.** 504건 전부 Claude Vision `provisional`, 사람 검수 0건.
   사람 GT 가 아닙니다. 우선 검수 대기열 79건이 `ism_accuracy_observation/labels/
   human_review_queue.csv` 에 있습니다.
2. **6개 데이터가 같은 날 같은 사무실**입니다. LODO 는 데이터셋 누수를 막지만
   **조명·카메라 조건 일반화는 검증하지 못합니다.** Hue 보정(F1)은 특히 조명 의존적입니다.
3. **held-out fold 수가 객체마다 다릅니다** (Dinosaur 1, Rabbit 2, Sauce 6).
   fold 3개 미만은 판정 E 로 두었습니다.
4. **Hue 보정 해상도가 5.625°(1 bin) 입니다.** 실제 편향의 정확한 크기는 이 격자로
   확정할 수 없습니다.
5. **§6 의 AUROC 는 HSV 단독 성능**이며 end-to-end FP/FN 이 아닙니다.
   운영 적용 시 semantic·appearance 게이트와의 상호작용은 별도 검증이 필요합니다.
6. **자동 전파(tracker/flow)의 실제 성능은 측정하지 않았습니다.** §7.5 의 "미측정" 표기
   항목은 추정이 아니라 미실험입니다.
7. **방안 B·C 의 비용 807/1,614 는 상한**입니다. 진짜 등장·소멸 횟수는 가시성 GT 없이
   알 수 없습니다.
8. **재현**: 모든 스크립트는 `--` 인자 없이 그대로 실행하면 같은 결과를 냅니다.
   환경은 `sam_yolo`(분석) / `sam6d_ros_humble`(blenderproc 렌더) 입니다.

---

## 12. 결론

```text
Semantic 전체 평균이 좋아지는 객체의 공통 원인:
꼬리(6~42위 view)가 상위 5개보다 변별력이 높기 때문이며, 이는 상관계수 +1.000 (n=10) 으로
확정된 유일한 기전이다. 구조적으로는 이 객체들의 "가장 잘 맞는 view" 가 혼동 객체와
공유되기 때문이다 — Dinosaur 는 r1 AUROC 0.536(우연)에서 r42 1.000 까지 단조 상승하고,
choco 도 0.93 → 0.99 로 상승한다. 즉 구분 정보가 최선의 정합이 아니라 정합이 실패하는
쪽에 들어 있다. 처음 세운 peakiness(곡선 뾰족함) 가설은 상관 −0.251 로 기각되었다.

Semantic 전체 평균이 나빠지는 객체의 공통 원인:
서로 다른 두 기전이 있다. (i) 포화형 — Sauce(r1 0.999)·Febreze(r1 0.966)는 상위 순위가
이미 거의 완벽해 37개 저유사 view 를 섞으면 희석만 되고, 최하위 순위는 오히려 역변별이다
(Sauce r42 0.28, Febreze r42 0.38). (ii) 전면 약화형 — Bear 는 어느 순위에서도 AUROC
0.49~0.70 으로, 포화가 아니라 CLS 자체가 Bear 를 구분하지 못한다. 따라서 Bear 는
aggregation 을 어떻게 바꿔도 해결되지 않는다. 한편 "불량 template view 제거" 가설은
공통 불량 view(view 0: 4객체, view 41: 3객체)를 실제로 찾았음에도 제거 효과가 0.000 이라
기각되었다.

객체별 권장 semantic aggregation:
- A (전체 평균 적합)      : choco_hazelnut_high, milk, Sikhye_high(무차별), Mugcup_high(무차별)
- B (Top-5 적합)          : Sauce_high, Febreze_high
- C (adaptive 필요)       : saffron — bottom5 가 6/6 fold 에서 선택되고 held-out 0.982 로
                            top5(0.928)·mean(0.947)을 모두 상회. 최선이 아닌 "최악 view 의
                            정합(일관성)" 축이 필요하며 현재 시스템에는 이 축이 없다
- D (불량 view 제거)      : 해당 객체 없음 (제거 효과 0.000 실측)
- E (라벨 부족으로 불가)  : Bear(전 순위 우연 근처, fold 3), Dinosaur(fold 1), Rabbit(fold 2)
전체로는 held-out 평균이 mean 0.944 > 객체별선택 0.943 > top5 0.929 이므로,
"객체별 최적 선택" 은 saffron 을 제외하면 과적합이다. 즉 baseline 을 통째로 바꾸는 것보다
saffron 단일 예외를 검증하는 쪽이 근거가 강하다.

*_high.ply 템플릿 HSV 판정:
- 기존 템플릿 대비: 개선 없음. AUROC 개선량 -0.0009(Bear) ~ +0.0070(Febreze), 평균 +0.0022.
  Bear 는 정점을 4배(162,828→651,312) 늘렸는데도 렌더 이미지 평균 절대차가 0.234/255 이고
  42뷰 평균 BGR 차이가 0.07 이다. 근본 원인은 tier 가 메시 밀도이지 색이 아니기 때문으로,
  Bear_middle 과 Bear_high 의 정점색 평균이 (226.7,167.5,131.5) 로 소수 첫째 자리까지 같다.
  더 결정적으로 10개 객체 중 4개(Dinosaur·Rabbit·Sauce_high·Sikhye_high)는 이미
  *_high.ply 로 렌더되어 있고(정규화 좌표 오차 <5e-05, 색 일치율 1.0000), choco 는 운영본이
  압축본 high 보다 4배 조밀하며, milk 는 압축본에 존재하지 않는다.
- 실사 crop 대비: 여전히 큰 격차. high 렌더 0.8159 vs 실사 LODO 0.9881.
- 실사 prototype 대체 가능 여부: 불가능. 다만 실사 촬영 없이도 격차의 56% 는 회수 가능하다 —
  객체별로 렌더 prototype 과 PLY 정점색 prototype 중 나은 쪽을 고르면(학습 fold 에서만 선택,
  10객체 중 9개가 fold 만장일치) held-out 0.8467 → 0.9256 이다.

렌더 HSV 개선을 위해 추가로 필요한 작업:
첫째, 렌더러의 계통적 Hue 편향 보정이 가장 효과가 크다. 공통 -5.6° 이동이 LODO 6/6 fold
전부에서 동일하게 선택되었고 Hue-only AUROC 0.7881 → 0.8522, Bear 는 0.5919 → 0.9865 이다.
단 Rabbit 같은 무채색 객체는 오히려 악화(0.348→0.225)하므로 채도 기준 예외가 필요하고,
보정 해상도가 1 bin = 5.625° 라 정확한 편향 크기는 이 격자로 확정할 수 없다.
둘째, 원인이 객체마다 다르므로 일괄 처방은 금물이다 — PLY 정점색을 직접 쓰면 Bear +0.436,
Sauce +0.222 로 개선되지만 Sikhye 는 -0.365, Febreze 는 -0.155 로 악화한다.
Sikhye 는 PLY 정점색 자체가 회색 편중(gray 비율 0.88)이라 PLY 문제이고, Bear/Sauce 는
색이 PLY 에 있는데 렌더가 잃는 렌더러 문제다. 셋째, 채도·명도 정규화(S4)는 Rabbit 이
0.934 → 0.281 로 붕괴하므로 채택하면 안 된다.

사용자가 직접 입력해야 하는 최소 GT:
25프레임마다 1장, 그 프레임에 "보이는 객체 이름 목록" 만. BBox 는 필요 없다.
이유는 proposal recall 의 분모가 "그 프레임에 그 객체가 보였는가" 이고, 후보 박스와의
대응은 이미 저장된 YOLO 박스로 자동 판정되기 때문이다. 결정적으로 방안 B·C·D 는 모두
검출 결과에서 출발하므로 "객체가 보이는데 후보가 없는" 사건을 원리적으로 관측할 수 없다.
균등 표본만이 이 분모를 만든다. 단 균등 표본을 라벨할 때는 현재 ISM 판정을 가린 채
독립적으로 적어야 확증 편향이 생기지 않는다.

6개 데이터의 예상 수동 라벨 수:
권장안 A-25 + 불확실 200장 = 301 프레임 (전체 2,596 의 11.6%, 프레임당 0.5분 가정 시 약 2.5시간).
균등 층만이면 101 프레임(3.9%, 0.8시간)이며, 이 규모로는 전체 합산 proposal recall 을
±5.5 %p 로 잰다. 객체별 비교까지 원하면 A-10(257 프레임, 2.1시간)이 필요하고 이때
객체별 정밀도가 ±11.0 %p 가 된다. 대조적으로 방안 D 는 1,894 프레임(73%), 방안 B 는
1,614(62%), 방안 C 는 807(31%) 로 모두 균등 표본보다 비싸다. 실제 요청 목록 303행은
results/human_label_request.csv 에 생성해 두었다.

자동 GT 확장 방법:
단순 인접 프레임 BBox 복사는 성립하지 않는다 — 연속 프레임 수락 박스의 IoU 중앙값이
0.452 이고 IoU>0.7 인 쌍이 32.3% 뿐이다(3 fps 사무실 스윕이라 프레임 간 카메라 이동이 크다).
tracker·optical flow·MobileSAM mask propagation 의 실제 성능은 이번 조사에서 측정하지
않았으므로 "몇 % 자동 생성 가능" 은 현재 산출물로는 확인할 수 없다. 확실한 것은 자동 전파
결과를 평가용 GT 로 쓰면 안 된다는 점이다 — 전파의 근거가 검출이므로 검출 성능 평가가
순환한다. 자동 전파는 평가 GT 가 아니라 탐색·오차분석 보조로만 쓴다.

사람이 반드시 재검수해야 하는 경우:
동일 라벨 3연속 미만 구간, 클래스 전환 프레임, Top1-Top2 gap < 0.05 프레임, 후보 0개 프레임,
tracker IoU/신뢰도 급락 지점. 다만 이 조건들의 합집합이 바로 방안 D(1,894 프레임, 73%)이므로,
"자동 전파 + 전수 검수" 는 균등 표본보다 비싸다는 점을 인지해야 한다.

현재 단계에서 운영 코드 변경:
없음.

다음 권장 실험:
렌더 템플릿에 공통 Hue -5.6° 보정을 적용하되 저채도 객체를 예외 처리하는 실험.
근거는 (a) LODO 6/6 fold 에서 동일한 이동량이 선택되어 객체별 우연이 아니라 렌더러의
계통 편향임이 확인되었고, (b) Bear 가 0.5919 → 0.9865 로 가장 크게 개선되며,
(c) 신규 데이터 수집이 전혀 필요 없고 기존 렌더 템플릿의 후처리만으로 끝나기 때문이다.
반드시 함께 확인할 것: Rabbit·Mugcup 등 저채도 객체의 회귀 여부(현재 Rabbit 은 공통 보정으로
0.348 → 0.225 악화), fold 별 개별 값(평균이 단일 fold 붕괴를 가리지 않도록), 그리고
점수 스케일 변화에 따른 임계 재보정. 이 실험이 끝난 뒤 F2(객체별 prototype 선택)를
같은 방식으로 검증하고, 그와 병행해 F3(균등 GT 101~257 프레임)을 수집하면
proposal recall 의 분모를 처음으로 확정할 수 있다.
```

---

## 다음 액션 제안

이번 조사는 관찰·검증만 수행했고 운영 코드는 변경하지 않았습니다. 다음 중 어느 것을
진행할지 알려주시면 이어서 작업하겠습니다.

1. **Hue 보정 실험 (F1)** — 렌더 템플릿 후처리만으로 끝나며 신규 데이터가 필요 없습니다.
   저채도 객체 예외 규칙을 함께 설계하고 LODO 로 회귀를 확인합니다.
2. **객체별 prototype 선택 실험 (F2)** — PLY 정점색을 prototype 으로 추가하고
   객체별 선택 규칙을 고정한 뒤 결정 단위(TP/FP/FN)로 재평가합니다.
   이때 임계 재보정이 반드시 동반되어야 합니다.
3. **균등 GT 수집 (F3)** — `results/human_label_request.csv` 의 101(또는 257) 프레임을
   직접 라벨해 주시면, proposal recall 의 분모를 처음으로 확정하고 직전 조사에서
   "확인할 수 없음" 으로 남긴 항목들을 닫을 수 있습니다.
   라벨 편의를 위한 개별 프레임 뷰어/시트를 먼저 만들어 드릴 수 있습니다.
4. **saffron bottom-k 실험 (F4)** — 단일 객체 예외 규칙의 위험을 감수할지,
   아니면 전 객체 공통 aggregation 을 유지할지 먼저 결정이 필요합니다.
5. **사람 검수 79건 (선결 과제)** — 위 1~4 모두가 잠정 라벨 위에 서 있으므로,
   우선 검수 대기열부터 처리하는 것이 가장 보수적인 선택입니다.

**질문**: 위 1~5 중 어느 것을 먼저 진행할까요? 특히 **3번(균등 GT 수집)** 은 사용자
작업 시간이 0.8~2.1시간 필요한데, 지금 이 비용을 지불할 의사가 있으신지, 아니면
먼저 코드만으로 가능한 1·2번으로 근거를 더 쌓은 뒤에 결정하시겠는지 알려주십시오.
