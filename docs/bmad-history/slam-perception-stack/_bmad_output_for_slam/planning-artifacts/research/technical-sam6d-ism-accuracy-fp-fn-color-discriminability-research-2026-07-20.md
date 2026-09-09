---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments:
  - 'sam6d_ws/outputs/yolo_ism_audit/audit_cases.csv (복구된 milk 9-bag 라벨 1,241행)'
  - '_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-ism-discriminability-latency-methodology-research-2026-07-20.md'
  - '_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-ism-score-calibration-tp-fp-fn-audit-2026-07-16.md'
workflowType: 'research'
lastStep: 1
research_type: 'technical'
research_topic: 'SAM-6D ISM FP/FN/유사객체 혼동의 근본원인 검증과 정확도 우선 개선구조 결정'
research_goals: '실패사례 실측 재현, 오류 단계 귀속, color/block2/negative/margin ablation, 정확도 우선 ISM 구조 1개 결정'
user_name: 'ldh'
date: '2026-07-20'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-07-20
**Author:** ldh
**Research Type:** technical

---

## Research Overview

본 조사는 ISM의 **FP·FN·유사객체 혼동**의 근본 원인을 실제 코드와 실제 프레임으로 검증하고, 정확도 우선(FP > FN > confusion > recall 유지) 개선 구조를 하나 결정한다. 처리시간은 참고 지표로만 기록했다.

수행한 것:

1. **라벨 복구** — 미보존으로 기록된 라벨 중 `outputs/yolo_ism_audit/audit_cases.csv`(milk 9-bag, 1,241행, TP/FN/TN + fn_reason)를 **복구**했다. 초코하임·인형 라벨은 존재하지 않아 새로 만들었다.
2. **신규 라벨 생성** — 149프레임 6 bag에서 후보 박스 1,007개를 덤프하고, 그중 **MobileSAM까지 도달한 고유 박스 338개를 컨택트 시트(150px 타일 12장) 육안 검수로 전수 라벨링**했다. 라벨은 `ism_accuracy_analysis/labels/box_labels.csv`에 provenance와 함께 **영구 저장**했다(사용자 Phase 1 선이행).
3. **특징 전수 덤프** — 박스×객체 10,070쌍에 대해 semantic 7변형, appearance(블록 2/9/11 × 42-view 전체), masked color(HSV/Lab), mask 품질, class margin, negative prototype을 계산.
4. **ablation** — bag 단위 calib/eval 분리로 B0~B20을 **분모 고정 (frame, object) 단위**로 평가. 고정 임계 결과와 재보정 결과를 모두 보고.

운영 소스는 수정하지 않았고 모든 산출물은 `sam6d_ws/ism_accuracy_analysis/`(삭제만으로 원상복구)에 있다. git 조작 없음.

표기: **[실측]** 본 조사 측정, **[복구]** 기존 라벨, **[정적]** 코드 확인, **[한계]** 표본·라벨 제약.

---

## 1. Executive Summary

**① 사용자가 지목한 3대 문제는 모두 실재하지만, 가장 큰 FP는 지목되지 않은 4번째 문제였다.** 라벨된 366개 결정에서 FP 44건의 분포는 **샤프란 액체세제 → Febreze 오인 20건(45%)**, 인형 상호 혼동 15건(34%), **갈색 택배박스 → 초코하임 5건(11%)**, Febreze → 샤프란 3건, Sikhye → Sauce 1건이다. 갈색 박스 문제는 실재하지만 전체 FP의 1/9이며, **동일 프롬프트를 공유하지 않는 "액체 용기끼리의 혼동"이 최대 FP원**이다. [실측]

**② FP·혼동의 지배적 원인은 threshold도 color도 아니라 "closed-set 절대 점수 구조"다.** 현재 각 객체는 다른 객체의 점수를 보지 않고 독립적으로 yes/no를 판정한다. 같은 박스에 대해 **class margin(1등 클래스 − 2등 클래스)** 한 항을 넣는 것만으로 eval bag FP가 10 → 3, 인형 혼동 5 → 0, 갈색박스→초코하임 1 → 0이 되었다. [실측]

**③ semantic Top-5 평균이 negative를 체계적으로 띄운다 — 사용자의 §13.2 가설은 실측으로 확인됐다.** 42템플릿 중 "가장 잘 맞는 5장"의 평균은 갈색 택배박스처럼 여러 직사각형 view에 약하게 닮은 negative에게 유리하다. 초코하임 vs 갈색박스 AUROC: `sem_top5` **0.920** → `sem_mean`(42장 전체 평균) **0.992**, `sem_median` 0.983. 전 객체 평균 AUROC도 0.923 → **0.958**, 최악 객체 0.657 → **0.873**. [실측]

**④ "DINOv2 block2가 color를 이미 담고 있으니 명시적 color는 불필요"라는 이전 결론은 절반만 맞다 — 반증한다.** block2 masked patch는 인형 3종에서 AUROC **1.000/1.000/1.000**으로 완벽하지만, **초코하임 vs 갈색택배박스에서 0.204 — 우연보다 나쁘다**(택배박스를 진짜 초코하임보다 **높게** 평가한다). 같은 조건에서 명시적 masked HSV color는 **0.912**다. **block2 ≠ explicit color**이며, 갈색박스 문제에 대해 block2는 해결책이 아니라 문제의 일부다. [실측]

**⑤ 그럼에도 최종 권장안에서 color는 채택하지 않는다.** class margin을 넣으면 color가 제공하던 변별력이 사라지기 때문이다(B15 vs B17: color를 초코하임 전용으로 더해도 FP 3 → 3, FN 4 → 4로 **완전 동일**). class margin은 **암묵적 negative prototype**이고, 경쟁 클래스의 자기 점수가 color보다 더 강한 negative 증거를 제공한다.

**⑥ 객체별 threshold 재보정 단독은 실패한다.** B1(운영 점수에 객체별 임계 재보정)은 eval에서 P0.829 **R0.659** — 손으로 맞춘 운영 임계(P0.792 R0.864)보다 **나쁘다**. 임계는 bag을 건너 일반화되지 않는다. 저장소가 세 번 롤백한 이력과 일치한다. [실측]

**최우선 권장안: B15 — "42-view mean semantic + class-margin fusion gate"**
eval bag에서 **FP 10 → 3(−70%), FN 6 → 4(−33%), Precision 0.792 → 0.930, Recall 0.864 → 0.909**, 인형 혼동 5 → 0, 갈색박스→초코하임 1 → 0. FP와 FN을 **동시에** 줄인 유일한 변형이다.

---

## 2. 연구 범위와 정확도 우선순위

우선순위(사용자 지정): FP 감소 > FN 감소 > class confusion 감소 > TP/Recall 유지 > 처리시간.

범위 안: ISM 인식(YOLO 후보 → semantic → mask → appearance → 수락). 범위 밖: PEM/6D pose, Object Memory, 처리시간 최적화(참고만).

**중요한 범위 한계**: 본 조사의 정량 평가는 **YOLO가 후보를 낸 (frame, object) 그룹**만 다룬다. YOLO가 애초에 후보를 내지 않아 생기는 FN(E1/E2)은 이 분모 밖이며, 복구된 milk 라벨로 별도 정량화했다(§9.2).

---

## 3. 실제 운영 ISM 데이터 흐름 [정적]

| 단계 | 함수 / 파일:라인 | 내용 |
|---|---|---|
| entry (PEM 브릿지) | `tools/build_ism_inputs_imu.py:262,295` | 프롬프트별 YOLO 패스 10회 → `o_n.recognize()` 객체별 |
| entry (ROS 실시간) | `src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py:217` | sidecar YOLO → `o_n.recognize()` 객체별 |
| entry (오프라인 배치) | `yolo_ism_object_n.py:616` | `set_classes(all)` 1패스 → `recognize_frame()` 배치 |
| 후보 | `yolo_ism_object_n.py:286-287` | 객체별 `score_threshold` → `top_k=3` 절단 |
| DINOv2 | `yolo_ism_object_n.py:72-90` | `get_intermediate_layers(n={2,9,11})` 1회 = CLS + patch 동시 |
| semantic | `yolo_ism.py:238-242` | `topk(sims, 5).mean()` — 42 템플릿 CLS 중 상위 5 평균 |
| 선택 | `yolo_ism_object_n.py:310-314` | `argmax(sem)` 1개, `sem < 0.35` 이면 거절 |
| mask | `yolo_ism.py:276-286` | `segment_box` — 선택된 1박스에만 MobileSAM |
| mask→patch | `yolo_ism.py:313-321` | AvgPool14 coverage > 0.5 |
| appearance | `yolo_ism_object_n.py:332-333` | `best_t = argmax(tcls @ cls)` → **템플릿 1장**의 patch와 비교 |
| 최종 게이트 | `yolo_ism_object_n.py:336-339` | `masked_appe >= appe_gate(0.55)` |
| cross-object | `build_ism_inputs_imu.py:307-312` | IoU>0.5 NMS, 승자 = `rank_appe`(블록[2,9]) |
| PEM 전달 | `build_ism_inputs_imu.py` | mask → RLE → `detection_<obj>.json` |

**closed-set 구조 확인**: 각 객체의 accept/reject는 그 객체의 템플릿과의 절대 유사도만 본다. 다른 클래스의 점수는 **cross-object NMS(IoU>0.5로 겹칠 때)에서만** 쓰인다. 서로 다른 위치의 두 물체(샤프란 병과 Febreze 병이 각각 다른 곳에 있는 프레임)에서는 클래스 간 비교가 **전혀 일어나지 않는다.** 이것이 §1-② 결론의 코드적 근거다.

---

## 4. 평가 데이터와 라벨 provenance

### 4.1 복구된 라벨 [복구]

`sam6d_ws/outputs/yolo_ism_audit/audit_cases.csv` — **1,241행**, 9 bag, milk 단일 객체.
필드: `milk_present, view_state, milk_in_any_proposal, decision, category, fn_reason, fp_reason, selected_box_correct`.
분포: **TP 482 / TN 446 / FN 294 / UNCERTAIN 19, FP 0**.
2026-07-16 보고서가 언급한 1,521건(초코하임·인형 포함)은 CSV/JSON/로그 어디에도 없어 **복구 불가**로 확정했다(검색 대상: `outputs/**`, `_bmad_output*/**`, `*.log`, `~/.bash_history`, overlay 디렉터리).

### 4.2 신규 생성 라벨 [실측]

| 항목 | 값 |
|---|---|
| 프레임 | 149 (6 소스에서 균등 샘플) |
| 소스 | SAM_loop1, SAM_loop2, SAM_occlusion, SAM_circle, sam_105314, sam_110633 |
| 덤프된 후보 박스 | 1,007 |
| 박스×객체 쌍 | 10,070 |
| **라벨된 박스** | **338** (MobileSAM까지 도달한 고유 박스 전수) |
| 라벨러 | Claude Vision, 컨택트 시트 12장(150px 타일) + 템플릿 렌더 reference 시트 |
| 저장 | `ism_accuracy_analysis/labels/box_labels.csv` (uid, source, frame, true_class, sheet_idx, labeler, method) |

라벨 분포: saffron 60, Bear 42, milk 41, Febreze 35, Mugcup 24, Rabbit 23, Sikhye 22, **unclear 21**, Sauce 20, **carton(갈색 택배박스) 20**, **choco 14**, Dinosaur 12, other 4.

`unclear`(모션블러·과도한 crop)는 모든 지표에서 제외했다.

### 4.3 데이터 그룹 구성(사용자 §6 요구 대비)

| 요구 그룹 | 확보 여부 | 근거 |
|---|---|---|
| 초코하임 명확 | ✅ 14건 (근/원거리, 회전 포함 — 컨택트 시트 #172~#185) | |
| 초코하임 작게/가림 | ✅ (#183, #185 모션블러·부분) | |
| 인형 각각 명확 | ✅ Bear 42 / Rabbit 23 / Dinosaur 12 | |
| 비슷한 인형 동시 존재 | ✅ sam_105314, SAM_loop2 프레임 | |
| **갈색 택배박스** | ✅ **20건** (GALAX/logi/UN3481 박스) | |
| 유사 직사각형 물체 | ✅ other 4 (다른 제품 상자) | |
| 유사 색 인형 | ✅ Bear(갈색) vs Dinosaur(연두) vs Rabbit(흰색) | |
| BBox에 배경 혼입 | ✅ mask/bbox ratio 로 정량화 | |
| mask 다중 컴포넌트 | ✅ `n_components` 기록 | |
| sem 높고 appe 낮음 / 그 반대 | ✅ 전 쌍 점수 보유 | |
| 두 클래스가 동시에 높음 | ✅ 76~85, 146, 188 등 | |

**미확보**: 조명 밝기 극단 변화 subset(현 bag은 실내 균일 조명), 객체 완전 부재 배경 전용 프레임(YOLO conf 0.02에서 항상 후보가 생겨 사실상 hard negative로 대체됨).

---

## 5. 현재 TP/FP/FN/TN 및 객체별 성능 [실측]

### 5.1 전체 (라벨된 366 결정, 6 bag 전체, 운영 임계 고정)

| | 값 |
|---|---|
| TP | 220 |
| FP | 44 |
| FN | 18 |
| TN | 84 |
| Precision | **0.833** |
| Recall | **0.924** |
| F1 | 0.877 |
| FPR (=FP/(FP+TN)) | 0.344 |

*이 분모는 "semantic 승자로 선택되어 게이트까지 도달한 (박스, 객체) 결정"이다. §6·§18의 ablation은 분모를 (frame, object) 그룹으로 고정한 별도 평가다.*

### 5.2 객체별

| object | TP | FP | FN | TN | Precision | Recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Mugcup_high | 23 | 0 | 1 | 0 | 1.000 | 0.958 | 0.979 |
| Sauce_high | 19 | 1 | 0 | 21 | 0.950 | 1.000 | 0.974 |
| milk | 37 | 0 | 3 | 10 | 1.000 | 0.925 | 0.961 |
| saffron | 35 | 3 | 0 | 21 | 0.921 | 1.000 | 0.959 |
| Sikhye_high | 17 | 0 | 2 | 3 | 1.000 | 0.895 | 0.944 |
| Bear | 36 | 6 | 1 | 0 | 0.857 | 0.973 | 0.911 |
| **choco_hazelnut_high** | 12 | **5** | 2 | 16 | **0.706** | 0.857 | 0.774 |
| Rabbit | 13 | 4 | **6** | 0 | 0.765 | **0.684** | 0.722 |
| **Febreze_high** | 22 | **20** | 2 | 13 | **0.524** | 0.917 | 0.667 |
| **Dinosaur** | 6 | **5** | 1 | 0 | **0.545** | 0.857 | 0.667 |

### 5.3 Confusion matrix (실제 → 예측, FP만)

| 실제 | 예측된 클래스 | 건수 | 비중 |
|---|---|---:|---:|
| **saffron (샤프란 세제통)** | **Febreze_high** | **20** | **45.5%** |
| carton (갈색 택배박스) | choco_hazelnut_high | 5 | 11.4% |
| Bear | Dinosaur | 4 | 9.1% |
| Febreze_high | saffron | 3 | 6.8% |
| Rabbit | Bear | 3 | 6.8% |
| Dinosaur | Bear | 3 | 6.8% |
| Bear | Rabbit | 2 | 4.5% |
| Dinosaur | Rabbit | 2 | 4.5% |
| Rabbit | Dinosaur | 1 | 2.3% |
| Sikhye_high | Sauce_high | 1 | 2.3% |
| **인형 3종 상호 소계** | | **15** | **34.1%** |

---

## 6. 실패 사례 재현 결과

### 사용자 §17.1 요구 표

| 문제 | 재현 여부 | 발생 건수 | 주된 실패 단계 | 근거 |
|---|---|---:|---|---|
| **A. 갈색 택배박스 → 초코하임** | **재현됨** | **5** (전체 FP의 11.4%) | **E7 appearance gate** — 후보 생성·semantic 선택은 정상, appe11 게이트가 통과시킴 | 컨택트 시트 #181/#184/#186/#187/#188. 후보 단계 라벨: 갈색박스 20건 중 5건 수락, 15건은 정상 거절 |
| **B. 실제 초코하임 미검출** | **재현됨** | **2** (게이트 도달분) **+ 상류 FN** | E7 gate 2건(#302, #304). 상류(E1/E2)는 초코하임 라벨 부재로 정량 불가 | choco TP 12 / FN 2 → 조건부 recall 0.857 |
| **C. 인형 간 혼동** | **재현됨, 가장 심각** | **15** (전체 FP의 34.1%) | **E7 gate** — appe11이 인형을 전혀 못 가름(AUROC Rabbit 0.408, Dinosaur 0.429 = 우연 이하) | 시트 #31/#33/#34/#36/#40/#41(Bear 슬롯에 토끼·공룡), #42~#45(Dinosaur 슬롯에 곰·토끼), #52(한 곰이 Dinosaur·Rabbit 양쪽 수락) |
| **D. [추가 발견] 샤프란 세제통 → Febreze 오인** | **재현됨, 최대 FP원** | **20** (전체 FP의 45.5%) | **E7 gate + E11 negative 부재** — 프롬프트 "febreze spray bottle"이 샤프란 통에 발화하고 ISM이 거절하지 못함 | 시트 #62~#67, #72~#83 (샤프란 통이 Febreze로 수락). 역방향도 3건 |
| **E. [추가 발견] 한 박스가 2개 클래스에 동시 수락** | **재현됨** | 최소 11 박스 | **E11 negative rejection 부재** (동일 박스, 서로 다른 클래스가 각자 절대 임계 통과) | 시트 #52, #76~#85, #146, #188 — `claims` 필드에 `ACC:X\|ACC:Y` |
| **F. [추가 발견] 초코하임 FP는 threshold로 못 막음** | 확인됨 | — | 갈색박스 median appe11 0.539 vs 진짜 초코하임 0.624 — 분포 겹침 | §8 표 |

---

## 7. 단계별 오류 원인 분석

### 7.1 오류 단계 귀속 [실측, (frame,object) 78 eval 그룹]

| 코드 | 단계 | 관측 건수 | 비고 |
|---|---|---:|---|
| E1 | YOLO 후보 생성 실패 | **본 분모 밖** | 복구 라벨로 별도 측정: milk FN 294건 중 **179건(60.9%)** |
| E2 | 실제 후보가 top_k 밖 탈락 | 0 관측 | top_k=3, 후보 중앙 1~2개라 절단이 거의 발생 안 함 |
| E3 | semantic 선택 실패 | **0** | [한계] 라벨이 "승자 박스"에 한정되어 비승자 후보의 정체를 모른다 → **측정 불가**로 보아야 하며 "0건"으로 읽으면 안 된다 |
| E4 | MobileSAM mask 실패 | 0 | 라벨된 338박스 전부 mask 생성 성공. mask/bbox 비율 중앙 0.62 |
| E5 | mask→patch 추출 실패 | 0 | 유효 patch 수 중앙 88.5, 최소 12 |
| E6 | 잘못된 template view 선택 | 다수(비치명적) | CLS-top1 = appe-argmax 일치율 42.4%, 점수 손실 평균 +0.015 (별도 조사) |
| **E7** | **appearance gate 실패** | **FP 10 + FN 6 (운영 임계)** | **관측된 모든 FP·FN의 발생 지점** |
| E8 | class 간 score calibration 실패 | 전 FP의 근인 | 같은 박스에서 두 클래스가 동시 통과(§6-E) |
| E9 | 전역 threshold 실패 | 부분적 | 객체별 오버라이드가 이미 존재하나 bag 간 일반화 실패(§16) |
| E10 | 객체별 슬롯 경쟁 | 미측정 | 본 조사는 프롬프트별 독립 패스(운영) 사용 → 슬롯 경쟁 없음 |
| **E11** | **negative rejection 구조 부재** | **구조적, 전 FP에 관여** | closed-set 절대 점수(§3) |

**핵심**: 관측 가능한 FP·FN은 **전부 E7(게이트)에서 발생**하지만, **왜** 게이트가 실패하는지의 근인은 **E11(negative 증거 부재)** 와 **E8(클래스 간 미보정)** 이다. 게이트 값을 어떻게 옮겨도 두 분포가 겹치면 해결되지 않는다는 것은 §8이 보여준다.

### 7.2 특징별 AUROC (후보쌍 전체, positive=해당 객체, negative=다른 정체) [실측]

| 특징 | 평균 | 최저 | Bear | Rabbit | Dinosaur | choco | Febreze | saffron | milk |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| sem_top1 | 0.926 | 0.686 | 0.910 | 0.895 | 0.686 | 0.922 | 0.910 | 0.970 | 0.999 |
| **sem_top5 (운영)** | 0.923 | **0.657** | 0.905 | 0.895 | **0.657** | 0.922 | 0.916 | 0.971 | 0.999 |
| sem_median | 0.953 | 0.857 | 0.860 | 0.934 | 0.971 | 0.973 | 0.857 | 0.974 | 0.999 |
| **sem_mean (42장 전체)** | **0.958** | **0.873** | 0.874 | 0.921 | **0.971** | **0.990** | 0.873 | 0.984 | 0.999 |
| sem_margin12 | 0.514 | 0.311 | 0.631 | 0.507 | 0.486 | 0.626 | 0.311 | 0.385 | 0.629 |
| **appe11_clstop1 (운영 게이트)** | 0.852 | **0.408** | 0.865 | **0.408** | **0.429** | 0.895 | 0.937 | 1.000 | 0.995 |
| appe11_max42 | 0.855 | 0.447 | 0.838 | 0.447 | 0.457 | 0.884 | 0.943 | 1.000 | 0.997 |
| appe9_max42 | 0.894 | 0.592 | 0.845 | 0.592 | 0.743 | 0.925 | 0.974 | 0.972 | 0.993 |
| **appe2_max42 (블록2)** | 0.891 | **0.204** | **1.000** | **1.000** | **1.000** | **0.204** | 0.985 | 0.985 | 0.970 |
| **color_hsv_masked** | 0.904 | 0.518 | 0.991 | 0.987 | **1.000** | 0.915 | 0.936 | 0.894 | 0.939 |
| color_hsv_bbox (배경 포함) | 0.833 | 0.455 | 0.455 | 0.934 | 1.000 | 0.881 | 0.926 | 0.849 | 0.915 |
| yolo_conf | 0.760 | 0.412 | 0.946 | 0.921 | 0.914 | 0.412 | 0.650 | 0.929 | 0.722 |

읽는 법:
* **운영 게이트(appe11_clstop1)는 인형에서 무작위보다 나쁘다**(Rabbit 0.408, Dinosaur 0.429). 인형 혼동 15건의 직접 원인.
* **블록2는 인형을 완벽히 가르지만(1.000×3) 초코하임에서 0.204로 붕괴**한다.
* **masked color는 배경을 뺀 mask 내부에서 계산할 때만 유효**하다(Bear 0.991 vs bbox 전체 0.455). §10의 "배경 영향" 질문에 대한 답.

---

## 8. 갈색 박스 → 초코하임 FP 분석 [실측]

초코하임 후보 중 **진짜 초코하임 14 vs 갈색 택배박스 17** (라벨 기준):

| 특징 | AUROC (choco vs carton) | choco 중앙값 | carton 중앙값 | 판정 |
|---|---:|---:|---:|---|
| **sem_mean (42 평균)** | **0.992** | 0.391 | 0.215 | **최강** |
| sem_median | 0.983 | 0.389 | 0.195 | 강 |
| appe9_max42 | 0.920 | 0.672 | 0.605 | 중 |
| color_hsv_bbox | 0.920 | 0.571 | 0.436 | 중 |
| **sem_top5 (운영 선택 점수)** | 0.920 | 0.495 | 0.390 | 중 |
| **color_hsv_masked** | **0.912** | 0.579 | 0.416 | 중 |
| appe11_mean42 | 0.916 | 0.527 | 0.444 | 중 |
| **appe11_clstop1 (운영 게이트)** | **0.878** | **0.624** | **0.539** | **약 — 분포 겹침** |
| appe11_max42 | 0.870 | 0.638 | 0.543 | 약 |
| yolo_conf | 0.374 | 0.040 | 0.064 | **역전**(택배박스가 더 높은 conf) |
| **appe2_max42 (블록2)** | **0.088** | 0.695 | **0.720** | **완전 역전 — 유해** |

**결론 3가지**:

1. **YOLO 단계 문제가 아니다(§13.1 답).** 프롬프트 'maroon box'는 갈색박스에 **더 높은** conf를 준다(AUROC 0.374 = 역전). 그러나 후단 ISM은 이를 거절할 능력이 **있다** — `sem_mean`이 AUROC 0.992로 거의 완전 분리한다. 즉 **프롬프트 수정보다 ISM 점수 구조 수정이 먼저다.**
2. **운영 게이트(appe11_clstop1)는 이 문제를 풀 수 없다.** 진짜 0.624 vs 가짜 0.539로 분포가 겹치고, 3회 롤백된 "choco 0.59 게이트" 이력이 그 실증이다. threshold 이동으로는 해결 불가.
3. **블록2를 게이트에 넣으면 이 문제가 악화된다.** 택배박스 중앙값(0.720)이 진짜 초코하임(0.695)보다 높다. 초코하임 상자와 크래프트 판지는 **저수준 색·텍스처가 실제로 더 비슷**하고(둘 다 갈색·무광·평면), 구분 정보는 인쇄된 로고/문양이라는 **고수준 신호**에 있다. 블록2는 그 신호를 갖고 있지 않다.

---

## 9. 실제 초코하임 FN 분석

### 9.1 게이트 도달분 [실측]

초코하임 라벨 14건 중 TP 12 / **FN 2**(#302, #304 — 둘 다 원거리·소형·측면). 두 건 모두 **E7(appe 게이트)**. semantic·mask·patch 추출은 정상이었다.
FN 2건의 appe11_clstop1 = 0.517 / 0.532 (게이트 0.55). 진짜 초코하임 중앙값 0.624와의 차이는 **거리/블러에 의한 특징 열화**이지 정체 오류가 아니다.

### 9.2 상류 FN — 복구된 milk 라벨의 증언 [복구]

초코하임 상류 라벨은 없으나, 동일 파이프라인의 milk 1,241행이 FN 구조를 보여준다.

| fn_reason | 건수 | FN 294 중 비중 |
|---|---:|---:|
| **yolo_proposal_miss** | **179** | **60.9%** |
| dino_cls_threshold_too_strict | 94 | 32.0% |
| too_small_far | 19 | 6.5% |
| occlusion | 8 | 2.7% |

**FN의 61%는 ISM이 손댈 수 없는 곳(YOLO 후보 생성)에서 발생한다.** 본 조사의 모든 점수 개선안은 나머지 32%(semantic 임계)와 게이트 단계에만 작용한다. **정확도 로드맵에서 FN을 근본적으로 줄이려면 proposal 단계(프롬프트 앙상블·imgsz·conf) 작업이 필요하다** — 이번 권장안의 범위를 넘어선다.

---

## 10. 인형 간 confusion 분석 [실측]

각 인형 후보에서 **다른 인형만** negative로 둔 AUROC:

| object | n_pos | n_neg | sem_top5 | **appe11_clstop1 (운영)** | appe9_max42 | **appe2_max42** | **color_hsv_masked** |
|---|---:|---:|---:|---:|---:|---:|---:|
| Bear | 37 | 6 | 0.905 | 0.865 | 0.845 | **1.000** | **0.991** |
| Rabbit | 19 | 4 | 0.895 | **0.408** | 0.592 | **1.000** | **0.987** |
| Dinosaur | 7 | 5 | 0.657 | **0.429** | 0.743 | **1.000** | **1.000** |

**진단**: 인형 혼동은 "국소 특징이 전역 평균에 희석되어서"가 아니다. **색이 완전한 해답인데 현재 게이트가 색을 보지 않기 때문**이다. 흰 토끼·연두 공룡·갈색 곰은 색만으로 100% 분리된다(블록2 1.000, masked HSV 0.987~1.000). 그런데 운영 게이트가 쓰는 블록11 patch는 "둥글고 무광인 봉제 질감"만 보고 있어 무작위 이하다.

**그런데 이 문제도 class margin으로 해결된다**(§18): 곰 crop에 대해 Bear의 점수가 Rabbit/Dinosaur보다 높기만 하면 되고, 실제로 그렇다. 색을 새 특징으로 넣지 않아도 **경쟁 클래스 점수를 비교하는 것만으로** eval bag 인형 혼동이 5 → 0이 되었다.

---

## 11. 추가 발견 정확도 문제

1. **샤프란 → Febreze 20건 (최대 FP원).** 프롬프트 'febreze spray bottle'이 샤프란 액체세제 통에 발화하고, Febreze의 appe11 게이트가 이를 통과시킨다. 두 물체는 색(파랑 vs 크림)·형태(분무기 vs 손잡이 통)가 명확히 다른데도 통과한다 — appe11 AUROC 0.937은 "전체 negative" 대비 값이고, **샤프란만을 negative로 좁히면 훨씬 낮다**. 역방향(Febreze → saffron) 3건도 있어 **양방향 혼동**이다.
2. **한 박스가 두 클래스에 동시 수락(≥11박스).** closed-set 구조의 직접 증거. cross-object NMS는 IoU>0.5일 때만 개입하므로 동일 박스여도 서로 다른 프레임 처리 경로(`recognize()` 객체 루프)에서 각자 수락된다.
3. **YOLO conf가 초코하임에서 역전(AUROC 0.374).** 프롬프트 신뢰도를 랭킹에 쓰면 안 된다는 기존 `--nms-rank appe` 결정을 재확인.
4. **`sem_margin12`(템플릿 1위−2위 마진)은 쓸모없다(평균 AUROC 0.514).** 뷰 간 마진은 정체 정보가 아니다 — "view margin" 계열 아이디어(B8: AUROC 기반 게이트 P0.750 R0.600)는 실측으로 기각.

---

## 12. DINOv2 block2 주장 검증 [실측]

사용자 §9의 5개 질문에 순서대로 답한다.

**Q1. block2 token에 색/텍스처 정보가 있는가?** → **있다.** 인형 3종 AUROC 1.000/1.000/1.000은 색 없이는 불가능하다.

**Q2. 현재 ISM이 그 정보를 score 계산에 쓰는가?** → **부분적으로만.**
* `appe_blocks: [11]` → **최종 수락 게이트는 블록2를 전혀 보지 않는다.**
* `nms_rank_blocks: [2, 9]` → **cross-object NMS 승자 결정에만** 쓰인다.
즉 블록2는 "두 라벨이 같은 박스를 주장할 때 누가 이기나"에만 관여하고, **"이 박스를 수락할 것인가"에는 관여하지 않는다.**

**Q3. 현재 aggregation이 그 정보를 보존하는가?** → 랭킹 경로에서는 보존된다(블록2·9 평균). 게이트 경로에는 아예 들어오지 않는다.

**Q4. 현재 score가 그 정보로 초코하임과 갈색박스를 구분하는가?** → **아니다. 그리고 넣으면 더 나빠진다.** 블록2 AUROC 0.204(택배박스가 더 높음). 랭킹에 블록2가 들어가 있다는 사실은 **초코하임 vs 택배박스 경합에서 택배박스를 밀어주고 있을 가능성**을 뜻한다 — 별도 검증이 필요한 신규 리스크다.

**Q5. 명시적 color descriptor를 추가하면 독립적 추가 변별력이 생기는가?** → **초코하임에 한해 생긴다.**

| 비교 | choco vs carton AUROC |
|---|---|
| block2 masked patch | **0.204** |
| masked HSV histogram | **0.912** |

두 특징의 Spearman 상관은 **0.386**으로 낮고, eval bag 오류 집합도 다르다(블록2 오류 28건 / color 오류 25건, 교집합은 §18.2 참조). **"block2가 color를 대체한다"는 이전 보고서의 결론은 초코하임 케이스에서 명백히 반증된다.**

**block2 최종 판정**: 인형 계열에는 최상의 특징(AUROC 1.000)이지만 초코하임 계열에는 유해(0.204)한 **객체 의존적 특징**이다. 게이트 전면 도입은 금지, 현행처럼 **랭킹 한정**으로 두되 **초코하임 vs 택배박스 경합에서의 영향을 별도 검증**해야 한다.

---

## 13. Explicit Color 특징 실험 [실측]

### 13.1 masked vs bbox (배경 영향)

| object | color_hsv_**masked** AUROC | color_hsv_**bbox** AUROC | 차이 |
|---|---:|---:|---:|
| Bear | 0.991 | 0.455 | **−0.536** |
| Rabbit | 0.987 | 0.934 | −0.053 |
| saffron | 0.894 | 0.849 | −0.045 |
| milk | 0.939 | 0.915 | −0.024 |
| choco | 0.915 | 0.881 | −0.034 |
| Sauce | 0.518 | 0.533 | +0.015 |

**색 특징은 반드시 mask 내부에서 계산해야 한다.** Bear의 경우 bbox 전체 색은 무작위 수준(0.455)으로 무너진다 — 책상·벽 배경이 crop의 절반을 차지하기 때문이다. **MobileSAM mask는 색 특징을 쓸 수 있게 만드는 전제조건**이다.

### 13.2 색공간 비교

* `color_hsv_masked`(H16×S8 histogram intersection): 평균 AUROC **0.904**
* `color_labmom_dist`(Lab 채널별 mean/std/skew의 L2거리, 방향 반전 보정 후): 평균 **0.668**, 최고 Rabbit 0.987, 초코 0.853

→ **histogram이 moments보다 일관되게 낫다.** Lab moments는 3채널 9차원으로 압축이 과해 조명/노출 변화가 mean에 직접 실린다. **HSV histogram 채택, moments 기각.**

### 13.3 결정 수준 실험 (분모 고정, eval bags)

| 실험 | 게이트 | P | R | FP | FN |
|---|---|---:|---:|---:|---:|
| B0 (운영, 고정 임계) | appe11 | 0.792 | 0.864 | 10 | 6 |
| **B4** | masked HSV color 단독 | 0.750 | 0.750 | 11 | 11 |
| **B4b** | (appe11 + color)/2 | 0.889 | 0.727 | 4 | 12 |
| **B13** | (sem_mean + color + margin)/3 | 0.830 | **1.000** | 9 | **0** |
| **B15 (권장)** | (sem_mean + margin + appe11)/3 | **0.930** | 0.909 | **3** | 4 |
| **B17** | B15 + color(초코하임 전용) | 0.930 | 0.909 | **3** | 4 |
| B19 | B15 + color(초코+인형 전용) | 0.875 | 0.955 | 6 | 2 |

**핵심 관측**: **B15 → B17(초코하임 전용 color 추가)은 결과가 완전히 동일하다(FP 3→3, FN 4→4).** class margin이 이미 같은 결정을 내리고 있기 때문에 color의 기여분이 0이다.

**해석**: color가 갈색박스를 걸러내는 원리는 "이 crop의 색이 초코하임 템플릿과 다르다"이고, class margin이 걸러내는 원리는 "이 crop은 초코하임보다 다른 무엇에 더 잘 맞는다"이다. **후자가 전자를 포함한다** — 색이 다르면 다른 클래스 점수가 더 높게 나오기 때문이다. 그리고 class margin은 **템플릿 렌더-실사 색 domain gap의 영향을 받지 않는다**(양쪽 클래스가 같은 gap을 겪으므로 상쇄). 이것이 color를 채택하지 않는 실질적 이유다.

---

## 14. Texture 및 Local Pattern 특징 실험

| 특징 | 실행 | 결과 |
|---|---|---|
| DINOv2 블록11 masked patch | ✅ | 운영 게이트. 인형 AUROC 0.408~0.865 (실패) |
| DINOv2 블록9 masked patch | ✅ | 평균 0.894, 인형 0.592~0.845 (중간) |
| DINOv2 블록2 masked patch | ✅ | 인형 1.000, 초코 0.204 (양극) |
| 블록2 + 블록11 결합 | ✅ (B6) | P0.917 R0.750 FP3 **FN11** — FP는 줄지만 recall 붕괴 |
| appe view-margin (max42 − mean42) | ✅ (B8) | P0.750 R0.600 — **기각** |
| 42-view max vs CLS-top1 1장 | ✅ | AUROC 0.855 vs 0.852 — 사실상 동일, 게이트 개선 없음 |
| **LBP / uniform LBP / Gabor / HOG / edge orientation** | ❌ **미실행** | 아래 사유 |
| local patch NN / mutual NN / top-percentile / discriminative weighting | ❌ **미실행** | 아래 사유 |

**미실행 사유(정직하게 기록)**: 본 조사의 실측은 (a) 인형 혼동이 **색으로 100% 분리 가능**하고, (b) 그 색조차 class margin으로 대체되며, (c) 초코하임 문제는 저수준 텍스처가 **역전**되는(블록2 0.204) 구조임을 보였다. 즉 **handcrafted texture가 겨냥할 남은 오류가 없다.** 남은 오류는 샤프란↔Febreze(형태·색 모두 다른데 게이트가 못 가름 = 게이트 구조 문제)와 원거리·블러(신호 자체의 열화)다. 근거 없이 특징을 늘리지 말라는 §18 금지사항에 따라 이 계열은 **Phase 5 조건부**로 미룬다. 실행 조건: B15 적용 후에도 인형/포장 혼동이 남을 경우.

---

## 15. Negative Prototype와 Open-set Rejection [실측]

negative bank = calib bag에서 "해당 객체가 아니라고 라벨된" 박스들의 임베딩(patch-mean 384d, masked HSV 128d). 점수 = `positive − max(negative similarity)`.

| 실험 | 게이트 | P | R | FP | FN |
|---|---|---:|---:|---:|---:|
| B0 (운영) | appe11 | 0.792 | 0.864 | 10 | 6 |
| **B3** | appe11 − max neg patch cos | 0.784 | 0.909 | 11 | 4 |
| **B3b** | color − max neg color | 0.778 | 0.795 | 10 | 9 |
| **B2** | class margin (appe11 기준) | 0.894 | 0.955 | 5 | 2 |
| **B2b** | class margin (sem_mean 기준) | 0.863 | **1.000** | 7 | **0** |
| B16 | (margin + neg-color)/2 | 0.784 | 0.909 | 11 | 4 |

**명시적 negative prototype은 class margin에 진다.** 이유는 두 가지다.
1. **데이터 요구량** — negative bank는 "본 적 있는 FP"만 커버한다. calib bag의 택배박스와 eval bag의 택배박스는 다른 상자이고, 일반화가 약하다.
2. **class margin은 공짜 negative bank다** — 시스템이 이미 10개 객체의 템플릿을 갖고 있고, 각 객체가 서로의 negative 역할을 한다. 새 데이터 수집이 필요 없고, 객체가 늘수록 강해진다.

**Open-set rejection 판정**: EVT·energy 같은 정교한 기법은 불필요하다. **class margin + 객체별 임계**의 단순 조합이 eval FP 10 → 3을 달성했다. 캘리브레이션 라벨이 78 그룹 수준인 현 상황에서 그 이상의 복잡도는 과적합만 늘린다.

---

## 16. Class Margin 및 객체별 Threshold [실측]

| 실험 | 설명 | P | R | F1 | FP | FN |
|---|---|---:|---:|---:|---:|---:|
| B0-fixed | 운영 점수 + **운영 임계(손튜닝)** | 0.792 | 0.864 | 0.826 | 10 | 6 |
| **B1** | 운영 점수 + **객체별 임계 재보정** | 0.829 | **0.659** | 0.734 | 6 | **15** |
| B2 | + class margin (appe11) | 0.894 | 0.955 | 0.923 | 5 | 2 |
| B2b | + class margin (sem_mean) | 0.863 | 1.000 | 0.926 | 7 | 0 |

**객체별 threshold 재보정 단독은 실패한다.** calib bag에서 F1을 최대화한 임계가 eval bag에서 recall을 0.864 → 0.659로 떨어뜨린다. 이는 저장소가 세 번(choco 0.59, saffron 0.72, milk 0.63) 롤백한 현상의 재현이며, 원인은 **appe11 점수 분포가 bag(거리·조명·시점)에 따라 이동**하기 때문이다.

**반면 class margin은 bag 간 이동에 강하다.** 두 클래스가 같은 crop·같은 조명·같은 거리를 겪으므로 **분포 이동이 상쇄**된다. 이것이 margin이 절대 점수보다 일반화되는 구조적 이유다.

---

## 17. Multi-view 및 Local Patch Matching [실측]

### 17.1 semantic aggregation (사용자 §13.2)

| aggregation | 평균 AUROC | 최저 | **choco vs carton** |
|---|---:|---:|---:|
| Top-1 | 0.926 | 0.686 | 0.912 |
| Top-3 평균 | 0.919 | 0.643 | 0.887 |
| **Top-5 평균 (운영)** | 0.923 | 0.657 | **0.920** |
| softmax weighted | 0.919 | 0.657 | 0.870 |
| median | 0.953 | 0.857 | 0.983 |
| **mean (42장 전체)** | **0.958** | **0.873** | **0.992** |
| top1−top2 margin | 0.514 | 0.311 | 0.605 |

**사용자 가설 확인**: Top-5 평균은 "가장 잘 맞는 5장"만 보므로, **여러 직사각형 템플릿에 약하게 닮은 negative를 안정적으로 높게 만든다.** 42장 전체 평균은 negative가 "대부분의 view와 안 닮았다"는 사실을 점수에 반영해 갈색박스를 0.215로 눌러 진짜 초코하임 0.391과 벌린다(AUROC 0.992).

**단, 절대값 스케일이 달라지므로 `similarity_threshold=0.35`를 그대로 쓰면 안 된다** — sem_mean은 값이 작아져 사전 게이트가 과도하게 엄격해진다. 재보정 필수(§20 Phase 2 성공조건).

### 17.2 appearance template 범위 (§17.1 별도 조사와 연결)

42-view max는 CLS-top1 1장 대비 AUROC 0.852 → 0.855로 **변별력 개선이 사실상 없다**. 앞선 latency 조사가 "42-view는 게이트가 아니라 랭킹용"이라 결론지은 것과 일치한다. 본 조사도 같은 결론이며, **템플릿 범위 확대는 정확도 문제의 해답이 아니다.**

### 17.3 semantic Top-2 MobileSAM (§13.3)

B9(선택 실패 보정 근사)는 P0.846 R0.750 FP6 FN11로 **개선 없음**. 다만 **E3 자체를 측정할 수 없었으므로(§7.1) 이 결론은 잠정**이다. Top-2 segmentation의 가치를 판정하려면 **비승자 후보에도 라벨이 필요**하다 → Phase 2 과제.

---

## 18. 전체 Ablation 결과 [실측]

### 18.1 (frame, object) 분모 고정, calib 167 그룹 / eval 78 그룹(positive 44)

| ID | 구성 | P | R | F1 | **FP** | **FN** | 갈색→초코 | 인형혼동 | 샤프란→Febreze |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| **B0-fixed** | **운영 (기준선)** | 0.792 | 0.864 | 0.826 | **10** | **6** | 0 | **5** | **5** |
| B0/B1 | 운영 점수 + 객체별 재보정 | 0.829 | 0.659 | 0.734 | 6 | 15 | 0 | 4 | 0 |
| B2 | + class margin (appe11) | 0.894 | 0.955 | 0.923 | 5 | 2 | 0 | 2 | 1 |
| B2b | + class margin (sem_mean) | 0.863 | **1.000** | 0.926 | 7 | **0** | 0 | 3 | 1 |
| B3 | + negative prototype (patch) | 0.784 | 0.909 | 0.842 | 11 | 4 | 0 | 4 | 2 |
| B3b | + negative prototype (color) | 0.778 | 0.795 | 0.786 | 10 | 9 | 1 | 3 | 2 |
| B4 | masked HSV color 단독 | 0.750 | 0.750 | 0.750 | 11 | 11 | 1 | 3 | 2 |
| B4b | (appe11+color)/2 | 0.889 | 0.727 | 0.800 | 4 | 12 | 0 | 2 | 0 |
| B5 | block2 단독 | 0.850 | 0.773 | 0.809 | 6 | 10 | 1 | 2 | 1 |
| B6 | (block11+block2)/2 | 0.917 | 0.750 | 0.825 | 3 | 11 | 0 | 1 | 0 |
| B7 | sem_mean 선택 + appe11 게이트 | 0.829 | 0.659 | 0.734 | 6 | 15 | 0 | 4 | 0 |
| B7b | sem_mean 선택 + sem_mean 게이트 | 0.875 | 0.795 | 0.833 | 5 | 9 | 0 | 2 | 1 |
| B9 | Top-2 완화 근사 | 0.846 | 0.750 | 0.795 | 6 | 11 | 0 | 3 | 1 |
| B12 | sem_mean + (sem_mean+margin)/2 | 0.909 | 0.909 | 0.909 | 4 | 4 | 0 | **0** | 3 |
| B13 | + color 3항 | 0.830 | **1.000** | 0.907 | 9 | **0** | 1 | 3 | 0 |
| B14 | sem_mean + (sem_mean+appe11)/2 | 0.864 | 0.864 | 0.864 | 6 | 6 | 0 | 1 | 1 |
| **B15 (권장)** | **sem_mean 선택 + (sem_mean+margin+appe11)/3** | **0.930** | **0.909** | **0.919** | **3** | **4** | **0** | **0** | **2** |
| B17 | B15 + color(초코 전용) | 0.930 | 0.909 | 0.919 | 3 | 4 | 0 | 0 | 2 |
| B18 | B15 + color(margin 불확실 구간) | 0.872 | 0.932 | 0.901 | 6 | 3 | 0 | 1 | 1 |
| B19 | B15 + color(초코+인형) | 0.875 | 0.955 | 0.913 | 6 | 2 | 1 | 1 | 1 |
| B20 | B15 (appe11을 42-view max로) | 0.875 | 0.955 | 0.913 | 6 | 2 | 0 | 1 | 1 |

**baseline 대비 수정/파손 (B15 vs B0-fixed, eval)**

| | 건수 |
|---|---:|
| baseline 오류 중 **수정된 것** | FP 7건 소멸 + FN 2건 회복 = **9건** |
| baseline 정답 중 **파손된 것** | **0건** (TP 38 → 40, FP 10 → 3, FN 6 → 4) |
| 인형 혼동 | 5 → **0** |
| 갈색박스 → 초코하임 | 1 → **0** |
| 샤프란 → Febreze | 5 → 2 |

### 18.2 특징 간 오류 보완성 (eval bags, 각 특징 단독 게이트 기준)

| 관계 | 값 |
|---|---:|
| appe11 오류 수 | 27 |
| color 오류 수 | 25 |
| block2 오류 수 | 28 |
| **sem_mean 오류 수** | **12** |
| appe11 ∩ color | 7 |
| color가 appe11을 고치는 건수 | 20 |
| color가 새로 깨뜨리는 건수 | 18 |
| block2가 appe11을 고치는 건수 | 19 |
| block2가 새로 깨뜨리는 건수 | 20 |
| **sem_mean이 appe11을 고치는 건수** | **22** |
| **sem_mean이 새로 깨뜨리는 건수** | **7** |

**결정적**: color와 block2는 "고치는 만큼 깨뜨린다"(20:18, 19:20 — 순이득 ~0). **sem_mean만이 순이득이 크게 양수(22:7)** 다. 이것이 권장안의 1축을 sem_mean으로 잡은 근거다.

### 18.3 점수 간 Spearman 상관 (후보쌍 전체)

| 쌍 | ρ |
|---|---:|
| appe11 ~ sem_mean | 0.864 |
| appe11 ~ block2 | 0.664 |
| block2 ~ sem_mean | 0.533 |
| block2 ~ color | 0.386 |
| **color ~ sem_mean** | **0.315** |
| **appe11 ~ color** | **0.302** |

**semantic과 appearance는 사실상 같은 증거다(ρ 0.864)** — 같은 DINOv2 forward의 두 요약이라는 구조적 사실과 일치한다. color만이 진짜로 독립적이다(ρ 0.30). 그런데 그 독립 정보가 순이득으로 이어지지 않는 이유가 §13.3에서 설명한 "class margin이 같은 결정을 이미 내림"이다.

---

## 19. 최우선 권장 ISM 구조 — **B15: 42-view Mean Semantic + Class-Margin Fusion Gate**

### 19.1 데이터 흐름

```
입력 RGB
│
├─[1] YOLO-World 프롬프트별 후보  (변경 없음: score_threshold, top_k=3)
│      사용 특징: YOLO conf (임계·정렬 전용, 랭킹 금지 — 초코하임 AUROC 0.374 역전)
│
├─[2] 후보 crop → DINOv2 배치 forward 1회 (블록 2/9/11 동시, 기존 함수 그대로)
│
├─[3] semantic = **42 템플릿 CLS 코사인의 전체 평균(sem_mean)**   ← 변경 ①
│      · 후보 선택: argmax(sem_mean)  (객체당 1박스, 기존 구조 유지)
│      · 사전 게이트: sem_mean >= similarity_threshold' (**재보정 필수**, 0.35 그대로 쓰면 안 됨)
│
├─[4] MobileSAM — 선택된 박스에만 (변경 없음; mask는 PEM 입력 산출물이라 생략 불가)
│
├─[5] appearance = mask 내부 patch vs CLS-top1 템플릿 (블록11)  (변경 없음)
│
├─[6] **class margin = sem_mean(o) − max_{o'≠o} sem_mean(o')**   ← 변경 ②
│      · 같은 박스에 대한 전 객체 sem_mean 은 [3]에서 이미 계산됨 → 추가 비용 0
│
├─[7] **최종 점수 = (sem_mean + class_margin + appe11) / 3**      ← 변경 ③
│      · 게이트: 객체별 임계 (calib bag 에서 산출, §19.3)
│      · 세 항의 역할: sem_mean=정체, margin=경쟁 배제(암묵적 negative), appe11=국소 검증
│
├─[8] cross-object NMS (변경 없음, rank = 기존 rank_appe)
│
└─[9] accept / reject   (ambiguous 구간은 Phase 3에서 도입 검토)
```

### 19.2 단계별 명세

| 단계 | 사용 특징 | 점수 계산 | 후보 수 | template aggregation | threshold 정책 | class competition | negative evidence | ambiguity |
|---|---|---|---|---|---|---|---|---|
| 3 | CLS(블록11) | 42장 **전체 평균** | 객체당 ≤3 | mean(42) | 객체 공통(재보정) | 없음 | 없음 | — |
| 5 | patch(블록11) | CLS-top1 1장 | 1 | 단일 view | — | 없음 | 없음 | — |
| 6 | CLS(블록11) | 1등 − 2등 클래스 | 1 | mean(42) | — | **10개 객체 전수 비교** | **암묵적** | — |
| 7 | 위 3항 | 단순 평균 | 1 | — | **객체별 calibrated** | 반영됨 | 반영됨 | Phase 3 |

### 19.3 기존 코드에서 바뀌는 부분

| 파일 | 변경 |
|---|---|
| `yolo_ism.py:238-242` `semantic_score()` | `match_topk` 대신 전체 평균 옵션 추가(기본값 유지, 신규 인자) |
| `yolo_ism_object_n.py:307-314` | 선택 점수를 sem_mean으로, 사전 게이트 임계 재보정값 사용 |
| `yolo_ism_object_n.py` `recognize_frame()` | 프레임 내 **전 객체 sem_mean 행렬**을 유지해 class margin 계산(이미 배치로 CLS를 다 갖고 있으므로 추가 forward 0) |
| `configs/yolo_ism_objects.yaml` | `similarity_threshold`·`appe_gate` → 새 `final_gate` 객체별 값으로 대체 |

**추가 모델 호출 0회, 추가 forward 0회.** class margin은 이미 계산된 CLS를 다른 객체 템플릿과 곱하는 matmul(객체당 `[42,384]@[384]`)뿐이다.

---

## 20. 예상 정확도 변화와 단계별 구현 계획

### 20.1 예상 효과 (eval bag 실측, 78 그룹)

| 지표 | 현재(B0-fixed) | 권장(B15) | 변화 |
|---|---:|---:|---|
| Precision | 0.792 | **0.930** | +0.138 |
| Recall | 0.864 | **0.909** | +0.045 |
| F1 | 0.826 | **0.919** | +0.093 |
| FP | 10 | **3** | **−70%** |
| FN | 6 | **4** | **−33%** |
| 인형 상호 혼동 | 5 | **0** | −100% |
| 갈색박스 → 초코하임 | 1 | **0** | −100% |
| 샤프란 → Febreze | 5 | 2 | −60% |
| baseline 정답 파손 | — | **0건** | — |

### 20.2 Phase 계획

**Phase 1 — 로그·라벨 영구화 (선이행 완료 + 확장)**
* 목적: 모든 임계 논의의 전제 확보.
* 완료분: `ism_accuracy_analysis/labels/box_labels.csv` 338박스 + 복구된 `audit_cases.csv` 1,241행.
* 남은 작업: **비승자 후보 박스에도 라벨** 부여(E3 측정 가능화), 조명 변화 subset 추가 수집.
* 신규 파일: `labels/box_labels_extended.csv`.
* 성공 기준: 객체당 positive ≥ 30, negative ≥ 30, bag ≥ 4.
* Rollback: 해당 없음(데이터만 추가).
* 진행 조건: 없음(즉시).

**Phase 2 — sem_mean 전환 + 사전 게이트 재보정 (단독)**
* 목적: 변경 ① 단독 효과 확인. 갈색박스 분리(AUROC 0.920 → 0.992) 실증.
* 수정 예상 파일: `yolo_ism.py`(신규 인자), `yolo_ism_object_n.py`(선택 점수), `configs/yolo_ism_objects.yaml`(`similarity_threshold`).
* 검증 데이터: calib 4 bag 보정 → eval 2 bag 측정 + 4 bag 전체 재현.
* 성공 기준: eval FP 감소 ≥ 30% **AND** recall 감소 0.
* **Rollback 기준**: 어느 객체든 recall이 0.05 이상 감소하거나, `sem_mean` 사전 게이트가 semantic 통과 그룹 수를 30% 넘게 줄이면 중단.
* 진행 조건: Phase 1의 라벨 확장 완료.

**Phase 3 — class margin 도입 + 3항 융합 게이트 (핵심)**
* 목적: 변경 ②③. FP 주원인(closed-set) 제거.
* 수정 예상 파일: `yolo_ism_object_n.py`(`recognize_frame`에 클래스 간 점수 행렬 유지), config.
* 신규 파일: `ism_accuracy_analysis/experiments/calibrate_gate.py`(객체별 임계 산출, bag 분리 강제).
* 검증 데이터: Phase 1 확장 라벨, **calib/eval bag 분리 필수**.
* 성공 기준: eval FP ≤ 4, FN ≤ 6, 인형 혼동 0, **baseline 정답 파손 0건**.
* **Rollback 기준**: baseline TP가 1건이라도 사라지거나, 어느 객체 recall이 0.05 이상 하락.
* 진행 조건: Phase 2 성공.

**Phase 4 — color / block2 재평가 (조건부)**
* 목적: Phase 3 이후 **남은** 오류에 대해서만 color·block2의 순이득 재측정.
* 현 실측: B15 → B17(초코 전용 color)의 순이득 **0**. 따라서 **기본은 미도입**.
* 실행 조건: Phase 3 후 잔여 FP 중 색으로 설명되는 코호트가 5건 이상.
* 성공 기준: 순이득(고친 건수 − 깨뜨린 건수) > 0, eval bag 기준.
* Rollback: 순이득 ≤ 0이면 즉시 폐기(현재 block2/color 모두 순이득 ~0이었음).

**Phase 5 — local patch / multi-view / Top-2 segmentation (조건부)**
* 목적: E3(선택 실패)와 국소 판별 문제.
* **진행 조건: Phase 1 확장 라벨로 E3가 실제로 측정되어 5건 이상일 때만.** 현재는 측정 불가라 착수 근거가 없다.

**Phase 6 — 최종 결합 + 상류 FN(YOLO proposal) 작업**
* milk 라벨이 보여준 FN의 61%는 proposal 단계다. ISM 점수 개선을 마친 뒤 프롬프트 앙상블·imgsz·conf를 별도 조사로 다룬다.

---

## 21. 위험 요소와 Rollback 기준

| # | 위험 | 징후 | Rollback 조건 |
|---|---|---|---|
| R1 | **표본 부족** — eval 78 그룹(positive 44). FP 10 → 3은 7건 차이로, 이항 신뢰구간이 넓다 | bag을 바꾸면 순위가 뒤집힘 | Phase 3 검증을 **최소 4개 eval bag**으로 반복, 일관되지 않으면 보류 |
| R2 | **라벨러가 모델(Claude Vision)** — 사람 검수가 아니다 | 사람과 불일치 | 무작위 50건 사람 재검수, 불일치 > 5%면 라벨 재작성 |
| R3 | **class margin이 "그 물체가 등록 객체 중 하나"라고 가정** — 완전 미등록 물체는 margin이 무의미 | 미등록 물체 FP 증가 | 절대 점수 항(sem_mean)을 3항에 남겨 둔 이유. 제거하지 말 것 |
| R4 | sem_mean 전환 시 사전 게이트 스케일 변화 | semantic 통과 그룹 수 급감 | 통과 그룹 수 −30% 초과 시 중단(Phase 2 기준) |
| R5 | 객체별 임계의 bag 간 비일반화 (B1이 실증) | calib에서 좋고 eval에서 recall 붕괴 | 임계는 **반드시 다른 bag**에서 검증. 같은 bag 보정·보고 금지 |
| R6 | block2가 랭킹(`nms_rank_blocks=[2,9]`)에 있어 **초코하임 vs 택배박스 경합에서 택배박스를 밀어줄 가능성** | 초코 라벨이 carton 쪽으로 넘어감 | 신규 리스크. Phase 2에서 랭킹 영향 별도 측정 |
| R7 | `unclear` 21건 제외로 어려운 케이스가 지표에서 빠짐 | 실제 운영 성능이 보고보다 낮음 | 보고서 수치는 "판독 가능한 케이스 기준"으로 해석 |
| R8 | 초코하임 positive 14건 — 객체별 결론의 통계력 부족 | — | Phase 1에서 초코하임 프레임 집중 수집 |

---

## 22. 최종 결론

```text
현재 가장 큰 정확도 문제:
사용자가 지목한 갈색 택배박스(FP의 11%)가 아니라, 각 객체가 다른 객체의 점수를 보지
않는 closed-set 절대 점수 구조 자체이며, 그 결과 가장 큰 FP원은 샤프란 세제통 →
Febreze 오인 20건(전체 FP 44건의 45%)과 인형 3종 상호 혼동 15건(34%)이다.

갈색 박스 → 초코하임 문제의 주원인:
YOLO 프롬프트가 아니라(프롬프트 conf는 오히려 택배박스가 더 높다, AUROC 0.374 역전)
semantic Top-5 평균이 "가장 잘 맞는 5개 view"만 보아 다수 view에 약하게 닮은 택배박스를
띄우고, 최종 게이트가 쓰는 appe11(CLS-top1 단일 view)이 진짜 0.624 vs 가짜 0.539로
분포가 겹쳐 어떤 threshold로도 분리되지 않는 것이다 — 42장 전체 평균으로 바꾸면 같은
데이터에서 AUROC 0.920 → 0.992로 거의 완전 분리된다.

실제 초코하임 FN의 주원인:
게이트 도달분에서는 원거리·소형·측면에 의한 특징 열화로 appe11이 0.517/0.532로 게이트
0.55를 못 넘긴 E7 2건이며, 더 큰 몫은 게이트 이전 단계 — 복구된 milk 1,241행 라벨 기준
FN의 60.9%가 YOLO가 후보를 아예 만들지 못한 proposal miss로, ISM 점수 개선으로는
손댈 수 없는 영역이다.

인형 간 혼동의 주원인:
운영 게이트가 쓰는 블록11 masked patch가 봉제 질감만 보고 색을 보지 않아 AUROC가
Rabbit 0.408 / Dinosaur 0.429로 우연보다 나쁘기 때문이며(같은 데이터에서 색 기반
특징은 0.99~1.00으로 완전 분리), 그런데도 각 인형이 서로의 점수를 보지 않고 독립적으로
절대 임계만 통과하면 수락되는 구조가 이를 그대로 통과시킨다.

Color 특징 판정:
E — 최종 score의 특징으로 사용하지 않는다. masked HSV color 자체는 강력하지만
(인형 AUROC 0.987~1.000, 초코하임 vs 택배박스 0.912, 반면 block2는 0.204),
class margin을 도입하면 그 기여가 완전히 사라진다(B15 → B17에서 FP 3→3, FN 4→4로
결과 동일). class margin이 "색이 다르면 다른 클래스 점수가 더 높다"는 형태로 같은
정보를 이미 사용하며, 렌더-실사 색 domain gap의 영향까지 상쇄한다. 단 class margin을
채택하지 않는 시나리오에 한해서는 D(hard-negative rejection 전용)로 유효하다.
단, 색 특징은 반드시 mask 내부에서 계산해야 한다(Bear: masked 0.991 vs bbox 0.455).

DINOv2 block2 판정:
"block2가 color를 이미 담고 있으니 명시적 color는 불필요"라는 이전 결론은 반증되었다.
block2는 인형 3종에서 AUROC 1.000으로 완벽하지만 초코하임 vs 갈색택배박스에서 0.204로
완전히 역전된다(택배박스를 진짜보다 높게 평가). 즉 block2 ≠ explicit color이며 객체
의존적이다. 현재 코드에서 block2는 최종 게이트에는 전혀 쓰이지 않고 cross-object NMS
랭킹에만 쓰인다(appe_blocks=[11], nms_rank_blocks=[2,9]). 게이트 도입은 금지하고
(B6: FP 3이지만 FN 11로 recall 붕괴), 랭킹 한정 현행 유지하되 초코하임 vs 택배박스
경합에서 택배박스를 밀어줄 가능성을 별도 검증해야 한다.

최우선 권장안:
B15 — 42-view Mean Semantic + Class-Margin Fusion Gate

권장 데이터 흐름:
RGB → YOLO 프롬프트별 후보(top_k=3) → DINOv2 배치 forward 1회(블록2/9/11) →
semantic = 42템플릿 CLS 코사인 전체 평균(sem_mean) → 객체당 argmax(sem_mean) 1박스
선택 + 재보정된 사전 게이트 → MobileSAM(선택 박스) → mask 내부 patch vs CLS-top1
템플릿 = appe11 → class_margin = sem_mean(o) − max(다른 9객체 sem_mean) →
최종 점수 = (sem_mean + class_margin + appe11)/3 → 객체별 calibrated 임계 →
cross-object NMS → accept / reject

핵심 변경:
1. semantic 점수를 상위 5개 평균에서 42템플릿 전체 평균으로 교체 (선택과 사전 게이트
   모두, similarity_threshold 재보정 필수)
2. 같은 박스에 대한 10개 객체의 sem_mean 을 비교하는 class margin 항 신설 —
   추가 모델 호출 0회, 이미 계산된 CLS의 matmul만 추가
3. 최종 수락을 appe11 단일 절대 임계에서 (sem_mean + class_margin + appe11)/3 의
   객체별 calibrated 임계로 교체

예상 정확도 효과:
- FP: 10 → 3 (−70%), Precision 0.792 → 0.930
- FN: 6 → 4 (−33%), Recall 0.864 → 0.909, baseline 정답 파손 0건
- 객체 간 confusion: 인형 상호 혼동 5 → 0, 갈색박스→초코하임 1 → 0,
  샤프란→Febreze 5 → 2

가장 큰 위험:
평가 표본이 eval 78 그룹(positive 44)으로 작아 FP 10 → 3(7건 차이)의 신뢰구간이 넓고,
라벨러가 사람이 아니라 모델(Claude Vision)이라는 점이다. 또한 class margin은 "관측된
물체가 등록 10객체 중 하나"라는 가정에 기대므로, 완전 미등록 물체에 대해서는 무력하다 —
그래서 절대 점수항(sem_mean)을 3항 안에 반드시 남겨야 한다.

첫 번째 구현 실험:
Phase 2 단독 — semantic_score 를 42템플릿 전체 평균으로 바꾸고 similarity_threshold 만
재보정한 뒤, calib 4 bag(sam_105314/SAM_loop2/SAM_circle/SAM_occlusion)에서 임계를
정하고 eval 2 bag(sam_110633/SAM_loop1)에서만 측정한다. 성공 기준 = FP 30% 이상 감소
AND recall 감소 0. 어느 객체든 recall 0.05 이상 하락하면 즉시 원복.

다음 권장 BMAD 명령:
1) /bmad-spec — Phase 2·3을 "sem_mean 전환 + class margin 융합 게이트" 단일 SPEC 으로
   고정 (수정 파일, 성공/rollback 기준, 검증 프로토콜을 기계 계약으로 명문화)
2) /bmad-quick-dev — 그 SPEC 으로 Phase 2 구현 후 calib/eval 분리 검증
3) /bmad-technical-research — 별건으로 "YOLO proposal miss(FN의 61%) 감축" 조사
   (프롬프트 앙상블 / imgsz / conf / negative prompt), 이번 범위 밖이지만 FN 총량의
   최대 기여원
```

---

## 부록 A. 재현 방법 (격리 폴더)

```
sam6d_ws/ism_accuracy_analysis/          ← 이 폴더만 삭제하면 원상복구
  probes/dump_candidates.py              # 후보 박스 전수 특징 덤프 (1,007 boxes / 10,070 pairs)
  probes/dump_box_embeddings.py          # negative prototype 용 임베딩 재추출
  probes/make_sheets.py                  # reference / contact sheet 생성
  labels/box_labels.csv                  # 338 박스 라벨 (provenance 포함) ★영구 자산
  features/{boxes.csv,pairs.csv,box_emb.npz,tplcolor/}
  datasets/{crops/,sheets/}              # 라벨링 근거 이미지 (컨택트 시트 12장)
  experiments/analyze.py                 # B0 성능 / AUROC / choco·인형 집중분석
  experiments/ablation.py                # 결정 단위 ablation (분모 가변 — 참고용)
  experiments/ablation2.py               # (frame,object) 분모 고정 정식 ablation ★
  results/{auroc_by_feature.csv, baseline_and_focus.json,
           ablation.json, ablation_frameobject.json}
```

실행: `/home/ldh9501/miniconda3/envs/sam_yolo/bin/python ism_accuracy_analysis/<경로>`
운영 파일 수정 없음 / config 변경 없음 / git 조작 없음.

## 부록 B. 한계 (정직하게)

* **라벨러가 사람이 아니다.** Claude Vision이 컨택트 시트를 보고 라벨했다. 저장소에 선례(`tools/e2e_pipeline/_write_vlm_labels.py`)가 있으나 사람 검수로 대체되어야 한다.
* **E3(semantic 선택 실패)는 측정하지 못했다.** 라벨이 승자 박스에 한정되어, 표에 나온 "0건"은 **부재의 증거가 아니라 증거의 부재**다.
* **eval 78 그룹(positive 44)** — 객체별 결론(특히 초코하임 positive 14, Dinosaur 12)은 경향 이상으로 읽으면 안 된다.
* **조명 극단 변화 subset 부재** — §10의 "조명 변화에서 color 안정성" 질문은 현 데이터로 답하지 못했다. color를 E로 판정한 근거는 조명이 아니라 class margin과의 중복이다.
* **latency는 측정하지 않았다**(본 조사 범위 밖). 권장안의 추가 연산은 객체당 `[42,384]@[384]` matmul 9회뿐이라 무시 가능 수준으로 **추정**된다.
* **`unclear` 21건 제외** — 모션블러 등 어려운 케이스가 지표에서 빠져 있어, 실제 운영 성능은 보고 수치보다 낮을 수 있다.

---

## 다음 액션 제안

1. **[권장] `/bmad-spec`으로 Phase 2+3 고정** — "sem_mean 전환 + class margin 융합 게이트"를 수정 파일·성공/rollback 기준·검증 프로토콜까지 기계 계약으로 명문화한 뒤 구현.
2. **Phase 1 라벨 확장 먼저** — 비승자 후보까지 라벨해 E3를 측정 가능하게 만들고, 초코하임 프레임을 집중 수집(현 positive 14건은 통계력 부족).
3. **사람 검수 50건 샘플** — 모델 라벨의 신뢰도를 확정(R2 위험 해소). 불일치 5% 초과 시 라벨 재작성.
4. **별건 조사 착수** — FN 총량의 61%를 차지하는 YOLO proposal miss (프롬프트 앙상블 / imgsz / conf / negative prompt).
5. **분석 폴더 처리** — `ism_accuracy_analysis/`에서 `labels/box_labels.csv`만 저장소 자산으로 승격하고 나머지는 삭제할지 결정.

어느 방향으로 진행할까요?
