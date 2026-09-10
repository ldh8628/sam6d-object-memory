---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments:
  - 'sam6d_ws/ism_accuracy_observation/ (본 조사 전수 덤프·라벨·결과)'
  - '_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-ism-accuracy-fp-fn-color-discriminability-research-2026-07-20.md'
  - '_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-ism-discriminability-latency-methodology-research-2026-07-20.md'
workflowType: 'research'
lastStep: 1
research_type: 'technical'
research_topic: 'SAM 6개 데이터 전수 관찰 — 클래스 Top-3 경쟁 / HSV 일반성 / YOLO proposal miss'
research_goals: '운영 baseline 고정 상태에서 세 현상을 독립 측정하고 근거 데이터를 영구 저장, 후속 실험 필요성만 판단'
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

## 1. Executive Summary

운영 baseline을 한 줄도 바꾸지 않고 **SAM 6개 데이터 2,596 프레임 전수**를 관찰했다. 결론 셋:

**① 클래스 경쟁은 실재하며, 6개 데이터 전부에서 일관되게 나타난다.**
동일 물리 후보(proposal group)에 **2개 이상 클래스가 동시에 최종 수락된 비율 8.78%**(1,098/12,504), 3개 이상 189건. IoU 임계를 0.3/0.5/0.7로 흔들어도 9.19%/8.78%/8.60%로 거의 변하지 않아 grouping 방식에 강건하다. 데이터셋별 5.3~11.0%로 전부 발생. 최다 쌍은 **Febreze+saffron 597건(54%)**, 인형 3쌍 합 776건(71%), choco+milk 65건. [실측]

**② 그런데 정답은 거의 항상 상위권에 살아 있다.** 라벨된 350 proposal group에서 정답 클래스가 appearance 기준 Top-1 91.1%, Top-2 95.7%, Top-3 97.1%. **오답이 Top-1을 차지했지만 정답이 Top-2/3에 있는 사례가 21건(6.0%)** — Top-1만 취하는 현 구조가 그만큼을 버리고 있다. 결정적으로 **Top1−Top2 gap이 정답Top1일 때 median 0.188, 오답Top1일 때 median 0.0217로 8.7배 차이**난다. class competition은 "필요한가"가 아니라 "신호가 이미 거기 있다"가 답이다. [실측]

**③ HSV는 초코하임 전용이 아니라 전 객체에서 유효하다 — 단 조건이 하나 붙는다.**
결정적 구분은 **레퍼런스를 무엇으로 만드느냐**다.
* **렌더 템플릿 기반 HSV**: 객체별 AUROC 0.584(Bear)~1.000, 평균 0.865 — 렌더-실사 color domain gap 때문에 객체마다 들쭉날쭉.
* **실사 crop prototype 기반 HSV (leave-one-dataset-out)**: **전 10객체 0.929~1.000**, 최악이 saffron 0.929. 인형 3종은 모두 **1.000**인데 같은 조건에서 운영 게이트(appe11)는 Bear 0.579 / Dinosaur 0.472 / Rabbit 0.548로 **무작위 수준**이다.
오프라인 fusion(6-fold LODO): 운영 고정 게이트 P0.726/R0.942(FP 104) → **appearance+HSV(실사) P0.946/R0.956(FP 16, FN 13)**. **이는 직전 조사의 "HSV 추가 가치 없음(판정 E)"을 뒤집는다** — 그 조사는 렌더 템플릿 HSV만 썼고 데이터도 훨씬 작았다. [실측]

**④ "YOLO-World가 후보를 못 만든다"는 가정은 사실상 틀렸다.**
이웃 프레임엔 후보가 있는데 이 프레임만 후보가 0인 **고립 dropout 410건**을 분해하면:
**Y1(raw 출력 자체가 없음) 1건(0.2%) / Y2(conf 0.02·NMS에서 제거) 315건(76.8%) / Y3(객체별 score_threshold) 94건(22.9%, 전부 인형) / top_k 절단 0건.**
즉 문제는 YOLO의 시각 능력이 아니라 **후처리 임계**다. 개선 방향이 근본적으로 달라진다. [실측]

**⑤ [추가 발견] 실제 객체의 26.9%는 "자기 프롬프트"가 제안하지 못했다.**
라벨된 실객체 박스 402개 중 108개(26.9%)는 그 객체의 프롬프트가 아니라 **다른 객체의 프롬프트**가 만든 박스였다. Febreze 56.9%, saffron 45.2%, Dinosaur 35.9%, Rabbit 34.4%. 프롬프트끼리 서로의 객체를 주워오고 있다. [실측]

**이번 단계에서 운영 코드에 적용한 변경: 없음.**

---

## 2. 분석 대상 SAM_* 6개 데이터 목록

`inventory/sam_dataset_inventory.csv` 전체. 원본은 `data_slam/260714_frame_data/SAM_data/recording_20260714_*`, 변환본은 `converted/sam_*`.

| dataset_name | absolute_path (변환본) | 형식 | color 프레임 | 길이 | RGB topic | Depth topic | 기존 visibility label | 기존 detection 결과 | frame_results | candidate log |
|---|---|---|---:|---:|---|---|---|---|---|---|
| sam_105018 | `.../converted/sam_105018` | ROS2 bag(sqlite3) | 401 | 133.5s | `/camera/camera/color/image_raw` | `/camera/camera/aligned_depth_to_color/image_raw` | 없음 | 있음(24 bundle) | 없음 | 없음→본 조사 최초 생성 |
| sam_105314 | `.../converted/sam_105314` | ROS2 bag | 594 | 99.0s | 〃 | 〃 | 없음 | 있음(594) | 없음 | 〃 |
| sam_105652 | `.../converted/sam_105652` | ROS2 bag | 549 | 183.3s | 〃 | 〃 | 없음 | 있음(24) | 없음 | 〃 |
| sam_110104 | `.../converted/sam_110104` | ROS2 bag | 547 | 182.3s | 〃 | 〃 | 없음 | 있음(24) | 없음 | 〃 |
| sam_110532 | `.../converted/sam_110532` | ROS2 bag | 152 | 51.3s | 〃 | 〃 | 없음 | 있음(24) | 없음 | 〃 |
| sam_110633 | `.../converted/sam_110633` | ROS2 bag | 353 | 118.0s | 〃 | 〃 | 없음 | 있음(24) | 없음 | 〃 |
| **합계** | | | **2,596** | 767.4s | | | | | | |

**"6개" 확정 근거**: `SAM_data/` 디렉터리에 정확히 6개의 recording이 있고, 기존 보고서들이 "6개 데이터셋"으로 인용한 이름(sam_105018/105314/105652/110104/110532/110633)과 정확히 일치하며, `configs/yolo_ism_objects.yaml` 주석의 "2,596-프레임 A/B"와 총 프레임 수가 일치한다.

**포함하지 않은 `SAM_*` 디렉터리(사유 명기)**: `data_slam/rgbd_imu_sdk_bag/SAM_circle · SAM_loop1 · SAM_loop2 · SAM_occlusion` — 이름은 `SAM_*`이지만 **다른 촬영 계열(rgbd_imu_sdk)** 이고 4개뿐이라 "6개"를 구성할 수 없다. 이번 분석 대상에서 제외했음을 inventory에 기록했다.

---

## 3. 운영 baseline과 분석 방법

### 3.1 코드로 확정한 운영 파라미터 [정적]

| 항목 | 값 | 근거 |
|---|---|---|
| 객체 클래스 수 | 10 (enabled) | `configs/yolo_ism_objects.yaml` |
| 프롬프트 목록 | milk carton / maroon box / febreze spray bottle / mug cup / white jug / orange bottle / yellow can / brown bear doll / white rabbit doll / green dinosaur doll | 〃 |
| YOLO 가중치·해상도 | yolov8m-worldv2.pt, imgsz=960 | 〃 |
| YOLO conf (공유 패스) | `min(객체별 score_threshold)` = 0.02 | `build_ism_inputs_imu.py:262` |
| 객체별 score_threshold | 기본 0.02 / **Bear 0.35 · Rabbit 0.30 · Dinosaur 0.30** | config |
| NMS | ultralytics 기본(iou=0.7, agnostic=False, 박스당 argmax 클래스) | `ultralytics.utils.nms` |
| top_k | 3 (전 객체) | config `defaults.top_k` |
| semantic score | 42 템플릿 CLS 코사인의 **상위 5개 평균** | `yolo_ism.py:238-242` |
| semantic 선택 | 객체당 argmax 1박스, `< similarity_threshold(0.35)` → 거절 | `yolo_ism_object_n.py:310-316` |
| appearance score | mask 내부 patch(블록11) vs **CLS-argmax 템플릿 1장** | `yolo_ism_object_n.py:332-333` |
| 최종 threshold | `appe_gate` = 0.55 (전 객체 공통) | config |
| 동일 BBox 다중 클래스 처리 | **가능** — 객체별 독립 루프 | `recognize()` 구조 |
| 클래스 완전 독립 추론 | **예** | 〃 |
| class 간 직접 비교 | **cross-object NMS(IoU>0.5)에서만** 존재 | `build_ism_inputs_imu.py:307-312` |

### 3.2 관찰 프로브 (운영 무변경)

`probes/dump_all_frames.py` — 운영 모듈을 import만 하고 수정하지 않는다. 판정 로직은 운영과 동일하되, 관찰을 위해 추가로:

* **RAW YOLO 패스**(conf=0.001, iou=0.95, max_det=300)를 프롬프트마다 한 번 더 실행 → "raw에 후보가 없음"과 "raw엔 있는데 후처리에서 제거됨"을 **라벨 없이** 분리.
* **운영 후보 박스 전부**에 MobileSAM 적용(운영은 승자에만) → 선택 실패와 게이트 실패를 같은 프레임에서 함께 관측.
* **박스 × 전 10클래스** 점수 계산(Top-3 관찰용; 판정에는 미사용).
* **HSV 히스토그램 원본 저장**(masked / bbox / background × H32·S32·V32·H16×S8) → bin 수·색공간·유사도 변형은 오프라인 재계산.

산출 규모: **박스 22,436 / 박스×클래스 224,360 / frame×object 25,960행**.

### 3.3 운영 baseline 결과 (2,596 프레임)

| dataset | 프레임 | 후보 (frame,object) 그룹 | accept | reject_semantic | reject_appearance |
|---|---:|---:|---:|---:|---:|
| sam_105018 | 401 | 1,520 | 502 | 797 | 221 |
| sam_105314 | 594 | 4,199 | 1,821 | 1,689 | 689 |
| sam_105652 | 549 | 3,987 | 1,683 | 1,625 | 679 |
| sam_110104 | 547 | 3,966 | 1,693 | 1,586 | 687 |
| sam_110532 | 152 | 973 | 397 | 385 | 191 |
| sam_110633 | 353 | 1,904 | 674 | 808 | 422 |
| **합계** | **2,596** | **16,549** | **6,770** | **6,890** | **2,889** |

| object | 후보 그룹 | accept | reject_semantic | reject_appearance |
|---|---:|---:|---:|---:|
| saffron | 2,374 | 985 | 827 | 562 |
| Febreze_high | 2,067 | 984 | 542 | 541 |
| Bear | 1,102 | 799 | 260 | 43 |
| choco_hazelnut_high | 1,981 | 783 | 794 | 404 |
| Sikhye_high | 1,504 | 698 | 715 | 91 |
| Sauce_high | 1,932 | 636 | 627 | 669 |
| milk | 1,826 | 554 | 900 | 372 |
| Rabbit | 683 | 462 | 188 | 33 |
| Mugcup_high | 2,264 | 448 | 1,684 | 132 |
| Dinosaur | 816 | 421 | 353 | 42 |

---

## 4. 라벨 provenance와 신뢰도

**⚠ 모든 정체 라벨은 Claude Vision이 생성한 provisional 라벨이며 사람 GT가 아니다.**

| 라벨 세트 | 건수 | 출처 | confidence | review_status |
|---|---:|---|---|---|
| `box_labels_new.csv` | 360 | 본 조사 컨택트 시트(150px 타일 12장) 육안 검수, 데이터셋별 60건 층화(수락 70% / 거절 30%) | provisional | not_reviewed |
| `box_labels_prior.csv` | 144 | 2026-07-20 직전 조사 라벨 중 이번 uid로 **좌표까지 정확히 일치**한 것만 이관 | provisional | not_reviewed |
| **`box_labels_merged.csv`** | **504** | 위 둘 병합 | provisional | not_reviewed |
| `human_review_queue.csv` | 79 | unclear 30건 + 무작위 10% 감사 표본 | — | pending |

라벨 분포: saffron 93, **carton(갈색 택배박스) 68**, Sikhye 52, Febreze 51, Sauce 40, Dinosaur 39, Rabbit 32, Bear 31, **unclear 30**, milk 26, Mugcup 19, choco 19, other 4.
데이터셋별: 105314 151 / 110633 113 / 나머지 4개 각 60.

**측정하지 못한 라벨**: 프레임 단위 **객체 가시성 GT**. 300px 썸네일 컨택트 시트(30프레임, `contact_sheets/frames/`)를 만들어 시도했으나, 이 데이터는 사무실을 훑는 카메라 스윕이라 대상 객체가 작게·부분적으로 등장해 **썸네일 해상도에서 신뢰할 만한 가시성 판정이 불가능**했다. 근거 없는 라벨을 만들지 않기 위해 **가시성 라벨은 생성하지 않았고**, 그 대신 §14–16의 라벨-불요 지표와 시간적 프록시로 대체했다. 이는 이번 조사의 최대 한계이며 §18 후속 C의 선결 과제다.

---

## 5. 전체 프레임 및 객체 통계

* 처리 프레임 2,596 (전수, stride=1)
* frame×object 관찰행 25,960 (`results/all_frames_object_results.csv`)
* 후보 박스 22,436, 박스×클래스 점수 224,360 (`results/hsv_feature_results.csv`)
* proposal group (IoU 0.5) 12,504
* 라벨된 박스 504 / 라벨된 candidate 쌍 770 / 라벨된 semantic 승자 결정 536

---

## 6. 동일 proposal의 클래스 Top-3 분석

### 6.1 grouping 방식과 민감도

`bbox IoU > τ` union-find. τ 민감도:

| τ | proposal group 수 | 2개 이상 클래스 고득점 | 비율 | **2개 이상 동시 수락** | **비율** | 3개 이상 수락 | Top1−Top2 gap median | gap p10 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.3 | 12,053 | 1,190 | 9.87% | 1,108 | **9.19%** | 190 | 0.1333 | 0.0155 |
| 0.5 | 12,504 | 1,180 | 9.44% | 1,098 | **8.78%** | 189 | 0.1330 | 0.0156 |
| 0.7 | 12,715 | 1,178 | 9.26% | 1,093 | **8.60%** | 189 | 0.1324 | 0.0157 |

→ **grouping 임계에 강건**. 이후 수치는 τ=0.5 기준.

`ambiguous_group` 처리: union-find로 병합되지 않은 박스는 각자 독립 그룹으로 남으며 억지 병합하지 않았다.

### 6.2 정답 클래스 순위 (라벨된 350 proposal group)

| 순위 기준 점수 | Top-1 | Top-2 누적 | Top-3 누적 | Top-3 밖 |
|---|---:|---:|---:|---:|
| **appearance (운영 게이트 점수)** | **91.1%** | 95.7% | 97.1% | 2.9% |
| semantic (top5) | 90.9% | 94.6% | 97.4% | 2.6% |
| **YOLO confidence** | **74.6%** | 88.9% | 95.4% | 4.6% |
| **HSV (실사 prototype, LODO)** | **95.7%** | 97.1% | 97.1% | 2.9% |

* **오답이 Top-1인데 정답이 Top-2** = 16건, **Top-3** = 5건 → 합 21건(6.0%)이 "Top-1만 취하는" 현 구조에서 회수 불가능하게 버려진다.
* **YOLO confidence로 순위를 매기면 Top-1 정확도가 74.6%로 최악** — 기존 `--nms-rank appe` 결정을 6개 데이터 전수로 재확인.
* **HSV가 순위 지표로 가장 좋다(95.7%)**.

### 6.3 gap의 판별력 — class margin이 필요한 직접 증거

| 상황 | Top1−Top2 gap (appearance) median |
|---|---:|
| 정답이 Top-1 (올바른 승자) | **0.1884** |
| 오답이 Top-1 (잘못된 승자) | **0.0217** |

**8.7배 차이.** 즉 "1등과 2등이 붙어 있다"는 사실 자체가 오답 신호다. 이 신호는 현재 파이프라인에서 **전혀 사용되지 않는다**.

---

## 7. 복수 클래스 동시 수락 분석

* **동시 수락 그룹 1,098건 / 12,504 = 8.78%**, 3개 이상 189건.
* 그룹당 평균 수락 클래스 수 0.541.
* 최종 수락 6,770건 중 **동시 수락에 관여한 비율은 그룹 기준 8.78%**이며, 관여 검출 수로는 최소 2,287건(1,098×2 + 189)이 "같은 물체를 두 이름으로 부른" 결과다.

### 데이터셋별 (τ=0.5)

| dataset | proposal group | 동시 수락 | 비율 |
|---|---:|---:|---:|
| sam_105018 | 1,331 | 71 | 5.33% |
| sam_105314 | 3,164 | 255 | 8.06% |
| sam_105652 | 2,802 | 277 | 9.89% |
| sam_110104 | 2,919 | 320 | **10.96%** |
| sam_110532 | 814 | 58 | 7.13% |
| sam_110633 | 1,474 | 117 | 7.94% |

**6개 데이터 전부에서 발생**하며 특정 데이터셋 아티팩트가 아니다.

---

## 8. 객체별·데이터셋별 confusion pair

### 8.1 동일 proposal group 동시 수락 쌍 (전수, 라벨 불필요)

| 쌍 | 건수 | 비중 |
|---|---:|---:|
| **Febreze_high + saffron** | **597** | 54.4% |
| Bear + Rabbit | 283 | 25.8% |
| Bear + Dinosaur | 279 | 25.4% |
| Dinosaur + Rabbit | 214 | 19.5% |
| **인형 3쌍 소계** | **776** | 70.7% |
| choco_hazelnut_high + milk | 65 | 5.9% |
| Sikhye_high + saffron | 13 | 1.2% |
| Sauce_high + saffron | 9 | 0.8% |
| Sauce_high + Sikhye_high | 8 | 0.7% |
| 기타 | 9 | 0.8% |

(합이 100%를 넘는 것은 3개 이상 수락 그룹이 여러 쌍에 계상되기 때문)

### 8.2 라벨된 FP (실제 정체 → 수락된 클래스)

사용자가 지목한 혼동을 별도 집계하면 — **갈색 택배박스 ↔ 초코하임**: 라벨된 carton 68건 중 choco로 수락된 사례가 존재하며(§8.1의 co-accept에는 carton이 클래스가 아니라 나타나지 않음), 라벨 기준 FP 상위는 `Dinosaur→Bear`, `Rabbit→Bear`, `Febreze→saffron`, `saffron→Febreze`, `carton→choco`, `carton→milk` 순이다(`results/confusion_pairs.csv`).
**milk ↔ 다른 흰색/직사각형 객체**: `carton→milk`가 확인되며 choco+milk 동시 수락 65건이 이를 뒷받침한다.

---

## 9. HSV histogram 추출 및 비교 방법

원본 히스토그램을 저장하고 변형은 전부 오프라인 재계산했다.

* **영역 3종**: MobileSAM mask 내부(masked) / BBox 전체(bbox) / mask 외부(background)
* **색공간 6종**: H, S, V, H+S, H+S+V, H16×S8 (2D)
* **bin 3종**: 32 → 16 → 8 (32bin 저장 후 인접 합산으로 축소, 재추출 불필요)
* **유사도 5종**: intersection, cosine, correlation, Bhattacharyya 계수, chi-square 거리(음수화)
* **레퍼런스 2종**: 렌더 템플릿 42장 / **실사 crop prototype**(라벨된 실물 crop, leave-one-dataset-out으로 자기 데이터셋 제외)

총 **130개 변형**을 10객체 × 474 라벨 박스에 대해 스윕(`results/hsv_variant_sweep.csv`).

### 9.1 변형 스윕 상위 10 (렌더 템플릿 레퍼런스)

| region | space | bins | metric | mean AUROC | min AUROC |
|---|---|---|---|---:|---:|
| **masked** | **H+S** | **32** | **Bhattacharyya** | **0.865** | 0.584 |
| masked | H+S+V | 32 | Bhattacharyya | 0.859 | 0.527 |
| masked | H+S | 8 | intersection | 0.856 | 0.644 |
| bbox | H+S | 32 | Bhattacharyya | 0.856 | 0.624 |
| masked | H+S | 8 | Bhattacharyya | 0.853 | 0.693 |
| masked | H+S+V | 8 | Bhattacharyya | 0.853 | 0.645 |
| masked | H16×S8 | 16×8 | intersection | 0.852 | 0.487 |
| masked | H16×S8 | 16×8 | chi2 | 0.851 | 0.490 |
| masked | H+S | 8 | chi2 | 0.850 | 0.620 |
| masked | H16×S8 | 16×8 | Bhattacharyya | 0.846 | 0.526 |

**관찰**:
* **masked가 bbox보다 일관되게 낫다** — 상위 10 중 8개가 masked. V 채널을 넣으면 min AUROC가 떨어진다(조명 민감).
* bin 수는 8~32에서 큰 차이가 없다 → **8 bin으로도 충분**(비용 이점).
* Bhattacharyya가 가장 안정적, cosine/correlation이 가장 약함.

---

## 10. 객체별 HSV 성능

`results/hsv_object_summary.csv`. positive = 그 객체로 라벨된 박스, negative = 그 객체의 후보였으나 정체가 다른 박스, hard negative = carton/other.

| 객체 | Positive | Negative | Hard neg | **HSV 렌더 AUROC** | **HSV 실사 AUROC** | sem AUROC | appe AUROC | HSV실사 vs HardNeg | appe vs HardNeg | ρ(HSV,sem) | ρ(HSV,appe) | 독립 추가 가치 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Bear | 28 | 19 | 0 | 0.998 | **1.000** | 0.691 | **0.579** | — | — | 0.381 | 0.325 | **매우 큼**(appe 붕괴 구간) |
| Dinosaur | 25 | 5 | 0 | 1.000 | **1.000** | 0.544 | **0.472** | — | — | 0.087 | 0.098 | **매우 큼** |
| Rabbit | 21 | 8 | 0 | 1.000 | **1.000** | 0.827 | **0.548** | — | — | 0.444 | 0.056 | **매우 큼** |
| Mugcup_high | 19 | 21 | 0 | 1.000 | 1.000 | 1.000 | 1.000 | — | — | 0.729 | 0.773 | 없음(이미 완벽) |
| Sauce_high | 40 | 73 | 21 | **0.499** | **1.000** | 0.998 | 0.999 | 1.000 | 1.000 | 0.645 | 0.707 | 렌더는 무용, 실사는 완벽 |
| Sikhye_high | 47 | 41 | 5 | 0.797 | **1.000** | 0.998 | 0.997 | 1.000 | 1.000 | 0.767 | 0.734 | 중간 |
| Febreze_high | 22 | 85 | 6 | 0.937 | 0.986 | 0.970 | 0.923 | 1.000 | 1.000 | 0.275 | 0.202 | 있음 |
| milk | 23 | 77 | 32 | 0.862 | 0.986 | 0.971 | 0.974 | **1.000** | 0.908 | 0.543 | 0.499 | 있음(hard neg에서) |
| choco_hazelnut_high | 18 | 61 | 44 | 0.991 | 0.964 | 0.936 | 0.916 | **0.964** | 0.916 | 0.594 | 0.515 | 있음(hard neg에서) |
| saffron | 51 | 86 | 12 | 0.857 | **0.929** | 0.921 | 0.997 | 0.912 | **1.000** | 0.729 | 0.757 | **없음/역효과** |
| **평균** | | | | **0.865** | **0.987** | 0.885 | 0.841 | | | 0.519 | 0.467 | |

**핵심 관찰 4가지**:

1. **렌더 템플릿 HSV는 객체마다 편차가 극심**(Sauce 0.499 = 무작위, Bear 0.584 vs Mugcup 1.000). 렌더-실사 color domain gap이 직접 원인이다.
2. **실사 prototype HSV는 전 객체 0.929~1.000**으로 균일하다. LODO이므로 데이터셋 간 일반화가 반영된 값이다.
3. **HSV가 가장 크게 이기는 곳은 운영 게이트가 무너지는 곳**이다 — 인형 3종에서 appe 0.472~0.579(무작위 수준) vs HSV 1.000. 그리고 그 구간에서 **상관도 가장 낮다**(ρ 0.056~0.444) → 진짜 독립 정보.
4. **유일한 역전은 saffron**: appe 0.997 > HSV 0.929, hard negative에서도 appe 1.000 > HSV 0.912. saffron(크림색 무광 통)은 색이 배경·milk와 겹친다.

### 객체군별 정리

| 객체군 | 구성 | HSV 실사 AUROC | 판정 |
|---|---|---:|---|
| **인형(봉제)** | Bear, Rabbit, Dinosaur | 1.000 / 1.000 / 1.000 | **HSV가 유일하게 작동하는 신호** |
| **다색 인쇄 포장** | choco, milk, Sikhye | 0.964~1.000 | 보완적(특히 갈색박스 대비) |
| **플라스틱 용기(유색)** | Febreze, Sauce, Mugcup | 0.986~1.000 | 유효하나 기존 점수도 충분 |
| **단색 무광 용기** | saffron | 0.929 | **기존 appe가 더 낫다** |

---

## 11. HSV의 데이터셋 간 일반화

`results/hsv_real_prototype_lodo.csv` — 6-fold leave-one-dataset-out, H+S/32bin/Bhattacharyya/masked.

| 객체 | fold 수 | mean AUROC | min | max |
|---|---:|---:|---:|---:|
| Dinosaur | 6 | 1.000 | 1.000 | 1.000 |
| Mugcup_high | 4 | 1.000 | 1.000 | 1.000 |
| Sauce_high | 6 | 0.999 | 0.994 | 1.000 |
| Bear | 5 | 0.999 | 0.997 | 1.000 |
| Rabbit | 6 | 0.998 | 0.991 | 1.000 |
| choco_hazelnut_high | 5 | 0.994 | 0.968 | 1.000 |
| Febreze_high | 5 | 0.991 | 0.976 | 1.000 |
| Sikhye_high | 6 | 0.983 | **0.898** | 1.000 |
| milk | 6 | 0.973 | **0.852** | 1.000 |
| saffron | 6 | 0.959 | **0.856** | 1.000 |

**데이터셋을 완전히 빼도 0.85 아래로 떨어지는 fold가 없다.** 다만 milk·saffron·Sikhye는 fold 편차가 있어(min 0.85~0.90) 이 세 객체가 일반화 취약점이다.

**⚠ 일반화의 경계**: 6개 데이터는 **같은 날 같은 사무실**에서 촬영됐다. 따라서 위 수치는 "시점·거리 변화에 대한 일반화"는 검증하지만 **조명 환경이 다른 새 세션에 대한 일반화는 검증하지 못한다.** 조명 극단 subset이 없다는 §4의 한계가 여기에 직접 걸린다.

---

## 12. Semantic / Appearance / HSV 오류 상관관계

* **Spearman ρ 평균**: HSV~sem 0.519, HSV~appe 0.467. 객체별로 **인형에서 0.056~0.444로 특히 낮다**(=독립).
* 반면 semantic과 appearance는 같은 DINOv2 forward의 두 요약이라 구조적으로 상관이 높다(직전 조사 실측 ρ 0.864).
* **HSV만 성공하는 사례**: 인형 3종 전체(appe AUROC 0.47~0.58 → HSV 1.00). 380여 건의 인형 동시 수락(Bear+Rabbit 283, Bear+Dinosaur 279, Dinosaur+Rabbit 214)이 여기에 해당한다.
* **HSV만 실패하는 사례**: saffron(HSV 0.929 < appe 0.997). saffron은 라벨 93건으로 최다 객체라 전체 FP에 미치는 영향이 크다 — HSV를 단독 게이트로 쓰면 안 되는 이유.
* **세 점수가 동시에 실패하는 사례**: choco vs carton hard negative에서 sem 0.936 / appe 0.916 / HSV 0.964 — 셋 다 완전 분리에는 못 미친다. 이 코호트는 어떤 단일 점수로도 풀리지 않는다.

---

## 13. HSV fusion 후보의 오프라인 비교

**6-fold leave-one-dataset-out**(5개 데이터에서 객체별 임계 산출 → 남은 1개에서만 평가). 결정 단위 = 라벨된 semantic 승자 536건. **운영 코드에는 적용하지 않았다.**

| 조합 | Precision | Recall | FP | FN | 상위 혼동 |
|---|---:|---:|---:|---:|---|
| **운영 고정 게이트 (baseline)** | 0.726 | 0.942 | **104** | **17** | saffron→Febreze, carton→choco, Dinosaur→Bear |
| S0 appearance (임계 재보정) | 0.859 | 0.897 | 43 | 30 | Dinosaur→Bear, Rabbit→Bear, Dinosaur→Rabbit |
| S1 semantic 단독 | 0.835 | 0.849 | 49 | 44 | Febreze→saffron, Dinosaur→Bear |
| S2 **HSV(실사) 단독** | 0.904 | 0.904 | 28 | 28 | Febreze→saffron, Sauce→Febreze |
| S3 HSV(렌더) 단독 | 0.850 | 0.911 | 47 | 26 | Sikhye→Sauce, saffron→Sauce |
| S4 semantic + appearance | 0.876 | 0.894 | 37 | 31 | Dinosaur→Bear, Rabbit→Bear |
| S5 semantic + HSV | 0.898 | 0.935 | 31 | 19 | Febreze→saffron, carton→milk |
| **S6 appearance + HSV(실사)** | **0.946** | **0.956** | **16** | **13** | Febreze→saffron, Dinosaur→Bear |
| S7 semantic + appearance + HSV | 0.932 | 0.945 | 20 | 16 | Febreze→saffron, carton→milk |

**해석**:
* **S6(appearance + 실사 HSV)가 FP·FN·Precision·Recall 모두에서 최고.** 운영 고정 게이트 대비 **FP 104 → 16(−85%)**, FN 17 → 13.
* **HSV의 레퍼런스가 결정적**: 같은 fusion 구조에서 렌더(S3) 47 FP vs 실사(S2) 28 FP.
* semantic을 추가하면(S7) 오히려 약간 나빠진다 — semantic이 appearance와 상관이 높아 새 정보를 주지 않으면서 노이즈를 더한다.
* 다만 **가중치는 전부 균등(0.5/0.5)이고 객체별 가중치는 시험하지 않았다.** 이번 단계 목적이 "후속 실험 가치 판단"이므로 최적화는 하지 않았다.

---

## 14. YOLO-World proposal recall

`results/yolo_candidate_stage_stats.csv` — 2,596 프레임 × 10 객체 = 25,960 frame-object 전수.

| object | frames | **raw=0** | post-NMS=0 | **top_k=0** | conf/NMS로 제거 | score_threshold로 제거 | top_k 절단 |
|---|---:|---:|---:|---:|---:|---:|---:|
| choco_hazelnut_high | 2,596 | **5** | 615 | 615 | 610 | 0 | 216 |
| saffron | 2,596 | **9** | 222 | 222 | 213 | 0 | 1,133 |
| Mugcup_high | 2,596 | 12 | 332 | 332 | 320 | 0 | 644 |
| milk | 2,596 | 97 | 770 | 770 | 673 | 0 | 113 |
| Febreze_high | 2,596 | 103 | 529 | 529 | 426 | 0 | 368 |
| Sikhye_high | 2,596 | 215 | 1,092 | 1,092 | 877 | 0 | 49 |
| Sauce_high | 2,596 | 270 | 664 | 664 | 394 | 0 | 69 |
| Rabbit | 2,596 | 454 | 870 | **1,913** | 416 | **1,043** | 0 |
| Bear | 2,596 | 777 | 951 | **1,494** | 174 | **543** | 0 |
| Dinosaur | 2,596 | 806 | 1,015 | **1,780** | 209 | **765** | 4 |

**읽는 법**:
* **raw=0**은 conf 0.001·거의 NMS 없음에서도 아무 박스가 없다는 뜻 — 강체 객체는 0.2~10% 수준으로 매우 드물다. 인형은 17~31%로 높은데, 이는 인형이 프레임에 없는 구간이 많다는 뜻이기도 해서 **raw=0을 곧바로 "YOLO가 못 봤다"로 읽으면 안 된다**(가시성 라벨 부재, §4).
* **인형 3종만 score_threshold로 대량 제거**(543/765/1,043)된다 — cross-talk 방지용 가드(Bear 0.35, Rabbit/Dinosaur 0.30)의 직접 대가.
* **top_k=3 절단은 saffron(1,133) · Mugcup(644) · Febreze(368)** 에서 크다. 이들은 후보가 많아 3개로 자르는 과정에서 무언가를 버린다.

---

## 15. 객체별 proposal miss

### 15.1 "자기 프롬프트가 자기 객체를 제안했는가" (라벨 기반, n=402)

라벨된 실객체 박스(carton/other/unclear 제외)에 대해, 그 박스가 **자기 클래스의 후보였는지** 확인.

| object | 라벨된 실객체 박스 | 자기 프롬프트가 제안 | **miss** | **miss율** |
|---|---:|---:|---:|---:|
| Mugcup_high | 19 | 19 | 0 | 0.0% |
| Sauce_high | 40 | 40 | 0 | 0.0% |
| choco_hazelnut_high | 19 | 18 | 1 | 5.3% |
| Sikhye_high | 52 | 47 | 5 | 9.6% |
| Bear | 31 | 28 | 3 | 9.7% |
| milk | 26 | 23 | 3 | 11.5% |
| Rabbit | 32 | 21 | 11 | **34.4%** |
| Dinosaur | 39 | 25 | 14 | **35.9%** |
| saffron | 93 | 51 | 42 | **45.2%** |
| Febreze_high | 51 | 22 | 29 | **56.9%** |
| **합계** | **402** | **294** | **108** | **26.9%** |

**실제 객체의 27%는 자기 프롬프트가 아니라 남의 프롬프트가 주워온 박스다.** Febreze와 saffron이 서로를 주워오고(§8의 597건 동시 수락과 같은 현상), 인형끼리도 마찬가지다.

**⚠ 표본 편향**: 이 지표는 "어떤 프롬프트든 박스를 만든" 위치만 대상으로 한다. 아무 프롬프트도 박스를 만들지 못한 객체는 이 분모에 없다.

### 15.2 시간적 프록시로 본 고립 dropout

이웃 프레임(t−1, t+1)에는 후보가 있는데 **t에서만 후보가 0**인 경우 = 객체가 여전히 보였을 개연성이 높은 dropout. **이는 사람 라벨이 아니라 시간적 프록시임을 명시한다.**

| object | 내부 프레임 | 고립 dropout | 비율 |
|---|---:|---:|---:|
| milk | 2,584 | 72 | 2.79% |
| Mugcup_high | 2,584 | 59 | 2.28% |
| Rabbit | 2,584 | 48 | 1.86% |
| Febreze_high | 2,584 | 42 | 1.63% |
| choco_hazelnut_high | 2,584 | 40 | 1.55% |
| Sikhye_high | 2,584 | 38 | 1.47% |
| Sauce_high | 2,584 | 34 | 1.32% |
| saffron | 2,584 | 30 | 1.16% |
| Bear | 2,584 | 28 | 1.08% |
| Dinosaur | 2,584 | 19 | 0.74% |
| **합계** | **25,840** | **410** | **1.59%** |

---

## 16. Raw YOLO miss와 후처리 제거 분석

고립 dropout 410건을 단계별로 분해:

| 단계 | 정의 | 건수 | 비중 |
|---|---|---:|---:|
| **Y1** | RAW YOLO(conf 0.001) 출력 자체가 없음 | **1** | **0.24%** |
| **Y2** | raw엔 있으나 conf 0.02 / NMS에서 제거 | **315** | **76.83%** |
| **Y3-thr** | post-NMS엔 있으나 **객체별 score_threshold**에서 제거 | **94** | **22.93%** |
| **Y3-topk** | score_threshold 통과분이 top_k=3에서 잘림 | **0** | **0.00%** |

객체별 Y3-thr 94건의 내역: **Rabbit 48 / Bear 27 / Dinosaur 19 — 전부 인형**(가드 임계 0.30~0.35).

**결론(사용자 §12.3의 핵심 질문에 대한 답)**:
> **raw YOLO miss와 후처리 제거 중 어느 쪽이 더 큰가? → 후처리가 압도적이다(76.8% + 22.9% = 99.8% vs 0.2%).**

즉 "YOLO-World가 객체를 못 본다"는 서술은 이 데이터에서 지지되지 않는다. **후보는 거의 언제나 raw 출력에 존재하며, conf 0.02 임계와 NMS가 그것을 버린다.** top_k를 늘려도 회수되는 것은 0건이므로 top_k 확대는 무의미하다.

**⚠ 이 결론의 범위**: 고립 dropout(410건)이라는 특정 코호트에서의 분해다. "객체가 보이는데 이웃 프레임에서도 계속 후보가 없는" 지속적 miss는 가시성 라벨 없이는 측정할 수 없으며, 이번 조사에서 다루지 못했다.

---

## 17. 대표 실패 사례

`contact_sheets/` 아래에 근거 이미지를 보존했다(각 타일에 인덱스, `index_map.csv`로 uid·dataset·frame·현재 판정 연결).

| 사례 | 근거 시트 | 관찰 |
|---|---|---|
| 동일 박스에 2클래스 이상 고득점·동시 수락 | `boxes_02 #71`(Dinosaur|Rabbit 동시), `boxes_06 #185`(Bear|Rabbit), `boxes_08 #241`(Bear|Rabbit) | 한 마리 공룡을 Bear·Rabbit·Dinosaur가 동시에 주장 |
| **샤프란 ↔ Febreze** | `boxes_02 #77 #78`, `boxes_08 #250~#255`, `boxes_10 #306~#311` | 크림색 세제통이 Febreze로, 파란 분무기가 saffron으로 상호 수락 |
| **갈색 택배박스 → 초코하임** | `boxes_01 #32 #33 #34`, `boxes_05 #151 #152 #154 #156 #157`, `boxes_11 #330` | GALAX/logi/UN3481 상자가 choco로 수락 |
| **초코하임 FN** | `boxes_09 #288 #294` | 진짜 초코하임 상자가 거절 |
| 인형 상호 혼동 | `boxes_00 #2 #6 #20 #21`, `boxes_02 #63 #64 #67 #69`, `boxes_10 #300 #301 #304` | 곰 슬롯에 토끼·공룡, 공룡 슬롯에 토끼 |
| carton → milk | `boxes_03 #91 #93`, `boxes_07 #212 #213` | 갈색 상자가 milk로 수락 |
| 인형 FN(가드 임계) | `boxes_07 #222`, `boxes_09 #282 #285`, `boxes_11 #342 #343` | 진짜 인형이 거절 |
| 프레임 단위(가시성 판정 불가 사례) | `contact_sheets/frames/*` | 300px 썸네일에서 소형·부분 객체 판정 불가 — §4 한계의 근거 |

---

## 18. 세 가지 후속 연구 방향

### 후속 A — Class Competition 실험

| 항목 | 내용 |
|---|---|
| **진행 필요** | **예 (최우선)** |
| **근거** | 동시 수락 8.78%가 6개 데이터 전부에서 발생(5.3~11.0%); 오답 Top-1이지만 정답이 Top-2/3인 사례 21건(6.0%); **Top1−Top2 gap이 정답Top1 0.188 vs 오답Top1 0.0217로 8.7배** — 판별 신호가 이미 데이터에 있고 추가 모델 호출이 0회 |
| **추가 데이터** | 비승자 후보 박스 라벨(현재 승자만 라벨 → 선택 실패를 측정 불가), 객체당 positive/negative 각 30건 이상 |
| **수정 범위** | `yolo_ism_object_n.recognize_frame()`에 프레임 내 전 객체 점수 행렬 유지 + margin 계산. `yolo_ism.py` 무수정 가능 |
| **검증 지표** | 동시 수락 그룹 수, 객체별 P/R/F1, Top-2 회수 건수, LODO |
| **위험** | "관측 물체가 등록 10객체 중 하나"라는 가정 — 미등록 물체(택배박스)에는 무력하므로 절대 점수항을 반드시 병기 |
| **순서** | **1번째** |

### 후속 B — semantic + appearance + HSV fusion 실험

| 항목 | 내용 |
|---|---|
| **진행 필요** | **예 (2순위)** — 단 **실사 prototype 형태로만** |
| **근거** | 실사 HSV LODO 전 객체 0.929~1.000; 운영 게이트가 무너지는 인형 구간에서 appe 0.47~0.58 vs HSV 1.00이고 상관도 0.06~0.44로 독립; 오프라인 fusion S6에서 FP 104→16 |
| **추가 데이터** | 객체별 실사 crop prototype(객체당 ≥10건, 데이터셋 ≥3개 분산), **조명 조건이 다른 세션**(현 6개는 같은 날 같은 사무실 — 조명 일반화 미검증) |
| **수정 범위** | HSV prototype 캐시 생성 스크립트 신규 + `recognize_frame()`의 점수 계산부. mask는 이미 있으므로 추가 모델 호출 0회 |
| **검증 지표** | 객체별 P/R/F1(특히 saffron 회귀 감시), LODO 6-fold, 조명 다른 세션에서 재현 |
| **위험** | ① **saffron은 HSV가 오히려 나쁨**(0.929 vs appe 0.997) → 전 객체 균등 가중은 위험 ② 실사 prototype은 라벨 의존 ③ 렌더 템플릿 HSV로 대체하면 이득의 절반이 사라짐(S3 FP 47 vs S2 28) |
| **순서** | **2번째** (A 적용 후 잔여 오류에 대해 측정) |

### 후속 C — Proposal recall 개선 실험

| 항목 | 내용 |
|---|---|
| **진행 필요** | **예 (3순위) — 단 목표를 바꿔서** |
| **근거** | 고립 dropout 410건 중 **raw YOLO 부재는 1건(0.2%)**, conf/NMS 제거 76.8%, 인형 가드 임계 22.9%, top_k 절단 0건. 즉 "YOLO 성능 개선"이 아니라 **"임계·NMS 정책 재설계"** 가 과제 |
| **추가 데이터** | **프레임 단위 객체 가시성 GT (필수)** — 이번 조사가 유일하게 만들지 못한 라벨. 고해상도 개별 프레임 검수 필요, 데이터셋당 ≥30 프레임 |
| **수정 범위** | conf 임계, NMS iou/multi_label, 인형 가드 임계 대체 수단(가드를 후단 class competition으로 이전) |
| **검증 지표** | 객체별 candidate recall, 인형 가드 제거 시 cross-talk 증가량, 최종 P/R |
| **위험** | conf를 낮추면 후보가 폭증해 top_k 절단(이미 saffron 1,133건)이 악화 → A(class competition) 없이 단독 진행하면 FP 급증 |
| **순서** | **3번째** (A가 후보 증가를 흡수할 수 있게 된 뒤) |

---

## 19. 우선순위와 다음 BMAD 명령

1. **후속 A(class competition)** — 신호가 가장 강하고(gap 8.7배), 추가 데이터·모델 비용이 0이며, C의 전제조건이다.
2. **후속 B(HSV fusion)** — A 적용 후 남는 오류(특히 인형)에 대해 실사 prototype으로 측정. saffron 회귀를 반드시 감시.
3. **후속 C(proposal 정책)** — A가 후보 증가를 흡수할 수 있게 된 뒤. 착수 전 **가시성 GT 라벨링**이 선결.

병행 과제: **사람 검수** — `labels/human_review_queue.csv` 79건(unclear 30 + 무작위 10% 감사). 불일치 5% 초과 시 라벨 전면 재작성.

---

## 20. 결론

```text
분석한 SAM 데이터:
- sam_105018  (401 프레임, 133.5s)
- sam_105314  (594 프레임,  99.0s)
- sam_105652  (549 프레임, 183.3s)
- sam_110104  (547 프레임, 182.3s)
- sam_110532  (152 프레임,  51.3s)
- sam_110633  (353 프레임, 118.0s)
  → 합계 2,596 프레임 전수 (stride=1), 박스 22,436 / 박스×클래스 224,360

Top-3 클래스 관찰 결과:
- 복수 클래스 고득점 비율: 9.44% (1,180 / 12,504 proposal group, IoU 0.5)
- 복수 클래스 동시 수락 비율: 8.78% (1,098 그룹, 3클래스 이상 189) — IoU 0.3/0.5/0.7 에서
  9.19% / 8.78% / 8.60% 으로 grouping 임계에 강건, 6개 데이터 전부에서 5.3~11.0%
- 정답 Top-1 포함률: 91.1% (appearance 기준) / 90.9% (semantic) / 74.6% (YOLO conf) / 95.7% (HSV 실사)
- 정답 Top-2 포함률: 95.7%
- 정답 Top-3 포함률: 97.1%  (Top-3 밖 2.9%)
- class competition 필요성: 필요하다. 오답이 Top-1 인데 정답이 Top-2/Top-3 인 사례가 21건(6.0%)
  이고, Top1−Top2 gap 이 정답Top1 일 때 median 0.1884 / 오답Top1 일 때 median 0.0217 로
  8.7배 차이 나 판별 신호가 이미 존재한다. 최다 경쟁쌍은 Febreze+saffron 597건(54%),
  인형 3쌍 776건(71%), choco+milk 65건이다.

HSV 전체 객체 판정:
A (모든 객체에 공통적으로 유효) — 단, **실사 crop prototype 형태에 한한다.**
렌더 템플릿 HSV 만 쓰면 판정은 C(특정 객체군에서만 유효)로 내려간다:
실사 prototype LODO AUROC 0.929~1.000 (평균 0.987) vs 렌더 템플릿 0.499~1.000 (평균 0.865).

HSV가 유효한 객체:
- Bear / Rabbit / Dinosaur — AUROC 1.000, 같은 조건에서 운영 게이트(appe11)는 0.579/0.548/0.472
  로 무작위 이하이며 상관도 ρ 0.056~0.444 로 낮다 → 유일하게 작동하는 독립 신호
- Mugcup_high / Sauce_high / Sikhye_high — 1.000 (기존 점수도 충분하므로 추가 이득은 작음)
- Febreze_high 0.986 / milk 0.986 / choco_hazelnut_high 0.964 — 보완적, 특히 갈색 택배박스
  hard negative 에서 milk 1.000(appe 0.908) · choco 0.964(appe 0.916) 로 우위

HSV가 불안정한 객체:
- saffron — 실사 0.929 < appe 0.997, hard negative 에서도 0.912 < appe 1.000. 크림색 무광
  용기라 배경·milk 와 색이 겹친다. 라벨 93건으로 최다 객체라 전 객체 균등 가중 시 위험이 크다
- milk / Sikhye_high — LODO fold 최저값이 0.852 / 0.898 로 데이터셋 간 편차가 있다
- 렌더 템플릿 기준으로는 Sauce_high 0.499(무작위) · Bear 0.584 · Rabbit 0.790 이 불안정

HSV의 기존 점수 대비 추가 가치:
- 오프라인 6-fold LODO: 운영 고정 게이트 P0.726/R0.942(FP 104, FN 17)
  → appearance+HSV(실사) P0.946/R0.956(FP 16, FN 13). FP −85%
- semantic 을 더하면 오히려 악화(S7 FP 20) — semantic 은 appearance 와 같은 DINOv2 forward
  의 요약이라 새 정보를 주지 않는다
- 상관 ρ(HSV, sem) 평균 0.519 / ρ(HSV, appe) 평균 0.467, 인형 구간에서 0.056~0.444
- 색 특징은 반드시 mask 내부에서 계산해야 한다(변형 스윕 상위 10 중 8개가 masked)
- 최적 변형: masked · H+S · 8~32 bin · Bhattacharyya (bin 8 로도 성능 동등 → 비용 이점)

권장 HSV 사용 후보:
- **객체군별 fusion** (공통 final score 아님, uncertainty-only 아님)
- 근거: 인형군에서는 HSV 가 사실상 유일한 신호(AUROC 1.000 vs appe 0.47~0.58)인 반면
  saffron 에서는 HSV 가 appe 보다 나쁘다(0.929 vs 0.997). 전 객체 공통 가중치를 쓰면
  saffron 회귀가 불가피하다. 다만 객체별 전용 규칙은 이 저장소가 세 번 롤백한 함정이므로,
  "객체 하나하나의 가중치"가 아니라 "인형 / 다색 포장 / 단색 무광 용기" 수준의 **객체군 가중치**
  로 제한하고 LODO 로 검증할 것을 권한다.

YOLO proposal miss 결과:
- 전체 visible positive: **측정하지 못함.** 프레임 단위 객체 가시성 GT 를 만들지 못했다
  (300px 썸네일에서 소형·부분 객체 판정 불가 → 근거 없는 라벨 생성을 회피).
  대신 라벨-불요 지표 두 가지로 대체했다.
- proposal miss (대체 지표 ①, 라벨 기반): 라벨된 실객체 박스 402건 중 **108건(26.9%)** 이
  자기 프롬프트가 아니라 다른 객체 프롬프트가 만든 박스였다
- proposal miss 비율 (대체 지표 ②, 시간적 프록시): 이웃 프레임엔 후보가 있는데 해당 프레임만
  후보 0 인 고립 dropout **410건 / 25,840 frame-object = 1.59%**
- 가장 심각한 객체: 자기 프롬프트 miss 기준 **Febreze_high 56.9% · saffron 45.2% ·
  Dinosaur 35.9% · Rabbit 34.4%**; 후보 도달 실패(top_k=0) 프레임 수 기준
  **Rabbit 1,913 · Dinosaur 1,780 · Bear 1,494**
- raw YOLO miss 비율: **410건 중 1건 (0.24%)** — RAW(conf 0.001, iou 0.95) 출력에는
  거의 언제나 후보가 존재한다
- NMS/top_k 제거 비율: **conf 0.02·NMS 제거 315건(76.83%) + 객체별 score_threshold 제거
  94건(22.93%, 전부 인형 가드) + top_k 절단 0건(0.00%)** = 후처리가 99.76%

현재 단계에서 적용할 변경:
없음 — 관찰과 오프라인 분석만 수행함.

다음 권장 실험 1:
Class competition 실험 — 프레임 내 전 객체 점수 행렬을 유지해 Top1−Top2 margin 을 계산하고,
동시 수락 8.78% 와 "오답 Top-1·정답 Top-2/3" 21건이 실제로 해소되는지 LODO 로 측정한다.
선결: 비승자 후보 박스 라벨(현재 승자만 라벨되어 선택 실패를 측정할 수 없음).
성공 기준 = 동시 수락 그룹 50% 이상 감소 AND 객체별 recall 감소 0.

다음 권장 실험 2:
객체군별 HSV fusion 실험 — 실사 crop prototype(masked H+S, 8~32bin, Bhattacharyya)을
객체군(인형 / 다색 포장 / 단색 무광 용기) 단위 가중으로 결합하고 6-fold LODO 로 검증한다.
반드시 **조명 조건이 다른 새 세션**을 하나 추가로 수집해 일반화를 확인한다(현 6개는 같은 날
같은 사무실). 성공 기준 = FP 50% 이상 감소 AND saffron recall 감소 0.

다음 권장 실험 3:
Proposal 정책 실험 — 목표를 "YOLO 성능 개선"이 아니라 "후처리 임계 재설계"로 재정의한다.
conf 0.02 인하 / NMS multi_label / 인형 가드 임계를 후단 class competition 으로 이전하는
세 가지를 각각 A/B 한다. 선결: 프레임 단위 가시성 GT (데이터셋당 ≥30 프레임, 고해상도 개별
검수). 실험 1 이 후보 증가를 흡수할 수 있게 된 뒤에 착수한다.

다음 권장 BMAD 명령:
/bmad-spec — 실험 1(class competition)을 "수정 파일 · 성공/rollback 기준 · LODO 검증
프로토콜 · 선결 라벨 요건"까지 기계 계약으로 고정한 뒤 /bmad-quick-dev 로 구현·검증.
```

---

## 부록 A. 산출물 (격리 폴더, 삭제만으로 원상복구)

```
sam6d_ws/ism_accuracy_observation/
  inventory/sam_dataset_inventory.csv
  probes/{dump_all_frames, analyze_top3, analyze_proposal, analyze_hsv,
          analyze_labeled, make_obs_sheets, merge_prior_labels}.py
  frame_dumps/{sam_*_boxes.csv, sam_*_pairs.csv, sam_*_yolo.csv, crops/}
  hsv_features/{sam_*_hsv.npz, template_render_hsv.npz}
  labels/{box_labels_new.csv ★, box_labels_prior.csv, box_labels_merged.csv ★,
          frame_object_labels.csv, human_review_queue.csv}
  contact_sheets/{boxes/ 12장, frames/ 5장, top3_class_confusions/, hsv_success_cases/,
                  hsv_failure_cases/, yolo_proposal_misses/, ambiguous_labels/}
  results/all_frames_object_results.csv        (25,960행)
  results/all_proposal_groups_top3.csv         (12,504행)
  results/yolo_proposal_miss_results.csv       (25,960행)
  results/hsv_feature_results.csv              (224,360행)
  results/hsv_object_summary.csv / hsv_variant_sweep.csv / hsv_real_prototype_lodo.csv
  results/class_top3_summary.{csv,json} / confusion_pairs.csv
  results/dataset_summary.csv / object_summary.csv
  results/proposal_group_gt_rank.csv / own_prompt_proposal_miss.csv
  results/isolated_candidate_dropout.csv / yolo_candidate_stage_stats.csv
  results/labeled_analysis.json
  reports/technical-sam6d-six-dataset-observation-report.md
```

운영 Python·launch·config 무수정 / fake 데이터 없음 / threshold 변경 없음 / git 조작 없음.
실행: `/home/ldh9501/miniconda3/envs/sam_yolo/bin/python ism_accuracy_observation/probes/<script>.py`

## 부록 B. 한계

* **라벨러가 사람이 아니다.** 504건 전부 Claude Vision provisional 라벨. `human_review_queue.csv` 79건이 사람 검수 대기.
* **프레임 단위 가시성 GT 부재** — proposal miss 의 진짜 분모(visible positive)를 세지 못했다. §16의 raw/후처리 분해는 "고립 dropout 410건"이라는 코호트 내부의 분해다.
* **조명 일반화 미검증** — 6개 데이터는 같은 날 같은 사무실. HSV 실사 prototype의 강한 성능이 조명 변화에서 유지되는지는 알 수 없다.
* **fusion 가중치 미최적화** — 균등(0.5/0.5)만 시험했고 객체별·객체군별 가중치, rank fusion, logistic regression은 후속 과제로 남겼다(§6.6 지침대로 관찰만 수행).
* **비승자 후보 라벨 부재** — semantic 선택 실패(정답 박스가 있는데 다른 박스가 선택됨)를 정량화할 수 없다.
* **`unclear` 30건 제외** — 모션블러 등 어려운 케이스가 지표에서 빠져 있어 실제 성능은 보고 수치보다 낮을 수 있다.
* **choco positive 19건 / Dinosaur negative 5건** 등 일부 셀의 표본이 작다. 객체별 결론은 경향으로 읽어야 한다.

---

## 다음 액션 제안

1. **[권장] `/bmad-spec`으로 실험 1(class competition) 고정** — 신호가 가장 강하고(gap 8.7배) 추가 비용이 0이며 나머지 실험의 전제다.
2. **사람 검수 79건 선행** — 모든 정확도 결론이 provisional 라벨 위에 서 있다. 불일치 5% 초과면 라벨 재작성.
3. **가시성 GT 라벨링 착수** — proposal 정책 실험(실험 3)의 선결 조건이자 이번 조사의 유일한 미측정 항목.
4. **조명이 다른 세션 1개 추가 수집** — HSV 실사 prototype의 일반화를 검증할 유일한 방법.
5. **분석 폴더 처리** — `labels/box_labels_merged.csv`(504건)만 저장소 자산으로 승격할지, 폴더 전체(204MB)를 보존할지 결정.

어느 방향으로 진행할까요?
