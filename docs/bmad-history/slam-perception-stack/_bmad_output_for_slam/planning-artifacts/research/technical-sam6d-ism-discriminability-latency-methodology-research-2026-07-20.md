---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments:
  - '_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-pipeline-timing-bottleneck-research-2026-07-06.md'
  - '_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-ism-score-calibration-tp-fp-fn-audit-2026-07-16.md'
  - '_bmad_output_for_slam/planning-artifacts/research/technical-sam6d-ism-render-real-domain-gap-score-research-2026-07-20.md'
workflowType: 'research'
lastStep: 1
research_type: 'technical'
research_topic: 'SAM-6D ISM 변별력 유지 + 프레임당 처리시간 최소화 방법론 설계'
research_goals: '실제 코드 기반 병목·변별력 한계 규명, 후보 구조 A~G 정량 비교, 적용할 최우선 방법론 1개 결정 및 단계별 구현·검증 계획 수립'
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

본 조사는 sam6d_ws ISM(YOLO-World proposal + DINOv2 semantic/appearance + MobileSAM mask) 경로에 대해 **"변별력을 유지하면서 프레임당 처리시간을 최소화하는 구조"** 를 결정하기 위한 기술 조사다. 구현이 아니라 **의사결정**이 산출물이며, 운영 소스는 한 줄도 수정하지 않았다.

방법:

1. **정적 분석** — 실제 entry point 3종(`tools/build_ism_inputs_imu.py`, `src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py`, `yolo_ism_object_n.main`)에서 함수 호출 경로와 tensor shape를 파일:라인 단위로 추적.
2. **신규 실측 3종** — 격리 폴더 `sam6d_ws/ism_methodology_analysis/`(삭제만으로 원상복구)에 read-only 프로브 3개를 작성해 실행. 운영 모듈은 **import만** 했고, NMS 옵션 변경은 프로브 프로세스 내부 monkeypatch로만 적용했다.
3. **기존 실측 재사용** — 2026-07-06 병목 프로브(V1~V8, opt2 A/B, 1,320행 판정 동일성)의 수치를 인용.

표기 규칙: **[실측]** = 본 조사 또는 인용 보고서에서 직접 측정한 값, **[정적]** = 코드에서 확인한 사실, **[추정]** = 계산·추론 결과. GT 라벨이 없는 항목은 정확도 결론을 내리지 않았다.

---

## 1. Executive Summary

**결론 1 — 현재 운영 경로의 시간 병목은 DINOv2도 MobileSAM도 아니라 "10회 반복되는 YOLO-World 패스"와 "객체별 반복 호출"이다.**
PEM 브릿지와 ROS 노드는 `o_n.recognize()`(객체별 루프)를 호출하고, 프롬프트별 YOLO 패스를 10회 돌린다. 201프레임 실측에서 프레임당 median **130.4 ms**(YOLO 63.2 + recognize 66.3). 이미 저장소에 존재하지만 **아무도 호출하지 않는** 배치 경로(`recognize_frame`)와 단일 multi-label YOLO 패스로 바꾸면 median **32.1 ms**(−75.4%), p90 169.5 → 38.8 ms. [실측]

**결론 2 — "42장 전체 patch 비교가 비싸다"는 전제는 거짓이다.**
템플릿 patch 캐시는 이미 디스크에 사전 계산되어 있고, 42장 전체를 **flat matmul 1회 + segment-amax**로 GPU에서 처리하면 검출당 **0.193 ms**다. 현재의 "템플릿 1장 · CPU ragged loop"는 **0.167 ms**. 즉 **1장 → 42장 확대 비용은 검출당 +0.026 ms** 이며, 이는 MobileSAM 1회(10.9 ms)의 0.24%다. [실측]

**결론 3 — 그런데 42장으로 넓혀도 "지금 당장의 정확도 이득"은 작고, 오히려 게이트 운영점을 흔든다.**
CLS-argmax 템플릿이 patch 기준 최적 템플릿과 일치하는 비율은 **42.4%(28/66)** — 사용자 가설대로 "Top-1은 대개 최적이 아니다"가 실측으로 확인됐다. 그러나 CLS-top1의 appe 순위 중앙값은 **1위(=2등)** 이고, `max(42) − appe(cls-top1)` 평균은 **+0.0152**(중앙값 +0.0077, 최대 +0.0669)에 불과하다. 절대 게이트(`appe_gate`)를 그대로 두고 점수만 올리면 이는 2026-07-15에 `appe_blocks=[2,9]`가 **검출 +16.1%(대부분 택배박스)** 로 롤백된 것과 **정확히 같은 실패**를 재현한다. [실측 + 기존 기록]

**결론 4 — 따라서 42장 전체 비교는 "게이트 점수"가 아니라 "랭킹·진단 채널"로 도입한다.**
이는 이미 저장소가 채택해 성공한 패턴이다(`nms_rank_blocks=[2,9]`: 게이트 점수는 그대로 두고 cross-object NMS 승자만 더 좋은 점수로 결정 → 인형 라벨 14/22 → 22/22, 검출 수 불변). 42-view max·argmax·margin은 **임계값이 없는 상대 판단**에만 쓰면 운영점이 움직일 수 없다.

**결론 5 — color/texture 별도 descriptor(HSV/Lab hist, LBP, Gabor)는 지금 추가하지 않는다.**
추가하려는 정보(색)는 이미 DINOv2 **블록 2**에 들어 있고, `get_intermediate_layers`로 **비용 +0.1%** 에 얻을 수 있음이 저장소에서 이미 측정됐다(choco vs 택배박스 AUC 블록2 0.55 / 블록9 1.00 / 블록11 0.93, Dinosaur vs rabbit 블록2 1.00 / 블록11 0.47). 새 descriptor를 붙이면 임계값·조명 민감도·캘리브레이션 데이터가 새로 필요해지는 반면, 같은 정보는 이미 무료로 접근 가능하다.

**최우선 권장안: "1-Pass Batched ISM + 42-view Ranking"** — 상세는 §10.

---

## 2. 실제 현재 ISM 데이터 흐름 (정적 분석)

### 2.1 실행 경로는 하나가 아니라 셋이고, **서로 다르다** [정적]

| entry point | YOLO 호출 | 인식 함수 | DINOv2 forward | MobileSAM 호출 |
|---|---|---|---|---|
| `tools/build_ism_inputs_imu.py:262,295` (PEM 브릿지 = 데이터셋 생성 주력) | **프롬프트당 1회 = 10회/frame** | `o_n.recognize()` 객체별 | proposal당 1회 | 통과 객체당 1회 |
| `src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py:217` (실시간 ROS 노드) | sidecar(별도 프로세스) | `o_n.recognize()` 객체별 | proposal당 1회 | 통과 객체당 1회 |
| `yolo_ism_object_n.py:616` (`main()`, 오프라인 배치) | `set_classes(all)` **1회** | `recognize_frame()` **배치** | **frame당 1회** | **frame당 1회** |

→ **2026-07-06에 검증된 −48% 배치 최적화(`recognize_frame`)는 저장소에 들어 있지만, 운영 진입점 2개는 여전히 옛 per-object 경로를 호출한다.** 이것이 이번 조사의 가장 큰 단일 발견이다.

### 2.2 프레임 처리 순서 (production = `recognize()` 경로) [정적]

```
RGB(640×480 또는 848×480)
→ YOLO-World ×10 pass (imgsz=960, conf=0.02)            # prompt별 독립 패스
→ prompt index별 box 라우팅
→ [객체 루프 ×10]
     score_threshold 필터 → top_k=3 절단
     [proposal 루프 ×≤3]
        crop_resize_pad → [3,224,224]
        dinov2_blocks_forward(model,[crop],...)          # batch=1, forward 1회
          └ get_intermediate_layers(n={2,9,11}) → CLS[384] + patch[256,384]/block
        semantic_score(cls, tcls[42,384], match_topk=5)  # 상위5 코사인 평균
     argmax(sem) 1개 선택 → sem < 0.35 이면 거절
     segment_box(MobileSAM, bbox 1개)                    # image encoder 매 호출 재실행
     masked_query_patches(patch, mask) → q_fg[Nq,384]    # Nq median 88.5
     best_t = argmax(tcls @ cls)                         # 템플릿 1장 선택 (CLS 기준)
     masked_appe_score(q_fg_cpu, tappe[best_t])          # CPU, 템플릿 1장
     masked_appe ≥ appe_gate(0.55) ? accept : reject
→ cross-object NMS (IoU>0.5, rank_appe 기준)
→ mask → RLE → detection_<obj>.json → PEM
```

### 2.3 확인된 세부 사실 [정적]

* **CLS와 patch는 이미 동일 forward 결과다.** `dinov2_blocks_forward`(`yolo_ism_object_n.py:72-90`)가 `get_intermediate_layers(..., return_class_token=True)` 한 번으로 둘 다 반환한다 → **후보 D는 이미 구현되어 있다.** 2026-07-06 이전의 중복 patch forward(14.2 ms/frame)는 제거 완료.
* **템플릿 특징은 완전 사전 계산.** `outputs/yolo_ism_object_n/template_features/<obj>_{cls,appe,appe_b*}.pt` — cls `[42,384]`, appe = 42개 가변 길이 `[Np_i,384]`(Bear: min 106 / median 141 / max 196, 합계 5,889 patch, **fp32 9.05 MB/객체**). 실행 중 재계산 없음. 단 **CPU에 상주**한다. [실측]
* **MobileSAM image encoder는 호출마다 재실행.** `segment_box`는 bbox 1개당 1회. `segment_boxes`(1회 호출, bbox 리스트)는 존재하지만 `recognize_frame`만 사용. 개별 vs 일괄 mask는 IoU=1.0000으로 검증 완료(2026-07-06).
* **mask는 증거가 아니라 산출물이다.** `build_ism_inputs_imu.py`가 `mask_to_rle_pytorch(r["mask"])`로 PEM 입력 seg를 만들고, ROS 노드도 `r["mask"]`를 PEM에 넘긴다 → **수락된 객체에 대해 MobileSAM을 건너뛰는 fast path는 원천적으로 불가능**(§6 후보 G 참조).
* **top_k는 3(config `defaults.top_k`), score_threshold는 0.02(기본) / Bear 0.35 / Rabbit·Dinosaur 0.30.** 후보 수는 "3~5"가 아니라 **객체별 임계값 통과분 중 상위 3개**다.
* **42장은 고정값이 아니라 렌더 산출물.** `cam_poses_level0.npy`가 42 pose이고 12개 template dir 전부 `rgb_*.png` 42장. 코드에는 42가 하드코딩되어 있지 않으며 `_template_paths()`가 세는 대로 따라간다 → 템플릿 수 증가는 코드 변경 없이 가능하다.

---

## 3. 현재 구조의 시간 병목 [실측]

### 3.1 신규 실측 — production 경로 전체 (201 steady frames, 4 bag: SAM_loop1/loop2/occlusion, sam_110633)

측정: `ism_methodology_analysis/scripts/bench_frame_pipeline.py`, warm-up 3 frame 제외, 모든 stage 경계에 `torch.cuda.synchronize()`, RTX PRO 6000 Blackwell / torch 2.12+cu130 / `sam_yolo` env.

| 항목 | P0 = 현재 production | P1 = 1-pass + 배치 | 변화 |
|---|---|---|---|
| **frame total (median)** | **130.41 ms** | **32.05 ms** | **−75.4%** |
| frame total (mean) | 125.50 ms | 29.21 ms | −76.7% |
| frame total (p90) | 169.49 ms | 38.77 ms | −77.1% |
| YOLO-World | 63.16 ms (10 pass) | 7.08 ms (1 pass) | −88.8% |
| recognize(인식 전체) | 66.29 ms | 24.94 ms | −62.4% |
| DINOv2 forward 호출/frame | 8.78 (batch=1 × 8.78회) | 9.96 crop → **forward 1회** | 호출 8.78 → 1 |
| MobileSAM 호출/frame | 2.98 | 0.83 | −72% |

### 3.2 단가 실측 (66 detections, `probe_template_scope.py`)

| stage | median | mean | p90 |
|---|---|---|---|
| YOLO-World (10 pass) | 65.36 ms | 65.70 | 68.09 |
| DINOv2 (객체 1개분, proposal ≤3, batch=1) | 3.88 ms | 4.89 | 9.86 |
| MobileSAM (bbox 1개, encoder 포함) | 10.92 ms | 11.09 | 12.22 |
| appearance, 템플릿 1장, CPU ragged (현재) | 0.167 ms | 0.161 | 0.202 |
| appearance, **42장 전체, GPU flat matmul** | **0.193 ms** | 0.198 | 0.221 |
| preprocess(normalize_rgb, frame 1회) | 0.58 ms | 0.59 | 0.66 |

### 3.3 병목 순위 재확정

2026-07-06 프로브는 `main()`(=단일 YOLO 패스) 기준이라 병목 1·2위가 MobileSAM(35.9%)·DINOv2 cls(26.1%)였다. **운영 경로는 YOLO를 10번 돌기 때문에 순위가 다르다:**

| 순위 | stage | production frame당 | 비중 |
|---|---|---|---|
| **1** | YOLO-World ×10 pass | 63.2 ms | **48%** |
| 2 | MobileSAM (2.98 call × 10.9) | ~32.5 ms | 25% |
| 3 | DINOv2 (8.78 forward × ~3.5) | ~30 ms | 23% |
| 4 | appearance similarity (CPU) | ~0.5 ms | 0.4% |
| 5 | semantic matmul `[42,384]@[384]` | ~0.03 ms | <0.1% |

**"템플릿 매칭이 병목"이라는 직관은 실측으로 반증된다.** semantic·appearance 유사도 연산은 합쳐서 전체의 0.5% 미만이다. 병목은 전부 **모델 호출 횟수**에 있다.

### 3.4 왜 YOLO 10패스인가 (그리고 왜 고칠 수 있는가) [실측]

`build_ism_inputs_imu.py:173-183`의 주석대로, `set_classes(전체)` 공유 패스는 박스를 잃는다. ultralytics NMS가 박스당 **argmax 클래스 1개만** 남기기 때문에, 흰 토끼가 "brown bear doll"로 라벨링되면 토끼 프롬프트 후보 목록에서 사라진다.

`bench_yolo_prompt_passes.py`(27 steady frames, 10 prompt, imgsz 960, conf 0.02):

| 변형 | latency median | 10-pass 기준 박스 recall |
|---|---|---|
| A. 프롬프트별 10 pass (현재) | 59.83 ms | (기준) 94 boxes |
| B. `set_classes(all)` 공유 1 pass | 6.32 ms | **82.98%** ← 문서화된 손실 재현 |
| C. **공유 1 pass + `multi_label=True` NMS** | **6.30 ms** | **98.94%** |

`multi_label=True`는 `ultralytics.utils.nms.non_max_suppression`이 이미 지원하는 인자다(8.4.64 확인). 박스 1개가 여러 프롬프트 라벨을 동시에 가질 수 있게 되어 argmax 붕괴가 사라지고, **비용은 10패스의 1/9.5**. 단 C는 기준 대비 **초과 박스 55개**(대부분 저 conf)를 함께 만들어 낸다 → §11의 리스크 항목.

---

## 4. 현재 구조의 변별력 한계

### 4.1 CLS-argmax 템플릿은 patch 기준 최적 템플릿이 **아니다** [실측]

66 detections(6객체, 3 bag):

| object | n | cls-top1 == appe-argmax | appe 정렬에서 cls-top1의 순위(중앙, 0=최적) | max42 − appe(cls-top1) 평균 | 현재 appe 평균 |
|---|---|---|---|---|---|
| Bear | 23 | **95.7%** | 0.0 | +0.0007 | 0.777 |
| Febreze_high | 6 | 33.3% | 4.0 | +0.0140 | 0.504 |
| Sauce_high | 3 | 33.3% | 7.0 | +0.0185 | 0.436 |
| choco_hazelnut_high | 5 | 20.0% | 1.0 | +0.0041 | 0.542 |
| milk | 23 | **4.3%** | 1.0 | +0.0318 | 0.701 |
| saffron | 6 | 16.7% | 3.0 | +0.0161 | 0.621 |
| **전체** | **66** | **42.4%** | **1.0** | **+0.0152** (중앙 +0.0077, 최대 +0.0669) | |

읽는 법:

* **가설은 맞다.** 절반 이상(57.6%)에서 CLS가 고른 템플릿은 patch 기준 최적이 아니다. milk는 거의 항상(95.7%) 틀린 뷰를 고른다.
* **그러나 피해는 작다.** cls-top1의 appe 순위 중앙값이 1(=2등), 평균 1.53이다. 즉 CLS는 "최적 뷰"는 못 집어도 "거의 최적 뷰"는 집는다. 점수 손실 평균 0.015.
* **원인 해석**: 42 pose는 구면을 촘촘히 덮으므로 인접 뷰들의 patch 점수가 서로 매우 가깝다. CLS(전역 형상 편향)와 patch(국소 텍스처)는 순서를 다르게 매기지만 상위권 집합은 겹친다.
* **위험한 케이스**는 평균이 아니라 최대값 쪽이다(최대 +0.0669). 게이트 근처(0.55)에서 0.067은 판정을 뒤집기 충분하다. 실제로 66건 중 1건이 `max42`로 fail→pass 전환됐다.

### 4.2 semantic과 appearance는 독립 증거가 아니다 [정적 + 기존 실측]

둘 다 **같은 DINOv2 ViT-S/14의 같은 forward**에서 나온다(CLS = 블록11 class token, appe = 블록11 patch token, `get_intermediate_layers` 1회). 즉 2단 게이트는 서로 다른 두 증거의 교집합이 아니라 **하나의 표현을 두 방식으로 요약한 것**이다. 2026-07-16 캘리브레이션 감사에서 인형류 semantic AUC가 Bear 0.57 / Dinosaur 0.43(무작위 이하)로 무너진 것과, 블록11 appe가 Dinosaur vs rabbit AUC 0.47로 같이 무너진 것은 이 상관성의 직접 증거다. **진짜 독립 증거를 원한다면 다른 층(블록2 = 색)이나 DINOv2 밖(depth/기하)에서 가져와야 한다.**

### 4.3 단일 슬롯(객체당 1 detection)이 FN의 구조적 원인 [정적 + 기존 실측]

`recognize()`는 객체당 argmax-semantic 박스 **1개만** 살린다. 실제 객체와 유사물이 같이 제안되면 하나는 게이트를 통과할 수 있었음에도 슬롯을 뺏겨 FN이 된다(2026-07-15 감사: Rabbit FP=0 / **FN=258**). `recognize_multi()`가 대안으로 존재하지만 `--multi`는 기본 OFF이고, 측정 결과 choco 택배박스 FP만 +9 늘어 기본값으로 거부됐다.

본 조사에서 이 결함이 다시 관측됐다: YOLO 후보를 늘리면(multi_label) 오히려 검출이 **줄어드는** 경우가 생긴다. 201프레임에서 P0 412 → P1 401 object-frame, 손실 내역 Rabbit 15 / Febreze 4 / Sauce 2 / Dinosaur 2 / 기타 4, 획득 milk 5 / choco 4 / Mugcup 3 / 기타 4. **후보가 늘면 단일 슬롯 경쟁이 심해져 진짜 객체가 밀린다.** 후보 확대와 슬롯 정책은 반드시 함께 다뤄야 한다.

### 4.4 2단 hard gate의 성질 [정적 + 기존 실측]

* semantic 게이트(0.35)는 **전역**, appe 게이트(0.55)는 **객체별 스케일**을 갖는 점수에 걸린 절대 임계다. config 주석이 기록하듯 appe 중앙값은 saffron 0.750 ~ choco 0.611로 객체마다 다르고, "모든 객체에 맞는 단일 값이 없다"가 이미 3회 롤백으로 확인됐다.
* hard gate의 실패 모드는 **비대칭**이다: semantic에서 떨어지면 appe를 볼 기회조차 없다(FN 회복 불가, 2026-07-16 감사에서 "FN은 sem으로 회복 불가" 확인). 반대로 appe만 낮은 진짜 객체는 조용히 사라진다(saffron 0.72 게이트 시도 → 제거된 69건 중 2/3가 진짜 → 롤백).
* **그럼에도 게이트를 유지해야 하는 이유**: 점수 융합으로 바꾸면 새 가중치·새 임계값이 필요하고, 그 캘리브레이션에 쓸 라벨이 저장소에 저장되어 있지 않다(2026-07-16 감사의 1,521건 라벨은 CSV로 보존되지 않았음 — 본 조사에서 파일 검색으로 확인).

---

## 5. 사용 가능한 특징별 분석

"계산비용"은 검출 1건 기준. 캐시 가능성 = 템플릿 쪽을 미리 계산해 둘 수 있는가.

| 특징 | 판별력 | 자세 민감도 | 조명 민감도 | 배경 영향 | 계산비용 | 캐시 |
|---|---|---|---|---|---|---|
| **DINOv2 CLS (블록11)** | 범주 수준 강함, 인스턴스 수준 약함(형상 편향 — 상자 둘을 못 가름, AUC Bear 0.57/Dino 0.43) | 중(전역 pooling이 일부 흡수) | 낮음 | **높음**(bbox 배경 포함, mask 미적용) | 이미 지불(공유 forward) | ✅ `[42,384]` = 64 KB/객체 |
| **mask 내부 patch (블록11)** | 텍스처 기반, 동색 FP 분리에 강함(choco vs 택배 AUC 0.93) | **높음**(뷰마다 다름 → 템플릿 42장 필요) | 중 | **낮음**(mask로 배경 제거) | 이미 지불 | ✅ 9.05 MB/객체 fp32 |
| **patch (블록2)** | **색** 신호. Dinosaur vs rabbit AUC 1.00, milk vs 택배 0.95 | 낮음 | **높음**(노출·화이트밸런스) | 낮음(mask 적용) | **+0.1%** (동일 forward에서 norm만 추가 적용) | ✅ 이미 `_appe_b2-9.pt`로 생성됨 |
| **mask 형태(area/aspect/solidity)** | 약함·뷰 의존 | **매우 높음** | 없음 | 낮음 | ~0.1 ms CPU | 부분(뷰별 silhouette) |
| **RGB/HSV/Lab histogram** | 색만. 블록2와 정보 중복 | 낮음 | **높음** | mask 필요 | ~0.1–0.3 ms CPU | ✅ |
| **color moments** | histogram의 저차 요약, 더 약함 | 낮음 | 높음 | mask 필요 | ~0.05 ms | ✅ |
| **LBP** | 국소 텍스처, 저해상도·블러에 취약 | 중 | 중(단조 조명 불변) | mask 필요 | ~0.5–1 ms CPU | ✅ |
| **Gabor** | 방향성 텍스처. 회전 비불변 | **높음** | 중 | mask 필요 | ~2–5 ms CPU (필터뱅크) | ✅ |
| **YOLO conf** | 프롬프트 적합도. 이미 사용(임계·정렬) | — | — | — | 0 | — |
| **semantic margin(1위−2위 후보)** | **미사용 자원**. 슬롯 경쟁의 확신도 | — | — | — | 0 | — |
| **42-view appe margin(max−mean)** | **미사용 자원**. 자기정규화(객체별 임계 불필요) | 자체 상쇄 | 중 | 낮음 | +0.026 ms | ✅ |
| **mask area / bbox area** | 제안 품질 sanity check | 중 | 없음 | — | 0(이미 계산됨, `mask_area_px`) | — |
| **MobileSAM stability/IoU score** | ultralytics `SAM` 래퍼가 노출하지 않음 | — | — | — | — | — |

### 핵심 판단: color/texture는 "추가"가 아니라 "이미 있는 것을 꺼내 쓰는" 문제다

저장소는 이미 이 실험을 했다. 501건 손라벨 기준 블록별 AUC(실 객체 vs 그 객체의 FP):

| object | 블록2(색) | 블록9 | 블록11(현 게이트) |
|---|---|---|---|
| choco vs 택배박스 | 0.55 | **1.00** | 0.93 |
| saffron vs 택배박스 | 0.83 | 0.89 | 0.85 |
| milk vs 택배박스 | **0.95** | 0.80 | 0.88 |
| Febreze vs saffron | **0.97** | 0.94 | 0.91 |
| Bear vs rabbit/dino | **0.92** | 0.49 | 0.56 |
| Dinosaur vs rabbit | **1.00** | 0.66 | 0.47 |

→ 손수 만든 HSV histogram이 제공할 정보(색)는 **블록2가 이미, 그것도 mask 정합·스케일 정합된 형태로, 추가 forward 없이(+0.1%)** 제공한다. 새 descriptor는 (a) 새 임계값, (b) 새 정규화, (c) 조명/노출 민감도, (d) 캘리브레이션 라벨을 새로 요구하는 반면 얻는 정보는 중복이다. **채택하지 않는다.** (완전 배제가 아니라 Phase 4의 조건부 실험으로 유보 — §12.)

---

## 6. 후보 ISM 구조 비교

정량 항목은 프레임당 median 기준. "DINOv2 fwd"는 **커널 런치 횟수**(배치는 1회로 계산).

| | A. 현행 최적화 | B. semantic Top-K patch | C. 42-view 전체 patch | D. mask-conditioned 단일 추론 | E. color/texture early reject | F. mask 형상 보조 | G. score fusion |
|---|---|---|---|---|---|---|---|
| 변별력 | 변화 없음 | 소폭↑(Δappe +0.008~0.015) | 소폭↑(동, 최대 +0.067) | 변화 없음 | 불명(검증 필요) | 낮음(뷰 의존) | 잠재적↑ |
| Recall 위험 | 없음 | 낮음(게이트 재보정 필요) | **중**(점수 상향 → 게이트 슬랙) | 없음 | **높음**(잘못된 조기 거절) | 중 | 중 |
| Latency | **−75%** [실측] | +0.01 ms | **+0.026 ms** [실측] | 이미 반영됨(0) | −0~3 ms(후보 감소분) | +0.1 ms | +0 |
| GPU 호출 | fwd 8.78→1, SAM 2.98→0.83 | 불변 | 불변 | 불변 | fwd 소폭↓ | 불변 | 불변 |
| 메모리 | 0 | +9 MB/객체(GPU) | +9 MB/객체(GPU, 10객체 90 MB) | 0 | 0 | 0 | 0 |
| 구현 난이도 | **낮음**(이미 있는 함수 호출로 교체) | 낮음 | 낮음 | **0(이미 구현됨)** | 중 | 낮음 | 높음 |
| 설명 가능성 | 불변 | ↑(어느 뷰가 이겼는지) | **↑↑**(42-view 프로파일) | 불변 | ↓ | ↑ | ↓(가중치 불투명) |
| 확장성(템플릿 T) | O(1) | O(K) | **O(T)**, T=42→168도 GPU 0.5 ms 미만 [추정] | — | — | — | — |
| 데이터 의존성 | **없음** | 게이트 재보정 | 게이트 재보정 | 없음 | 임계값 신규 | 임계값 신규 | **라벨 필요(부재)** |

### 개별 판정

**A (현행 최적화) — 채택, 최우선.** "최적화"라 부르기도 민망하다: 이미 검증된 배치 함수 `recognize_frame`이 존재하고 운영 경로만 호출하지 않는다. 2026-07-06에 4 bag × 330 object-row = **1,320행 전수 비교에서 decision·accepted·selected_template_id 차이 0, |Δscore| 최대 0.000000** 으로 판정 동일성이 증명됐다. **정확도 리스크가 수학적으로 0인 −62% 개선**을 방치 중이다.

**B (Top-K 템플릿) — 불채택(C의 열등 부분집합).** 실측상 K=1과 K=42의 비용 차이가 GPU flat matmul 기준 0.081 ms → 0.178 ms(nq=256)에 불과하다. K를 튜닝할 이유가 없다. K는 "42가 비싸다"는 전제 하에서만 의미 있는 하이퍼파라미터이며 그 전제가 실측으로 반증됐다.

**C (42-view 전체) — 조건부 채택(랭킹·진단 채널로).** 비용 근거:

| 구현 | nq=64 | nq=128 | nq=256 |
|---|---|---|---|
| CPU ragged loop, K=1 (**현재**) | 0.061 ms | 0.080 ms | 0.117 ms |
| CPU ragged loop, K=42 (순진한 확장) | 2.950 ms | 3.511 ms | 4.764 ms |
| CPU flat matmul, K=42 | 0.429 ms | 0.631 ms | 1.146 ms |
| **GPU flat matmul, K=42 (권장)** | **0.106 ms** | **0.152 ms** | **0.178 ms** |

연산량은 nq=256·5,889 patch에서 **1.16 GFLOP**. Blackwell에서 이는 0.18 ms — 즉 **파이썬 루프가 비용이었지 행렬곱이 비용이 아니었다.** 실제 검출 nq 중앙값은 88.5이므로 운영 비용은 0.193 ms [실측]. 수치 동일성도 확인: flat matmul의 `t_top1` 성분과 기존 ragged 점수의 차이 최대 **0.000000**.

**D (mask-conditioned 단일 추론) — 이미 구현됨, 추가 작업 없음.** `dinov2_blocks_forward` 1회에서 CLS+patch 동시 획득, mask는 `masked_query_patches`가 224 grid에 AvgPool로 정렬. 중복 forward는 2026-07-06에 제거됐고 patch는 승자만 device→host 전송한다. **저장 tensor는 proposal당 `[256,384]` fp32 = 393 KB, 프레임당 10 proposal이어도 3.9 MB.**

**E (color/texture early rejection) — 불채택.** ① 정보가 블록2와 중복(§5). ② 절감 대상이 잘못됐다 — DINOv2는 이제 프레임당 forward 1회(배치)이므로 후보 3개를 2개로 줄여도 **배치 크기만 줄 뿐 호출은 그대로**다. 절감 상한이 P1 기준 수 ms 미만. ③ 조기 거절은 회복 불가능한 FN을 만든다. 저장소는 이미 "임계값으로 진짜/가짜를 가르려다 3회 롤백"한 이력이 있다.

**F (mask 형상 보조) — 부분 채택(품질 sanity check로만).** 관측각에 따라 silhouette가 크게 변하므로 형상 게이트는 부적절하다. 다만 `mask_area_px`는 **이미 계산되어 CSV에 기록 중**이며 비용 0이다. 이를 "0에 가까운 mask" 또는 "bbox를 거의 꽉 채우는 mask(=배경 흡착)" 같은 **명백한 실패의 sanity check**로만 쓰는 것은 안전하다. 판별 점수로 승격하지 않는다.

**G (score fusion) — 현 단계 불채택.** 라벨이 없다. 로지스틱/MLP는 물론이고 rule-based 가중치조차 가중치를 정할 근거가 필요하며, 이 저장소에서 "눈대중으로 정한 임계"는 세 번 롤백됐다(choco 0.59, saffron 0.72, milk 0.63 — 마지막에는 롤백 자체가 틀렸음이 밝혀졌다). fusion으로 가려면 **먼저 라벨 세트를 CSV로 영구 보존**하는 작업이 선행되어야 한다.

### 추가 후보 H (본 조사에서 제안) — **1-Pass Batched ISM**

A + C(랭킹 한정) + YOLO 단일 multi-label 패스의 결합. §10에서 상술.

---

## 7. Hard Gate vs Score Fusion 분석

| | Hard gate (현재) | Weighted score fusion | Learned fusion |
|---|---|---|---|
| 필요한 임계값 수 | 2 (sem 0.35 / appe 0.55) + 객체별 오버라이드 | 가중치 6개 + 최종 임계 1개 | 모델 파라미터 + 임계 1개 |
| 필요한 라벨 | 없음(현 운영값은 경험적) | **있어야 함** | **많이 있어야 함** |
| 실패 모드 | 조용한 FN, 순차 의존 | 한 점수의 스케일 이탈이 전체를 오염 | 과적합(현재 6객체·6데이터셋 규모에서 심각) |
| 롤백 용이성 | 값 하나 되돌리면 끝 | 가중치 조합 전체 재검증 | 재학습 |
| 설명 가능성 | **높음**(어느 게이트에서 떨어졌는지 CSV에 기록됨) | 중 | 낮음 |

**판정: hard gate 유지.** 근거는 이론이 아니라 이 저장소의 이력이다 — 절대 임계 자체가 아니라 **"라벨 없이 임계/가중치를 정하는 행위"** 가 반복 실패의 원인이었다. 대신 다음 두 가지를 추가한다.

1. **랭킹은 게이트와 분리한다.** 이미 `nms_rank_blocks`가 성공시킨 패턴. 랭킹에는 임계가 없으므로 운영점이 움직일 수 없고, 따라서 "더 좋은 점수"를 무료로 쓸 수 있다.
2. **점수는 전부 기록한다.** 42-view 프로파일(max/mean/margin/argmax), semantic margin, mask_area_ratio를 CSV에 남기되 **판정에는 쓰지 않는다.** 이것이 나중에 fusion을 정당하게 도입할 유일한 경로다(라벨 + 저장된 점수 = 캘리브레이션 가능).

---

## 8. Template Patch 비교 범위 분석

| 범위 | 검출당 비용(GPU flat) | cls-top1 오선택 영향 | 게이트 운영점 |
|---|---|---|---|
| 1장 (CLS-argmax, 현재) | 0.167 ms (CPU ragged) | 57.6%에서 준최적 뷰 사용, 평균 −0.015 손실 | 현행 유지 |
| Top-3 | 0.082 ms | 대부분 회복(순위 중앙값 1) | 상향 이동 |
| Top-5 | 0.090 ms | 거의 전부 회복 | 상향 이동 |
| Top-10 | 0.118 ms | 전부 회복(최대 순위 15는 예외) | 상향 이동 |
| **42장 전체** | **0.178~0.193 ms** | 정의상 완전 | **상향 이동(평균 +0.015, 최대 +0.067)** |

**결정: 42장 전체를 계산하되, 게이트에는 기존 정의(CLS-top1 템플릿 점수)를 그대로 쓴다.**

이유:
* 42장 전체 계산은 사실상 무료이고(+0.026 ms) 부분집합(Top-K)은 얻는 게 없다. 42-view 프로파일 전체를 손에 쥐면 argmax·margin·분산 같은 파생 신호가 공짜로 따라온다.
* 그러나 `max(42)`를 **게이트 점수로 승격하면** 모든 점수가 올라가 고정 임계 0.55가 느슨해진다. 이는 2026-07-15 `appe_blocks=[2,9]` 사건과 같은 실패 형태다(검출 5,476→6,357, +16.1%, 증가분 대부분 택배박스). 66건 표본에서 관측된 flip은 1건뿐이지만, **표본이 작고 방향이 항상 "느슨해지는 쪽"** 이라는 점이 핵심 위험이다.
* 반면 랭킹(cross-object NMS 승자 결정)에 쓰면 임계가 없으므로 검출 수가 바뀔 수 없다 — 바뀌면 그건 버그다. `nms_rank_blocks`에서 이미 검증된 안전한 도입 경로다.

---

## 9. Color/Texture 특징 채택 여부

**결론: 신규 hand-crafted color/texture descriptor(HSV/Lab histogram, color moments, LBP, Gabor)는 채택하지 않는다.**

1. **정보 중복** — 필요한 색 정보는 DINOv2 블록2 patch token에 있고, 501건 손라벨 AUC에서 색 기반 분리가 필요한 전 객체(milk 0.95, Febreze 0.97, Bear 0.92, Dinosaur 1.00)에 대해 블록11보다 우수하다.
2. **비용 우위** — 블록2는 `get_intermediate_layers`의 같은 forward에서 norm만 추가 적용해 얻는다(**+0.1%**, 측정됨). 별도 forward였다면 +104%였을 것이다. LBP·Gabor는 CPU에서 0.5~5 ms로 오히려 더 비싸다.
3. **정합 우위** — 블록2는 이미 mask·224 crop·정규화 파이프라인에 정합되어 있다. histogram은 bin 수, 색공간, 정규화, mask 처리, 거리척도를 새로 정해야 하고 그 각각이 새 하이퍼파라미터다.
4. **안정성** — histogram/moments는 노출·화이트밸런스 변화에 직접 노출된다. 본 데이터셋은 D435i/D455 자동노출 환경이며, 2026-07-20 도메인갭 조사에서 render-real 색 차이가 이미 문제로 지목됐다.
5. **단, 색이 필요 없다는 뜻은 아니다.** 색은 채택하되 **경로가 블록2**라는 것이다. 현 config는 이미 `nms_rank_blocks=[2,9]`로 랭킹에만 색을 넣었고, 게이트(`appe_blocks=[11]`)에는 넣지 않았다 — 이 분리가 옳다.

**유보 조건**: 블록2 랭킹으로도 분리되지 않는 잔여 FP 코호트(예: Bear — 어느 블록에서도 FP 제거율 0%)가 라벨로 특정되면, 그때 해당 코호트에 한해 color descriptor를 재검토한다.

---

## 10. 최우선 권장 ISM 구조 — **1-Pass Batched ISM + 42-view Ranking**

### 10.1 데이터 흐름

```
입력 RGB (1 frame)
│
├─[1] normalize_rgb 1회/frame                          [캐시 X, 0.58 ms 실측]
│
├─[2] YOLO-World 1 pass, set_classes(전체 프롬프트),
│     multi_label=True NMS, imgsz=960, conf=min(객체별 임계)
│     → 박스당 여러 프롬프트 라벨 허용                   [7.1 ms 실측, 10 pass 대비 −89%]
│
├─[3] 객체별 후보 확정: score_threshold(객체별) → top_k=3 절단
│     후보 수: 프레임당 crop 합계 median 10             [특징: YOLO conf — 임계·정렬에만 사용]
│
├─[4] DINOv2 ViT-S/14 **배치 forward 1회** (전 객체·전 proposal crop stack)
│     get_intermediate_layers(n={2,9,11}, return_class_token=True)
│     → CLS[N,384] + patch[N,256,384] @블록 2/9/11      [+0.1% 비용, 커널 런치 N→1]
│
├─[5] semantic score = topk5_mean(cls · tcls[42,384])   [템플릿 42장, GPU 캐시, 0.03 ms]
│     └ 게이트 1: sem < similarity_threshold(0.35) → 거절
│     └ 객체별 argmax(sem)로 1 박스 선택 + semantic margin 기록(판정 미사용)
│
├─[6] MobileSAM **1회 호출**, 통과 박스 전부를 bboxes 리스트로
│     → image encoder 1회 재사용                        [IoU=1.0000 동일성 기검증, 호출 2.98→0.83]
│     ※ 생략 불가: mask는 PEM 입력(RLE seg) 산출물이다
│
├─[7] mask → patch grid 정합(AvgPool14, cover>0.5) → q_fg[Nq,384], Nq median 88.5
│
├─[8] appearance = q_fg @ flat_templates[5889,384].T  → segment-amax → per-view[42]
│     템플릿 캐시는 **GPU 상주 fp16/fp32**, 42장 전체    [0.193 ms 실측, +0.026 ms]
│     ├ gate_score  = per_view[argmax_cls(t)]           ← 정의·값 모두 현행과 동일(Δ=0.000000)
│     ├ rank_score  = per_view.max()  (블록 [2,9] 평균)  ← 임계 없음, 랭킹 전용
│     └ 기록 전용   = argmax_view, mean42, margin=max−mean, sem_margin, mask_area_ratio
│     └ 게이트 2: gate_score < appe_gate(0.55, 객체별 오버라이드) → 거절
│
├─[9] cross-object NMS (IoU>0.5), 승자 = rank_score 최대
│
└─[10] 수락 → mask RLE + bbox + 점수 전부 CSV/JSON → PEM
```

### 10.2 각 단계 명세

| 단계 | 사용 특징 | 계산 시점 | 캐시 | 후보/템플릿 수 | 점수 | early exit | gate/fusion |
|---|---|---|---|---|---|---|---|
| 2 | YOLO conf | frame 1회 | 텍스트 임베딩(set_classes 시 1회) | — | conf | — | — |
| 3 | conf | frame | — | 객체당 ≤3 | — | 후보 0 → 객체 스킵 | 임계 |
| 4 | CLS+patch(블록2/9/11) | frame 1회(배치) | — | crop ~10 | — | — | — |
| 5 | CLS | frame | 템플릿 CLS **GPU** `[42,384]` | 42 | topk5 평균 코사인 | **sem 미달 → MobileSAM 미실행** | hard gate |
| 6 | — | frame 1회 | — | 통과 박스 전부 | — | — | — |
| 8 | mask 내부 patch | 검출당 | 템플릿 patch **GPU** flat `[5889,384]` | **42 전체** | segment-amax → per-view | 없음 | hard gate(현행 정의) |
| 9 | rank_score | frame | — | — | max over 42, 블록[2,9] | — | 랭킹(임계 없음) |

### 10.3 권장 이유

**변별력이 나빠지지 않는 이유(그리고 어디서 좋아지는가)**
게이트 점수의 **정의와 수치를 바꾸지 않는다**(flat matmul의 해당 성분과 기존 값의 차이 최대 0.000000 — 실측 검증). 따라서 FP/FN 수는 원리적으로 불변이다. 개선은 전부 **임계 없는 결정**에서 온다: cross-object NMS 승자가 "CLS가 우연히 고른 뷰"가 아니라 "42뷰 최적 매칭"으로 결정된다. 이는 `nms_rank_blocks`가 인형 라벨을 14/22 → 22/22로 고친 것과 같은 메커니즘이며 검출 수는 바뀌지 않았다.

**빨라지는 이유**
모델 호출 횟수를 줄인다: YOLO 10회 → 1회, DINOv2 커널 런치 8.78회 → 1회(배치), MobileSAM 2.98회 → 0.83회(프레임 1회 호출). 특징 개수를 줄인 것이 아니라 **호출 구조**를 바꿨다. 실측 median 130.4 → 32.1 ms.

**42/Top-K/1장 중 무엇이 옳은가**
계산은 **42장 전체**(무료·정보 최대), 게이트는 **1장(CLS-top1) 정의 유지**(운영점 보존), 랭킹은 **42장 max**. Top-K는 존재 이유가 없다.

**hard gate vs fusion**
hard gate 유지. 라벨이 없는 상태에서 가중치를 정하는 것은 이 저장소에서 이미 3회 실패한 행위다. 대신 fusion에 필요한 모든 원료(42-view 프로파일, margin, mask ratio)를 지금부터 CSV에 축적한다.

**색/텍스처**
신규 descriptor 제외, 블록2 경로 채택(랭킹 한정, 이미 config에 존재).

---

## 11. 예상 Latency 및 정확도 변화

### 11.1 정량 (201 steady frames, 4 bag) [실측]

| 지표 | 현재(P0) | 권장(H) | 변화 |
|---|---|---|---|
| frame median | 130.41 ms | 32.05 ms (+0.03/검출) | **−75.4%** |
| frame p90 | 169.49 ms | 38.77 ms | −77.1% |
| 등가 FPS | 7.7 | 31.2 | ×4.1 |
| YOLO 패스/frame | 10 | 1 | −90% |
| DINOv2 forward 호출/frame | 8.78 | **1** | −89% |
| MobileSAM 호출/frame | 2.98 | 0.83 | −72% |
| patch 비교 범위 | 템플릿 1/42 | **42/42** | ×42 커버리지, +0.026 ms |
| patch 비교 비용/검출 | 0.167 ms(CPU) | 0.193 ms(GPU) | +16% |
| GPU 템플릿 캐시 | 0(CPU 상주) | 9.05 MB × 객체 수(10객체 ≈ 90 MB fp32 / 45 MB fp16) | 97 GB 중 0.09% |

### 11.2 FP/FN 위험 [실측 + 판단]

* **Phase 1(배치화만)** — 판정 변화 **0**. 1,320행 전수 동일성이 이미 증명됨. 리스크 없음.
* **Phase 2(YOLO 단일 multi-label 패스)** — **가장 큰 위험 지점.** 박스 recall은 98.9%로 회복되지만 초과 박스 55/94가 생기고, 201프레임에서 수락 object-frame 412 → 401(손실 Rabbit 15 · Febreze 4 · Sauce 2 · Dinosaur 2 · 기타 4 / 획득 milk 5 · choco 4 · Mugcup 3 · 기타 4, 프레임 판정 일치 163/201). **원인은 YOLO가 아니라 "객체당 슬롯 1개"** — 후보가 늘면 argmax-semantic 경쟁에서 진짜 객체가 밀린다. Phase 2는 슬롯 정책 수정과 **묶어서만** 출시한다.
* **Phase 3(42-view 랭킹)** — 검출 수 불변이 성공 기준(변하면 버그). 라벨 승자 변경만 발생.
* **게이트 점수를 max42로 바꾸는 변형** — 채택 금지. 66건 중 1건 fail→pass가 이미 관측됐고 방향이 항상 완화 쪽이다.

### 11.3 템플릿 수 확장성 [추정, 실측 외삽]

GPU flat matmul은 `[Nq,384]×[384,T·140]`. T=42 → 0.178 ms(nq=256). T=168(4배)이면 4.6 GFLOP → **0.5 ms 미만**, 캐시 36 MB/객체. **템플릿을 4배로 늘려도 프레임 예산의 1.5%.** 즉 도메인갭 대응으로 뷰를 늘리는 선택지가 시간 측면에서 열려 있다 — 현재 구조에서는 이 사실이 보이지 않았다.

---

## 12. 단계별 구현·검증 계획

### Phase 1 — 운영 진입점을 배치 경로로 전환 (리스크 0, 이득 최대)

* **목표**: `build_ism_inputs_imu.py` / `sam6d_multiobject_node.py`가 `recognize()` 대신 `recognize_frame()`을 호출.
* **수정 예상 파일**: `tools/build_ism_inputs_imu.py`(~290-296), `src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py`(~213-219). `yolo_ism*.py`는 **무수정**.
* **측정 지표**: frame latency median/p90, DINOv2 호출/frame, MobileSAM 호출/frame, 그리고 **detection JSON byte 동일성**.
* **성공 기준**: 4 bag × 전 프레임에서 detection JSON(RLE 포함) **byte 동일**, latency −50% 이상.
* **Rollback**: JSON 차이 1건이라도 발생 시 즉시 원복(한 줄 되돌리기). `--multi` 경로는 `recognize_multi`를 유지해야 하므로 별도 분기.

### Phase 2 — YOLO 10패스 → 1패스(multi_label) + 슬롯 정책

* **목표**: 프롬프트별 패스 제거. **반드시** 후보 증가에 대한 슬롯 대책과 동시 적용.
* **수정 예상 파일**: `tools/build_ism_inputs_imu.py`(YOLO 초기화/predict 블록), 커스텀 postprocess 래퍼 1개 신규(운영 파일 무수정 원칙 유지 시 별도 모듈).
* **측정 지표**: 박스 recall(10패스 기준), 객체별 수락 수 증감, Rabbit/Dinosaur/Bear 라벨 정확도, latency.
* **성공 기준**: 박스 recall ≥ 98%, **객체별 수락 수 감소 0**, latency −40% 이상.
* **Rollback 기준**: 어느 객체든 수락 수가 5% 이상 감소하면 중단. (현 실측: Rabbit −15 → **현재 상태로는 실패**. 슬롯 대책 없이는 출시 금지.)
* **슬롯 대책 후보**(별도 A/B): ① `recognize_multi` + within-object NMS, ② 객체당 슬롯 2개, ③ 후보를 semantic으로 재정렬 후 상위 2개 게이트 통과 허용.

### Phase 3 — appearance 42-view GPU 랭킹 도입

* **목표**: 템플릿 캐시 GPU 상주 + flat matmul, `rank_appe`를 42-view max로 교체. **게이트 점수 정의 불변.**
* **수정 예상 파일**: `yolo_ism_object_n.py`의 `masked_appe_blocks`(추가 함수), `prepare_objects`(flat/segment 캐시 생성).
* **측정 지표**: 검출 수(불변이어야 함), gate_score Δ(0이어야 함), 라벨 변경 건수, cls-top1 vs appe-argmax 일치율, latency Δ.
* **성공 기준**: **검출 수 Δ=0, gate_score |Δ|<1e-6**, 라벨 변경분의 시각 검수에서 개선 우세.
* **Rollback**: 검출 수가 1건이라도 변하면 즉시 원복(그것은 게이트가 오염됐다는 뜻).

### Phase 4 — 라벨 세트 영구화 → 그 다음에야 fusion/색 재검토

* **목표**: 게이트/가중치 논의를 가능하게 만드는 전제조건 확보. 검출별 `(frame, object, bbox, 42-view 프로파일, sem, sem_margin, mask_ratio, 사람 라벨)` CSV를 **저장소에 커밋**.
* **측정 지표**: 라벨 건수, 객체별 TP/FP 균형, 점수별 AUC.
* **성공 기준**: 객체당 최소 100건(TP/FP 각 30건 이상).
* **주의**: 2026-07-16의 1,521건 라벨은 CSV로 남지 않아 **재사용 불가**(본 조사에서 파일 검색으로 확인). 같은 실수를 반복하지 않는 것이 이 Phase의 실질적 목표다.

### Phase 5 — adaptive cascade / early exit

* **목표**: semantic 미달 시 MobileSAM 미실행(이미 구조적으로 그러함)을 넘어, **불확실 구간에만** 블록2 랭킹·다중 슬롯을 계산.
* **주의**: **수락 객체에 대한 MobileSAM 생략은 채택 불가** — mask가 PEM 입력 산출물이기 때문. early exit은 "거절 경로 단축"으로만 정의한다.
* **성공 기준**: latency p90 추가 −10% 이상, 판정 변화 0.

---

## 13. 위험 요소 및 Rollback 조건

| # | 위험 | 징후 | Rollback 조건 |
|---|---|---|---|
| R1 | **multi_label 후보 증가로 단일 슬롯 경쟁 악화** (실측 Rabbit −15) | 특정 객체 수락 수 급감 | 객체별 수락 수 −5% 초과 → Phase 2 중단 |
| R2 | **42-view max를 게이트로 승격하는 유혹** — 점수가 일제히 상승해 고정 임계가 슬랙 | 검출 수 증가, 택배박스 FP 증가 | 검출 수 Δ≠0 → 즉시 원복 (2026-07-15 `[2,9]` 사건 재현) |
| R3 | GPU 템플릿 캐시 메모리(객체 수 증가 시) | 객체 30개 시 270 MB fp32 | fp16 전환(45→135 MB) 또는 필요 객체만 상주 |
| R4 | 배치 forward의 crop 수 폭증(후보 증가 시 GPU 메모리) | `[N,256,384]` N 급증 | N 상한(예: 64) 두고 초과분은 2배치 분할 |
| R5 | multi_label monkeypatch가 ultralytics 버전 업에서 깨짐 | NMS 시그니처 변경 | 버전 핀 + 프로브로 시그니처 자동 확인 |
| R6 | 본 조사 표본 편중(66 detections / 6 객체, 라벨 없음) | 특정 객체 결론 과일반화 | §4.1 수치는 "경향"으로만 인용, 정확도 결론 금지 |
| R7 | 계측 자체의 왜곡(stage마다 `synchronize`) | 절대값 과대 | 상대 비교로만 해석, 최종 검증은 계측 OFF wall time |

---

## 14. 최종 결론

현재 ISM의 문제는 "특징이 부족하다"가 아니라 **"이미 가진 것을 낭비하고 있다"** 이다.

* 시간 낭비: 같은 이미지에 YOLO를 10번, 같은 프레임에 DINOv2를 9번, 같은 image encoding을 3번 돌린다. 저장소에는 이 셋을 각각 1번으로 줄이는 코드가 이미 있거나(배치 경로) 라이브러리가 이미 지원한다(multi_label NMS). **median 130 → 32 ms.**
* 정보 낭비: 42장의 템플릿 patch 특징을 사전 계산해 두고 검출당 **1장만** 쓴다. 나머지 41장을 다 쓰는 비용은 **+0.026 ms**다. 그런데 그 41장을 게이트에 쓰면 운영점이 흔들리므로, **랭킹과 진단**으로 쓴다.
* 반면 새 특징(color histogram, LBP, Gabor)은 **추가하지 않는다** — 그 정보는 DINOv2 블록2에 이미 있고, +0.1%에 꺼내 쓸 수 있으며, 새 임계값과 캘리브레이션 데이터를 요구하지 않는다.

```text
최우선 권장안:
1-Pass Batched ISM + 42-view Ranking

권장 데이터 흐름:
RGB → YOLO-World 1 pass(multi_label NMS) → 객체별 score_threshold/top_k=3 후보 → DINOv2 배치 forward 1회(CLS+patch, 블록 2/9/11 동시) → semantic(42 템플릿 CLS, topk5) 게이트 & 객체당 1박스 선택 → MobileSAM 1회 호출(통과 박스 전부) → mask 정합 patch → 42 템플릿 flat matmul 1회(GPU) → 게이트 점수는 기존 CLS-top1 뷰 값 그대로 hard gate, 랭킹만 42-view max → cross-object NMS → 수락 + mask RLE → PEM

핵심 변경:
1. 운영 진입점(PEM 브릿지·ROS 노드)을 per-object recognize()에서 이미 검증된 배치 recognize_frame()으로 전환 — 판정 동일성 기증명, latency −62%
2. YOLO-World 프롬프트별 10패스를 multi_label NMS 단일 패스로 대체 — 박스 recall 98.9% 유지, 63.2 → 7.1 ms (단, 객체당 단일 슬롯 정책 수정과 반드시 동시 적용)
3. appearance를 템플릿 1장 CPU 루프에서 42장 GPU flat matmul + segment-amax로 확대 — 검출당 +0.026 ms, 게이트 점수 정의는 불변(Δ=0.000000), 42-view max는 cross-object 랭킹 전용

사용할 특징:
- DINOv2 CLS(블록11) — semantic 게이트, 현행 유지
- DINOv2 mask-내부 patch(블록11) — appearance 게이트 점수, 현행 정의 유지
- DINOv2 mask-내부 patch(블록2·9) — 랭킹 전용(색+텍스처), 추가 forward 없음
- 42-view appearance 프로파일(max / argmax / mean / margin) — 랭킹과 기록용
- YOLO conf, semantic margin, mask_area_px — 기록 및 sanity check 전용(판정 미사용)

제외하거나 보류할 특징:
- RGB/HSV/Lab histogram, color moments — 블록2와 정보 중복, 조명 민감, 신규 임계값 요구
- LBP / Gabor — CPU 0.5~5 ms로 더 비싸고 뷰·회전 민감
- mask 형상 게이트(solidity/compactness/silhouette 일치) — 관측각 의존이 커서 고정 게이트 부적합
- score fusion(가중합·로지스틱·MLP) — 캘리브레이션 라벨 부재, Phase 4 이후로 보류
- color/texture 기반 early rejection — 배치화 이후 절감 상한이 수 ms 미만인 반면 FN 위험은 회복 불가

현재 구조 대비 예상 장점:
- frame latency median 130.41 → 32.05 ms (−75.4%), p90 169.49 → 38.77 ms, 등가 7.7 → 31.2 FPS [실측, 201 frames]
- 모델 호출: YOLO 10→1, DINOv2 커널 런치 8.78→1, MobileSAM 2.98→0.83 회/frame
- 템플릿 커버리지 1/42 → 42/42, 비용 +0.026 ms/검출 (cls-top1이 최적 뷰인 경우는 42.4%뿐이었다)
- 템플릿을 42→168장으로 늘려도 유사도 비용 0.5 ms 미만 — 도메인갭 대응 여지가 시간 제약에서 풀림
- 게이트 점수 정의 불변이므로 Phase 1·3의 정확도 리스크가 원리적으로 0

가장 큰 위험:
- YOLO 단일 패스가 후보를 늘리면 "객체당 슬롯 1개" 구조 때문에 진짜 객체가 밀려 FN이 증가한다 (실측: 201프레임에서 수락 412 → 401, Rabbit −15). 슬롯 정책 수정 없이 Phase 2를 단독 출시하면 속도를 얻고 recall을 잃는다.
- 부차 위험: 42-view max를 게이트 점수로 승격하려는 유혹 — 점수가 일제히 상승해 고정 임계 0.55가 느슨해지며, 2026-07-15 appe_blocks=[2,9] 롤백(검출 +16.1%, 대부분 택배박스)과 동일한 실패가 재현된다.

첫 번째 구현 실험:
- Phase 1 단독: build_ism_inputs_imu.py와 sam6d_multiobject_node.py의 recognize() 호출을 recognize_frame()으로 교체하고, SAM_* 4 bag 전 프레임에 대해 생성된 detection JSON(RLE seg 포함)의 byte 동일성을 검증한다. 성공 기준 = JSON byte 동일 + latency −50% 이상. 차이가 1건이라도 나오면 즉시 원복.
```

---

## 부록 A. 재현 방법 (격리 폴더)

```
sam6d_ws/ism_methodology_analysis/          ← 이 폴더만 삭제하면 원상복구
  scripts/bench_patch_sim.py                # 템플릿 patch 유사도 K=1..42, CPU/GPU
  scripts/probe_template_scope.py           # cls-top1 vs appe-argmax + stage 단가
  scripts/bench_yolo_prompt_passes.py       # 10패스 vs 공유 vs multi_label
  scripts/bench_frame_pipeline.py           # P0(production) vs P1(권장) frame 실측
  results/bench_patch_sim.json
  results/template_scope_detections.csv     # 66 detections, 24 필드
  results/template_scope_timing.json
  results/bench_yolo_passes.json
  results/wide/bench_frame_pipeline{.json,_frames.csv}   # 201 frames
```

실행: `/home/ldh9501/miniconda3/envs/sam_yolo/bin/python ism_methodology_analysis/scripts/<script>.py`
공통: warm-up 3프레임 제외, stage 경계 `torch.cuda.synchronize()`, median·mean·p90 기록, 모델 로딩 시간 분리.
운영 파일 수정 없음 / 설정값 변경 없음 / git 조작 없음. multi_label은 프로브 프로세스 내부 monkeypatch로만 적용된다.

## 부록 B. 본 조사의 한계

* **라벨 없음.** Precision/Recall/F1/AUC는 산출하지 않았다. §4.1의 42.4%는 "CLS와 patch의 뷰 선택 불일치율"이지 오검출률이 아니다.
* **표본**: template scope 프로브는 66 detections / 6 객체 / 3 bag. Sauce_high는 n=3으로 객체별 수치는 경향 이상으로 읽지 말 것.
* **계측 왜곡**: stage마다 synchronize를 걸어 절대값이 실제 운영보다 크다. P0/P1 비교는 동일 조건이므로 상대값은 유효하다.
* **P1의 decision drift는 두 변경(YOLO 후보 집합 + 배치화)이 섞여 있다.** 배치화 단독은 1,320행 동일성이 기증명이므로 drift는 후보 집합 변경에 귀속되나, Phase 2에서 단독 A/B로 재확인해야 한다.
* **MobileSAM의 stability/IoU 점수는 ultralytics `SAM` 래퍼가 노출하지 않아** mask 품질 신호 후보에서 제외했다.

---

## 다음 액션 제안

1. **[권장] Phase 1 실행** — 운영 진입점 2곳을 `recognize_frame()`으로 전환하고 detection JSON byte 동일성으로 검증. 리스크 0, 이득 −62% recognize 시간. (구현은 `bmad-quick-dev` 또는 `bmad-spec` → dev)
2. **Phase 2 사전 실험만 먼저** — multi_label 단일 패스를 켠 상태에서 슬롯 정책 3안(recognize_multi / 슬롯 2개 / semantic 재정렬)을 A/B하여 Rabbit −15가 사라지는 안을 고른 뒤에 출시.
3. **Phase 4 선행 착수(라벨 영구화)** — 향후 게이트·fusion 논의를 가능하게 하려면 라벨 CSV가 먼저다. 지금 축적을 시작하면 다음 조사부터 정확도 결론을 낼 수 있다.
4. **분석 폴더 처리** — `ism_methodology_analysis/`를 보존할지(재현용) 삭제할지 결정.

어느 방향으로 진행할까요?
