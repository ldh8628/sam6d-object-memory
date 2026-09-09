---
title: 'Phase 1C 구현 — HSV active gate + Dinosaur 클래스 hue +9 보정'
type: 'implementation-result'
created: '2026-07-22'
status: 'review'
baseline_commit: '8f86c227037275974e38943d17ca8fc12a60e563'
verdict: 'GO — HSV active + Dinosaur hue correction 채택, texture verifier 미적용/기각'
outputs: 'outputs/phase1c_hsv_dinosaur_color_correction/'
---

# Phase 1C 구현 결과

> **판정: GO.** HSV 색상 게이트를 운영에서 **활성화**(t=0.1214)하고, Dinosaur 클래스 렌더 reference에만
> **+9 OpenCV hue 보정**을 적용했다(캐시에 반영 → 런타임 객체별 분기 없음). texture negative verifier는
> **운영 미적용**(사람 GT AUROC 0.834, clean gate 아님), texture rescue verifier는 **기각**(AUROC 0.508).
> 사람 GT 935 재현: **F1 0.676 → 0.757, FP 334 → 86, Dinosaur TP 33 → 39(+0 FP)**. 실 GPU에서 게이트
> 동작 확인. git 조작 없음.

---

## 1. Executive Summary

- **활성화**: HSV 색상 hard gate(semantic·appearance 통과 후보에 색 유사도 검사, `score ≥ 0.1214`이면
  통과, 미만이면 제거) + Dinosaur 클래스 reference hue **+9 OpenCV 단위**(= +18° in 0-360) 보정.
- **미적용/기각**: texture negative verifier(shadow/off), texture rescue verifier(disabled).
- **성능(사람 GT 935)**: F1 0.6761 → **0.7572**, FP 334 → **86**, Dinosaur TP 33 → **39**(새 FP 0).
- **최종 결론**: Phase 1C **GO**.

## 2. 초보자를 위한 용어 설명

- **detection/bbox**: 물체가 있을 법한 사각형 위치.
- **mask**: 그 사각형 안에서 실제 물체 픽셀만 남긴 형태.
- **ROI/crop**: bbox로 잘라낸 이미지 조각.
- **HSV / hue / saturation / value**: 색을 색상(hue)·채도(saturation)·밝기(value)로 나눈 표현. hue는
  0~360°(코드에선 OpenCV 0~179)의 **원형** 값이라 359° 다음이 0°.
- **semantic score**: DINOv2가 "이 조각이 등록 물체의 전체 형태와 얼마나 닮았나"(높을수록 유사).
- **appearance score**: 마스크 안 국소 질감이 템플릿과 얼마나 닮았나(높을수록 유사).
- **HSV score**: 색 분포가 등록 렌더 색과 얼마나 닮았나(높을수록 유사). 이번에 켠 게이트.
- **texture score**: DINOv2 patch로 인쇄 무늬/로고까지 비교(연구용).
- **threshold**: 통과/탈락을 가르는 기준값. 여기선 `score ≥ threshold`면 통과.
- **TP/FP/TN/FN**: 맞게검출/오검출/맞게제외/놓침. **precision**=TP/(TP+FP), **recall**=TP/(TP+FN),
  **F1**=둘의 조화평균. **AUROC**=두 집단을 점수로 얼마나 잘 가르나(0.5=우연, 1=완벽).
- **shadow mode**: 계산·기록만 하고 판정은 안 바꾸는 상태. **rescue verifier**: 제거된 후보를 되살리는 검사.

## 3. 실제 코드 실행 흐름 (파일·함수)

진입점 2개, 모두 `configs/yolo_ism_objects.yaml`을 `o_n.load_config`로 읽음:
- 배치: `yolo_ism_object_n.py::main()` → `recognize_frame()`
- 라이브 ROS: `src/sam6d_ros/.../sam6d_multiobject_node.py` → `o_n.recognize()` (파일 무수정)

흐름(실제 함수):
```
RGB → YOLO-World predict(공유 1-pass) → build_prompt_groups 라우팅 → score_threshold 0.02 필터 → top_k=3
 → dinov2_blocks_forward(CLS) → semantic_score ≥ similarity_threshold(0.35)
 → segment_box(MobileSAM) → masked_appe_blocks(block11) ≥ appe_gate(0.55)
 → _apply_hsv_gate(o,res,bgr):  ism_hsv.shadow_score(bgr,box,mask,proto) ≥ hsv_gate_threshold(0.1214)
      · hsv_gate_enabled=true 이므로 미달 시 res['accepted']=False, decision='no-object(below-hsv)'
 → 최종 accept/reject (+ CSV/PoseArray/detection.json)
```
- HSV authority: `ism_hsv.py`(feat_rgb/query_hist/similarity/build_reference/shadow_score/load_cache).
- 게이트 삽입: `yolo_ism_object_n.py:_apply_hsv_gate` (recognize/recognize_frame accept 지점 3곳 호출).

## 4. HSV 특징 계산 과정 (`ism_hsv.py`)

- **어느 이미지**: 최종 수락 후보의 **box crop + MobileSAM mask**(마스크 내부 픽셀만).
- **query**: `cv2.cvtColor(BGR2HSV)` 후 **H×S joint 히스토그램**(bins 16×8=128, 범위 [0,180]×[0,256]),
  **L1 정규화**. query에는 hue/sat 보정 안 함.
- **reference(템플릿)**: 42장 렌더의 mask 내부 픽셀 → hue **−3 OpenCV**(=−6° 전역 보정)·**채도 ×1.3** →
  동일 히스토그램. **Dinosaur만 추가 +9 OpenCV**(§5).
- **유사도**: `bc = Σ√(q·p)` (Bhattacharyya 계수), `sim = max_over_42(1 − √(1 − bc))`. **높을수록 유사.**
- **정규화/방향**: L1, score∈[0,1], 게이트 `sim ≥ 0.1214`면 통과.
- **invalid**: 빈 crop/mask·캐시 누락 → proto None → `similarity`=1.0 (**fail-open**, 절대 잘못 제거 안 함).
  수식·파라미터 코드 확정치는 `ism_hsv.py` 상단 상수와 `_apply_hsv_gate`.

## 5. Dinosaur hue +9 보정

- **왜 Dinosaur만**: 실제 Dinosaur가 렌더(황록 H≈78°)보다 **더 초록·청록(H68~108°)**으로 관측되어 일부
  진짜 Dinosaur가 색으로 오탈락. 전역 −6°는 다른 객체엔 맞지만 Dinosaur는 **반대(+green)** 필요.
- **실험 근거**: LODO(5데이터 보정선택→held-out) **6폴드 전부 +9 안정**, Dino TP 33→39(+6), 새 FP 0,
  **+12 이상은 overshoot(→25)**. (`research_a_dinosaur_color/results/dinosaur_template_correction_sweep.csv`)
- **단위 변환(핵심)**: 검증에서 쓴 값은 **+9 OpenCV hue 단위**(0~179 공간). OpenCV 180 = 360°이므로
  **+9 OpenCV = +18°(0-360)**. (사용자 예시 "+9°→OpenCV 4.5"와 달리, 검증된 값은 OpenCV 단위 9다 — 이
  단위 구분을 코드·config에 명시.)
- **적용 위치**: **reference/템플릿** 픽셀에 적용(query 아님), `feat_rgb`의 −3 위에 더해짐. wrap-around는
  `(h + 9) % 180`로 원형 처리(`ism_hsv._apply_hue_ocv`).
- **런타임 분기 없음**: 보정은 **캐시 빌드 시 Dinosaur_hsv.npz에 반영**. 런타임 `_apply_hsv_gate`는 객체
  구분 없이 로드된 proto만 사용 → `object=="Dinosaur"` 분기 0.
- **타클래스 비영향**: config에서 Dinosaur만 `hsv_hue_correction_ocv: 9`, 나머지 0 → 다른 9객체 proto·판정
  불변(단위테스트 `test_dinosaur_only_correction`).

## 6. HSV threshold 0.1214 선택 근거

- 후보 grid = 수락 후보 HSV score의 quantile. 목적함수 = **F1-max**(연구 mode B와 동일).
- LOO 6폴드에서 **t* = 0.1214 전 폴드 동일**(완벽 안정), 6폴드 전부 FP 감소 → 배포 단일값 0.12140 동결.
  (`phase1b_validation/fold_freeze.csv`)
- 민감도(`outputs/.../csv/phase1c_threshold_sweep_hsv.csv`): t를 낮추면 FP↑(0.10서 94, 0.05서 143),
  Dino는 t≤0.11에서만 추가 보존. 0.1214는 FP 최소(86)와 F1 최대의 균형점.
- **과적합/데이터 의존성**: 6폴드 안정이나 provisional 아닌 사람 GT 935에서 확정. 조명·화이트밸런스가
  크게 다른 신규 환경에선 재캘리브 필요(§13).

## 7. Texture negative verifier 분석 (운영 미적용)

- **feature/score**: DINOv2 block11 patch를 42템플릿과 매칭한 max(P2). 실사 reference 불필요.
- **사람 GT**: 84 accepted-as-choco = choco-visible 52 / FP 32.
- **결과**(`figures/texture_negative/`, `metrics/texture_reproduction.json`): **AUROC 0.834**. thr 0.60 →
  choco 47/52·FP 8/32, thr 0.625 → choco 39/52·FP 26/32. 기준별 최적 threshold(F1-max, Youden J,
  recall≥0.9, recall≥0.8)가 서로 다르며, **어느 것도 낮은 TP손실로 FP 대량제거를 못 함**(분포 겹침).
- **미채택 이유**: clean operating point 부재. **운영 판정에 미반영**(운영 코드에 texture 경로 없음).
  향후 채택하려면 사람 GT 확장 + 명시적 임계 승인 필요.

## 8. Texture rescue verifier 기각 근거

- **대상 280**: HSV가 제거한 accepted 후보 전수 = 진짜 TP casualty 32 / HSV가 옳게 제거한 FP 248.
- **결과**(`figures/texture_rescue/`, `csv/texture_rescue_threshold_sweep.csv`): TP casualty texture 평균
  0.659 ≈ FP 평균 0.658, **AUROC 0.508(우연)**. thr 0.62: TP 29/32 복구에 **FP 194/248 재유입**(Bear
  62/68·Rabbit 67/68·Dino 36/39).
- **원리적 실패**: HSV가 제거한 FP는 애초 **texture가 유사**해 sem+appe를 통과한 것(cross-fire 인형·갈색
  박스). **오직 색만 분리**했으므로 texture로는 재분리 불가.
- **완전 비활성**: 운영 코드에 rescue 경로 없음. config에도 없음. 분석 재현만 존재.

## 9. 시각화 해설 (`outputs/.../figures/`)

- `pipeline/phase1c_pipeline.png` — 전체 흐름(초록=핵심 게이트, 노랑=보류 texture-neg, 빨강=기각 rescue),
  각 단계 입력·score·판정 영향 표기.
- `hsv/hsv_score_distribution.png` — 진짜 TP vs 오검출 FP의 HSV score 분포 + threshold 0.1214 수직선
  ("이상이면 통과"). FP가 대부분 임계 아래로 몰려 제거됨을 보여줌.
- `dinosaur/dinosaur_before_after.png` — Dino TP 33→39 막대 + 렌더/보정/실제 Hue 비교(보정이 렌더를 실제
  쪽으로 이동).
- `texture_negative/texture_negative_roc_tradeoff.png` — ROC(AUROC 0.834) + TP유지/FP제거 trade-off(깨끗한
  임계 없음).
- `texture_rescue/texture_rescue_overlap_readmission.png` — 두 집단 겹침(AUROC 0.508) + thr 0.62에서 FP
  194 재유입.
- `summary/phase1c_beginner_summary.png` — 초보자용 before/after(TP·FP·F1) + 한국어 요약 결론.
- `representative_cases/` — 실제 프레임 사례: Rabbit threshold 복구, HSV가 cross-fire(곰→Rabbit) 제거,
  Dinosaur 색 분포, choco vs 갈색박스.

## 10. 구현 변경 사항

**수정(운영):**
- `configs/yolo_ism_objects.yaml` — `hsv_gate_enabled: true`(활성화), `hsv_hue_correction_ocv: 0` 기본 +
  Dinosaur 오버라이드 `9`; texture verifier 미적용/기각을 주석 명시.
- `yolo_ism_object_n.py` — `_apply_hsv_shadow`→`_apply_hsv_gate`로 확장(active 시 HSV 미달 후보 reject);
  `_OVERRIDABLE`에 `hsv_hue_correction_ocv` 추가; `prepare_objects`에서 캐시 hue_correction 불일치 검증;
  `load_config` hsv_cache 유도(기존).
- `ism_hsv.py` — `_apply_hue_ocv` 신규(원형 hue shift); `build_reference/save_cache/load_cache/_template_signature`에
  `hue_correction_ocv` 인자·서명·검증 추가.
- `tools/build_hsv_template_cache.py` — 객체별 `hsv_hue_correction_ocv` 반영.

**신규(테스트/평가/시각화, 운영 무영향):** `tests/test_phase1c.py`; `research_a_.../evaluation/eval_phase1c.py`;
`combined_summary/make_phase1c_figures.py`; `outputs/phase1c_hsv_dinosaur_color_correction/` 전체.

**backward-compat**: config에 새 키가 없으면 `hsv_hue_correction_ocv` 기본 0(보정 없음), `hsv_gate_enabled`
없으면 False(OFF). 원본 템플릿·모델·PLY·기존 CSV 무수정.

**rollback**: `hsv_gate_enabled: false`로 HSV OFF(현행 이전 동작). Dino 보정 제거 = config
`hsv_hue_correction_ocv` 삭제 + 캐시 재빌드(`--force`). ⚠ `git checkout` 금지(두 운영 파일에 이전 세션
미커밋 변경 공존).

## 11. 검증 결과

- **compile**: `py_compile` OK(driver/ism_hsv/builder).
- **unit test**: `tests/test_phase1c.py` **12/12 PASS**(hue wrap-around, Dino-only, config active, gate
  shadow/active/disabled 의미, fail-open empty/missing, 비수락 미계산). 회귀: Phase 1A 5/5, 1B 13/13.
- **전체 평가(사람 GT 935, t=0.1214)** `outputs/.../csv/phase1c_config_comparison.csv`:

| 구성 | TP | FP | FN | Precision | Recall | F1 |
|---|---:|---:|---:|---:|---:|---:|
| A baseline (HSV OFF) | 648 | 334 | 287 | 0.660 | 0.693 | 0.6761 |
| B HSV active (Dino 미보정) | 616 | 86 | 319 | 0.878 | 0.659 | 0.7526 |
| **C HSV active + Dino +9 (Phase 1C)** | **622** | **86** | **313** | **0.879** | **0.665** | **0.7572** |

- **Dinosaur**: HSV+Dino0 33/0/55 → **HSV+Dino9 39/0/49** (TP +6, 새 FP 0).
- **실 GPU 런타임 확인**: sam_110633 25프레임 → HSV로 12건 `no-object(below-hsv)` 제거, hsv 필드 55건
  기록, Dino 수락 확인. **active 게이트 실동작.**
- **texture 재현**: negative AUROC **0.834**, rescue AUROC **0.508** (연구 수치와 일치).
- **기존 수치와 차이**: C의 F1(0.7572)이 목표 0.753보다 높음 = Dino +9의 +6 TP 때문(B가 0.7526로 목표
  일치). 원인 규명됨, 실패 아님.

## 12. 운영 구성 (config 발췌)

```yaml
defaults:
  hsv_gate_enabled: true
  hsv_gate_shadow_mode: false
  hsv_gate_threshold: 0.12140
  hsv_hue_correction_ocv: 0          # 기본 보정 없음
objects:
  - name: Dinosaur
    hsv_hue_correction_ocv: 9        # +9 OpenCV(=+18deg), 캐시에 반영
```
texture negative/rescue verifier는 운영 파이프라인에 **존재하지 않음**(연구 shadow/기각).

## 13. 남은 위험

- 사람 GT 표본(특히 choco 52·Dino 55)이 작아 texture·경계 임계는 지표적.
- 조명·카메라 화이트밸런스가 다른 환경에서 Dino +9·HSV 0.1214가 재캘리브 필요할 수 있음(hue shift
  dataset 의존).
- **새 갈색 물체**는 HSV로 못 거름(색 동일) — texture가 못 풀었으므로 여전히 열린 문제.
- class alias: 현재 config는 `Dinosaur` 단일 이름. 신규 파이프라인이 다른 alias를 쓰면 정규화 필요.

## 14. 최종 결론

```text
Phase 1C: GO
HSV active: 채택 (t=0.1214)
Dinosaur hue +9(OpenCV): 채택 (캐시 반영, 런타임 분기 없음)
Texture negative verifier: 운영 미적용 (shadow/off, 사람 GT 임계 승인 대기)
Texture rescue verifier: 배제 (원리적 실패, AUROC 0.508)
운영 코드 변경: config + yolo_ism_object_n.py + ism_hsv.py(신규) + builder(신규)
```

## 다음 권장 작업
1. 실 bag 전체(6데이터)로 배치 재생성해 CSV/overlay 운영 산출물 최종 확인(대량 GPU).
2. texture negative verifier 사람 GT 확장 → 임계 승인 시 shadow→active 별도 story.
3. 새 갈색 물체 오검출(HSV·texture 모두 미해결)은 별도 접근(예: 형상·문자 OCR) 필요.
