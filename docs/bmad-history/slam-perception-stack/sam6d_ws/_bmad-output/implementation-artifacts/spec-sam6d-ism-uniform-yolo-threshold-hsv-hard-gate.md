---
title: 'SAM-6D ISM 보수적 정확도 개선 — 전 객체 YOLO threshold 0.02 통일 + Hue 보정 렌더 HSV hard gate'
type: 'spec'
created: '2026-07-22'
status: 'draft'
baseline_commit: '8f86c227037275974e38943d17ca8fc12a60e563'
authority_code:
  - yolo_ism_object_n.py                                     # 운영 드라이버(게이트/로더/라우팅)
  - configs/yolo_ism_objects.yaml                            # 운영 config
  - src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py        # 라이브 ROS 노드(출력 emitter)
  - _ism_research_2026_07/yolo_localization_research/probes/eval_pipeline_end2end.py   # HSV 게이트 authority(mode B)
  - _ism_research_2026_07/ism_fusion_research/ply_hsv/build_prototype_colors.py        # render42 색 authority
  - _ism_research_2026_07/ism_color_validation/results/fold_params.csv                 # t_hsv 캘리브레이션 값
scope: '구현 명세만 작성. 운영 코드/config 무수정, git 무조작.'
---

> **본 명세는 구현 계약서다.** 코드는 이 문서를 authority 로 구현한다. 모든 수치·경로는
> 실제 저장소(baseline `8f86c22`)에서 확인해 고정했으며, 확정 불가한 값은 §17·개방질문에
> 명시했다. 추측값 없음.

---

## 1. Problem Statement

SAM-6D ISM(YOLO-World proposal → DINOv2 semantic → MobileSAM mask → DINOv2 appearance
게이트) 파이프라인은 두 가지 확인된 손실을 안고 있다.

1. **객체별 YOLO `score_threshold` guard 가 실제 후보를 죽인다.** Bear·Rabbit·Dinosaur 는
   교차오검(cross-fire) 을 막으려 0.30~0.35 의 높은 threshold 를 갖는데, 이 값이 특히 **Rabbit
   의 정답 후보까지 제거**한다(사람 GT 기준 Rabbit 이 미검출의 큰 비중). 나머지 객체는 이미
   0.02 를 쓴다 — 즉 guard 는 3개 객체에만 걸린 비대칭 장치다.
2. **색이 다른 오검을 거를 채널이 없다.** semantic/appearance 는 형태·질감 기반이라 색만
   다른 유사 형태(예: choco ↔ 갈색 택배박스)를 구분하지 못한다. 색 정보를 **점수로 더하면**
   같은 색 객체의 FP 가 폭증하지만, **수락 후보를 제거하는 hard gate 로만** 쓰면 안전하게
   FP 를 줄일 수 있음이 오프라인에서 검증됐다.

본 명세는 이 둘을 **보수적으로**(운영 아키텍처 불변, 되돌리기 가능, FP 비증가) 결합한다.
라우팅 제거·YOLOE union·semantic aggregation 변경 등 더 공격적인 안은 §4/§18 로 분리한다.

---

## 2. Validated Evidence

사람이 라벨한 **가시 객체 GT 935건**(6개 `SAM_*` 데이터, `gt_input/user_visibility_gt.csv`)과
LODO(leave-one-dataset-out) 평가. Authority: `eval_pipeline_end2end.py`(mode B: Hue 보정 렌더
HSV hard gate 포함).

| 구성 | 후보/라우팅 | TP | FP | FN | Precision | F1 |
|---|---|---:|---:|---:|---:|---:|
| 객체별 threshold(현행 guard) + HSV gate | cur \| R0 | 589 | 86 | 346 | 0.873 | 0.7317 |
| **전 객체 0.02 통일 + HSV gate** | **cur \| R2** | **616** | **86** | **319** | **0.878** | **0.7526** |

→ **threshold 통일 효과(HSV 고정): ΔTP +27 / ΔFP ±0 / ΔFN −27.** Rabbit 이 개선의 주 수혜자.

**HSV hard gate 의 단독 FP 절감 효과**(color-validation LODO, 별도 하니스): FP 228→89
(`sam6d-hsv-gate-validated`) 및 935건 재검 FP 256→98·precision .713→.865
(`sam6d-gt-validated-hsv-gate`). **점수 가산 방식은 choco FP 3~4배 폭증으로 기각** — gate 전용만
안전.

**⚠ 증거 해석 주의(정직성):**
- 위 표의 "현행" 행(cur|R0)은 **HSV gate 를 이미 포함**한다. 오늘의 실제 운영 코드에는 HSV
  게이트가 **없다**. 표는 "HSV 를 고정한 채 threshold 만 바꾼" 순수 효과(+27/±0)를 분리한 것이다.
  실제 운영(현행: guard + HSV 없음) 대비 본 명세(0.02 + HSV) 의 결합 델타는 별도 하니스에서
  B0 로 측정해야 한다(§14).
- 평가 GT 는 **프레임 단위**(객체가 프레임에 보이는가) 이지 **박스 위치 정확성이 아니다**
  (`eval_pipeline_end2end.py:18`). "TP 616" 은 픽셀 정확 검출이 아니라 "가시 프레임에서 그
  객체를 수락함" 을 뜻한다. §17 리스크 참조.

---

## 3. Scope

1. **객체별 YOLO `score_threshold` guard 제거 → 전 객체 0.02 단일값.** single source of truth =
   `defaults.score_threshold`(config).
2. **Hue 보정 렌더 HSV hard gate 추가.** semantic·appearance 판정을 이미 통과한 후보에 대해서만
   색 유사도로 최종 accept/reject. 3단계(1A threshold → 1B HSV shadow → 1C HSV active) 점진 활성.
3. **진단 로깅·오프라인 검증 스크립트·단위/회귀 테스트** 신설.
4. `hsv_gate_enabled=false` + config 원복만으로 **완전 롤백** 가능.

유지되는 아키텍처 불변식(§5 그대로): 프롬프트별 YOLO 10-pass(공유 1-pass 라우팅)·클래스 라우팅·
top_k=3·semantic Top-5(`sem_top5`)·MobileSAM·appearance(block11)·PEM 전달 포맷.

---

## 4. Out of Scope

아래는 본 명세에서 **구현·설계 혼입 모두 금지**한다(각각 별도 story):

- semantic Top-5 → 전체 평균(sem_mean) 변경, Top-5/mean fusion, 객체별 semantic aggregation
- DINOv2 block2+block11 조건부 patch 검증기, class margin
- **클래스 라우팅 제거**(union 에서 FP +50 로 역효과 확인: union|R0 F1 .7560 → union|R1 .7378)
- **YOLOE 단독 교체 / 현행+YOLOE union**(recall/precision trade-off 미결)
- 입력 해상도 증가, tile detection, prompt ensemble, temporal/depth proposal
- PEM 수정, score_threshold 재학습(0.02 는 기존 운영점), 조명 환경 대응
- 실사 crop prototype, PLY 전체 vertex histogram, `*_high.ply` 신규 렌더, 실카메라 onboarding 이미지
- HSV 를 semantic/appearance **점수에 가산**하는 모든 방식(§2 근거로 영구 배제)

---

## 5. Current Runtime Flow (baseline `8f86c22`)

운영 진입점은 `recognize_frame()`(배치 드라이버, `yolo_ism_object_n.py:425`)과
`recognize()`(라이브 ROS 노드가 호출, `:273`). 두 경로 게이트 동일.

```
RGB 프레임
→ YOLO-World 공유 1-pass, conf = min(전 객체 score_threshold)   [:583, :598  set_classes :577]
→ box 를 prompt-index 그룹으로 라우팅 (build_prompt_groups)      [:259-267, 소비 :442]
→ 객체별: score_threshold 필터 → top_k=3                        [:446 s>=score_threshold, :447 [:top_k]]
→ crop → DINOv2 CLS → semantic Top-5(sem_top5) 최고 후보 선택
→ semantic 게이트  best_sem >= similarity_threshold             [:481]
→ MobileSAM mask → best template(argmax tcls@cls) → masked-appe(block11)
→ appearance 게이트  masked_appe >= appe_gate  → accept/reject   [:514  ← 유일 accept 지점]
```

- 객체별 threshold(현행): Bear 0.35(`yolo_ism_objects.yaml:256`), Rabbit 0.30(`:264`),
  Dinosaur 0.30(`:272`), 그 외 = `defaults.score_threshold: 0.02`(`:13`). top_k=3(`:12`).
- **현행 코드에 HSV 게이트는 없다.**
- 출력 emitter 2종:
  - **배치**(`yolo_ism_object_n.py` main): 객체별 CSV(`CSV_FIELDS` `:96-98`) + overlay JPG.
  - **라이브 ROS**(`sam6d_multiobject_node.py`): `recognize()`(`:217`) → `PoseArray`(topic `~/poses`,
    `:111/:234`) + BOP-style `detection.json`(`:175-178`, PEM 입력).

---

## 6. Target Runtime Flow

```
RGB 프레임
→ (변경) YOLO-World 공유 1-pass, conf = 0.02 (전 객체 동일)
→ 클래스 라우팅 (불변)
→ 객체별: score_threshold=0.02 필터 → top_k=3 (불변)
→ DINOv2 CLS → semantic Top-5 최고 후보 선택 (불변)
→ semantic 게이트 (불변)
→ MobileSAM mask → masked-appe(block11) (불변)
→ appearance 게이트 (불변)
→ (신규) HSV hard gate:  hsv_score(accepted box, mask) >= hsv_gate_threshold
       accept  iff  (semantic pass) AND (appearance pass) AND (hsv pass)
```

동작 모드(§7 config 로 제어):

| 모드 | enabled | shadow | 효과 | 대응 Phase |
|---|---|---|---|---|
| OFF(현행 동등) | false | false | HSV 계산·판정 없음. 판정 = §5 와 bit-identical | — |
| SHADOW | false | true | hsv_score·would_hsv_reject 계산·로깅, **판정 불변** | 1B |
| ACTIVE | true | (무시) | HSV 를 accept 게이트의 conjunct 로 적용 | 1C |

Phase 1A(threshold 통일)는 HSV OFF 상태에서 config 만 바꾼다.

---

## 7. Config Contract

파일: `configs/yolo_ism_objects.yaml`. 로더: `load_config()`(`yolo_ism_object_n.py:108`) —
`defaults` 를 각 객체에 병합(`o = dict(defaults); o.update(raw)` `:122-123`), 반환 `(defaults, objs)`.
overridable 키 화이트리스트: `_OVERRIDABLE`(`:40-44`).

### 7.1 Threshold 통일 (Phase 1A)

- `defaults.score_threshold: 0.02` (`:13`) — **유일 source of truth. 값 변경 없음.**
- **제거**: Bear(`:256`)·Rabbit(`:264`)·Dinosaur(`:272`) 의 `score_threshold` override 3줄.
- 제거 후 어떤 객체도 `score_threshold` override 를 갖지 않아야 한다(검증: §13 단위테스트 UT-1).
- 하위호환: 병합 로직·`_OVERRIDABLE` 불변. 미래에 override 가 다시 들어와도 코드는 동작하되,
  **검증기가 경고**한다(아래 로더 검증).
- 로더 검증 추가(선택, `load_config` 내): 객체가 `score_threshold` 를 갖고 그 값이
  `defaults.score_threshold` 와 다르면 `logging.warning("object %s overrides score_threshold=%.3f
  (uniform-0.02 정책 위반 가능)")`. **거부하지 않고 경고만**(하위호환).

### 7.2 HSV gate 키 (신규, `defaults` 아래 + `_OVERRIDABLE` 등록)

기존 키 네이밍(snake_case, defaults 배치, 객체 override 허용)을 따른다.

```yaml
defaults:
  # ... 기존 키 ...
  hsv_gate_enabled: false          # bool. false=OFF/SHADOW, true=ACTIVE (§6)
  hsv_gate_shadow_mode: true       # bool. enabled=false 일 때만 의미. true=계산·로깅, 판정 불변
  hsv_gate_threshold: 0.122        # float. 동결 배포값 = 6-fold LODO t_hsv 평균 (§8-8, 개방질문 OQ-1)
  hsv_hue_shift_deg: -6            # int. 0-360 도 기준. 코드에서 round(deg/2) 로 OpenCV(0-180) 변환
  hsv_sat_gain: 1.3               # float. 채도 배율
  hsv_hist_bins: [16, 8]          # [H_bins, S_bins]. joint H×S = 128-D
  hsv_lowsat_exception: false      # bool. 저채도 예외. 기본 false = 배포 결정 (§8-7, §17-R3)
  # hsv_cache 는 지정 없으면 auto-derive (§9)
```

- `_OVERRIDABLE`(`:40-44`)에 위 8개 키 추가 — 객체별 override 허용(단 배포 정책은 전 객체 균일).
- `hsv_gate_enabled`·`hsv_gate_shadow_mode`·`hsv_gate_threshold` 는 배포 시 **전 객체 균일**하게
  `defaults` 에만 둔다.

---

## 8. HSV Feature Contract

Authority: `eval_pipeline_end2end.py`(mode B, 검증 완료 최종 구성). 아래 값은 **코드에서 고정 확인**.
✅=코드 pinned, ❌=코드에 고정값 없음(계산/미채택).

| 파라미터 | 값 | pinned | 출처 |
|---|---|---|---|
| 색 변환 | `cv2.COLOR_BGR2HSV`(입력 RGB→BGR 역순 `[:,:,::-1]`) | ✅ | `eval_pipeline_end2end.py:54-55` |
| Hue 범위 | OpenCV 0–180 | ✅ | `:56, :58` |
| Hue 보정 | −6°(0-360) → 코드 적용 `round(-6/2)=-3` (OpenCV 단위), `%180` | ✅ | `:39, :56` |
| Hue 보정 대상 | **레퍼런스 템플릿만**(query 는 미보정) | ✅ | `:56` vs `:72-74` |
| 채도 gain | ×1.3, `clip(...,0,255)` | ✅ | `:39, :57` |
| 채도 gain 대상 | **레퍼런스 템플릿만** | ✅ | `:57` |
| 히스토그램 | joint H×S, bins [16,8]=128, 범위 [0,180]×[0,256], **L1 정규화**(÷합) | ✅ | `:58-59` |
| query feature | 마스킹 크롭의 H×S 128-D 히스토그램(H·S·V·HS 224-D 중 HS 블록 `slice(96,224)`) | ✅ | `:38, :74` |
| 템플릿 수 | **42** views(`render42`) | ✅ | `build_prototype_colors.py:115,132` |
| 집계 | 42개 템플릿 유사도의 **max** | ✅ | `:98` |
| 유사도 | `bc=Σ√(q·p)`; `sim=(1−√(1−bc)).max()`; **높을수록 유사** | ✅ | `:97-98` |
| 게이트 | `sim >= hsv_gate_threshold` | ✅ | `:123` |
| **threshold t_hsv** | **동결값 없음 — LODO fold 별 F1-max 캘리브** (평균 ≈0.122, 범위 0.113–0.136) | ❌ | `fold_params.csv`; grid `:175` |
| **저채도(Rabbit) 예외** | **pinned code 에 없음.** 탐색 eval 에만 존재(LOWSAT_TH=20/255, 객체 평균채도, Rabbit만); 최종 결정 "예외 없음" | ❌ | `eval_fusion_modes.py:41,101-121`; report `technical-sam6d-offline-hsv-fusion-minimal-review-2026-07-21.md:156` |

### 8.1 런타임 query HSV 계산(구현 필수 절차)

수락 후보의 box + MobileSAM mask 로부터:
1. box 로 BGR sub-crop, 동일 box 로 mask sub.
2. `cv2.calcHist([hsv],[0,1], mask, [16,8], [0,180,0,256]).flatten()` → **마스킹 픽셀만**.
3. L1 정규화(÷합, 0-div 가드 `max(sum,1e-12)`) → `q` (128-D). **query 는 hue/sat 보정 안 함.**
4. `PROTO[obj]`(§9, 42×128) 와 §8 유사도로 `hsv_score` 산출.

### 8.2 배포 threshold 동결 (핵심 결정)

t_hsv 는 코드에 고정값이 없고 LODO fold 별로 F1-max 로 정해진다. 배포 시엔 held-out 이 없으므로
**단일 값으로 동결**해야 한다. 본 명세는 `hsv_gate_threshold: 0.122`(6-fold 평균)를 시작값으로
고정하고, **§14 오프라인 검증에서 전 6-데이터(non-LODO) F1-max 로 최종 단일값을 확정**한다(OQ-1).
AC-2 의 목표 수치(§15)는 per-fold 캘리브로 산출됐으므로, 동결값 적용 시 소폭 편차는 허용 오차
내에서 재검증한다(§14 B2-frozen, §15 AC-2b).

### 8.3 저채도 예외 결정

배포는 **예외 없음**(`hsv_lowsat_exception: false`)을 채택한다. 근거: AC-2 목표 수치를 낸 pinned
mode-B 코드에 예외가 없고, 최종 color-validation 결정이 "저채도 예외 없음"(svfall 은 AUROC +0.007
이나 gate FP 89→95 악화)이기 때문. 리서치 리포트 일부가 예외를 권한 이력은 §17-R3 로 기록한다.
config 키는 남겨 되돌릴 여지만 둔다(기본 false).

---

## 9. Template Cache Contract

운영은 이미 객체별 `.pt` 특징 캐시를 auto-derive 한다(`load_config:134-137` →
`outputs/yolo_ism_object_n/template_features/<name>_{cls,appe}.pt`). HSV 캐시도 같은 규약으로 신설.

- **경로**: `hsv_cache` 미지정 시 `outputs/yolo_ism_object_n/template_features/<name>_hsv.pt`.
- **내용**: 객체별 `PROTO[name]` = shape `[42, 128]` float64 (§8 규약으로 보정·정규화된 H×S 히스토그램).
- **생성 소스**: 객체 `template_dir` 의 42개 `rgb_*.png`/`mask_*.png` 렌더 쌍. 각 렌더의
  **mask 내부 픽셀**에 −3(OpenCV) hue shift + ×1.3 sat gain 적용 후 §8 히스토그램. 이는 리서치
  아티팩트 `prototype_colors.npz`(`{name}__render42`, raw RGB `[42,20000,3]`,
  `build_prototype_colors.py`)와 **동일 색원**을 운영 경로로 재현하는 것이다 — 운영은 리서치
  npz 에 의존하지 않고 `template_dir` 에서 직접 빌드.
- **캐시 유효성**: `build_template_appe_blocks`(`yolo_ism_object_n.py:147-190`)의 `blocks_key`
  패턴을 참고해, HSV 캐시 blob 에 `{hue_shift, sat_gain, hist_bins}` 서명을 저장하고 불일치 시
  재빌드. 원본 렌더/PLY/모델 파일은 **읽기 전용, 무수정**.
- 42개 렌더의 mask·crop 은 기존 appe 캐시 빌드 루프(`:163-176`)가 이미 materialize 하므로, HSV
  히스토그램을 같은 루프에서 계산해 이미지 재로딩을 피한다(구현 권장, 필수 아님).

---

## 10. Decision Logic

단일 accept 지점은 appearance 게이트(`recognize_frame:514`, `recognize:336`, `recognize_multi:412`).
HSV 는 여기의 **conjunct** 로 삽입한다.

```
# 기존
accept = (best_sem >= similarity_threshold) AND (masked_appe >= appe_gate)

# 목표
sem_pass  = best_sem  >= o["similarity_threshold"]         # 불변
appe_pass = masked_appe >= o["appe_gate"]                  # 불변
if o["hsv_gate_enabled"]:
    hsv_pass = hsv_score >= o["hsv_gate_threshold"]        # ACTIVE
else:
    hsv_pass = True                                        # OFF/SHADOW → 판정 영향 없음
accept = sem_pass AND appe_pass AND hsv_pass
```

- `hsv_score` 는 **sem_pass AND appe_pass 를 통과한 최종 선택 후보에 대해서만** 계산한다(비용
  최소화 + "이미 통과한 후보만 제거" 시맨틱). SHADOW 모드에서는 계산은 하되 `hsv_pass` 를 accept
  에 반영하지 않고 로그에만 남긴다(§11).
- 3개 경로(`recognize_frame` `:514`, `recognize` `:336`, `recognize_multi` `:412`)에 **동일 로직**을
  적용해야 라이브 ROS(=`recognize`)와 배치(=`recognize_frame`)가 일치한다.
- `reject_reason` 우선순위: `no_yolo_candidate` → `yolo_threshold` → `semantic_reject` →
  `appearance_reject` → `hsv_reject` → `accepted`. 앞 단계에서 탈락하면 뒷 단계 미평가.

---

## 11. Logging Contract

수락/거절 결정 추적을 위해 아래 필드를 진단 로그(배치 CSV 확장 컬럼 또는 사이드카 JSONL)로 저장.

**per-candidate 진단(§6 필수 정보):**
```
dataset_name, frame_id, object_name,
candidate_count_before_threshold, candidate_count_after_threshold,
selected_bbox, yolo_confidence,
semantic_score, appearance_score, hsv_score,
semantic_pass, appearance_pass, hsv_pass,
would_hsv_reject,          # (appe_pass) AND (hsv_score < hsv_gate_threshold)
final_decision, reject_reason
```

- `candidate_count_before/after_threshold`: score_threshold 필터 전후 후보 수(§7.1 통일 효과 계측).
- `would_hsv_reject`: SHADOW 모드에서 "HSV 를 켰다면 거절됐을" 후보 표시 — 판정은 불변.
- 배치 경로: 기존 `CSV_FIELDS`(`yolo_ism_object_n.py:96-98`)에 **HSV/count 컬럼만 append**
  (기존 컬럼·순서 불변 → 하위호환).
- 라이브 ROS 경로: `PoseArray`·`detection.json` 필드는 **불변**. HSV 진단은 debug 로그/사이드카로만.

---

## 12. File Change Plan

정확 경로·라인은 baseline `8f86c22` 기준 확인값.

**수정:**
| 파일 | 변경 | 근거 라인 |
|---|---|---|
| `configs/yolo_ism_objects.yaml` | Bear/Rabbit/Dinosaur `score_threshold` override 3줄 제거; `defaults` 에 HSV 키 8개 추가 | `:256, :264, :272`; `:12-13` |
| `yolo_ism_object_n.py` | `_OVERRIDABLE` 에 HSV 키 추가; `load_config` 에 threshold override 경고 검증; `prepare_objects` 에 HSV 템플릿 캐시 로드/빌드 attach(`o["thsv"]`); accept 게이트 3곳에 HSV conjunct + 진단 필드; count 로깅 | `:40-44, :108, :193`; 게이트 `:336, :412, :514`; count `:286/446` |
| `yolo_ism.py` | (원칙적 무수정) HSV 헬퍼가 필요하면 신규 함수만 추가, 기존 함수 시그니처 불변 | `git diff` 최소 |
| `src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py` | 코드 변경 불필요(=`recognize()` 재사용). HSV 는 config 로만 활성. 출력 포맷 불변 검증 대상 | `:217` |

**신규:**
| 파일 | 목적 |
|---|---|
| `tools/build_hsv_template_cache.py` (신규 위치는 구현 시 결정) | 객체 `template_dir`→`<name>_hsv.pt`(§9) 빌드 도구 |
| `tests/test_hsv_gate.py` | HSV 단위 테스트(§13.1) |
| `tests/test_uniform_threshold.py` | threshold 통일 회귀 테스트(§13.1-2) |
| `_ism_research_2026_07/.../eval_offline_b0b1b2.py` | 6-dataset B0/B1/B2 검증(§14). 연구 폴더에 배치(운영 트리 오염 금지) |

> 원본 모델(`mobile_sam.pt`, DINOv2 ckpt)·렌더 템플릿·PLY 는 **읽기 전용**. `yolo_ism.py` 는
> 가급적 무수정(멀티객체 spec 의 "yolo_ism.py 무수정" 불변식 유지).

---

## 13. Unit Test Plan

### 13.1 단위 테스트

- **UT-1** 로드된 전 객체의 `score_threshold == 0.02`(override 부재 확인).
- **UT-2** 객체별 override 가 남아 있으면 로더가 경고를 낸다(하위호환: 거부 아님).
- **UT-3** Hue circular wrap-around: `(h + round(-6/2)) % 180` 이 경계(h=0,1,2,179)에서 음수/오버플로
  없이 0–179 유지.
- **UT-4** 히스토그램 L1 정규화: 임의 크롭에서 `q.sum() ≈ 1`(빈 마스크는 0-div 가드로 유한값).
- **UT-5** 템플릿 캐시 로딩: `<name>_hsv.pt` shape `[42,128]`, 서명 불일치 시 재빌드 트리거.
- **UT-6** threshold 경계값: `sim == hsv_gate_threshold` 는 pass(>=), `sim < t` 는 reject.
- **UT-7** SHADOW 안전성: `enabled=false, shadow=true` 에서 `accept` 불리언이 OFF 모드와 동일,
  `hsv_score`·`would_hsv_reject` 만 채워짐.
- **UT-8** ACTIVE 단조성: `enabled=true` 는 OFF 대비 accept 를 **추가하지 않는다**(HSV 는 제거만) —
  같은 프레임에서 `accepted_active ⊆ accepted_off`.
- **UT-9** 유사도 정합: 동일 히스토그램 self-match `sim==1.0`; 직교 히스토그램 `sim==0.0`.

### 13.2 회귀 테스트(운영 흐름 불변)

- 프롬프트별 YOLO 실행 / 클래스별 후보 라우팅(`build_prompt_groups`) / top_k=3 / semantic Top-5 /
  MobileSAM / masked-appe(block11) / PEM `detection.json`·`PoseArray` 포맷.
- **RG-1** OFF 모드 배치 산출 CSV 가 baseline 과 **field-level 동일**(HSV/count 신규 컬럼 제외).
- **RG-2** OFF 모드 라이브 `detection.json`·`PoseArray` byte/field 동일.

---

## 14. Offline Evaluation Plan

동일 GT(`user_visibility_gt.csv` 935건 가시)·동일 LODO split·`eval_pipeline_end2end.py` 와 동일
후단(sem_top5+appe11+HSV). 신규 스크립트는 연구 폴더에 둔다.

세 구성 비교:
- **B0** = 현행 객체별 threshold + HSV **OFF**(오늘의 실제 운영과 동등) — 결합 델타의 진짜 기준선.
- **B1** = 전 객체 0.02 + HSV **OFF** — threshold 통일 단독 효과.
- **B2** = 전 객체 0.02 + HSV **ACTIVE** — 본 명세 최종.

추가 2모드로 threshold 동결을 검증:
- **B2-lodo**: t_hsv per-fold F1-max(=`eval_pipeline_end2end.py` 재현) → AC-2a 대조.
- **B2-frozen**: t_hsv=`hsv_gate_threshold`(전 6-데이터 F1-max 로 확정한 단일값) → 배포 실제.

산출: `TP/FP/FN/precision/F1` 전체 + **객체별**(특히 Rabbit) + reject_reason 분포.

---

## 15. Acceptance Criteria

- **AC-1 (Threshold 통일)** 로드된 전 객체 `score_threshold==0.02`, override 부재, 클래스 라우팅·
  top_k=3 불변.
- **AC-2a (성능 재현, LODO)** B2-lodo 가 다음을 재현: `TP≥616, FP≤86, FN≤319, F1≥0.752`.
  수치 이탈 시 **threshold 재조정 전에** 먼저 조사: 평가 GT 버전 / HSV reference(render42) 버전 /
  LODO split / 후보 dump / IoU·중복 후보 처리 / frame·object 집계 방식.
- **AC-2b (동결 threshold)** B2-frozen 이 AC-2a 대비 허용 오차 내(FP 증가 0, TP 손실 ≤ 3). 이탈 시
  `hsv_gate_threshold` 를 전 6-데이터 F1-max 로 재확정(§8.2, OQ-1).
- **AC-3 (FP 비증가)** `FP_B2 <= FP_B0`. 현행 대비 FP 증가 시 **실패**.
- **AC-4 (Rabbit 개선)** Rabbit 의 TP↑·FN↓ (객체별 guard 사용 대비). 근거: threshold guard 가
  Rabbit 정답 후보를 제거하던 문제.
- **AC-5 (Shadow 안전성)** SHADOW 모드 최종 detection(배치 CSV 기존 컬럼 / 라이브 `detection.json`·
  `PoseArray`)이 OFF baseline 과 field-level 동일. HSV/count debug 필드만 추가 허용.
- **AC-6 (Rollback)** `hsv_gate_enabled=false` + 객체별 threshold config 원복만으로 현행 동작 복귀.
  원본 모델·템플릿 무수정.

---

## 16. Rollback Plan

- **즉시(런타임)**: `hsv_gate_enabled: false`(+`hsv_gate_shadow_mode: false`) → HSV 완전 OFF. 판정
  = §5 와 동일.
- **threshold 원복**: config 에 Bear 0.35 / Rabbit 0.30 / Dinosaur 0.30 override 3줄 재삽입.
- **완전 원복**: 위 두 config 편집 되돌리기 + HSV 키 제거. 코드가 config-gated 이므로 코드 롤백
  불필요.
- rollback 발동 기준(사전 고정, `sam6d-yolo-localization-yoloe` 정책 계승): FP 가 baseline 대비
  악화(AC-3 위반) 또는 프레임 처리시간 +50% 초과 시.
- HSV 캐시 blob 은 `outputs/` 산출물이라 삭제만으로 원복. 원본 모델/템플릿/PLY 무수정.

---

## 17. Risks

- **R1 (프레임 단위 GT 한계)** AC-2 의 TP/FP 는 "객체가 프레임에 보이는가" 기준이지 **박스 위치
  정확성이 아니다**(`eval_pipeline_end2end.py:18`). 잘못된 위치의 박스라도 색·형태가 맞으면 TP 로
  집계될 수 있다. 배포 후 육안/PEM 하류에서 위치 품질 별도 확인 필요.
- **R2 (t_hsv 하니스 민감도)** t_hsv 는 GT·render42·split 버전에 민감(범위 0.113–0.136). 동결값
  0.122 는 특정 하니스 산출. 다른 데이터·조명에서 재캘리브 필요할 수 있음(AC-2 조사 절차로 흡수).
- **R3 (저채도 예외 이력 충돌)** pinned code 는 예외 없음, 일부 리서치 리포트는 "평균채도<20/255
  객체(Rabbit) Hue 보정 미적용" 권고. 본 명세는 **예외 없음** 채택(AC-2 수치 정합). Rabbit HSV 가
  배포에서 문제되면 `hsv_lowsat_exception` 로 svfall 재도입 검토(별도 story).
- **R4 (증거 기준선 불일치)** §2 "현행"(cur|R0)은 HSV 포함이라 오늘 운영(HSV 없음)과 다르다.
  결합 델타의 진짜 기준선은 §14 B0. B0 를 반드시 측정해 사용자에 보고.
- **R5 (query/reference 비대칭)** hue/sat 보정은 템플릿에만 적용(query 미보정). 실카메라 색
  분포가 바뀌면 보정 오프셋(−6°/×1.3)이 최적이 아닐 수 있음 — 조명 대응은 §4 out-of-scope.
- **R6 (비용)** 수락 후보에만 HSV 계산하므로 프레임당 소수 박스. 그래도 42-템플릿 비교 추가 →
  처리시간 +50% 초과 시 rollback(§16).
- **R7 (경로 이중화)** 게이트가 `recognize`/`recognize_frame`/`recognize_multi` 3곳에 중복. 한 곳만
  고치면 라이브/배치 불일치. 3곳 동시 수정 + RG-1/RG-2 로 강제.

---

## 18. Deferred Follow-up

이번 명세에서 **구현·설계 혼입 금지**. 각각 별도 story:

- **Story 2** — DINOv2 block2+block11 조건부 patch 검증기(`appe_I_b2b11` AUROC 0.9419, FP 89→69).
- **Story 3** — semantic Top-5 vs mean 공통 fusion(α0.3).
- **Story 4** — YOLOE union 의 recall/precision trade-off 검증 및 채택 여부(union|R0 F1 .7560 상한,
  Rabbit TP 30→55 vs Mugcup 45→41 손해; 7.6배 속도). 30건 표본 검수(`sample_review_queue.csv`)
  선행.

---

## 개방 질문 (Open Questions)

- **OQ-1** 배포 `hsv_gate_threshold` 단일값: 시작값 0.122(6-fold 평균) 고정 후, §14 B2-frozen 에서
  전 6-데이터 F1-max 로 확정한다. 확정값이 0.122 와 다르면 config 갱신 — 최종 숫자는 검증 실행
  후 결정(현재 미확정).
- **OQ-2** HSV 진단 로그 물리 포맷: 배치 CSV append vs 사이드카 JSONL. 라이브 ROS 는 debug 로그로
  한정(출력 포맷 불변). 구현 시 사용자 확인 권장.
- **OQ-3** `build_hsv_template_cache.py` 배치 위치(운영 `tools/` vs 연구 폴더). 운영 자산이므로
  운영 트리 권장하나 사용자 선호 확인.

---

## 다음 액션 제안

1. **(추천) 본 명세 승인 → Phase 1A 구현(threshold 통일)부터 착수.** config 3줄 제거 + UT-1/UT-2 +
   RG-1(OFF 동등성) — 가장 낮은 리스크, ΔFP 0 이 이미 근거 확정.
2. **명세 보완**: OQ-1~3 을 지금 결정하고 §7/§11/§12 에 반영.
3. **B0 우선 측정**: 구현 전 §14 B0(현행 HSV 없음) 기준선을 먼저 산출해 결합 델타를 정직하게 확정.

어느 쪽으로 진행할까요? (1 / 2 / 3, 또는 다른 방향)
