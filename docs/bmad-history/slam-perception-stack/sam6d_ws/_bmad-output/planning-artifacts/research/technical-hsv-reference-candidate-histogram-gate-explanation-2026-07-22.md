---
title: 'SAM-6D ISM HSV 색상 hard gate — reference/candidate 히스토그램 생성·비교·판정 전 과정 (교육·발표용)'
type: 'technical-research'
created: '2026-07-22'
status: 'final'
scope: 'code-based investigation only — 운영 코드/설정/캐시/데이터셋 무수정'
authority: 'ism_hsv.py (hsv-authority-1.0-modeB-2026-07-22)'
---

# SAM-6D ISM HSV 색상 hard gate 완전 해설

> **작업 원칙 준수 선언**: 이 조사에서 운영 코드·config·cache·dataset을 **수정하지 않았다.**
> threshold 변경 없음, 알고리즘 신규 구현 없음, 기존 파일 로그 추가 없음, git add/commit/push 없음.
> 아래 수치 중 "실측"으로 표기된 것은 기존 함수를 **읽기 전용으로 호출**해 얻은 값이다.
> 일반적인 HSV/히스토그램 이론 설명은 `[일반 이론]`, 이 프로젝트 구현은 `[본 구현]`으로 구분한다.

---

## 1. Executive Summary

| 항목 | 확정 사실 (코드 근거) |
|---|---|
| 권위 모듈 | `sam6d_ws/ism_hsv.py` (205줄). 파라미터는 **모듈 상수로 동결**(config에서 못 바꿈) |
| 게이트 위치 | semantic(0.35) → MobileSAM mask → masked-appe(0.55) **모두 통과한 뒤**, `accepted=True`가 된 후보에만 적용 |
| 사용 픽셀 (reference) | 렌더 템플릿 42뷰의 `rgb_{i}.png` 중 `mask_{i}.png > 0` 인 픽셀만, 뷰당 20000개 결정론적 샘플 |
| 사용 픽셀 (candidate) | 원본 프레임 BGR의 **bbox crop ∩ MobileSAM full-frame mask** |
| 특징 | **H×S joint 2D 히스토그램, 16×8 = 128차원**, L1 정규화, float64. V(밝기) **미사용** |
| 비교 | `cv2.compareHist` **사용 안 함**. NumPy 직접 구현 Bhattacharyya 계수 → `sim = 1 − √(1 − BC)` |
| 집계 | 42개 뷰 각각과 비교 후 **max** (한 뷰만 색이 맞아도 통과) |
| score 방향 | **[0,1], 높을수록 유사** |
| 판정식 | `hp = bool(score >= thr)` — **`>=` 이므로 정확히 0.1214면 통과** (`yolo_ism_object_n.py:139`) |
| 실패 시 | `res["accepted"]=False`, `res["decision"]="no-object(below-hsv)"` (`yolo_ism_object_n.py:149-150`) |
| Dinosaur +9 | **캐시 빌드 시 reference 픽셀에만** 적용(`(h+9)%180`). 런타임에 객체별 분기 **없음** |
| fail 정책 | **fail-open** (캐시 없음/빈 crop → score 1.0 → 통과) |
| 현재 모드 | `hsv_gate_enabled: true`, `hsv_gate_shadow_mode: false` → **ACTIVE (실제로 제거함)** |

**가장 주의할 발견 3가지**

1. **config의 HSV 파라미터 4개는 읽히지 않는다.** `hsv_hue_shift_deg`, `hsv_sat_gain`, `hsv_hist_bins`,
   `hsv_lowsat_exception`은 `_OVERRIDABLE` 목록(`yolo_ism_object_n.py:47`)에만 존재하고 **어떤 코드도 참조하지
   않는다**(repo 전수 grep 확인). 실제 값은 `ism_hsv.py:30-32`의 상수다. config를 고쳐도 동작은 안 바뀐다.
2. **라이브 ROS 노드에는 HSV 게이트가 없다.** `ism_hsv.py` docstring은 "두 운영 진입점이 동일 score를 계산한다"고
   적혀 있으나, `src/sam6d_ros/sam6d_ros/sam6d_multiobject_node.py`에는 `hsv` 문자열이 **0회** 등장한다. 다만 이
   노드는 `o_n.recognize()`를 호출하므로(`sam6d_multiobject_node.py:40` import) **게이트는 recognize 내부에서
   실제로 걸린다** — 노드가 HSV 필드를 로깅/발행하지 않을 뿐이다. (docstring이 부정확, 동작은 정상)
3. **Hue의 원형성(circularity)이 비교 단계에서는 무시된다.** hue 보정 시에는 `% 180`으로 원형 처리하지만,
   히스토그램 bin은 선형이고 Bhattacharyya는 bin 간 이동 비용이 없다 → H=179와 H=0은 "가장 먼" 두 bin이 된다.

---

## 2. 한 문장 설명

> **HSV gate는, semantic·appearance 검증을 이미 통과한 검출 후보에 대해, MobileSAM 마스크 내부 픽셀의
> 색상(Hue)–채도(Saturation) 분포를 미리 렌더링해 둔 42장 템플릿의 색 분포와 비교하여, 가장 잘 맞는 한 뷰와의
> 유사도가 0.1214 미만이면 "색이 명백히 다르다"는 이유로 그 후보를 제거하는 마지막 색상 검증 단계이다.**

---

## 3. HSV gate가 파이프라인에서 담당하는 역할

세 검증기는 서로 **다른 질문**을 한다.

| 검증기 | 질문 | 특징 | 임계 | 코드 |
|---|---|---|---|---|
| semantic | "이 crop 전체가 등록 객체의 **개념/형태**와 닮았나?" | DINOv2 **CLS** 토큰 코사인, top-5 평균 | 0.35 | `yolo_ism.semantic_score` |
| appearance | "마스크 내부 **국소 질감**이 템플릿과 닮았나?" | DINOv2 block11 **patch** 토큰 코사인 | 0.55 | `masked_appe_blocks` (`yolo_ism_object_n.py:420~`) |
| **HSV** | "마스크 내부 **색 구성**이 렌더 색 분포와 충분히 닮았나?" | H×S 히스토그램 Bhattacharyya | 0.1214 | `ism_hsv.shadow_score` |

- HSV gate는 **detector가 아니다.** 새 후보를 만들지 않는다. `accepted=True`인 후보만 본다.
- HSV gate는 **classifier가 아니다.** 여러 객체 중 하나를 고르지 않는다. 이미 라벨이 정해진 후보에
  대해 "그 라벨의 색이 맞는가"만 본다(cross-object 순위 결정은 별도의 `nms_rank_blocks` 경로).
- HSV gate는 **feature matcher/pose estimator가 아니다.** 히스토그램은 **픽셀 위치 정보를 전부 버린다.**
- 정확한 정의: **negative verifier (hard gate)** — "명백히 색이 다른 것을 떨어뜨리는" 단방향 필터.

GT 935건 검증 실적(`phase1c-hsv-dinosaur-color-correction-implementation.md` §11):
FP 334 → 86 (−74%), Precision 0.660 → 0.879, 대가로 Recall 0.693 → 0.665.

---

## 4. 전체 데이터 흐름

### 4.1 실제 코드 흐름 (배치 진입점 `recognize_frame`)

```
configs/yolo_ism_objects.yaml
      │  load_config()           yolo_ism_object_n.py:152-190
      ▼
prepare_objects()               yolo_ism_object_n.py:~230-290
      ├─ tcls / tappe 템플릿 특징 로드
      └─ [HSV] ism_hsv.load_cache(o["hsv_cache"], tdir)      :272   ← 캐시 I/O는 여기 "단 1회"
             · hue_correction_ocv(cache) != config → proto=None (stale 취급)  :274-276
             · o["_hsv_proto"] (42×128), o["_hsv_meta"] 메모리 상주            :278
                     ↓ (이후 프레임 루프. 캐시 재읽기 0회)
─────────────────────── 프레임 루프 ───────────────────────
RGB 프레임 (bgr, rgb)
      ↓
YOLO-World predict  (프롬프트 그룹당 공유 1-pass)
      ↓
score_threshold 0.02 필터  →  top_k = 3
      ↓
DINOv2 1-pass (프레임의 모든 crop 동시)  →  CLS
      ↓
semantic_score  ≥ 0.35 ?  ── No ─→ decision="no-object(below-sim)"   (HSV 계산 안 함)
      │ Yes                                    ※ 객체당 최고 sem 후보 1개만 생존
      ▼
MobileSAM segment_boxes()  → full-frame bool mask                     :565
      ↓
masked_appe_blocks(block 11)  ≥ 0.55 ?  ── No ─→ "no-object(below-appe)"
      │ Yes → accepted=True, decision="detected"
      ▼
_apply_hsv_gate(o, res, bgr)                        yolo_ism_object_n.py:596
      ├─ 가드: HSV off / not accepted / box None → 즉시 return          :135
      ├─ score = ism_hsv.shadow_score(bgr, res["box"], res["mask"], o["_hsv_proto"])   :138
      ├─ hp = (score >= 0.1214)                                          :139
      ├─ CSV 필드 기록: hsv_score/hsv_threshold/hsv_pass/would_hsv_reject/…  :140-146
      └─ enabled AND not hp →  accepted=False, decision="no-object(below-hsv)"  :148-150
      ↓
accept → PEM(6D pose) / detection.json / PoseArray
```

### 4.2 확인된 위치 관계

| 질문 | 답 | 근거 |
|---|---|---|
| semantic 이전/이후? | **이후** | semantic 실패 시 `return`/`continue`로 `_apply_hsv_gate` 미도달 (`:385-387`, `:557-560`) |
| appearance 이전/이후? | **이후** | `_apply_hsv_gate` 호출이 appe 분기 뒤 (`:412`, `:596`) |
| MobileSAM mask 이전/이후? | **이후** | `res["mask"] = mask` (`:404`, `:587`) 가 게이트보다 앞. 마스크를 **입력으로 씀** |
| 후보별 호출 횟수 | **객체당 프레임당 최대 1회** | semantic 단계에서 객체당 best 후보 1개만 남음(`st["best"]`) |
| 프레임당 캐시 읽기 | **0회** (준비 단계 1회만) | `prepare_objects:272`, 루프 안에 `load_cache` 없음 |
| 호출 지점 개수 | **4곳** | `:396`(use_mask=false, recognize), `:412`(recognize), `:582`(use_mask=false, recognize_frame), `:596`(recognize_frame) |
| 실패 후보 상태 | `accepted=False`, `decision="no-object(below-hsv)"` | `:149-150` |
| HSV off일 때 | 함수가 **즉시 return**, `hsv_*` 키 자체가 res에 생기지 않음 | `:135`, 테스트 `test_disabled_is_noop` |

### 4.3 shadow vs active

| | `hsv_gate_enabled` | `hsv_gate_shadow_mode` | score 계산 | CSV 기록 | 판정 변경 |
|---|---|---|---|---|---|
| OFF | false | false | ✗ | ✗ | ✗ |
| SHADOW (Phase 1B) | false | true | ✓ | ✓ (`would_hsv_reject`) | **✗** |
| **ACTIVE (현재)** | **true** | false | ✓ | ✓ | **✓ 제거** |

`_hsv_on()` (`:122-124`)이 "계산 여부", `hsv_gate_enabled`(`:148`)가 "제거 여부"를 각각 결정한다.
현재 운영 config(`configs/yolo_ism_objects.yaml:16-17`)는 `enabled: true / shadow: false` → **ACTIVE 확정**.
`tests/test_phase1c.py::test_config_active`가 이를 강제한다.

---

## 5. Reference HSV 캐시 생성 과정

### 5.1 입력 이미지 (`ism_hsv.build_reference`, :113-142)

| 항목 | 확정 사실 |
|---|---|
| 디렉터리 | config `template_dir` (예: `template/Dinosaur/templates`) |
| 파일 | `rgb_{i}.png`, `mask_{i}.png`, `xyz_{i}.npy` — **i = 0..41 고정 루프** (`for i in range(42)`) |
| 포맷 | RGB/mask는 **PNG**, xyz는 **.npy** |
| 로딩 | `cv2.imread(rp)` → **BGR, uint8, (512,512,3)** (실측). alpha 채널 **없음**(IMREAD_UNCHANGED로도 3채널) |
| 배경 | 알파가 아니라 별도 `mask_{i}.png`(uint8, 값 {0,255})로 표현. rgb의 배경 픽셀은 검정이 아니라 **회색 계열**(실측 `rgb_0.png[0,0] = [63,69,69]`) → 마스크 없이는 반드시 오염됨 |
| 뷰 개수 | 최대 42, 실제 Dinosaur = **42/42 사용**(캐시 `n_view=42`, `proto.shape=(42,128)` 실측) |
| 선택 | 전 뷰 사용. 대표 뷰 선택 **없음** |
| xyz의 역할 | 픽셀에는 안 쓰이고 **존재 검사만** — 3파일이 모두 있어야 그 뷰를 채택(`:130`, 연구 코드 parity 목적) |

> 참고: `template/Dinosaur/templates`에는 126개 파일(=42×3)이 있다.

### 5.2 객체 픽셀 선택

실제 코드(`ism_hsv.py:127-137`):

```python
m   = cv2.imread(mp, cv2.IMREAD_GRAYSCALE) > 0     # :127  binary mask
bgr = cv2.imread(rp)
if bgr is None or m.sum() < MIN_MASK_PX:  continue  # :129  MIN_MASK_PX = 50
px  = _sub(bgr[m][:, ::-1], seed=i)                 # :135  마스크 내부만, BGR→RGB
```

선택 조건을 수식으로:

```
valid_pixel(x,y)  =  mask_i(x,y) > 0
view_accepted(i)  =  exists(rgb_i) AND exists(mask_i) AND exists(xyz_i)
                     AND count(valid_pixel) >= 50
```

- **saturation / value 조건 없음.** `hsv_lowsat_exception: false`가 config에 있지만 코드에는 저채도 예외
  자체가 없다(docstring `ism_hsv.py:23-24`가 "mode-B에는 저채도 예외가 없다"고 명시).
- **안티에일리어싱 경계 픽셀**: 마스크가 `>0` 이진화이므로, 렌더러가 반투명 경계를 마스크에 포함시켰다면
  경계의 배경 혼합색도 포함된다. 마스크 자체는 {0,255} 2값(실측)이라 **소프트 경계는 이미 이진화된 상태**.
- **유효 픽셀 부족**: 뷰 단위 스킵(50px 미만). 전 뷰가 스킵되면 `(0,128) 빈 proto, n_view=0` 반환(`:139-140`)
  → 빌더가 `[WARN empty]` 출력하고 캐시를 **쓰지 않는다**(`build_hsv_template_cache.py:46-48`).

**결정론적 샘플링 `_sub`** (`:100-110`): 뷰당 정확히 `SUB_N = 20000` 픽셀.
- N > 20000 → `RandomState(i).choice(N, 20000, replace=False)` 비복원 추출
- N < 20000 → 원본 전체 + `randint`로 뽑은 복원 추출로 채워 20000개 맞춤
- seed = **뷰 인덱스 i** → 재실행 시 완전 동일. 연구 `build_prototype_colors.sub()` 재현이 목적.

### 5.3 BGR/RGB → HSV 변환

경로가 **두 번 뒤집힌다**. 정확히 따라가면:

```
cv2.imread(rgb_i.png)            → BGR
bgr[m][:, ::-1]                  → RGB   (ism_hsv.py:135)
[_apply_hue_ocv 내부]  [:,:,::-1] → BGR → cv2.COLOR_BGR2HSV → 보정 → COLOR_HSV2BGR → [:, ::-1] → RGB   (:78-84)
[feat_rgb 내부]        [:,:,::-1] → BGR → cv2.COLOR_BGR2HSV                                        (:41-42)
```

- 사용 함수는 **`cv2.COLOR_BGR2HSV` 단 하나** (`RGB2HSV`는 repo 운영 코드에 없음).
- 채널 순서 해석은 **정합**: 함수 인터페이스가 "RGB 픽셀 리스트"이고 진입 시마다 `::-1`로 BGR로 되돌려
  BGR2HSV에 넣는다. 왕복이 많아 읽기 어렵지만 **오류는 없다**(테스트 `test_rgb_to_hsv_authority_parity`가
  연구 prototype과 `max|diff| == 0.0` 임을 검증).
- 자료형: `uint8` 입력 → OpenCV 8-bit HSV → **H ∈ [0,179], S ∈ [0,255], V ∈ [0,255]** (`[일반 이론]`이자
  `[본 구현]`; 히스토그램 range `[0,180]`이 이를 확정).
- **1 OpenCV hue unit = 2° (0-360 기준)** — OpenCV가 360°를 180단계로 압축하기 때문. 테스트가 명시적으로
  이 관계를 문서화한다(`tests/test_phase1c.py::test_hue_wraparound_degrees_doc`: `assert 9*2 == 18`).

### 5.4 Reference 전역 색 보정 (`feat_rgb`, :38-48)

```python
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)              # :42  int16 승격
hsv[..., 0] = (hsv[..., 0] + int(round(HUE_SHIFT_DEG / 2))) % 180        # :43  -6/2 = -3 ocv, 원형
hsv[..., 1] = np.clip(hsv[..., 1] * SAT_GAIN, 0, 255)                    # :44  ×1.3, 클립
```

- `HUE_SHIFT_DEG = -6` (0-360 기준) → OpenCV 단위로 **-3**. 전 객체 공통.
- `SAT_GAIN = 1.3` → 렌더가 실사보다 채도가 낮은 domain gap 보정. 전 객체 공통.
- **query에는 이 보정이 없다.** 보정은 "렌더를 실사 쪽으로 옮기는" 단방향 캘리브레이션.
- `int16` 승격 덕에 `-3` 더하기에서 **uint8 언더플로가 없다**(`% 180`이 음수를 다시 [0,180)로 넣는다).

### 5.5 히스토그램 구성

```python
x = cv2.calcHist([hsv.astype(np.uint8)], [0, 1], None,
                 [HBINS, SBINS], [0, 180, 0, 256]).flatten()   # ism_hsv.py:45-46
return (x / max(x.sum(), 1e-12)).astype(np.float64)            # :47
```

| 항목 | 값 |
|---|---|
| 사용 채널 | **H(0)과 S(1)** — **V는 미사용** (조명 밝기 불변성 확보) |
| 차원 | **2D joint 히스토그램** (H와 S의 결합 분포, 주변분포 2개가 아님) |
| bin | H = 16, S = 8 |
| bin 폭 | H: 180/16 = **11.25 ocv unit = 22.5°**, S: 256/8 = **32** |
| range | H `[0,180)`, S `[0,256)` |
| 함수 | **`cv2.calcHist`** (NumPy histogram2d/histogramdd 아님) |
| 값 | 픽셀 카운트 → L1 정규화 |
| 정규화 | **L1** (`x / x.sum()`), 즉 **확률분포**. L2/min-max 아님 |
| 0 나눗셈 | `max(x.sum(), 1e-12)` (reference) / `if s > 0 else x` (query) → **NaN 불가** |
| smoothing | **없음** (라플라스 보정·커널 스무딩 모두 없음) |
| flatten | **함** (16×8 → 1D 128) |
| dtype | calcHist는 float32 반환 → **float64로 캐스팅** 후 저장 |

```
feature dimension = N_H × N_S = 16 × 8 = 128
reference bank    = [V, 128] = [42, 128]      (Dinosaur 실측 shape)
```

실측 검증: `Dinosaur_hsv.npz` proto의 행 합 = `[1.00000001, 0.99999999, ...]` (float32 누적 오차 범위), 최소값 0.0.

### 5.6 여러 템플릿의 결합 방식

**결합하지 않는다.** 42뷰 각각의 히스토그램을 **개별 행으로 보존**한다 (`:141  return np.stack(feats)`).

```
build_reference → np.stack([feat_rgb(view_0), …, feat_rgb(view_41)])  →  proto[42, 128]
```

- 평균/median/대표뷰/hue-cluster prototype — **전부 아님**.
- 결합은 **런타임에 score 단계에서 max로** 수행된다(§7).
- **영향**: "뷰별 조명·자기그림자로 색이 변하는 것"을 평균으로 뭉개지 않고, 후보가 **어느 한 뷰와만
  맞아도 통과**시킨다 → recall 보호, 대신 42번의 기회를 주므로 FP 제거력은 약해지는 방향.

### 5.7 도식 A — Reference cache build (실제 순서)

```
template/<obj>/templates/{rgb_i.png, mask_i.png, xyz_i.npy}   i = 0..41
        │
        ├─ 3파일 존재 + mask 픽셀 ≥ 50 인 뷰만 채택            (ism_hsv.py:123-131)
        ▼
cv2.imread → BGR  ──[ mask>0 인 픽셀만 ]──> bgr[m]             (:127, :135)
        ▼
BGR → RGB  ( [:, ::-1] )                                       (:135)
        ▼
_sub(seed=i)  : 결정론적 20000 픽셀 샘플                        (:100-110)
        ▼
★ 클래스 hue 보정  _apply_hue_ocv(px, +9)   ← Dinosaur만        (:136, :74-84)
   RGB→BGR→HSV,  H = (H + 9) % 180,  HSV→BGR→RGB
        ▼
★ 전역 보정 + 히스토그램  feat_rgb                              (:38-48)
   RGB→BGR→HSV(int16), H = (H − 3) % 180, S = clip(S×1.3,0,255)
        ▼
cv2.calcHist([H,S], bins=[16,8], range=[0,180]×[0,256]) → 128-D (:45)
        ▼
L1 정규화 (x / Σx)                                              (:47)
        ▼
np.stack(42개) → proto [42,128] float64                         (:141)
        ▼
save_cache → <feature_dir>/<name>_hsv.npz  (+ signature/버전/메타)  (:149-157)
```

> **핵심**: hue 보정은 **HSV 변환 직후·히스토그램 생성 이전에, 픽셀 단위로** 두 번(클래스 +9 → 전역 −3)
> 적용된다. 히스토그램 bin을 사후에 굴리는 방식이 **아니다**. 그래서 bin 폭(11.25 ocv)보다 작은 보정도
> 정확히 반영된다.

---

## 6. Runtime candidate HSV 특징 생성 과정

### 6.1 입력 영역 (`shadow_score`, :192-205)

```python
x1, y1, x2, y2 = (int(v) for v in box)
crop  = bgr[y1:y2, x1:x2]            # :199  원본 프레임 BGR의 bbox crop
if crop.size == 0: return 1.0        # :201  fail-open
mcrop = mask[y1:y2, x1:x2] if mask is not None else None   # :202
q = query_hist(crop, mcrop)          # :203
```

| 후보 | 사용? |
|---|---|
| 원본 RGB 전체 이미지 | ✗ |
| YOLO bbox crop | ✓ (BGR 원본 해상도) |
| MobileSAM mask 내부 | ✓ |
| **bbox ∩ mask** | ✓ **← 실제 사용 영역** |
| resize / padding crop | ✗ (**리사이즈·패딩 없음** — appe 경로의 `crop_resize_pad`와 다름) |
| depth mask 결합 | ✗ |

좌표계 변환:

```
원본 프레임 좌표 (H,W)
   │  bgr[y1:y2, x1:x2]               → crop 좌표 (h,w) = (y2-y1, x2-x1)
   │  mask[y1:y2, x1:x2]              → 동일 crop 좌표계 (mask는 full-frame이라 동일 슬라이스)
   ▼
cv2.calcHist(..., mask=m)             → m>0 인 픽셀 인덱스만 누적
```

**전달 경로**: `segment_boxes()`(`yolo_ism.py:289-310`)가 full-frame **bool** 마스크 생성(필요 시 원본
해상도로 `INTER_NEAREST` 리사이즈) → `res["mask"] = mask`(`yolo_ism_object_n.py:404`/`:587`) →
`_apply_hsv_gate`가 `res.get("mask")`로 꺼냄(`:138`) → `shadow_score`가 bbox로 슬라이스.

### 6.2 Mask 처리 (`query_hist`, :50-62)

```python
hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)          # :55
m = None
if mask_crop is not None:
    m = (np.asarray(mask_crop).astype(np.uint8) * 255)    # :58  bool → {0,255}
    if int((m > 0).sum()) == 0:  m = None                 # :59-60  전부 0이면 마스크 해제
x = cv2.calcHist([hsv], [0, 1], m, [16, 8], [0,180,0,256]).flatten()   # :61
s = x.sum();  return (x / s if s > 0 else x).astype(np.float64)        # :62-63
```

| 항목 | 확정 사실 |
|---|---|
| dtype/shape | 입력 bool full-frame → crop 후 `uint8×255` (OpenCV mask 규약) |
| threshold | 별도 threshold 없음. **이미 bool 이진 마스크** (soft mask 아님) |
| mask=None | **bbox 전체 픽셀 사용** (배경 포함) — 조용한 fallback |
| mask 전부 0 | 위와 동일하게 **bbox 전체로 fallback** (`:59-60`) — 최소 면적 하한 **없음** |
| bbox 밖 픽셀 | 슬라이스 자체가 bbox이므로 **구조적으로 차단됨** |
| hole/파편 마스크 | 별도 처리 없음. MobileSAM 결과 그대로 |
| 저채도·저명도 제거 | **없음** (검정 픽셀도 H=0,S=0 bin에 그대로 누적) |
| depth 결합 | 없음 |

### 6.3 Reference vs Runtime 전처리 대조표

| 항목 | Reference (`feat_rgb`+`build_reference`) | Runtime candidate (`query_hist`) | 동일? |
|---|---|---|---|
| 원본 채널 순서 | imread→BGR, 내부에서 RGB↔BGR 왕복 | 프레임 BGR 그대로 | 실질 동일 |
| HSV 변환 함수 | `cv2.COLOR_BGR2HSV` (:42) | `cv2.COLOR_BGR2HSV` (:55) | **✓** |
| 유효 픽셀 조건 | 렌더 mask>0, 뷰 ≥50px | MobileSAM mask>0 (없으면 bbox 전체) | 개념 동일, **fallback 차이** |
| 픽셀 샘플링 | 뷰당 20000 결정론 샘플 | **전 픽셀** (샘플링 없음) | **✗** |
| 히스토그램 채널 | H, S | H, S | ✓ |
| bin 수 | 16 × 8 | 16 × 8 | ✓ |
| range | [0,180]×[0,256] | [0,180]×[0,256] | ✓ |
| 정규화 | L1 | L1 | ✓ |
| dtype | float64 (calcHist float32 → cast) | float64 | ✓ |
| **Hue 보정** | 전역 **−3 ocv** + 클래스 보정(Dino +9) | **없음** | **✗ 의도적 비대칭** |
| **Saturation 보정** | **×1.3** | **없음** | **✗ 의도적 비대칭** |
| resize/interpolation | 없음(렌더 원본) | 없음(원본 crop) | ✓ |

**비대칭의 이유와 위험**
- *이유*: 보정은 "렌더 → 실사" 방향의 domain-gap 캘리브레이션이므로 한쪽(reference)에만 걸어야 한다.
  양쪽에 걸면 상대 차이가 0이 되어 의미가 없다. 연구 mode B와의 parity를 위해 동결되어 있다.
- *위험*: 보정값(−6°, ×1.3, Dino +9)은 **검증에 쓰인 데이터셋의 카메라/조명에 종속**된다. 화이트밸런스가
  다른 새 환경에서는 reference가 잘못된 방향으로 이동해 TP를 깎을 수 있다(Phase 1C §13이 같은 위험 명시).
- *샘플링 차이*: reference만 20000 샘플. L1 정규화 후 **분포 추정치**를 비교하므로 편향은 없지만, 마스크
  픽셀 수가 20000보다 훨씬 적은 뷰는 복원추출로 부풀려져 **분산이 과소평가**될 수 있다.

---

## 7. 히스토그램 비교 함수와 수식

### 7.1 실제 구현 (`ism_hsv.similarity`, :64-70)

```python
def similarity(q, proto):
    if proto is None or len(proto) == 0:
        return 1.0                                          # :67-68  fail-open
    bc = np.sqrt(np.maximum(q[None, :] * proto, 0.0)).sum(1) # :69
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())   # :70
```

**`cv2.compareHist`를 쓰지 않는다.** NumPy로 직접 구현한 Bhattacharyya 계열이다.

수식:

```
BC_i  =  Σ_{b=0..127}  sqrt( q[b] · p_i[b] )          (Bhattacharyya coefficient, 뷰 i)
sim_i =  1 − sqrt( 1 − BC_i )
score =  max_{i=0..41} sim_i
```

### 7.2 `[일반 이론]` vs `[본 구현]`

`[일반 이론]` OpenCV `HISTCMP_BHATTACHARYYA`가 돌려주는 것은 **거리** `d_H = sqrt(1 − BC)` 이고,
- 두 분포가 동일 → BC = 1 → d_H = 0
- 겹침이 전혀 없음 → BC = 0 → d_H = 1
- **작을수록 유사**

`[본 구현]` 그 거리를 그대로 유사도로 뒤집는다: `sim = 1 − d_H`.

| 상황 | BC | d_H | **sim (본 구현 score)** |
|---|---|---|---|
| 완전히 동일한 히스토그램 | 1 | 0 | **1.0** (최대) |
| 전혀 겹치지 않는 히스토그램 | 0 | 1 | **0.0** (최소) |
| 절반 겹침 (예 BC=0.5) | 0.5 | 0.707 | 0.293 |
| BC = 0.9 | 0.9 | 0.316 | 0.684 |

- **범위 [0,1], 값이 클수록 유사.** (`yolo_ism_object_n.py:139` 주석이 "higher score = more similar"로 명시)
- **주의**: L1 정규화된 두 분포에 대해 `BC ≤ 1`이 수학적으로 보장되지만(Cauchy–Schwarz), 부동소수 오차로
  `1 − BC`가 미세 음수가 될 수 있어 `np.maximum(0.0, ·)`으로 **NaN을 원천 차단**한다(:70).
- 마찬가지로 `np.maximum(q*proto, 0)`(:69)로 음수 곱 방지 → **`sqrt`의 NaN 불가**.
- 빈 히스토그램(모두 0, query 쪽): BC = 0 → sim = 0 → **fail-closed(제거)**.
- 빈 proto(캐시 미스): **1.0 반환 → fail-open(통과)**. 두 "빈" 케이스의 정책이 **반대**임에 유의.

### 7.3 sim 값이 왜 이렇게 작은가 (중요한 직관)

`sim = 1 − sqrt(1 − BC)` 는 BC가 1에 매우 가까워야만 커진다. BC = 0.5여도 sim은 0.29에 불과하다.
threshold 0.1214를 BC로 환산하면:

```
sim ≥ 0.1214  ⟺  sqrt(1 − BC) ≤ 0.8786  ⟺  BC ≥ 0.2280
```

즉 **"128차원 분포끼리 Bhattacharyya 겹침이 22.8% 이상이면 통과"** 라는 뜻이다. 매우 관대한 기준이며,
이것이 HSV gate가 "약간 다른 색"이 아니라 **"명백히 다른 색"만 떨어뜨리는 negative verifier**인 이유다.

---

## 8. 여러 reference와의 matching 정책

### 8.1 정책 = **max over 42 views**

```
hsv_score = max_{i ∈ 채택된 뷰}  [ 1 − sqrt(1 − BC(q, p_i)) ]
```

| 질문 | 답 |
|---|---|
| 최종 score 수식 | 위 식 (`ism_hsv.py:70`의 `.max()`) |
| 어느 뷰가 결정했는지 알 수 있나 | **기록되지 않는다.** `.max()`만 반환, `argmax` 미보존. best template id/index 로그 **없음** |
| 한 뷰만 맞아도 통과? | **예.** 42개 중 최댓값 하나로 결정 |
| view-dependent 조명 흡수 | 42뷰가 각기 다른 자기그림자/하이라이트를 가지므로 그중 가장 유사한 뷰가 자동 선택됨 |
| FP/FN 영향 | **FN↓ / FP↑ 방향.** 42번의 기회는 recall을 지키지만, 우연히 한 뷰와만 맞는 오검출도 통과시킨다. mean/median이었다면 반대(FP↓/FN↑) |

### 8.2 실측 숫자 예시 (Dinosaur 캐시, 읽기 전용 실행)

렌더 뷰 3(`rgb_3.png`)을 "카메라 후보"인 것처럼 넣어 실제 함수를 호출한 결과:

```
후보 = Dinosaur 렌더 뷰 3 (mask 적용)
  vs Dinosaur proto[42] → hsv_score = 0.5909   ≥ 0.1214  → PASS
  vs milk      proto[42] → hsv_score = 0.0000   < 0.1214  → below-hsv (완전 배제)
  vs Dinosaur, 단 mask 없이 bbox 전체 → 0.4065  (배경 유입으로 0.59 → 0.41 하락)
```

> **마스크의 효과가 수치로 드러난다**: 같은 후보인데 배경을 포함시키면 score가 31% 떨어진다.

max 정책 vs 다른 정책의 대비(§10의 4×4 예제에서 실측한 42뷰 개별 sim):

```
per-view sim:  min 0.0258,  median 0.0554,  max 0.1104
  max    정책 → 0.1104  <  0.1214 → 탈락
  mean   정책 → ≈0.058  <  0.1214 → 탈락 (훨씬 큰 폭으로)
  median 정책 → 0.0554  <  0.1214 → 탈락
```

이 사례에서는 세 정책이 같은 결론이지만, **max와 mean의 값 차이가 2배**라는 점이 중요하다.
threshold 0.1214는 **max 정책 위에서 튜닝된 값**이므로 집계 방식을 바꾸면 threshold를 반드시 재보정해야 한다.

---

## 9. Threshold 0.1214 판정 로직

### 9.1 판정식 (확정)

```python
# yolo_ism_object_n.py:137-139
thr   = float(o.get("hsv_gate_threshold", 0.0))
score = ism_hsv.shadow_score(bgr, res["box"], res.get("mask"), o.get("_hsv_proto"))
hp    = bool(score >= thr)              # ← 비교 연산자는 >=

# :148-150
if bool(o.get("hsv_gate_enabled", False)) and not hp:
    res["accepted"] = False
    res["decision"] = "no-object(below-hsv)"
```

정리하면:

```
PASS   ⟺  hsv_score >= 0.12140
REJECT ⟺  hsv_score <  0.12140   →  decision = "no-object(below-hsv)"
```

**score == 0.1214 이면 통과한다** (`>=`).

### 9.2 threshold 소스와 override

| 질문 | 답 |
|---|---|
| 어디서 읽나 | `configs/yolo_ism_objects.yaml` → `defaults.hsv_gate_threshold: 0.12140` (:18) |
| 코드 기본값 | `o.get("hsv_gate_threshold", 0.0)` → **키가 없으면 0.0** = 사실상 전원 통과 |
| 객체별 override | **가능.** `_OVERRIDABLE`(`:46`)에 포함, `load_config`가 `o = dict(defaults); o.update(raw)`로 병합(`:167-168`). 현재 config는 **어떤 객체도 override하지 않음** (전 객체 0.1214) |
| CLI override | **없음.** `parse_args()`(`:604~`)에 HSV 관련 인자 없음 |
| ROS 파라미터 override | **없음.** 노드는 config 파일 경로만 받고 HSV 키를 건드리지 않음 |
| cache metadata와 연결 | **연결 안 됨.** 캐시에는 hue_shift/sat_gain/bins/hue_correction만 저장되고 **threshold는 저장되지 않는다** → threshold만 바꿔도 캐시는 stale이 되지 않는다 |

### 9.3 실패/이상 케이스 정책

| 케이스 | 반환 | 정책 |
|---|---|---|
| 캐시 missing/stale/corrupt/authority-mismatch → proto=None | sim = 1.0 (`:67-68`) | **fail-OPEN** (통과) |
| hue_correction 캐시≠config → proto=None (`:274-276`) | sim = 1.0 | **fail-OPEN** |
| bbox 면적 0 → crop.size == 0 | 1.0 (`:200-201`) | **fail-OPEN** |
| mask가 전부 0 | mask 무시하고 bbox 전체로 계산 (`:59-60`) | 계산은 계속 (silent) |
| query 히스토그램이 전부 0 | sim = 0.0 | **fail-CLOSED** (제거) |
| gate disabled(shadow) | score/필드만 기록 (`:140-146`), 판정 불변 | 로깅만 |
| gate enabled | `accepted=False` 로 **실제 제거** | 제거 |

테스트가 이 정책들을 고정한다: `test_failopen_empty_crop`, `test_failopen_missing_proto`,
`test_shadow_does_not_change_decision`, `test_active_rejects_on_fail`, `test_not_computed_when_rejected`
(`tests/test_phase1c.py`, 12/12 PASS 기록).

### 9.4 threshold 산출 근거 (Phase 1C/1B 산출물에서 추적)

`_bmad-output/implementation-artifacts/phase1c-hsv-dinosaur-color-correction-implementation.md` §6·§11 기준:

| 항목 | 값 |
|---|---|
| 검증 데이터 | **사람 라벨 visible-object GT 935건** |
| 후보 grid | 수락 후보 HSV score의 **quantile** |
| 목적함수 | **F1-max** (연구 mode B와 동일) |
| 교차검증 | **LOO 6폴드(데이터셋 단위)** — `t* = 0.1214`가 **6폴드 전부 동일**, 전 폴드 FP 감소 |
| 근거 파일 | `phase1b_validation/fold_freeze.csv`, `outputs/phase1c_hsv_dinosaur_color_correction/csv/phase1c_threshold_sweep_hsv.csv` |
| confusion (구성 B, Dino 미보정) | TP 616 / FP 86 / FN 319 |
| **confusion (구성 C = 현행)** | **TP 622 / FP 86 / FN 313**, Precision 0.879, Recall 0.665, **F1 0.7572** |
| baseline (HSV OFF) | TP 648 / FP 334 / FN 287, P 0.660, R 0.693, F1 0.6761 |
| FP 제거율 | **334 → 86 (−74.3%)** |
| TP 유지율 | **622/648 = 96.0%** (TP casualty 26~32건) |
| 민감도 | t=0.10 → FP 94, t=0.05 → FP 143. 0.1214가 FP 최소·F1 최대 |
| Dinosaur 보정 전후 | 보정 전 TP 33 / FP 0 / FN 55 → 보정 후 **TP 39 / FP 0 / FN 49** |
| 실 GPU 확인 | sam_110633 25프레임 → 12건 `no-object(below-hsv)` 제거, hsv 필드 55건 기록 |

**AUROC**: HSV score 자체의 AUROC 수치는 Phase 1C 산출물에 **기재되어 있지 않다**(AUROC 0.834/0.508은
채택되지 않은 texture verifier의 값이다). §21에 미확인 사항으로 기록한다.

---

## 10. 실제 숫자를 사용한 단계별 예제

> **주의**: 아래 4×4 이미지는 설명용 축소 예시이지만, **bin 수(16×8)·range·정규화·비교식·threshold는 실제
> 구현 그대로**이고, score는 `ism_hsv` 함수를 **실제로 호출해 얻은 실측값**이다. 축약한 것은 이미지 크기와
> 표에 나열한 bin 개수뿐이다.

**1단계 — 4×4 BGR 이미지와 마스크**

```
BGR crop (4×4)                     mask (4×4)
. . . .                            0 0 0 0
. G G .        G = (B,G,R)=(40,180,60)     0 1 1 0
. G G .        . = (0,0,0)  검정 배경       0 1 1 0
. . . .                            0 0 0 0
```

**2~3단계 — foreground 픽셀 선택**: mask>0 인 **4픽셀**만 사용. 검정 배경 12픽셀은 `calcHist`의 mask 인자로
**누적에서 제외**된다.

**4단계 — 픽셀별 HSV (실측)**

```
cv2.cvtColor(BGR2HSV) → H=56, S=198, V=180     (4픽셀 모두 동일)
H=56 ocv = 112°  (초록)
```

**5단계 — bin 배정**

```
H bin = floor(56 / (180/16)) = floor(56 / 11.25) = 4      (H bin 4 = [45.0, 56.25) ocv = [90°,112.5°))
S bin = floor(198 / (256/8)) = floor(198 / 32)   = 6      (S bin 6 = [192, 224))
flat index = 4 × 8 + 6 = 38
```

**6~7단계 — 카운트와 정규화 (실측)**

```
count[38] = 4, 나머지 127개 bin = 0
L1 정규화 → q[38] = 4/4 = 1.0,  나머지 0.0
→ query nonzero bins: [(38, h=4, s=6, 1.0)]      ← 실제 query_hist() 출력
```

**8~9단계 — reference와 비교 (실측, Dinosaur_hsv.npz)**

Dinosaur reference의 뷰0 상위 bin(실측):

```
(h=4, s=3) 0.4123   (h=4, s=2) 0.2116   (h=3, s=3) 0.1803   (h=4, s=4) 0.1483   (h=4, s=5) 0.0179
Dinosaur hue 주변분포(42뷰 평균): bin3 = 0.411, bin4 = 0.589, 나머지 0
```

→ **hue bin은 맞다(4)**. 그러나 reference의 채도는 s bin 2~5에 퍼져 있고 query는 s bin 6 한 곳에만 몰려 있다.

```
BC_i  = Σ sqrt(q[b]·p_i[b]) = sqrt(1.0 × p_i[38])   (q가 한 bin뿐이라 항이 하나)
sim_i = 1 − sqrt(1 − BC_i)

42뷰 실측: min 0.0258, median 0.0554, max 0.1104 (argmax = 뷰 25)
hsv_score = max = 0.11037
```

**10~11단계 — threshold 판정**

```
0.11037 >= 0.12140 ?  →  False
→ res["accepted"] = False
→ res["decision"] = "no-object(below-hsv)"
```

**이 예제가 알려주는 것**: hue가 정확히 맞아도 **채도 분포가 어긋나면 탈락**한다. H×S joint 히스토그램이므로
S축의 불일치가 그대로 벌점이 된다(§17의 "화이트밸런스/노출 민감도" 위험과 직결).

**대조군(실측)**: 실제 Dinosaur 렌더 뷰를 후보로 넣으면 → **0.5909 (PASS)**, milk proto와 비교하면 → **0.0000 (REJECT)**.

---

## 11. Dinosaur hue correction `+9` 완전 분석

### 11.1 설정과 판독 경로

```yaml
# configs/yolo_ism_objects.yaml
defaults:
  hsv_hue_correction_ocv: 0        # 전 객체 기본 = 보정 없음
objects:
  - name: Dinosaur
    hsv_hue_correction_ocv: 9      # 이 클래스만
```

| 질문 | 답 (근거) |
|---|---|
| 어느 함수가 읽나 | ① `tools/build_hsv_template_cache.py:38` `hc = int(o.get("hsv_hue_correction_ocv", 0))` — **적용 주체** ② `yolo_ism_object_n.py:273` `want_hc` — **검증만** |
| 객체 이름 기준인가 | 실질적으로 그렇다. config가 `objects[].name == "Dinosaur"` 항목에만 키를 둔다. **코드에는 `=="Dinosaur"` 문자열 분기가 존재하지 않는다** |
| reference에 적용 | **✓** (`ism_hsv.py:136` `px = _apply_hue_ocv(px, hue_correction_ocv)`) |
| runtime 입력에 적용 | **✗** (`query_hist`에 hue 보정 코드 없음) |
| 히스토그램 **전** HSV H채널에? | **✓ 그렇다** — 픽셀 단위 H 시프트, `feat_rgb`의 calcHist보다 앞 |
| 히스토그램 **후** bin 이동? | **✗ 아니다** |
| cache build 시 적용 | **✓** — `build_reference(tdir, hue_correction_ocv=hc)` |
| cache load 후 runtime 적용 | **✗** — 로드된 proto를 그대로 사용 |
| 객체별 runtime 분기 없음이 맞나 | **맞다.** `_apply_hsv_gate`는 `o["_hsv_proto"]`만 쓰고 이름을 보지 않는다(`:127-150`). 테스트 `test_dinosaur_only_correction`이 config 레벨에서 이를 고정 |
| wrap-around 수행? | **✓** `hsv[..., 0] = (hsv[..., 0] + int(d_ocv)) % 180` (`ism_hsv.py:82`) |
| modulo 180 사용? | **✓** (180이지 179가 아님 — H 값 범위가 [0,179]이므로 정확) |
| unsigned overflow 위험? | **없음.** `.astype(np.int16)`(`:81`)로 승격 후 연산 → uint8 wrap 불가 |
| +9 ocv = +18°? | **✓** OpenCV 180 = 360°, 배율 2. `tests/test_phase1c.py::test_hue_wraparound_degrees_doc`가 `9*2 == 18`을 명시 |

### 11.2 wrap-around 동작 (코드로 검증됨)

```
보정 전 H = 175 (ocv) = 350°
보정값   = +9  (ocv) = +18°
단순 덧셈 = 184        → OpenCV 범위 [0,179] 초과
(184) % 180 = 4        → 최종 H = 4 (ocv) = 8°
```

이것이 **실제로 실행되는지**는 단위 테스트가 증명한다 — `tests/test_phase1c.py:22-33`:

```python
for base in (0, 5, 170, 175, 179):
    rgb = cv2.cvtColor(np.array([[[base,200,200]]], np.uint8), cv2.COLOR_HSV2RGB)...
    h2 = ocv_hue(ism_hsv._apply_hue_ocv(rgb, 9))
    assert 0 <= h2 <= 179
    assert abs(((base + 9) % 180) - h2) <= 2      # 175 → 4 를 명시적으로 검사
```

(±2 허용은 RGB↔HSV 왕복 반올림 오차 때문이며, `_apply_hue_ocv`가 HSV→BGR→RGB로 되돌리는 구조상 불가피하다.)

### 11.3 두 보정의 합성

Dinosaur reference 픽셀에는 보정이 **두 번, 순차로** 걸린다.

```
원본 렌더 H
   ├─ _apply_hue_ocv(+9)   : (H + 9) % 180        ← 클래스 보정 (ism_hsv.py:82)
   └─ feat_rgb(-3)         : (H' − 3) % 180       ← 전역 보정  (ism_hsv.py:43)
= 순 효과 (H + 6) % 180  =  +12° (0-360 기준)
```

다른 9개 객체는 전역 −3만 걸려 순 효과가 **−6°** 이다. 즉 Dinosaur만 **반대 방향**으로 이동한다.

### 11.4 왜 Dinosaur만인가 (Phase 1C 근거)

- **현상**: 실제 초록 공룡 인형이 카메라에서 **H 68~108°** 범위로 넓게 읽히는데, 렌더는 황록 **H≈78°**의
  좁은 분포다. 특히 더 초록/청록으로 보이는 실제 뷰들이 렌더의 hue bin과 어긋나 **진짜 Dinosaur가 색으로
  오탈락**했다(구성 B에서 Dino FN 55).
- **전역 −6°는 반대 방향**: 다른 객체(우유, 갈색박스 등)에는 −6°가 맞지만 Dinosaur에는 **+방향**이 필요했다.
  전역값을 바꾸면 나머지 9객체가 망가진다 → **클래스 단위 예외**가 유일한 해법.
- **검증**: LODO(5데이터로 보정값 선택 → held-out 평가) **6폴드 전부 +9로 수렴**. Dino TP 33→39(+6),
  **새 FP 0**. **+12는 overshoot**(→TP 25). 근거 CSV: `research_a_dinosaur_color/results/dinosaur_template_correction_sweep.csv`.
- **런타임 비용 0**: 캐시에 구워 넣었으므로 추론 시 분기가 없다. `Dinosaur_hsv.npz`의 `hue_correction_ocv = 9`가
  실제로 저장되어 있다(실측 확인).
- **안전장치**: config와 캐시의 보정값이 다르면 `prepare_objects`가 **stale로 간주해 proto를 버린다**
  (`yolo_ism_object_n.py:274-276`) → 잘못된 보정으로 조용히 판정되는 일이 없다(대신 fail-open으로 게이트가
  꺼진 것처럼 동작하므로 로그 확인이 필요).

> 관련 배경: 별도 리서치(`technical-hsv-domain-gap-global-calibration-augmentation-asset-correction-2026-07-22.md`)는
> "색만으로는 Dino 복구와 FP 억제를 동시에 달성 불가"라고 결론지었다. Phase 1C의 +9는 **FP를 늘리지 않는
> 범위에서 얻을 수 있는 부분 회복**(TP +6)이라는 위치에 있다.

---

## 12. Cache 구조와 유효성 검증

### 12.1 생성·저장

```bash
# 생성 명령 (오프라인 전용)
~/miniconda3/envs/sam_yolo/bin/python tools/build_hsv_template_cache.py [--config PATH] [--force]
```

| 항목 | 값 |
|---|---|
| 저장 경로 | `outputs/yolo_ism_object_n/template_features/<name>_hsv.npz` (`yolo_ism_object_n.py:183-184`) |
| 포맷 | **NumPy `.npz`** (`np.savez`, 비압축) |
| 객체별 key | 파일명의 `<name>` = config `objects[].name` |
| 현존 파일 | 10개 = 활성 객체 10개와 1:1 (실측: Bear, Rabbit, Dinosaur, milk, saffron, choco_hazelnut_high, Febreze_high, Mugcup_high, Sauce_high, Sikhye_high) |
| 런타임 자동 생성 | **하지 않는다** — "spec: no cache generation at operation start" (`ism_hsv.py:167`) |
| 캐시 없을 때 | 즉시 실패하지 않고 **fail-open으로 게이트만 무력화** + 경고 출력(`:281-282`) |
| 메모리 로드 | `prepare_objects`에서 **객체당 1회**, 이후 `o["_hsv_proto"]`로 상주. 프레임 I/O **0회** |

### 12.2 실제 schema (`Dinosaur_hsv.npz` 실측)

```
Dinosaur_hsv.npz
  ├─ proto              (42, 128)  float64   ← 뷰별 L1 정규화 H×S 히스토그램 뱅크
  ├─ signature          ()  <U40             ← sha1 hex, 콘텐츠 지문
  ├─ authority_version  ()  <U34   "hsv-authority-1.0-modeB-2026-07-22"
  ├─ hue_shift          ()  int64   -6       ← 전역 hue 보정(0-360 기준), 기록용
  ├─ sat_gain           ()  float64 1.3      ← 전역 채도 게인, 기록용
  ├─ hbins              ()  int64   16
  ├─ sbins              ()  int64   8
  ├─ n_view             ()  int64   42
  ├─ template_dir       ()  <U74   ".../template/Dinosaur/templates"
  └─ hue_correction_ocv ()  int64   9        ← Phase 1C 클래스 보정
```

**저장되지 않는 것**: threshold, 객체 이름(파일명이 대신함), best-view 정보, 원본 파일 경로 목록.

### 12.3 stale 검증 (`_template_signature`, :86-98)

```python
h.update(f"{HSV_AUTHORITY_VERSION}|{HUE_SHIFT_DEG}|{SAT_GAIN}|{HBINS}|{SBINS}|{SUB_N}|hc{hue_correction_ocv}")
for i in range(42):
    for pre in ("rgb", "mask", "xyz"):
        st = os.stat(p);  h.update(f"{pre}{i}:{st.st_size}:{int(st.st_mtime)}")
```

시그니처가 커버하는 것 / 못 하는 것:

| 변경 | 감지? |
|---|---|
| 템플릿 PNG 교체(크기 또는 mtime 변화) | ✓ |
| `ism_hsv` 상수(HUE_SHIFT/SAT_GAIN/bins/SUB_N) 변경 | ✓ |
| authority 버전 변경 | ✓ (별도 필드로도 이중 검사, `:180-182`) |
| `hsv_hue_correction_ocv` 변경 | ✓ (시그니처 + `prepare_objects:274-276` 이중 검사) |
| **threshold 변경** | **✗** (시그니처에 없음 — 다만 threshold는 캐시와 무관하므로 문제 아님) |
| **파일 내용은 바뀌었는데 크기·mtime이 동일** | **✗** (해시가 아니라 stat 기반) |

**설정 변경 후 캐시 미갱신 시**: `load_cache`가 `status="stale"` + `proto=None` → 런타임은
`[hsv] <name> cache stale -> fail-open (score=1.0, no reject). run tools/build_hsv_template_cache.py`
를 출력하고 **그 객체의 게이트만 조용히 무력화**된다. 크래시하지 않는다.

정상 로드 시 출력 예:
```
[hsv] Dinosaur             ACTIVE proto=(42, 128) thr=0.1214 hue_corr=9ocv ver=9d949efbf427
```

---

## 13. 도식 모음

### 도식 B — Runtime gate (실제)

```
Input RGB frame (bgr, rgb)
    ↓
YOLO-World 공유 1-pass  →  프롬프트 그룹 라우팅  →  conf ≥ 0.02  →  top_k=3
    ↓
DINOv2 1-pass (CLS)  →  semantic_score
    ├─ < 0.35 ─────────────────────────────→ "no-object(below-sim)"   [HSV 미계산]
    ↓ ≥ 0.35 (객체당 best 후보 1개만 생존)
MobileSAM segment_boxes → full-frame bool mask
    ↓
masked_appe (DINOv2 block11 patch)
    ├─ < 0.55 ─────────────────────────────→ "no-object(below-appe)"  [HSV 미계산]
    ↓ ≥ 0.55  →  accepted = True, "detected"
┌───────────────── _apply_hsv_gate ─────────────────┐
│  crop = bgr[y1:y2, x1:x2]                         │
│  mcrop = mask[y1:y2, x1:x2]                       │
│  q = query_hist(crop, mcrop)   [H×S 16×8, L1]     │
│  bc_i = Σ√(q·proto_i)  for i in 0..41             │
│  score = max_i (1 − √(1 − bc_i))                  │
│  hp = (score >= 0.1214)                           │
└───────────────────────────────────────────────────┘
    ├─ hp = True  → accepted 유지 → PEM(6D pose) / detection.json / PoseArray
    └─ hp = False → accepted = False, "no-object(below-hsv)"
```

### 도식 C — 색 분포 비교 개념

```
 Reference (Dinosaur, 42뷰 중 1개)          Candidate (카메라 후보)
 H-bin:  0 1 2 3 4 5 ... 15                H-bin:  0 1 2 3 4 5 ... 15
        [. . . ▇ █ . ... .]                       [. . . █ . . ... .]
 S-bin:  0..7  (▇=0.18 █=0.41)              S-bin:  0..7
        └── 128-D L1 분포 ──┘                      └── 128-D L1 분포 ──┘
                    ↘                     ↙
                  BC = Σ √(q[b] · p[b])   (128개 항)
                            ↓
                  sim = 1 − √(1 − BC)          ∈ [0,1], 높을수록 유사
                            ↓
                  score = max over 42 views
                            ↓
              score ≥ 0.1214 ?  ── yes → accept
                                └─ no  → "no-object(below-hsv)"
```

---

## 14. Semantic / Appearance / HSV 검증의 차이

| | Semantic | Appearance | **HSV** |
|---|---|---|---|
| 질문 | 이 영역이 요청 객체 **개념**과 관련 있나 | 이 영역의 **고수준 시각 특징**이 reference와 닮았나 | 이 영역 foreground의 **색 분포**가 reference 색 분포와 충분히 닮았나 |
| 특징 | DINOv2 CLS (384-D) | DINOv2 block11 patch (마스크 필터링) | H×S 히스토그램 (128-D) |
| 학습 | 대규모 self-supervised | 대규모 self-supervised | **학습 없음. 순수 통계** |
| 공간 정보 | 암묵적 보존 | 패치 단위 보존 | **완전 폐기** |
| 조명 강건성 | 높음 | 중간 | **낮음** (V는 뺐지만 S와 hue는 화이트밸런스에 민감) |
| 색 민감도 | **낮음**(shape-biased, 프로젝트 내 다수 실측) | 중간 | **최고** |
| 역할 | 후보 선별 | 후보 선별 | **negative verifier (제거 전용)** |
| 임계 | 0.35 | 0.55 | 0.1214 |

**HSV가 필요한 정확한 이유**: DINOv2 CLS/patch 특징은 문헌상 **shape-biased**이며, 이 프로젝트의 실측에서도
"줄무늬 초코 상자 vs 민무늬 갈색 택배박스"나 "곰 인형 vs 토끼 인형"처럼 **형태가 같고 색만 다른 쌍**을
가르지 못했다. HSV는 그 정확한 실패 축(색)을 직교적으로 보완한다.

---

## 15. 제거 가능한 FP와 발생 가능한 TP 손실

### 15.1 HSV가 잘 제거하는 FP

| 유형 | 근거 |
|---|---|
| **cross-fire 인형** (곰 프롬프트가 흰 토끼를 잡음, 그 반대도) | Phase 1C: HSV가 제거한 248 FP 중 Bear 68 / Rabbit 68 / Dino 39가 rescue 대상에 잡힘 = 이들이 색으로 걸러진 다수 |
| **색이 다른 상자류** (흰 우유팩 프롬프트 ↔ 갈색 택배박스) | milk proto vs 초록 후보 실측 sim = 0.0 처럼 색이 다르면 score가 바닥으로 떨어짐 |
| **밝기만 비슷한 무채색 배경 구조물** | S 채널이 분리 |
| 전체 성과 | **FP 334 → 86 (−74%)**, precision 0.660 → 0.879 |

### 15.2 HSV가 제거하지 **못하는** FP

| 유형 | 이유 |
|---|---|
| **초코 상자 ↔ 갈색 택배박스** | 색이 실제로 같다. 히스토그램으로는 원리적으로 구분 불가 |
| **saffron(크림색 주전자) ↔ 택배 상자 흰 면** | 같은 저채도 밝은 색. 프로젝트 최대 FP원(45%)이 여기 |
| **동일 색의 다른 객체 일반** | HSV는 색만 본다 |
| **texture만 다른 객체** | 히스토그램에 texture 정보가 전혀 없음 |
| 후속 시도 결과 | texture negative verifier AUROC 0.834 → **깨끗한 operating point 없어 미적용**; texture rescue AUROC **0.508(우연)** → 기각. "HSV가 제거한 FP는 애초에 texture가 유사해서 sem+appe를 통과한 것" |

### 15.3 HSV 때문에 제거될 수 있는 TP

| 요인 | 위험 |
|---|---|
| **렌더-실사 domain gap** | 가장 큰 요인. Dinosaur가 대표 사례(보정 전 TP 33/FN 55) |
| **화이트밸런스 변화** | hue 전체가 이동 → reference와 어긋남. −6°/×1.3 전역 보정은 **검증 데이터셋 조건에 종속** |
| **조명 강도 변화** | V는 안 쓰므로 직접 영향은 없으나, **저노출/과노출이 S를 눌러/키워** S bin을 이동시킴 (§10 예제가 정확히 이 실패 모드) |
| **그림자·반사광** | 그림자 영역은 S↑V↓, 하이라이트는 S↓ → 분포 왜곡. 42뷰 max 정책이 일부 흡수 |
| **저채도 객체(흰 토끼, 크림 주전자)** | S가 낮으면 hue가 수치적으로 불안정. 저채도 예외 처리가 **의도적으로 없음**(mode-B 최종 결정) |
| **동일 객체의 다른 면** | 42뷰 커버리지에 의존. 렌더에 없는 면이면 탈락 가능 |
| **마스크 오류** | 마스크가 배경을 크게 포함하면 score 급락(실측 0.59 → 0.41) |
| 실제 규모 | **TP 648 → 622 (26건, 4.0% 손실)**. Recall 0.693 → 0.665 |

---

## 16. 장점과 한계

**장점**
1. **직교성** — DINOv2가 못 보는 축(색)을 본다. 실측으로 FP 74% 제거.
2. **비용 거의 0** — 학습 없음, 신경망 없음, 캐시 로드 1회, 후보당 128차원 히스토그램 1개.
3. **완전 결정론** — 동일 입력에 항상 동일 score. 재현성 검증 완료(연구↔운영 parity 18,060 후보에서 오차 0).
4. **fail-open 설계** — 캐시 사고가 검출을 죽이지 않는다.
5. **shadow 모드 내장** — 판정을 안 바꾸고 측정만 하는 경로가 코드에 상주.
6. **클래스 예외가 런타임에 새지 않음** — 보정을 캐시에 굽는 설계로 추론 코드가 객체 불가지론적.

**한계**
1. **같은 색 객체 구분 불가** — 원리적. 최대 FP원(saffron↔Febreze, choco↔갈색박스)이 여기 남는다.
2. **공간 정보 전무** — 로고·줄무늬·형태 무시.
3. **조명/WB 종속** — 보정값 3개(−6°, ×1.3, Dino +9)와 threshold가 특정 촬영 조건에 캘리브레이션됨.
4. **hue 원형성 미반영** — 비교 단계에서 bin 0과 15가 최대 거리.
5. **best-view 미기록** — 왜 통과/탈락했는지 사후 분석이 어렵다.
6. **max 정책의 관용성** — 42번의 기회가 FP를 통과시키는 방향으로 작동.
7. **config 파라미터 4개가 죽어 있음** — 설정이 실제 동작을 반영하지 않는 문서/코드 불일치(§17).

---

## 17. 구현 품질 및 잠재 위험

| # | 위험 항목 | 분류 | 근거 |
|---|---|---|---|
| 1 | RGB/BGR 채널 순서 불일치 | **현재 코드상 방지됨** | 전 경로가 `BGR2HSV`로 통일, `::-1` 왕복이 정합. `test_rgb_to_hsv_authority_parity`가 연구 구현과 `max|diff|=0.0` 검증 |
| 2 | histogram range 오류 | **현재 코드상 방지됨** | reference/query 모두 `[0,180,0,256]` 동일 리터럴(`:45-46`, `:61`) |
| 3 | **H의 circular 특성 무시 (bin 0 ↔ 15)** | **확인됨 (설계상 한계)** | bin은 선형, Bhattacharyya는 bin 간 전달 없음. H=179와 H=0 픽셀은 겹침 0. 실사용 객체 대부분이 hue 극단(적색)이 아니라 영향은 제한적이나 **적색 객체 추가 시 실제 위험** |
| 4 | 검정/회색 렌더 배경 유입 | **현재 코드상 방지됨** | 반드시 `mask_i.png > 0` 픽셀만 사용(`:127,:135`). 배경이 검정이 아니라 회색([63,69,69] 실측)이므로 마스크가 **필수**인데, 코드가 예외 없이 적용 |
| 5 | 저채도 픽셀의 hue 불안정 | **확인됨 (의도적 미처리)** | 저채도 예외 없음(`ism_hsv.py:23-24`, `hsv_lowsat_exception: false`). S를 joint 축으로 쓰는 것이 부분적 완화책이지만 흰 토끼/크림 주전자에는 취약 |
| 6 | 마스크 경계 background contamination | **잠재 위험** | 마스크 팽창/침식·경계 erosion 없음. MobileSAM 마스크가 헐거우면 그대로 유입. 실측: 마스크 제거 시 0.59→0.41로 민감 |
| 7 | reference/runtime resize·interpolation 차이 | **해당 없음** | 양쪽 다 리사이즈하지 않음(마스크 리사이즈는 `INTER_NEAREST`로 라벨 보존) |
| 8 | histogram 정규화 불일치 | **현재 코드상 방지됨** | 양쪽 L1. `test_hist_l1_normalized`가 두 경로 모두 검사 |
| 9 | 빈 histogram | **확인됨 (정책 비대칭)** | query 빈 히스토그램 → sim 0 → **제거**(fail-closed). proto 빈 뱅크 → **1.0 통과**(fail-open). 두 정책이 반대이며 문서화는 되어 있음 |
| 10 | zero division | **현재 코드상 방지됨** | `max(x.sum(), 1e-12)`(:47), `x/s if s>0 else x`(:62) |
| 11 | NaN | **현재 코드상 방지됨** | `np.maximum(...,0)` 두 곳(:69,:70)이 sqrt 음수 입력 차단 |
| 12 | OpenCV dtype overflow | **현재 코드상 방지됨** | `.astype(np.int16)` 승격 후 hue 가감(`:42`, `:81`), S는 `np.clip(...,0,255)`(:44) |
| 13 | hue correction wrap-around 오류 | **현재 코드상 방지됨** | `% 180` 두 곳(:43, :82) + 단위 테스트가 175→4 경계 검증 |
| 14 | **cache ↔ config mismatch** | **현재 코드상 방지됨 (단 조용함)** | 시그니처 + authority 버전 + `hue_correction` 3중 검사. 다만 불일치 시 **경고 출력 후 fail-open**이라 게이트가 꺼진 줄 모를 수 있음 |
| 15 | **config 키가 코드에서 안 읽힘** | **확인됨 (중요)** | `hsv_hue_shift_deg`/`hsv_sat_gain`/`hsv_hist_bins`/`hsv_lowsat_exception` = `_OVERRIDABLE`에만 존재, 참조 0회(repo 전수 grep). 실제 값은 `ism_hsv.py:30-32` 상수. **config를 고쳐도 동작 불변 → 오해 유발** |
| 16 | object name key mismatch | **잠재 위험** | 캐시 파일명이 `objects[].name`에서 유도된다(`:183-184`). 이름을 바꾸면 새 이름 캐시가 없어 조용히 fail-open. Phase 1C §13도 "class alias" 위험으로 지목 |
| 17 | 여러 reference aggregation 편향 | **확인됨** | max 정책 = 42번의 기회 → FP 통과 방향 편향. threshold가 이 정책 위에서 튜닝됨 |
| 18 | threshold 과적합 가능성 | **확인됨 (완화됨)** | 사람 GT 935 + LOO 6폴드 전부 t*=0.1214 → 데이터셋 간 안정. 그러나 6개 데이터셋 모두 **같은 카메라·같은 실내**이므로 신규 환경 일반화는 미검증 |
| 19 | 동일 색 객체 구분 불가 | **확인됨 (원리적)** | §15.2. texture 대안 2건 모두 실패 |
| 20 | 조명·auto white balance 민감도 | **잠재 위험 (미측정)** | WB 변동 하의 정량 측정 자료를 찾지 못함. §10 예제가 S bin 이동만으로 탈락함을 보임 |
| 21 | **docstring 부정확 (ROS 노드)** | **확인됨** | `ism_hsv.py:6-9`가 "라이브 노드도 동일 score를 계산"이라 하지만 노드 파일에 `hsv` 0회. 실제로는 `o_n.recognize()` 경유로 게이트가 걸리므로 **동작은 정상, 문서만 오해 유발** + 노드가 hsv 필드를 발행/로깅하지 않음 |
| 22 | best-view 미기록으로 디버깅 곤란 | **확인됨** | `.max()`만 반환(`:70`), argmax 미보존 |
| 23 | 시그니처가 stat 기반(해시 아님) | **잠재 위험** | 크기·mtime이 동일한 내용 변경은 감지 못 함. 실무상 발생 확률 낮음 |
| 24 | 마스크 최소 면적 하한 없음(runtime) | **잠재 위험** | reference는 50px 하한이 있으나 query에는 없음. 극소 마스크는 히스토그램이 극도로 sparse해져 §10처럼 낮은 score → TP 손실 방향 |
| 25 | reference만 20000 샘플 / query는 전 픽셀 | **잠재 위험 (경미)** | L1 정규화로 편향은 상쇄. 마스크 픽셀 수가 20000 미만인 뷰는 복원추출로 분산 과소평가 |

> 본 조사에서 개선 코드는 구현하지 않았다.

---

## 18. 1분 발표 스크립트

> "왜 필요하냐면, DINOv2 기반의 의미·외형 검증은 형태 편향이 있어서 **형태가 같고 색만 다른 물체**를 못
> 가릅니다. 갈색 곰 인형 프롬프트가 흰 토끼를 잡고, 초코 상자 프롬프트가 갈색 택배박스를 잡습니다.
>
> 그래서 마지막에 색 검증을 하나 붙였습니다. 어떤 픽셀을 쓰냐면, YOLO 박스로 자른 영역 안에서 **MobileSAM
> 마스크 내부 픽셀만** 씁니다. 배경은 완전히 제외합니다.
>
> 무슨 특징을 만드냐면, 그 픽셀들의 **색상(Hue) 16칸 × 채도(Saturation) 8칸, 총 128칸짜리 2차원
> 히스토그램**을 만들고 합이 1이 되게 정규화합니다. 밝기(V)는 조명에 흔들리니까 뺐습니다.
>
> 어떻게 비교하냐면, 미리 렌더링해 둔 42장 템플릿 각각의 히스토그램과 Bhattacharyya 겹침을 계산해서
> **가장 잘 맞는 한 장의 점수**를 씁니다. 점수는 0에서 1, 높을수록 색이 비슷합니다.
>
> threshold 0.1214로 무엇을 결정하냐면 — 이 값 **이상이면 통과, 미만이면 `below-hsv`로 제거**합니다.
> 사람이 라벨링한 935건 GT에서 **오검출이 334건에서 86건으로 74% 줄었고**, F1이 0.676에서 0.757로 올랐습니다.
>
> 전체 파이프라인에서의 역할은 명확합니다. 이건 **탐지기도 분류기도 아니고**, 이미 통과한 후보 중에서
> **색이 명백히 다른 것만 떨어뜨리는 마지막 거름망**입니다."

---

## 19. 5분 기술 발표 스크립트

**① 왜 HSV인가 — RGB보다 색 비교에 유리한 이유** `[일반 이론]`
RGB는 세 축이 모두 밝기와 얽혀 있습니다. 같은 초록색 물체라도 조명이 어두워지면 (R,G,B)가 전부 같이
줄어들어서 "다른 색"처럼 보입니다. HSV는 이걸 **색상(Hue) / 채도(Saturation) / 명도(Value)** 로 분리합니다.
조명이 변해도 Hue는 상대적으로 유지됩니다. 그래서 저희는 **H와 S만 쓰고 V는 아예 버렸습니다**. 밝기 변화에
대한 1차 방어입니다. 참고로 OpenCV는 360°를 180단계로 압축해서 저장하기 때문에 **코드의 hue 1단위 = 실제 2°**
입니다. 이 단위 구분이 뒤에 나올 "+9"를 이해하는 데 중요합니다.

**② foreground mask가 왜 필요한가** `[본 구현]`
바운딩 박스에는 배경이 잔뜩 섞여 있습니다. 특히 저희 렌더 템플릿은 **배경이 검정이 아니라 회색(63,69,69)**
이라서 마스크 없이 히스토그램을 만들면 배경 색이 그대로 특징이 됩니다. 실측으로 보여드리면, 같은 공룡 후보를
**마스크 있이 계산하면 0.59, 마스크 없이 박스 전체로 계산하면 0.41** 입니다. 31% 차이입니다. 그래서
reference는 렌더러가 준 `mask_i.png`를, runtime은 **MobileSAM이 만든 full-frame 마스크**를 씁니다.

**③ 히스토그램이 무엇을 버리고 무엇을 남기는가** `[일반 이론]`
히스토그램은 **픽셀의 위치를 전부 버리고 색의 분포만 남깁니다.** "초록이 60%, 노랑이 40%"만 남고, 그게
어디에 있었는지는 사라집니다. 장점은 회전·이동·부분 가림에 무관하다는 것이고, 단점은 **로고나 줄무늬 같은
공간 패턴을 전혀 못 본다**는 것입니다. 저희가 이걸 감수한 이유는, 공간 패턴은 이미 앞단의 DINOv2 appearance
검증이 담당하기 때문입니다. HSV는 **직교적인 축 하나**를 맡습니다.

**④ reference 생성 과정** `[본 구현]`
객체마다 CAD를 42개 시점에서 렌더링한 이미지가 있습니다. 각 뷰에서 마스크 내부 픽셀만 뽑고, 뷰 인덱스를
시드로 **결정론적으로 2만 개를 샘플링**합니다. 여기에 두 가지 색 보정을 겁니다 — 전 객체 공통으로
**Hue −3단위(=−6°), 채도 ×1.3**. 이건 "렌더가 실사보다 채도가 낮고 색이 살짝 치우쳐 있다"는 domain gap을
메우는 캘리브레이션입니다. **중요한 건 이 보정을 reference에만 걸고 카메라 쪽에는 안 건다**는 겁니다. 양쪽에
걸면 차이가 상쇄돼서 의미가 없어지니까요. 그다음 16×8 히스토그램을 만들고 L1 정규화합니다. **42개 뷰를
평균 내지 않고 42×128 행렬 그대로 `.npz` 캐시에 저장합니다.**

**⑤ runtime 후보 생성 과정** `[본 구현]`
카메라 프레임에서, 이미 semantic 0.35와 appearance 0.55를 통과한 후보의 박스를 잘라내고, MobileSAM 마스크로
전경만 남기고, **완전히 같은 파라미터로** 128차원 히스토그램을 만듭니다. 차이는 딱 하나 — **색 보정을 안 한다**는 것뿐입니다.

**⑥ histogram matching** `[본 구현]`
`cv2.compareHist`를 쓰지 않고 NumPy로 직접 계산합니다. 두 분포의 **Bhattacharyya 계수** `BC = Σ√(q·p)`를
구하고, `sim = 1 − √(1 − BC)` 로 변환합니다. 완전히 같으면 1, 전혀 안 겹치면 0. 이걸 42개 뷰에 대해 다 하고
**최댓값**을 씁니다. 즉 **한 시점에서만 색이 맞아도 통과**합니다. 이건 의도된 관용입니다 — 물체의 어느 면을
보고 있는지 모르니까요.

**⑦ threshold hard gate** `[본 구현]`
`score >= 0.1214` 면 통과, 미만이면 `no-object(below-hsv)`. 이 숫자는 사람이 직접 라벨링한 935건 GT에서
**F1 최대화**로 뽑았고, 데이터셋 단위 leave-one-out 6폴드에서 **여섯 번 모두 정확히 같은 값**이 나왔습니다.
직관적으로 환산하면 **"128차원 분포 겹침이 22.8% 이상이면 통과"** 라는 아주 관대한 기준입니다. 명백히 다른
색만 떨어뜨리겠다는 설계 의도가 그대로 숫자에 들어 있습니다. 결과는 **FP 334→86, precision 0.66→0.88,
TP는 648→622로 4% 손실**입니다.

**⑧ Dinosaur hue correction** `[본 구현]`
한 클래스만 예외가 있습니다. 실제 초록 공룡 인형이 카메라에서 **H 68~108°** 로 넓게 읽히는데 렌더는 황록
78° 좁은 분포라서, 진짜 공룡이 색 때문에 탈락했습니다. 전역 보정은 −6°인데 공룡한테는 **반대 방향**이
필요했던 거죠. 그래서 Dinosaur reference에만 **+9 OpenCV 단위, 즉 +18°** 를 추가로 겁니다. 검증에서 6폴드
전부 +9로 수렴했고, **TP 33→39, 새 오검출 0**이었습니다. +12는 오버슈트라 오히려 25로 떨어집니다.
구현상 아름다운 점은 — 이 보정을 **캐시 빌드 때 구워 넣기 때문에 추론 코드에는 `if name == "Dinosaur"`
같은 분기가 단 한 줄도 없다**는 것입니다. 런타임은 완전히 객체 불가지론적입니다. wrap-around는 `% 180`으로
처리하고, 175+9=184가 4가 되는 경계를 단위 테스트가 검사합니다.

**⑨ 장점과 한계**
장점은 학습이 필요 없고, 비용이 사실상 0이고, 완전히 결정론적이고, 캐시가 없어도 fail-open이라 시스템을
죽이지 않는다는 것. 한계는 명확합니다 — **같은 색 물체는 원리적으로 못 가릅니다.** 초코 상자와 갈색
택배박스는 색이 진짜로 같아서 이 게이트를 그냥 통과합니다. texture로 풀어보려고 두 번 시도했는데 하나는
AUROC 0.834로 깨끗한 임계가 없었고, 다른 하나는 0.508로 사실상 우연이었습니다. 그리고 threshold와 보정값
세 개가 **저희 촬영 조건에 캘리브레이션되어 있어서**, 조명이나 화이트밸런스가 크게 다른 환경에서는 재보정이
필요합니다. 이게 현재 열려 있는 다음 과제입니다.

---

## 20. 핵심 코드 근거 표

| 설명 항목 | 실제 파일 | 함수/클래스 | 변수/설정 | Line | 확인 내용 |
|---|---|---|---|---:|---|
| HSV gate 진입 | `sam6d_ws/yolo_ism_object_n.py` | `_apply_hsv_gate` | `res["accepted"]`, `res["box"]` | 127-150 | accepted=True·box 존재 시에만 실행. 호출 지점 4곳(396/412/582/596) |
| 게이트 on/off 판정 | `yolo_ism_object_n.py` | `_hsv_on` | `hsv_gate_enabled`, `hsv_gate_shadow_mode` | 122-124 | 둘 중 하나면 "계산", enabled만 "제거" |
| Reference histogram 생성 | `sam6d_ws/ism_hsv.py` | `build_reference` | `template_dir`, 42뷰 루프, `MIN_MASK_PX=50` | 113-142 | rgb/mask/xyz 3파일 존재 + mask≥50px 뷰만 채택 |
| Reference 색보정+히스토그램 | `ism_hsv.py` | `feat_rgb` | `HUE_SHIFT_DEG=-6`, `SAT_GAIN=1.3` | 38-48 | H=(H−3)%180, S=clip(S×1.3), calcHist |
| 결정론적 픽셀 샘플링 | `ism_hsv.py` | `_sub` | `SUB_N=20000`, `seed=i` | 100-110, 135 | RandomState(뷰 인덱스) |
| Candidate 영역 추출 | `ism_hsv.py` | `shadow_score` | `bgr[y1:y2,x1:x2]`, `mask[y1:y2,x1:x2]` | 192-205 | bbox crop ∩ full-frame mask, 리사이즈 없음 |
| Candidate histogram 생성 | `ism_hsv.py` | `query_hist` | `cv2.calcHist([hsv],[0,1],m,[16,8],[0,180,0,256])` | 50-62 | 색보정 없음. mask 전부 0이면 bbox 전체로 fallback |
| Histogram 정규화 | `ism_hsv.py` | `feat_rgb` / `query_hist` | `x/max(x.sum(),1e-12)` / `x/s if s>0` | 47, 62-63 | 양쪽 L1, float64 |
| Histogram 비교 | `ism_hsv.py` | `similarity` | `bc = np.sqrt(np.maximum(q*proto,0)).sum(1)` | 69 | Bhattacharyya 계수, `cv2.compareHist` 미사용 |
| 최종 score 계산 | `ism_hsv.py` | `similarity` | `(1.0 - np.sqrt(np.maximum(0,1-bc))).max()` | 70 | 42뷰 **max**, [0,1] 높을수록 유사 |
| Threshold 판정 | `yolo_ism_object_n.py` | `_apply_hsv_gate` | `hp = bool(score >= thr)` | 139 | **`>=`** — 동값이면 통과 |
| threshold 값 | `configs/yolo_ism_objects.yaml` | `defaults` | `hsv_gate_threshold: 0.12140` | 18 | 객체별 override 없음(전 객체 동일) |
| below-hsv 처리 | `yolo_ism_object_n.py` | `_apply_hsv_gate` | `res["decision"]="no-object(below-hsv)"` | 148-150 | `accepted=False`로 뒤집음 |
| CSV 기록 | `yolo_ism_object_n.py` | `CSV_FIELDS` / writer | `hsv_score/hsv_threshold/hsv_pass/would_hsv_reject/…` | 110-112, 727-730 | 6개 컬럼 append-only |
| Dinosaur hue correction (적용) | `ism_hsv.py` | `_apply_hue_ocv` | `hsv[...,0]=(hsv[...,0]+d_ocv)%180` | 74-84, 136 | reference 픽셀에만, 히스토그램 **이전**, int16 승격 |
| Dinosaur hue correction (설정) | `configs/yolo_ism_objects.yaml` | `objects[Dinosaur]` | `hsv_hue_correction_ocv: 9` | 297 부근 | 나머지 객체 0 |
| Dinosaur hue correction (빌드) | `tools/build_hsv_template_cache.py` | `main` | `hc = int(o.get("hsv_hue_correction_ocv",0))` | 38, 45 | 캐시에 구워 넣음 |
| Cache 검증 | `ism_hsv.py` | `load_cache` / `_template_signature` | `signature`, `authority_version` | 86-98, 159-190 | stat 기반 sha1 + 버전 이중 검사 |
| Cache↔config 보정값 검증 | `yolo_ism_object_n.py` | `prepare_objects` | `want_hc`, `hue_correction-mismatch` | 273-278 | 불일치 시 proto=None (fail-open) |
| Cache 1회 로드 | `yolo_ism_object_n.py` | `prepare_objects` | `o["_hsv_proto"]` | 267-286 | 프레임 루프 내 I/O 0회 |
| MobileSAM mask 생성 | `sam6d_ws/yolo_ism.py` | `segment_box` / `segment_boxes` | full-frame bool, `INTER_NEAREST` | 276-310 | HSV가 소비하는 마스크의 출처 |
| 단위 테스트 (Phase 1C) | `tests/test_phase1c.py` | 12 테스트 | wrap-around/Dino-only/active/fail-open | 전체 | 12/12 PASS 기록 |
| 단위 테스트 (Phase 1B) | `tests/test_hsv_shadow.py` | 13 테스트 | 연구 parity `max|diff|=0.0`, L1 | 22-60 등 | shadow 안전성 |
| **읽히지 않는 config 키** | `yolo_ism_object_n.py` | `_OVERRIDABLE` | `hsv_hue_shift_deg` 외 3개 | 47 | 선언만 있고 참조 0회 (repo 전수 grep) |

---

## 21. 확인되지 않은 사항

코드/산출물에서 **확정하지 못한** 것들이다. 추정하지 않고 그대로 기록한다.

1. **HSV score 자체의 AUROC** — Phase 1C 산출물은 F1/precision/recall/confusion만 보고한다. 보고서에 나오는
   AUROC 0.834·0.508은 **채택되지 않은 texture verifier**의 값이며 HSV의 값이 아니다.
2. **`hsv_gate_threshold` sweep의 전체 범위** — 인용된 지점은 t=0.05/0.10/0.1214 세 개뿐이다. 원본 CSV
   (`outputs/phase1c_hsv_dinosaur_color_correction/csv/phase1c_threshold_sweep_hsv.csv`)의 전 구간은 이번에
   열어보지 않았다.
3. **`HUE_SHIFT_DEG = -6` / `SAT_GAIN = 1.3` 의 산출 근거** — `ism_hsv.py`는 연구 코드
   `ism_fusion_research/ply_hsv/`에서 왔다고만 밝힌다. 이 두 값이 어떤 최적화로 정해졌는지는 이번 조사
   범위에서 확정하지 못했다(§관련 리서치 파일 존재는 확인).
4. **화이트밸런스/조명 변동에 대한 정량 민감도** — 실험 자료를 찾지 못했다. §17 #20을 "잠재 위험(미측정)"으로
   분류한 이유다.
5. **클래스별 HSV score 분포·편차 수치** — Phase 1C는 Dinosaur만 클래스별 수치를 제시한다. 나머지 9객체의
   score 분포/클래스별 TP 손실 내역은 산출물에서 찾지 못했다.
6. **TP casualty 26 vs 32의 차이** — §11(구성 A→C, TP 648→622 = 26)과 §8(rescue 대상 "TP casualty 32")의
   숫자가 다르다. 집계 기준(구성 B 기준 32 vs 구성 C 기준 26)의 차이로 보이나 산출물에 명시가 없어 확정하지
   않는다.
7. **라이브 ROS 노드에서 HSV 필드의 관측 가능성** — 노드가 `o_n.recognize()`를 호출하므로 게이트는 걸리지만,
   `hsv_score` 등이 detection.json/토픽으로 나가는지는 노드 코드에 HSV 참조가 0회여서 **나가지 않는 것으로
   보이며**, 실제 출력물로 확인하지는 않았다.
8. **`xyz_{i}.npy` 존재 검사의 실질적 필요성** — 코드 주석은 "research parity"라고만 한다. xyz 없이 rgb/mask만
   있는 뷰가 실제로 존재하는지는 확인하지 않았다.

---

## 22. 결론 — 필수 12문항 답변

| # | 질문 | 답 |
|---|---|---|
| 1 | Reference HSV 특징은 어떤 이미지·픽셀에서? | `template/<obj>/templates/rgb_{0..41}.png` 42장의, `mask_{i}.png > 0` 인 픽셀만. 뷰당 20000개 결정론적 샘플(seed=뷰 인덱스). 뷰 채택 조건은 rgb/mask/xyz 3파일 존재 + 마스크 ≥ 50px |
| 2 | Candidate HSV 특징은 어떤 영역·mask에서? | 원본 프레임 BGR의 **YOLO bbox crop ∩ MobileSAM full-frame bool mask**. 리사이즈·패딩 없음. 마스크가 없거나 전부 0이면 bbox 전체로 fallback |
| 3 | 실제 사용 channel | **H와 S만. V는 사용하지 않음** |
| 4 | bin과 정규화 | H 16 bin([0,180)), S 8 bin([0,256)) joint 2D → flatten 128-D. **L1 정규화**(확률분포), float64, smoothing 없음 |
| 5 | 여러 template을 어떻게 하나로? | **합치지 않는다.** 42×128 뱅크로 저장하고 **런타임에 score max**로 집계 |
| 6 | 어떤 함수·수식으로 비교? | `cv2.compareHist` 아님. NumPy 직접: `BC=Σ√(q·p)`, `sim=1−√(1−BC)` (`ism_hsv.py:69-70`) |
| 7 | score의 범위·방향 | **[0,1], 높을수록 유사.** 동일=1.0, 겹침 없음=0.0 |
| 8 | 0.1214는 어떤 비교식에? | `hp = bool(score >= 0.1214)` (`yolo_ism_object_n.py:139`). **동값이면 통과.** 미만이면 `accepted=False`, `decision="no-object(below-hsv)"` |
| 9 | Dinosaur +9는 언제·어떻게? | **캐시 빌드 시**, **reference 픽셀에**, **히스토그램 생성 이전**에 `(H+9)%180`. 그 뒤 `feat_rgb`의 전역 −3이 더해져 순 효과 +6 ocv(=+12°). 런타임에 객체별 분기 **없음**. +9 ocv = +18°(0-360) |
| 10 | semantic·appearance와 무엇이 다른가? | 학습된 특징이 아닌 **순수 색 통계**이고, **공간 정보를 완전히 버린다**. semantic/appearance가 shape-biased여서 못 가르는 "형태 같고 색 다른" 축을 직교적으로 담당한다. 후보를 만들거나 라벨을 고르지 않고 **제거만** 한다 |
| 11 | 전체 성능 기여는? | 사람 GT 935 기준 **FP 334→86(−74%), Precision 0.660→0.879, F1 0.6761→0.7572**. 대가는 TP 648→622(−4.0%), Recall 0.693→0.665. Dinosaur는 hue 보정으로 TP 33→39(새 FP 0) |
| 12 | 신뢰할 수 있는 부분과 한계 | **신뢰**: 수식·전처리가 연구 구현과 비트 단위 parity(18,060 후보 오차 0), 결정론적, 25개 단위 테스트가 fail-open/wrap-around/L1/parity를 고정, threshold가 6폴드 LOO에서 완전 안정. **한계**: 같은 색 객체 구분 원리적 불가(초코↔갈색박스, saffron↔흰 면), hue 원형성 비반영, 보정 3값+threshold가 특정 촬영조건 종속, best-view 미기록으로 사후 분석 곤란, **config의 HSV 파라미터 4개가 실제로는 읽히지 않아 설정과 동작이 불일치** |

---

## 23. 다음 액션 제안

이 보고서는 조사 전용이며 어떤 운영 자산도 변경하지 않았다. 아래 중에서 진행 방향을 골라 달라.

**A. 문서/설정 정합성 정리 (저위험, 판정 불변)**
- `_OVERRIDABLE`의 죽은 키 4개(`hsv_hue_shift_deg`/`hsv_sat_gain`/`hsv_hist_bins`/`hsv_lowsat_exception`)를
  제거하거나, 반대로 `ism_hsv` 상수를 config에서 읽도록 배선한다.
- `ism_hsv.py` docstring의 "라이브 ROS 노드도 동일 score를 계산한다"를 실제 구조(노드는 `o_n.recognize()`
  경유, HSV 필드 미발행)에 맞게 수정한다.
- 판정에 영향 0. 오해로 인한 미래 사고를 막는 것이 목적.

**B. 관측 가능성 보강 (중위험, 판정 불변)**
- `similarity`가 `argmax` 뷰 인덱스도 반환하도록 하고 CSV에 `hsv_best_view`를 추가한다.
- 왜 통과/탈락했는지 사후 분석이 가능해진다. 특히 TP casualty 26건의 원인 규명에 직접 쓰인다.

**C. 열린 위험의 정량화 (조사)**
- 화이트밸런스/노출 변동 하의 hsv_score 민감도 측정(§21 #4).
- 클래스별 HSV score 분포와 TP 손실 내역 산출(§21 #5) — Dinosaur 외 9객체는 현재 깜깜이다.

**D. 남은 FP 문제 (연구)**
- 색으로 못 가르는 saffron↔Febreze, choco↔갈색박스. texture 두 시도가 모두 실패했으므로, HSV와 직교하는
  **또 다른 축**(형상/크기/depth 일관성)이 필요한지 재검토.

**질문**: 어느 방향을 먼저 진행할까? A(문서 정합성)는 즉시 처리 가능하고, B(best-view 로깅)는 운영 코드
수정이 필요하므로 별도 승인이 필요하다. C·D는 새 리서치 사이클을 여는 규모다.
