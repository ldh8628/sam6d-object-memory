---
status: ready
created: 2026-06-09
project: sam6d_ws
basis: _bmad-output/implementation-artifacts/spec-sam6d-reliability-fix.md, _bmad-output/planning-artifacts/prds/prd-sam6d_ws-2026-06-08/prd.md
supersedes: "PRD Success Criteria / Acceptance Test 의 sam6d_milk_verify 단일 baseline 기준 (폐기)"
goal: 신규 2종 ros2 bag 기준 검증 — no-object False Positive 제거 + pose stability 정량/시각 검증
---

# Validation Plan: SAM-6D 추론 신뢰성 개선 (신규 데이터셋 기준)

> **기존 검증 데이터(`output/sam6d_milk_verify` 단일 baseline)는 폐기**하고, 본 문서가 Story 검증 기준의 단일 출처(single source of truth)가 된다.
> **구현 코드는 수정하지 않는다.** 본 문서는 검증 절차/판정 기준만 정의한다. 검증은 이미 구현된 디버그 로깅(`_write_debug_log`, FR-9)이 산출하는 프레임별 CSV/JSON을 소비한다.

## 데이터셋 (실제 경로 확인 완료)

| 논리 이름(요청) | **실제 경로** | 내용 | color/depth 프레임 | 검증 목적 |
|---|---|---|---|---|
| `SLAM_with_milk_nomilk` | `data/milk_nomilk_bag/` (`bag_0.db3`) | Milk 등장↔사라짐 반복, 미존재 구간 포함, 미존재 중 손으로 위치 수정 장면 포함 | 2486 / 2339 (82.9 s) | no-object 시 pose 미출력, False Positive 검증, 재등장 시 추정 재개 |
| `only_Milk` | `data/only_milk/` (`bag_0.db3`) | Milk 상시 가시, 카메라만 소폭 이동하며 동일 객체 연속 촬영 | 865 / 837 (28.9 s) | 연속 프레임 pose jitter, Δtranslation/Δrotation 안정성, stabilization 효과 |

- 공통 토픽: RGB `/camera/camera/color/image_raw`, Depth `/camera/camera/aligned_depth_to_color/image_raw` → `no_cli_milk.yaml`과 동일(토픽 변경 불요).
- **경로 주의**: 요청서의 `data/ros2_bag/...`는 현재 존재하지 않는다. 실제 경로는 위 표 기준이며, 본 문서는 실제 경로를 사용한다. (원하면 `data/ros2_bag/SLAM_with_milk_nomilk`, `data/ros2_bag/only_Milk` 심볼릭 링크로 정합 가능)
- **GT 부재 주의**: 두 bag 모두 GT pose 라벨이 없다. milk **가시성**은 사람이 프레임 구간으로 라벨링하며(`manually_labeled_milk_visible`), pose **정확도**는 본 검증의 1차 합격 기준에서 제외한다(가시성 기반 분류 + 변동량만 평가).

## 수정된 Story 요약

대상 객체(Milk)가 **없는 프레임에서는 pose를 출력하지 않고**(no-object/False-Positive 제거), **상시 가시 + 카메라 소폭 이동 시에는 pose 추정이 흔들리지 않도록**(temporal stability) 한 구현을, 위 2종 실주행 ros2 bag으로 정량 지표와 시각화 이미지로 검증한다. 검증은 구현 코드를 변경하지 않고, 노드가 이미 남기는 프레임별 디버그 로그(FR-9)와 수동 가시성 라벨을 결합해 수행한다.

## 수정된 Acceptance Criteria

| ID | 기준 | 데이터셋 | 산출물 |
|---|---|---|---|
| **AC-1** | `SLAM_with_milk_nomilk`에서 **Milk 객체가 없는 프레임 수**를 계산할 수 있어야 한다. | nomilk | `metrics.json.total_no_milk_frames` |
| **AC-2** | Milk 미존재 프레임 중 **pose 미출력 프레임 수(TN)** 와 **잘못 출력한 프레임 수(FP)** 를 계산할 수 있어야 한다. | nomilk | `metrics.json.true_negative_no_pose_frames`, `false_positive_pose_frames` |
| **AC-3** | **False Positive 예시**와 **True Negative 예시**를 이미지로 저장해야 한다. | nomilk | `visualizations/false_positive_examples/*`, `true_negative_examples/*` |
| **AC-4** | `only_Milk`에서 **연속 프레임 pose stability 시각화**를 생성해야 한다. | only_Milk | `visualizations/pose_stability_sequence/grid_*.png` |
| **AC-5** | 연속 프레임별 **translation/rotation 변화량**을 CSV 또는 JSON으로 저장해야 한다. | both | `frame_results.csv` (`delta_translation`, `delta_rotation_deg`) |
| **AC-6** | 최종 검증 보고서에 **정량 지표 + 시각화 이미지 경로**가 포함되어야 한다. | both | 본 문서 §검증 보고서 + `metrics.json` |
| **AC-7** | **Stabilization ON/OFF 정량 비교**: 단일 run에서 raw pose와 stabilized pose를 함께 저장하고, 각각의 **variance(spread)** 와 **reduction rate**(=1−stab/raw)를 산출하며, raw vs stabilized **비교 시각화**를 생성해야 한다. | only_Milk (TP 연속구간) | `metrics.json.stabilization_comparison`, `visualizations/stabilization_compare/*.png` |

## 수정된 Validation Plan

### 검증 축
1. **No-Object 정확도 (문제 A)** — `SLAM_with_milk_nomilk`
   - milk 미존재 프레임에서 pose 미출력 비율(TN rate) ↑, 오출력(FP rate) ↓.
   - milk 재등장 프레임에서 K프레임 이내 추정 재개(recovery) 확인.
2. **Pose Stability (문제 B)** — `only_Milk`
   - 연속 프레임 Δtranslation / Δrotation 분포 측정, stabilize on/off 비교.
3. **시각 증거** — 두 데이터셋 모두 사람이 눈으로 판정 가능한 이미지 산출.

### 분류 정의 (decision_type)
가시성 라벨(`manually_labeled_milk_visible`) × pose 출력 여부(`pose_output_exists`)의 2×2:

| | pose 출력됨 | pose 미출력 |
|---|---|---|
| **milk 보임** | TP | FN |
| **milk 안 보임** | **FP** | **TN** |

- `predicted_milk_visible` := `pose_output_exists`(no-object 판정 결과가 곧 가시성 예측).
- nomilk bag: 4종 모두 발생. only_Milk bag: 상시 가시 → TP/FN만, stability가 주 평가.

### 정량 지표 (필수, `SLAM_with_milk_nomilk`)
```
total_no_milk_frames:           # 가시성 라벨=false 인 프레임 수
true_negative_no_pose_frames:   # 그중 pose_output_exists=false (TN)
false_positive_pose_frames:     # 그중 pose_output_exists=true  (FP)
false_positive_rate:            # FP / total_no_milk_frames
true_negative_rate:             # TN / total_no_milk_frames   ( = 1 - FPR )
```
부가 지표: `total_milk_frames`, `tp_frames`, `fn_frames`, `recall(=TP/(TP+FN))`, `recovery_frames`(재등장→첫 TP 까지 평균 프레임 수).

### 정량 지표 (필수, `only_Milk`)
- **Stabilization 비교 (AC-7, 단일 run, 연속 TP 구간)**:
  - `translation_spread_mm`: raw / stabilized / **reduction_rate** (평균 위치 대비 결합 표준편차)
  - `rotation_spread_deg`: raw / stabilized / **reduction_rate** (평균 회전 대비 각도 RMS)
  - `frame_to_frame_delta_*`: 연속 프레임 변화량 mean/p95/max (raw vs stabilized)
- `pose_score` 분포: mean / min
- **방식**: 노드가 한 번의 ON run에서 raw pose(`t_*_mm`, `q*_raw`)와 stabilized pose
  (`t_*_stab_mm`, `q*_stab`)를 **함께** 디버그 CSV에 기록 → 추가 OFF run 없이 비교 가능.
  (별도 OFF run이 필요하면 `stabilize.enabled:false` config로 1회 더 실행해 교차검증)

## 수정된 Test Procedure

> 전제: 패키지 빌드(config는 share로 설치되므로 config 변경 시 재빌드) → bag 재생 → 노드가 프레임별 디버그 로그 적재 → 수동 가시성 라벨 작성 → 분석 스크립트가 `metrics.json` / `frame_results.csv` / 시각화 산출.

**Step 0 — 환경/빌드**
```bash
cd ~/temp_ws/CLI_environment/sam6d_ws
colcon build --packages-select sam6d_ros
source install/setup.bash
```

**Step 1 — 데이터셋별 추론 실행 (디버그 로그 적재)**
- 데이터셋별 config 2개(`bag.path`/`output_dir`만 교체, 코드 무수정)는 **생성 완료**:
  - `src/sam6d_ros/config/no_cli_milk_nomilk.yaml` → `data/milk_nomilk_bag`, out `outputs/validation/SLAM_with_milk_nomilk/_run`
  - `src/sam6d_ros/config/no_cli_only_milk.yaml` → `data/only_milk`, out `outputs/validation/only_Milk/_run`
- `bag.play:true` 라 launch가 `start_delay`(60s, 모델 로드) 후 자동 `ros2 bag play`. `debug_log.enabled:true` + `benchmark.skip_file_save:false` → 프레임별 `sam6d_debug.csv` + `<frame_id>/detection_pem.json`(R/bbox/mask) 적재.
- **주의**: launch 인자는 `config:=` (`params_file:=` 아님).
```bash
ros2 launch sam6d_ros sam6d_inference.launch.py config:=src/sam6d_ros/config/no_cli_milk_nomilk.yaml
ros2 launch sam6d_ros sam6d_inference.launch.py config:=src/sam6d_ros/config/no_cli_only_milk.yaml
```

**Step 1b — RGB 프레임 추출 (시각화 소스; 노드는 프레임별 RGB 미저장)**
- 노드의 stem 인덱싱(`_input_image_index`, synced 콜백마다 증가)을 동일 sync로 재현해 `<frame_id>.png` 저장:
```bash
# T1
python3 tools/validation/extract_bag_frames.py --out outputs/validation/SLAM_with_milk_nomilk/frames --queue 30 --slop 0.08
# T2 (별도 터미널)
ros2 bag play data/milk_nomilk_bag
```

**Step 2 — 수동 가시성 라벨링 (GT 부재 대응)**
- 스켈레톤 생성 후 milk 미존재 프레임을 0으로 편집:
```bash
python3 tools/validation/make_visibility_template.py \
    --debug-dir outputs/validation/SLAM_with_milk_nomilk/_run/debug \
    --out       outputs/validation/SLAM_with_milk_nomilk/visibility_labels.csv
# → 편집: milk 안 보이는 frame_id 의 manually_labeled_milk_visible 을 0 으로
# only_Milk 는 상시 가시이므로 --labels 생략 가능(전부 visible 간주)
```

**Step 3 — 분석/지표 산출 (`tools/validation/build_validation_report.py`, 구현 코드 무관)**
- 디버그 CSV(raw+stabilized pose) + 가시성 라벨을 조인 → `frame_results.csv` + `metrics.json`.
- **FR-10**: 노드가 raw(`t_*_mm`,`q*_raw`)와 stabilized(`t_*_stab_mm`,`q*_stab`)를 함께 기록하므로,
  단일 ON run에서 `stabilization_comparison`(spread raw/stab + reduction_rate)을 바로 산출한다.
  추가 OFF run 없이 안정화 효과를 정량화한다(원하면 `stabilize.enabled:false` config로 교차검증).
- **numpy 의존성**: 현재 conda env(`sam6d_ros_humble`)에 numpy 1.26.4 설치됨 → rotation spread 계산.
  numpy 없으면 **rotation_spread_deg 만 생략**되고 translation stability·frame count·FP/TN·
  frame_results.csv·시각화는 정상 동작(`metrics.json.stabilization_comparison.numpy_available` 플래그로 표시).

**Step 4 — 시각화 생성 (`tools/validation/make_visualizations.py`)**
- `frame_results.csv` + 추출 RGB + `detection_pem.json`(mask/bbox/R/t) + `camera_json` → TN/FP/stability 3종 자동 렌더.

**Step 5 — 보고서 작성 (AC-6)**
- `metrics.json` 정량값 + 시각화 경로를 본 문서 말미 §검증 보고서 표에 채운다.

## 생성해야 할 시각화 목록

| # | 이름 | 데이터셋 | 선정 조건 | overlay/표기 | 저장 위치 |
|---|---|---|---|---|---|
| 1 | **True Negative 예시** | nomilk | `decision_type=TN` 프레임 샘플(≥3장) | 라벨 텍스트 **"No Milk Detected / No Pose Output"** | `SLAM_with_milk_nomilk/visualizations/true_negative_examples/` |
| 2 | **False Positive 예시** | nomilk | `decision_type=FP` 프레임 전부 또는 상위 점수 N장 | bbox + mask + pose axis(또는 t/quat 텍스트) + **"False Positive: Milk Pose Estimated"** | `SLAM_with_milk_nomilk/visualizations/false_positive_examples/` |
| 3 | **Pose Stability 연속 프레임 grid** | only_Milk | 연속 프레임 구간(예: 8~16장) | 프레임별 `frame_id`, `translation`, `Δtranslation/Δrotation`, `pose_score`, 가능 시 pose axis overlay → 단일 grid 이미지 | `only_Milk/visualizations/pose_stability_sequence/grid_*.png` |
| 4 | **Stabilization 비교 플롯 (AC-7)** | only_Milk | 연속 TP 구간 | t_X/t_Y/t_Z + Δtrans 시계열에 **raw(빨강) vs stabilized(파랑)** 중첩, 제목에 spread·reduction_rate | `only_Milk/visualizations/stabilization_compare/stabilization_compare.png` |

## 저장해야 할 Metrics

`outputs/validation/SLAM_with_milk_nomilk/metrics.json`
```json
{
  "bag_name": "SLAM_with_milk_nomilk",
  "total_frames": 0,
  "total_no_milk_frames": 0,
  "true_negative_no_pose_frames": 0,
  "false_positive_pose_frames": 0,
  "false_positive_rate": 0.0,
  "true_negative_rate": 0.0,
  "total_milk_frames": 0,
  "tp_frames": 0,
  "fn_frames": 0,
  "recall": 0.0,
  "recovery_frames_mean": 0.0,
  "config": {"pose_score_min": 0.3, "ism_score_min": 0.3, "stabilize_enabled": true, "deterministic_seed": 1}
}
```

`outputs/validation/only_Milk/metrics.json` (핵심: `stabilization_comparison`, AC-7)
```json
{
  "bag_name": "only_Milk",
  "total_frames": 0,
  "tp_frames": 0, "fn_frames": 0, "recall": 0.0,
  "stabilization_comparison": {
    "n_published_frames": 0,
    "translation_spread_mm": {"raw": 0.0, "stabilized": 0.0, "reduction_rate": 0.0},
    "rotation_spread_deg":   {"raw": 0.0, "stabilized": 0.0, "reduction_rate": 0.0},
    "frame_to_frame_delta_translation_mm": {
      "raw": {"mean": 0.0, "p95": 0.0, "max": 0.0},
      "stabilized": {"mean": 0.0, "p95": 0.0, "max": 0.0},
      "reduction_rate_mean": 0.0},
    "frame_to_frame_delta_rotation_deg": {
      "raw": {"mean": 0.0, "p95": 0.0, "max": 0.0},
      "stabilized": {"mean": 0.0, "p95": 0.0, "max": 0.0},
      "reduction_rate_mean": 0.0}
  }
}
```
> `reduction_rate = 1 − stabilized/raw`. `spread`=평균 대비 표준편차(positional/rotational jitter),
> `frame_to_frame_delta`=연속 프레임 변화량. 모두 **연속 TP 구간**에서 산출(FP/위치점프 제외).

## 저장해야 할 CSV/JSON Schema

`outputs/validation/<bag>/frame_results.csv` — 프레임당 1행:

| 컬럼 | 타입 | 설명 |
|---|---|---|
| `bag_name` | str | `SLAM_with_milk_nomilk` / `only_Milk` |
| `frame_id` | int | 노드 frame_counter |
| `timestamp` | float | 메시지 stamp (sec) |
| `manually_labeled_milk_visible` | bool | 수동 가시성 라벨 (GT 대체) |
| `predicted_milk_visible` | bool | = `pose_output_exists` |
| `pose_output_exists` | bool | 노드가 pose 발행했는지 |
| `bbox_score` | float | ISM geometric 대표(없으면 null) |
| `mask_score` | float | ISM appearance 대표(없으면 null) |
| `similarity_score` | float | ISM semantic |
| `pose_score` | float | PEM pred_pose_score |
| `final_decision` | str | `PUBLISH` / `NO_OBJECT` / `NO_DETECTION` |
| `decision_type` | str | `TP` / `TN` / `FP` / `FN` |
| `translation_raw_xyz` | float[3] | **raw**(안정화 전) t(mm) |
| `translation_stab_xyz` | float[3] | **stabilized**(발행) t(mm) |
| `rotation_raw_quat` | float[4] | raw quaternion |
| `rotation_stab_quat` | float[4] | stabilized quaternion |
| `delta_translation_raw` / `_stab` | float | 직전 프레임 대비 \|Δt\| (mm), raw/stab 각각 |
| `delta_rotation_raw_deg` / `_stab_deg` | float | 직전 대비 geodesic angle (deg), raw/stab 각각 |
| `visualization_image_path` | str | 해당 프레임 시각화 경로(있으면) |

> **FR-10 반영**: 노드 `_write_debug_log`가 이제 raw(`t_*_mm`, `q*_raw`)와 stabilized(`t_*_stab_mm`, `q*_stab`)를 **함께** 기록한다. 분석 스크립트가 둘을 비교해 variance·reduction rate를 산출한다. `decision_type`/`manually_labeled_milk_visible`는 분석 단계에서 라벨 조인으로 보강.

## 결과 저장 디렉터리 구조

```
outputs/validation/
  SLAM_with_milk_nomilk/
    metrics.json
    frame_results.csv
    visibility_labels.csv
    visualizations/
      true_negative_examples/
      false_positive_examples/
  only_Milk/
    metrics.json
    frame_results.csv
    visibility_labels.csv
    visualizations/
      pose_stability_sequence/
      stabilization_compare/        # AC-7: raw vs stabilized 비교 플롯
```

## 실행 명령어 (전체 — `SLAM_with_milk_nomilk` 예시)

```bash
cd ~/temp_ws/CLI_environment/sam6d_ws
# 0) 빌드 (config 변경 반영)
colcon build --packages-select sam6d_ros && source install/setup.bash

# 1) 추론 (config:= 주의; bag 자동 재생·디버그 로그 적재)
ros2 launch sam6d_ros sam6d_inference.launch.py config:=src/sam6d_ros/config/no_cli_milk_nomilk.yaml

# 1b) RGB 프레임 추출 (T1 스크립트 + T2 bag play)
python3 tools/validation/extract_bag_frames.py --out outputs/validation/SLAM_with_milk_nomilk/frames &
ros2 bag play data/milk_nomilk_bag    # 끝나면 위 추출기 Ctrl-C

# 2) 가시성 라벨 스켈레톤 생성 후 편집(milk 없는 frame_id → 0)
python3 tools/validation/make_visibility_template.py \
    --debug-dir outputs/validation/SLAM_with_milk_nomilk/_run/debug \
    --out       outputs/validation/SLAM_with_milk_nomilk/visibility_labels.csv

# 3) 지표/frame_results 산출
python3 tools/validation/build_validation_report.py --bag SLAM_with_milk_nomilk \
    --debug-dir outputs/validation/SLAM_with_milk_nomilk/_run/debug \
    --pem-root  outputs/validation/SLAM_with_milk_nomilk/_run \
    --labels    outputs/validation/SLAM_with_milk_nomilk/visibility_labels.csv \
    --out       outputs/validation/SLAM_with_milk_nomilk

# 4) 시각화 3종
python3 tools/validation/make_visualizations.py \
    --frame-results outputs/validation/SLAM_with_milk_nomilk/frame_results.csv \
    --frames-dir    outputs/validation/SLAM_with_milk_nomilk/frames \
    --pem-root      outputs/validation/SLAM_with_milk_nomilk/_run \
    --camera-json   src/sam6d_ros/config/d455f_camera.json \
    --out           outputs/validation/SLAM_with_milk_nomilk/visualizations

# only_Milk 는 stabilize 검증용 — config 만 no_cli_only_milk.yaml 로 바꿔 동일 반복
#   (라벨 생략 가능: build_validation_report 에서 --labels 빼면 전부 visible 간주)
#   stability grid 는 --mode stability 로 단독 생성 가능
```

> `tools/validation/README.md` 에 동일 절차와 옵션 설명을 정리해둔다.

## 성공/실패 판정 기준

**문제 A — `SLAM_with_milk_nomilk`**
- **PASS**: `false_positive_pose_frames == 0` (하드 타깃). 즉 `true_negative_rate == 1.0`.
- **조건부 PASS**: `false_positive_rate ≤ 0.05` 이고 baseline(개선 전) 대비 FP ≥ 90% 감소 — floor 재튜닝 항목으로 기록.
- **재개(recovery) PASS**: milk 재등장 후 평균 `recovery_frames ≤ 5`.
- **FAIL**: 위 미달 또는 recall(`TP/(TP+FN)`) 이 baseline 대비 2%p 초과 하락.

**문제 B — `only_Milk`**
- **PASS (AC-7)**: `stabilization_comparison` 의 `translation_spread_mm.reduction_rate > 0` 및
  `rotation_spread_deg.reduction_rate > 0` (stabilized < raw). 목표 권장치: translation/rotation
  **reduction_rate ≥ 0.3** (정지·소폭이동 구간 "충분히 안정", PRD Resolved Decision #4).
- **PASS(결정성)**: `deterministic.enabled=true` + 동일 입력 반복 시 출력 분산↓.
- **FAIL**: reduction_rate ≤ 0 (stabilize가 변동을 줄이지 못함), 또는 추정 단절(연속 FN) 발생.

**전체 합격**: AC-1~6 산출물 모두 존재 + 문제 A PASS + 문제 B PASS.

## 검증 보고서 (실행 후 작성 — AC-6)

| 항목 | 값 | 근거 경로 |
|---|---|---|
| total_no_milk_frames | _TBD_ | metrics.json |
| true_negative_no_pose_frames | _TBD_ | metrics.json |
| false_positive_pose_frames | _TBD_ | metrics.json |
| false_positive_rate | _TBD_ | metrics.json |
| true_negative_rate | _TBD_ | metrics.json |
| trans spread raw→stab (mm) / reduction | _TBD_ | stabilization_comparison |
| rot spread raw→stab (deg) / reduction | _TBD_ | stabilization_comparison |
| TN 예시 이미지 | _TBD_ | true_negative_examples/ |
| FP 예시 이미지 | _TBD_ | false_positive_examples/ |
| Stability grid | _TBD_ | pose_stability_sequence/ |
| Stabilization 비교 플롯 (AC-7) | _TBD_ | stabilization_compare/ |
| 판정 | _PASS/FAIL_ | — |
