---
status: done
created: 2026-06-08
project: sam6d_ws
basis: docs/sam6d-pose-rca-analysis-2026-06-08.md, _bmad-output/planning-artifacts/prds/prd-sam6d_ws-2026-06-08/prd.md
goal: SAM-6D 추론 신뢰성 — no-object 거부 + pose 안정화 + 결정성 + 디버그
---

# Spec: SAM-6D 추론 신뢰성 개선 (최소 패치)

## Phase 1 — 구현 전 분석 요약 (RCA 근거)

- **문제 A (객체 없음 false positive)**: 최종 confidence floor 없음. ISM `final_score`(`run_batch_inference_fast.py:416`)와 PEM `pred_pose_score`(`model_utils.py:281-283`)가 계산만 되고, `run_pem_single`이 `pem_poses`에 무조건 기록(`:707-717`) → 노드가 무조건 발행(`sam6d_inference_node.py:842-849, 898`).
- **문제 B (jitter)**: 라이브 경로 시드 부재 + 프레임마다 실행되는 무시드 랜덤 2개(`np.random.choice` `run_batch_inference_fast.py:587-594`, `torch.rand` `model_utils.py:221`) + temporal filter 부재.
- **핵심 통찰**: 점수가 노드 `pem_poses`까지 전달되고(`:829`), 백엔드 랜덤은 전역 numpy/torch RNG 사용 → **노드에서 프레임 단위 재시드 + 발행 직전 필터/안정화**로 백엔드 미수정 최소 패치 가능.

## Phase 2 — 수정 계획

### 수정 파일 (2 + 1)
1. `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py` — 핵심 로직 (additive)
2. `src/sam6d_ros/config/params.yaml` — 신규 파라미터 운영값
3. `src/sam6d_ros/config/no_cli_milk.yaml` — 동일 파라미터 정합 (있는 경우)
- **`run_batch_inference_fast.py` 미수정** (전역 RNG 재시드로 결정성 확보 → 백엔드 blast radius 0)

### 수정 함수 / 신규 메서드 (node)
| 대상 | 종류 | 역할 | FR |
|---|---|---|---|
| `_declare_parameters` / `_load_parameters` | 수정(additive) | 신규 파라미터 선언·로드 | 전체 |
| `_apply_determinism()` | 신규 | 프레임 단위 seed(random/np/torch) + cudnn | FR-8/5 |
| `_filter_and_decide(pem_poses)` | 신규 | confidence floor 적용 + no-object 판정 | FR-1~4 |
| `_stabilize_poses(poses)` | 신규 | EMA + jump/outlier rejection | FR-5~7 |
| `_write_debug_log(...)` | 신규 | CSV append + per-frame JSON | FR-9 |
| `_load_models` | 수정 1줄 | cudnn deterministic 1회 설정 | FR-8 |
| `_run_inference` | 수정 | 위 메서드 호출 배선 | 전체 |
| `_run_inference_from_images` | 무수정 | (빈 poses → 빈 PoseArray로 "no object" 신호 유지) | FR-1 |

### 신규 파라미터 (코드 기본값 = 기존 동작 유지 / params.yaml = 운영값) — NFR-3
| 파라미터 | 코드 기본 | params.yaml | 의미 |
|---|---|---|---|
| `confidence.pose_score_min` | 0.0(off) | 0.3 | PEM pose_score floor (FR-2/3) |
| `confidence.ism_score_min` | 0.0(off) | 0.3 | ISM final_score floor (FR-2) |
| `stabilize.enabled` | false | true | 안정화 on/off (FR-5) |
| `stabilize.ema_alpha` | 0.0 | 0.6 | EMA 계수(0=즉시반영) (FR-6) |
| `stabilize.jump_trans_mm` | 0.0(off) | 50.0 | 위치 점프 임계 (FR-7) |
| `stabilize.jump_rot_deg` | 0.0(off) | 30.0 | 회전 점프 임계 (FR-7) |
| `deterministic.enabled` | false | true | 시드/결정성 (FR-8) |
| `deterministic.seed` | 1 | 1 | 시드값 |
| `debug_log.enabled` | false | true | 디버그 저장 (FR-9) |
| `debug_log.dir` | "" | output_dir/debug | 저장 경로 |

### 예상 영향도
- **정상 동작**: 모든 신규 기능은 파라미터 게이트. 코드 기본값(off/0.0)이면 **기존과 100% 동일** → 회귀 위험 낮음 (NFR-3).
- **결정성 재시드**: 프레임 단위 고정 시드 → 정지 입력 동일 출력. 단, FPS/CUDA 비결정 잔존 가능 → "충분히 안정" 기준(PRD AT-5)으로 검증.
- **성능**: 필터/EMA/로깅은 µs~ms 수준. cudnn.deterministic만 처리량 영향 가능 → 측정(NFR-2).
- **No-object**: 빈 poses → 빈 PoseArray 발행(기존 graceful 경로), TF 미발행. 다운스트림은 "0개 검출"로 해석.

### Debug 저장 항목 (요청 매핑)
frame_id, ism_score(=bbox/similarity 대표), pem_score(=pose_score), final_score, selected_candidate(best_idx), final_decision(PUBLISH/NO_OBJECT/FILTERED), n_raw, n_pass, t_mm, R → CSV(append) + per-frame JSON.
- 주의: semantic/appearance/geometric 분해 점수는 노드에 전달되지 않음(ISM 내부). 분해 로깅이 필요하면 `run_ism_single` 반환 확장(별도 옵션, Should).

## Acceptance (Phase 4 검증)
- `python3 -m py_compile` 두 파일.
- 코드 기본값으로 기존 동작 불변(파라미터 미설정 시).
- 실행 테스트 명령어 + 예상 결과 제공.

## 데이터셋 기반 검증 (2026-06-09 갱신)
- **기존 `output/sam6d_milk_verify` 단일 baseline 검증은 폐기.** 실주행 ros2 bag 2종으로 교체:
  - `SLAM_with_milk_nomilk` (실제 `data/milk_nomilk_bag/`) — 문제 A(False Positive/no-object).
  - `only_Milk` (실제 `data/only_milk/`) — 문제 B(pose stability).
- 검증 절차/AC/Test Procedure/시각화/Metrics/CSV Schema/판정 기준의 단일 출처: **`_bmad-output/implementation-artifacts/validation-sam6d-reliability-fix.md`**.

## FR-10 — Stabilization ON/OFF 정량 비교 (Change Request 2026-06-09, 구현 완료)
- **코드 변경** (`sam6d_inference_node.py`, additive):
  - `_run_inference`: 안정화 직전 `poses_raw = list(poses)` 캡쳐 → `raw_by_obj`/`stab_by_obj`(object_id 매핑) 구성 후 `_write_debug_log`에 전달.
  - `_mat_to_quat` 정적 메서드 신규(R→quat, scipy와 일치 검증).
  - `_write_debug_log`: CSV에 `t_*_stab_mm`, `q*_raw`, `q*_stab` 컬럼 추가(기존 `t_*_mm`=raw 유지) + per-frame JSON에 `poses`(raw/stab) 추가. → 기존 컬럼 보존(additive), 호환.
- **검증 도구** (`tools/validation/`):
  - `build_validation_report.py`: 연속 TP 구간에서 `stabilization_comparison`(translation/rotation spread raw·stab·reduction_rate, frame-to-frame delta) 산출.
  - `make_visualizations.py`: `--mode stabcmp/all` → raw vs stabilized 비교 플롯.
- **검증**: py_compile OK, `_mat_to_quat` scipy 일치, 합성 데이터로 reduction_rate(translation 0.47 / rotation 0.64)·플롯 생성 확인.
