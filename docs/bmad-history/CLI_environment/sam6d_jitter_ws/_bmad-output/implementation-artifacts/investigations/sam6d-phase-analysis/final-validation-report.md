# SAM6D Milk Reliability Final Validation Report

Date: 2026-06-09

## 1. Baseline 결과

평가 입력:
- Output directory: `output/sam6d_milk_verify`
- 평가 frame 수: 28
- Positive/static label 수: 28
- Milk 미존재 label 수: 0
- Dynamic scene label 수: 0
- Ground-truth pose 수: 0

Detection baseline:
- TP: 26
- FP: 0
- TN: 0
- FN: 2
- Precision: 1.0000
- Recall: 0.9286
- False Positive Rate: 산출 불가, negative frame 0개
- False Negative Rate: 0.0714

Pose baseline:
- ADD: 산출 불가, GT pose 없음
- ADD-S: 산출 불가, GT pose 없음
- Rotation Error: 산출 불가, GT pose 없음
- Translation Error: 산출 불가, GT pose 없음

Temporal Stability baseline:
- Pose count: 25
- Position Std: `[0.7594, 4.4713, 1.0406]` mm
- Rotation Std: `50.6585` deg
- Frame-to-Frame Translation Delta Mean: `5.2613` mm
- Frame-to-Frame Translation Delta Max: `16.1796` mm
- Frame-to-Frame Rotation Delta Mean: `30.9589` deg
- Frame-to-Frame Rotation Delta Max: `179.3731` deg
- Jitter Score: `35.6121`

문제 1 baseline, 현재 사용 가능한 출력 기준:
- ISM candidate count: 130
- Filter pass count: 해당 없음
- Final pose output count: 25
- False Positive 감소율: 산출 불가, Milk 미존재 Bag/label 없음

## 2. 개선 후 결과

구현된 기능:
- Feature A Object Existence Verification: ISM candidate 없음 또는 best ISM score 미달 시 PEM 생략
- Feature B Candidate Confidence Filtering: final score, area, depth valid ratio 기반 ISM 후보 필터
- Feature C Geometric Verification: projected bbox IoU 및 depth residual 검증 코드 구현, 현재 config에서는 disabled
- Feature D Pose Confidence Scoring: PEM final score 미달 pose publish 억제
- Feature E Pose Temporal Stabilization: translation/rotation smoothing 및 jump hold

Active config:
- `candidate_filter.enabled=true`
- `candidate_filter.min_final_score=0.55`
- `candidate_filter.min_mask_area=9000`
- `candidate_filter.min_bbox_area=9000`
- `candidate_filter.min_depth_valid_ratio=0.80`
- `object_existence.min_score=0.55`
- `pose_confidence.min_score=0.55`
- `temporal_filter.enabled=true`
- `temporal_filter.translation_alpha=0.35`
- `temporal_filter.rotation_alpha=0.35`
- `temporal_filter.max_translation_jump_mm=25.0`
- `temporal_filter.max_rotation_jump_deg=120.0`
- `temporal_filter.hold_last_good_frames=2`

Detection improved, existing saved output에 filter rule을 적용한 offline 평가:
- TP: 26
- FP: 0
- TN: 0
- FN: 2
- Precision: 1.0000
- Recall: 0.9286
- False Positive Rate: 산출 불가, negative frame 0개
- False Negative Rate: 0.0714

Temporal Stability improved after filtering only:
- Pose count: 25
- Position Std: `[0.7557, 4.1757, 1.0295]` mm
- Rotation Std: `58.0866` deg
- Frame-to-Frame Translation Delta Mean: `4.9513` mm
- Frame-to-Frame Translation Delta Max: `14.5282` mm
- Frame-to-Frame Rotation Delta Mean: `38.2838` deg
- Frame-to-Frame Rotation Delta Max: `179.9901` deg
- Jitter Score: `42.6504`

Temporal Stability improved after filtering and smoothing:
- Pose count: 25
- Position Std: `[0.2822, 1.9877, 0.4825]` mm
- Rotation Std: `42.1782` deg
- Frame-to-Frame Translation Delta Mean: `1.3860` mm
- Frame-to-Frame Translation Delta Max: `3.9715` mm
- Frame-to-Frame Rotation Delta Mean: `16.2738` deg
- Frame-to-Frame Rotation Delta Max: `56.4161` deg
- Jitter Score: `18.3386`

## 3. 정량 비교

Detection:
- Precision: `1.0000 -> 1.0000`, 변화 `0.0000`
- Recall: `0.9286 -> 0.9286`, 변화 `0.0000`
- FPR: 산출 불가, negative frame 0개
- FNR: `0.0714 -> 0.0714`, 변화 `0.0000`

Temporal Stability, filtering + smoothing 기준:
- Position Std X: `0.7594 -> 0.2822` mm, 감소율 `62.84%`
- Position Std Y: `4.4713 -> 1.9877` mm, 감소율 `55.54%`
- Position Std Z: `1.0406 -> 0.4825` mm, 감소율 `53.63%`
- Rotation Std: `50.6585 -> 42.1782` deg, 감소율 `16.74%`
- F2F Translation Delta Mean: `5.2613 -> 1.3860` mm, 감소율 `73.66%`
- F2F Translation Delta Max: `16.1796 -> 3.9715` mm, 감소율 `75.45%`
- F2F Rotation Delta Mean: `30.9589 -> 16.2738` deg, 감소율 `47.43%`
- F2F Rotation Delta Max: `179.3731 -> 56.4161` deg, 감소율 `68.55%`
- Jitter Score: `35.6121 -> 18.3386`, 감소율 `48.50%`

문제 1, Milk 미존재 장면:
- ISM 후보 발생 수: 산출 불가, Milk 미존재 Bag 없음
- 필터 통과 수: 산출 불가, Milk 미존재 Bag 없음
- 최종 Pose 출력 수: 산출 불가, Milk 미존재 Bag 없음
- False Positive 감소율: 산출 불가, Milk 미존재 Bag/label 없음

## 4. Failure Case 분석

검증 데이터 결측:
- Milk 미존재 Bag이 없어 Feature A/B/D의 false positive 억제 성능을 FPR로 검증하지 못했다.
- GT pose가 없어 ADD, ADD-S, Rotation Error, Translation Error를 산출하지 못했다.
- Dynamic scene Bag이 없어 temporal filter가 동적 추적 지연 또는 lag를 만드는지 검증하지 못했다.

정적 장면 jitter:
- Baseline frame-to-frame rotation max가 `179.3731` deg이다.
- Filtering only 이후에도 frame-to-frame rotation max가 `179.9901` deg로 유지된다.
- Temporal smoothing 후 max가 `56.4161` deg로 낮아졌지만, 원 pose hypothesis의 180 deg급 ambiguity 자체를 제거한 것은 아니다.

## 5. Remaining Issues

- Success Criteria `Milk 미존재 이미지 False Positive Detection Rate 5% 이하`는 현재 데이터로 판정 불가이다.
- Success Criteria `Pose 출력 억제 성공률 95% 이상`은 현재 데이터로 판정 불가이다.
- ADD/ADD-S 기반 pose 정확도 회귀 검증은 GT pose 부재로 판정 불가이다.
- Feature C Geometric Verification은 runtime code가 구현되었지만 현재 config에서 disabled이므로 production threshold 검증 전이다.
- `_run_ism_single_area_rerank()` 경로는 legacy optional path이며 기본 config에서 disabled이다. 기본 fast path에는 candidate filtering이 적용되어 있다.

## 6. 추가 개선안

수치 검증을 완료하기 위한 최소 데이터:
- Milk 존재 Bag: 현재 `output/sam6d_milk_verify`로 28 positive/static frame 평가 완료
- Milk 미존재 Bag: 최소 100 frame 이상, frame-level `milk_present=false` label 필요
- 정적 장면 Bag: 현재 25 pose output 기준 temporal metric 산출 완료
- 동적 장면 Bag: temporal lag 측정용 frame label 또는 GT trajectory 필요
- GT pose: ADD/ADD-S, Rotation Error, Translation Error 산출용 per-frame pose 필요

다음 구현 검증 순서:
- Milk 미존재 Bag으로 Feature A/B/D threshold sweep 수행
- GT pose가 있는 Milk 존재 Bag으로 Feature C threshold sweep 수행
- Dynamic Bag으로 temporal alpha/jump threshold sweep 수행
- `detection_ism_debug.json`의 reject reason 분포를 이용해 false positive 제거 사유를 frame별 집계

## 7. Production 적용 가능성

현재 production 적용 가능 항목:
- Candidate Confidence Filtering: 구현 완료, active config 적용됨
- Object Existence Verification: 구현 완료, active config 적용됨
- Pose Confidence Scoring: 구현 완료, active config 적용됨
- Pose Temporal Stabilization: 구현 완료, active config 적용됨
- Evaluation Pipeline: 구현 완료, 결측 데이터는 null과 missing reason으로 보고

현재 production 적용 보류 항목:
- Geometric Verification: 구현 완료, config disabled. GT/negative/dynamic dataset에서 threshold 검증 후 enable 필요
- FPR 기준 release gate: Milk 미존재 Bag 부재로 미판정
- ADD/ADD-S 기준 release gate: GT pose 부재로 미판정

검증 명령:
```bash
python3 -m py_compile sam6d_master/SAM-6D/run_batch_inference_fast.py src/sam6d_ros/sam6d_ros/sam6d_inference_node.py tools/evaluate_pose_reliability.py
python3 tools/evaluate_pose_reliability.py --output-dir output/sam6d_milk_verify --labels-csv _bmad-output/implementation-artifacts/investigations/sam6d-phase-analysis/available_positive_static_labels.csv --report-path _bmad-output/implementation-artifacts/investigations/sam6d-phase-analysis/post_implementation_positive_static_metrics.json
```

Source metric JSON:
- `_bmad-output/implementation-artifacts/investigations/sam6d-phase-analysis/post_implementation_positive_static_metrics.json`
