---
status: done
slug: sam6d-e2e-multiobject-eval
date: 2026-06-16
owner: Ldh9501
---

# SPEC: SAM-6D 9-bag × 16-object End-to-End 추론 + GT-less 품질 평가

## 목표
`data/ros2_bag`의 ROS2 bag 9개와 `template/`의 객체 템플릿 16개를 입력으로,
프레임 추출(stride=10) → 객체별 SAM-6D 포즈 추정 → Overlay/CSV 저장 →
GT 없는 자동 품질평가 → 최종 리포트까지 End-to-End 자동화한다.

## 확정된 설계 결정 (사용자 승인)
1. **실행 전략 = 파일럿 우선.** 먼저 1개 bag(`two_table_around`) × 16객체로 전체
   파이프라인을 검증·튜닝하고, 결과 확인(checkpoint) 후 나머지 8개 bag 일괄 실행.
2. **PEM 실행 범위 = ISM 게이트 적용.** 16객체 ISM은 모두 실행하되 PEM(포즈)은
   `det_score_thresh`(0.2) 이상 통과한 객체-프레임에만 실행. 통과 못한 프레임은
   CSV에 `status=ism_below_thresh`로 기록(무의미한 대량 포즈 생성 억제).
3. **VLM 평가 = 대표 표본 샘플링.** 38k 전수 대신 bag×object 대표 프레임 + score
   극단(고/저) 사례를 수백 장 샘플링하여 Claude Vision 라벨링. 전수는 score 기반
   자동 휴리스틱 라벨.

## 입력 인벤토리 (검증됨)
- **객체 16종** (template 폴더명 = object_id; `_logs` 제외): Bear, choco_hazelnut_color_high/low,
  Dinosaur, Febreze_high/low, Milk_scaled_195mm, Mugcup_color_high/low, Rabbit, Rack, saffron,
  Sauce_high/low, Sikhye_high/low. 각 폴더는 `templates/{rgb,mask,xyz}_*.npy|png` 42뷰 보유.
- **CAD 매핑**: `data/cad/<object_id>/*.ply` (1:1), 예외 1건 `Milk_scaled_195mm → data/cad/milk/Milk.ply`.
- **bag 9종**: high_texture_around, high_texture_far_close, low_texture_around, low_texture_far_close,
  two_table_around, two_table_around_goback, two_table_diagonal1, two_table_diagonal2, two_table_goback.
  일부는 `<bag>/bag/bag_0.db3` 중첩. 토픽: color `/camera/camera/color/image_raw`,
  aligned depth `/camera/camera/aligned_depth_to_color/image_raw`, info `/camera/camera/color/camera_info`.
- **엔진**: `sam6d_master/SAM-6D/run_batch_inference_fast.py` (ISM+PEM 1회 로드 후 프레임 루프;
  객체당 `--cad_path`,`--template_dir`; `vis_pem/{stem}.png`, `{stem}/detection_pem.json` 산출).
- **HW**: RTX PRO 6000 ×4 (97GB), conda env `sam6d_ros_humble`.

## 산출물 트리
```
outputs_e2e/
  _frames/{bag}/{rgb,depth}/{idx:06d}.png, camera.json        # 추출 프레임(객체 공유)
  _raw/{bag}/{object}/...                                      # 엔진 원시 출력
  {bag}/{object}/frame_{idx:06d}.png                           # 주석 Overlay
  {bag}/{object}/pose_results.csv                              # frame_idx,score,tx..tz,qx..qw,status
  _logs/{run.log,summary.csv,error.csv,preflight.json}
  evaluation/pose_quality_summary.csv
  evaluation/pose_quality_report.md
  evaluation/montage/{bag}/{object}.png
```

## 컴포넌트 (tools/e2e_pipeline/)
- `00_preflight.py` — 템플릿/CAD/메타 존재·bag 재생·토픽 자동탐색 검증, skip 리스트 산출.
- `01_extract_frames.py` — color+aligned_depth 타임스탬프 동기화, stride=10, camera.json(cam_K) 생성.
- `02_run_inference.py` — (bag×object) 오케스트레이션. 4-GPU 라운드로빈, 동시 N. 객체/프레임/bag 실패 격리, 절대 전체 중단 금지. run.log/summary.csv/error.csv.
- `03_postprocess.py` — vis_pem → object name+score+status 주석 frame_*.png, R→quaternion으로 pose_results.csv.
- `04_evaluate.py` — GT-less 휴리스틱 라벨(detection/pose) + 대표 표본 선정.
- `05_montage.py` — bag/object 대표 montage.
- `06_report.py` — pose_quality_report.md + 최종 요약.

## GT-less 평가 가능성 (정직성)
- **계산 불가**(GT 부재): ADD, ADD-S, Rotation/Translation Error, 엄밀 TP/TN/FP/FN. → 리포트에 "불가"로 명시.
- **계산 가능**(프록시): final/ISM/PEM score 분포, depth 일관성(tz vs depth median), 검출률, frame당 객체 경합, VLM 정성 라벨.

## Acceptance Criteria
- AC1 (preflight): Given 16 템플릿/9 bag, When 00_preflight 실행, Then 각 객체의 templates+CAD 존재와 각 bag의 3토픽 존재를 검증하고 누락은 skip 리스트로 기록하며 비정상 종료하지 않는다.
- AC2 (frames): Given 한 bag, When 01_extract 실행(stride=10), Then `_frames/{bag}/rgb`,`depth` 동수의 동기화 PNG와 `camera.json`(cam_K 9원소)을 생성한다.
- AC3 (inference): Given 추출 프레임 + 16객체, When 02_run 실행, Then 객체 실패는 해당 객체만 skip하고 나머지를 계속 처리하며 error.csv에 사유를 남긴다.
- AC4 (outputs): Given 한 (bag,object) 성공, Then `frame_{idx}.png`(RGB|렌더포즈, object name·score 표시)와 `pose_results.csv`(9필드+status)가 생성된다.
- AC5 (eval): Given 전 결과, When 04/06 실행, Then pose_quality_summary.csv와 pose_quality_report.md가 생성되고 GT 필요 지표는 "불가"로 명시된다.

## Out of Scope
- GT 6D 포즈 구축, 정량 ADD/ADD-S, 멀티객체 동시 추론(객체별 단일 템플릿 유지), 실시간 ROS 노드 변경.
