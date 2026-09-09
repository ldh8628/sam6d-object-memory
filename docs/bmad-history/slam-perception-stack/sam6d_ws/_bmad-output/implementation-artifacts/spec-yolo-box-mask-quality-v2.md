---
title: 'YOLO Box/Mask Quality v2 — GT-Accurate Failures + milk_0609 (3 datasets)'
type: 'chore'
created: '2026-06-10'
status: 'in-review'
baseline_commit: '503e871dc4ff6dd4b4d4b03b21dd3f939ea4bbf3'
context:
  - '{project-root}/outputs/yolo_test/yolo_box_mask_quality_report.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-yolo-box-mask-quality.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** v1 box/mask 검증은 BOX-ONLY READY였으나 (1)SLAM 혼동행렬이 `frame_results.csv` decision_type 기반이었고 (2)failures 폴더가 GT 무관 `low_conf`까지 담아 milk-부재 정상거부가 실패로 오분류됨, (3)일반화 데이터 부족. 정확한 0/1 GT와 3번째 bag이 필요.

**Approach:** **오프라인 전용**(프로덕션 무수정). 세 데이터셋(SLAM_with_milk_nomilk / only_Milk / milk_0609)에 YOLO-World(yolov8m, 'milk carton') box + box-guided mobile_sam mask 실행. **SLAM은 frame-정렬 0/1 GT `visibility_labels_thisrun.csv`(311/311)로 TP/TN/FP/FN 재계산**(사용자 승인 — 지정 visibility_labels.csv는 57만 정렬). failures는 **진짜 오류만**(SLAM FP/FN, only_Milk missed, milk_0609 unknown_gt 저신뢰). 확장 per_frame.csv + 보고서 + READY/BOX-ONLY/NOT 판정.

## Boundaries & Constraints

**Always:**
- **오프라인·시각화 산출만.** 프로덕션 코드 0줄 수정. YOLO/SAM은 `sam_yolo` env, bag 추출은 `sam6d_ros_humble`(read-only).
- **SLAM GT = `outputs/validation/SLAM_with_milk_nomilk/visibility_labels_thisrun.csv`** (frame-정렬 311/311, 1=present/0=absent, present118/absent193). gt_source 컬럼에 명시. frame_results.csv는 GT로 미사용.
- conf sweep 0.01/0.02/0.05/0.10, detected = conf ≥ thr. confusion_type ∈ {TP,TN,FP,FN}(GT 있을 때), 없으면 {det,nodet}.
- **failures 저장 규칙(엄격)**: SLAM=FP·FN만, only_Milk=missed(present 가정 하 미검출)만, milk_0609(GT없음)=no-detection/low-conf만 `unknown_gt` 라벨. TP/TN/정상검출 저장 금지.
- only_Milk·milk_0609는 RGB 미추출 → bag에서 순차 추출. only_Milk=all-present 가정 명시. milk_0609=GT 없으면 unlabeled, **GT 날조 금지**.
- 오버레이는 frame_id·prompt·conf·bbox·(mask)·gt_label·pred_label 표시.

**Ask First:**
- milk_0609 color 토픽이 기본과 다르면 실제 토픽 보고. 프레임 수가 매우 커서(>2000) 산출물 과다 시 stride 적용 여부 보고.

**Never:**
- `sam6d_ros_humble` 패키지/ROS 노드/publish/FastSAM/SAM-6D score·ranking·threshold/workspace·depth gate 수정. 프로덕션 ultralytics 업그레이드. git add/commit/push. GT 날조.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output | Error Handling |
|---|---|---|---|
| SLAM 혼동 | 311 frame + thisrun GT | thr별 TP/TN/FP/FN·P/R/F1 | N/A |
| SLAM failures | FP/FN 프레임만 | failures/ 오버레이(gt+pred) | TP/TN 제외 |
| only_Milk | 865 frame(all-present) | recall + conf/bbox/mask 분산, missed만 failures | N/A |
| milk_0609 추출 | bag color 토픽 | 순차 PNG + 토픽·수 보고 | 토픽 다르면 보고 |
| milk_0609 무GT | 라벨 없음 | det rate·conf·안정성·시각, unknown_gt failures | TP/FP 주장 금지 |
| 확장 CSV | 전 데이터셋 | per_frame.csv(지정 컬럼) | N/A |

</frozen-after-approval>

## Code Map

- `tools/validation/extract_frames_from_bag.py` -- 범용 추출기(기존, 재사용; milk_0609도 동일 CLI).
- `tools/validation/yolo_box_mask_quality.py` -- v1 도구(**개정 대상**): GT 소스 교체, failures 규칙 엄격화, milk_0609 추가, 확장 CSV, gt/pred 오버레이.
- `outputs/validation/SLAM_with_milk_nomilk/frames` -- SLAM 311 프레임.
- `outputs/validation/SLAM_with_milk_nomilk/visibility_labels_thisrun.csv` -- **SLAM 권위 GT(311/311)**.
- `data/only_milk`, `data/milk_0609` -- bag(color `/camera/camera/color/image_raw`).
- `outputs/yolo_test/<dataset>/{frames,boxes,masks,failures}` + `per_frame.csv` -- 산출.

## Tasks & Acceptance

**Execution:**
- [x] `tools/validation/extract_frames_from_bag.py` -- milk_0609 추출(color `/camera/camera/color/image_raw`, **1964 frames**). 기존 기능 충분, 무개정. ✓
- [x] `tools/validation/yolo_box_mask_quality.py` -- 개정: (a)SLAM GT=visibility_labels_thisrun.csv, (b)milk_0609 데이터셋 추가(무GT 경로), (c)failures 엄격화(SLAM FP/FN·only_Milk missed·milk_0609 unknown_gt), (d)확장 per_frame.csv(dataset,frame_id,gt_label,gt_source,detected,confidence,threshold,pred_label,confusion_type,bbox_x1..y2,bbox_area,mask_area,box_mask_iou,coverage,leakage,yolo_ms,mask_ms), (e)오버레이 gt+pred 표시. 헤더 validation-only 문장 유지.
- [x] `outputs/yolo_test/yolo_box_mask_quality_report.md` -- v2 재작성(**판정=BOX-ONLY READY**): Executive Summary + Dataset Summary + **SLAM (visibility_labels_thisrun.csv 기준)** + only_Milk + milk_0609 + Box/Mask Quality + Failure Audit + Runtime + Production Cleanup Note(지정 문장) + **READY/BOX-ONLY READY/NOT READY** + Recommended Next Story. "다음 액션 제안" 마무리.

**Acceptance Criteria:**
- Given SLAM, when 평가, then TP/TN/FP/FN이 `visibility_labels_thisrun.csv`(gt_source 명시) 기준 thr별로 계산됨(frame_results 미사용).
- Given SLAM failures 폴더, then **FP·FN 이미지만** 존재(TP/TN 없음).
- Given only_Milk failures, then **missed 이미지만** 존재.
- Given milk_0609, then 추출·평가되고 GT 없으면 det rate/conf/안정성만 보고(혼동행렬 주장 없음), failures는 unknown_gt 라벨.
- Given 각 데이터셋, then 지정 컬럼의 `per_frame.csv` 생성, 보고서에 판정·Production Cleanup Note(정확 문장) 포함.

## Verification

**Commands:**
- `conda run -n sam6d_ros_humble python tools/validation/extract_frames_from_bag.py --bag data/milk_0609 --out outputs/yolo_test/milk_0609/frames --topic /camera/camera/color/image_raw`
- `conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset SLAM_with_milk_nomilk`
- `conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset only_Milk`
- `conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset milk_0609`

**Manual checks:**
- SLAM failures/ 안에 TP/TN 이미지가 없음(FP/FN만).
- per_frame.csv의 gt_source가 SLAM에서 visibility_labels_thisrun.csv.
- 프로덕션 `sam6d_ros_humble` ultralytics 8.0.135 불변.
