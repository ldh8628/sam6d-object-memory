---
title: 'YOLO-World Box/Mask Quality Validation on Two Bags (Hybrid Readiness)'
type: 'chore'
created: '2026-06-10'
status: 'in-review'
baseline_commit: '503e871dc4ff6dd4b4d4b03b21dd3f939ea4bbf3'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/report-yoloworld-shadow-validation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** YOLO-World shadow 검증으로 'milk carton' 게이트가 SAM-6D FP 82~98% 제거·latency 2.3배 개선을 보였으나, **box가 milk를 실제로 잘 감싸는지 / mask가 PEM 입력으로 쓸 만한지** 미검증. Hybrid(box-guided/mask 전달) 설계 전 두 bag(`SLAM_with_milk_nomilk`=출현/소멸, `only_Milk`=항상 존재)에서 box·mask 품질과 안정성을 확인해야 함.

**Approach:** **오프라인 전용**(프로덕션 무수정). `sam_yolo` env에서 YOLO-World(yolov8m→s fallback, prompt 'milk carton') box 검출 + box-guided SAM mask(Option B)를 두 데이터셋에 실행 → box/mask 품질 지표, dataset별 P/R/F1·안정성, 실패사례, 런타임을 산출하고 **READY / BOX-ONLY READY / NOT READY** 판정.

## Boundaries & Constraints

**Always:**
- **오프라인·시각화 산출만.** 산출물 = 추출 프레임 + 신규 스크립트 + 오버레이/CSV + 보고서. 프로덕션 코드 0줄 수정.
- YOLO/SAM 실행은 **`sam_yolo` env에서만**. bag 추출(읽기전용)은 ROS env(`sam6d_ros_humble`) 사용 가능하나 **패키지 무변경**.
- 출력 루트 `outputs/yolo_test/<dataset>/{boxes,masks,failures}/` + dataset별 summary CSV + `outputs/yolo_test/yolo_box_mask_quality_report.md`. 이미지명에 frame_id+conf 포함(예 `000355_conf0.87_box.png`).
- **SLAM_with_milk_nomilk**: 기존 311 정렬 프레임(`SLAM_with_milk_nomilk/frames`) + GT(`frame_results.csv` decision_type, `visibility_labels.csv`) 재사용. present=118/absent=193, SAM-6D FP=120 기준.
- **only_Milk**: RGB 미추출 → bag `data/only_milk/bag_0.db3` color 토픽 순차 추출. all-present 가정 하 recall=detected/total. 미검출은 FN 또는 실제 부재로 양가 보고(은폐 금지).
- mask = **Option B**: YOLO box를 prompt로 ultralytics SAM(mobile_sam) mask 생성. 지표: mask area, mask bbox, IoU(yolo box vs mask bbox), box내 coverage, box외 leakage.
- frame 추출기는 **범용**(`extract_frames_from_bag.py`): `--bag`(파일/디렉터리), `--out`, `--topic`(기본 color) 필수 + `--max-frames/--stride/--start-index/--prefix` 선택. 순차 인덱스 PNG. 다른 bag에도 재사용.
- `yolo_box_mask_quality.py`는 **validation-only 도구** — 스크립트 헤더·보고서 "Production Cleanup Note"에 정확한 문장 포함: `yolo_box_mask_quality.py is a validation-only tool and must not be shipped as part of the final production runtime.` 제품 런타임/ROS 의존성에 미포함.

**Ask First:**
- only_Milk 프레임 수가 매우 크거나(>1000) 추출이 ROS 의존으로 실패 시 보고 후 진행/대안.
- SAM mask 가중치(mobile_sam.pt) 다운로드(네트워크) 진행 여부.

**Never:**
- `sam6d_ros_humble` 패키지/ROS 노드/publish/FastSAM/SAM-6D score·ranking·threshold/workspace·depth gate 수정. ultralytics 업그레이드(프로덕션). git add/commit/push.
- 게이트/mask를 실제 파이프라인에 연결(이번 Story=측정·시각화만). recall 손실·mask 불량 은폐.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output | Error Handling |
|---|---|---|---|
| SLAM box/mask | 311 frame + GT | TP/FP/TN/FN·P/R/F1 + box/mask 지표 + 오버레이 | N/A |
| only_Milk 추출 | bag color 토픽 | 순차 PNG + frame 수 보고 | ROS 실패시 Ask First |
| only_Milk 안정성 | 추출 frame(all-present) | recall, conf·bbox·mask 분산 | 미검출=FN/부재 양가표기 |
| box-guided mask | YOLO box→SAM | IoU·coverage·leakage·area | box 없으면 mask 생략 |
| 실패사례 | 미검출/저conf/부분box/leak mask | failures/ 저장 + 사유 CSV | N/A |
| 런타임 | warm GPU | YOLO ms + mask ms + total | N/A |

</frozen-after-approval>

## Code Map

- `tools/validation/extract_bag_frames.py` -- 기존 추출기(sync-index 노드 방식, 참고). 신규는 범용 순차 reader.
- `outputs/validation/SLAM_with_milk_nomilk/frames/<fid>.png` -- SLAM 311 프레임(재사용).
- `outputs/validation/SLAM_with_milk_nomilk_baseline_bak/frame_results.csv` -- SLAM GT(decision_type, depth_valid_ratio).
- `outputs/validation/only_Milk/frame_results.csv` -- only_Milk 109 TP GT(참조).
- `outputs/validation/yoloworld_shadow_cache.json` -- SLAM YOLO conf 캐시(yolov8s/m, 재사용 가능).
- `data/only_milk/bag_0.db3` + `metadata.yaml` -- only_Milk bag(color `/camera/camera/color/image_raw` 865msg).

## Tasks & Acceptance

**Execution:**
- [x] `tools/validation/extract_frames_from_bag.py` (신규, ROS env, **범용**) -- rosbag2_py로 `--bag`(파일/디렉터리)의 `--topic` 이미지 순차 디코드 → `--out/<prefix><idx:06d>.png`. 옵션 `--max-frames/--stride/--start-index/--prefix`. 읽기전용. frame 수 보고. 다른 bag 재사용 가능.
- [x] `tools/validation/yolo_box_mask_quality.py` (신규, `sam_yolo` env, **validation-only**) -- 헤더에 정확한 문장 `yolo_box_mask_quality.py is a validation-only tool and must not be shipped as part of the final production runtime.` 포함. 한 데이터셋(frames dir + optional GT)에 YOLO box(yolov8m, prompt 'milk carton', conf sweep 0.01/0.02/0.05/0.10) + box-guided SAM mask. per-frame CSV(frame_id,detected,conf,bbox xyxy,area,infer_ms,mask_area,mask_bbox,box_mask_iou,coverage,leakage,mask_ms). boxes/masks/failures 오버레이(frame_id+conf 파일명). SLAM은 GT로 TP/FP/TN/FN·P/R/F1, only_Milk는 recall+conf/bbox/mask 분산. 프로덕션 무수정.
- [x] `outputs/yolo_test/yolo_box_mask_quality_report.md` (신규, **판정=BOX-ONLY READY**) -- Executive Summary 5 + Input Datasets + Frame Extraction Summary + SLAM Results + only_Milk Results + Box Quality + Mask Quality + Failure Audit + Runtime + **Production Cleanup Note**(지정 문장 포함) + **READY/BOX-ONLY READY/NOT READY** + Recommended Next Story. "다음 액션 제안" 절 마무리.

**Acceptance Criteria:**
- Given 두 데이터셋, when 스크립트 실행, then dataset별 detection 지표와 box·mask 품질 지표가 CSV+오버레이로 산출됨.
- Given SLAM GT, then TP/FP/TN/FN·P/R/F1과 사전 제거 가능 SAM-6D FP 수가 보고됨.
- Given only_Milk, then recall과 conf/bbox center·area/mask area 분산(안정성)이 보고되고 미검출 프레임이 사유와 함께 failures/에 저장됨.
- Given box-guided mask, then IoU·box내 coverage·box외 leakage가 정량화되어 "mask가 PEM 입력으로 충분한지" 판단 근거 제공.
- Given 런타임, then YOLO ms + mask ms + total이 SAM-6D 0.878s와 비교되고 **READY/BOX-ONLY READY/NOT READY** 중 하나가 명시됨.

## Verification

**Commands:**
- `conda run -n sam6d_ros_humble python tools/validation/extract_frames_from_bag.py --bag data/only_milk --out outputs/yolo_test/only_Milk/frames --topic /camera/camera/color/image_raw`
- `conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset SLAM_with_milk_nomilk`
- `conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset only_Milk`

**Manual checks:**
- 프로덕션 `sam6d_ros_humble` ultralytics 8.0.135 불변.
- 오버레이 이미지가 box/mask/conf/frame_id를 표시, 실패사례에 사유 기록.
- 보고서 판정이 4기준(FP감소·recall·box안정·mask충분·latency)과 일치, 손실 은폐 없음.
