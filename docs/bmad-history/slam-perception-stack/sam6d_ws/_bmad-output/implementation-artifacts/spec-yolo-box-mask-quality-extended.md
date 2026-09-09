---
title: 'YOLO Box/Mask Quality Extended — high_texture_around + high_texture_far_close generalization'
type: 'chore'
created: '2026-06-12'
status: 'in-review'
baseline_commit: '503e871dc4ff6dd4b4d4b03b21dd3f939ea4bbf3'
context:
  - '{project-root}/outputs/yolo_test/yolo_box_mask_quality_report.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-yolo-box-mask-quality-v2.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 기존 BOX-ONLY READY 판정은 3개 데이터셋(SLAM_with_milk_nomilk / only_Milk / milk_0609)에서만 검증됨 — 전부 milk가 화면 주변 텍스처가 적거나 거리 변화가 제한된 조건. Hybrid(YOLO box → SAM mask → PEM) 설계로 넘어가기 전, **고텍스처 주변 환경**과 **원근(far↔close) 변화** 두 신규 ROS2 bag에서 동일 검증을 반복해 결론의 **일반화 가능성**을 확증해야 한다. 특히 "YOLO confidence의 데이터셋 의존성"이 저텍스처 특성인지 구조적 문제인지를 가린다.

**Approach:** **오프라인 전용**(프로덕션 무수정). **전 파이프라인을 단일 env `sam_yolo`에서 실행**(사용자 지정) — 추출도 `sam6d_ros_humble`가 아니라 sam_yolo에서. ROS 미설치 sam_yolo에서는 **순수 파이썬 `rosbags`(humble typestore)** 로 `.db3`를 직접 읽어 RGB 추출(검증 완료: enc rgb8 480×640). 두 신규 bag(`high_texture_around` 3629 color / `high_texture_far_close` 2189 color)을 추출 → 기존 `yolo_box_mask_quality.py`에 두 데이터셋을 **gt_mode="none"(unknown_gt)** 로 추가 → YOLO-World(yolov8m-worldv2, 'milk carton') box + box-guided mobile_sam mask 실행. **둘 다 GT 없음** → det rate·conf 분포·bbox/mask 안정성만 보고(혼동행렬 주장 없음, GT 날조 금지). 산출물 과다 방지를 위해 **stride 2 추출**(around≈1815, far_close≈1095; 시간 커버리지 보존, 명시 보고 — silent cap 금지). 5개 데이터셋 cross-comparison 표 + confidence 일반화 분석 + 갱신된 READY/BOX-ONLY/NOT 판정.

## Boundaries & Constraints

**Always:**
- **오프라인·시각화 산출만.** 프로덕션 코드 0줄 수정. **추출·YOLO·SAM 전부 `sam_yolo` env에서 실행**(사용자 지정 — sam6d_ros_humble 미사용). 추출은 `rosbags`(pure-python, humble typestore)로 `.db3` 직접 읽기, bag은 read-only.
- 두 신규 bag 모두 GT 없음 → `gt_mode="none"`. det/nodet만, **TP/TN/FP/FN·P/R/F1 주장 금지**. failures = `unknown_gt_lowconf`/`unknown_gt_nodet`만.
- 모델 `yolov8m-worldv2.pt`(실패 시 s fallback), prompt 'milk carton', conf sweep 0.01/0.02/0.05/0.10, detected = conf ≥ 0.05.
- **stride=2 추출**을 적용하고 추출된 정확 프레임 수를 보고(no silent cap). color 토픽 `/camera/camera/color/image_raw`(양 bag 동일 확인됨).
- sam_yolo에 추가 설치된 검증용 의존성(`rosbags` 및 그 deps: ruamel.yaml/zstandard/lz4/apsw)을 보고서에 기록. **프로덕션 env(sam6d_ros_humble) 무변경**.
- 오버레이는 frame_id·prompt·conf·bbox·(mask) 표시. 확장 per_frame.csv(기존 컬럼 스키마 동일) 데이터셋별 생성.
- 보고서는 5개 데이터셋(기존 3 + 신규 2) 비교표 포함, confidence 일반화 분석으로 "저텍스처 특성 vs 구조적 문제" 결론.

**Ask First:**
- color 토픽이 `/camera/camera/color/image_raw`와 다르면 보고. stride 2로도 산출물이 과다하면(>2500 PNG/dataset) stride 상향 여부 보고.

**Never:**
- `sam6d_ros_humble` 패키지/ROS 노드/publish/FastSAM/SAM-6D score·ranking·threshold/workspace·depth gate 수정. 프로덕션 ultralytics 업그레이드. git add/commit/push. GT 날조.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output | Error Handling |
|---|---|---|---|
| around 추출(sam_yolo) | bag color, stride 2, rosbags | 순차 PNG(~1815) + 토픽·수 보고 | 토픽 다르면 보고 |
| far_close 추출(sam_yolo) | bag color, stride 2, rosbags | 순차 PNG(~1095) + 토픽·수 보고 | 토픽 다르면 보고 |
| 무GT 평가 | 라벨 없음 | det rate·conf·bbox/mask 안정성·시각화 | TP/FP 주장 금지 |
| failures | conf<0.05 프레임 | unknown_gt_low/nodet 오버레이만 | 정상검출 제외 |
| cross-comparison | 5개 데이터셋 per_frame.csv | conf mean/std·det rate·IoU·cov·leak 표 | N/A |
| confidence 일반화 | high_texture vs 기존 | 상승/구조적 결론 | N/A |

</frozen-after-approval>

## Code Map

- `tools/validation/extract_frames_from_bag.py` -- 범용 추출기(검증 도구): **`--backend {rosbag2,rosbags,auto}` 추가**(lazy import). sam_yolo에서 동작하도록 rosbags 경로(humble typestore + numpy reshape 디코딩, 기존 `imgmsg_to_bgr` 인코딩 로직 재사용) 추가. 기존 rosbag2 경로·CLI 후방호환 유지. **sam6d_ros_humble 패키지 아님(검증 tool 수정 허용)**.
- `tools/validation/yolo_box_mask_quality.py` -- DATASETS dict에 `high_texture_around`/`high_texture_far_close`(gt_mode="none") **2개 항목만 추가**. 로직·CSV·overlay·failures 규칙 무변경(헤더 validation-only 문장 유지).
- `data/ros2_bag/high_texture_around`, `data/ros2_bag/high_texture_far_close` -- 신규 bag(color `/camera/camera/color/image_raw`).
- `outputs/yolo_test/<dataset>/{frames,boxes,masks,failures}` + `per_frame.csv` -- 산출.
- `outputs/yolo_test/yolo_box_mask_quality_extended_report.md` -- 신규 확장 보고서(5 데이터셋).

## Tasks & Acceptance

**Execution:**
- [x] `tools/validation/extract_frames_from_bag.py` -- `--backend {auto,rosbag2,rosbags}` 추가(lazy import, humble typestore), 기존 rosbag2 경로 후방호환. sam_yolo에서 추출: around **1815**(3629→stride2), far_close **1095**(2189→stride2), color `/camera/camera/color/image_raw`, enc rgb8. ✓
- [x] `tools/validation/yolo_box_mask_quality.py` -- DATASETS에 high_texture_around/high_texture_far_close(gt_mode="none") 추가. 그 외 로직 무변경. ✓
- [x] 두 데이터셋 평가 실행(sam_yolo) → frames/boxes/masks/failures + per_frame.csv 생성. failures=unknown_gt_lowconf만(263/277), gt_source=none·{det,nodet}만 확인. ✓
- [x] `outputs/yolo_test/yolo_box_mask_quality_extended_report.md` -- 5종 비교·conf 일반화(far_close corr 0.878)·**판정 BOX-ONLY READY 유지**·Production Cleanup Note·다음 액션 제안 포함. ✓

**Acceptance Criteria:**
- Given 두 bag, when sam_yolo에서 rosbags로 추출, then color 토픽·정확 프레임 수(stride 2 반영)가 보고됨(sam6d_ros_humble 미사용).
- Given 두 데이터셋, then box overlay·mask overlay·failures(unknown_gt만)·per_frame.csv 생성, GT 없으므로 혼동행렬 주장 없음.
- Given 5개 데이터셋, then conf mean/std·det rate·bbox 안정성·mask IoU/coverage/leakage 비교표 생성.
- Given confidence 일반화 분석, then high_texture에서 conf 상승 여부 + 저텍스처 특성 vs 구조적 문제 결론 제시.
- Given 보고서, then 갱신 판정 + Production Cleanup Note(정확 문장) 포함, 프로덕션 무수정.

## Verification

**Commands (전부 sam_yolo):**
- `conda run -n sam_yolo python tools/validation/extract_frames_from_bag.py --backend rosbags --bag data/ros2_bag/high_texture_around --out outputs/yolo_test/high_texture_around/frames --topic /camera/camera/color/image_raw --stride 2`
- `conda run -n sam_yolo python tools/validation/extract_frames_from_bag.py --backend rosbags --bag data/ros2_bag/high_texture_far_close --out outputs/yolo_test/high_texture_far_close/frames --topic /camera/camera/color/image_raw --stride 2`
- `conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset high_texture_around`
- `conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset high_texture_far_close`

**Manual checks:**
- failures/ 안에 unknown_gt_* 만 존재(정상검출 없음).
- per_frame.csv의 gt_source = none, confusion_type ∈ {det,nodet}.
- 프로덕션 `sam6d_ros_humble` 불변(이번 작업에서 전혀 호출 안 함). 기존 rosbag2 추출 경로도 후방호환 유지.
