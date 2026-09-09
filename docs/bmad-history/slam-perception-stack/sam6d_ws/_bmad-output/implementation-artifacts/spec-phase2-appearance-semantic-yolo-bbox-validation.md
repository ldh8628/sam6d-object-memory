---
title: 'Phase 2 ISM 병목 재검증 — Appearance Block2·9 / Semantic Top-5·전체평균 / YOLO-World BBox 미생성'
type: 'chore'
created: '2026-07-22'
status: 'done'
context: []
baseline_commit: '10ec30f1ad027a5c0d05bce9cf5f0f663dfce429'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Phase 1C(HSV gate t0.1214 + Dinosaur +9 OCV)로 FP는 334→86으로 수렴했으나 잔여 오류는 FN(recall≈.665) 중심이고, appearance(DINOv2 block11 단독)·semantic(top-5 hard gate)·YOLO-World bbox 미생성이 아직 검증되지 않았다. 이 세 축이 실제로 개선 여지가 있는지 데이터로 판정해야 다음 운영 구현 대상을 고를 수 있다.

**Approach:** Phase 1C를 고정 기준선으로 두고, 운영 판정을 바꾸지 않는 read-only 실험 경로에서 세 Workstream을 검증한다 — (A) DINOv2 block2/9/11을 단일 forward로 추출해 단독·fusion 판별력 비교, (B) raw 42-view semantic 배열로 top-k·전체평균·곱셈·가중결합 비교, (C) YOLO-World를 저 conf/해상도/NMS로 재생해 bbox 미생성 failure-stage를 분류·보완책 비교. 6개 GT 데이터셋 기준 LODO로 누수 차단, 통합 P0~P5 구성으로 이득을 합산하고, 최종적으로 운영 구현 후보 1~2개만 근거와 함께 선정한다(자동 적용 금지).

## Boundaries & Constraints

**Always:**
- Phase 1C 값 동결: HSV t=0.1214, Dinosaur hue +9 OCV, texture negative/rescue 비활성. 새 실험이 Phase 1C 결과를 바꾸면 안 된다.
- 실제 코드 우선. 이미 확정된 사실: 운영 semantic=**top-5 mean**(match_topk:5, sim gate 0.35, HARD), 운영 appe=**block 11**(0-based last, appe_gate 0.55, per-patch matched cosine, best_t 앵커), block2/9/11은 `get_intermediate_layers(x,n=[2,9,11])` **단일 forward**로 동시 추출(+0.1%), 검출기=**YOLO-World**(yolov8m-worldv2, imgsz960, ultralytics 기본 NMS iou0.7 class-aware — yaml에 미설정), conf gate=0.02, top_k=3. 게이트 순서 YOLO→semantic→mask→appe→HSV.
- 용어 분리: `dinosaur_class`(객체) vs `dinov2_block2/9/11`(feature). `dino_score` 금지.
- 데이터 누수 차단: 6개 GT 데이터셋(sam_105018/105314/105652/110104/110532/110633) **dataset 단위 LODO**로 weight·threshold 선택, holdout에서 1회만 최종 보고. frame 단위 분리 금지.
- GT는 frame-level visibility(user_visibility_gt.csv, 339프레임, user_reviewed=="yes"만). 935 = visible (frame×object) positive cell 수. per-box IoU GT는 없다 → Workstream C는 IoU recall 대신 **visibility recall**(visible frame에 target box ≥1) 사용.
- 결측값은 임의 0 대체 금지. 빈 값과 원인을 별도 컬럼에 기록.
- 산출물은 `outputs/phase2_appearance_semantic_yolo_bbox/`에만. Phase 1C 산출물·원본 GT 무수정.

**Ask First:**
- 실험 결과가 어떤 Workstream에서든 채택 조건을 명확히 만족해서 **운영 config/판정 변경**을 하고 싶어질 때 → HALT. 이번 작업은 검증·후보 선정까지만.
- 6개 GT 데이터셋 프레임의 GPU 재추출이 불가(GPU 미가용/OOM)할 때 → HALT하고 캐시 부분(2개 겹치는 데이터셋)만으로 축소할지 확인.

**Never:**
- git add/commit/push. 운영 파일(`yolo_ism_object_n.py`, `configs/yolo_ism_objects.yaml`, `ism_hsv.py`)의 판정 로직 변경. color/texture/HSV 추가 튜닝.
- 가짜 bag/이미지/GT 생성. 자동 추정 라벨을 정답으로 사용. provisional과 human GT 혼용.
- test set 통계로 normalization·weight·threshold 선택.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Block2·9 동시추출 | 후보 ROI 1개 | 단일 forward에서 appearance_block2/9/11_score 3개 산출, 재forward 0회 | forward 실패 시 해당 uid skip + 사유 기록 |
| Semantic 유효<5 | valid view score n<5 | available-n mean 정책(운영 top-k=min(k,N)와 일치), 정책 컬럼 기록 | NaN/Inf/empty→후보 invalid 표시, 0 대체 안 함 |
| YOLO raw 0건 | visible frame, predict(conf0.005) 결과 0 | failure_stage=F0(raw 없음)로 분류 | RPC/OOM/timeout은 F6로 분리 기록 |
| GPU 미가용 | cuda 접근 실패 | HALT, 캐시 축소안 질의 | — |
| LODO fold | 6 dataset | fold별 선택→holdout 1회 평가, seed 기록 | dataset<2 fold면 경고 |

</frozen-after-approval>

## Code Map

- `yolo_ism_object_n.py` -- 운영 파이프라인. `dinov2_blocks_forward`(:79), `masked_appe_blocks`(:296), `recognize_frame`(:499 게이트 순서), CSV(:103). **읽기 전용 참조**, 판정 변경 금지.
- `yolo_ism.py` -- `semantic_score`(:238 top-5), `build_template_cls`(:162), query CLS(:117), `crop_resize_pad`(:80). 참조용.
- `configs/yolo_ism_objects.yaml` -- 동결 기준선 값 출처. 스냅샷만 복사, 무수정.
- `_ism_research_2026_07/ism_accuracy_analysis/features/pairs.csv` -- appe2/9/11+sem변형 캐시(10070행, 단 GT 6셋 중 105314·110633만 겹침 → A는 6셋 GPU 재추출 필요).
- `_ism_research_2026_07/ism_fusion_research/semantic_fusion/box_cls.npz`(22436 box CLS+42템플릿) / `.../semantic_analysis/view_sims.npz`(uid→42) -- B raw semantic 오프라인 재현.
- `_ism_research_2026_07/yolo_localization_research/pipeline/cur/{ds}_pairs.csv` -- 6 GT셋 per-candidate 기저표(yolo_conf+sem_top5+appe11+게이트).
- `_ism_research_2026_07/yolo_localization_research/probes/dump_proposals.py` -- C용 conf/imgsz/agnostic/multi-label 스윕 로직(재사용).
- `_ism_research_2026_07/gt_input/user_visibility_gt.csv` + `frames/<ds>/frame_*.png` -- 339프레임 human GT + 재생용 RGB.
- `.../research_a_dinosaur_color/evaluation/eval_phase1c.py` -- per-(frame,object) grid TP/FP/FN/F1 + Mann-Whitney AUROC 로직(재사용).

## Tasks & Acceptance

**Execution:**
- [x] `outputs/phase2_appearance_semantic_yolo_bbox/` -- 디렉토리 트리 + provenance/config 스냅샷·git-hash·seed 기록 완료.
- [x] `_ism_research_2026_07/phase2_bottleneck/probes/dump_phase2_candidates.py` -- 6 GT셋 270프레임 단일 forward(=270) 덤프, appe2/9/11 + raw 42-view semantic + hsv, parity maxabs≈5e-6. `appearance_block_scores.csv`,`semantic_raw_scores.csv`,`phase2_candidates.csv`(18060행) 산출.
- [x] `_ism_research_2026_07/phase2_bottleneck/eval_appearance_blocks.py` -- A0~A4 + block9 승(F1.7653), class 편중 발견 → `appearance_*`,`metrics/appearance_metrics.json`.
- [x] `_ism_research_2026_07/phase2_bottleneck/eval_semantic_aggregation.py` -- S0~S7, KEEP(변형 무개선) → `semantic_*`,`metrics/semantic_metrics.json`.
- [x] `_ism_research_2026_07/phase2_bottleneck/collect_yolo_bbox_diagnostics.py` -- 5 config raw 덤프(conf0.005) → `raw/yolo_raw/*`.
- [x] `_ism_research_2026_07/phase2_bottleneck/eval_yolo_bbox_recovery.py` -- F0~F2 taxonomy(F2=140) + conf/NMS/prompt/해상도 sweep → `yolo_*`,`metrics/yolo_bbox_metrics.json`.
- [x] `_ism_research_2026_07/phase2_bottleneck/eval_phase2_integrated.py` -- P0 정확재현(622/86/.7572) + P1/P3 GPU + 중복분석 → `integrated_pipeline_metrics.csv`,`metrics/integrated_metrics.json`,`phase2_per_candidate_debug.csv`(3390행).
- [x] `_ism_research_2026_07/phase2_bottleneck/make_phase2_figures.py` -- 별도 생성(진행/완료).
- [x] `tests/test_phase2_eval.py` -- 8/8 pass(단일forward·유효<5·F0/F2·LODO누수·결측None·AUROC).
- [x] `_bmad-output/implementation-artifacts/phase2-appearance-semantic-yolo-bbox-validation.md` -- 13섹션 보고서 + outputs/report 사본.

**Acceptance Criteria:**
- Given Phase 1C 회귀 테스트, when test_uniform_threshold/test_hsv_shadow/test_phase1c 실행, then 전부 pass하고 운영 config 값(t0.1214, Dino+9, gate enabled)이 그대로다.
- Given 임의 실패 candidate, when 디버그 CSV 조회, then YOLO raw 유무·소멸 stage·semantic 42배열 요약·block2/9/11·각 threshold·최종판정·사유를 모두 추적할 수 있다.
- Given 각 Workstream, when 채택 조건 대조, then KEEP/CHANGE/MORE-DATA를 holdout 근거와 함께 판정하고, 조건 미달 후보는 P구성·최종 선정에서 제외한다.
- Given weight·threshold 선택, when 최종 성능 보고, then 선택은 LODO 폴드에서, 보고는 holdout에서만 이뤄져 동일 데이터 재사용이 없다.
- Given 작업 종료, when git status 확인, then staged/commit 없음, 운영 판정 로직 diff 없음(shadow/read-only만).

## Design Notes

- **A 단일 forward 필수 증명**: `nms_rank_blocks:[2,9]`가 이미 운영에서 두 블록을 한 forward로 뽑는 산증거. 재forward 횟수는 코드로 카운트해 0임을 보고(AC 6조건).
- **A 캐시 한계**: `ism_accuracy_analysis/features/pairs.csv`는 GT 6셋 중 2셋만 겹침. 6셋 정합 위해 `dump_phase2_candidates.py`로 재추출(frames 존재). 2셋 캐시는 sanity 대조용으로만.
- **B "top-5×전체평균" 의미 확정**: 이전 연구 문서에 결합식 정의 없음 → 곱(S5)·기하평균(S6)·가중합(S7=α·top5+(1-α)·all)을 모두 독립 비교, 임의 택일 금지.
- **정규화 누수 주의**: min-max/z-score는 LODO train 폴드 통계로만 fit.
- **C 기준 NMS**: yaml에 iou 미설정 → 기준선은 ultralytics 기본 iou0.7·class-aware. 스윕은 이 기본을 명시적으로 기준점 표기.
- **AUROC**: rank기반 Mann-Whitney(`eval_fusion.py:115` 방식), np.trapz/sklearn 금지.

## Verification

**Commands:**
- `~/miniconda3/envs/sam_yolo/bin/python -m pytest tests/test_uniform_threshold.py tests/test_hsv_shadow.py tests/test_phase1c.py -q` -- expected: 전부 pass(Phase 1C 회귀).
- `~/miniconda3/envs/sam_yolo/bin/python tests/test_phase2_eval.py` -- expected: 신규 단위 테스트 pass.
- `~/miniconda3/envs/sam_yolo/bin/python -m py_compile _ism_research_2026_07/phase2_bottleneck/*.py` -- expected: 컴파일 성공.
- `git status --porcelain` -- expected: 운영 파일 판정 diff 없음, staged 없음.

**Manual checks:**
- `outputs/phase2_appearance_semantic_yolo_bbox/report/` + CSV/figures 존재, 통합 summary 1장에 6개 질문 답, 최종 구현 후보 1~2개와 근거 명시.
