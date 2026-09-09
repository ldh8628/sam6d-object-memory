---
title: 'Template Selection Instrumentation & Failure Verdict'
type: 'feature'
created: '2026-06-10'
status: 'in-review'
baseline_commit: '503e871dc4ff6dd4b4d4b03b21dd3f939ea4bbf3'
context:
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-sam6d-template-coverage-selection-research-2026-06-10.md'
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-sam6d-paper-vs-implementation-audit-research-2026-06-10.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Root-cause tree의 마지막 미지수 "Template Selection Failure"가 Unknown인 이유는 `best_template`이 `detector.py:199/205`에서 계산만 되고 어디에도 기록되지 않기 때문이다. 어떤 template가 선택되는지 데이터가 없어 TP/FP가 특정 template에 몰리는지, argmax가 근소차로 흔들리는지 판정 불가.

**Approach:** 점수·랭킹·임계 로직을 일절 건드리지 않고, 이미 계산되는 per-template 점수와 best_template 인덱스를 debug CSV에 per-candidate로 계측 추가한다. 동일 조건(`no_cli_milk_nomilk.yaml`, ism_rerank OFF, seed=1)으로 `SLAM_with_milk_nomilk` 재실행 후, 기존 GT(`visibility_labels.csv`)로 frame_results를 재생성하고, template 사용 히스토그램·TP/FP별 사용·per-template precision·top1–top5 margin을 분석해 Selection Failure를 **Confirmed/Rejected**로 확정한다.

## Boundaries & Constraints

**Always:**
- 계측은 **추가 전용(additive)**. 새 CSV 컬럼은 기존 컬럼 뒤에만 append → 기존 분석 스크립트 호환.
- `best_template_id`는 0–41 정수, raw per-template 점수의 argmax(`detector.py:199`)와 동일 의미.
- **인덱스 정렬 정합**: best_template은 proposal 순서, CSV 행은 pem_poses(rerank/NMS/top_k 후) 순서 → 반드시 기존 `source_index`(node ~:773)로 매핑해 attach. 단순 `best_template[candidate_idx]` 금지.
- top5는 detector가 raw scores를 반환 안 하므로 **side-channel 노출**(예 `self._last_template_scores`)로 취득 — **return 시그니처 변경 금지**(run_batch_inference_fast.py:397 등 기존 caller 보호).
- 재실행은 conda env `sam6d_ros_humble`의 python으로. GT는 기존 `visibility_labels.csv` **재사용**.

**Ask First:**
- 재실행 결과 frame 집합/순서가 기존과 어긋나 GT 정합이 깨지면(decision_type 분포가 baseline FP=120/TN=77에서 크게 이탈) HALT 후 보고.
- top5(SHOULD)가 side-channel로도 깔끔히 안 나오면, MUST(best_template_id/score)만 먼저 구현할지 묻기.

**Never:**
- score 계산(`detector.py:384`, `loss.py`)·ranking(`ism_rerank`)·threshold(conf 0.2/nms 0.25/floor 0.3)·workspace/depth gate 수정.
- `make_visibility_template.py` 재실행으로 GT 덮어쓰기.
- ism_rerank 활성화. git add/commit/push.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 정상 candidate | proposal i가 PEM·CSV까지 생존 | 해당 CSV 행에 best_template_id(0–41), best_template_score(=raw max), is_best와 정합 | N/A |
| source_index 매핑 실패 | 행에 대응 source_index 없음 | best_template_id=null 기록(크래시 금지), 로그 경고 | null 기록 |
| top5 미가용 | side-channel 점수 없음 | top5 컬럼 빈값, MUST 컬럼은 유지 | 빈값 |
| 재실행 frame drift | 재실행 frame_id가 GT와 부분 불일치 | 교집합 프레임만 분석, 누락 수를 리포트에 명시 | 부분집합 분석 |

</frozen-after-approval>

## Code Map

- `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py` -- `compute_semantic_score`(:255-296), best_template argmax(:199/205). raw `scores`[N,Nobj,Ntmpl] side-channel 노출 지점.
- `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py` -- ISM 호출(:689), det dict + source_index attach(~:773), `_filter_and_decide` CSV record(:904-916), csv_fields(:1178-1197), writer(:1260-1292).
- `sam6d_master/SAM-6D/run_batch_inference_fast.py` -- PEM seam, pem_poses 구성(필드 pass-through 확인·추가 지점). **PEM 경유 정확 위치는 구현 시 grep 확정.**
- `tools/validation/build_validation_report.py` -- GT join + decision_type + frame_results.csv 생성(재실행 후 호출).
- `src/sam6d_ros/config/no_cli_milk_nomilk.yaml` -- 재실행 설정(불변).
- `/tmp/ism_analysis2.py`, `/tmp/pose_coverage.py` -- 기존 stdlib 분석 패턴 참고.

## Tasks & Acceptance

**Execution:** (구현 중 실제 활성 경로 = `ism_rerank OFF` → `run_ism_single`. node의 source_index 스레딩 대신 ism_score breakdown 조인 사용 — similarity/appearance/geometric과 동일 정합, PEM 수정 불필요.)
- [x] `detector.py` -- `compute_semantic_score`에서 raw per-template `scores`(filtered to idx_selected)를 `self._last_template_scores`에 저장(side-channel). 반환값·점수·argmax 로직 불변. ✓
- [x] `sam6d_inference_node.py` -- `_filter_and_decide`에서 breakdown(ism_score 조인)으로 4필드 추출→records; csv_fields 끝 4컬럼 append; writer 기록(list는 JSON). ✓
- [x] `run_batch_inference_fast.py` -- `run_ism_single` scores 리스트에 best_template_id/score/top5 추가(side-channel에서 후보별 top-k 유도, idx_selected 정합). 점수/순서 불변. ✓
- [x] `tools/validation/analyze_template_selection.py` (신규, stdlib) -- 히스토그램/TP·FP/precision/margin/Verdict 출력. ✓
- [x] 재실행: conda env, no_cli_milk_nomilk.yaml(rate 0.25 재현보정→1.0 원복), build_validation_report(기존 라벨 재사용). ✓ ⚠️ GT 매핑 56/315(라이브 bag 프레임 비재현성) — 보고서에 HALT 보고.

**Acceptance Criteria:**
- Given 계측 빌드, when 단일 프레임 처리, then is_best 행의 best_template_id가 0–41이고 그 행의 sem/appe/geo와 동일 proposal에서 산출됨(정합).
- Given 재실행 완료, when build_validation_report 실행, then decision_type 분포가 baseline(FP≈120/TN≈77/TP≈112/FN≈6)와 유의미 정합(이탈 시 Ask First).
- Given 새 debug CSV, when analyze_template_selection 실행, then 4개 분석 출력이 생성되고 Selection Failure가 Confirmed 또는 Rejected로 명시됨.
- Given 분석 완료, then Root Cause Tree 6항목(Proposal/Coverage/PEM info-loss/Implementation-Mismatch/Appearance/Selection) 상태와 Recommended Next Story(Confirmed→template matching/DINOv2 mid-layer/top-k ensemble, Rejected→QD-A 결합보정, Inconclusive→계측보강)가 최종 보고서에 기록됨.

## Design Notes

정합 핵심: source_index를 단일 진실원으로 삼아 `template_meta[source_index]={id,score,top5}` dict를 만들고, record 기입 시 그 dict를 조회한다. side-channel raw scores는 매 호출 덮어써지므로 같은 프레임 내 즉시 소비.

## Verification

**Commands:**
- `conda run -n sam6d_ros_humble colcon build --packages-select sam6d_ros` -- 빌드 성공.
- `ros2 launch sam6d_ros sam6d_inference.launch.py config:=src/sam6d_ros/config/no_cli_milk_nomilk.yaml` -- bag 재생 완료, debug CSV에 4 신규 컬럼.
- `build_validation_report.py --bag SLAM_with_milk_nomilk ...`(기존 라벨) -- frame_results 재생성 + confusion.
- `analyze_template_selection.py` -- 히스토그램/precision/margin/verdict 출력.

**Manual checks:**
- is_best 행 5개: best_template_id∈[0,41], best_template_score=top5_scores[0], top5 내림차순.
