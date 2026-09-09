---
title: 'YOLO-World Shadow Validation for Milk Presence Gating (GO/NO-GO)'
type: 'chore'
created: '2026-06-10'
status: 'in-review'
baseline_commit: '503e871dc4ff6dd4b4d4b03b21dd3f939ea4bbf3'
context:
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-realtime-detect-then-segment-fp-reduction-research-2026-06-10.md'
  - '{project-root}/_bmad-output/planning-artifacts/research/technical-rgb-texture-discriminability-research-2026-06-10.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Root-cause가 Appearance Ambiguity로 좁혀져 SAM-6D 내부 score 개선은 종료. 다음 후보는 YOLO-World front gate(Detect-then-Segment). 그러나 통합 전 (1)YOLO-World가 milk recall을 유지하며 (2)no-object FP(현 120)를 사전 제거하고 (3)absent 스킵으로 latency를 유지/개선하는지 미검증.

**Approach:** **오프라인 전용 shadow 실험**(프로덕션 무수정)으로 격리 env의 YOLO-World를 311프레임에 실행 → prompt별 recall, presence P/R/F1, FP 제거량, recall 손실, ms/frame 실측, ISM/PEM 스킵 latency 시뮬을 산출하고 **GO / GO(Shadow Only) / NO-GO** 판정.

## Boundaries & Constraints

**Always:**
- **오프라인 분석만.** 산출물 = 격리 env + 신규 분석 스크립트 + 보고서. 프로덕션 코드 0줄 수정.
- **격리 환경 필수.** 프로덕션 `sam6d_ros_humble`(ultralytics 8.0.135, FastSAM 의존)은 YOLOWorld 미지원 → 절대 업그레이드 금지. 별도 conda env **`sam_yolo`**(ultralytics>=8.1) 신규 생성 후 거기서만 실행. 향후 YOLO-World+SAM-6D 통합의 기반 env로 재사용 가능하게 구성. 모든 YOLO-World 패키지 설치는 `sam_yolo`에서만.
- **환경 생성·설치 과정을 보고서에 기록**(생성 명령, 설치 패키지·버전, `sam6d_ros_humble` 무영향 확인).
- GT 재사용: `..._baseline_bak/frame_results.csv`(315 풀-GT, decision_type=TP/FN→present, FP/TN→absent). present=118, absent=197. 프레임 이미지=`SLAM_with_milk_nomilk/frames/<fid>.png`(311장, CSV와 교집합으로 평가).
- presence 판정 = YOLO-World box(해당 prompt class) conf ≥ 임계 1개 이상 → detected. 임계는 sweep(예 0.0/0.05/0.1/…)하여 recall-우선 동작점 보고.
- latency: GPU warm-up 후 N프레임 평균 ms/frame 실측(동일 RTX PRO 6000). seed 고정.

**Ask First:**
- 격리 env 생성·모델 가중치 다운로드(yolov8s/m-worldv2.pt, 네트워크 사용, 수십 MB)를 진행해도 되는지. (CHECKPOINT에서 승인)
- frame↔GT 매칭률이 낮아 평가 표본이 크게 줄면(예 <250) 보고 후 진행.

**Never:**
- ROS 노드/publish 로직/FastSAM/ISM/PEM/score/threshold/workspace·depth gate 수정. 프로덕션 env 패키지 변경. git add/commit/push.
- gate를 실제 파이프라인에 연결(이번 Story는 측정만). recall 손실을 GO 판정에서 은폐.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|---|---|---|---|
| 정상 shadow | 311 frame + 315 GT | prompt별 recall, P/R/F1, FP제거량, recall손실, ms/frame, 스킵 latency | N/A |
| env/weights 부재 | 격리 env 미생성 | CHECKPOINT 승인 후 생성·다운로드 | 네트워크 실패 시 HALT·보고 |
| frame↔GT 미스매치 | fid 교집합 | 교집합만 평가, 표본수 명시 | 표본<250시 Ask First |
| present 누락(recall 손실) | YOLO miss on TP/FN | recall 손실·FN 증가량 정량, GO 차단 근거 | N/A |
| 동작점 트레이드오프 | conf sweep | recall 유지 동작점에서 FP제거량 보고 | N/A |

</frozen-after-approval>

## Code Map

- `outputs/validation/SLAM_with_milk_nomilk_baseline_bak/frame_results.csv` -- 315 풀-GT(decision_type). present/absent 라벨 source.
- `outputs/validation/SLAM_with_milk_nomilk/frames/<fid>.png` -- 311 RGB 프레임(YOLO-World 입력).
- `outputs/validation/SLAM_with_milk_nomilk/visibility_labels.csv` -- 보조 GT(frame_id, milk_visible) 교차검증용.
- `tools/validation/qd_a_fusion_benchmark.py` -- 기존 stdlib/numpy AUC·CV 패턴 참고.
- `/tmp/rerun_logs/launch2.log` -- SAM-6D 0.878s/frame, GPU 0.694s 런타임 baseline(스킵 시뮬 기준).

## Tasks & Acceptance

**Execution:**
- [x] `sam_yolo` conda env (신규, ultralytics 8.4.64 / torch 2.12.0+cu130) + yolov8s/m-worldv2.pt 다운로드 -- 격리 실행 환경(통합 기반 재사용 가능). 프로덕션 무영향(8.0.135 불변 확인). 생성·설치 로그 보고서 기록. ✓
- [x] `tools/validation/yoloworld_shadow_eval.py` (신규) -- prompt 후보 sweep, presence 벤치(P/R/F1/혼동행렬), conf-threshold sweep, FP 제거량(120 대비), recall 손실(118 present 대비), ms/frame 실측, ISM/PEM 스킵 latency 시뮬. **읽기 전용 — 프로덕션 코드/데이터 무수정.** ✓
- [x] `_bmad-output/implementation-artifacts/report-yoloworld-shadow-validation.md` (신규) -- 요청 형식 보고서. **GO/NO-GO = GO (Shadow Only).** ✓

**Acceptance Criteria:**
- Given 311 frame + GT, when shadow eval 실행, then prompt별 recall과 presence P/R/F1이 **수치로** 산출됨.
- Given conf sweep, then "recall 유지(SAM-6D 0.95 대비)" 동작점에서 제거 가능한 FP 개수와 recall 손실/FN 증가량이 명시됨.
- Given 실측 ms/frame과 absent 197/315, then ISM/PEM 스킵 시 예상 평균 latency·FPS가 계산되어 현 0.878s와 비교됨.
- Given 결과, then **GO / GO(Shadow Only) / NO-GO** 중 하나가 기준(recall 유지 & FP 감소 & latency 유지·개선)에 따라 명시되고 Recommended Next Story 제시.

## Verification

**Commands:**
- `conda run -n sam_yolo python tools/validation/yoloworld_shadow_eval.py` -- prompt/P/R/F1/FP제거/recall손실/ms·FPS/판정 출력.

**Manual checks:**
- 프로덕션 `sam6d_ros_humble` 패키지 버전 불변 확인(`conda run -n sam6d_ros_humble python -c "import ultralytics;print(ultralytics.__version__)"` == 8.0.135).
- 보고서의 recall 손실이 정직하게 표기(은폐 없음), GO 판정이 3조건과 일치.
