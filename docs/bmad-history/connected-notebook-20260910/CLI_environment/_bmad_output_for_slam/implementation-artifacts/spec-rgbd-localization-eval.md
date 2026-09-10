---
title: 'RGB-D-only Localization Reproducibility Eval (ORB3 easyLC vs RTAB)'
type: 'feature'
created: '2026-06-19'
status: 'in-review'
context: []
baseline_commit: 'e6f2c31'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 4개 data_slam bag을 RGB-D-only로 ORB3-SLAM·RTAB-Map에 다시 입력했을 때, 기존 저장 결과(`orb3_easyLC/`, `rtab/`) 대비 위치추정이 얼마나 재현되는지 정량·시각 평가가 없다.

**Approach:** 각 dataset×SLAM의 5개 기준 런 중 "대표(중앙값) 런"을 자동 선택해 기준으로 삼고, 신규 RGB-D 런을 각 3회 실행(rate=1.0)한 뒤, Umeyama 정렬(SE3/Sim3) 후 ATE/RPE/translation·rotation 오차·tracking-lost·성공률을 계산하고 overlay·tracking-status PNG와 최종 비교 보고서를 `slam_comparison_report/localization_eval/`에 생성한다. GT가 없으므로 지표는 **run-to-run 재현성**으로 해석하며 보고서에 명시한다.

## Boundaries & Constraints

**Always:** 모든 경로는 `CLI_environment/` 기준 상대경로. SLAM 입력은 RGB-D topic만(odom/tf/GT/외부 pose 미사용). ORB3는 `conda activate orbslam3`, RTAB는 `conda activate rtabmap`. 신규 런은 기존 검증된 `run_orbslam_four_bags.py`/`run_rtabmap_four_bags.py`(RGB-D, rgbd_odometry)를 재사용. 공통 trajectory 포맷 `timestamp,x,y,z,qx,qy,qz,qw`(CSV). 실행 명령은 run_log.txt와 보고서에 기록. ORB3는 절대 타임스탬프, RTAB는 상대시간(t−t0) 기반으로 pose를 연관.

**Ask First:** 기준 폴더(`orb3_easyLC/`,`rtab/`) 또는 기존 run* 결과를 덮어쓰거나 삭제해야 하는 상황. 신규 SLAM이 한 조건에서 3회 모두 실패(빈 trajectory)하는 경우.

**Never:** 기존 `orb3_easyLC/`·`rtab/` 결과 수정/덮어쓰기. ORB3 결과를 RTAB 기준과 교차 GT로 혼용. RGB-D 외 센서/pose 입력. ORB 소스 strict/relaxed 재변경(현재 relaxed/easyLC 빌드 유지).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 정상 평가 | 기준 런 5개 + 신규 런 3개 존재 | 대표런 선택→정렬→지표 JSON+3 PNG+CSV 생성 | N/A |
| RTAB 절대시간 불일치 | 신규/기준 t0 상이 | t−t0로 정규화 후 최근접 연관(임계 dt) | 임계 초과 pair 제외, 비율 기록 |
| 희소 기준(RTAB one_lap=6 pose) | pose 수 적음 | 가능한 pair로 ATE 계산 | metrics에 n_pairs·"sparse" 경고 기록 |
| tracking lost | pose 누락/급점프 구간 | tracking_status.png에 구간 표시 | 결측 frame 비율 성공률에 반영 |
| 신규 런 일부 실패 | 3회 중 1~2회 빈 결과 | 성공한 런만 집계, 실패수 기록 | 3회 전부 실패 시 HALT(Ask First) |

</frozen-after-approval>

## Code Map

- `slam_comparison_report/orb3_easyLC/{ds}/run{1-5}/trajectory.txt` — ORB3 기준(TUM); 대표런 자동선택 대상
- `slam_comparison_report/rtab/{ds}/run{1-5}/trajectory.txt` — RTAB 기준(TUM, 희소)
- `orbslam_ws/scripts/run_orbslam_four_bags.py` — ORB3 신규 런 (env: ORB_BAG_RATE=1.0, ORB_OUTPUT_SUBDIR, ORB_DISABLE_DENSE)
- `rtabmap_ws/scripts/run_rtabmap_four_bags.py` — RTAB 신규 런 (env: RTAB_BAG_RATE=1.0, RTAB_OUTPUT_SUBDIR)
- `slam_comparison_report/localization_eval/` — 신규 산출물 루트(생성)

## Tasks & Acceptance

**Execution:**
- [x] `slam_comparison_report/loc_eval/select_reference.py` -- 각 ds×system에서 run1-5 상호 Umeyama-정렬 후 평균 ATE 최소인 런을 대표로 선택, `reference_index.md` 기록
- [x] `slam_comparison_report/loc_eval/traj_io.py` -- TUM→공통 CSV 변환 + 로드/연관(상대시간 선형보간) 유틸
- [x] `slam_comparison_report/loc_eval/metrics.py` -- Umeyama(SE3/Sim3) 정렬, ATE/RPE, translation·rotation err(mean/median/max/std), 성공률, tracking-lost 구간 산출 → trajectory_metrics.json, framewise_error.csv
- [x] `slam_comparison_report/loc_eval/viz.py` -- top_view_reference/estimated/overlay.png, tracking_status.png, trajectory_error_plot.png 생성
- [x] `slam_comparison_report/loc_eval/run_localization_eval.sh` -- 24 신규 세션(4×2×3, rate=1.0) 오케스트레이션 + 위 스크립트 호출 + per-run run_log.txt
- [x] `slam_comparison_report/localization_eval/localization_comparison_report.md` -- 14개 필수 항목 보고서 작성

**Acceptance Criteria:**
- Given 4 dataset, when 평가 완료, then 각 `localization_eval/{ds}/{orb3_slam,rtabmap}/`에 estimated_trajectory.csv·aligned_estimated_trajectory.csv·reference_trajectory.csv·trajectory_metrics.json·top_view_overlay.png·tracking_status.png 존재.
- Given GT 부재, when 보고서 작성, then "재현성 해석" 및 RTAB 희소·시간정규화 한계를 명시.
- Given 기존 기준 폴더, when 전체 실행, then `orb3_easyLC/`·`rtab/` 내용 무변경(타임스탬프·바이트 동일).

## Design Notes

- 대표런 선택: 한 ds×system의 5런을 쌍별 Umeyama 정렬해 ATE 행렬(5×5) 구성, 행평균 최소 런 = 중앙값 대표.
- 정렬: 기본 Sim3(스케일 포함, ORB 단안성 스케일 드리프트 흡수) + SE3 결과 병기. 적용 종류를 metrics.json에 기록.
- 연관: 두 trajectory를 t−t0로 정규화, 최근접 시간 매칭(|dt|<0.05s). ORB3는 절대시간도 일치하므로 동일 결과.
- tracking-lost: 인접 pose 시간간격 > 기대간격×3, 또는 위치 점프 > 임계 → lost/jump 마킹.
- 신규 3런 집계: ds×system당 대표 기준에 대해 3런 각각 지표 산출 후 median/std 요약.

## Verification

**Commands:**
- `find slam_comparison_report/localization_eval -name trajectory_metrics.json | wc -l` -- expected: 8 (4 ds × 2 system)
- `find slam_comparison_report/localization_eval -name top_view_overlay.png | wc -l` -- expected: 8
- `md5sum` 기존 기준 trajectory 일부 비교 -- expected: 실행 전후 동일(무변경)

**Manual checks:**
- overlay PNG 1~2개 육안: 기준(검정)·정렬된 신규(주황) 경로가 합리적으로 겹치는지.
- 보고서 14개 항목 모두 존재 및 재현성 해석 명시 확인.
