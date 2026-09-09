---
title: '0729 변환 BAG ORB-SLAM3 실행 및 3D/2D 위치 시각화'
type: 'feature'
created: '2026-07-31'
status: 'draft'
review_loop_iteration: 0
context:
  - '_bmad-output/implementation-artifacts/spec-convert-0729-rgbd-bags.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 변환·검증된 0729 ROS 2 RGB-D BAG 3개에 대해 ORB-SLAM3를 실제 실행한 결과가 없으며, 사용자가 확인할 수 있는 3D 지도와 위에서 본 2D 지도/현재 추정 위치 애니메이션도 없다.

**Approach:** 검증된 prebuilt ORB-SLAM3 ROS 2 런타임으로 세 BAG을 순차 실행하고 `output/orbslam3_0729/`에 원본 SLAM 산출물, 3D 지도, top-down 현재 위치 GIF를 저장한다. 실행·렌더링·검증은 재현 가능한 스크립트와 manifest로 남긴다.

## Boundaries & Constraints

**Always:**
- 입력은 `data_slam/0729/converted/`의 `0729_long_circle_inner_rectified`, `0729_smaill_circle_inner_rectified`, `0729_small_8_rectified` 세 세션과 각각 대응하는 `data_slam/0729/settings/*.yaml`로 고정한다.
- 각 BAG은 독립된 전체 세션으로 순차 실행하며 기존 보정 settings를 그대로 사용한다.
- 확인된 `/home/etri/CLI_environment/orbslam_ws/install`과 그 workspace의 `ORBvoc.txt`를 명시하고, 경로·명령·exit code·검증 결과를 manifest에 기록한다.
- 결과에는 최소 `CameraTrajectory.txt`, `KeyFrameTrajectory.txt`, sparse/dense map PCD, dense map PLY, 3D 미리보기 PNG, top-down 최종 PNG, 추정 경로와 현재 위치가 시간에 따라 표시되는 GIF가 포함되어야 한다.
- 기존 Python 환경은 변경하지 않고 시각화 의존성은 NumPy 1.x 기반 독립 환경에 설치한다.
- 기존 BAG, settings, 사용자 변경 파일을 수정하지 않으며 산출물은 `output/orbslam3_0729/`에만 쓴다.

**Ask First:**
- prebuilt 런타임이 실행 불가능해 ORB-SLAM3 재빌드 또는 자산 교체가 필요한 경우.
- BAG/SLAM 자체 오류로 세션을 생략하거나 입력/settings를 변경해야 하는 경우.
- 기존의 유효한 동일 세션 결과를 덮어써야 하는 경우.

**Never:**
- legacy BAG을 대신 쓰거나 일부 성공을 전체 완료로 보고하지 않는다.
- 손실 자세를 보간하거나 synthetic map/trajectory를 결과로 제시하지 않는다.
- `git commit`, `git push`, 기존 환경 패키지 업그레이드, 원본 데이터 삭제를 수행하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 정상 세션 | 입력과 런타임이 유효함 | SLAM 원본, PCD/PLY, PNG 2종, 현재 위치 GIF 생성 | 해당 없음 |
| 필수 입력 누락 | BAG/settings/runtime/vocabulary 중 하나가 없음 | SLAM 시작 전 실패하고 누락 경로를 명시 | 다른 입력으로 대체하지 않음 |
| SLAM 실행 실패 | 비정상 종료, trajectory/map 부족 | 세션을 failed로 기록하고 로그 보존 | 나머지는 계속하되 전체는 failed |
| 렌더링 실패 | PCD/trajectory 파싱 또는 GIF 인코딩 실패 | SLAM 원본은 보존하고 렌더 단계 실패를 기록 | 가짜/빈 시각화를 만들지 않음 |
| 기존 결과 존재 | 세션 출력 디렉터리가 비어 있지 않음 | 기본 실행은 중단해 덮어쓰기를 방지 | 명시적 재개/덮어쓰기 정책 없이는 변경하지 않음 |

</frozen-after-approval>

## Code Map

- `orbslam_ws/scripts/run_orbslam_0729.py` -- 입력 검사, 순차 launch, 로그/manifest와 결과 검증.
- `orbslam_ws/scripts/render_orbslam_0729.py` -- dense PCD와 trajectory의 3D/top-down PNG·GIF 렌더링.
- `orbslam_ws/requirements-visualization.txt` -- 독립 렌더링 환경 의존성.
- `slam_comparison/src/analyze_slam.py` -- PCD/trajectory 로딩과 현재 위치 GIF의 기준 구현.
- `output/orbslam3_0729/` -- 세션별 실행 결과와 전체 summary/manifest.

## Tasks & Acceptance

**Execution:**
- [ ] `orbslam_ws/requirements-visualization.txt` -- NumPy 1.x, matplotlib, Pillow를 고정한다.
- [ ] `orbslam_ws/scripts/run_orbslam_0729.py` -- 정확한 세 세션을 순차 실행하고 설정·로그·원본 결과·검증 상태를 세션별로 수집한다.
- [ ] `orbslam_ws/scripts/render_orbslam_0729.py` -- 실제 dense map과 trajectory로 3D PNG, top-down PNG, 현재 위치 GIF를 생성한다.
- [ ] `output/orbslam3_0729/` -- 스크립트를 실제 BAG에 실행하고 세션별 결과 및 전체 summary를 저장한다.

**Acceptance Criteria:**
- Given 입력이 존재할 때, when 실행기를 시작하면, then 세 세션이 순차 처리되고 명령과 런타임 provenance가 기록된다.
- Given 성공한 세션일 때, when 검증하면, then 복수 유효 자세와 비어 있지 않은 sparse/dense PCD 및 dense PLY가 있다.
- Given 유효한 dense map과 trajectory일 때, when 렌더러를 실행하면, then 실제 점군 기반 3D 미리보기와 top-down 지도 위에서 경로 및 현재 추정 위치가 이동하는 GIF가 생성된다.
- Given 모든 처리가 끝났을 때, when `summary.json`을 검사하면, then 세 세션별 성공/실패와 파일 크기·pose/frame 수가 정직하게 집계되며 일부 실패는 전체 성공으로 표시되지 않는다.

## Spec Change Log

## Design Notes

prebuilt 런타임은 이 장비에서 실행 이력이 있고 ROS 2 executable과 vocabulary가 확인된 경로다. 2D 평면은 trajectory 변화가 가장 작은 축을 수직으로 선택하며, GIF는 누적 경로와 현재 자세를 표시한다. 3D 미리보기만 결정적으로 subsampling하고 PCD/PLY 원본은 보존한다.

## Verification

**Commands:**
- `python -m py_compile orbslam_ws/scripts/run_orbslam_0729.py orbslam_ws/scripts/render_orbslam_0729.py` -- expected: 문법 오류 없음.
- `python orbslam_ws/scripts/run_orbslam_0729.py --check-only` -- expected: 세 BAG/settings와 runtime/vocabulary 계약 모두 통과.
- `python orbslam_ws/scripts/run_orbslam_0729.py` -- expected: 세 세션 실행 후 manifest 생성.
- `python orbslam_ws/scripts/render_orbslam_0729.py --output-root output/orbslam3_0729` -- expected: 세션별 PNG 2종과 GIF 생성.
- `python orbslam_ws/scripts/run_orbslam_0729.py --verify-only` -- expected: 세 세션의 필수 원본·시각화 결과와 summary가 모두 PASS.

**Manual checks (if no CLI):**
- 각 GIF를 열어 top-down 점군 지도 위에서 추정 경로가 누적되고 현재 위치 마커가 마지막 자세까지 이동하는지 확인한다.
- 3D 미리보기와 PLY/PCD의 형상이 동일 세션의 top-down 지도와 공간적으로 일관적인지 확인한다.
