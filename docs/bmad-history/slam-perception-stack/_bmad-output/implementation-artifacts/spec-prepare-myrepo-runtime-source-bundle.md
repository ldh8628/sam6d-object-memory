---
title: 'MyRePo 네 알고리즘 실행 번들 공개'
type: 'chore'
created: '2026-07-31T00:00:00+09:00'
status: 'done'
review_loop_iteration: 0
baseline_commit: '14610d3afedd661e0660d08a50a3191141986644'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** ORB3-SLAM, HDL-GRAPH-SLAM, RTAB-MAP, SAM-6D 작업본에는 실행 코드와 입력·가중치·빌드·연구 산출물이 섞여 있어 기업 공유용 GitHub 자료로 바로 사용할 수 없다.

**Approach:** `MyRePo` 최신 `main`에서 새 브랜치를 만들고 네 개의 최상위 폴더에 재현에 필요한 소스·설정·설치 도구·문서만 구성한다. 각 README에는 외부 자산 링크, 실제 명령, 이 노트북의 검증된 입력·결과 수치와 대표 PNG를 넣고 커밋·push 후 Draft PR을 연다.

## Boundaries & Constraints

**Always:** 폴더명은 `ORB3-SLAM/`, `HDL-GRAPH-SLAM/`, `RTAB-MAP/`, `SAM-6D/`로 고정한다. 원본 워크스페이스와 기존 미커밋 변경은 보존한다. colcon/CMake가 참조하는 source/header/manifest/launch/config와 라이선스를 유지하고, README 명령은 저장소 상대경로와 입력 인자로 동작하게 한다. 결과 예시는 `/home/etri/CLI_environment`의 실제 실행 PNG와 로그·요약에서 검증한 수치만 사용하며 실행일·입력 종류·한계를 명시한다. SAM-6D upstream 재배포 권리가 불명확하므로 upstream 소스 전체 대신 고정 버전 다운로드 스크립트와 로컬 ROS 래퍼·portable config만 공개한다.

**Ask First:** 10 MiB 초과 파일, 라이선스가 불명확한 제3자 코드·바이너리, 비공개 다운로드 URL, 또는 네 폴더 밖의 기존 파일 변경이 필수라면 중단하고 확인한다. Draft PR 이후 `main` 병합은 별도 승인 없이는 하지 않는다.

**Never:** 입력 bag/DB3/MCAP, ORB vocabulary 원문, 모델 가중치/checkpoint, CAD/template, PCD/PLY/DB, `build/install/log/output`, 캐시, BMAD·실험 보고서 전체를 커밋하지 않는다. upstream 예제 이미지를 노트북 실행 결과로 표현하거나, GT 없는 지표를 정확도·ATE로 주장하거나, 사용자 홈 절대경로를 실행 기본값으로 남기지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 정상 공개 | 네 작업본과 실제 결과가 존재 | 네 폴더, README, 대표 PNG, 검증 스크립트가 push됨 | N/A |
| 외부 자산 없음 | vocabulary/weights/CAD/input이 없음 | 공식 링크·체크섬/배치 경로·생성 절차 제공 | 링크나 목적지가 불명확하면 검증 실패 |
| 경로·산출물 혼입 | 홈 경로 또는 금지 확장자가 존재 | bundle 검사 실패 | 복사본만 수정하고 원본 보존 |
| 즉시 E2E 불가 | 로컬 환경 또는 자산이 빠짐 | README에 확인된 빌드 단계와 현재 차단점 구분 | 성공으로 과장하지 않음 |

</frozen-after-approval>

## Code Map

- `orbslam_ws/src/{ORB_SLAM3,orbslam3_ros2}/` -- 커스텀 ORB3 코어·ROS 2 RGB-D 실행 경로.
- `hdlgraphslam_ws/src/{hdl_graph_slam,fast_gicp,ndt_omp}/` -- ROS 2 HDL graph 및 CPU registration 의존성.
- `rtabmap_ws/src/{rtabmap,rtabmap_ros}/` -- RTAB-Map 코어와 ROS 2 wrapper.
- `sam6d_ws/src/sam6d_ros/`, `sam6d_ws/sam6d_master/SAM-6D/` -- 공개 가능한 wrapper와 upstream 설치·실행 계약의 근거.
- `/home/etri/CLI_environment/*/output/` -- README에 선별 복사할 실제 결과 PNG·수치의 provenance.
- `/home/etri/foundpose/release/MyRePo-worktree/` -- 유일한 제품 변경 및 GitHub publish 대상.

## Tasks & Acceptance

**Execution:**
- [x] `ORB3-SLAM/`, `HDL-GRAPH-SLAM/`, `RTAB-MAP/`, `SAM-6D/` -- 최소 runtime tree, 환경 정의, 자산 다운로드/검증 스크립트, `.gitignore`를 구성한다.
- [x] `*/README.md`, `*/docs/results/` -- 준비→빌드→입력 배치→실행→결과 확인 순서와 공식 링크, 실제 입력·결과 표, 알고리즘별 대표 PNG를 기록한다.
- [x] `scripts/verify_slam_bundles.sh` -- 필수 manifest, shell/Python 문법, 깨진 링크, 금지 파일·홈 경로·대파일을 검사한다.
- [x] GitHub 게시 준비 -- `origin/main` 기준 `agent/add-slam-runtime-bundles` 브랜치에서 의도한 네 폴더와 검증 도구만 변경한다. 리뷰 완료 후 명시적 stage/commit/push 및 Draft PR 생성은 게시 단계에서 수행한다.

**Acceptance Criteria:**
- Given 자산과 입력이 없는 clone, when README와 installer를 확인하면, then 공식 획득 위치·예상 배치 경로·실행 명령을 알고 누락 시 명확한 오류를 받는다.
- Given 네 번들, when 검증 스크립트를 실행하면, then manifest/문법/금지 패턴/10 MiB 제한 검사가 통과하고 원본 입력·출력이 없다.
- Given 결과 README, when 이미지를 열고 provenance와 비교하면, then ORB 3,810 RGB-D frame, HDL 878 scan/4 KF/1 loop, RTAB 145 node/140 exported pose, SAM ISM·PEM 후보와 한계가 근거와 일치한다.
- Given 원본 dirty worktree, when publish가 끝나면, then 기존 사용자 변경은 그대로이며 MyRePo Draft PR URL과 검증 결과를 보고한다.

## Spec Change Log

- 2026-08-05: 사용자가 기존 “로컬 정리·결과 제외” 범위를 GitHub push와 실제 결과 이미지 포함으로 명시적으로 변경하여 경계·작업·AC를 갱신함.

## Design Notes

네 폴더는 기존 workspace 중첩 구조를 유지해 상대경로 의존성을 보존한다. RTAB/ORB/HDL 수정 소스는 해당 라이선스와 함께 싣고, SAM upstream은 설치 시 clone하여 라이선스 수락·버전 출처가 분리되도록 한다. README는 “검증된 로컬 결과”와 “새 clone에서 필요한 준비”를 별도 절로 나눈다.

## Verification

**Commands:**
- `bash scripts/verify_slam_bundles.sh` -- 네 번들의 구조·문법·링크·금지 항목 검사 성공.
- `find ORB3-SLAM HDL-GRAPH-SLAM RTAB-MAP SAM-6D -type f -size +10M -print` -- 출력 없음.
- `git diff --check && git status --short` -- whitespace 오류가 없고 의도한 네 폴더·검증 도구만 변경됨.
- `gh pr view --web`에 해당하는 PR 조회 -- 원격 Draft PR이 존재하고 base가 `main`임.

## Suggested Review Order

**공개 계약과 재현 절차**

- 네 번들의 공통 공개 원칙과 ORB 실행·결과 근거를 먼저 확인한다.
  [`README.md:1`](../../../foundpose/release/MyRePo-worktree/ORB3-SLAM/README.md#L1)

- ROS1 예제를 ROS2 입력으로 변환하는 HDL 경계를 확인한다.
  [`README.md:21`](../../../foundpose/release/MyRePo-worktree/HDL-GRAPH-SLAM/README.md#L21)

- RTAB 입력 토픽·DB 출력 계약과 실제 결과 한계를 확인한다.
  [`README.md:29`](../../../foundpose/release/MyRePo-worktree/RTAB-MAP/README.md#L29)

- 재배포하지 않는 SAM upstream·모델 자산의 설치 계약을 확인한다.
  [`README.md:1`](../../../foundpose/release/MyRePo-worktree/SAM-6D/README.md#L1)

**외부 자산 경계**

- 고정 revision을 원자적으로 설치해 부분 clone을 남기지 않는다.
  [`install_upstream.sh:4`](../../../foundpose/release/MyRePo-worktree/SAM-6D/scripts/install_upstream.sh#L4)

- 공식 ROS1 bag을 명시적 ROS2 목적지로 변환한다.
  [`convert_ros1_bag.sh:4`](../../../foundpose/release/MyRePo-worktree/HDL-GRAPH-SLAM/scripts/convert_ros1_bag.sh#L4)

**배포 안전장치**

- 필수 manifest·라이선스·helper가 모두 존재하는지 검사한다.
  [`verify_slam_bundles.sh:5`](../../../foundpose/release/MyRePo-worktree/scripts/verify_slam_bundles.sh#L5)

- 대용량 입력·모델·생성 산출물과 비승인 이미지를 차단한다.
  [`verify_slam_bundles.sh:64`](../../../foundpose/release/MyRePo-worktree/scripts/verify_slam_bundles.sh#L64)
