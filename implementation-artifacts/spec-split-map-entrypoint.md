---
title: 두 노트북 맵·URDF 단일 명령 실행
type: feature
created: 2026-09-08
status: in-review
baseline_commit: 9ff2a44
---

<frozen-after-approval reason="사용자가 전달한 계획의 구현을 명시적으로 요청함">

## Intent

일반 SAM 터미널에서 `python3 integration/create_map_urdf_split.py --name SESSION`으로
양쪽 카메라 녹화, 원격 맵·URDF 생성, 결과 회수를 실행한다. 표준 라이브러리 진입점이
기존 설정 파서와 환경 실행 도우미를 이용해 `create_map_urdf.py --two-host-config`로 넘긴다.

## Boundaries & Constraints

Always: 기본 설정은 저장소 `integration/two_host.local.yaml`, 명시적 `--config` 지원.
지원 인자는 name/config/baseline/camera-timeout/input-check-seconds/input-check-only/help뿐이다.
잘못된 설정과 인자는 카메라 시작 전에 거부한다. help는 conda 없이 실행한다.
양쪽 worker에 timeout을 전달하고 기존 단일 호스트 동작을 유지한다.
환경 설치·빌드는 최초 준비에만 수행한다. 사용자는 commit/push/기존 배포 명령 실행을 승인했다.
SLAM master 1 READY 후 SAM slave 3 시작, slave 먼저 종료, 최소 600초,
유실률 ≤0.1%, timestamp p95 ≤5ms, 합산 PTP ≤1ms 검사를 유지한다.
원본·로그·실패 보고서와 기존 세션을 보존한다.

Never: 품질 우회, mode 2 또는 단일 장비 fallback, 원격 SAM 모델 설치,
촬영할 때 패키지 설치나 checkout 변경, 실시간 추론/ObjectMemory/LAN DDS 확대.

## I/O & Edge-Case Matrix

| 상황 | 입력 | 동작 |
|---|---|---|
| 도움말 | --help | conda 없이 0 종료 |
| 정상 | 새 name, 기본 또는 명시적 config | 로컬 realsense 환경에서 기존 분산 경로 실행 |
| 인자 오류 | 미지원 옵션, 잘못된 name/숫자, 600초 미만 | 카메라 시작 전 오류 |
| 설정 오류 | 누락/잘못된 설정 | 단일 장비로 전환하지 않고 오류 |
| 자식 종료 | 실패 코드 또는 신호 | exec로 기존 처리기에 직접 전달 |
| 장비 실패 | SSH/worker/PTP/품질 실패 | 기존 정리와 실패 증거 보존 |

</frozen-after-approval>

## Code Map

- `integration/two_host.py`: stdlib 설정 파서, 환경 명령, preflight, transport.
- `integration/create_map_urdf.py`: 기존 CLI/검증과 분산 경로 선택.
- `integration/two_host_map.py`: 녹화·동기·전송·원격 처리 제어.
- `integration/two_host_worker.py`: camera timeout 소비와 원격 calibration.
- `integration/deploy_remote.py`: push된 SHA 배포, vocabulary, ORB 빌드/self-test.

## Tasks & Acceptance

- [x] `integration/create_map_urdf_split.py`: stdlib 진입점과 조기 인자 검증.
- [x] `integration/two_host_map.py`: timeout을 두 camera worker로 전달.
- [x] `integration/test_create_map_urdf_split.py`, `integration/test_two_host_map.py`: 진입점/환경 명령/종료/신호 및 timeout 검사.
- [x] `integration/two_host.local.yaml`, `integration/TWO_HOST.md`: 실제 원격 conda 경로와 실행 방법.
- [x] 양쪽 녹화 의존성 확인, 원격 누락 ORB 의존성 보충, commit/push/배포 및 SHA 일치.
- [ ] 실제 600초 녹화부터 맵·URDF 회수까지 수행하고 품질 측정. PTP 등 외부 조건이 막으면 미완료를 명시.

Given 일반 터미널, when 새 명령 실행, then 설정된 로컬 conda로 기존 분산 경로를 실행한다.
Given 양쪽 환경 준비, when 배포, then ROS 메시지 정의와 배포 SHA가 일치한다.
Given 실제 촬영 통과, when 원격 calibration, then residual p90 ≤0.10m/5도 및 거리 ≤1m 통과 결과를 회수하고 0.03m/1도 목표도 보고한다.

## Verification

- stdlib 진입점 검사, 기존 create_map_urdf self-test, two_host/map/worker 검사.
- 기존 deploy_remote.py의 ORB build/runtime/self-test 및 양쪽 probe.
- 실제 명령과 two_host_report/sync_quality/calibration quality 보고서로 현장 완료 여부 판정.

## Spec Change Log


## 구현 검증과 현장 제한

진입점 4개, 분산 map/worker 8개, 공통 transport/deploy 13개, realtime 회귀 4개,
카메라 shutdown 1개 테스트와 기존 create_map_urdf self-test 통과.
로컬 realsense에서 실제 ROS RGBD bag 변환/재생 1개, sam6d에서 receiver drain 1개 통과.
3개 독립 리뷰에서 READY 예산 문제를 찾아 timeout에 따른 순차 대기와 baseline 시간을 반영했다.
수정 후 진입점/map/realtime 검사 및 diff whitespace 검사 통과.

원격 conda는 `/home/jucpark/anaconda3/etc/profile.d/conda.sh`로 확인했다.
양쪽 Python 3.12.14/Jazzy와 녹화 imports는 정상이다. 원격 ROS 메시지 버전을
로컬 검증 버전에 맞췄고 초기 ORB 코어·wrapper 빌드와 배포 self-test가 통과했다.
배포 manifest 양쪽 일치, 녹화 메시지·서비스 6종 정의 해시 일치, 양쪽 ORB runtime 검사 통과.
양쪽 실제 ROS bag 변환·재생도 통과했다. 원격의 단일 장비용 replay 테스트는
테스트 실행 시 CONDA_SH를 실제 anaconda3 경로로 지정했다(프로덕션 분산 경로는 기존 환경 상속).
환경 변경 전 원격 `output/split_map_environment_before`에 explicit 목록과 YAML export를 저장했다.

실장비 전체 acceptance는 미완료다. 60초 PTP 12표본 중 1개가 1175.653us로
1000us를 초과했다. 로컬 녹화 예산 48,375,640,064 bytes 대비 여유
28,264,431,616 bytes로 약 18.7GiB가 더 필요하다. 원본을 삭제하거나 품질 검사를 우회하지 않는다.
카메라 열거 결과도 계획과 반대다: SAM에 `253822302376`, SLAM에 `253822301680`이
연결돼 있다. 설정된 SLAM master/SAM slave 역할과 serial은 변경하지 않았으며,
카메라 연결을 바꾼 뒤 실제 sync mode/10분 촬영/calibration 품질을 검증해야 한다.
장비 검사 근거는 Git 제외 `output/split_map_verification/`에 보존한다.

## Suggested Review Order

- 일반 터미널에서 설정 검증 후 기존 환경 명령으로 프로세스를 교체한다.
  [create_map_urdf_split.py:13](../integration/create_map_urdf_split.py#L13)
- 양쪽 worker의 timeout과 제어기의 준비 예산을 함께 반영한다.
  [two_host_map.py:92](../integration/two_host_map.py#L92)
- 실제 exec/신호와 장비 없는 분산 회귀를 검사한다.
  [test_create_map_urdf_split.py:14](../integration/test_create_map_urdf_split.py#L14)
- 최초 준비와 일상 촬영 명령을 설명한다.
  [TWO_HOST.md:40](../integration/TWO_HOST.md#L40)
