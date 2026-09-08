---
title: 단일 monorepo의 두 노트북 배포와 실행
type: feature
created: 2026-09-08
status: in-review
baseline_commit: 0221ae5d635e9e6cb3cbf02ab2a0f6e404fba
---

<frozen-after-approval reason="사용자가 전달한 구현 계획 승인">

## Intent

현재 노트북에서 같은 소스를 배포하고 원격 ORB-SLAM3와 로컬 SAM/ObjectMemory를 제어한다.
기존 단일 호스트 CLI는 유지하고 `--two-host-config`로 분산 실행을 선택한다.

## Boundaries & Constraints

Always: 기존 notebook 수정과 Git 이력을 보존한다. 원격 dirty tree를 덮어쓰지 않는다.
동일 SHA와 메시지 타입, PTP, 카메라 serial 및 실제 mode 1/3을 검사한다.
Git에는 코드만 저장하며 자산은 checksum 검증과 staging 승격을 사용한다.
영상은 각 호스트에만 남기고 LAN에는 기존 네 ORB topic만 전달한다.
프레임에 대응하는 pose가 없는 경우 fusion과 overlay를 차단한다.
장애와 map identity 변경 시 in-flight inference까지 차단한다.

Never: 모드 2 자동 fallback, 원격 SAM weights 설치, GPU ORB/IMU/다른 SLAM 도입.

## I/O & Edge-Case Matrix

| 상황 | 입력 | 동작 |
|---|---|---|
| 배포 | push된 tag/SHA, clean 양쪽 | detached checkout, 필요 자산/ORB 빌드, deployment 기록 |
| 배포 실패 | dirty, 미push ref, checksum/build 오류 | 실패 반환, 완료 manifest 미발행 |
| 맵 | 두 카메라와 PTP 준비 | master READY 뒤 slave 시작, slave 먼저 종료 |
| 동기 불량 | 10분 미달, loss >0.1%, p95 >5ms | 보고서와 원본 보존, 맵 품질 승인 거부 |
| calibration | 병진 p90 >0.10m, 회전 >5도, 거리 >1m | reference fallback 포함 실패 처리 |
| 실시간 | timestamp exact/interpolated pose | 현재 map과 healthy lease 확인 후 fusion/overlay |
| 장애 | PTP/LAN/ORB/map 변경 | fail closed, tracking-lost, 진단 회수 |

</frozen-after-approval>

## Code Map

- `integration/two_host.py`, `deploy_remote.py`: 설정, SSH, 자산 동기화, 배포.
- `integration/two_host_worker.py`, `two_host_map.py`: 단일 역할 녹화, 원격 맵 생성.
- `integration/two_host_realtime.py`, `two_host_bridge.py`: 실행 제어와 제한된 pose 통신.
- `sam6d_ws/realtime/`: 기존 timestamp matching, shared memory, 결과 게시 안전성.
- `objectmemory_ws/object_memory/ros/object_memory_node.py`: fusion 입력 계약.

## Tasks & Acceptance

- [x] monorepo 승격과 snapshot 보존, 산출물 제외.
- [x] 배포 CLI와 설정 예제, PTP/환경 preflight와 self-test.
- [x] master/slave worker와 10분 sync 보고서, 원격 strict calibration.
- [x] 분산 realtime CLI, 네 topic bridge, fault와 timestamp 기반 결과 차단.
- [x] 로컬 테스트와 ROS self-test, release tag 및 배포 문서.
- [ ] 실제 두 호스트 배포와 카메라 acceptance 측정(연결 설정/하드웨어 필요).

Given 분산 설정이 없을 때, 기존 CLI를 실행하면 단일 호스트 동작을 유지한다.
Given 잘못된 pose나 만료된 guard가 있을 때, inference가 끝나도 fusion/overlay를 갱신하지 않는다.
Given 실행 실패가 있을 때, partial 결과와 실패 사유를 report에 보존한다.

## Verification

기존 integration self-test, SAM pose/SHM tests와 ObjectMemory 테스트를 실행한다.
신규 stdlib 테스트는 배포/동기/lease/오류 경계를 검증한다.
실제 원격/10분 카메라/움직임 구간 acceptance는 측정 없이는 통과로 표시하지 않는다.

## Spec Change Log

## Design Notes

카메라 domain은 localhost로 격리하고 두 ROS context를 가진 bridge가 네 topic만 LAN domain으로 연결한다.
이는 static peers가 토픽 허용목록 역할을 하지 못하는 문제를 방지한다.

## 검증 기록과 남은 현장 작업

2026-09-08 코드/오프라인/로컬 ROS 검증 완료. 실제 두 호스트 배포는 SSH timeout으로 실행하지 못했고 PTP 인터페이스도 아직 미설정이다. 원격 환경 설치, 양쪽 deployment manifest 일치, 실제 mode3 10분과 네 이동 구간 품질 검증은 미완료이며 성공으로 표시하지 않는다.

- `python3 integration/test_two_host.py`: 13개 stdlib 테스트 통과.
- `python integration/test_two_host_map.py`: 8개 동기/lease/strict calibration 테스트 통과.
- `python integration/test_two_host_realtime.py`: 4개 config/failure/acceptance 테스트 통과.
- 기존 pose/SHM/input/shutdown: 36개 + 20 subtest 통과.
- 실제 Jazzy bridge 양방향 CDR/네 topic graph/map change/종료 통과.
- 실제 receiver drain과 native RGBD bag 변환/재생 테스트 통과.
- ORB C++ current_map_id + PreSave regression, native/error/QoS ROS 실행 통과.
- Receiver/inference/fusion/viewer guard와 mask/depth quality self-check 통과.

리뷰의 patch 항목을 수정했다: 같은 grandmaster와 합산 시계 오차, SAM 환경 probe, 표본 기반 acceptance, 모든 runtime child의 stdin lease, atomic no-replace transfer, 양쪽 manifest invalidation, calibration failure log 보존, 불완전 CAD coverage, pose publisher identity/중복 검사.
기존 ORB PreSave가 빈 현재 map을 bad로 바꾸어 종료가 멈추는 문제도 실제 재현 후 저장용 목록만 필터하도록 수정했다. 수정 전 실패하는 C++ 회귀 검사와 원래 SaveAtlas 설정의 ROS 정상 종료로 확인했다.

## Suggested Review Order

- 배포/설정/시계/동기화 계약:
  [two_host.py](../integration/two_host.py), [deploy_remote.py](../integration/deploy_remote.py)
- 맵 생성과 stdin lease:
  [two_host_map.py](../integration/two_host_map.py), [two_host_worker.py](../integration/two_host_worker.py)
- 실행과 topic 격리:
  [two_host_realtime.py](../integration/two_host_realtime.py), [two_host_bridge.py](../integration/two_host_bridge.py)
- 최종 결과 gate:
  [two_host_guard.py](../sam6d_ws/realtime/two_host_guard.py), [two_host_quality.py](../sam6d_ws/realtime/two_host_quality.py)
- 재현 명령과 현장 연결 상태:
  [TWO_HOST.md](../integration/TWO_HOST.md)
