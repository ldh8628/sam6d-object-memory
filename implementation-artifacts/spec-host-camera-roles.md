---
title: 연결된 노트북 기준으로 카메라 역할 자동 선택
type: bugfix
created: 2026-09-08
status: in-review
baseline_commit: d32b82b2a4e891c799b89f885fbb74a94eba7541
---

<frozen-after-approval reason="사용자가 기존 고정 serial 계획을 수정하고 호스트 기준 자동 인식을 명시적으로 요청">

## Intent

카메라 serial을 장비별 상수로 고정하지 않는다. SAM 노트북에 연결된 카메라는 SAM,
원격 SLAM 노트북에 연결된 카메라는 SLAM으로 실행마다 자동 선택한다.
기존 분산 맵 생성·전송·calibration을 재사용한다.

## Boundaries & Constraints

Always: 실제 장치 조회는 카메라 스트림 시작 전에 각 호스트에서 수행한다.
선택된 serial을 worker DeviceInfo로 검증하고 세션·역할 보고서에 저장한다.
원격 master 1 / 로컬 slave 3, 기존 품질·PTP·원본 보존 기준은 유지한다.
기본값은 auto이며 여러 장치의 모호함은 임의로 결정하지 않는다.

Never: 특정 serial을 역할 상수로 고정, 카메라 연결 교환 요구, 서로 다른 카메라의 기존 보정 결과 자동 재사용.

## I/O & Edge-Case Matrix

| 상황 | 결과 |
|---|---|
| 각 호스트 한 대 | local→SAM, remote→SLAM, 실제 serial 기록 |
| 카메라 교체/교환 후 새 세션 | 새로 연결된 serial로 역할 자동 선택 |
| 0대 또는 여러 대 auto | 촬영 전 오류; 여러 대는 명시적 serial 선택 가능 |
| 명시적 serial 미연결, 중복 serial, 조회 실패 | worker 시작 전 오류 |
| 기존 맵 재사용 | 현재 실측 역할/serial이 보정 당시와 일치해야 통과 |

</frozen-after-approval>

## Code Map

- `integration/two_host.py`: auto 설정, 기존 serial 파서 재사용, 호스트별 조회·역할 확정.
- `integration/two_host_map.py`: worker 시작 전 조회 및 실제 serial 보고서 전파.
- `integration/two_host_realtime.py`: 자동 선택 후 기존 맵 계약 비교.

## Tasks & Acceptance

- [x] 설정의 serial 기본값을 auto로 변경하고 기존 명시적 선택 호환.
- [x] 호스트별 실제 장치 조회, worker/보고서에 실제 serial 전달.
- [x] 기존 맵 계약 검사를 자동 선택 이후 적용.
- [x] 자동 선택/교환/모호함/실패/기존 맵 재사용 회귀 검사.
- [x] 로컬 설정과 배포 예제/운영 문서를 자동 선택으로 변경.
- [ ] commit/push/양쪽 배포와 실제 연결 자동 인식 검증.

Given 각 호스트 한 대, when 새 세션 실행, then serial 값과 관계없이 호스트 기준 역할을 선택한다.
Given 카메라 교체, when 기존 맵 재사용, then 기존 보정과 다르면 재보정을 요구한다.

## Verification

`test_two_host_cameras.py`, `test_two_host.py`, `test_create_map_urdf_split.py`,
`test_two_host_map.py`, `test_two_host_realtime.py`, 기존 맵 self-test.
기존 deploy_remote 명령과 실제 호스트 조회로 배포 SHA/선택된 카메라를 확인한다.
PTP/공간으로 막힌 10분 녹화와 맵·URDF 회수는 자동 인식 검증과 구분한다.

## Suggested Review Order

- 호스트별 조회와 auto 설정: [two_host.py](../integration/two_host.py)
- 맵 실행에 실제 serial 전달: [two_host_map.py](../integration/two_host_map.py)
- 이전 보정 계약 유지: [two_host_realtime.py](../integration/two_host_realtime.py)
- 카메라 교환과 오류 검증: [test_two_host_cameras.py](../integration/test_two_host_cameras.py)

## Spec Change Log

이전 split-map 계획의 고정 serial/연결 교환 요구는 이번 사용자 지시로 대체됐다.
