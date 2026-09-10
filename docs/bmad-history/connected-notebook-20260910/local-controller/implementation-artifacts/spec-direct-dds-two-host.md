---
title: 두 노트북 실시간 pose의 직접 DDS 전송
type: feature
created: 2026-09-09
status: done
baseline_commit: b581b2d8c7b89f9434096a37d4845c6e556009a1
context: []
---

<frozen-after-approval reason="사용자가 양쪽 노트북 변경과 배포를 명시적으로 요청함">

## Intent

**Problem:** 실시간·녹화 재생 split 실행은 ROS 메시지를 SSH로 전달하고 다시 발행한다. 사용자는 원격 ORB publisher와 로컬 SAM subscriber의 유선 ROS2 DDS 직접 통신을 요청했다.

**Approach:** 카메라의 기존 SHM/localhost 환경을 보존하며 ORB와 SAM 처리 프로세스만 같은 domain에서 유선 DDS에 참여시킨다. SSH 브리지를 구독 전용 상태 감시로 대체한다. 양쪽 노트북에 동일 소스를 배포하고 실제 합성 메시지로 검증한다.

## Boundaries & Constraints

**Always:** 기존 수정·지도·녹화 보존. Wi-Fi와 PTP IPv6 유지. 유선 IPv4 주소와 직접 연결 검증. 카메라 영상은 호스트 내부에서 처리. map ID 변경, 중복 발행자, 상태 타임아웃, 감시 프로세스 단절 시 기존 fail-closed 동작 유지. 지연 측정에서 계산된 값과 미측정 값을 구분.

**Ask First:** 관리자 인증이 필요한 시스템 변경은 사용자의 인증을 요청한다.

**Never:** 카메라 재보정이나 물리 이동 검증을 합성 DDS 테스트로 대체하지 않는다. SSH 전송으로 조용히 fallback하지 않는다. 기존 데이터 삭제, 전체 작업 트리 덮어쓰기, 요청 없는 Git push를 수행하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|---|---|---|---|
| 정상 | 동일 domain, 유선 주소 준비 | 원격 원본 publisher의 네 토픽을 로컬에서 수신 | 통계 저장 |
| 카메라 격리 | private camera publisher | 로컬 처리 노드만 영상 수신 | LAN 영상 전달을 테스트에서 검사 |
| 주소 오류 | 유선 주소 미설정 또는 Wi-Fi 경로 | 카메라 시작 전 실패 | 구체적인 인터페이스·주소 오류 |
| map 변경 | 초기 ID 이후 다른 ID | unhealthy | 기존 fusion guard 닫힘 |
| 끊김 | 원격 상태 발행 중단 | 2초 lease 이후 unhealthy | 정상 pose로 위장하지 않음 |
| 중복 | 같은 토픽의 추가 publisher | unhealthy | 실행 실패 및 원인 기록 |

</frozen-after-approval>

## Code Map

- `integration/two_host.py`: 환경·SSH·파일 전송·프로세스 lease.
- `integration/two_host_bridge.py`: 기존 토픽 QoS 및 BridgeHealth 재사용.
- `integration/two_host_worker.py`: ORB 자식 프로세스에만 DDS 환경 설정.
- `integration/two_host_realtime.py`, `integration/run_object_memory_replay_split.py`: 실행 연결 및 감시.
- `integration/two_host_dds.py`: 직접 DDS 환경, 유선 probe, 구독 전용 감시.
- `integration/test_two_host_dds.py`: 환경 및 실제 두 호스트 검증.

## Tasks & Acceptance

**Execution:**
- [x] DDS 환경·유선 검사·감시를 구현하고 기존 ROS QoS/상태 검사를 재사용.
- [x] live/replay의 ORB·SAM을 직접 연결하고 SSH 데이터 브리지 제거.
- [x] 양쪽 전용 프로필과 설정에 IPv4 /30 주소 적용, 소스 백업·해시 검증 후 배포.
- [x] 오프라인 회귀 및 양쪽 합성 DDS 테스트, 가능한 녹화 재생 검증 수행.
- [x] 실행 명령과 실제 검증 결과를 문서에 기록.

**Acceptance Criteria:**
- Given 양쪽 주소가 준비됨, when split 실시간 경로를 실행, then ORB 원본 publisher가 SAM subscriber에 DDS로 직접 전달하고 SSH는 실행 제어·파일 전송에만 사용된다.
- Given 카메라 없는 합성 source, when 반대 호스트에서 수신, then pose 값·원본 publisher GID·late-join 상태·map 변경·상태 타임아웃·정상 종료를 검증한다.
- Given 배포 완료, when 양쪽 변경 파일의 SHA256을 비교, then 동일하며 기존 작업의 사본이 남는다.

## Design Notes

현재 base domain 72의 camera domain 73을 유지한다. 기존 lan=True는 다른 domain을 쓰므로 그대로 재사용할 수 없다. 새로운 직접 DDS 환경은 동일 domain에서 SHM과 유선 UDP 및 loopback 발견만 허용한다. 원격 ORB와 로컬 SAM의 기존 메시지 타입·QoS는 호환되므로 C++ 재빌드를 추가하지 않는다. 녹화 재생의 /clock은 각 호스트 내부에서 유지한다.

## Verification

- 기존 관련 Python unittest와 새로운 환경/상태 검사.
- 실제 두 노트북의 합성 DDS 전송 결과 JSON 및 처리 지연 분포.
- 배포 SHA256, 유선 route, 네트워크 프로필, PTP 상태 기록.
- 가능하면 기존 기록으로 전체 ORB/SAM 경로를 headless 검증. 물리 카메라 정확도는 별도 검증 대상.

## Review & Results

직접 DDS 및 전체 재생 PASS. 세부 결과는 [RESULTS.md](../output/dds_direct_20260909/RESULTS.md).
SAM 발행자 검사를 수신/프레임/결과 경계에 추가했다. 카메라 strict loopback 설정 및 ObjectMemory 원격 map ID 직접 구독을 실제 양쪽 실행으로 검증했다. 기존 객체 정렬 품질 FAIL과 물리 이동 미측정은 통신 PASS와 구분한다.

## Suggested Review Order

1. live/replay 실행 연결: [two_host_realtime.py](../integration/two_host_realtime.py#L211), [replay_split.py](../integration/run_object_memory_replay_split.py#L125).
2. 유선 프로필 및 구독 전용 감시: [two_host_dds.py](../integration/two_host_dds.py#L35), [private XML](../integration/fastdds_input.xml#L1).
3. pose 수신·결과 경계 보호: [sam6d_receiver_node.py](../sam6d_ws/realtime/sam6d_receiver_node.py#L316).
4. 실제 두 호스트 검증: [test_two_host_dds.py](../integration/test_two_host_dds.py#L103).
