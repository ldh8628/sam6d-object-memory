---
title: 두 카메라 무인 입력 검증
status: done
baseline_commit: NO_VCS
context: []
---

## Intent

2026-09-07 23:35 사용자 지시: 오전 7시까지 사용자 응답 없이 두 카메라 입력 완전성을 최우선으로 개선/검증한다. 카메라 장면이 불량하여 맵 생성/localization 성공은 요구하지 않는다. 실제 파일은 integration/create_map_urdf.py 및 integration/run_object_memory_realtime.py이다.

## Constraints

기존 /tmp/rgbd_input_validation 후보를 계속 사용. SLAM 253822302376, SAM 253822301680. profile/FPS 유지, 입력 손실 덮어쓰기/복제 금지. 녹화/변환/맵 생성/pose 정확도 개선은 후순위. 원래 동작은 유지하고 입력만 검증하는 명시적 모드는 finite --input-check-seconds와 함께 사용. 사용자 선택창을 띄우지 않는 재현 명령과 코드 경로 제공.

## Bounded implementation task

- [x] create_map_urdf.py에 --input-check-only 옵션: 카메라/공통 입력 검사 경로를 실제 사용하되 raw bag/SQLite/지도 생성은 생략. --input-check-seconds > 0 필수. 입력 검사 PASS/FAIL/INCOMPLETE 결과와 serial 역할을 저장하고 종료.
- [x] --input-check-only 실행에서 serial 인자는 기존 확정 serial 두 개를 명시적으로 전달하는 재현 명령 사용. 추가 자동 선택 정책이나 가짜 map/pose를 만들지 않음.
- [x] 기본 실행/--record 의미를 보존하는 최소 self-test 추가/실행.

## Ownership

위 구현은 integration/create_map_urdf.py만 수정한다. 공통 input_check/input_integrity, camera_publish, SAM receiver, ORB, realtime launcher는 root가 별도 처리한다. 카메라 실행/장치 열거/하드웨어 reset/실제 하드웨어 검증은 root만 수행한다.

## Root follow-through

공통 Fast DDS XML에 64 MiB SHM transport 추가; 카메라 SIGTERM 종료 여유 25초; realtime은 map camera_roles.json의 검증된 역할을 자동 사용한다. 두 카메라 기준 측정 및 ORB/SAM 전체 실행에서 120초 PASS를 확인했다. 네 부하 조건과 실제 프로젝트 적용 후 기본 촬영/300초 실시간 검증을 완료했다. 원래 네 조건의 녹화 검증을 생략하여 production 합격으로 일반화하지 않는다.

## Completion scope

최우선 두 카메라 입력은 실제 프로젝트에 적용·검증했다. 기본 지도 촬영 각 899/899, realtime 300초 각 8,989/8,989 및 네 120초 조건 PASS. 선택 ORB map viewer의 한 차례 SIGSEGV는 원인 미확정이며, 추가 GDB 검사는 전 프레임을 소비했으나 계산 큐 지연으로 FAIL이었다. 이 맵 단계 예외와 pose 정확도는 입력 경로 완료 판정과 구별한다. 상세 증거는 ../README.md 및 ../FINAL_RESULT.json 참조.

## Suggested Review Order

- 저장된 역할과 자동 기준 측정으로 무인 시작한다.
  [run_object_memory_realtime.py:171](../../integration/run_object_memory_realtime.py#L171)

- 맵 불가 장면에서도 실제 입력 경로를 검사한다.
  [create_map_urdf.py:240](../../integration/create_map_urdf.py#L240)

- 측정 창과 종료·기준 측정의 증거를 보존한다.
  [input_check.py:19](../../integration/input_check.py#L19)

- 첫·마지막 프레임 누락까지 실패로 판정한다.
  [input_integrity.py:172](../../integration/input_integrity.py#L172)

- 큰 RGBD 메시지에 맞는 DDS 공유메모리를 사용한다.
  [fastdds_input.xml:1](../../integration/fastdds_input.xml#L1)

- 두 카메라를 함께 종료하고 대기 시간을 보장한다.
  [camera_publish.py:338](../../integration/camera_publish.py#L338)

- 자동 검사와 재현 명령을 확인한다.
  [README.md:267](../../integration/README.md#L267)

