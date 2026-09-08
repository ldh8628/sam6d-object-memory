# 두 카메라 입력 검증 — 2026-09-08

기존 `REPORT.md`는 두 번째 카메라 연결 전의 실패 이력을 보존한 문서다. 이 문서는 두 카메라 연결 후의 수정과 검증을 기록한다. 실제 프로젝트에 입력 변경을 적용했고 기본 촬영 및 실시간 경로를 다시 검증했다. 선택 ORB map viewer의 별도 예외는 아래에 구분해 보존한다.

## 확인한 원인

드라이버 `frame_callback` 진입/종료, `rcl_publish` 진입/종료, 최소 C++/Python 수신기를 같은 호스트의 monotonic clock으로 계측했다. 프레임 원본 stamp로 단계들을 연결했으며 센서 시각을 호스트 시각에서 빼지 않았다.

| 같은 30초 구간 | 기존 SHM 설정 | SHM 64 MiB |
|---|---:|---:|
| 드라이버 publish 간격 p95 | 36.80 ms | 36.99 ms |
| C++ 수신 간격 p95 | 56.25 ms | 37.10 ms |
| Python 수신 간격 p95 | 56.50 ms | 37.74 ms |
| publish → C++ RGBD 수신 p95 | 26.91 ms | 1.90 ms |
| publish → C++ metadata 수신 p95 | 약 2.8초 | 약 1.8 ms |

처음 지연이 증가한 경계는 드라이버 발행 이후 DDS 전달이다. 약 1.5 MiB인 RGBD 메시지에 비해 기본 Fast DDS 공유메모리 segment는 약 0.5 MiB였다. 64 MiB 설정에서 전달 지연과 metadata 손실이 해소되는 A/B 결과를 확인했다. 정확한 내부 buffer overwrite 동작까지 직접 관찰했다는 뜻은 아니다.

최종 코드는 기존 `rmw_fastrtps_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`를 유지하고 공통 XML에 64 MiB SHM transport를 추가한다. 두 RGBD 및 네 metadata publisher history 120, reliable receiver depth 120, 애플리케이션 FIFO 60을 사용한다. RGB 640×480×30, depth 848×480×30, aligned depth 640×480을 유지했고 하드웨어 sync는 케이블 확인이 없어 사용하지 않았다.

근거: [Fast DDS 큰 데이터 전송](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/use_cases/large_data/large_data.html), [SHM segment 설정](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html), [설치 RMW의 LOCALHOST transport 처리](https://raw.githubusercontent.com/ros2/rmw_fastrtps/8.4.3/rmw_fastrtps_shared_cpp/src/participant.cpp). 계측 소스와 원본 로그: `/tmp/camera_input_cause_20260907`.

## 실행 경로

- `integration/create_map_urdf.py`가 실제 파일명이다. `--input-check-only --input-check-seconds N`으로 맵을 만들 수 없는 장면에서도 실제 카메라 수신 경로를 검사할 수 있다. 무인 검사 명령에는 확정된 두 serial을 지정한다.
- `integration/run_object_memory_realtime.py`는 지도 폴더의 `camera_roles.json`을 자동 사용한다. SLAM **253822302376**, SAM **253822301680**. 명시적 serial 인자가 있으면 기존 역할 검증을 유지한다.
- baseline 인자가 없으면 부하 시작 전에 두 카메라만으로 30초(+준비 10초) 기준 측정을 수행한다. baseline 실패/미완료는 계속 진행하지 않는다.
- ORB/SAM의 콜백은 bounded FIFO에 넣고 단일 작업자가 모든 입력을 순서대로 소비한다. 추론/compact viewer의 최신 프레임 선택은 원본 입력·MCAP 완전성 집계와 구별한다.
- 두 카메라의 종료를 동시에 시작하고 wrapper는 자식들의 종료 시간을 보장한다. 검사 구간의 첫/마지막 프레임 누락과 publisher QoS 변경도 검사한다.

## 검증의 한계

사용자가 안내한 대로 장면이 불량하여 localization은 실패했고 대부분 검출이 0개였다. ORB/SAM/모델은 실제로 실행했지만, 객체가 다수 검출되어 무거운 PEM 처리가 반복되는 조건이나 정상 TRACKING_OK 상태의 pose 정확도 합격을 의미하지 않는다. 실패 상태의 mask-tracker 변경은 포함하지 않는다. 입력 완전성은 아래의 유한 측정 구간에 대한 증거이며 모든 향후 환경의 무조건적인 무손실 보장은 아니다.

시작 시 일시적 USB control-transfer 경고는 원본 로그에 남겼다. 초기 진단 hook의 심볼 조회 오류, 후보의 모듈/template/cache 경로 누락으로 실패한 실행도 보존했고 성능 근거에서는 제외했다. 기존 실험 bag은 `preserved_bag_archives.json`의 SHA-256으로 검증한 `.zst`에 원본 그대로 보존했으며 복원 방법은 `RESTORE_OLD_BAGS.md`에 있다.

## 최종 검사 및 적용

최우선 입력 경로는 PASS. [최종 결과](../FINAL_RESULT.json), [실행 및 보존 위치](../README.md). 선택 map viewer의 맵 리셋 예외와 pose 정확도는 이 PASS에 포함하지 않는다.

## 네 부하 조건 — 각 120초

수량은 RGB metadata watermark로 닫은 각 카메라의 source 구간이다. 고정 host 창의 FPS는 별도로 JSON에 기록한다. 아래 간격·큐 수치는 monitor/ORB/SAM 전체 중 최댓값이다. 모든 조건에서 RGB/depth metadata 번호 누락·중복·역순, 소비 누락 및 큐 초과는 0이었다.

| 실행 | 녹화 | Viewer | SLAM 수신/소비 | SAM 수신/소비 | 수신 간격 p95/max(ms) | 큐 대기 p95/max(ms) | 판정 |
|---|---|---|---:|---:|---:|---:|---|
| [1](two_realtime_shm64_models/input_integrity.json) | 끔 | 끔 | 3596/3596 | 3596/3596 | 44.26/70.28 | 1.42/16.29 | PASS |
| [2](two_realtime_shm64_record_auto/input_integrity.json) | 켬 | 끔 | 3596/3596 | 3596/3596 | 43.69/98.82 | 1.05/38.90 | PASS |
| [3](two_realtime_shm64_view_auto/input_integrity.json) | 끔 | 켬 | 3596/3596 | 3595/3595 | 43.01/89.76 | 3.66/20.52 | PASS |
| [4](two_realtime_shm64_record_view/input_integrity.json) | 켬 | 켬 | 3595/3595 | 3596/3596 | 44.15/71.58 | 1.46/16.78 | PASS |

기준 측정: [create_map 입력 전용 120초](../candidate_output/unattended_input_shm64_120/input_integrity.json). 기본 인자의 자동 30초 기준 측정도 [실제 실행](two_realtime_shm64_record_auto/input_baseline/input_integrity.json)에서 PASS했다. 첫 세 하드웨어 결과와 녹화 결과는 판정 경계 보강 후에도 [재판정 PASS](review_reassessment.json)였고 원본 보고서를 덮어쓰지 않았다.

재생: 녹화/Viewer 끔 실행의 전체 bag은 SLAM 3,976 / SAM 3,926쌍을 [native](two_realtime_shm64_record_auto/native_replay/replay_integrity.json)와 [SQLite 8개 토픽](two_realtime_shm64_record_auto/sqlite_replay/replay_integrity.json)에서 전부 같은 내용·순서로 수신했다. [변환](two_realtime_shm64_record_auto/compatibility_sqlite/conversion_integrity.json)은 모든 필드와 bag timestamp를 보존했다. 녹화/Viewer 켬의 전체 bag도 SLAM 3,921 / SAM 3,945쌍 [native 재생 PASS](two_realtime_shm64_record_view/native_replay/replay_integrity.json)다. 준비·종료 시 카메라별 실제 발행 구간이 달라 전체 bag 수량은 서로 다르며, 이를 누락으로 해석하지 않는다.

Viewer는 Wayland Qt backend를 사용했다. X11 전용 xwininfo에 창이 나타나지 않는 것을 viewer 미실행으로 판정하지 않았다. 실제 viewer 프로세스·Qt Wayland plugin 및 종료까지의 실행 로그를 확인했다.

## 실제 프로젝트 적용 후 확인

기존 입력 소스 및 ORB 설치본을 `/home/etri/sam6d_object_memory/input_validation_20260908/before_input_changes.tar.gz`에 백업하고 27개 입력 소스 파일을 적용했다. 기존의 다른 사용자 변경은 그대로 보존했다. 원래 `orbslam_ws/install_jazzy`에 core/wrapper 두 패키지를 재빌드했고, 실제 라이브러리 경로가 원래 프로젝트를 가리키는 것을 확인했다. 소스 SHA manifest와 빌드·검사 로그는 위 폴더에 있다.

실제 `create_map_urdf._record` 기본 촬영 경로에서 자동 baseline 후 30초 측정: 각 카메라 **899/899쌍**, 원본 bag과 일치, PASS. 준비/종료 포함 전체 bag은 SLAM 1,223 / SAM 1,225쌍이며 SQLite 네 토픽씩 모든 필드·내용·순서·timestamp 일치로 변환했다. 결과는 `output/input_capture_validation_20260908_plain`에 보존했다. 장면 불량으로 offline 맵 생성 단계는 실행하지 않았다.

선택 `--map-viewer`의 별도 시도에서는 반복 초기화/reset 도중 ORB가 SIGSEGV로 종료했다. 해당 실행은 `output/input_capture_validation_20260908`의 INCOMPLETE로 보존하고 조사 중이다. 기본 촬영 PASS나 SAM viewer의 네 조건 PASS로 이 실패를 가리지 않는다.

## 실제 설치본 300초 실시간 검증

`ORB_INSTALL` 및 debug prefix를 설정하지 않고 원래 프로젝트의 realtime 진입점을 실행했다. 저장된 역할 자동 지정 → 자동 30초 camera-only baseline → ORB/SAM/ObjectMemory/Viewer → 300초 측정 → 수신/소비 drain까지 **PASS**했다. 두 카메라 각각 **8,989/8,989프레임**, frame number 누락·중복·순서 오류 0, host FPS **29.9633**. 전체 단계 중 host 간격 p95 최대 **42.129ms**, 최대 **80.058ms**; 큐 대기 p95 최대 **2.604ms**, 최대 **30.239ms**; callback p95 최대 **0.006223ms**였다. [원본 결과](../realtime_300/input_integrity.json).

선택 map viewer의 첫 SIGSEGV 원인은 확정하지 못했다. 후보/실제 설치본 각각 1,223프레임의 offline replay, 실제 카메라 GDB 120초에서는 종료를 재현하지 못했다. live GDB의 전체 ORB 입력 3,909개는 모두 소비했고, 측정 구간도 SLAM 3,595 / SAM 3,596개가 전부 수신·소비됐다. 다만 반복 reset 중 ORB 큐 p95가 26.997ms라 해당 진단은 **FAIL**로 보존했다. GDB에 의한 영향과 맵 계산 자체의 지연을 분리하지 않았으며, 이를 정상 map viewer 성능 합격으로 바꾸지 않았다. 추측에 따른 tracker/core 수정은 하지 않았다. [진단 요약](../map_viewer_diagnostic_summary.json).

최종 카메라/ORB/SAM 실행은 종료했다. 여유 공간은 약 16 GiB이며, 추가 녹화/호환 변환은 용량 사전 검사에 필요한 공간을 확보해야 한다. 기존 사용자 데이터는 삭제하지 않았다.
