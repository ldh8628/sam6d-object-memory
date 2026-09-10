---
title: 원격 지도 시각화와 CUDA ORB descriptor 실행
type: feature
created: 2026-09-09
status: done
baseline_commit: b581b2d8c7b89f9434096a37d4845c6e556009a1
context: []
---

<frozen-after-approval reason="사용자가 요청한 구현 범위">

## Intent

**Problem:** split 지도 생성은 원격 노트북에 화면이 없어 촬영 중 추적 품질을 확인하기 어렵다. 원격 RTX 5090 Laptop의 CUDA descriptor 구현은 실험 명령에서만 활성화되고 octave마다 전송하여 기존 측정에서 CPU보다 느렸다.

**Approach:** 지도 생성의 원격 live preview 및 후처리 viewer를 기본으로 활성화하고 headless 선택을 제공한다. split 지도/실시간 경로가 명시적으로 CUDA descriptor backend를 사용하도록 연결한다. 기존 CPU 알고리즘과 descriptor를 보존하면서 octave별 GPU 전송과 kernel 호출을 프레임 단위로 묶어 비용을 줄이고 원격에서 실측한다.

## Boundaries & Constraints

**Always:** 기존 작업트리 수정과 녹화/Atlas를 보존한다. 기존 입력·동기화·보정·tracking 검증은 유지한다. CUDA 오류를 CPU 성공으로 숨기지 않는다. GPU 사용과 전체 SLAM 가속을 구분하고 측정 근거를 기록한다. 원격 desktop의 기존 사용자 세션에 창을 띄우며 X 접근 제어를 해제하지 않는다.

**Ask First:** 기존 사용자 실행을 중단하거나 기존 지도·녹화 삭제가 필요한 경우.

**Never:** GPU를 사용한다는 이유만으로 정확도 개선을 주장하거나 특징점·초기화 문턱을 낮추지 않는다. 이번 변경은 전체 SLAM 최적화의 CUDA 재작성 범위가 아니다.

## I/O & Edge-Case Matrix

| 상황 | 입력 | 기대 동작 | 오류 처리 |
|---|---|---|---|
| 신규 지도 | split 기본 옵션 | 원격 촬영 preview와 보정 재생 viewer, CUDA 실행 | 실패 원인 명시 |
| 무화면 실행 | --headless | GUI 없이 기존 처리 수행 | desktop 불필요 |
| 입력 검사만 | --input-check-only 또는 --observe-only | 추가 ORB preview 없이 기존 입력 검사 | 기존 품질 정책 유지 |
| 기존 녹화 | --from-capture / --resume | 후처리 viewer/backend 옵션 전달 | 원본 보존 |
| GPU 불가 | CUDA 라이브러리/장치 오류 | 시작 실패 | CPU 자동 대체 금지 |
| CPU 비교 | --orb-backend cpu | 원래 descriptor 계산 | CUDA 장치 불필요 |
| remote desktop 없음 | 기본 GUI 요청 | 해결 방법과 --headless 안내 | 무화면 성공으로 숨기지 않음 |
</frozen-after-approval>

## Code Map

- `integration/create_map_urdf_split.py`, `create_map_urdf.py`, `two_host_map.py`, `two_host_worker.py`: CLI, 녹화 preview, 원격 후처리 전달.
- `integration/camera_extrinsic_localization.py`: 동일 지도에서 양쪽 녹화 ORB 실행 설정.
- `integration/run_object_memory_realtime_split.py`, `run_object_memory_realtime.py`, `two_host_realtime.py`, `run_object_memory_replay_split.py`: 원격 localization backend 전달.
- `orbslam_ws/src/orbslam3_ros2/launch/orb_slam.launch.py`: ORB 노드의 명시적 backend 및 GUI 환경.
- `orbslam_ws/src/ORB_SLAM3/src/ORBextractor.cc`, `OrbCudaDescriptors.cu`: CPU descriptor 및 선택적 CUDA kernel.
- `orbslam_ws/src/ORB_SLAM3/tests/check_cuda_descriptors.cc`: descriptor 일치/feature extraction 시간 검사.
- `orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp`: 반복 종료 신호 중 Atlas/궤적 저장 보호.
- `integration/run_orb_slam_gpu.py`: 같은 녹화 입력의 전체 SLAM 비교.

## Tasks & Acceptance

**Execution:**
- [x] 지도 viewer의 기본 활성화/headless 전달, 원격 desktop 연결, 입력 검사 경로 제외.
- [x] split 지도 및 실시간/replay의 CUDA 기본 실행과 명시적 CPU 선택, 실패 검사.
- [x] octave GPU 호출을 batch하여 기존 descriptor·keypoint·scatter 순서 보존.
- [x] 기존 관련 Python 검사 및 확장 descriptor 검사 실행.
- [x] 원격 파일 차이를 보존한 배포/빌드, GUI smoke test, CPU/CUDA 같은 입력 실측 및 결과 문서화.

**Acceptance Criteria:**
- Given 원격 desktop 세션과 정상 입력, when split 지도 생성, then 원격 ORB viewer에 영상 특징점과 지도/카메라 추적이 표시된다.
- Given 동일 이미지/ORB 설정, when CPU와 CUDA 추출, then 모든 descriptor 바이트와 keypoint 위치·각도·octave가 일치한다.
- Given 같은 녹화 구간, when CPU/CUDA 비교, then 입력 timestamp와 처리 수/추적 coverage를 확인한 속도 결과를 남긴다.
- Given CUDA split 실행, when ORB가 입력을 처리, then 실제 CUDA backend 실행 로그가 존재한다.

## Verification

- Python unittest: split entrypoint, map worker/resume, realtime, launch/runtime 옵션과 실패 처리.
- 원격 `check_cuda_descriptors`: 합성·실제 영상의 bit-exact 검사 및 반복 timing.
- 원격 `run_orb_slam_gpu.py --compare`: 기존 밝은 녹화의 같은 구간 비교.
- 원격 desktop에서 ORB viewer 실행과 창 존재 확인; 자체 시험 프로세스 종료 확인.

## Spec Change Log

- 검토: 명시하지 않은 launch backend는 기존 환경변수 선택을 보존한다. XAUTHORITY 미설정은 기본 X11 인증을 실제 접속으로 검사한다.
- 검토: 이미 통과한 calibration의 export 재개는 GPU/desktop probe를 요구하지 않는다. Preview는 기존 domain ORB가 있으면 카메라를 열기 전에 거부한다.
- 검토: CUDA attestation은 라이브러리 로드가 아닌 첫 descriptor 계산 성공 뒤에만 출력한다.
- 실제 GUI 재시험: 중복 SIGINT가 ROS handler 해제 후 ORB 저장을 중단하는 문제를 재현했다. node를 먼저 정리하고 rclcpp::shutdown을 마지막에 호출해 Atlas/궤적 저장을 보호한다. 기존 원본과 실패 시험 로그는 보존한다.

## Design Notes

ORB-SLAM3 전체가 GPU로 옮겨지는 것은 아니다. detection/orientation/blur/matching/BA는 기존 CPU 동작을 유지한다. GPU batching은 CPU 특징점/descriptor 계약을 보존하는 성능 변경이며 실제 이동 정확도에는 별도 기준 궤적이 필요하다.

## Verification Results

- Python unittest 59개, 자체 검사 3개 PASS.
- RTX 5090 Laptop에서 descriptor 12case bit-exact PASS, 첫 CUDA 호출 실패 주입 검사 PASS.
- 실제 원격 desktop GUI 및 600pose 저장/정상 종료 2회 PASS.
- CPU/CUDA/역순 녹화의 입력 899개 모두 일치/추적 PASS. Whole-SLAM 가속 및 정확도 향상은 입증하지 못했다.
- 원격 소스 14파일 및 설치 launch hash 검증 PASS. 원격 core/node 빌드·설치 완료.
- [상세 검증 기록](../output/remote_viewer_cuda_1788920426738/summary.md)

## Suggested Review Order

- 기본 시각화와 backend 선택 진입점
  [create_map_urdf_split.py:18](../integration/create_map_urdf_split.py#L18)
- ORB 전용 CUDA/desktop 환경과 실패 처리
  [orb_runtime.py:33](../integration/orb_runtime.py#L33)
- 촬영 preview와 입력 소비자 종료 처리
  [two_host_worker.py:128](../integration/two_host_worker.py#L128)
- 원본 특징점 보존 및 프레임 단위 GPU 호출
  [ORBextractor.cc:1168](../orbslam_ws/src/ORB_SLAM3/src/ORBextractor.cc#L1168)
- 저장 완료까지 종료 신호 처리 유지
  [rgbd_node.cpp:1687](../orbslam_ws/src/orbslam3_ros2/src/rgbd_node.cpp#L1687)
- CLI/desktop/CUDA/기존 ORB 보호 검증
  [test_orb_runtime.py:18](../integration/test_orb_runtime.py#L18)
