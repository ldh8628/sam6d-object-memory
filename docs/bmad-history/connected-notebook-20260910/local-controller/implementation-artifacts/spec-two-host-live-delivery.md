---
title: 두 호스트 지도·실시간 오버레이 및 동기화/GPU/IMU 실험
created: 2026-09-09
status: in-review
baseline_commit: b581b2d8c7b89f9434096a37d4845c6e556009a1
context: []
---

<frozen-after-approval reason="사용자가 07시까지 전체 구현·실행을 명시적으로 위임함">
## Intent
사용자가 지정한 여섯 작업을 모두 수행한다. 현재 두 노트북 각각 D455f 한 대, 9핀 연결, LAN 및 RTX5090 환경에서 정지/비동기/외부동기 촬영을 비교하고, 기존 실시간 파이프라인을 재사용하는 split 진입점을 구현한다. 카메라 고정 프레임 장착 후 지도 생성, 저장 지도 localization, SAM 영상에 지도 객체 자세 오버레이를 실행하도록 준비한다. 별도로 GPU SLAM 비교, IMU 회전 보정, 객체 랜드마크 기반 위치 보정 실험 경로를 제공한다.

## Boundaries & Constraints
Always: 기존 작업 보존, 실행 가능한 코드와 측정 결과 제공, 카메라·맵 좌표 및 serial 검증, GPU 사용 사실 측정, 시간 정합 오류 및 tracking lost를 정상 pose로 위장하지 않음. 현재 카메라를 움직일 수 없으므로 정지 검증과 이동 검증을 구분. PTP 시험 한계는 기존 5ms이며 설정 가능한 상태 유지. 대용량 추가 녹화는 최소화.
Never: CPU ORB를 GPU라고 표기, 물리적 보정 없이 올바른 rig extrinsic이라고 주장, 정지 장면의 성공을 이동/회전 정확도 검증으로 주장, 사용자 데이터 임의 삭제.

## I/O & Edge-Case Matrix
| 상황 | 입력 | 결과 |
|---|---|---|
| 같은 조건 비교 | 카메라 정지, mode0/0, mode1/3 | 동일 시간 PTP 전체 로그·입력 지표, 실측 비교 |
| Split realtime | 저장 atlas, 객체 등록, 두 serial, 보정 URDF | remote localization pose, local SAM 영상 오버레이 |
| 추적 실패/시계 이상 | LOST 또는 lease expiry | overlay/fusion 차단과 명시적 상태 |
| GPU 비교 | 동일 입력 CPU/GPU | 실제 장치·단계별/전체 시간 및 품질 비교 |
| IMU | timestamp·extrinsic·noise 검증 | 원시 IMU/회전 비교와 보정 실험, 미초기화 표시 |
| 객체 보정 | 신뢰 가능한 대응 landmark | robust residual 기반 보정 후보, 부족/퇴화 거부 |
</frozen-after-approval>

## Code Map
- integration/two_host*.py: 설정, 카메라 worker, map, pose transport와 realtime orchestration.
- integration/run_object_memory_realtime.py: 기존 로컬 실행 및 split 분기.
- orbslam_ws/src: ORB-SLAM3 C++ 및 ROS Python wrapper.
- sam6d_ws/realtime: SAM 추론, pose matching, overlay.
- objectmemory_ws/object_memory: 객체 landmark 저장과 추정.

## Tasks & Acceptance
- [x] 동일 조건 세 단계 실행 및 로그 분석 결과 저장.
- [x] integration/run_object_memory_realtime_split.py 공통 실행 경로 수정·테스트 및 두 호스트 GUI 재생 완료.
- [x] 정지 지도/URDF PASS 및 새 Atlas 밝은 기록 localization PASS, 현재 실제 카메라 실행·정리 검증, 사용자 명령 제공.
- [ ] 현재 실제 카메라의 성공적인 live localization/객체 등록: 거의 검은 영상에서0pose로 실패. 조명/시야 복구 후 필요.
- [x] 별도 GPU 실행 및 CPU/GPU 비교 도구 실제 검증(혼합 CUDA 단계, 속도 향상 없음).
- [x] 별도 IMU/GPU 실행과 실측 검증; 실제 운동 초기화·정확도는 미검증.
- [x] 객체 대응 보정 합성·실제 저장 관측 실행; 진단 후보만 산출.

Acceptance: Given 두 호스트 연결, when comparison 실행, then 같은 조건의 단계별 실측 통계가 남는다. Given 유효 atlas·객체맵·rig 보정, when split realtime 실행, then 영상에 지도 객체 pose가 표시되고 프레임 정합과 localization 지표가 남는다. Given GPU 또는 IMU 실험, when 실행, then 지원 여부 및 실제 처리 장치/초기화/성능이 정직하게 기록된다. Given 잘못된 map/serial/pose, when 실행, then 데이터 오염 없이 중단 또는 미추적 상태를 표시한다.

## Verification
관련 기존 unittest, 새 진입점과 수학 로직 회귀 검증, 양쪽 실제 실행, GPU 프로파일 및 동일 입력 비교, 결과 JSON/스크린샷/사용자 실행 안내. 이동·회전 실험은 사용자 부재와 고정 카메라로 수행할 수 없어 별도 명시.


## Review and Runtime Findings

Diff-only blind review followed by current-code edge/acceptance review found and repaired:

- Interrupted raw-copy, conversion-marker corruption, successful-calibration/export resume gaps. Files are copied atomically, cross-filesystem donors fall back to copying, existing conversions are byte-verified, and completed calibration can finish exporting without recalibration. Local map/resume15 tests and remote resume5 tests passed.
- Relative Atlas paths in the standalone GPU runner, IMU live RGB intrinsics, stale/short IMU sample windows. Actual CUDA saved-atlas localization and live IMU startup were rerun after fixes.
- Replay/live camera-model mismatch and foreign-frame landmarks. Added serial/profile/distortion checks and explicit0.25px maximum same-ray displacement, retaining saved Atlas K. Pose/frame failures remain rejected.
- ROS image subscription API mismatch, temporary Atlas identity publication, shutdown signal ownership, and queued-frame pose-wait accounting surfaced during actual two-host execution.

The review tool rejected additional reviewer threads at its concurrency limit. The context-free blind reviewer continued with project access for the remaining edge/acceptance perspectives; the user-authorized implementation was not halted for an internal workflow limit.

## Runtime Evidence

- PTP: `output/sync_compare_20260909_0032_clean/comparison.json`.
- Full two-host replay: `output/260904_test_01/object_memory/replay_split_20260909_013826/validation.md`; GUI clean exit0,2595matchedframes,835objectprojectionframes,inputfailures0.
- Physical-object recorded overlay: `output/260904_test_02/object_memory/realtime_20260904_153542/recorded_overlay_20260909_012700/report.json`; best4-object frame PASS, full recording alignment FAIL.
- GPU: `output/gpu_native_comparison_20260909/summary.md`; same1799 inputs, both1799poses; CUDA descriptor median5.81%slower.
- IMU: `output/readiness/imu_object_summary_20260909.json`;200Hz input and actual CUDA rotation validated, stationary inertial initialization NOT_INITIALIZED.
- Object correction: `output/readiness/object_correction_recorded_20260909/summary.json`;4real recorded candidates, correlated map/prior, no ground-truth claim.
- Map: user transient service `two-host-map-static-20260909.service` avoids terminal execution lifetime; original raw capture preserved.

## Suggested Review Order

- Two-host entry point and executable options.
  [run_object_memory_realtime_split.py:13](../integration/run_object_memory_realtime_split.py#L13)
- Calibrated camera and saved-map frame contracts.
  [calibrated_camera.py:6](../integration/calibrated_camera.py#L6)
- Pose transport, leases and process ownership.
  [two_host_bridge.py:1](../integration/two_host_bridge.py#L1)
- Reusable capture processing and partial-result recovery.
  [two_host_map.py:245](../integration/two_host_map.py#L245)
- Explicit CPU/CUDA stage and IMU experiments.
  [run_orb_slam_gpu.py:1](../integration/run_orb_slam_gpu.py#L1)
- Independent object-correspondence diagnostic.
  [object_pose_correction.py:33](../integration/object_pose_correction.py#L33)
- Actual commands, limitations and user workflow.
  [OVERNIGHT_RUNBOOK.md:1](../integration/OVERNIGHT_RUNBOOK.md#L1)


## Final Validation Status

New map/URDF PASS at01:50: 18390pairs,613.128s,p90 residual4.235mm/.106018deg. Live cameras executed and cleaned up, camera identity/calibration/input/PTP checks PASS, but3884RGBD with0pose in almost black input caused READY timeout/exit2. Same new Atlas on lit source recording PASS450inputs449poses. Final report: `output/overnight_20260909/RESULTS.md`. Code implementation is review-verified; overall requested live outcome and moving IMU accuracy remain incomplete due current physical input conditions. Do not label this spec fully done.
