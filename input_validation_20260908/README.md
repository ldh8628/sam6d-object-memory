# 두 카메라 입력 개선 결과 — 2026-09-08

**최우선 입력 경로의 수정·적용·검증을 완료했다.** 실제 프로젝트에서 기본 지도 촬영 30초는 각 카메라 899/899프레임, 실시간 ORB/SAM/Viewer 300초는 각 8,989/8,989프레임으로 PASS했다. 네 녹화/Viewer 조합도 각각 120초 PASS했다. 해상도와 30 FPS 설정을 유지했다.

원인은 드라이버 이후 Fast DDS 전달 지연이었다. RGBD 메시지보다 작은 기본 SHM segment를 공통 64 MiB로 설정하고, native RGBD 입력·순차 worker·명시적 무결성 검사를 적용했다. 원래 프로젝트의 `orbslam_ws/install_jazzy`도 재빌드했다.

- [최종 기계 판독 결과](FINAL_RESULT.json)
- [원인·전체 검증 보고서](verification/REPORT_TWO_CAMERAS.md)
- [실제 설치본 300초 결과](realtime_300/input_integrity.json)
- [기본 지도 촬영 결과](../output/input_capture_validation_20260908_plain/input_integrity.json)
- [네 조건 JSON](verification/two_camera_matrix.json)
- [적용 소스 checksum](applied_source_manifest.json)

## 재현

아래 명령은 프로젝트 루트에서 실행한다. 카메라가 다른 프로세스에서 사용 중이면 먼저 그 실행을 종료한다. 출력 이름은 매번 새로 지정한다.

```bash
cd /home/etri/sam6d_object_memory

# 맵이 생성되지 않는 장면에서도 입력만 검사; 두 serial 확정
python3 integration/create_map_urdf.py --name next_input_check \
  --slam-serial 253822302376 --sam-serial 253822301680 \
  --input-check-only --input-check-seconds 120

# 지도에 저장된 역할 자동 사용; 별도 선택창/ORB_INSTALL 지정 불필요
python3 integration/run_object_memory_realtime.py \
  --map-dir output/260904_test_03 --input-check-seconds 120 --view

# 실제 지도 촬영 단계만 재현(현재 장면으로 offline 맵 생성은 요구하지 않음)
# 원본+SQLite 변환에 필요한 공간이 있어야 함
python3 input_validation_20260908/verification/verify_map_capture.py \
  /home/etri/sam6d_object_memory next_capture_check
```

`--input-baseline`이 없으면 부하 시작 전에 30초 기준 측정을 자동 수행한다(준비 10초 별도). Realtime에 `--record`를 추가하면 native RGBD/metadata 원본 MCAP을 기록한다. 추가 녹화/변환은 현재 약 16 GiB보다 많은 공간을 요구할 수 있으며 사전 검사에서 중단한다.

## 보존과 남은 범위

원래 소스/설치본은 `before_input_changes.tar.gz`, 원래 README는 `before_documentation.tar.gz`에 있다. 기존 사용자 변경을 포함한 백업이며 다른 작업 파일은 덮어쓰지 않았다. 진단 원본은 `camera_diagnostics/`, 준비·종료를 포함한 각 실행 로그는 `verification/`, 최초 카메라 전용 측정은 `candidate_output/`에 보존했다. `/tmp`의 이전 경로는 이 보존 위치를 가리키는 링크다.

큰 과거 bag은 SHA-256 대조 후 원본 그대로 `.zst`에 보존했다. [checksum/보존 경로](verification/preserved_bag_archives.json)를 확인하고, 여유 공간 확보 후 해당 파일에 `zstd -d --keep <파일.mcap.zst>` 또는 `<파일.db3.zst>`를 실행하면 복원된다.

**선택 ORB map viewer 예외는 미해결이다.** 불량 장면에서 맵 리셋을 반복하다 SIGSEGV가 한 번 발생했고, 해당 촬영은 INCOMPLETE다. 이후 긴 offline replay와 실제 GDB 실행에서는 종료가 재현되지 않았다. GDB 실행은 전 프레임을 소비했으나 ORB 큐 p95 27ms로 FAIL이었다. 이를 입력 PASS나 맵 성능 합격으로 숨기지 않았다. [진단](map_viewer_diagnostic_summary.json).

맵/localization 성공, 정상 TRACKING_OK 상태의 pose 정확도, 다수 객체 검출로 PEM 부하가 커지는 조건은 별도 검증 대상이다. 실패 상태의 mask-tracker 변경은 함께 적용하지 않았다.
