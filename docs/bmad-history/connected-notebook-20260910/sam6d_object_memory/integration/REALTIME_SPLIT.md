# 두 노트북 실시간 ObjectMemory

SAM 노트북에서 실행한다. 두 카메라는 같은 프레임에 단단히 고정하고, 9핀 sync 케이블과 LAN을 연결한다. ORB pose·map ID·tracking·ready는 **유선 ROS2 DDS로 원본 publisher에서 SAM subscriber에 직접 전달**한다. SSH는 프로세스 시작·종료 및 파일 전송, PTP는 두 호스트 시계 동기화에 사용한다. 촬영 동기화 펄스는 카메라 sync 케이블을 사용한다.

기본 설정의 ROS domain은 73(`ros_domain_id + 1`)이다. 카메라·녹화·입력 검사는 SHM/loopback 전용이며, ORB·SAM·ObjectMemory가 loopback 및 전용 유선 IPv4에서 DDS를 사용한다. ObjectMemory도 원격 map ID를 직접 구독한다. 구독 전용 감시기가 원본 publisher GID·map 변경·상태 타임아웃을 검사하며, SAM은 pose 수신 및 결과 사용 경계에서 발행자를 재검사한다. SSH pose 재발행이나 자동 fallback은 없다. 처리 노드의 다른 ROS 관리 토픽도 발견될 수 있으나 카메라 원본 영상은 호스트 내부에서 전달한다.

`integration/two_host.local.yaml`의 `network.local_ip`와 `remote_ip`는 **유선 전용 주소**여야 한다. 현재 SAM은 `192.168.240.1/30`, SLAM은 `192.168.240.2/30`이다. 실행 전에 양쪽 주소가 연결된 물리 유선 인터페이스에 있고, 상대 경로가 그 인터페이스를 직접 사용하는지 검사한다. Wi-Fi 경로로 실행하지 않는다. 프로필에 저장된 주소는 어댑터 교환 후에도 기존 유선 연결 선택 로직으로 적용된다.

## 1. 지도와 카메라 간 변환 생성

```bash
cd /home/etri/sam6d_object_memory
python3 integration/create_map_urdf_split.py --name my_rig_map --input-check-seconds 600
```

`my_rig_map`은 기존에 없는 새 이름이다. 준비·워밍업 이후 600초 수집과 입력/동기화 검사를 통과하고, 겹치는 장면을 보며 고정된 두 카메라를 함께 움직여 SLAM 지도와 extrinsic을 계산해야 한다. 원본 녹화와 맵·URDF 파일 전송에는 rsync/SSH를 계속 사용한다. 고정 프레임을 설치한 뒤에는 기존 장착 상태에서 계산한 URDF를 재사용하지 않는다. `distributed_map.json`의 PASS는 실제 검사를 거친 결과여야 한다.

기본으로 **원격 SLAM 노트북**의 로그인된 데스크톱에 `ORB-SLAM3: Map Viewer`와 `ORB-SLAM3: Current Frame` 창을 연다. 촬영 중에는 현재 특징점과 sparse map의 추적 상태를 확인하고, 촬영 후에는 실제 지도 생성 및 SAM 카메라 localization 재생을 확인한다. 촬영 preview는 별도 임시 지도로, 최종 Atlas/URDF는 기존 녹화 검증 및 보정 절차에서 생성한다. `--headless`는 양 단계의 창을 끄고, `--view`는 기본 표시를 명시한다. `--from-capture`/`--resume`에서도 후처리 표시 옵션을 적용한다. `--input-check-only`/`--observe-only`는 ORB preview 없이 카메라 입력만 검사한다.

SSH X forwarding 대신 원격 사용자의 기존 `DISPLAY`/`XAUTHORITY`를 사용한다. 원격 데스크톱이 없거나 접근할 수 없으면 오류와 `--headless` 안내를 출력한다.

두 split 진입점은 기본으로 `--orb-backend cuda`를 사용한다. 이는 ORB descriptor sampling을 GPU에서 수행하는 **CPU/CUDA 혼합 실행**이며, 특징점 검출·방향·blur·matching·tracking·BA는 CPU에서 수행한다. CUDA 장치/라이브러리를 검사하고 실제 backend 실행을 로그로 확인하며 실패 시 CPU로 자동 대체하지 않는다. CPU 비교는 `--orb-backend cpu`로 명시한다. 빌드 및 측정은 [GPU 실행 문서](GPU_SLAM_EXPERIMENT.md)를 참고한다.

## 2. 지도 좌표에 객체 등록

```bash
python3 integration/run_object_memory_realtime_split.py \
  --map-dir output/my_rig_map --seconds 120
```

창이 기본으로 열린다. ORB는 저장 atlas에서 localization만 수행하며, SAM 검출을 ObjectMemory가 지도 좌표에 누적한다. 저장 위치는 실행 출력의 `object_memory.json`이다. 객체가 실제로 보이는 시점에 등록하고, 파일의 `objects`가 비어 있지 않은지 확인한다. `--headless`는 창 없이 실행한다. 원본 RGBD 저장은 `--record`를 명시할 때만 한다.

실시간 split은 `--pose-match-ms 8` 범위의 같은 촬영 프레임 pose를 사용한다.
다음 프레임을 기다리는 pose 보정과 앞뒤 pose 보간은 적용하지 않는다. 허용치는
30Hz 반주기(16.667ms) 미만이어야 하며, 범위 밖 pose는 사용하지 않는다.
`--slam-wait-ms` 기본 100ms는 계산 중인 같은 촬영 프레임 pose의 도착 제한이다.
9핀 연결만으로 정확한 동시 촬영을 가정하지 말고 하드웨어 카운터·timestamp·PTP로 확인한다.

SAM 입력 수신/검사는 계속 실행한다. 추론 중 수신된 프레임은 다음 추론에서 생략하고,
추론 완료 후 처음 수신된 프레임 하나를 공유메모리에 보관하여 다음 입력으로 사용한다.
`frames.jsonl`의 실제 monotonic/unix 시작·완료 시각과 source 순번으로 이를 확인할 수 있다.
기존 `t_start_ns`/`t_done_ns`는 녹화 시간 환산 필드이므로 실제 지연 분석에는 새
`actual_*` 필드를 사용한다. `sam_written`은 실제 공유메모리 갱신 횟수다.

추적 실패 중에는 카메라 화면과 동적인 지도 위치·객체 표시를 중단하고 `TRACKING LOST`
경과시간만 표시한다. 추적이 회복되면 자동 재개한다. 수신한 tracking-state의 관측 구간,
손실 횟수·누적 시간·최장 시간은 `two_host_report.json`의 `pose_transport`에 기록한다.
통신 단절과 관측 시작 이전의 추적 손실 시간은 이 통계로 확정할 수 없다.

## 3. 저장 객체를 고정하여 실시간 표시

분산 실행 도입 전의 `260904_test_01` 같은 지도는 `--legacy-map`을 명시하여 시험할 수 있다.
`camera_roles.json`의 시리얼과 실제 두 카메라, 저장 내부 파라미터, 현재 PTP/입력 검사는 유지한다.
과거 동기화 검증과 현재 장착 상태의 일치는 미검증으로 보고하며 `distributed_map.json`의 PASS를 생성하지 않는다.
같은 카메라의 내부 파라미터 드리프트 허용치는 기본 0.25픽셀이다. 대략적인 정합 시험에만
`--camera-tolerance-px 1`처럼 명시적으로 완화할 수 있으며, 실제 차이와 허용치를 실행 보고서에 기록한다.
시리얼·해상도·왜곡 계수 검사는 계속 적용하고 저장 Atlas의 내부 파라미터는 변경하지 않는다.
Atlas는 같은 상대 경로로 원격 `output/<지도>/camera_extrinsic/map_slam/`에 체크섬 검증하여 준비해야 한다.

```bash
python3 integration/run_object_memory_realtime_split.py \
  --map-dir output/260904_test_01 --legacy-map --view --seconds 60
```

```bash
python3 integration/run_object_memory_realtime_split.py \
  --map-dir output/my_rig_map \
  --object-map output/my_rig_map/object_memory/realtime_YYYYMMDD_HHMMSS/object_memory.json
```

등록된 객체 pose는 변경하지 않는다. 각 SAM RGB 프레임의 시각에 대응하는 SLAM pose와 URDF로 SAM 카메라 pose를 계산하고, 저장된 객체 좌표축과 CAD 박스를 매 프레임 투영한다. 따라서 오버레이 갱신은 SAM 추론 속도에 묶이지 않는다. 저장 객체의 map frame, SE(3), ID, confidence를 검사한다. 현재 overlay가 class name을 키로 사용하므로 같은 이름의 여러 객체를 담은 저장 파일은 명시적으로 거부한다.

Ctrl-C로 양쪽 worker를 종료하고 결과를 저장한다. 자동 검증은 `--seconds 60`, 시작 제한은 `--ready-timeout 600`으로 지정한다. `two_host_report.json`에는 실행 여부와 측정 품질을 구분하여 기록한다. Git revision 차이는 경고이며, 카메라 serial/역할·map geometry·PTP·tracking guard는 계속 검사한다. 현재 PTP 시험 한계는 로컬 설정의 5000 us이며, 이것은 1 ms 정확도를 보장하지 않는다.

추적을 잃거나 map ID가 바뀌거나 연결/시계 감시가 끊기면 유효 pose로 취급하지 않는다. 정지 카메라만으로는 이동·회전 정확도나 고정 프레임의 실제 extrinsic을 검증할 수 없다.

## 연결 검증 (카메라 불필요)

```bash
python3 integration/test_two_host_dds.py --config integration/two_host.local.yaml
```

두 호스트에서 domain 171의 합성 pose를 실제 유선 DDS로 전달해 원본 publisher GID 일치, pose 값, 늦게 참여한 구독자의 map/ready 수신, 양쪽 private 영상 격리, 상태 단절·복구, map 변경 거부 및 종료를 검사한다. 결과는 `output/readiness/direct_dds_*/result.json`에 남는다. 이 테스트는 지도·물리 카메라 정확도 검증을 대신하지 않는다. 실행 전 양쪽 `two_host_dds.py`, `two_host_bridge.py`, `two_host_realtime.py`, `test_two_host_dds.py`를 동일 버전으로 배포한다.

새 노트북에 처음 설정할 때는 사용하지 않는 전용 서브넷을 선택하고 프로필과 YAML을 함께 맞춘다. 현재 구성의 설정 명령은 다음과 같다. 인터페이스명은 `python3 integration/two_host_network.py interface`로 확인한다.

```bash
# SAM 노트북
nmcli connection modify sam-ptp-ipv6-slave ipv4.method manual ipv4.addresses 192.168.240.1/30 ipv4.never-default yes
nmcli device reapply YOUR_SAM_WIRED_INTERFACE

# SLAM 노트북: 권한이 없으면 sudo 사용
sudo nmcli connection modify remote-slam-wired ipv4.method manual ipv4.addresses 192.168.240.2/30 ipv4.never-default yes
sudo nmcli device reapply YOUR_SLAM_WIRED_INTERFACE
```

Wi-Fi와 PTP IPv6 주소는 유지한다. 유선 프로필에 다른 IPv4 설정이 있는 시스템에서는 기존 값을 먼저 확인한다.

## 저장 영상으로 두 호스트 전체 경로 재현

카메라 입력 대신 기존 보정된 atlas·URDF·객체맵과 녹화 영상을 사용한다. 원래 timestamp를 보존하고 양쪽 노트북에서 역할별 topic만 재생한다. 이 경로에는 현재 카메라/프레임의 보정이나 PTP 검증 결과를 적용하지 않는다.

```bash
python3 integration/run_object_memory_realtime_split.py \
  --map-dir output/260904_test_01 \
  --replay-dataset output/260904_test_01 \
  --object-map output/260904_test_01/object_memory/realtime_20260904_094953/object_memory.json \
  --headless
```

`--headless`를 빼면 영상 창을 표시한다. 약 97초 분량 전체를 실행하며 최초 실행은 SLAM 입력 bag와 atlas를 다른 노트북에 복사한다. `two_host_replay_report.json`은 실제 pose 정합, 등록 객체가 화면 안에 투영된 프레임, 영상 수신을 검사한다. `overlay_latest.png`로 화면을 확인한다. 녹화 timestamp를 사용하므로 이 결과의 capture-to-arrival 지연은 실시간 성능 수치가 아니다.

카메라 역할·serial·영상 크기·왜곡 모델을 지도 생성 입력과 비교한다. 작은 공장 내부 보정 변화는 같은 광선의 영상 모서리 위치 변화가 최대 0.25 px인 경우에만 허용하고, 저장 atlas의 K는 그대로 둔다. 실제 차이는 보고서에 남긴다. 별도 GPU 실험의 엄격한 설정 일치 정책과는 구분된다. serial이 없는 구형 녹화는 지도를 만든 원래 `info.json`을 그대로 사용하는 경우만 허용한다. 다른 입력의 serial을 확인할 수 없으면 거부한다.

## 실제 객체가 있는 압축 녹화의 오버레이 확인

```bash
python3 integration/validate_recorded_object_overlay.py \
  --run-dir output/260904_test_02/object_memory/realtime_20260904_153542 \
  --map-dir output/260904_test_02
```

저장된 SLAM 위치와 SAM 마스크를 사용한다(`RECORDED_POSES`). RGB와 실제 기록된 16-bit 깊이 프레임을 source sequence와 두 timestamp가 모두 일치할 때만 연결한다. 기존 실시간 투영·품질 평가 함수를 그대로 실행하고 `overlay_best.png`, `rgb_best.png`, `depth_best_mm.png`, 프레임별 결과와 `report.json`을 새 폴더에 저장한다. 객체 위치 파일은 변경하지 않는다. `best_frame_alignment_status`는 선택된 장면의 정합, `object_alignment_status`는 전체 검사 대상의 정합이다. ORB·SAM 재추론과 실시간 지연 검증은 위의 두 호스트 전체 재현 결과를 확인한다.
