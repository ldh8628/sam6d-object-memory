# 단일 카메라 ORB-SLAM3 localization ROS 발행

`run_orb3_publish.py`는 **사전에 생성된 Atlas를 읽고**, RealSense RGB-D 카메라 한 대의 입력을 순서대로 처리하여 지도 기준 카메라 pose를 ROS 2 Jazzy로 발행한다. Python은 실행·설정·종료·결과 보고를 담당하고, 영상 수신과 ORB-SLAM3는 `orb3_live_node` C++ 실행 파일에서 처리한다.

## 실행

현재 작업 폴더 `/mnt/c/ldh_ws/orb3slam_ws`에서도 동일 이름의 진입 스크립트를 사용할 수 있다. 버전 관리되는 구현은 `sam6d-object-memory/run_orb3_publish.py`이다.

```bash
cd /mnt/c/ldh_ws/orb3slam_ws/sam6d-object-memory
python3 run_orb3_publish.py \
  --atlas /absolute/path/to/your_map.osa \
  --settings /absolute/path/to/mapping_camera.yaml \
  --serial YOUR_CAMERA_SERIAL
```

- `--atlas`: 실제 환경에서 정상 저장된 `.osa`. 빈 지도는 카메라를 열기 전에 거부한다.
- `--settings`: **그 지도를 만들 때 사용한 OpenCV 카메라 YAML**. ROS launch용 YAML이 아니다. 해상도·내부 파라미터·왜곡 계수·depth 단위가 연결 카메라와 맞아야 한다. SDK와 지도 카메라 모델의 투영 차이도 시작 전에 검사한다.
- `--serial`: 한 대만 연결된 경우 생략할 수 있다. 두 대 이상이면 필수다.
- 이 작업공간에서 Ubuntu-24.04 WSL 터미널로 실행해도 기존 `Ubuntu-22.04`의 Jazzy 환경으로 전달한다. ROS·SAM 재설치, SSH 접속, 네트워크 주소 변경은 하지 않는다.
- 다른 설치에서는 Jazzy와 이 저장소의 install 환경을 source하고 `export ORB3_PUBLISH_ENV=1`을 설정한다.
- 빌드: `bash wsl2/build_orb.sh`. `orbslam3_ros2`, ORB core 및 `distributed_slam_interfaces`를 함께 빌드한다.
- 파일·환경 확인만: 위 명령에 `--check`를 붙인다. 이 검사는 카메라를 열지 않고 Atlas 내부 역직렬화도 하지 않는다. 실제 실행 시 Atlas 내부의 유효 키프레임을 추가로 검사한다.

Ctrl-C로 수신을 종료하고 이미 FIFO에 들어온 프레임을 처리한 뒤 정상 종료한다. 출력 경로는 실행 시 표시된다. `--output`으로 새 디렉터리를 지정할 수 있다. 기존 디렉터리를 덮어쓰지 않는다. 원본 Atlas의 실행 전후 SHA256을 비교하고, `System.SaveAtlasToFile` 설정은 실행용 YAML에서 제거한다.

WSL 배포판 사이로 실행을 전달할 때는 Python 부모가 실행별 공유 제어 파일로 종료 요청을 전달한다. `wsl.exe`의 신호 전달에 의존하지 않으며, 다른 실행이나 다른 WSL 프로세스를 종료하지 않는다.

## 모든 프레임 처리의 의미

종료 신호가 RGB와 depth 도착 사이에 들어오면 이미 받은 프레임의 나머지 짝까지 수신한 뒤 멈춘다. 짝이 끝내 도착하지 않으면 입력 단절로 보고한다. 실행 중 바꾼 카메라 옵션은 종료 시 원래 값으로 복원한다.

이 실행기는 최신 프레임으로 대기열을 덮어쓰거나 추론 간격에 따라 입력을 건너뛰지 않는다. RGB·depth 원시 callback을 각각 검사하고 직접 구성한 RGB-D 쌍을 별도 메모리에 복사한 뒤, FIFO의 모든 쌍에 `TrackRGBD`를 한 번씩 호출한다. 영상·점군은 ROS로 발행하지 않는다.

카메라 RGB와 depth 프레임 번호, 타임스탬프 단조 증가, 두 스트림 간 시간 차이를 검사한다. 프레임 번호 누락·중복·역순, 불완전한 쌍, 시간 불일치, 5초 입력 단절, FIFO 초과는 실행 실패로 남긴다. 미처리 프레임의 나이가 기본 250ms를 넘으면 수신을 중단하고 이미 받은 프레임을 처리한 뒤 실패로 보고한다(`--max-backlog-ms`). 기본 FIFO 용량은 120쌍이며 `--queue-capacity`로 변경한다. 용량을 늘려도 33ms 성능 판정이 개선되지는 않는다. 대기 시간까지 같은 프레임의 지연에 포함된다.

**물리 장치·USB·운영체제의 프레임 무손실이나 가려짐·블러·지도 밖 시야에서의 매 프레임 정확한 위치는 소프트웨어만으로 보장할 수 없다.** 지도 매칭이 불가능하면 `tracking=false`를 발행하고 계속 재localization한다. 입력 무손실 조건이 깨지면 수신을 종료하고 실패 개수와 지연을 보고한다. 마지막 정상 pose를 반복하거나 가짜 pose를 생성하지 않는다.

## ROS 인터페이스와 LAN

| 토픽 | 메시지 | 의미 / QoS |
|---|---|---|
| `/orbslam3/pose` | `geometry_msgs/PoseStamped` | 지도 추적이 유효한 프레임만 발행. Best Effort, Keep Last 1 |
| `/distributed_slam/state` | `distributed_slam_interfaces/State` | 매 처리 프레임의 pose·유효 여부·촬영 시각·순번·지도 ID·지연 시각. Best Effort, Keep Last 1 |
| `/distributed_slam/session` | `distributed_slam_interfaces/Session` | 지도 SHA256·세션 ID·모드. Reliable, Transient Local |

기본 식별자 길이에서 측정한 직렬화 크기는 PoseStamped 76바이트, State 252바이트로, 두 메시지를 30Hz로 보낼 때 payload 합계는 약 9.84KB/s다. DDS·UDP·Ethernet 부가량은 제외했으며 실제 LAN 패킷 캡처 결과는 아니다.

pose는 **`T_map_camera`**, 즉 카메라 optical 좌표를 지도 좌표로 변환하는 SE(3)이다. 단위는 미터, quaternion 순서는 ROS의 x,y,z,w다. ORB-SLAM3 `T_camera_map`을 역변환한 값이다. `header.stamp`는 추론 완료 시각이 아니라 RGB 촬영 시각이다. `header.frame_id` 기본값은 `map`이며, 지도 축을 ENU나 robot base 축으로 임의 변경하지 않는다. `tracking=false`일 때 state의 pose를 사용하면 안 된다.

기본 `ROS_DOMAIN_ID=77`, Fast DDS, LAN 서브넷 발견을 사용한다. 반대 노트북에서:

```bash
source /opt/ros/jazzy/setup.bash  # 실제 B의 Jazzy 설치 경로 사용
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
ros2 topic echo /orbslam3/pose geometry_msgs/msg/PoseStamped \
  --qos-reliability best_effort
```

사용자가 나중에 검증할 원격 노트북의 주소·SSH ID는 이 실행에 필요하지 않다. 실제 LAN 통신은 양쪽 인터페이스·방화벽·WSL mirrored networking과 동일 domain 설정이 갖춰져야 한다. `--localhost-only`는 동일 PC 시험용이며 LAN 운용에서는 생략한다. 현재 코드 검증 결과를 반대 노트북 수신 성공으로 간주하지 않는다.

`/orbslam3/pose`는 표준 메시지라 B에 별도 메시지 패키지가 필요하지 않다. 프레임별 유효 여부와 순번까지 구독하려면 B에도 이 저장소의 `distributed_slam_interfaces` 패키지를 빌드·source해야 한다.

Discovery Server를 사용할 때는 B에서 서버를 실행한 후 A에 `--discovery-server 10.77.0.2:11811`을 추가하고, B의 구독 환경에도 같은 `ROS_DISCOVERY_SERVER`를 지정한다. 서버를 준비하지 않았다면 이 옵션을 넣지 않는다. 카메라 영상과 점군은 프로세스 밖으로 나오지 않는다.

Best Effort는 ROS/LAN 전송의 모든 메시지 도착을 보장하지 않는다. 원격 수신에서 state 순번 누락을 세어야 한다. 입력 전 프레임을 처리하는 요구와 지연을 줄이기 위한 DDS 전송 정책은 각각 검증한다. 수신기는 입력 단절 시 마지막 pose를 계속 사용하면 안 되며, 수신 타임아웃과 `tracking_epoch` 변화로 보간 구간을 끊어야 한다. 정상 종료는 session의 `mode=stopped`로 알린다. 종료 알림을 프레임 state에 추가하지 않아 마지막 프레임을 덮어쓰지 않는다.

## 회전 대응과 실행 옵션

- 기본 `--rotation-fallback on`: localization에서 운동 예측이 실패하거나 지도와의 연결을 잃으면, 같은 프레임에서 기준 키프레임 BoW 매칭과 기하 최적화를 재시도한다. 정상 운동 예측 경로에는 추가 재매칭 비용이 없다.
- 이전 프레임이 아닌 **현재 프레임**에서 temporal VO로 전환된 결과도 지도 pose로 발행하지 않도록 core를 수정했다. 잠깐 추적을 잃은 상태도 저장 지도에서 재localization하고, 복구 직후 pose 차이를 속도로 사용하지 않는다. 첫 입력 프레임도 바로 localization을 시도한다.
- 기본 `--features 1200 --opencv-threads 3`: 현재 Ultra 7 258V·640×480·30Hz의 회전 재생과 SDK 입력 비교로 선택했다. 프레임 수나 해상도를 줄이지 않고 ORB 특징점 예산만 고정한다. 다른 장비·해상도는 같은 재생 시험으로 확인한다.
- `--features 0`: 지도 생성 설정의 특징점 수를 그대로 사용한다. 특징점을 더 줄이면 회전·저질감 구간 매칭이 나빠질 수 있으므로 실측한 결과로 선택한다.
- SDK auto-exposure priority를 0으로 설정하여 자동 노출이 촬영 FPS를 낮추지 않도록 한다. `--color-exposure`는 SDK color exposure 옵션 값으로 수동 지정한다. **color·depth 센서의 노출 단위가 같다고 가정하지 않는다.** 밝기·블러와 실제 프레임 번호를 확인하고 조정한다.
- `--color-exposure 0`은 카메라의 기존 자동/수동 노출 모드를 유지한다. 자동 노출이 반드시 켜져 있다고 가정하지 않는다.
- IMU가 있다고 기존 visual RGB-D Atlas에 IMU 모드를 임의로 켜지 않는다. 관성 보정·시각 동기화·그에 맞는 지도 생성 및 core localization 경로 검증이 필요하다.

근거와 실측 결과는 `RESEARCH.md`, `VALIDATION.md`에 정리한다.

## 결과 파일

- `frames.csv`: 수신 순번·카메라 RGB/depth 번호·촬영 시각·대기열 진입·추적 시작/완료·발행·상태·특징점·지도 매칭 수·지연·pose. RGB/depth 짝 대기, 정렬·복사, 추적 FIFO 대기와 추적 스레드의 CPU 시간도 진단용으로 기록한다.
- `trajectory.tum`: 유효한 pose만 저장. 누락 구간을 메우지 않는다.
- `run.json`, `settings.yaml`, `parameters.yaml`: 재현에 필요한 지도·설정 해시와 실제 실행 파라미터.
- `console.log`: native 로그.
- `summary.json`: RGB/depth 원시 수신·쌍 구성·처리·정상 pose 수, 프레임 손실, 실제 촬영/수신 Hz, FIFO 최대 길이, 동일 프레임의 로컬 지연 p99/최대값, Atlas 불변 여부. 파일 기록은 별도 스레드에서 수행한다.

`local_33ms_met`는 유효 추적 프레임의 로컬 입력→ROS 발행 p99 결과다. 추적 실패를 포함한 전체 지연과 실패 개수도 별도로 보고한다. 카메라 입력 시 측정 시작점은 RGB 원시 callback 진입이므로 depth 대기와 정렬 시간도 포함된다. **원격 B 수신까지 33ms 달성했다는 의미가 아니다.** 이 코드는 `lan_latency_verified`와 `full_requirement_verified`를 자동 참으로 만들지 않는다. 실제 카메라+LAN의 장시간 시험에서 별도로 판정해야 한다.

메시지의 `publish_ns`는 발행 호출 직전 UTC 시각이다. CSV의 `enqueue_to_publish_ms`는 해당 프레임의 두 발행 호출이 반환된 뒤까지 monotonic clock으로 측정한 값이다. 비동기 발행 호출의 반환은 네트워크 전달 완료가 아니다.

종료 코드는 `0`=입력 처리 완료, `2`=설정·시작 실패, `3`=입력 계약 실패, `4`=Atlas 변경 감지, `5`=처리는 완료했으나 유효 pose가 전혀 없음이다. `0`만으로 33ms나 전 프레임 정확도를 통과했다고 판단하면 안 된다. `startup_seconds`는 카메라 수신 전 vocabulary·Atlas·SLAM 준비 시간이며, `slam_shutdown_seconds`는 SLAM의 종료 대기 시간이다.

## 카메라 없는 재현

```bash
bash orb3_publish/download_benchmarks.sh
python3 orb3_publish/prepare_tum.py /opt/orb-live/benchmarks/rgbd_dataset_freiburg1_xyz
python3 orb3_publish/prepare_tum.py /opt/orb-live/benchmarks/rgbd_dataset_freiburg1_rpy
bash orb3_publish/benchmark_env.sh --output /opt/orb-live/benchmarks/my_orb3_test
```

`benchmark.py`는 `xyz`로 테스트 지도를 만들고 `rpy`에 fallback on/off를 번갈아 비교한다. 테스트 지도는 실제 카메라 운용 지도가 아니다. 데이터셋 짝 구성에서 제외된 RGB/depth 원본 수는 각 데이터 디렉터리의 `association_report.json`에 남는다.

기존 Atlas의 데이터 재생은 `run_orb3_publish.py ... --dataset DATASET_DIR`로 수행한다. `--replay-rate 1`은 원본 시각에 맞춘 재생이고 `0`은 무제한 속도 FIFO 부하 시험이다. `--realsense-bag FILE.bag`는 SDK 입력 경로의 별도 재생 시험이며 실제 카메라 검증으로 표시되지 않는다.
