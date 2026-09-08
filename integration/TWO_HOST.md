# 두 노트북 실행

동일한 monorepo release를 사용한다. 원격은 ORB와 RealSense용 Jazzy conda 환경만 필요하며 SAM 모델/weights는 전송하지 않는다. 기존 명령은 `--two-host-config`가 없으면 그대로 동작한다.

## 설정과 배포

```bash
cp integration/two_host.example.yaml integration/two_host.local.yaml
# SSH 대상/port, 양쪽 IP, 유선 PTP 인터페이스와 conda 경로를 편집한다.
python3 integration/deploy_remote.py --config integration/two_host.local.yaml --ref two-host-v1
```

배포 전에 로컬 HEAD가 해당 tag/SHA이고 clean이어야 하며 GitHub에도 같은 ref가 있어야 한다. 원격은 OpenSSH key/agent 인증, GitHub 접근, Python3, git, rsync, 호환되는 Jazzy/realsense 환경과 colcon/C++ 빌드 의존성이 필요하다. SSH alias도 가능하며 `ssh.port` 기본값은 22다. 예제의 환경 경로는 실제 호스트에 맞춘다. 배포 스크립트는 환경이나 시스템 패키지를 자동 설치하지 않는다.

새 경로는 clone, 기존 저장소는 origin/dirty 검증 후 detached checkout한다. Vocabulary는 SHA-256 확인 후 Linux atomic no-replace rename으로 staging을 승격한다. 기존 자산과 checksum이 다르면 덮어쓰지 않는다. ORB source fingerprint가 바뀌거나 설치/성공 manifest가 없으면 두 ORB 패키지를 빌드한다. 환경/message definition hash와 self-test가 통과한 뒤 양쪽에 동일 `deployment.json`을 기록한다. 실패한 빌드는 원격 성공 manifest를 남기지 않는다. 자산은 Git에서 제외된다.

설정 파서는 표준 라이브러리만 사용한다. 예제와 같은 두 단계 scalar YAML 또는 JSON을 허용하며 YAML anchor, list, inline comment 등은 거부한다. SSH 비밀번호와 private key를 넣지 않는다.

## PTP

실행 전에 유선 연결과 PTP를 별도로 준비한다. 원격 SLAM 노트북이 master, 로컬 SAM 노트북이 slave다. Wi-Fi는 PTP 인터페이스로 허용하지 않는다. `ethtool -T INTERFACE`로 실제 지원을 확인한다.

하드웨어 PTP 예시(인터페이스 이름을 바꾸고 각 명령은 별도 터미널/서비스에서 실행):

```bash
# 원격
sudo ptp4l -i REMOTE_INTERFACE -m --priority1 10
sudo phc2sys -s CLOCK_REALTIME -c REMOTE_INTERFACE -w -m
# 로컬
sudo ptp4l -i LOCAL_INTERFACE -s -m
sudo phc2sys -s LOCAL_INTERFACE -c CLOCK_REALTIME -w -m
```

하드웨어 timestamp가 없다면 유선 NIC가 software-transmit/receive를 지원하는지 확인하고 양쪽 `ptp4l`에 `-S`를 사용한다. 이 모드에서는 ptp4l이 시스템 시계를 직접 조정하므로 PHC/phc2sys가 필요하지 않다. **software PTP도 실측 1 ms와 10분 카메라 검증을 통과해야 한다.** 실행 중 다른 시간 서비스가 같은 시계를 조정하지 않도록 시스템 시간 구성을 관리한다.

실행 사용자가 `pmc -u -b 0 'GET PORT_PROPERTIES_NP' 'GET TIME_STATUS_NP' 'GET TIME_PROPERTIES_DATA_SET'` 응답을 읽을 수 있어야 한다. 하드웨어 방식은 해당 `/dev/ptpN` 읽기 권한도 필요하다. preflight는 daemon이 설정한 인터페이스, 실제 timestamp 방식, master/slave 상태, master offset과 grandmaster identity를 읽는다. 두 호스트 grandmaster가 같아야 하며 로컬 SAM 환경의 ROS/message type도 별도로 확인한다. 하드웨어 방식에서는 UTC/TAI 차이를 반영하여 시스템 시계–PHC 오차와 측정 불확실성을 양쪽 모두 합산하여 호스트 간 1 ms 제한을 검사한다. 단순한 ptp4l 실행 여부로 통과시키지 않는다.

[linuxptp ptp4l](https://www.linuxptp.org/documentation/ptp4l/), [phc2sys](https://www.linuxptp.org/documentation/phc2sys/).

## 맵과 URDF

```bash
python3 integration/create_map_urdf_split.py --name SESSION
# 10분 검증 뒤 자동 종료하려면 --input-check-seconds 600
# 다른 설정 파일: --config /path/to/two_host.yaml
```

일반 터미널에서 실행하면 저장소의 `integration/two_host.local.yaml`을 읽고 `environment.local_conda_sh`와 `orb_env`로 로컬 환경을 활성화한다. 원격 SLAM 터미널에서 별도 명령을 실행할 필요는 없다. 최초 환경 준비·배포는 위 단계에서 수행하며 촬영 명령은 패키지 설치나 Git checkout을 하지 않는다. `--help`는 conda 없이 실행된다.

지원 옵션은 `--name`, `--config`, `--baseline`(기본 0.095 m), `--camera-timeout`(기본 30초, 양쪽 worker에 적용), `--input-check-seconds`(0 또는 600초 이상), `--input-check-only`, `--help`다. 기본 실행은 최소 검사 시간 이후 Enter로 종료한다. `--input-check-only`는 녹화·동기 검사까지만 수행하고 맵을 만들지 않는다. 잘못된 설정/옵션은 카메라 시작 전에 거부하며 단일 장비로 자동 전환하지 않는다. 기존 `python integration/create_map_urdf.py --two-host-config ... --name SESSION`과 단일 장비 명령도 유지한다.

실행자는 두 호스트 commit/ROS/PTP를 검증하고 원격 master READY 이후 로컬 full slave를 시작한다. 각 worker는 실행 중 DeviceInfo serial과 실제 sync parameter를 읽는다. 두 카메라 모두 RGBD와 metadata를 자기 디스크에만 기록한다. stdin heartbeat가 끊기면 worker는 자기 자식 프로세스 그룹을 닫는다. 종료는 slave, master 순이다.

30초의 각 카메라 baseline 및 준비 시간 외에 최소 600초 공통 구간을 기록한다. 카메라 frame counter 역행/중복, 유실률 0.1% 초과, 대응 ROS 시각과 sensor timestamp drift p95 5 ms 초과는 실패한다. sensor clock의 독립적인 부팅 epoch만 처음 30쌍으로 보정하고 이후 drift는 그대로 평가한다. 기존 호스트별 input integrity 검사에서 한 건의 누락도 실패하므로 현재 유실 허용은 이보다 엄격하다. 실패 원본과 보고서는 보존하며 mode 2로 바꾸지 않는다. [D455 full slave 사례](https://github.com/IntelRealSense/librealsense/issues/8419).

두 bag을 검증한 뒤 SAM 녹화만 원격 staging에 전송한다. 기존 extrinsics pipeline이 원격에서 atlas/dense map/localization/URDF를 생성한다. 저장된 rig reference fallback은 분산 calibration 성공으로 인정하지 않는다. 병진 p90 0.10 m, 회전 p90 5도, 카메라 거리 1 m hard gate와 더 엄격한 0.03 m/1도 품질 목표를 구분한다.

로컬에는 URDF, extrinsic/quality/role JSON, 카메라 intrinsics info, dense PCD와 `distributed_map.json`만 돌아온다. atlas와 SLAM bag은 원격에 남는다. 녹화 실패는 `two_host_report.json`에 남는다. 기존 세션을 덮어쓰는 `--force`는 분산 촬영에서 허용하지 않는다.

## 실시간

```bash
python integration/run_object_memory_realtime.py \
  --two-host-config integration/two_host.local.yaml \
  --map-dir output/SESSION --view
# --record: 각 호스트 RGBD 로컬 녹화와 로컬 SAM 처리 녹화
# --input-check-seconds 60: 준비 후 60초 실행, 생략하면 Ctrl-C 종료
```

검증된 distributed map과 같은 release/camera contract가 필요하다. 원격은 atlas localization과 SLAM master, 로컬은 SAM slave·receiver·inference·ObjectMemory·viewer를 실행한다. 프레임은 기존 latest-frame shared memory를 사용한다.

카메라와 소비자는 `ros_domain_id + 1`, localhost/SHM에서 동작한다. LAN domain은 별도 프로세스 bridge만 사용하며 다음 네 application topic을 CDR 그대로 전달한다.

| Topic | Type | QoS |
|---|---|---|
| `/orbslam3/pose` | `geometry_msgs/msg/PoseStamped` | reliable, volatile |
| `/orbslam3/map_id` | `std_msgs/msg/String` | reliable, transient local |
| `/orbslam3/tracking_state` | `std_msgs/msg/String` | reliable, volatile |
| `/orbslam3/ready` | `std_msgs/msg/String` | reliable, transient local |

LAN profile은 64 MiB SHM과 UDPv4를 함께 쓰며 `ROS_STATIC_PEERS`에는 상대 IP만 둔다. bridge의 rosout/parameter 이벤트를 끄고 bounded IPC를 사용해 RGBD가 LAN에 연결되지 않게 한다. Fast DDS의 프로세스 전역 XML 설정 때문에 두 endpoint를 별도 프로세스로 분리한다. [Fast DDS SHM](https://fast-dds.docs.eprosima.com/en/v3.2.1/fastdds/transport/shared_memory/shared_memory.html).

tracking state 값은 표시용이며 fusion을 허용하는 근거가 아니다. SAM frame의 exact/interpolated pose가 있을 때만 SAM 시각의 pose와 detection을 게시한다. nearest-only 매칭은 분산 모드에서 거부한다. 매칭 이후에도 inference 반환과 ObjectMemory fusion 시점에 heartbeat/map를 재검사한다. LAN/ORB/PTP 장애나 첫 map identity 이후 변경은 fail closed로 처리하고 viewer는 이전 영상/marker를 지우며 TRACKING LOST를 표시한다. 자동 재개 대신 새 실행을 시작한다.

종료 시 원격 trajectory 및 JSON/JSONL/log 진단만 회수한다. 실패 시에도 로컬 report는 남으며 접속 불가로 회수하지 못한 원격 진단 경로가 report에 보존된다.

## 보고서와 검증

`two_host_report.json`에 commit/환경/PTP 표본, 카메라 입력 진단, ORB consume p50/p95, tracking ratio, exact/interpolated/missed 매칭, SAM 수신/처리/생략과 추론시간, calibration, 실제 ObjectMemory mask/depth 계측을 기록한다. 표본이 없는 값은 null/UNMEASURED다. Frame별 ObjectMemory 비교 근거는 `frames.jsonl`의 `diagnostics.object_memory_quality`에 남는다.

PoseStamped에는 촬영 시각만 있고 원격 송신 시각이 없으므로 **측정값은 촬영→pose 도착 지연**이다. 순수 LAN 지연은 별도 송신시각 인터페이스 없이 분리할 수 없어 null로 기록한다. topic/type 계약은 유지한다. 마지막 표본을 보관하는 bounded percentile window는 metrics에 표본 수로 표시하며 원시 JSONL은 남긴다.

정지·직선·저속 yaw·빠른 yaw 구간을 실제로 기록한 뒤 tracking ≥95%, tracking 중 frame–pose 매칭 ≥98%, 촬영→pose 도착 p95 <100 ms, 가시 처리 프레임의 mask overlap ≥0.5/depth residual ≤100 mm 통과율 ≥95%를 평가한다. 실측 임계값은 PASS/FAIL/INCOMPLETE로 판정하며 표본 없이 PASS를 내지 않는다. tracking 중 매칭 비율은 header 없는 tracking state의 최신 수신값을 계측용으로만 사용한다. 네 가지 움직임 구간의 기록·주석은 별도 현장 검증이며, 이를 수행하지 않은 실행은 전체 하드웨어 acceptance를 통과했다고 간주하지 않는다. 분산 후에도 오버레이가 틀리면 이 보고서로 성능 가설을 검토한다.

```bash
python3 integration/test_two_host.py
python3 integration/test_create_map_urdf_split.py
python integration/test_two_host_map.py
python integration/test_two_host_realtime.py
python integration/two_host_bridge.py --self-test
# Jazzy ROS 환경
python integration/test_two_host_bridge_ros.py
python integration/test_receiver_input_ros.py
python integration/test_rgbd_capture_ros.py
# realsense 환경 + orbslam_ws/install_jazzy/setup.bash
python integration/verify_orb_runtime.py
python sam6d_ws/realtime/test_two_host_guard.py
python integration/create_map_urdf.py --self-test
python integration/run_object_memory_realtime.py --self-test
python integration/camera_extrinsic_localization.py --self-test
# 기존 SAM tests는 해당 workspace에서 실행
cd sam6d_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_slam_pose_memory.py tests/test_shm_slam_context.py
```

## 2026-09-08 현장 상태

로컬 IP는 `10.119.19.162`, 원격 SSH alias는 `slam-codex`(port 10022), 원격 저장소는 `/home/jucpark/sam6d_object_memory`다. SSH 연결과 원격 `/home/jucpark/anaconda3/etc/profile.d/conda.sh`의 Python 3.12.14/Jazzy `realsense` 환경을 확인했다. 양쪽 녹화 imports는 정상이며 원격 RealSense ROS 4.57.7, sensor_msgs 5.3.7, ros2interface 0.32.9를 로컬 검증 버전에 맞췄다. ORB 코어·wrapper 빌드와 양쪽 런타임 self-test, 배포 manifest 일치, 녹화 메시지·서비스 6종 해시 일치 및 양쪽 ROS bag 변환·재생을 확인했다. 환경 변경 전 목록은 원격 `output/split_map_environment_before`에 저장했다.

유선 software PTP는 SAM `enx00e04caa7ca7` SLAVE, SLAM `enx00e04cbaf0a3` MASTER이며 같은 grandmaster다. 다만 60초/12표본 중 최대 합산 오차가 **1175.653 μs**로 **1000 μs** 기준을 초과했다. SAM 녹화 공간도 **45.1 GiB 필요 / 26.3 GiB 여유**로 부족하다. 카메라도 계획과 반대로 SAM에 `253822302376`, SLAM에 `253822301680`이 연결돼 있다. 설정은 SLAM master `253822302376` / SAM slave `253822301680`을 유지한다. 실제 10분 녹화·맵·URDF 결과 회수는 PTP 안정화, 약 19 GiB 추가 공간 확보, 카메라 연결 교환 후 검증해야 한다. 진입점/회귀 검사 통과를 실장비 완료로 간주하지 않는다.

Git 이전 `notebook` 브랜치는 보존했고 수정 snapshot은 `notebook-snapshot-20260908`에 있다. `output/migration_backup_20260908/notebook.bundle`과 patch가 로컬 복구본이다. migration은 역사 재작성 없이 snapshot 다음 commit에서 기존 tree를 `sam6d_ws/` 아래로 옮겼다.
