# 다른 노트북 Codex 작업 지시서

이 문서를 읽은 Codex는 아래 작업을 실제로 수행하고, 가능한 검증까지 완료하라. 설명이나 계획만 제출하지 말라. 물리적 케이블 연결, sudo 인증, 상대 노트북의 설정처럼 직접 수행할 수 없는 항목은 필요한 조작을 구체적으로 알려주고 독립적으로 가능한 준비를 계속하라.

## 목표와 역할

이 노트북을 두 호스트 시스템의 **원격 SLAM 노트북**으로 준비한다. SAM 노트북이 SSH로 배포·카메라·맵 생성·실시간 실행·종료를 제어한다.

| 항목 | 이 노트북: SLAM 원격 | 상대 노트북: SAM 제어기 |
|---|---|---|
| 사용자/호스트 | jucpark / jucpark-device | etri / 실제 호스트명 확인 필요 |
| 마지막 알려진 IP | 10.57.238.153 | 10.119.19.162 |
| SSH | jucpark@10.57.238.153, 포트 10022 | 원격에서 역방향 SSH는 필수 아님 |
| 저장소 예정 경로 | /home/jucpark/sam6d_object_memory | /home/etri/sam6d_object_memory |
| 카메라 serial | 253822302376, mode 1 master | 253822301680, mode 3 full slave |
| PTP | master | slave |
| conda | realsense: ROS 2 Jazzy, RealSense, ORB | realsense + sam6d |

위 값은 2026-09-08 전달받은 정보다. 실행 시 다시 확인하라. 이 노트북의 Wi-Fi는 wlp129s0f0, enp130s0는 연결 없음, enx00e04cbaf0a3는 연결됐지만 IPv4 없음으로 보고됐다. 세 NIC 모두 하드웨어 PTP timestamp 지원이 없다고 보고됐다. 상대에서 이 노트북으로 SSH 시도는 timeout이었다. 원격 저장소/conda 경로와 환경 유무는 아직 확인되지 않았다.

## 1. 작업 원칙과 소스 확보

- 저장소: https://github.com/ldh8628/sam6d-object-memory.git
- 작업 문서가 있는 브랜치: `two-host-v1-migration`. 기본 브랜치에 있다고 가정하지 말라.
- 기존 저장소가 있으면 최상위 경로, origin, branch, HEAD, dirty 상태를 먼저 확인하라. 기존 변경을 reset/clean하거나 강제 checkout하지 말라. 다른 저장소나 비어 있지 않은 일반 폴더를 덮어쓰지 말라.
- 예정 경로가 없으면 다음과 같이 전체 monorepo를 clone할 수 있다. origin은 제어기와 같은 아래 HTTPS URL을 유지한다.

```bash
git clone --branch two-host-v1-migration \
  https://github.com/ldh8628/sam6d-object-memory.git \
  /home/jucpark/sam6d_object_memory
```

- 기존 checkout이 있으면 문서를 읽기 위해 fetch하거나 문서만 별도 임시 경로로 받아라. 배포할 정확한 SHA의 선택과 최종 checkout은 제어기의 `deploy_remote.py`가 담당한다.
- `integration/TWO_HOST.md`, `integration/two_host.example.yaml`, `integration/deploy_remote.py`, `integration/two_host.py`, ORB 두 패키지의 package.xml/CMakeLists.txt를 읽고 현재 계약을 따르라.
- 소스 수정·commit·push는 SAM 제어기에서만 한다. 원격에서 소스 문제를 찾으면 재현 명령과 필요한 수정 내용을 보고하라. 환경 설치와 호스트 설정은 이 노트북에서 수행한다.
- SAM 모델/weights, SAM 추론 환경을 설치하지 말라. build/install/log, conda 환경, bag, atlas, PCD를 Git에 넣지 말라. 비밀번호나 private key를 수집·출력하지 말라.

## 2. 실제 장비 상태와 SSH 준비

다음 명령부터 실행한다. 없는 명령은 누락으로 기록하고 필요한 도구 설치 후 다시 실행한다.

```bash
hostname
whoami
cat /etc/os-release
uname -m
ip -br address
ip route
df -h
free -h
sudo -n systemctl status ssh --no-pager
sudo -n ss -ltnp
ethtool -T enp130s0
ethtool -T enx00e04cbaf0a3
command -v conda
command -v rsync
command -v ptp4l
command -v pmc
```

설치된 도구를 재사용한다. Ubuntu에서 누락된 기본 도구는 다음 목록 중 필요한 것만 설치한다. sudo가 인증을 요구하면 사용자가 로컬 터미널에서 인증하도록 안내하고 비밀번호를 요청하지 않는다.

```bash
sudo apt-get update
sudo apt-get install git rsync openssh-server linuxptp ethtool build-essential
```

SSH는 실제 리스닝 포트와 유효 설정을 검사한다. 10022가 열려 있지 않으면 기존 접속 경로를 유지하면서 설정을 백업하고 수정한다. `sshd -t` 검사 후 reload하고 결과를 확인한다. 방화벽은 현재 정책을 확인해 필요한 연결만 허용하며 전체 비활성화하지 않는다. 기존 authorized_keys를 보존한다. 제어기의 공개키가 없으면 공개키만 전달받아 등록하거나 제어기에서 ssh-copy-id를 사용하도록 안내한다.

자체 접속 성공만으로 제어기 연결 성공이라고 보고하지 말라. 최종적으로 **SAM 제어기에서** 다음 명령의 성공이 필요하다. IP가 바뀌면 실제 주소를 사용한다.

```bash
ssh -p 10022 -o BatchMode=yes -o ConnectTimeout=5 \
  jucpark@10.57.238.153 hostname
```

## 3. 유선망과 PTP master 준비

양쪽의 유선 인터페이스와 사용 가능한 IP를 확인한다. 케이블 연결·상대 IP가 미정이면 그 정보를 요청하고 다른 환경 준비를 진행한다. IP를 임의로 확정하거나 사용 중인 Wi-Fi/기본 경로를 끊지 말라. 두 장비가 통신 가능한 같은 유선망을 구성하고 SSH, PTP, DDS UDP 통신을 실제로 확인한다. DDS application domain은 72, 카메라 내부 domain은 73이며 관련 설정은 실행기가 제공한다.

Wi-Fi를 PTP 인터페이스로 사용하지 않는다. 하드웨어 timestamp가 없으면 유선 NIC의 software-transmit/receive 지원을 확인하고 software PTP를 사용한다. 설치된 버전의 도움말과 [ptp4l 공식 문서](https://www.linuxptp.org/documentation/ptp4l/)를 확인하라.

아래 `REMOTE_INTERFACE`는 검증된 실제 유선 인터페이스 이름으로 바꾼다. 기존 ptp4l 프로세스/서비스와 중복 실행하지 않는다. 이 노트북은 master이므로 slave 옵션 `-s`를 넣지 않는다.

```bash
sudo ptp4l -i REMOTE_INTERFACE -S -m --priority1 10
```

실행을 유지할 터미널 또는 기존 서비스 구성을 사용하고 실행 방법·로그 경로·종료 방법을 보고한다. software 방식에서는 PHC용 phc2sys를 추가하지 않는다. 같은 시스템 시계를 여러 servo가 동시에 조정하지 않도록 기존 시간 서비스를 조사한다. 상대 slave의 설정은 상대 작업으로 남기고, master의 시간 기준까지 무조건 끄지 않는다.

실제 작업 사용자 jucpark로 다음 조회가 가능해야 한다. 현재 코드는 pmc 기본 UDS 경로를 사용하므로 root에서만 성공하거나 다른 소켓에서만 성공하는 상태를 완료로 처리하지 않는다. 필요한 접근은 제한된 사용자/그룹 권한으로 구성한다.

```bash
pmc -u -b 0 'GET PORT_PROPERTIES_NP' 'GET TIME_STATUS_NP' 'GET TIME_PROPERTIES_DATA_SET'
python3 integration/two_host.py ptp --role remote --interface REMOTE_INTERFACE
```

단독 master의 offset 0은 두 호스트 동기 성공의 근거가 아니다. 제어기의 slave가 연결된 뒤 양쪽 grandmaster identity 일치와 합산 시계 오차 ≤1000 us를 검사해야 한다. 10분 카메라 검증도 별도로 필요하다. 불합격을 숨기거나 threshold를 완화하지 말라.

## 4. ORB용 conda/ROS 환경 준비

conda가 PATH에 없으면 실제 설치 위치를 조사한다. 기존 `realsense` 환경이 있으면 먼저 활성화하고 버전/패키지를 확인한다. 기본 conda_sh 후보는 `/home/jucpark/miniconda3/etc/profile.d/conda.sh`이며 실제 경로를 보고해야 한다.

필요한 환경 계약은 ROS 2 Jazzy, `rmw_fastrtps_cpp`, colcon/C++14 빌드 도구, RealSense 카메라와 messages, rosbag2 Python/저장 플러그인, numpy/PyYAML, ORB의 OpenCV/Eigen/Pangolin/OpenGL/Boost 등이다. 정확한 의존성은 소스의 package.xml/CMakeLists.txt와 import, 설치된 패키지를 확인해서 결정하라. `sam6d_ws/environment.yml` 전체를 원격 환경 설치용으로 사용하지 말라.

환경이 없으면 현재 OS에 맞는 conda/ROS 공급 경로를 공식 문서에서 확인해 설치한다. OS 버전이 맞지 않는 ROS apt 저장소를 섞지 말라. 제어기 환경과의 호환성을 확인할 자료가 없으면 필요한 패키지/버전 목록 또는 제어기의 환경 export를 구체적으로 요청하라. 환경 설치가 끝나기 전에는 실행 준비 완료로 표시하지 말라.

활성화 후 아래 검사를 수행한다. 환경명과 source 경로는 실제 값으로 바꾼다.

```bash
conda activate realsense
python --version
colcon --help
cmake --version
python -c 'import rclpy, rosbag2_py, numpy, yaml'
printenv ROS_DISTRO
ros2 pkg prefix rmw_fastrtps_cpp
ros2 pkg prefix realsense2_camera
ros2 interface show realsense2_camera_msgs/msg/RGBD
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show std_msgs/msg/String
git ls-remote https://github.com/ldh8628/sam6d-object-memory.git refs/heads/two-host-v1-migration
```

ROS_DISTRO는 jazzy여야 한다. 최종 message definition hash 비교는 제어기 배포 검사가 담당한다. ORB vocabulary는 제어기가 checksum 검증 후 rsync한다. 임의 파일로 대체하지 않는다. 초기 ORB 빌드와 self-test도 제어기의 배포 명령이 담당하므로 성공 manifest를 직접 작성하지 않는다.

## 5. 카메라와 저장 공간

RealSense 도구로 실제 연결된 장비와 serial `253822302376`을 확인한다. USB 권한·연결 속도·드라이버 문제는 환경 설정으로 해결한다. 장비가 없으면 물리적 연결 필요로 보고한다. SAM 카메라는 상대 노트북에 연결한다.

두 카메라 사이의 hardware sync 케이블과 고정된 상대 배치가 필요하다. 네트워크 PTP 연결과 카메라 sync 케이블은 별개다. runtime worker가 mode 1과 serial을 읽어 확인하므로 독립 카메라 프로세스를 계속 띄워 장치를 점유하지 말라. mode 3 실패 시 mode 2로 자동 변경하지 않는다.

이 노트북은 SLAM bag뿐 아니라 전송받는 SAM bag, atlas, dense map과 staging 사본을 저장한다. 촬영 설정 기준 필요 공간을 계산해 여유를 보고한다. 기존 산출물을 임의 삭제하지 않는다.

## 6. 제어기에 돌려줄 결과와 다음 단계

확인 결과를 `output/remote_readiness.md`에 저장한다. 저장소가 아직 없거나 기존 경로를 사용할 수 없다면 저장소 밖의 별도 작업 폴더에 저장한다. 보고서를 Git에 commit/push하지 않는다. 사용자에게 핵심 결과를 복사 가능한 텍스트로 제공한다.

보고서에는 각 항목을 PASS / FAIL / NOT_TESTED로 구분하고 증거 명령·결과를 적는다.

- 실제 hostname, OS/아키텍처, 사용자, 유선/Wi-Fi IP, 유선 인터페이스와 timestamp 지원
- SSH 주소/포트, key 인증 준비, **제어기에서 실제 접속 확인 여부**
- 저장소 경로/origin/HEAD/dirty 상태, GitHub 접근 여부
- conda_sh 절대 경로, 환경명, ROS/RMW/RealSense 버전과 의존성 검사 결과
- PTP 방식·서비스·인터페이스·port state·grandmaster identity·offset, 일반 사용자 pmc 성공 여부
- 상대 slave 미연결이면 양쪽 동기 검증은 NOT_TESTED
- 카메라 serial/연결 상태, runtime sync mode 검증 여부, 디스크 여유
- 수정한 시스템 설정과 복구 방법, 남은 물리 작업/인증/제어기 작업
- 제어기의 two_host.local.yaml에 넣을 ssh.target/port/remote_root, network.remote_ip/remote_ptp_interface, environment.remote_conda_sh/orb_env의 **확인된 값만**

준비 이후 아래 명령은 **SAM 제어기에서 실행**한다. 이 원격 노트북에서 실행하지 말라. 이 문서가 추가된 HEAD는 기존 `two-host-v1` 태그보다 새로우므로, 제어기는 GitHub에 push된 현재 clean HEAD의 40자리 SHA로 배포한다. 태그를 강제로 옮기지 않는다.

```bash
cd /home/etri/sam6d_object_memory
python3 integration/deploy_remote.py \
  --config integration/two_host.local.yaml --ref "$(git rev-parse HEAD)"

conda activate sam6d
python integration/create_map_urdf.py \
  --two-host-config integration/two_host.local.yaml \
  --name SESSION_001 --input-check-seconds 600

python integration/run_object_memory_realtime.py \
  --two-host-config integration/two_host.local.yaml \
  --map-dir output/SESSION_001 --view --record
```

세션명은 기존 결과와 겹치지 않게 정한다. 배포 성공 후에만 촬영하고, 10분 동기/맵 검증 성공 후에만 실시간 실행한다. 카메라/ORB 시작과 종료는 제어기가 수행한다. 양쪽 deployment.json 일치, 10분 동기 검증, 정지·직선·저속/빠른 yaw 품질 측정, LAN/PTP/ORB 장애 검증은 별도의 현장 완료 조건이며 원격 환경 준비만으로 완료했다고 보고하지 말라.
