# SSH로 원격 Codex 작업 실행

현재 SAM Codex가 작업을 결정하고 `remote_codex.py`로 SLAM Codex를 호출한다. 원격은 별도 작업 세션을 사용하며, 결과와 세션 ID를 SAM의 `output/remote_codex/RUN_ID/`에 저장한다. 소스 수정·commit·배포는 SAM에서 수행한다.

## 재부팅 후 시작

같은 USB NIC와 케이블을 연결하고 **SLAM → SAM 순서**로 실행한다. 앱 목록/바탕화면의 SLAM **유선 연결**, SAM **개발 재개** 아이콘을 사용할 수 있다. 두 아이콘은 같은 `integration/two_host_dev.sh`를 실행하며 연결된 NIC로 역할을 판단한다.

아이콘 대신 양쪽 터미널에서 다음 한 줄을 실행해도 된다.

```bash
bash ~/sam6d_object_memory/integration/two_host_dev.sh
```

SLAM에서는 `remote-slam-wired` 연결을 활성화하고 끝난다. SAM에서는 `sam-ptp-ipv6-slave` 연결을 활성화한 뒤 SSH와 원격 Codex 로그인을 확인하고 현재 개발 대화를 재개한다. 이미 활성화된 연결은 재연결하지 않는다. Wi-Fi, 저장된 IP 설정, 부팅 자동 연결 여부와 PTP 프로세스는 변경하지 않는다.

SAM의 대화 ID는 Git에서 제외된 `output/remote_codex/controller_session.txt`에 저장한다. 파일이 없으면 재개 안내가 포함된 새 대화를 연다. 현재 대화를 다른 창에서 동시에 열지 않도록, 개발 재개 아이콘은 기존 Codex를 종료한 뒤 사용한다. 세션 파일과 `output/remote_codex/` 결과를 유지하면 다음 부팅에도 맥락을 이어갈 수 있다.

Codex에 전달되는 재개 안내에는 실행 문서 읽기, Git/SSH 상태 확인, 원격 `--resume-latest` 사용 지시가 포함되어 있어 별도 프롬프트를 외울 필요가 없다. 연결 오류가 나면 Codex를 시작하지 않고 원인을 표시한다.

연결만 켜거나 Codex를 열지 않고 확인할 수도 있다.

```bash
bash integration/two_host_dev.sh --connect-only
bash integration/two_host_dev.sh --check-only
```

이 시작기는 개발 연결을 복구한다. PTP/카메라 실행과 1 ms 오차 검증은 실험 전에 별도로 수행한다. 완전한 전원 재부팅 시험은 아직 수행하지 않았으며, 양쪽 현재 환경의 연결 재사용과 SAM의 SSH/Codex 확인을 실제 검증했다.

## 원격 작업 호출

준비 조건은 OpenSSH key 인증, 원격 로그인 셸에서 사용 가능한 Codex와 로그인 상태, 기존 프로젝트 경로, `flock`과 `timeout`이다. SSH 별칭은 로컬 OpenSSH 설정에서 관리한다. `two_host.local.yaml`의 `ssh.target`에 별칭을 사용할 수 있다. IPv6 link-local 별칭에는 로컬 인터페이스 scope가 필요하며, ROS DDS용 IPv4 설정은 별도로 검증해야 한다.

```bash
python3 integration/remote_codex.py \
  --config integration/two_host.local.yaml \
  --prompt-file output/task.txt
```

기본 모델은 요청된 `gpt-6-astra`, reasoning effort는 `high`, sandbox는 `read-only`, 제한 시간은 300초다. 필요한 출력/빌드 작업에는 명시적으로 `--sandbox workspace-write`를 사용한다. 승인 입력을 기다리지 않으며 부족한 권한은 작업 결과에 보고하도록 지시한다. 시스템 관리자 권한은 별도 설정이 필요하고 이 실행기가 부여하지 않는다.

`events.jsonl`은 원격 Codex 이벤트, `stderr.log`는 진단 로그, `final.md`는 응답, `request.json`은 전달 작업, `report.json`은 실행 상태와 세션 ID다. `status: completed`는 Codex 응답이 완료됐다는 뜻이며, 하드웨어 검증 PASS를 뜻하지 않는다. 실제 판정은 명령 결과와 측정 근거를 확인한다.

이 실행기가 만든 전용 세션은 이전 report를 지정해 이어간다.

```bash
python3 integration/remote_codex.py \
  --config integration/two_host.local.yaml \
  --prompt-file output/followup.txt \
  --resume-report output/remote_codex/RUN_ID/report.json
```

보고서 경로를 기억하지 않으려면 `--resume-report` 대신 `--resume-latest`를 사용한다. 같은 SSH 대상/프로젝트의 완료된 작업 중 가장 최근 보고서를 선택하며, 없으면 새 전용 세션으로 시작한다. 사람의 다른 대화는 선택하지 않는다.

같은 원격 프로젝트의 자동 작업은 파일 잠금으로 하나만 실행한다. 이미 실행 중이면 종료 코드 75로 실패한다. 사람의 별도 Codex 세션까지 잠그지는 않으므로 자동 작업 중 동일 자원 변경을 중복 지시하지 않는다. 임의의 기존 대화나 `--last`를 사용하지 않는다.

원격에도 시간 제한을 두므로 SSH가 끊겨도 무제한 실행되지 않는다. 로컬 timeout/실패 로그를 보존하며 작업을 자동 재시도하지 않는다. 시간 제한은 Codex 호출에 적용되며 작업이 별도로 시작한 시스템 서비스의 정리까지 보장하지 않는다. 이 스크립트 자체는 상주 제어기가 아니며, 현재 Codex가 필요한 작업을 한 번씩 호출하는 방식이다.

## 실제 연결 검증

2026-09-08, `slam-codex`를 통해 jucpark-device에 SSH 공개키 인증 성공, Codex CLI 0.153.4 및 ChatGPT 로그인 확인. 실제 Astra 작업을 호출하고 세션 ID와 응답을 회수했으며 같은 세션의 후속 작업도 성공했다. 별도의 두 번째 잠금 요청은 코드 75로 차단됐다.

읽기 전용 Codex sandbox에서는 host netlink 접근이 차단되고 호스트 IPv6가 보이지 않는 현상이 관찰됐다. 따라서 해당 sandbox의 프로세스/네트워크 관측만으로 호스트 상태를 단정하지 않는다. 필요하면 SAM 제어기가 직접 SSH로 읽은 실제 명령 결과를 원격 Codex의 후속 프롬프트에 포함한다. 프로세스가 원격 직접 SSH에서도 관측되지 않으면 별도 실행 상태 확인이 필요하다.

이 장비의 실제 master 바이너리는 `/usr/local/libexec/remote-slam-ptp4l`이므로 `pgrep -x ptp4l`만으로는 찾을 수 없다. 전체 명령줄과 PTP 응답을 함께 확인한다. 실제 SSH에서 master 프로세스와 SAM의 SLAVE/GM 일치를 확인했다.

```bash
python3 integration/remote_codex.py --self-test
python3 integration/test_two_host_dev.py
```

[Codex 비대화형 실행 공식 문서](https://learn.chatgpt.com/docs/non-interactive-mode)
