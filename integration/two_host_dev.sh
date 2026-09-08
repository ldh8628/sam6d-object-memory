#!/usr/bin/env bash
# One entry point on both laptops; no sudo or persistent network changes.
set -euo pipefail
if ! command -v codex >/dev/null 2>&1; then
    export PATH="$HOME/.local/bin:$PATH"
fi
mode=${1:-start}
case "$mode" in
    start|--connect-only|--check-only) ;;
    *) echo 'Usage: two_host_dev.sh [--connect-only|--check-only]' >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo 'Too many arguments.' >&2; exit 2; }
fail() { echo "오류: $*" >&2; exit 1; }
finish() {
    rc=$?
    trap - EXIT
    if [[ -t 0 && ( $rc -ne 0 || ${role:-} == SLAM ) ]]; then
        read -r -p 'Enter를 누르면 창을 닫습니다. ' _ || true
    fi
    exit "$rc"
}
trap finish EXIT
(( EUID != 0 )) || fail 'sudo 없이 일반 사용자로 실행하세요.'
root=$(dirname -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")")
sam_nic=enx00e04caa7ca7
slam_nic=enx00e04cbaf0a3
if [[ -d /sys/class/net/$sam_nic && ! -d /sys/class/net/$slam_nic ]]; then
    role=SAM; nic=$sam_nic; profile=sam-ptp-ipv6-slave
elif [[ -d /sys/class/net/$slam_nic && ! -d /sys/class/net/$sam_nic ]]; then
    role=SLAM; nic=$slam_nic; profile=remote-slam-wired
else
    fail '해당 노트북의 USB NIC 하나를 연결하세요. SAM/SLAM 역할을 판별할 수 없습니다.'
fi
for command in nmcli ip; do command -v "$command" >/dev/null || fail "$command 설치가 필요합니다."; done
[[ $(cat "/sys/class/net/$nic/carrier") == 1 ]] || fail "$nic 유선 케이블 연결을 확인하세요."
configured_nic=$(nmcli -g connection.interface-name connection show id "$profile") || fail "$profile 연결 프로필이 없습니다."
[[ "$configured_nic" == "$nic" ]] || fail "$profile 프로필의 인터페이스가 $nic와 다릅니다."
active_profile=$(nmcli -g GENERAL.CONNECTION device show "$nic")
if [[ "$active_profile" != "$profile" ]]; then
    echo "[$role] 유선 연결 활성화: $profile"
    nmcli --wait 20 connection up id "$profile" ifname "$nic" || fail '유선 연결 활성화에 실패했습니다.'
else
    echo "[$role] 유선 연결이 이미 활성화돼 있습니다."
fi
addresses=$(ip -6 -o address show dev "$nic" scope link)
[[ "$addresses" == *'inet6 fe80:'* && "$addresses" != *tentative* && "$addresses" != *dadfailed* ]] || fail '사용 가능한 IPv6 link-local 주소가 없습니다.'
echo "$addresses"
if [[ "$mode" == --connect-only || "$role" == SLAM ]]; then
    echo "[$role] 연결 완료. SAM의 'SAM 개발 재개'를 실행하면 됩니다."
    exit 0
fi
command -v codex >/dev/null || fail '현재 노트북의 Codex 실행 경로를 확인하세요.'
echo '[SAM] SLAM SSH 및 Codex 로그인 확인 중...'
ssh -o BatchMode=yes -o ConnectTimeout=5 slam-codex \
    'bash -lc '\''set -e; hostname; command -v codex; codex --version; codex login status'\''' \
    || fail "SLAM에서 'SLAM 유선 연결'을 먼저 실행하세요. 계속 실패하면 ssh slam-codex 설정/인증을 확인하세요."
[[ "$mode" != --check-only ]] || { echo '[SAM] 개발 연결 확인 완료. Codex는 시작하지 않았습니다.'; exit 0; }
[[ -z ${CODEX_THREAD_ID:-} ]] || fail '실행 중인 Codex 안에서는 --check-only를 사용하세요. 개발 재개는 일반 터미널/아이콘에서 실행하세요.'
prompt='두 노트북 개발을 재개한다. integration/REMOTE_CODEX.md를 읽고 현재 Git/SSH 상태를 확인하라. 원격 작업은 integration/remote_codex.py와 --resume-latest로 이어가라. 호스트 진단은 직접 SSH 관측과 sandbox 관측을 구분하라. 소스 수정과 commit은 SAM에서만 한다. PTP/카메라는 별도 실행 요청이 있을 때 시작하고 이전 실측 PASS를 재부팅 후에도 유효하다고 가정하지 말라. 먼저 현재 상태와 다음 작업을 짧게 보고하라.'
session_file="$root/output/remote_codex/controller_session.txt"
cd -- "$root"
if [[ -s "$session_file" ]]; then
    session=$(cat "$session_file")
    [[ "$session" =~ ^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$ ]] || fail '저장된 controller session ID 형식이 올바르지 않습니다.'
    exec codex resume -C "$root" -m gpt-6-astra -c 'model_reasoning_effort="high"' "$session" "$prompt"
else
    exec codex -C "$root" -m gpt-6-astra -c 'model_reasoning_effort="high"' "$prompt"
fi
