#!/usr/bin/env bash
# systemd retries this check until the USB link is ready. No clock/process changes.
set -euo pipefail
nic=${1:?interface required}
role=${2:?SAM or SLAM required}
[[ "$role" == SAM || "$role" == SLAM ]]
[[ "$nic" == enx00e04caa7ca7 || "$nic" == enx00e04cbaf0a3 ]]
[[ $(cat "/sys/class/net/$nic/carrier") == 1 ]] || { echo 'Waiting for USB cable.' >&2; exit 1; }
addresses=$(ip -6 -o address show dev "$nic" scope link)
[[ "$addresses" == *'inet6 fe80:'* && "$addresses" != *tentative* && "$addresses" != *dadfailed* ]] || {
    echo 'Waiting for usable IPv6 link-local address.' >&2; exit 1;
}
# Check executable paths: the old master is renamed remote-slam-ptp4l.
for process in /proc/[0-9]*/exe; do
    executable=$(readlink "$process" 2>/dev/null) || continue
    case "${executable##*/}" in
        *ptp4l|*ptp4l\ \(deleted\)|phc2sys|chronyd|ntpd|timemaster)
            echo "Existing clock daemon: $executable ($process). Refusing a duplicate." >&2
            exit 1 ;;
        systemd-timesyncd)
            [[ "$role" != SAM ]] || { echo 'SAM timesyncd is still running.' >&2; exit 1; } ;;
    esac
done
