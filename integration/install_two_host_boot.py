#!/usr/bin/env python3
"""Prepare or install persistent wired/PTP startup without restarting live PTP."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import pwd
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
UNIT = 'two-host-ptp.service'
MARKER = '# Managed by sam6d_object_memory/integration/install_two_host_boot.py\n'
HOSTS = {
    'SAM': ('enx00e04caa7ca7', 'sam-ptp-ipv6-slave', 'etri'),
    'SLAM': ('enx00e04cbaf0a3', 'remote-slam-wired', 'jucpark'),
}


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def unit_text(role):
    nic, _, user = HOSTS[role]
    conflicts = 'ptp4l.service phc2sys.service chrony.service chronyd.service ntp.service ntpd.service timemaster.service remote-slam-ptp.service'
    if role == 'SAM':
        conflicts += ' systemd-timesyncd.service'
    flags = '-s' if role == 'SAM' else '--priority1 10 --masterOnly 1'
    return MARKER + f'''[Unit]
Description=Two-host {role} software IPv6 PTP
Wants=NetworkManager.service
After=NetworkManager.service {conflicts}
Conflicts={conflicts}
StartLimitIntervalSec=0

[Service]
Type=simple
Group={user}
UMask=0007
ExecStartPre=/usr/local/libexec/two-host-ptp-check {nic} {role}
ExecStart=/usr/local/libexec/two-host-ptp4l -f /etc/two-host-ptp.conf -i {nic} -S -6 -m {flags}
Restart=on-failure
RestartSec=10
TimeoutStartSec=15
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
'''


def prepare(destination):
    roles = [role for role, (nic, _, _) in HOSTS.items() if Path('/sys/class/net', nic).is_dir()]
    if len(roles) != 1:
        raise ValueError('Connect exactly one known SAM/SLAM USB NIC.')
    role = roles[0]
    nic, profile, user = HOSTS[role]
    pwd.getpwnam(user)
    if run('nmcli', '-g', 'connection.interface-name', 'connection', 'show', profile) != nic:
        raise ValueError('Saved NetworkManager profile/interface mismatch.')
    binary = (ROOT / 'output/ptp_slave_setup/linuxptp-v3.1.1-usb/ptp4l' if role == 'SAM'
              else Path('/usr/local/libexec/remote-slam-ptp4l'))
    version = run(str(binary), '-v')
    if not version.startswith('3.1.1'):
        raise ValueError(f'Expected patched linuxptp 3.1.1, got {version}')
    # Verify the USB compatibility fix is in the actual binary, not just its filename.
    if b'omits software TX capability' not in binary.read_bytes():
        raise ValueError('USB compatibility patch marker missing.')
    destination.mkdir(parents=True, exist_ok=True)
    (destination / UNIT).write_text(unit_text(role))
    (destination / 'two-host-ptp.conf').write_text(MARKER + '''[global]
# Keep the currently tested 1 second intervals; tune separately with measurements.
logSyncInterval 0
logMinDelayReqInterval 0
''')
    shutil.copyfile(ROOT / 'integration/two_host_ptp_check.sh', destination / 'two-host-ptp-check')
    (destination / 'two-host-ptp-check').chmod(0o755)
    shutil.copyfile(binary, destination / 'two-host-ptp4l')
    (destination / 'two-host-ptp4l').chmod(0o755)
    manifest = dict(role=role, interface=nic, profile=profile, user=user, binary_source=str(binary),
                    binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), version=version)
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    subprocess.run(['bash', '-n', str(destination / 'two-host-ptp-check')], check=True)
    return manifest


def install(stage, manifest):
    state_dir = Path('/var/lib/two-host-ptp')
    state_file = state_dir / 'installation.json'
    targets = {
        UNIT: Path('/etc/systemd/system') / UNIT,
        'two-host-ptp.conf': Path('/etc/two-host-ptp.conf'),
        'two-host-ptp-check': Path('/usr/local/libexec/two-host-ptp-check'),
        'two-host-ptp4l': Path('/usr/local/libexec/two-host-ptp4l'),
    }
    if not state_file.exists() and any(p.exists() or p.is_symlink() for p in targets.values()):
        raise ValueError('Unmanaged target already exists; refusing to overwrite.')
    if state_file.exists():
        state = json.loads(state_file.read_text())
        if state['role'] != manifest['role']:
            raise ValueError('Installed role differs from detected NIC.')
    else:
        state = dict(manifest)
        state['previous_autoconnect'] = run('nmcli', '-g', 'connection.autoconnect', 'connection', 'show', manifest['profile'])
        state['previous_retries'] = run('nmcli', '-g', 'connection.autoconnect-retries', 'connection', 'show', manifest['profile'])
        state['timesyncd_enabled'] = subprocess.run(['systemctl', 'is-enabled', '--quiet', 'systemd-timesyncd.service']).returncode == 0
        state_dir.mkdir(mode=0o755, parents=True, exist_ok=True)
        state_file.write_text(json.dumps(state, indent=2) + '\n')
    for name, target in targets.items():
        # Preserve local interval calibration on repeated installation.
        if name == 'two-host-ptp.conf' and target.exists():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as f:
            temporary = Path(f.name)
            f.write((stage / name).read_bytes())
        temporary.chmod(0o755 if name in ('two-host-ptp-check', 'two-host-ptp4l') else 0o644)
        temporary.replace(target)
    subprocess.run(['systemd-analyze', 'verify', str(targets[UNIT])], check=True)
    subprocess.run(['systemctl', 'daemon-reload'], check=True)
    # Enable only. Do not replace the existing foreground slave or transient master.
    subprocess.run(['systemctl', 'enable', UNIT], check=True)
    if manifest['role'] == 'SAM':
        subprocess.run(['systemctl', 'disable', 'systemd-timesyncd.service'], check=True)
    subprocess.run(['nmcli', 'connection', 'modify', manifest['profile'],
                    'connection.autoconnect', 'yes', 'connection.autoconnect-retries', '0'], check=True)
    print('설치 완료. 현재 PTP는 유지되며, 다음 부팅부터 유선 연결과 PTP가 자동 시작됩니다.')
    print('상태: systemctl status two-host-ptp.service | 로그: journalctl -u two-host-ptp.service -b')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', action='store_true', help='requires sudo; enable for next boot, never restart now')
    parser.add_argument('--status', action='store_true', help='read-only startup and live PTP check')
    args = parser.parse_args()
    if args.status:
        if args.install:
            parser.error('--status and --install are mutually exclusive')
        from two_host import ptp_probe
        for role, (nic, _, _) in HOSTS.items():
            if not Path('/sys/class/net', nic).is_dir():
                continue
            enabled = subprocess.run(['systemctl', 'is-enabled', '--quiet', UNIT], stderr=subprocess.DEVNULL).returncode == 0
            print(f'[{role}] PTP 부팅 자동 시작: {"설정됨" if enabled else "미설치 (최초 sudo 설치 필요)"}')
            try:
                sample = ptp_probe(nic, 'local' if role == 'SAM' else 'remote')
                if sample['grandmaster_id'] != '00e04c.fffe.baf0a3':
                    raise ValueError('예상 grandmaster와 다릅니다.')
                if sample['timestamping'] != 'software':
                    raise ValueError('예상 software timestamp 방식과 다릅니다.')
                if role == 'SAM' and subprocess.run(['systemctl', 'is-active', '--quiet', 'systemd-timesyncd.service']).returncode == 0:
                    raise ValueError('SAM에서 timesyncd와 PTP가 동시에 실행 중입니다.')
                print(f'[{role}] PTP {sample["port_state"]}, GM {sample["grandmaster_id"]}, offset {sample["offset_us"]:.1f} us')
                print('단일 표본입니다. 카메라 실행 전 양쪽 1 ms 연속 검증이 필요합니다.')
            except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as exc:
                print(f'[{role}] PTP 준비 미완료: {exc}')
            return
        raise ValueError('Known USB NIC missing.')
    if args.install and os.geteuid() != 0:
        parser.error('Run: sudo python3 integration/install_two_host_boot.py --install')
    if args.install:
        with tempfile.TemporaryDirectory(prefix='two-host-boot-') as directory:
            stage = Path(directory)
            install(stage, prepare(stage))
    else:
        stage = ROOT / 'output/two_host_boot_setup'
        manifest = prepare(stage)
        print(json.dumps(manifest, indent=2))
        print(f'준비 완료 (시스템 변경 없음): {stage}')
        print(f'설치: sudo python3 {ROOT}/integration/install_two_host_boot.py --install')


if __name__ == '__main__':
    main()
