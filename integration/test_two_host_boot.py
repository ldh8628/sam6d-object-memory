#!/usr/bin/env python3
"""Check boot units and clock-owner guards without root or changing host services."""
import os
from pathlib import Path
import subprocess
import tempfile
from unittest.mock import patch

from install_two_host_boot import HOSTS, UNIT, install, unit_text


def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        # Isolate validation from unrelated host units (e.g. local NVIDIA ordering cycles).
        for name in ('sysinit.target', 'basic.target', 'shutdown.target'):
            (root / name).write_text('[Unit]\nDescription=Test dependency\nDefaultDependencies=no\n')
        (root / 'NetworkManager.service').write_text('[Service]\nExecStart=/usr/bin/true\n')
        for role in HOSTS:
            text = unit_text(role)
            assert 'Restart=on-failure' in text and 'StartLimitIntervalSec=0' in text
            assert 'Group=' + HOSTS[role][2] in text
            conflicts = next(line for line in text.splitlines() if line.startswith('Conflicts='))
            assert ('systemd-timesyncd.service' in conflicts) == (role == 'SAM')
            assert 'remote-slam-ptp.service' in conflicts
            assert ('--masterOnly 1' in text) == (role == 'SLAM')
            assert (' -s\n' in text) == (role == 'SAM')
            # Only executable paths are substituted: syntax/dependencies remain the real unit.
            unit = root / UNIT
            unit.write_text(text.replace('/usr/local/libexec/two-host-ptp-check', '/usr/bin/true')
                           .replace('/usr/local/libexec/two-host-ptp4l', '/usr/bin/true'))
            subprocess.run(['systemd-analyze', 'verify', str(unit)], check=True,
                           env=dict(os.environ, SYSTEMD_UNIT_PATH=str(root)))
        nic = HOSTS['SAM'][0]
        carrier = root / 'sys' / nic / 'carrier'; carrier.parent.mkdir(parents=True)
        carrier.write_text('1\n')
        commands = root / 'commands'; commands.mkdir()
        ip = commands / 'ip'
        ip.write_text('#!/bin/bash\necho "inet6 fe80::1234/64 scope link ${TEST_ADDRESS_STATE:-}"\n')
        ip.chmod(0o755)
        processes = root / 'proc'; processes.mkdir()
        script = Path(__file__).with_name('two_host_ptp_check.sh').read_text()
        script = script.replace('/sys/class/net', str(root / 'sys')).replace('/proc/', str(processes) + '/')
        checker = root / 'check.sh'; checker.write_text(script)
        env = dict(os.environ, PATH=str(commands) + ':' + os.environ['PATH'])
        def check(role='SAM', **changes):
            return subprocess.run(['bash', str(checker), nic, role], env=dict(env, **changes),
                                  capture_output=True, text=True)
        assert check().returncode == 0
        assert check(TEST_ADDRESS_STATE='tentative').returncode != 0
        assert check(TEST_ADDRESS_STATE='dadfailed').returncode != 0
        carrier.write_text('0\n')
        assert check().returncode != 0
        carrier.write_text('1\n')
        process = processes / '123' / 'exe'; process.parent.mkdir()
        for daemon in ('remote-slam-ptp4l', 'two-host-ptp4l', 'ptp4l (deleted)', 'phc2sys', 'chronyd', 'ntpd', 'timemaster'):
            process.symlink_to('/bin/' + daemon)
            assert check().returncode != 0, daemon
            process.unlink()
        process.symlink_to('/lib/systemd/systemd-timesyncd')
        assert check().returncode != 0
        assert check('SLAM').returncode == 0
        # Redirect all installation targets and commands: no real /etc, sudo or NM writes.
        filesystem = root / 'installation'; filesystem.mkdir()
        stage = root / 'stage'; stage.mkdir()
        for name in (UNIT, 'two-host-ptp.conf', 'two-host-ptp4l', 'two-host-ptp-check'):
            (stage / name).write_text('initial\n')
        def mapped_path(value):
            path = Path(value)
            return path if path.is_relative_to(root) else filesystem / str(path).lstrip('/')
        def fake_query(*args):
            return '-1' if 'connection.autoconnect-retries' in args else 'no'
        with patch('install_two_host_boot.Path', side_effect=mapped_path), \
             patch('install_two_host_boot.run', side_effect=fake_query), \
             patch('install_two_host_boot.subprocess.run', return_value=subprocess.CompletedProcess([], 0)) as commands_run:
            manifest = dict(role='SAM', profile=HOSTS['SAM'][1])
            install(stage, manifest)
            config = filesystem / 'etc/two-host-ptp.conf'
            config.write_text('calibrated\n')
            install(stage, manifest)
            assert config.read_text() == 'calibrated\n'
            calls = [call.args[0] for call in commands_run.call_args_list]
            assert ['systemctl', 'enable', UNIT] in calls
            assert ['systemctl', 'disable', 'systemd-timesyncd.service'] in calls
            assert all(not any(arg in ('--now', 'start', 'stop', 'restart', 'up', 'down') for arg in call) for call in calls)
            for name in ('two-host-ptp4l', 'two-host-ptp-check'):
                assert (filesystem / 'usr/local/libexec' / name).stat().st_mode & 0o777 == 0o755
    print('PASS: both systemd units, link readiness, renamed/deleted daemons, SAM clock ownership, master NTP.')
    print('PASS: install enables next boot only; repeated install preserves calibration.')


if __name__ == '__main__':
    main()
