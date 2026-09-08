#!/usr/bin/env python3
"""Exercise startup decisions with fake commands; never change a real connection."""
import json
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import remote_codex


def main():
    source = Path(__file__).with_name('two_host_dev.sh')
    subprocess.run(['bash', '-n', str(source)], check=True)
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / 'integration').mkdir()
        launcher = root / 'integration/two_host_dev.sh'
        shutil.copyfile(source, launcher)
        commands = root / 'commands'; commands.mkdir()
        stub = '''#!/usr/bin/python3
import json,os,sys
from pathlib import Path
name=Path(sys.argv[0]).name
args=sys.argv[1:]
with open(os.environ['TEST_TRACE'],'a') as f: f.write(json.dumps([name,*args])+'\\n')
if name=='nmcli':
    if 'connection.interface-name' in args: print('enx00e04caa7ca7')
    elif 'GENERAL.CONNECTION' in args: print(os.environ.get('TEST_PROFILE','sam-ptp-ipv6-slave'))
elif name=='ip': print('4: enx00e04caa7ca7 inet6 fe80::1234/64 scope link')
elif name=='ssh': sys.exit(int(os.environ.get('TEST_SSH_EXIT','0')))
elif name=='cat' and args==['/sys/class/net/enx00e04caa7ca7/carrier']: print('1')
elif name=='cat': os.execv('/bin/cat',['cat',*args])
'''
        for name in ('nmcli', 'ip', 'ssh', 'codex', 'cat'):
            p = commands / name; p.write_text(stub); p.chmod(0o755)
        env = dict(os.environ, PATH=str(commands) + ':' + os.environ['PATH'], TEST_TRACE=str(root / 'trace'))
        env.pop('CODEX_THREAD_ID', None)
        # NIC detection deliberately uses the actual SAM device; all mutable commands are fake.
        assert Path('/sys/class/net/enx00e04caa7ca7').is_dir(), 'Run startup checks on the SAM laptop.'
        def invoke(*args, **changes):
            (root / 'trace').write_text('')
            result = subprocess.run(['bash', str(launcher), *args], env=dict(env, **changes), capture_output=True, text=True)
            calls = [json.loads(line) for line in (root / 'trace').read_text().splitlines()]
            return result, calls
        result, calls = invoke('--check-only')
        assert result.returncode == 0, result.stderr
        assert not any(c[0] == 'codex' or c[0] == 'nmcli' and 'up' in c for c in calls)
        result, calls = invoke('--connect-only', TEST_PROFILE='--')
        assert result.returncode == 0, result.stderr
        assert sum(c[0] == 'nmcli' and 'up' in c for c in calls) == 1
        assert not any(c[0] in ('ssh', 'codex') for c in calls)
        result, calls = invoke(TEST_SSH_EXIT='255')
        assert result.returncode != 0 and not any(c[0] == 'codex' for c in calls)
        state = root / 'output/remote_codex'; state.mkdir(parents=True)
        session = '12345678-1234-1234-1234-123456789abc'
        (state / 'controller_session.txt').write_text(session + '\n')
        result, calls = invoke()
        assert result.returncode == 0, result.stderr
        codex = [c for c in calls if c[0] == 'codex'][0]
        assert codex[1] == 'resume' and session in codex and '--resume-latest' in codex[-1]
        (state / 'controller_session.txt').write_text('invalid-id')
        result, calls = invoke()
        assert result.returncode != 0 and not any(c[0] == 'codex' for c in calls)
        # Exercise the actual guard with real OS locks, including stale files.
        lines = source.read_text().splitlines()
        start = next(i for i, line in enumerate(lines) if line.strip().startswith('writer_lock='))
        end = next(i for i, line in enumerate(lines) if line.strip().startswith('exec codex resume'))
        guard = 'set -euo pipefail\nwriter_lock=$1\n' + '\n'.join(lines[start + 1:end]) + '\necho RESUME_ALLOWED\n'
        lock_path = root / 'writer.lock'
        def probe_guard():
            return subprocess.run(['bash', '-c', guard, 'test', str(lock_path)], capture_output=True, text=True, check=True).stdout
        assert 'RESUME_ALLOWED' in probe_guard()
        with lock_path.open('w') as lock:
            assert 'RESUME_ALLOWED' in probe_guard()
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            output = probe_guard()
            assert '사용 중' in output and 'RESUME_ALLOWED' not in output
            fcntl.flock(lock, fcntl.LOCK_UN)
            assert 'RESUME_ALLOWED' in probe_guard()
        original_root = remote_codex.ROOT
        try:
            remote_codex.ROOT = root
            cfg = {'ssh': {'target': 'slam-codex', 'remote_root': '/remote/project'}}
            for i, (target, status) in enumerate([('slam-codex', 'completed'), ('other', 'completed'), ('slam-codex', 'failed')]):
                p = state / str(i) / 'report.json'; p.parent.mkdir()
                p.write_text(json.dumps(dict(runner='remote_codex_v1', target=target, remote_root='/remote/project', status=status, session_id=session)))
                os.utime(p, (i + 1, i + 1))
            assert remote_codex.latest_report(cfg) == state / '0/report.json'
            assert remote_codex.latest_report({'ssh': dict(target='missing', remote_root='/remote/project')}) is None
        finally:
            remote_codex.ROOT = original_root
    print('PASS: connection preservation, SSH failure, session resume, active/stale writer locks, report selection.')


if __name__ == '__main__':
    main()
