"""Hardware-free checks: python -m unittest discover -s integration -p test_create_map_urdf_split.py."""
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import unittest


class SplitEntrypointChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / 'project with spaces'
        integration = self.root / 'integration'
        integration.mkdir(parents=True)
        for name in ('create_map_urdf_split.py', 'two_host.py'):
            shutil.copyfile(Path(__file__).with_name(name), integration / name)
        self.launcher = integration / 'create_map_urdf_split.py'
        (integration / 'create_map_urdf.py').write_text('''
import json, os, sys, time
print(json.dumps(dict(argv=sys.argv[1:], pid=os.getpid(), cwd=os.getcwd(),
    conda=os.environ.get('SPLIT_CONDA'), overlay=os.environ.get('SPLIT_OVERLAY'),
    domain=os.environ.get('ROS_DOMAIN_ID'))), flush=True)
if os.environ.get('SPLIT_WAIT'):
    time.sleep(60)
else:
    print(sys.stdin.readline().rstrip(), flush=True)
    sys.exit(int(os.environ.get('SPLIT_EXIT', '0')))
''')
        bindir = self.base / 'bin'
        bindir.mkdir()
        (bindir / 'python').symlink_to(sys.executable)
        conda = self.base / 'conda profile.sh'
        conda.write_text('''conda() {
    test "$1" = activate || return 1
    export SPLIT_CONDA="$2"
    export PATH="$SPLIT_BIN:$PATH"
    : > "$SPLIT_ACTIVATED"
}
''')
        overlay = self.root / 'orbslam_ws/install_jazzy/setup.bash'
        overlay.parent.mkdir(parents=True)
        overlay.write_text('export SPLIT_OVERLAY=loaded\n')
        self.config = dict(
            ssh=dict(target='user@slam', remote_root='/remote/project'),
            network=dict(local_ip='192.168.1.1', remote_ip='192.168.1.2', ros_domain_id=72,
                         local_ptp_interface='eth0', remote_ptp_interface='eth1', ptp_max_offset_us=1000),
            cameras=dict(slam_serial='253822302376', sam_serial='253822301680',
                         slam_sync_mode=1, sam_sync_mode=3),
            environment=dict(local_conda_sh=str(conda), orb_env='test realsense'))
        self.default_config = integration / 'two_host.local.yaml'
        self.default_config.write_text(json.dumps(self.config))
        self.activated = self.base / 'activated'
        self.env = {**os.environ, 'SPLIT_BIN': str(bindir), 'SPLIT_ACTIVATED': str(self.activated)}
        self.env.pop('SPLIT_WAIT', None)
        self.env.pop('SPLIT_EXIT', None)

    def run_cli(self, *args, **kwargs):
        return subprocess.run([sys.executable, str(self.launcher), *args],
                              cwd=self.base, env=self.env, text=True,
                              capture_output=True, timeout=5, **kwargs)

    def test_default_config_environment_stdin_and_exit_code(self):
        self.env['SPLIT_EXIT'] = '23'
        result = self.run_cli('--name', 'session with spaces', input='Enter input\n')
        self.assertEqual(result.returncode, 23, result.stderr)
        record, echoed = result.stdout.splitlines()
        record = json.loads(record)
        self.assertEqual(record['argv'], [
            '--name=session with spaces', '--two-host-config', str(self.default_config),
            '--baseline', '0.095', '--camera-timeout', '30.0', '--input-check-seconds', '0.0'])
        self.assertEqual((record['cwd'], record['conda'], record['overlay'], record['domain']),
                         (str(self.root), 'test realsense', 'loaded', '73'))
        self.assertEqual(echoed, 'Enter input')

    def test_relative_explicit_config_and_all_options(self):
        config = self.base / 'explicit config.yaml'
        self.default_config.rename(config)
        result = self.run_cli('--name', 'capture', '--config', config.name,
                              '--baseline', '.12', '--camera-timeout', '47.5',
                              '--input-check-seconds', '600', '--input-check-only', input='\n')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout.splitlines()[0])['argv'], [
            '--name=capture', '--two-host-config', str(config), '--baseline', '0.12',
            '--camera-timeout', '47.5', '--input-check-seconds', '600.0', '--input-check-only'])

    def test_help_and_invalid_arguments_never_activate_environment(self):
        result = self.run_cli('--help')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--input-check-only', result.stdout)
        invalid = [[], ['--force'], ['--camera-t', '30'], ['--two-host-config', 'x'],
                   ['--name', ''], ['--name', '../capture'], ['--name', '.'], ['--name', '/tmp'],
                   ['--config', 'missing.yaml']]
        invalid += [[option, value] for option in ('--baseline', '--camera-timeout')
                    for value in ('0', '-1', 'nan', 'inf', 'wrong')]
        invalid += [['--input-check-seconds', value] for value in ('-1', '599', 'nan', 'inf')]
        for argv in invalid:
            with self.subTest(argv=argv):
                result = self.run_cli(*(['--name', 'capture'] if argv else []), *argv)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn('error:', result.stderr)
                self.assertFalse(self.activated.exists())
        for content in ('{}', 'not yaml', '{"ssh": null}'):
            self.default_config.write_text(content)
            result = self.run_cli('--name', 'capture')
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertFalse(self.activated.exists())
        self.default_config.unlink()
        self.assertEqual(self.run_cli('--help').returncode, 0)
        self.assertEqual(self.run_cli('--name', 'capture').returncode, 2)
        self.assertFalse(self.activated.exists())

    def test_exec_preserves_pid_and_delivers_signals(self):
        self.env['SPLIT_WAIT'] = '1'
        for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            with self.subTest(signal=signum):
                process = subprocess.Popen([sys.executable, str(self.launcher), '--name', 'capture'],
                    cwd=self.base, env=self.env, text=True,
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                try:
                    self.assertTrue(select.select([process.stdout], [], [], 5)[0], 'target not ready')
                    record = json.loads(process.stdout.readline())
                    self.assertEqual(record['pid'], process.pid)
                    process.send_signal(signum)
                    _, stderr = process.communicate(timeout=5)
                    self.assertEqual(process.returncode, -signum, stderr)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.communicate()


if __name__ == '__main__':
    unittest.main()
