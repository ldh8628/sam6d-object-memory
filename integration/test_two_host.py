#!/usr/bin/env python3
"""Offline deployment/config/lease regression checks; Python standard library only."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import deploy_remote as deploy
import two_host as host


def configuration():
    return dict(ssh=dict(target='tester@host', port=10022, remote_root='/remote/repo'),
                network=dict(local_ip='192.0.2.1', remote_ip='192.0.2.2', ros_domain_id=72,
                             local_ptp_interface='eth0', remote_ptp_interface='eth1', ptp_max_offset_us=1000),
                cameras=dict(slam_serial='123', sam_serial='456', slam_sync_mode=1, sam_sync_mode=3))


class HostChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.load(configuration())

    def load(self, config):
        path = self.root / 'config.json'
        path.write_text(json.dumps(config))
        return host.load_config(path)

    def test_config_validation_and_shell_quoting(self):
        self.assertEqual(self.config['ssh']['port'], 10022)
        plain = configuration(); plain['ssh'].pop('port')
        self.assertEqual(self.load(plain)['ssh']['port'], 22)
        bad_values = [('ssh', 'port', True), ('ssh', 'port', 0), ('ssh', 'port', 65536),
                      ('ssh', 'target', '-oProxyCommand=bad'), ('ssh', 'target', 'host;id'),
                      ('ssh', 'remote_root', '/'), ('ssh', 'remote_root', '/tmp/../root'),
                      ('network', 'local_ip', 'not-an-ip'), ('network', 'remote_ip', '192.0.2.1'),
                      ('network', 'ros_domain_id', True), ('network', 'ros_domain_id', 232),
                      ('network', 'ptp_max_offset_us', float('nan')),
                      ('network', 'ptp_max_offset_us', 1001), ('network', 'local_ptp_interface', 'eth0;id'),
                      ('cameras', 'slam_serial', 123), ('cameras', 'sam_serial', '123'),
                      ('cameras', 'sam_sync_mode', 2)]
        for group, key, value in bad_values:
            with self.subTest(group=group, key=key, value=value):
                cfg = configuration(); cfg[group][key] = value
                with self.assertRaises(ValueError): self.load(cfg)
        for group, key in [('ssh', 'password'), ('environment', 'private_key')]:
            cfg = configuration(); cfg.setdefault(group, {})[key] = 'secret'
            with self.assertRaises(ValueError): self.load(cfg)
        payload = ['python3', '-c', "print('$(touch /tmp/never) `id`')", 'a b', "a'b", 'line\nbreak']
        command = host.ssh_command(self.config, payload)
        self.assertEqual(command[command.index('-p') + 1], '10022')
        self.assertEqual(shlex.split(command[-1]), payload)
        script = host.local_command(self.config, payload, lan=True)[-1]
        self.assertEqual(shlex.split(script.rsplit('exec ', 1)[1]), payload)
        self.assertIn('ROS_DOMAIN_ID=72', script)
        self.assertIn('ROS_STATIC_PEERS=192.0.2.2', script)
        private = host.local_command(self.config, payload)[-1]
        self.assertIn('ROS_DOMAIN_ID=73', private)
        self.assertIn("ROS_STATIC_PEERS=''", private)
        yaml = self.root / 'config.yaml'
        yaml.write_text('ssh:\n  target: host\n  target: again\n')
        with self.assertRaises(ValueError): host.load_config(yaml)

    def test_atomic_json_hashes_and_symlinks(self):
        target = self.root / 'nested' / 'report.json'
        host.atomic_json(target, {'old': 1})
        with self.assertRaises(ValueError): host.atomic_json(target, {'bad': float('inf')})
        self.assertEqual(json.loads(target.read_text()), {'old': 1})
        data = self.root / 'asset'; data.mkdir()
        (data / 'a').write_bytes(b'abc')
        (data / 'sub').mkdir(); (data / 'sub' / 'b').write_bytes(b'def')
        self.assertEqual(host.tree_hashes(data), {'a': hashlib.sha256(b'abc').hexdigest(),
                                                'sub/b': hashlib.sha256(b'def').hexdigest()})
        link = self.root / 'link'; link.symlink_to(data)
        with self.assertRaises(ValueError): host.tree_hashes(link)
        (data / 'escape').symlink_to(target)
        with self.assertRaises(ValueError): host.tree_hashes(data)
        empty = self.root / 'empty'; empty.mkdir()
        with self.assertRaises(ValueError): host.tree_hashes(empty)
        with self.assertRaises(FileNotFoundError): host.tree_hashes(self.root / 'absent')

    def transfer_patches(self, corrupt=False):
        def remote_python(config, code, *args):
            if 'runpy.run_path' in code and args and str(args[0]).endswith('/integration/two_host.py'):
                args = (str(Path(host.__file__).resolve()), *args[1:])
            if "m['tree_hashes']" in code:
                return json.dumps(host.tree_hashes(args[-1]))
            stream = io.StringIO()
            with patch.object(sys, 'argv', ['-c', *map(str, args)]), contextlib.redirect_stdout(stream):
                exec(code, {'__name__': '__main__'})
            return stream.getvalue().strip()
        def rsync(argv, **kwargs):
            self.assertEqual(argv[0], 'rsync')
            self.assertIn('--protect-args', argv)
            self.assertIn('ssh -p 10022', argv[argv.index('-e') + 1])
            prefix = self.config['ssh']['target'] + ':'
            source, dest = [Path(a.removeprefix(prefix)) for a in argv[-2:]]
            if source.is_dir(): shutil.copytree(source, dest)
            else: shutil.copy2(source, dest)
            if corrupt:
                file = next(p for p in dest.rglob('*') if p.is_file()) if dest.is_dir() else dest
                file.write_bytes(b'corrupted during transfer')
            return subprocess.CompletedProcess(argv, 0)
        return patch.object(host, 'remote_python', side_effect=remote_python), patch.object(host.subprocess, 'run', side_effect=rsync)

    def test_checked_sync_single_file_and_directory_both_directions(self):
        for directory in (False, True):
            for to_remote in (False, True):
                with self.subTest(directory=directory, to_remote=to_remote):
                    base = self.root / f'{directory}-{to_remote}'; base.mkdir()
                    source, dest = base / 'source', base / 'destination'
                    if directory:
                        source.mkdir(); (source / 'payload').write_bytes(b'asset')
                    else: source.write_bytes(b'asset')
                    remote_patch, run_patch = self.transfer_patches()
                    with remote_patch, run_patch as run:
                        checksums = host.checked_sync(self.config, source, dest, to_remote)
                        self.assertEqual(checksums, host.tree_hashes(source))
                        self.assertTrue(dest.exists())
                        self.assertFalse(list(base.glob('*.staging-*')))
                        before = run.call_count
                        self.assertEqual(host.checked_sync(self.config, source, dest, to_remote), checksums)
                        self.assertEqual(run.call_count, before)
                        (dest / 'payload' if directory else dest).write_bytes(b'existing different')
                        with self.assertRaisesRegex(ValueError, 'refusing to replace'):
                            host.checked_sync(self.config, source, dest, to_remote)
                        self.assertEqual((dest / 'payload' if directory else dest).read_bytes(), b'existing different')

    def test_promotion_never_replaces_a_concurrent_destination(self):
        for directory in (False, True):
            staging, destination = self.root / ('staging' + str(directory)), self.root / ('final' + str(directory))
            if directory:
                staging.mkdir(); destination.mkdir()
            else:
                staging.write_text('new'); destination.write_text('old')
            with self.assertRaises(FileExistsError):
                host.promote(staging, destination)
            self.assertTrue(staging.exists())
            if not directory:
                self.assertEqual(destination.read_text(),'old')

    def test_checked_sync_bad_checksum_retains_staging_without_promotion(self):
        for to_remote in (False, True):
            source = self.root / f'source-{to_remote}'; source.mkdir()
            (source / 'payload').write_bytes(b'original')
            dest = self.root / f'dest-{to_remote}'
            remote_patch, run_patch = self.transfer_patches(corrupt=True)
            with remote_patch, run_patch:
                with self.assertRaisesRegex(RuntimeError, 'checksum mismatch'):
                    host.checked_sync(self.config, source, dest, to_remote)
            self.assertFalse(dest.exists())
            self.assertEqual(len(list(self.root.glob(dest.name + '.staging-*'))), 1)

    def test_remote_session_ready_heartbeat_close_and_eof(self):
        log = self.root / 'lease.log'
        code = ("import sys\nfrom pathlib import Path\n"
                "print('worker log', flush=True)\nprint('TWO_HOST_READY {\"pid\": 123}', flush=True)\n"
                "with Path(sys.argv[1]).open('a', buffering=1) as stream:\n"
                " for line in sys.stdin:\n"
                "  stream.write(line)\n"
                "  if line.strip() == 'STOP': break\n"
                " stream.write('EOF_OR_STOP\\n')\n")
        command = [sys.executable, '-u', '-c', code, str(log)]
        with patch.object(host, 'local_command', return_value=command), contextlib.redirect_stderr(io.StringIO()):
            session = host.RemoteSession(self.config, ['fake'], local=True)
            try:
                self.assertEqual(session.wait_ready(timeout=3), {'pid': 123})
                deadline = time.monotonic() + 3
                while (not log.exists() or 'PING' not in log.read_text()) and time.monotonic() < deadline: time.sleep(.02)
                self.assertIn('PING', log.read_text())
                session.check()
            finally:
                session.close()
            session.close()
            self.assertIn('STOP\nEOF_OR_STOP', log.read_text())
            self.assertEqual(session.process.returncode, 0)
            session.process.stdout.close()
            other = host.RemoteSession(self.config, ['fake'], local=True)
            try:
                other.wait_ready(timeout=3)
                with other.lock: other.process.stdin.close()
                self.assertEqual(other.process.wait(timeout=3), 0)
                with self.assertRaisesRegex(RuntimeError, 'worker exited'): other.check()
            finally:
                other.close(); other.process.stdout.close()
        with patch.object(host, 'local_command', return_value=[sys.executable, '-c', 'raise SystemExit(7)']):
            failed = host.RemoteSession(self.config, ['fake'], local=True)
            with self.assertRaises(RuntimeError): failed.wait_ready(timeout=3)
            with self.assertRaises(RuntimeError): failed.close()
            failed.process.stdout.close()
            try: failed.process.stdin.close()
            except OSError: pass

    def test_clock_pair_budget_and_grandmaster(self):
        clock = dict(offset_us=0, system_offset_us=0, measurement_uncertainty_us=0, grandmaster_id='one')
        self.assertEqual(host.validate_clock_pair(dict(local=dict(clock),remote=dict(clock)),1000)['host_clock_error_bound_us'],0)
        with self.assertRaisesRegex(ValueError, 'combined'):
            host.validate_clock_pair(dict(local=dict(clock,system_offset_us=900),remote=dict(clock,system_offset_us=-900)),1000)
        with self.assertRaisesRegex(ValueError, 'grandmaster'):
            host.validate_clock_pair(dict(local=dict(clock),remote=dict(clock,grandmaster_id='two')),1000)

    def test_ptp_fields_and_nonfinite_offsets(self):
        text = 'portState SLAVE\nmaster_offset -250\ngmPresent true\ngmIdentity clock1\ncurrentUtcOffset 37\nptpTimescale 1\n'
        value = host.parse_ptp(text, 'local')
        self.assertEqual(value['offset_us'], -.25)
        self.assertEqual(value['utc_offset'], 37)
        self.assertTrue(value['ptp_timescale'])
        self.assertEqual(host.parse_ptp(text.replace('SLAVE', 'MASTER'), 'remote')['port_state'], 'MASTER')
        invalid = [text.replace('SLAVE', 'MASTER'), text.replace('true', 'false'),
                   text.replace('master_offset -250\n', ''), text + 'master_offset 5\n',
                   text + 'portState SLAVE\n', text.replace('currentUtcOffset 37', 'currentUtcOffset stale')]
        invalid.extend(text.replace('-250', value) for value in ('nan', 'inf', '-inf', 'stale'))
        for value in invalid:
            with self.subTest(text=value), self.assertRaises(ValueError): host.parse_ptp(value, 'local')


    def test_ptp_software_and_hardware_clock_binding(self):
        interfaces = self.root / 'network'; wired = interfaces / 'eth0'; wired.mkdir(parents=True)
        original_path = Path
        path = lambda value: interfaces if str(value) == '/sys/class/net' else original_path(value)
        text = ('portState SLAVE\nmaster_offset -250\ngmPresent true\ngmIdentity clock1\ncurrentUtcOffset 37\n'
                'ptpTimescale 1\ninterface eth0\ntimestamping SOFTWARE\n')
        with patch.object(host, 'Path', side_effect=path), patch.object(host, 'execute', return_value=text):
            result = host.ptp_probe('eth0', 'local')
            self.assertEqual(result['timestamping'], 'software')
            self.assertEqual(result['system_offset_us'], 0)
            self.assertIsNone(result['phc'])
            (wired / 'wireless').mkdir()
            with self.assertRaisesRegex(ValueError, 'wired'): host.ptp_probe('eth0', 'local')
            (wired / 'wireless').rmdir()
            with self.assertRaisesRegex(ValueError, 'missing'): host.ptp_probe('absent', 'local')
        for changed in (text.replace('interface eth0', 'interface eth1'),
                        text + 'interface eth0\n', text.replace('SOFTWARE', 'unsupported')):
            with patch.object(host, 'Path', side_effect=path), patch.object(host, 'execute', return_value=changed):
                with self.assertRaises(ValueError): host.ptp_probe('eth0', 'local')
        (wired / 'device' / 'ptp' / 'ptp7').mkdir(parents=True)
        with patch.object(host, 'Path', side_effect=path), \
             patch.object(host, 'execute', return_value=text.replace('SOFTWARE', 'HARDWARE')), \
             patch.object(host.os, 'open', return_value=9) as opened, \
             patch.object(host.os, 'close') as closed, \
             patch.object(host.time, 'time_ns', side_effect=[100000000000, 100000000200] * 5), \
             patch.object(host.time, 'clock_gettime_ns', return_value=137000500100) as clock:
            result = host.ptp_probe('eth0', 'local')
            opened.assert_called_once_with('/dev/ptp7', host.os.O_RDONLY)
            closed.assert_called_once_with(9)
            clock.assert_called_with(((~9) << 3) | 3)
            self.assertEqual(result['system_offset_us'], -500)
            self.assertEqual(result['measurement_uncertainty_us'], .1)
            self.assertEqual(result['phc'], 'ptp7')


class DeploymentChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = configuration()
        self.sha, self.origin = 'a' * 40, 'git@github.com:team/repo.git'
        self.exists = False
        self.local_dirty = self.remote_dirty = self.unpushed = self.build_failure = False
        self.previous = None; self.installed = False
        self.events = []; self.remote_manifest = None
        self.probe = dict(commit=self.sha, environment=dict(ros_distro='jazzy', rmw='rmw_fastrtps_cpp',
                                                           domain=73, message_types={'pose': 'hash'}))
        patches = [patch.object(deploy, 'ROOT', self.root), patch.object(deploy, 'git', side_effect=self.git),
                   patch.object(deploy, 'execute', side_effect=self.execute),
                   patch.object(deploy, 'remote_python', side_effect=self.remote_python),
                   patch.object(deploy, 'checked_sync', side_effect=self.sync),
                   patch.object(deploy, 'ssh_command', side_effect=lambda cfg, argv: ['SSH', *argv]),
                   patch.object(deploy, 'remote_command', side_effect=lambda cfg, argv, **kw: ['REMOTE', *argv]),
                   patch.object(deploy, 'local_command', side_effect=lambda cfg, argv, **kw: ['LOCAL', *argv]),
                   patch.object(deploy.subprocess, 'run', side_effect=self.run_command)]
        for item in patches: item.start(); self.addCleanup(item.stop)

    def git(self, *args):
        self.events.append(('git', args))
        if args == ('status', '--porcelain'): return ' M file' if self.local_dirty else ''
        if args == ('remote', 'get-url', 'origin'): return self.origin
        if args[0] == 'rev-parse':
            return 'source-tree-' + args[-1].split(':')[-1] if ':' in args[-1] else self.sha
        if args[0] == 'fetch':
            if self.unpushed: raise subprocess.CalledProcessError(128, args)
            return ''
        if args[0] == 'ls-remote': return self.sha + '\trefs/tags/two-host-v1'
        raise AssertionError(args)

    def execute(self, argv, **kwargs):
        self.events.append(('execute', tuple(argv)))
        if argv[0] in ('LOCAL', 'REMOTE'): return json.dumps(self.probe)
        self.assertEqual(argv[0], 'SSH')
        if argv[-1] == '--show-toplevel': return self.config['ssh']['remote_root']
        if argv[-1] == '--porcelain': return ' M remote.py' if self.remote_dirty else ''
        if argv[-3:] == ['remote', 'get-url', 'origin']: return self.origin
        if argv[-1] == 'HEAD': return self.sha
        return ''

    def remote_python(self, config, code, *args):
        self.events.append(('python', code))
        if 'unlink(missing_ok=True)' in code:
            self.remote_manifest = None
            return ''
        if "m['atomic_json']" in code:
            self.remote_manifest = json.loads(args[-1]); return ''
        if 'p.read_text()' in code: return json.dumps(self.previous)
        if 'install_jazzy/setup.bash' in code: return str(self.installed)
        if '.exists()' in code: return str(self.exists)
        raise AssertionError(code)

    def sync(self, *args, **kwargs):
        self.events.append(('sync', 'vocabulary'))
        return {'.': 'asset-hash'}

    def run_command(self, argv, **kwargs):
        self.events.append(('run', tuple(argv)))
        if 'colcon' in argv and self.build_failure: raise subprocess.CalledProcessError(2, argv)
        return subprocess.CompletedProcess(argv, 0)

    def deploy(self):
        with contextlib.redirect_stdout(io.StringIO()): return deploy.deploy(self.config, 'two-host-v1')

    def test_new_clone_build_and_matching_manifests(self):
        manifest = self.deploy()
        self.assertTrue(manifest['rebuilt'])
        self.assertEqual(manifest['commit'], self.sha)
        self.assertEqual(json.loads((self.root / 'deployment.json').read_text()), self.remote_manifest)
        self.assertTrue(any(event[0] == 'execute' and 'clone' in event[1] for event in self.events))
        builds = [event for event in self.events if event[0] == 'run' and 'colcon' in event[1]]
        self.assertEqual(len(builds), 1)
        self.assertIn('orbslam3_core', builds[0][1]); self.assertIn('orbslam3_ros2', builds[0][1])
        self.assertFalse(any('sam6d' in str(event) for event in builds))

    def test_existing_checkout_skips_unchanged_orb_and_rebuilds_changed_source(self):
        self.exists = self.installed = True
        self.previous = dict(orb_source_sha256=deploy.source_fingerprint(self.sha), built_unix_s=123)
        manifest = self.deploy()
        self.assertFalse(manifest['rebuilt']); self.assertEqual(manifest['built_unix_s'], 123)
        self.assertFalse(any(event[0] == 'execute' and 'clone' in event[1] for event in self.events))
        self.assertFalse(any(event[0] == 'run' and 'colcon' in event[1] for event in self.events))
        self.events.clear(); self.previous['orb_source_sha256'] = 'changed'
        self.assertTrue(self.deploy()['rebuilt'])
        self.assertTrue(any(event[0] == 'run' and 'colcon' in event[1] for event in self.events))

    def test_local_dirty_remote_dirty_and_unpushed_stop_before_checkout(self):
        for scenario in ('local_dirty', 'remote_dirty', 'unpushed'):
            with self.subTest(scenario=scenario):
                self.exists = True; self.events.clear(); setattr(self, scenario, True)
                with self.assertRaises((ValueError, subprocess.CalledProcessError)): self.deploy()
                self.assertFalse(any(event[0] == 'execute' and 'checkout' in event[1] for event in self.events))
                self.assertFalse(any(event[0] == 'sync' for event in self.events))
                setattr(self, scenario, False)
        with patch.object(deploy, 'git', side_effect=lambda *a: '' if a[0] == 'ls-remote' else self.git(*a)):
            with self.assertRaisesRegex(ValueError, 'missing or differs'): deploy.resolve_pushed('two-host-v1')
        with self.assertRaises(ValueError): deploy.resolve_pushed('--upload-pack=evil')

    def test_build_failure_invalidates_remote_success_before_assets(self):
        self.exists = True; self.previous = {'commit': 'old'}; self.remote_manifest = self.previous
        self.build_failure = True
        with self.assertRaises(subprocess.CalledProcessError): self.deploy()
        self.assertIsNone(self.remote_manifest)
        self.assertFalse((self.root / 'deployment.json').exists())
        invalidation = next(i for i, event in enumerate(self.events) if event[0] == 'python' and 'unlink' in event[1])
        transfer = next(i for i, event in enumerate(self.events) if event[0] == 'sync')
        self.assertLess(invalidation, transfer)
        self.assertFalse(any(event[0] == 'python' and "m['atomic_json']" in event[1] for event in self.events))


if __name__ == '__main__':
    unittest.main()
