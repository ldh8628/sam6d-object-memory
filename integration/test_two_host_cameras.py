#!/usr/bin/env python3
"""Offline checks for camera roles assigned by their connected host."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from test_two_host import configuration
import two_host as host


class CameraChecks(unittest.TestCase):
    def load(self, cameras):
        config = configuration()
        config['cameras'] = cameras
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(config))
            return host.load_config(path)

    def auto_config(self):
        return self.load(dict(slam_sync_mode=1, sam_sync_mode=3))

    def resolve(self, config, local, remote):
        def execute(argv, **kwargs):
            self.assertEqual(argv[1:], ['python', 'integration/two_host.py', 'cameras'])
            result = {'LOCAL': local, 'REMOTE': remote}[argv[0]]
            if isinstance(result, Exception):
                raise result
            return json.dumps(result)

        with patch.object(host, 'local_command', side_effect=lambda cfg, argv, **kw: ['LOCAL', *argv]), \
             patch.object(host, 'remote_command', side_effect=lambda cfg, argv, **kw: ['REMOTE', *argv]), \
             patch.object(host, 'execute', side_effect=execute):
            return host.resolve_cameras(config)

    def test_serials_default_to_auto_and_keep_sync_modes(self):
        expected = dict(slam_serial='auto', sam_serial='auto', slam_sync_mode=1, sam_sync_mode=3)
        self.assertEqual(self.auto_config()['cameras'], expected)
        self.assertEqual(self.load(expected)['cameras'], expected)
        for value in (123, None, '', '123;id'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load(dict(expected, sam_serial=value))

    def test_swapping_physical_cameras_preserves_host_roles(self):
        for local, remote in [('123', '456'), ('456', '123')]:
            with self.subTest(local=local):
                config = self.auto_config()
                resolved = self.resolve(config, [local], [remote])
                self.assertEqual(resolved, dict(sam_serial=local, slam_serial=remote,
                                                slam_sync_mode=1, sam_sync_mode=3))
                self.assertEqual(config['cameras'], resolved)

    def test_explicit_selectors_choose_from_multiple_cameras(self):
        config = self.load(configuration()['cameras'])
        self.assertEqual(self.resolve(config, ['789', '456'], ['123', '987']), config['cameras'])
        self.assertEqual(config['cameras']['sam_serial'], '456')
        self.assertEqual(config['cameras']['slam_serial'], '123')

    def test_ambiguous_missing_duplicate_and_invalid_reports_do_not_mutate_config(self):
        reports = [([], ['123']), (['123'], []), (['123', '456'], ['789']),
                   (['123'], ['456', '789']), (['123'], ['123']),
                   (['123'], ['not-a-serial']), (['123'], [456]),
                   (['123'], {'serial': '456'})]
        for local, remote in reports:
            with self.subTest(local=local, remote=remote):
                config = self.auto_config()
                before = copy.deepcopy(config)
                with self.assertRaises(ValueError):
                    self.resolve(config, local, remote)
                self.assertEqual(config, before)

    def test_missing_explicit_selector_fails_on_either_host(self):
        for local, remote in [(['789'], ['123']), (['456'], ['987'])]:
            with self.subTest(local=local, remote=remote):
                config = self.load(configuration()['cameras'])
                before = copy.deepcopy(config)
                with self.assertRaises(ValueError):
                    self.resolve(config, local, remote)
                self.assertEqual(config, before)

    def test_remote_enumeration_failure_preserves_unresolved_config(self):
        config = self.auto_config()
        before = copy.deepcopy(config)
        failure = subprocess.CalledProcessError(255, ['ssh'])
        with self.assertRaises(subprocess.CalledProcessError):
            self.resolve(config, ['123'], failure)
        self.assertEqual(config, before)

    def test_enumeration_deduplicates_asic_aliases_and_propagates_errors(self):
        devices = ('Device info:\n Serial Number : 456\n Asic Serial Number : 789\n'
                   'Device info:\n Serial Number : 123\n Asic Serial Number : 987\n'
                   'Device info:\n Serial Number : 456\n Asic Serial Number : 789\n')
        with patch.object(host, 'execute', return_value=devices) as execute:
            self.assertEqual(host.camera_serials(), ['123', '456'])
            execute.assert_called_once_with(['rs-enumerate-devices'], timeout=10)
        for failure in (FileNotFoundError('rs-enumerate-devices'),
                        subprocess.CalledProcessError(1, ['rs-enumerate-devices']),
                        subprocess.TimeoutExpired(['rs-enumerate-devices'], 10)):
            with self.subTest(failure=failure), patch.object(host, 'execute', side_effect=failure):
                with self.assertRaises(type(failure)):
                    host.camera_serials()
        with patch.object(host, 'execute', return_value='No device detected'):
            self.assertEqual(host.camera_serials(), [])
        with patch.object(host, 'execute', return_value='Device info:\n Serial Number : invalid\n'):
            with self.assertRaises(ValueError):
                host.camera_serials()


if __name__ == '__main__':
    unittest.main()
