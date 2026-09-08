"""Run: python -m unittest discover -s integration -p test_camera_shutdown.py."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import camera_publish as camera


class CameraShutdownCheck(unittest.TestCase):
    def test_both_camera_groups_receive_shutdown_before_waiting(self):
        child = """
from pathlib import Path
import signal, sys, time
own, other = map(Path, sys.argv[1:])
def close(*_):
    own.touch()
    deadline = time.monotonic() + 2
    while not other.exists() and time.monotonic() < deadline:
        time.sleep(.01)
    raise SystemExit(0 if other.exists() else 2)
signal.signal(signal.SIGINT, close)
print('ready', flush=True)
while True:
    time.sleep(1)
"""
        processes = []
        real_popen = subprocess.Popen

        def spawn(command, **kwargs):
            process = real_popen(command, stdout=subprocess.PIPE, text=True, **kwargs)
            processes.append(process)
            self.assertEqual(process.stdout.readline().strip(), 'ready')
            return process

        def interrupt(_):
            raise KeyboardInterrupt

        with tempfile.TemporaryDirectory() as directory:
            def command(serial, *args):
                return [sys.executable, '-c', child, str(Path(directory)/serial),
                        str(Path(directory)/('2' if serial == '1' else '1'))]

            clock = SimpleNamespace(monotonic=time.monotonic, sleep=lambda seconds:
                interrupt(seconds) if seconds == .5 else time.sleep(seconds))
            try:
                with patch.object(camera, 'camera_command', command), \
                     patch.object(camera, 'wait_camera_frame'), \
                     patch.object(camera.subprocess, 'Popen', spawn), \
                     patch.object(camera, 'time', clock):
                    camera.run({'slam_serial': '1', 'sam_serial': '2'},
                               argparse.Namespace(roles_out=None, hardware_sync=False,
                                                  ros_domain_id=191, camera_timeout=2))
                self.assertEqual([process.returncode for process in processes], [0, 0])
            finally:
                for process in processes:
                    camera._stop(process, timeouts=(.1, .1, 1))
                    process.stdout.close()


if __name__ == '__main__':
    unittest.main()
