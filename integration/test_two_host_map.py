"""Run: python -m unittest discover -s integration -p test_two_host_map.py."""
from copy import deepcopy
from contextlib import redirect_stdout
from io import StringIO
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from camera_publish import camera_arguments, camera_command
from input_check import InputCheckSession
from two_host_map import synchronization_report, worker_argv


def frames(count=18040):
    # Independent sensor boot clocks and counter epochs, synchronized acquisition.
    hosts = []
    for host in (0, 1):
        rows = []
        for i in range(count):
            for stream in ('rgb', 'depth'):
                rows.append(dict(kind='metadata', stream=stream, frame_number=i+host*12345,
                    hardware_counter=True, sensor_time_ms=i*1000/30+host*700000,
                    stamp_ns=1_700_000_000_000_000_000+round(i*1e9/30)+host*1_000_000))
        hosts.append(rows)
    return hosts


class TwoHostMapChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hosts = frames()

    def test_independent_boot_clocks_and_counter_epochs(self):
        report = synchronization_report(*self.hosts)
        self.assertEqual(report['status'], 'PASS', report)
        self.assertEqual(report['streams']['rgb']['counter_offset'], 12345)
        self.assertAlmostEqual(report['streams']['depth']['sensor_drift_ms']['p95'], 0)
        self.assertGreaterEqual(report['streams']['rgb']['duration_s'], 600)

    def test_short_capture_drift_counter_order_and_loss_fail(self):
        self.assertEqual(synchronization_report(*frames(100))['status'], 'FAIL')
        for fault in ('drift', 'duplicate', 'backwards', 'loss', 'ros_clock', 'missing_evidence'):
            with self.subTest(fault=fault):
                master, slave = deepcopy(self.hosts)
                if fault == 'drift':
                    for i, row in enumerate(slave):
                        row['sensor_time_ms'] += i/len(slave)*20
                elif fault == 'duplicate':
                    slave[400]['frame_number'] = slave[398]['frame_number']
                elif fault == 'backwards':
                    slave[400]['frame_number'] -= 3
                elif fault == 'loss':
                    del slave[400:800]
                elif fault == 'ros_clock':
                    for row in slave:
                        row['stamp_ns'] += 8_000_000
                else:
                    slave[400].pop('hardware_counter')
                self.assertEqual(synchronization_report(master, slave)['status'], 'FAIL')

    def test_small_loss_allowed_and_epoch_does_not_absorb_late_drift(self):
        master, slave = deepcopy(self.hosts)
        del slave[400:402]
        report = synchronization_report(master, slave)
        self.assertEqual(report['status'], 'PASS', report)
        self.assertEqual(report['streams']['rgb']['hosts']['sam']['missing'], 1)
        for row in slave[len(slave)//2:]:
            row['sensor_time_ms'] += 10
        self.assertEqual(synchronization_report(master, slave)['status'], 'FAIL')

    def test_single_camera_arguments_and_inherited_monitor_environment(self):
        argv = camera_arguments('123', 'sam_camera', 3)
        self.assertIn('depth_module.inter_cam_sync_mode:=3', argv)
        self.assertIn('serial_no:=_123', argv)
        self.assertIn('ROS_DOMAIN_ID=72', camera_command('123', 'sam_camera', 72)[2])
        session = InputCheckSession.__new__(InputCheckSession)
        session.environment = None
        self.assertEqual(session._command(['python', 'check.py']), [sys.executable, 'check.py'])
        config = dict(cameras=dict(slam_serial='123',sam_serial='456',slam_sync_mode=1,sam_sync_mode=3),
                      network=dict(ros_domain_id=72))
        self.assertIn('73', worker_argv(config, 'slam', '/tmp/capture'))
        self.assertNotIn('--record', worker_argv(config, 'sam', '/tmp/live', record=False))

    def test_calibration_rejects_reference_and_exports_only_review_artifacts(self):
        from two_host_worker import finalize_map
        for fallback in (False, True):
            with self.subTest(fallback=fallback), tempfile.TemporaryDirectory() as tmp:
                dataset = Path(tmp)
                for role in ('slam', 'sam'):
                    source = dataset / 'distributed' / role
                    source.mkdir(parents=True)
                    (source / 'capture_report.json').write_text('{"status":"PASS"}')
                    (source / 'info.json').write_text(json.dumps({'serial':role+'123'}))
                calibration = dict(calibration_source='rig reference' if fallback else 'stable transform cluster',
                    translation_norm_m=.5, residual_translation_p90_m=.02, residual_rotation_p90_deg=.5)
                def calibration_run(*_, **__):
                    out = dataset / 'camera_extrinsic'
                    (out / 'map_slam').mkdir(parents=True)
                    (out / 'camera_extrinsic.json').write_text(json.dumps(calibration))
                    for name in ('camera_extrinsic.urdf', 'summary.json', 'map_slam/orbslam3_dense_map.pcd',
                                 'map_slam/map_test.osa'):
                        (out / name).write_text('test')
                with patch('rgbd_capture.convert_to_sqlite'), patch('two_host_worker.run_calibration', side_effect=calibration_run), \
                        redirect_stdout(StringIO()):
                    args = SimpleNamespace(dataset=dataset, baseline=.095, ros_domain_id=73)
                    if fallback:
                        with self.assertRaisesRegex(RuntimeError, 'replaced by a rig reference'):
                            finalize_map(args)
                        self.assertFalse((dataset / 'export').exists())
                    else:
                        finalize_map(args)
                        self.assertTrue((dataset / 'export/converted/info.json').is_file())
                        self.assertTrue((dataset / 'export/camera_extrinsic/map_slam/orbslam3_dense_map.pcd').is_file())
                        self.assertFalse(list((dataset / 'export').rglob('*.osa')))

    def test_controller_slave_stops_first_and_only_sam_capture_is_uploaded(self):
        import two_host
        import two_host_map
        config = dict(cameras=dict(slam_serial='123',sam_serial='456',slam_sync_mode=1,sam_sync_mode=3),
                      ssh=dict(remote_root='/remote/project'), network=dict(ros_domain_id=72,
                      local_ptp_interface='eth0',remote_ptp_interface='eth1',ptp_max_offset_us=1000))
        events, transfers = [], []
        class Session:
            def __init__(self, _, argv, local=False):
                self.role = 'finalize' if 'finalize-map' in argv else 'sam' if local else 'slam'
                self.process = SimpleNamespace(poll=lambda:0)
                events.append('start '+self.role)
            def wait_ready(self, _):
                events.append('ready '+self.role)
                return {}
            def check(self):
                pass
            def close(self):
                events.append('stop '+self.role)
        def transfer(_, source, destination, to_remote=True):
            transfers.append((source,destination,to_remote))
            if not to_remote and str(source).endswith('/export'):
                destination = Path(destination)
                (destination / 'camera_extrinsic').mkdir(parents=True)
                (destination / 'converted').mkdir()
                (destination / 'camera_extrinsic/quality_report.json').write_text('{"status":"PASS"}')
                (destination / 'camera_roles.json').write_text('{}')
            return {}
        with tempfile.TemporaryDirectory() as tmp, \
                patch.multiple(two_host, ROOT=Path(tmp), RemoteSession=Session, checked_sync=transfer), \
                patch('two_host.load_config', return_value=config), patch('two_host.preflight', return_value={}), \
                patch('two_host.execute', return_value='{"offset_us":0,"system_offset_us":0,"measurement_uncertainty_us":0,"grandmaster_id":"clock1"}'), \
                patch('two_host.local_command', return_value=[]), patch('two_host.remote_command', return_value=[]), \
                patch('two_host_map.time.monotonic', side_effect=[0, 700, 701]), \
                patch('two_host_map.read_rows', return_value=[]), \
                patch('two_host_map.synchronization_report', return_value={'status':'PASS'}), \
                patch('subprocess.run'), redirect_stdout(StringIO()):
            two_host_map.run(SimpleNamespace(two_host_config=Path('config'),name='test',
                input_check_seconds=600,force=False,camera_timeout=30,baseline=.095,input_check_only=False))
            self.assertEqual(events, ['start slam','ready slam','start sam','ready sam','stop sam','stop slam',
                                      'start finalize','ready finalize','stop finalize'])
            uploaded = [(a,b) for a,b,to_remote in transfers if to_remote]
            self.assertEqual(len(uploaded), 1)
            self.assertTrue(str(uploaded[0][0]).endswith('/distributed/sam'))
            report = json.loads((Path(tmp)/'output/test/two_host_report.json').read_text())
            self.assertEqual(report['status'], 'PASS')

    def test_local_command_worker_lease_stops_gpu_process_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            pidfile=Path(tmp)/'child.pid'
            script='import os,pathlib,sys,time; pathlib.Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(60)'
            proc=subprocess.Popen([sys.executable,str(Path(__file__).with_name('two_host_worker.py')),
                'command','--',sys.executable,'-c',script,str(pidfile)],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            try:
                self.assertTrue(proc.stdout.readline().startswith('TWO_HOST_READY '))
                import time
                deadline=time.monotonic()+3
                while not pidfile.exists() and time.monotonic()<deadline:
                    time.sleep(.01)
                pid=int(pidfile.read_text())
                out,err=proc.communicate('STOP\n',timeout=10)
                self.assertEqual(proc.returncode,0,err)
                with self.assertRaises(ProcessLookupError):
                    os.killpg(pid,0)
            finally:
                if proc.poll() is None:
                    proc.kill();proc.communicate()

    def test_lease_eof_expiry_and_stop_clean_up_process_group(self):
        script = '''
import json, os, signal, subprocess, sys, threading, time
from two_host_worker import Lease, StopRequested
from camera_publish import _stop
def stop(*_): raise StopRequested()
signal.signal(signal.SIGUSR1, stop)
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], start_new_session=True)
lease = Lease(.3)
try:
    threading.Thread(target=lease.run, daemon=True).start()
    print(child.pid, flush=True)
    time.sleep(60)
except StopRequested:
    pass
finally:
    lease.done.set()
    stopped = _stop(child, timeouts=(.1, .1, .1))
print(json.dumps(dict(stopped=stopped,error=lease.error)), flush=True)
'''
        env = {**os.environ, 'PYTHONPATH': str(Path(__file__).parent)}
        for event in ('STOP\n', '', 'expiry'):
            with self.subTest(event=event):
                proc = subprocess.Popen([sys.executable, '-c', script], env=env,
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                pid = int(proc.stdout.readline())
                try:
                    if event == 'expiry':
                        # Keeping stdin open without a heartbeat must still end the lease.
                        proc.wait(timeout=5)
                        out, err = proc.communicate()
                    else:
                        out, err = proc.communicate(event, timeout=5)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        try:
                            os.killpg(pid, 9)
                        except ProcessLookupError:
                            pass
                        proc.communicate()
                self.assertEqual(proc.returncode, 0, err)
                result = json.loads(out)
                self.assertTrue(result['stopped'])
                self.assertEqual(result['error'], None if event == 'STOP\n' else
                                 'controller lease expired' if event == 'expiry' else 'controller lease EOF')
                with self.assertRaises(ProcessLookupError):
                    os.killpg(pid, 0)


if __name__ == '__main__':
    unittest.main()
