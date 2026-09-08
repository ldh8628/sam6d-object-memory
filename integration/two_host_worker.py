#!/usr/bin/env python3
"""Leased single-camera worker and remote map processing (run in realsense conda)."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import threading
import time

from camera_publish import SAM_CAMERA, SLAM_CAMERA, _stop, camera_arguments, topics, write_roles
from input_integrity import write_json


class StopRequested(BaseException):
    pass


class Lease:
    """SSH stdin is a lease: explicit STOP is clean; EOF/expiry is a failure."""
    def __init__(self, timeout=8.0):
        self.timeout, self.error = timeout, None
        self.done = threading.Event()

    def run(self):
        last, pending = time.monotonic(), b''
        while not self.done.is_set():
            readable, _, _ = select.select([sys.stdin.fileno()], [], [], .2)
            if readable:
                data = os.read(sys.stdin.fileno(), 4096)
                if not data:
                    self.error = 'controller lease EOF'
                    break
                pending += data
                if len(pending) > 4096:
                    self.error = 'invalid controller lease message'
                    break
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    if line.strip() == b'STOP':
                        os.kill(os.getpid(), signal.SIGUSR1)
                        return
                    if line.strip():
                        last = time.monotonic()
            if time.monotonic() - last > self.timeout:
                self.error = 'controller lease expired'
                break
        if not self.done.is_set():
            os.kill(os.getpid(), signal.SIGUSR1)


def camera_identity(camera, expected_serial, expected_mode, timeout):
    """Read the running device service and parameter, not requested launch values."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    from realsense2_camera_msgs.srv import DeviceInfo
    from sensor_msgs.msg import CameraInfo
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('two_host_camera_check')
    namespace = '/camera/' + camera
    try:
        def call(service, typ, request):
            client = node.create_client(typ, namespace + service)
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError('camera service missing: ' + service)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise TimeoutError('camera service did not respond: ' + service)
            return future.result()
        device = call('/device_info', DeviceInfo, DeviceInfo.Request())
        params = call('/get_parameters', GetParameters,
                      GetParameters.Request(names=['depth_module.inter_cam_sync_mode']))
        mode = parameter_value_to_python(params.values[0])
        if device.serial_number != expected_serial or mode != expected_mode:
            raise RuntimeError(f'camera identity mismatch: {device.serial_number}, sync={mode}')
        messages = []
        node.create_subscription(CameraInfo, topics(camera)['caminfo'], messages.append,
                                 qos_profile_sensor_data)
        deadline = time.monotonic() + timeout
        while not messages and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not messages:
            raise TimeoutError('camera info missing')
        info = messages[-1]
        camera_info = dict(width=info.width, height=info.height, K=list(info.k), D=list(info.d),
                           distortion_model=info.distortion_model)
        from create_map_urdf import _valid_camera_info
        if not _valid_camera_info(camera_info):
            raise RuntimeError('invalid running camera intrinsics')
        return dict(serial=device.serial_number, sync_mode=int(mode), camera=camera_info,
                    firmware=device.firmware_version, usb=device.usb_type_descriptor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def role_info(output, role, identity):
    """Validate this role's native bag and write the existing calibration info contract."""
    from create_map_urdf import _metadata_payload
    camera = SLAM_CAMERA if role == 'slam' else SAM_CAMERA
    metadata = _metadata_payload(output / 'capture_raw/metadata.yaml')
    counts = {t['topic_metadata']['name']: t['message_count']
              for t in metadata['topics_with_message_count']}
    expected = [topics(camera)[k] for k in ('rgbd', 'rgb_metadata', 'depth_metadata')]
    if any(counts.get(topic, 0) <= 0 for topic in expected):
        raise RuntimeError('role bag lacks RGBD or hardware metadata')
    duration = metadata['duration']['nanoseconds'] / 1e9
    if duration <= 0:
        raise RuntimeError('role bag has no duration')
    return dict(identity, path=role.upper(), duration_s=duration,
                t_start_ns=metadata['starting_time']['nanoseconds_since_epoch'],
                color_count=counts[expected[0]], color_hz=counts[expected[0]] / duration,
                color_topic=topics(camera)['rgb'], depth_topic=topics(camera)['depth'],
                caminfo_topic=topics(camera)['caminfo'])


def run_camera(args):
    from input_check import InputCheckSession, ensure_camera_baseline
    from rgbd_capture import check_space, recorder_args
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    camera = SLAM_CAMERA if args.role == 'slam' else SAM_CAMERA
    camera_process = recorder = orb = session = None
    log = (output / 'worker.log').open('x')
    error, ready, integrity, identity, disk_budget = None, False, None, None, None
    lease = Lease()
    def request_stop(_signum, _frame):
        raise StopRequested()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1):
        signal.signal(sig, request_stop)
    thread = threading.Thread(target=lease.run, daemon=True)
    try:
        thread.start()
        if args.record:
            disk_budget = check_space(output, args.record_seconds + 90, cameras=1)
        camera_process = subprocess.Popen(camera_arguments(args.serial, camera, args.sync_mode),
            stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        identity = camera_identity(camera, args.serial, args.sync_mode, args.camera_timeout)
        processes = [('camera', camera_process)]
        settings = dict(hardware_sync=True, sync_mode=args.sync_mode)
        baseline = (ensure_camera_baseline(None, output, args.ros_domain_id,
                    {camera: args.serial}, processes, args.camera_timeout, settings=settings,
                    environment=None) if args.record else None)
        if args.orb_config:
            orb = subprocess.Popen(['ros2', 'launch', 'orbslam3_ros2', 'orb_slam.launch.py',
                f'config:={args.orb_config.resolve()}'], stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                start_new_session=True)
            processes.append(('ORB', orb))
        if args.record:
            recorder = subprocess.Popen(recorder_args(output / 'capture_raw', cameras=(camera,)),
                stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
            processes.append(('recorder', recorder))
        session = InputCheckSession(output, args.ros_domain_id, {camera: args.serial},
            record=args.record, capture=output / 'capture_raw' if args.record else None,
            baseline=baseline, is_baseline=not args.record, settings=settings, environment=None)
        session.wait_ready(processes, args.camera_timeout)
        session.arm()
        identity.update(role=args.role, camera_name=camera, output=str(output))
        write_json(output / 'ready.json', identity)
        print('TWO_HOST_READY ' + json.dumps(identity), flush=True)
        ready = True
        while True:
            failed = [(name, p.returncode) for name, p in processes if p.poll() is not None]
            if failed:
                raise RuntimeError(f'worker child exited: {failed}')
            session.done()
            time.sleep(.2)
    except StopRequested:
        error = lease.error or (None if ready else 'worker stopped before READY')
    except BaseException as exc:
        error = repr(exc)
    finally:
        lease.done.set()
        # Finish the measured source interval before closing publishers and their DDS tail.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1):
            signal.signal(sig, signal.SIG_IGN)
        try:
            if session:
                session.close_window()
                time.sleep(.2)
        except Exception as exc:
            error = error or repr(exc)
        stopped = [_stop(camera_process, timeouts=(40., 5., 2.))]
        stopped += [_stop(recorder), _stop(orb)]
        try:
            if session:
                integrity = session.finish(drained=all(stopped), error=error)
            if not all(stopped):
                raise RuntimeError('worker process groups did not stop')
            if args.record and ready:
                write_json(output / 'info.json', role_info(output, args.role, identity))
            elif identity:
                write_json(output / 'info.json', identity)
        except Exception as exc:
            error = error or repr(exc)
        log.close()
        report = dict(status='PASS' if not error and integrity and integrity['status']=='PASS' else 'FAIL',
                      role=args.role, serial=args.serial, sync_mode=args.sync_mode,
                      error=error, integrity_status=integrity['status'] if integrity else None,
                      ready=ready, stopped=all(stopped), disk_budget=disk_budget)
        write_json(output / 'capture_report.json', report)
    if error:
        raise RuntimeError(error)
    if not integrity or integrity['status'] != 'PASS':
        raise RuntimeError(f'camera input validation failed: {output / "input_integrity.json"}')


def run_calibration(command, log_path):
    log = log_path.open("w")
    process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, env={**os.environ, 'TWO_HOST_INHERIT_ENV': '1'},
                               start_new_session=True)
    try:
        if process.wait():
            raise RuntimeError('remote calibration process failed')
    finally:
        log.close()
        # The existing ORB launcher needs up to 90s to close its own process group.
        if not _stop(process, timeouts=(100., 5., 2.)):
            raise RuntimeError('calibration child process did not stop')


def finalize_map(args):
    from rgbd_capture import convert_to_sqlite
    from camera_extrinsic_localization import quality_issues
    dataset = args.dataset.resolve()
    converted = dataset / 'converted'
    converted.mkdir(exist_ok=False)
    info = {'session': dataset.name, 'generated_by': 'integration/two_host_worker.py'}
    roles = {}
    for role in ('slam', 'sam'):
        source = dataset / 'distributed' / role
        report = json.loads((source / 'capture_report.json').read_text())
        if report['status'] != 'PASS':
            raise RuntimeError(f'{role} capture failed validation')
        info[role.upper()] = json.loads((source / 'info.json').read_text())
        roles[role + '_serial'] = info[role.upper()]['serial']
        convert_to_sqlite(source / 'capture_raw', converted / role.upper())
    write_json(converted / 'info.json', info)
    write_roles(dataset / 'camera_roles.json', roles)
    out = dataset / 'camera_extrinsic'
    calibration_log = dataset / 'calibration.log'
    try:
        run_calibration([sys.executable, str(Path(__file__).with_name('camera_extrinsic_localization.py')),
            '--inputs', str(converted), '--outputs', str(out), '--baseline', str(args.baseline),
            '--ros-domain-id', str(args.ros_domain_id)], calibration_log)
    except BaseException as exc:
        out.mkdir(parents=True, exist_ok=True)
        write_json(out / 'quality_report.json', dict(status='FAIL', failures=[repr(exc)],
            calibration_log=str(calibration_log),
            log_tail=calibration_log.read_text(errors='replace')[-16000:] if calibration_log.exists() else ''))
        raise
    result = json.loads((out / 'camera_extrinsic.json').read_text())
    issues = quality_issues(result, 1.0, .10, 5.0)
    if result.get('calibration_source') != 'stable transform cluster':
        issues.append('calibration was replaced by a rig reference')
    if result.get('calibration_attempt_rejection_reasons'):
        issues.extend(result['calibration_attempt_rejection_reasons'])
    quality = dict(status='FAIL' if issues else 'PASS', failures=issues,
        calibration=result, target_translation_p90_m=.03, target_rotation_p90_deg=1.,
        target_met=result['residual_translation_p90_m'] <= .03 and result['residual_rotation_p90_deg'] <= 1.)
    from input_integrity import read_rows, stats
    quality['orb_processing'] = {}
    for stage in ('map_slam', 'localize_sam'):
        health = out / stage / 'orb_input_health.jsonl'
        rows = read_rows([health]) if health.exists() else []
        durations = [r['consume_ms'] for r in rows if r.get('consumed') and 'consume_ms' in r]
        timing = stats(durations)
        timing['p50'] = sorted(durations)[(len(durations)-1)//2] if durations else None
        quality['orb_processing'][stage] = timing
    write_json(out / 'quality_report.json', quality)
    if issues:
        raise RuntimeError('distributed calibration rejected: ' + '; '.join(issues))
    export = dataset / 'export'
    (export / 'camera_extrinsic/map_slam').mkdir(parents=True, exist_ok=False)
    (export / 'converted').mkdir()
    shutil.copy2(converted / 'info.json', export / 'converted/info.json')
    for name in ('camera_extrinsic.json', 'camera_extrinsic.urdf', 'quality_report.json', 'summary.json'):
        shutil.copy2(out / name, export / 'camera_extrinsic' / name)
    shutil.copy2(out / 'map_slam/orbslam3_dense_map.pcd',
                 export / 'camera_extrinsic/map_slam/orbslam3_dense_map.pcd')
    shutil.copy2(dataset / 'camera_roles.json', export / 'camera_roles.json')
    print(json.dumps(dict(status='PASS', export=str(export), calibration=result)), flush=True)


def run_finalize(args):
    lease = Lease()
    def request_stop(_signum, _frame):
        raise StopRequested()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1):
        signal.signal(sig, request_stop)
    try:
        threading.Thread(target=lease.run, daemon=True).start()
        print('TWO_HOST_READY ' + json.dumps({'operation': 'finalize-map', 'dataset': str(args.dataset)}),
              flush=True)
        finalize_map(args)
    except StopRequested:
        raise RuntimeError(lease.error or 'map processing stopped by controller') from None
    finally:
        lease.done.set()


def run_command(args):
    """Apply the same controller lease to local SAM/Memory process groups."""
    lease, process = Lease(), None
    def request_stop(_signum, _frame):
        raise StopRequested()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1):
        signal.signal(sig, request_stop)
    try:
        threading.Thread(target=lease.run, daemon=True).start()
        process = subprocess.Popen(args.argv, stdin=subprocess.DEVNULL, start_new_session=True, stdout=sys.stderr)
        print('TWO_HOST_READY ' + json.dumps({'operation':'command'}), flush=True)
        code = process.wait()
        if code:
            raise RuntimeError(f'leased command exited: {code}')
    except StopRequested:
        if lease.error:
            raise RuntimeError(lease.error) from None
    finally:
        lease.done.set()
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1):
            signal.signal(sig, signal.SIG_IGN)
        if not _stop(process):
            raise RuntimeError('leased command cleanup failed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    camera = sub.add_parser('camera')
    camera.add_argument('--role', choices=('slam', 'sam'), required=True)
    camera.add_argument('--serial', required=True)
    camera.add_argument('--sync-mode', type=int, choices=(1, 3), required=True)
    camera.add_argument('--output', type=Path, required=True)
    camera.add_argument('--record', action='store_true')
    camera.add_argument('--record-seconds', type=float, default=600.)
    camera.add_argument('--orb-config', type=Path)
    camera.add_argument('--camera-timeout', type=float, default=30.)
    camera.add_argument('--ros-domain-id', type=int, required=True)
    finalize = sub.add_parser('finalize-map')
    finalize.add_argument('--dataset', type=Path, required=True)
    finalize.add_argument('--baseline', type=float, default=.095)
    finalize.add_argument('--ros-domain-id', type=int, required=True)
    command = sub.add_parser('command')
    command.add_argument('argv', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command == 'command':
        if args.argv[:1] == ['--']:
            args.argv.pop(0)
        if not args.argv:
            parser.error('command requires argv')
        return run_command(args)
    if not 0 <= args.ros_domain_id <= 232:
        parser.error('invalid ROS domain')
    if args.command == 'camera':
        if args.sync_mode != (1 if args.role == 'slam' else 3):
            parser.error('SLAM must use mode 1 and SAM must use mode 3')
        if not math.isfinite(args.camera_timeout) or args.camera_timeout <= 0:
            parser.error('camera timeout must be positive')
        if not math.isfinite(args.record_seconds) or args.record_seconds <= 0:
            parser.error('record seconds must be positive')
    elif not math.isfinite(args.baseline) or args.baseline <= 0:
        parser.error('baseline must be positive')
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    try:
        (run_camera if args.command == 'camera' else run_finalize)(args)
    except (OSError, ValueError, RuntimeError, TimeoutError, subprocess.SubprocessError) as exc:
        parser.exit(1, str(exc) + '\n')


if __name__ == '__main__':
    main()
