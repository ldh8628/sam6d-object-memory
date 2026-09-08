"""Remote ORB/local SAM execution with timestamp and clock fault gates."""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

from two_host import (ROOT, RemoteSession, atomic_json, checked_sync, execute, load_config,
                      local_command, preflight, remote_command, remote_python, ssh_command, validate_clock_pair)
from two_host_map import worker_argv


def write_configs(args, config, output, remote_output, manifest):
    import yaml
    from camera_publish import SAM_CAMERA, SLAM_CAMERA, topics
    from run_object_memory_realtime import _urdf_transform
    from run_object_memory_rosbag import _write_run_configs
    map_root = args.map_dir.resolve()
    extrinsic = json.loads((map_root / 'camera_extrinsic/camera_extrinsic.json').read_text())
    urdf = args.urdf or map_root / 'camera_extrinsic/camera_extrinsic.urdf'
    extrinsic['X_slam_camera_to_sam_camera'] = _urdf_transform(urdf).tolist()
    remote_map = Path(manifest['remote_dataset'])
    atlas = remote_map / 'camera_extrinsic/map_slam' / ('map_' + str(extrinsic['dataset']) + '.osa')
    # Config generation only needs intrinsics. Atlas remains exclusively on the worker.
    _, orb_path, sam_path = _write_run_configs(map_root / 'converted', extrinsic,
        atlas, output, 1., args.view, args.slam_wait_ms)
    orb = yaml.safe_load(orb_path.read_text())
    orb['topics'].update({k: topics(SLAM_CAMERA)[k] for k in ('rgbd', 'rgb', 'depth')})
    orb['runtime'].update(input_require_publishers=True, input_camera=SLAM_CAMERA,
                           input_serial=config['cameras']['slam_serial'], input_queue_size=60,
                           input_qos_depth=120, enable_viewer=False, save_map_points_on_shutdown=False)
    orb['frames']['camera'] = 'slam_camera_color_optical_frame'
    orb['orbslam3']['vocabulary_path'] = str(Path(config['ssh']['remote_root']) /
        'orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    orb['orbslam3']['settings_path'] = str(remote_output / 'orb_config/orb_settings.yaml')
    orb['output']['dir'] = str(remote_output / 'orbslam3')
    orb['bag']['path'] = str(remote_map / 'converted/SLAM')
    orb_path.write_text(yaml.safe_dump(orb, sort_keys=False))
    bundle = output / 'orb_config'
    bundle.mkdir()
    (bundle / 'orb_config.yaml').write_text(orb_path.read_text())
    (bundle / 'orb_settings.yaml').write_text((orb_path.parent / 'orb_settings.yaml').read_text())
    # Remove the local dangling atlas link produced by the shared config writer.
    (orb_path.parent / atlas.name).unlink()
    sam = yaml.safe_load(sam_path.read_text())
    sam['topics'] = {k: topics(SAM_CAMERA)[k] for k in ('rgbd', 'rgb', 'depth', 'caminfo')}
    sam['runtime'].update(use_sim_time=False, idle_exit_s=0, qos_depth=120,
                           input_queue_size=60, publish_native_camera_info=False)
    sam['slam'].update(pose_match_tolerance_ms=args.pose_match_ms,
                       two_host_guard_file=str(output / 'two_host_guard.json'),
                       two_host_metrics_path=str(output / 'sam_metrics.json'))
    sam['output']['live_recording']['enabled'] = args.record
    sam['output']['topdown_map'] = str(map_root / 'camera_extrinsic/map_slam/orbslam3_dense_map.pcd')
    sam_path.write_text(yaml.safe_dump(sam, sort_keys=False))
    return bundle, sam_path, atlas, extrinsic


def clock_sample(config):
    def one(role):
        argv = ['python3', str((ROOT if role == 'local' else Path(config['ssh']['remote_root'])) /
                 'integration/two_host.py'), 'ptp', '--role', role,
                '--interface', config['network'][role + '_ptp_interface']]
        result = json.loads(execute(argv if role == 'local' else ssh_command(config, argv), timeout=7))
        if abs(result['offset_us']) + abs(result['system_offset_us']) + result['measurement_uncertainty_us'] > config['network']['ptp_max_offset_us']:
            raise RuntimeError(role + ' PTP/system clock exceeds 1ms gate')
        return result
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(one, ('local', 'remote')))
    return validate_clock_pair(dict(local=results[0], remote=results[1], checked_unix_s=time.time()),
                               config['network']['ptp_max_offset_us'])


def collect_diagnostics(config, remote_output, output, report):
    # No video, atlas or dense map is returned at realtime shutdown.
    code = """import pathlib,sys,shutil
root=pathlib.Path(sys.argv[1]); out=root/'diagnostics_export'; out.mkdir(exist_ok=True)
for folder in ('orbslam3','camera','bridge'):
 p=root/folder
 for f in p.rglob('*'):
  if f.is_file() and not f.is_symlink() and f.suffix in ('.json','.jsonl','.log','.txt'):
   dest=out/f.relative_to(root); dest.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(f,dest)
print(out)
"""
    remote_python(config, code, remote_output)
    report['diagnostics_checksums'] = checked_sync(config, str(remote_output / 'diagnostics_export'),
        output / 'remote_diagnostics', to_remote=False)


def finalize_report(output, report):
    from input_integrity import read_rows, stats
    for name, path in [('sam', output / 'sam_metrics.json'),
                       ('pose_transport', output / 'bridge/two_host_bridge_metrics.json')]:
        if path.exists():
            report[name] = json.loads(path.read_text())
        else:
            report[name] = dict(status='UNMEASURED', reason='process did not produce samples')
    orb_paths = list((output / 'remote_diagnostics').rglob('orb_input_health.jsonl'))
    rows = read_rows(orb_paths)
    timing = [float(r['consume_ms']) for r in rows if r.get('consumed') and 'consume_ms' in r]
    timing_stats = stats(timing)
    timing_stats['p50'] = sorted(timing)[(len(timing)-1)//2] if timing else None
    report['orb'] = dict(processing_ms=timing_stats, consumed=len(timing),
        tracking_ratio=report['pose_transport'].get('tracking_ratio'))
    counts = report['sam'].get('pose_matching', {})
    total = sum(counts.get('tracking_' + k, 0) for k in ('exact', 'interpolated', 'missed'))
    matched = sum(counts.get('tracking_' + k, 0) for k in ('exact', 'interpolated'))
    fraction = matched / total if total else None
    quality = report['sam'].get('object_memory_quality', {})
    ratio = report['pose_transport'].get('tracking_ratio')
    delay = report['pose_transport'].get('capture_to_arrival_ms_p95')
    gates = dict(orb_tracking=ratio >= .95 if ratio is not None else None,
                 pose_matching=fraction >= .98 if fraction is not None else None,
                 pose_capture_to_arrival=0 <= delay < 100 if delay is not None else None,
                 object_memory_quality=quality.get('gate_passed'))
    report['acceptance'] = dict(status='FAIL' if False in gates.values() else
        'INCOMPLETE' if None in gates.values() else 'PASS', gates=gates,
        tracking_frame_pose_match_fraction=fraction,
        tracking_metric_basis='latest received headerless tracking state; display-only, never a fusion gate',
        orb_tracking_min=.95, pose_match_min=.98, capture_to_pose_arrival_p95_ms_max=100.,
        mask_overlap_min=.5, depth_residual_mm_max=100., visible_frame_pass_fraction_min=.95,
        motion_segments='UNMEASURED: operator must record/annotate stationary, straight, slow/fast yaw')
    report['cameras_input'] = {}
    for role, path in [('sam', output / 'camera/input_integrity.json'),
                       ('slam', output / 'remote_diagnostics/camera/input_integrity.json')]:
        report['cameras_input'][role] = json.loads(path.read_text()) if path.exists() else None
    report['execution_status'] = report['status']
    if report['status'] in ('PASS', 'STOPPED'):
        report['status'] = report['acceptance']['status']
        if report['status'] == 'FAIL':
            report['failures'].append('measured realtime acceptance thresholds failed')
    atomic_json(output / 'two_host_report.json', report)


def run(args):
    config = load_config(args.two_host_config)
    map_root = args.map_dir.resolve()
    manifest = json.loads((map_root / 'distributed_map.json').read_text())
    if manifest['status'] != 'PASS' or manifest['synchronization']['status'] != 'PASS':
        raise ValueError('a passed distributed map and 10-minute mode-3 validation are required')
    if manifest['cameras'] != config['cameras']:
        raise ValueError('camera roles/modes differ from the calibrated map')
    expected_remote_map = Path(config['ssh']['remote_root']) / 'output' / map_root.name
    if Path(manifest['remote_dataset']) != expected_remote_map:
        raise ValueError('map belongs to a different remote root/session')
    output = (args.output or ROOT / 'output' / map_root.name / 'object_memory' /
              ('realtime_' + datetime.now().strftime('%Y%m%d_%H%M%S'))).resolve()
    output.mkdir(parents=True, exist_ok=False)
    remote_output = expected_remote_map / 'object_memory' / output.name
    guard = output / 'two_host_guard.json'
    report = dict(schema_version=1, operation='realtime', status='INCOMPLETE', cameras=config['cameras'],
                  local_output=str(output), remote_output=str(remote_output), clock_samples=[],
                  calibration=manifest.get('calibration'), failures=[])
    sessions = {}
    pool, future = ThreadPoolExecutor(max_workers=1), None
    def gate(healthy=False, map_id=None, reason='starting'):
        atomic_json(guard, dict(healthy=healthy, map_id=map_id, updated_unix_s=time.time(), reason=reason))
    gate()
    try:
        report['preflight'] = preflight(config)
        if report['preflight']['local']['commit'] != manifest['preflight']['local']['commit']:
            raise ValueError('map/realtime commit differs; use its release or recreate calibration')
        bundle, sam_path, atlas, extrinsic = write_configs(args, config, output, remote_output, manifest)
        report['config_checksums'] = checked_sync(config, bundle, str(remote_output / 'orb_config'))
        # This small config contains no credentials and is excluded from Git.
        runtime_config = output / 'two_host.runtime.json'
        atomic_json(runtime_config, config)
        checked_sync(config, runtime_config, str(remote_output / 'two_host.runtime.json'))
        remote_python(config,
            "import pathlib,sys; a,o=map(pathlib.Path,sys.argv[1:]); assert a.is_file(), 'missing atlas'; o.mkdir(exist_ok=False); (o/a.name).symlink_to(a)",
            atlas, remote_output / 'orbslam3')
        command = worker_argv(config, 'slam', remote_output / 'camera', record=args.record)
        command += ['--record-seconds', str(args.input_check_seconds or 600), '--orb-config', str(remote_output / 'orb_config/orb_config.yaml'),
                    '--camera-timeout', str(args.camera_timeout)]
        sessions['remote_camera'] = RemoteSession(config, command)
        report['remote_ready'] = sessions['remote_camera'].wait_ready(args.ready_timeout)
        sessions['remote_bridge'] = RemoteSession(config, ['python', 'integration/two_host_bridge.py',
            '--config', str(remote_output / 'two_host.runtime.json'), '--role', 'remote',
            '--output', str(remote_output / 'bridge')])
        sessions['remote_bridge'].wait_ready(args.camera_timeout)
        sessions['local_bridge'] = RemoteSession(config, ['python', 'integration/two_host_bridge.py',
            '--config', str(runtime_config), '--role', 'local', '--output', str(output / 'bridge')], local=True)
        sessions['local_bridge'].wait_ready(args.camera_timeout)
        command = worker_argv(config, 'sam', output / 'camera', record=args.record)
        command += ['--record-seconds', str(args.input_check_seconds or 600), '--camera-timeout', str(args.camera_timeout)]
        sessions['local_camera'] = RemoteSession(config, command, local=True)
        report['local_ready'] = sessions['local_camera'].wait_ready(args.ready_timeout)
        for name, command in [
            ('SAM', ['env', 'SAM6D_VIEWER_POSE_SYNC=0', 'SAM6D_TWO_HOST_GUARD_FILE=' + str(guard),
                'ros2', 'launch', str(ROOT / 'sam6d_ws/realtime/launch/sam6d_split.launch.py'), 'config:=' + str(sam_path)]),
            ('ObjectMemory', ['python', str(ROOT / 'objectmemory_ws/object_memory/ros/object_memory_node.py'),
                '--ros-args', '-p', 'pose_topic:=/sam6d/camera_pose', '-p', 'detections_topic:=/sam6d/detections',
                '-p', 'caminfo_topic:=/camera/sam_camera/color/camera_info',
                '-p', 'landmarks_topic:=/object_memory/landmarks', '-p', 'map_frame:=' + str(extrinsic['dataset']),
                '-p', 'output_path:=' + str(output / 'object_memory.json'),
                '-p', 'two_host_guard_file:=' + str(guard), '-p', 'exact_pose_only:=true'])]:
            sessions[name] = RemoteSession(config, ['python', 'integration/two_host_worker.py', 'command', '--', *command],
                local=True, environment=config['environment']['sam_env'], source=False)
            sessions[name].wait_ready(args.camera_timeout)
        started, ready_deadline = None, time.monotonic() + args.ready_timeout
        last_clock, pinned_map = time.monotonic(), None
        future = pool.submit(clock_sample, config)
        while True:
            for session in sessions.values():
                session.check()
            if future.done():
                sample = future.result()
                report['clock_samples'].append(sample)
                last_clock = time.monotonic()
                future = pool.submit(clock_sample, config)
            if time.monotonic() - last_clock > 3:
                raise RuntimeError('PTP monitoring lease expired')
            status_path = output / 'bridge/bridge_status.json'
            status = json.loads(status_path.read_text()) if status_path.exists() else {}
            if status.get('map_id') and pinned_map is None:
                pinned_map = status['map_id']
            if pinned_map and status.get('map_id') != pinned_map:
                raise RuntimeError('ORB map identity changed')
            transport_ok = status.get('healthy') and 0 <= time.time() - status.get('updated_unix_s', 0) < 2
            if started is not None and not transport_ok:
                raise RuntimeError('LAN/ORB tracking heartbeat lost: ' + str(status.get('reason')))
            healthy = bool(transport_ok and pinned_map and report['clock_samples'])
            gate(healthy, pinned_map, 'healthy' if healthy else 'waiting for ORB pose transport')
            if started is None:
                if time.monotonic() > ready_deadline:
                    raise TimeoutError('distributed SAM/ORB READY timeout')
                if healthy and (output / 'READY').exists():
                    started = time.monotonic()
                    print(f'TWO-HOST READY: {output}', flush=True)
            elif args.input_check_seconds and time.monotonic() - started >= args.input_check_seconds:
                break
            time.sleep(.1)
        report['status'] = 'PASS'
    except KeyboardInterrupt:
        report['status'] = 'STOPPED'
    except BaseException as exc:
        report['status'] = 'FAIL'
        report['failures'].append(repr(exc))
        raise
    finally:
        try:
            gate(reason='stopped or fault')
        except OSError as exc:
            report['failures'].append('guard write: ' + repr(exc))
        # Source shutdown order follows the physical cable: slave then master.
        for name in ('local_camera', 'remote_camera', 'local_bridge', 'remote_bridge', 'SAM', 'ObjectMemory'):
            if name in sessions:
                try:
                    sessions[name].close()
                except Exception as exc:
                    report['failures'].append(name + ': ' + repr(exc))
        pool.shutdown(wait=True, cancel_futures=True)
        if 'remote_camera' in sessions:
            try:
                collect_diagnostics(config, remote_output, output, report)
            except Exception as exc:
                report['failures'].append('diagnostics: ' + repr(exc))
        if report['failures']:
            report['status'] = 'FAIL'
        finalize_report(output, report)
    if report['status'] == 'FAIL':
        raise RuntimeError('two-host run failed; see ' + str(output / 'two_host_report.json'))
