"""Capture on two hosts, verify hardware synchronization, then calibrate remotely."""
from __future__ import annotations

from bisect import bisect_left
from collections import Counter
import json
import math
from pathlib import Path
import select
import statistics
import sys
import time

from input_integrity import read_rows, stats


def synchronization_report(slam_rows, sam_rows, *, minimum_seconds=600.):
    """Align counters using PTP ROS time; remove only the initial sensor clock epoch."""
    report = dict(status='PASS', minimum_seconds=minimum_seconds, streams={}, failures=[])
    for stream in ('rgb', 'depth'):
        rows = [[r for r in source if r.get('kind') == 'metadata' and r.get('stream') == stream]
                for source in (slam_rows, sam_rows)]
        summary = {'hosts': {}}
        report['streams'][stream] = summary
        bad = False
        for role, values in zip(('slam', 'sam'), rows):
            valid = bool(values) and all(type(r.get('frame_number')) is int and
                r['frame_number'] >= 0 and r.get('hardware_counter') and
                isinstance(r.get('sensor_time_ms'), (int, float)) and
                math.isfinite(r['sensor_time_ms']) and type(r.get('stamp_ns')) is int for r in values)
            if not valid:
                report['failures'].append(f'{role}/{stream}: hardware metadata missing')
                bad = True
                continue
            counters = [r['frame_number'] for r in values]
            changes = [b-a for a,b in zip(counters, counters[1:])]
            lost = sum(max(0, delta-1) for delta in changes)
            fraction = lost / (len(values) + lost)
            summary['hosts'][role] = dict(received=len(values), missing=lost, loss_fraction=fraction,
                                          duplicates=changes.count(0), backwards=sum(d<0 for d in changes))
            if any(delta <= 0 for delta in changes) or fraction > .001:
                report['failures'].append(f'{role}/{stream}: counter order or loss >0.1%')
                bad = True
            if any(b['stamp_ns'] <= a['stamp_ns'] or b['sensor_time_ms'] <= a['sensor_time_ms']
                   for a,b in zip(values, values[1:])):
                report['failures'].append(f'{role}/{stream}: timestamp order')
                bad = True
        if bad:
            continue
        master, slave = rows
        stamps = [r['stamp_ns'] for r in master]
        offsets = []
        # Counter epochs differ by device/start time. Anchor once using the shared PTP clock.
        for row in slave:
            if not stamps[0] <= row['stamp_ns'] <= stamps[-1]:
                continue
            i = bisect_left(stamps, row['stamp_ns'])
            candidates = master[max(0, i-1):i+1]
            match = min(candidates, key=lambda r: abs(r['stamp_ns']-row['stamp_ns']))
            if abs(match['stamp_ns']-row['stamp_ns']) <= 16_666_667:
                offsets.append(row['frame_number'] - match['frame_number'])
            if len(offsets) == 60:
                break
        if len(offsets) < 30:
            report['failures'].append(f'{stream}: insufficient corresponding frame anchors')
            continue
        counter_offset, count = Counter(offsets).most_common(1)[0]
        if count / len(offsets) < .95:
            report['failures'].append(f'{stream}: ambiguous initial counter alignment')
            continue
        master_index = {r['frame_number']: r for r in master}
        pairs = [(master_index[r['frame_number']-counter_offset], r) for r in slave
                 if r['frame_number']-counter_offset in master_index]
        duration = (min(pairs[-1][0]['stamp_ns'], pairs[-1][1]['stamp_ns']) -
                    max(pairs[0][0]['stamp_ns'], pairs[0][1]['stamp_ns'])) / 1e9
        raw_sensor_deltas = [b['sensor_time_ms']-a['sensor_time_ms'] for a,b in pairs]
        initial_offset = statistics.median(raw_sensor_deltas[:30])
        drift = [abs(delta-initial_offset) for delta in raw_sensor_deltas]
        ros_deltas = [abs(b['stamp_ns']-a['stamp_ns'])/1e6 for a,b in pairs]
        summary.update(counter_offset=counter_offset, paired_frames=len(pairs), duration_s=duration,
            sensor_initial_epoch_offset_ms=initial_offset, sensor_drift_ms=stats(drift),
            corresponding_ros_timestamp_difference_ms=stats(ros_deltas))
        if duration < minimum_seconds:
            report['failures'].append(f'{stream}: common recording shorter than {minimum_seconds:g}s')
        if summary['sensor_drift_ms']['p95'] > 5 or summary['corresponding_ros_timestamp_difference_ms']['p95'] > 5:
            report['failures'].append(f'{stream}: corresponding timestamp p95 exceeds 5ms')
    if report['failures']:
        report['status'] = 'FAIL'
    return report


def worker_argv(config, role, output, *, record=True, record_seconds=600.):
    return ['python', 'integration/two_host_worker.py', 'camera', '--role', role,
            '--serial', config['cameras'][role + '_serial'], '--sync-mode',
            str(config['cameras'][role + '_sync_mode']), '--ros-domain-id',
            str(config['network']['ros_domain_id'] + 1), '--output', str(output),
            *(['--record', '--record-seconds', str(record_seconds)] if record else [])]


def run(args):
    from two_host import (ROOT, RemoteSession, atomic_json, checked_sync, load_config,
                          execute, local_command, preflight, remote_command, validate_clock_pair)
    config = load_config(args.two_host_config)
    duration = args.input_check_seconds or 600.
    if not math.isfinite(duration) or duration < 600:
        raise ValueError('mode 3 requires at least 600 seconds of recorded synchronization validation')
    if args.force:
        raise ValueError('two-host capture requires a new session name; existing bags are preserved')
    dataset = ROOT / 'output' / args.name
    dataset.mkdir(parents=True, exist_ok=False)
    remote_dataset = Path(config['ssh']['remote_root']) / 'output' / args.name
    local_output, remote_output = dataset / 'distributed/sam', remote_dataset / 'distributed/slam'
    report = dict(schema_version=1, operation='map', status='INCOMPLETE', session=args.name,
                  local_dataset=str(dataset), remote_dataset=str(remote_dataset),
                  cameras=config['cameras'], clock_samples=[], transfers=[])
    remote = local = processing = None
    try:
        report['preflight'] = preflight(config)
        remote = RemoteSession(config, worker_argv(config, 'slam', remote_output, record_seconds=duration))
        report['remote_ready'] = remote.wait_ready(max(180., args.camera_timeout))
        local = RemoteSession(config, worker_argv(config, 'sam', local_output, record_seconds=duration), local=True)
        report['local_ready'] = local.wait_ready(max(180., args.camera_timeout))
        started, next_check, prompted = time.monotonic(), 0., False
        print('[two-host] mode 3 동기 검증을 포함하여 10분 이상 기록합니다.', flush=True)
        while True:
            local.check()
            remote.check()
            now = time.monotonic()
            if now >= next_check:
                sample = {'checked_unix_s': time.time()}
                for role in ('local', 'remote'):
                    command = ['python', 'integration/two_host.py', 'ptp', '--role', role,
                               '--interface', config['network'][role + '_ptp_interface']]
                    ptp = json.loads(execute((local_command if role == 'local' else remote_command)(
                                             config, command), timeout=10))
                    sample[role] = ptp
                    if (abs(ptp['offset_us']) + abs(ptp['system_offset_us']) +
                            ptp['measurement_uncertainty_us'] > config['network']['ptp_max_offset_us']):
                        report['clock_samples'].append(sample)
                        raise RuntimeError(f'{role} PTP offset exceeded limit')
                validate_clock_pair(sample, config['network']['ptp_max_offset_us'])
                report['clock_samples'].append(sample)
                next_check = time.monotonic() + 5.
            # Both monitors use a ten-second warmup and need a final source watermark.
            if now - started >= duration + 11:
                if args.input_check_seconds:
                    break
                if not prompted:
                    print('10분 검증 완료. 촬영을 끝내려면 Enter를 누르세요.', flush=True)
                    prompted = True
                readable, _, _ = select.select([sys.stdin], [], [], .2)
                if readable:
                    if not sys.stdin.readline():
                        raise RuntimeError('recording requires Enter confirmation')
                    break
            else:
                time.sleep(.2)
        # Full slave must stop before the master stops generating synchronization pulses.
        local.close()
        local = None
        remote.close()
        remote = None
        metadata = dataset / 'distributed/slam_metadata'
        metadata.mkdir(parents=True)
        for name in ('info.json', 'capture_report.json', 'input_integrity.json',
                     'slam_camera_metadata_health.jsonl'):
            report['transfers'].append(checked_sync(config, str(remote_output / name),
                                                   metadata / name, to_remote=False))
        sync = synchronization_report(read_rows([metadata / 'slam_camera_metadata_health.jsonl']),
            read_rows([local_output / 'sam_camera_metadata_health.jsonl']))
        report['synchronization'] = sync
        atomic_json(dataset / 'sync_quality.json', sync)
        if sync['status'] != 'PASS':
            raise RuntimeError('mode 3 synchronization failed: ' + '; '.join(sync['failures']))
        if args.input_check_only:
            report['status'] = 'PASS'
            return
        report['transfers'].append(checked_sync(config, local_output,
                                               str(remote_dataset / 'distributed/sam')))
        processing = RemoteSession(config, ['python', 'integration/two_host_worker.py',
            'finalize-map', '--dataset', str(remote_dataset), '--baseline', str(args.baseline),
            '--ros-domain-id', str(config['network']['ros_domain_id']+1)])
        processing.wait_ready(60)
        while processing.process.poll() is None:
            time.sleep(.2)
        processing.close()
        processing = None
        imported = dataset / 'export'
        report['transfers'].append(checked_sync(config, str(remote_dataset / 'export'), imported,
                                               to_remote=False))
        (imported / 'camera_extrinsic').rename(dataset / 'camera_extrinsic')
        (imported / 'camera_roles.json').rename(dataset / 'camera_roles.json')
        (imported / 'converted').rename(dataset / 'converted')
        imported.rmdir()
        report['calibration'] = json.loads((dataset / 'camera_extrinsic/quality_report.json').read_text())
        report['status'] = 'PASS'
        atomic_json(dataset / 'distributed_map.json', report)
        print(f'complete: {dataset}', flush=True)
    except BaseException as exc:
        report.update(status='FAIL', error=repr(exc))
        if processing is not None and processing.process.poll() is not None:
            quality_path = dataset / 'camera_extrinsic/quality_report.json'
            try:
                checked_sync(config, str(remote_dataset / 'camera_extrinsic/quality_report.json'),
                             quality_path, to_remote=False)
                report['calibration'] = json.loads(quality_path.read_text())
            except Exception as retrieval_error:
                report['calibration_report_retrieval_error'] = repr(retrieval_error)
        raise
    finally:
        # Closing both is required even when the first cleanup fails.
        cleanup_errors = []
        for session in (local, remote, processing):
            if session is not None:
                try:
                    session.close()
                except Exception as exc:
                    cleanup_errors.append(repr(exc))
        if cleanup_errors:
            report.update(status='FAIL', cleanup_errors=cleanup_errors)
        atomic_json(dataset / 'two_host_report.json', report)
        if cleanup_errors and sys.exc_info()[0] is None:
            raise RuntimeError('worker cleanup failed: ' + '; '.join(cleanup_errors))
