"""Bounded sequential intake and evidence-based RGB-D integrity (no ROS imports)."""
from __future__ import annotations

from collections import Counter, defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import statistics
import threading
import time


FPS = 30.0
QUEUE_SIZE = 60
QOS_DEPTH = 120


def stamp_ns(message):
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def rgbd_identity(message):
    return {"stamp_ns": stamp_ns(message.rgb),
            "depth_stamp_ns": stamp_ns(message.depth)}


def content_digest(message):
    # CDR alignment padding is unspecified and can change on serialization.
    # Hash all semantic fields plus the exact image buffer (including row padding).
    def plain(value):
        if hasattr(value, 'get_fields_and_field_types'):
            return {key: plain(getattr(value, key)) for key in value.get_fields_and_field_types()
                    if key != 'data'}
        if hasattr(value, 'tolist'):
            return value.tolist()
        if isinstance(value, (list, tuple)):
            return [plain(v) for v in value]
        return value
    digest = hashlib.sha256(json.dumps(plain(message), sort_keys=True,
                                      allow_nan=False).encode())
    if hasattr(message, 'data'):
        digest.update(memoryview(message.data))
    return digest.hexdigest()


def content_hashes(message):
    return {key: content_digest(getattr(message, key))
            for key in ('rgb', 'depth', 'rgb_camera_info', 'depth_camera_info')}


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    temporary.replace(path)


class SequentialInput:
    """Callback only enqueues; one worker describes, consumes and writes every row."""
    def __init__(self, path, consume, describe, *, stage, camera, serial='', capacity=60):
        if capacity < 1:
            raise ValueError('input queue capacity must be positive')
        self.capacity, self.consume, self.describe = capacity, consume, describe
        self.tags = dict(stage=stage, camera=camera, serial=serial)
        self.condition = threading.Condition()
        self.queue, self.rejected = deque(), deque()
        self.accepting = True
        self.received = self.consumed = self.failures = 0
        self.last_received_ns = 0
        self.error = None
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.path.open('x', encoding='utf-8')
        self.worker = threading.Thread(target=self._run, name=f'{stage}-{camera}', daemon=False)
        self.worker.start()

    def put(self, payload):
        entered = time.monotonic_ns()
        with self.condition:
            if not self.accepting:
                return False
            row = dict(self.tags, received_ns=entered, sequence=self.received)
            self.received += 1
            self.last_received_ns = entered
            if len(self.queue) >= self.capacity:
                # Only small identity records are retained for overflow; never image buffers.
                row.update(self.describe(payload, identity_only=True),
                           event='queue_overflow', consumed=False)
                self.rejected.append(row)
                self.failures += 1
                self.accepting = False  # First overflow aborts intake; drain retained frames.
            else:
                self.queue.append((payload, row))
            row['callback_ms'] = (time.monotonic_ns() - entered) / 1e6
            self.condition.notify()
            return 'event' not in row

    def _emit(self, row):
        self.log.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
        self.log.flush()

    def _run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.queue or self.rejected or not self.accepting)
                    rejected = list(self.rejected)
                    self.rejected.clear()
                    item = self.queue.popleft() if self.queue else None
                    done = not self.accepting and not self.queue and not item
                for row in rejected:
                    self._emit(row)
                if done:
                    break
                if item is None:
                    continue
                payload, row = item
                started = time.monotonic_ns()
                row['queue_ms'] = (started - row['received_ns']) / 1e6
                try:
                    row.update(self.describe(payload, identity_only=False))
                    result = self.consume(payload, row)
                    if result:
                        row.update(result)
                    row['consumed'] = True
                    self.consumed += 1
                except Exception as exc:
                    self.failures += 1
                    row.update(consumed=False, event='consume_error', error=repr(exc))
                row['consume_ms'] = (time.monotonic_ns() - started) / 1e6
                row['completed_ns'] = time.monotonic_ns()
                self._emit(row)
        except Exception as exc:
            self.error = exc
            self.failures += 1
        finally:
            self.log.close()

    def close(self):
        with self.condition:
            self.accepting = False
            self.condition.notify_all()
        self.worker.join()
        if self.error is not None:
            raise RuntimeError(f'input health writer failed: {self.path}') from self.error
        return self.failures == 0


def stats(values):
    values = sorted(float(v) for v in values)
    if not values:
        return {'count': 0, 'mean': None, 'p95': None, 'max': None}
    return {'count': len(values), 'mean': statistics.fmean(values),
            'p95': values[max(0, math.ceil(.95 * len(values)) - 1)], 'max': values[-1]}


def intervals(values, scale=1e6):
    return stats([(b-a)/scale for a, b in zip(values, values[1:])])


def read_rows(paths):
    rows = []
    for path in paths:
        with Path(path).open(encoding='utf-8') as handle:
            for line in handle:
                rows.append(json.loads(line))
    return rows


def summarize(rows, config, *, baseline=None, bag_rows=None):
    """Missing evidence stays INCOMPLETE; observed violations always remain FAIL."""
    start, end = config.get('start_ns'), config.get('end_ns')
    failures, missing, cameras = list(config.get('runtime_events', [])), list(config.get('evidence_errors', [])), {}
    if not start or not end or end <= start or not config.get('window_closed'):
        missing.append('measurement_window_not_closed')
    if not config.get('drained'):
        missing.append('input_or_recorder_not_drained')
    if not config.get('qos_verified'):
        missing.append('publisher_subscriber_qos_not_verified')
    if config.get('error'):
        missing.append(str(config['error']))
    if start and end and config.get('seconds') and end-start < (config['seconds']-.01)*1e9:
        missing.append('measurement_shorter_than_requested')
    ordered = sorted(rows, key=lambda r: (r.get('stage', ''), r.get('camera', ''),
                                          r.get('sequence', r.get('received_ns', 0))))
    for row in ordered:
        if row.get('event'):
            failures.append({k: row[k] for k in ('event', 'stage', 'camera', 'stream',
                            'stamp_ns', 'depth_stamp_ns', 'sequence', 'error') if k in row})
    for camera, serial in config.get('cameras', {}).items():
        raw = [r for r in ordered if r.get('camera') == camera]
        if any(r.get('serial') != serial for r in raw if r.get('kind') == 'rgbd'):
            failures.append(dict(event='serial_mismatch', camera=camera))
        monitor = [r for r in raw if r.get('stage') == 'monitor']
        meta = {stream: [r for r in monitor if r.get('kind') == 'metadata'
                         and r.get('stream') == stream] for stream in ('rgb', 'depth')}
        # Anchor the source interval on color metadata, then match even delayed arrivals
        # by the original source stamp. Host clocks are never subtracted from sensor time.
        anchors = meta['rgb']
        lower = next((r['stamp_ns'] for r in anchors if start and r['received_ns'] >= start), None)
        upper = next((r['stamp_ns'] for r in anchors if end and r['received_ns'] >= end), None)
        if lower is None or upper is None or upper <= lower:
            missing.append(f'{camera}:missing_source_window_watermarks')
        in_window = lambda r: (lower is not None and upper is not None
                               and lower <= r.get('stamp_ns', -1) < upper)
        live = [r for r in monitor if r.get('kind') == 'rgbd' and in_window(r)]
        c = {'serial': serial, 'source_window_ns': [lower, upper], 'streams': {},
             'stages': {}, 'received_rgbd': len(live)}
        cameras[camera] = c
        if not serial:
            missing.append(f'{camera}:serial_missing')
        def metadata_in_window(row, stream):
            if stream == 'rgb':
                return in_window(row)
            # Depth is independently stamped. Its endpoints are the paired depths,
            # never the numeric RGB bounds. Missing combined frames are checked via RGB metadata.
            return bool(live and live[0]['depth_stamp_ns'] <= row.get('stamp_ns',-1)
                        <= live[-1]['depth_stamp_ns'])
        for stream, all_meta in meta.items():
            segment, previous = 0, None
            for row in all_meta:
                number = row.get('frame_number')
                if number is None:
                    if metadata_in_window(row, stream): missing.append(f'{camera}/{stream}:frame_number_missing')
                    continue
                if previous is not None:
                    delta = number - previous['frame_number']
                    if delta < 0:
                        segment += 1
                    # A missing boundary frame is revealed by the next receipt, which
                    # may already be outside the window. Compare the whole receipt span.
                    crosses_window = (start and end and previous['received_ns'] < end
                                      and row['received_ns'] >= start)
                    if delta != 1 and (metadata_in_window(row, stream) or crosses_window):
                        event = ('duplicate' if delta == 0 else
                                 'restart_or_out_of_order' if delta < 0 else 'sensor_metadata_gap')
                        failure = dict(event=event, stage='sensor_driver_metadata', camera=camera,
                                       stream=stream, previous=previous['frame_number'], current=number)
                        if delta > 1:
                            failure['missing_range'] = [previous['frame_number']+1, number-1]
                        failures.append(failure)
                row['restart_segment'] = segment
                previous = row
            selected = [r for r in all_meta if metadata_in_window(r, stream)]
            source_times = [r.get('sensor_time_ms') for r in selected]
            valid_times = bool(source_times) and all(v is not None and math.isfinite(v) for v in source_times)
            hz = ((len(source_times)-1)*1000/(source_times[-1]-source_times[0])
                  if valid_times and len(source_times)>1 and source_times[-1]>source_times[0] else None)
            c['streams'][stream] = dict(count=len(selected), sensor_fps=hz,
                                        sensor_interval_ms=intervals(source_times, 1) if valid_times else stats([]))
            if not selected or hz is None or any(not r.get('hardware_counter') for r in selected):
                missing.append(f'{camera}/{stream}:hardware_metadata_insufficient')
            elif not 29.5 <= hz <= 30.5:
                failures.append(dict(event='sensor_fps', camera=camera, stream=stream, actual=hz))
            if valid_times and any(b<=a for a,b in zip(source_times,source_times[1:])):
                failures.append(dict(event='sensor_time_order', camera=camera, stream=stream))
            by_stamp = defaultdict(list)
            for r in all_meta:
                by_stamp[r.get('stamp_ns')].append(r)
            image_stamp = 'stamp_ns' if stream == 'rgb' else 'depth_stamp_ns'
            expected = Counter(r['stamp_ns'] for r in selected)
            actual = Counter(r[image_stamp] for r in live)
            if expected != actual:
                failures.append(dict(event='metadata_rgbd_mismatch', camera=camera, stream=stream,
                    missing_stamps=list((expected-actual).elements()),
                    extra_stamps=list((actual-expected).elements())))
            for frame in live:
                matches = by_stamp[frame[image_stamp]]
                if len(matches) != 1 or matches[0].get('frame_number') is None:
                    missing.append(f'{camera}/{stream}:ambiguous_metadata_association')
                else:
                    frame.setdefault('identity', {})[stream] = {
                        'serial': serial, 'segment': matches[0].get('restart_segment', 0),
                        'number': matches[0]['frame_number'], 'stamp_ns': frame[image_stamp]}
        expected_pairs = [(r['stamp_ns'], r['depth_stamp_ns']) for r in live]
        if len(set(expected_pairs)) != len(expected_pairs):
            failures.append(dict(event='duplicate_rgbd', camera=camera))
        if any(b[0] <= a[0] or b[1] <= a[1] for a,b in zip(expected_pairs,expected_pairs[1:])):
            failures.append(dict(event='rgbd_order', camera=camera))
        for stage in ['monitor', *config.get('consumers', {}).get(camera, [])]:
            selected = [r for r in raw if r.get('stage') == stage and r.get('kind') == 'rgbd' and in_window(r)]
            pairs = [(r['stamp_ns'], r['depth_stamp_ns']) for r in selected]
            if stage != 'monitor' and pairs != expected_pairs:
                failures.append(dict(event='consumer_input_mismatch', camera=camera, stage=stage,
                                     expected=len(expected_pairs), actual=len(pairs)))
            if not selected:
                missing.append(f'{camera}/{stage}:no_frames')
            if any(r.get('consumed') is not True for r in selected):
                failures.append(dict(event='unconsumed_frames', camera=camera, stage=stage))
            stat = {'received': len(selected), 'consumed': sum(r.get('consumed') is True for r in selected)}
            for key, p95_limit, max_limit in [('callback_ms', 2, None), ('queue_ms', 10, 100)]:
                samples = [r[key] for r in selected if r.get(key) is not None]
                stat[key] = stats(samples)
                if len(samples) != len(selected):
                    missing.append(f'{camera}/{stage}:{key}_missing')
                if samples and (stat[key]['p95'] > p95_limit or max_limit and max(samples)>max_limit):
                    failures.append(dict(event=key, camera=camera, stage=stage, actual=stat[key]))
            receive = [r['received_ns'] for r in raw if r.get('stage')==stage and r.get('kind')=='rgbd'
                       and start is not None and end is not None and start <= r['received_ns'] < end]
            stat['host_interval_ms'] = intervals(receive)
            stat['host_received_in_window'] = len(receive)
            stat['host_fps'] = len(receive)*1e9/(end-start) if start and end and end>start else None
            boundary = intervals([start,*receive,end]) if start and end and end>start else stats([])
            stat['host_boundary_interval_ms'] = boundary
            if boundary['max'] is not None and boundary['max']>100:
                failures.append(dict(event='host_window_gap', camera=camera, stage=stage, actual=boundary))
            if stat['host_fps'] is not None and not 29.5<=stat['host_fps']<=30.5:
                failures.append(dict(event='host_fps', camera=camera, stage=stage, actual=stat['host_fps']))
            interval = stat['host_interval_ms']
            if interval['count'] and (interval['p95']>50 or interval['max']>100):
                failures.append(dict(event='host_interval_ms', camera=camera, stage=stage, actual=interval))
            waits = [r['queue_ms'] for r in selected if r.get('queue_ms') is not None]
            # Compare one-second means; report a sustained rise only across all quartiles.
            blocks = [statistics.fmean(waits[i:i+30]) for i in range(0,len(waits)-29,30)]
            quarters = [statistics.fmean(blocks[i*len(blocks)//4:(i+1)*len(blocks)//4]) for i in range(4)] if len(blocks)>=4 else []
            stat['queue_quartile_means_ms'] = quarters
            if quarters and quarters[-1]-quarters[0]>2 and all(b>a for a,b in zip(quarters,quarters[1:])):
                failures.append(dict(event='sustained_queue_growth', camera=camera, stage=stage, actual=quarters))
            c['stages'][stage] = stat
        if baseline is None:
            if not config.get('is_baseline'):
                missing.append('camera_only_baseline_missing')
        else:
            baseline_config = baseline.get('config', {})
            if (not baseline_config.get('is_baseline') or baseline_config.get('record')
                    or any(baseline_config.get('consumers', {}).values())):
                missing.append('baseline_not_camera_only')
            if config.get('settings', {}).get('hardware_sync') != baseline_config.get('settings', {}).get('hardware_sync'):
                missing.append('baseline_hardware_sync_differs')
            if baseline.get('status') != 'PASS':
                missing.append('camera_only_baseline_not_passed')
            for key in ('profile','rmw','qos_depth','publisher_profile_sha256'):
                if config.get(key) != baseline.get('config', {}).get(key):
                    missing.append('baseline_configuration_differs:'+key)
            base = baseline.get('cameras', {}).get(camera, {})
            base_fps = base.get('stages', {}).get('monitor', {}).get('host_fps')
            fps = c['stages']['monitor']['host_fps']
            if base.get('serial') != serial or not base_fps or not fps:
                missing.append(f'{camera}:baseline_not_comparable')
            else:
                c['fps_drop_fraction'] = 1-fps/base_fps
                if c['fps_drop_fraction']>.01:
                    failures.append(dict(event='load_fps_drop', camera=camera, actual=c['fps_drop_fraction']))
        if config.get('record'):
            if bag_rows is None:
                missing.append('recording_replay_comparison_missing')
            else:
                bag = [r for r in bag_rows if r.get('camera')==camera and r.get('kind')=='rgbd' and in_window(r)]
                c['bag_count'] = len(bag)
                for stream in ('rgb','depth'):
                    live_meta = [r for r in meta[stream] if metadata_in_window(r, stream)]
                    recorded_meta = [r for r in bag_rows if r.get('camera')==camera
                                     and r.get('kind')=='metadata' and r.get('stream')==stream and metadata_in_window(r, stream)]
                    if [(r['stamp_ns'],r.get('metadata')) for r in live_meta] != [
                            (r['stamp_ns'],r.get('metadata')) for r in recorded_meta]:
                        live_ids = Counter((r['stamp_ns'],r.get('metadata',{}).get('frame_counter')) for r in live_meta)
                        bag_ids = Counter((r['stamp_ns'],r.get('metadata',{}).get('frame_counter')) for r in recorded_meta)
                        failures.append(dict(event='recorder_metadata_mismatch',camera=camera,stream=stream,
                            expected=len(live_meta),actual=len(recorded_meta),
                            missing=[dict(stamp_ns=s,frame_number=n) for s,n in (live_ids-bag_ids).elements()],
                            extra=[dict(stamp_ns=s,frame_number=n) for s,n in (bag_ids-live_ids).elements()]))
                actual_pairs = [(r['stamp_ns'], r['depth_stamp_ns']) for r in bag]
                if actual_pairs != expected_pairs:
                    failures.append(dict(event='recorder_frame_mismatch', camera=camera,
                        missing=list((Counter(expected_pairs)-Counter(actual_pairs)).elements()),
                        extra=list((Counter(actual_pairs)-Counter(expected_pairs)).elements())))
                if any(not r.get('hashes') for r in live):
                    missing.append(f'{camera}:live_content_hashes_missing')
                elif [r.get('hashes') for r in live] != [r.get('hashes') for r in bag]:
                    failures.append(dict(event='recorder_content_or_order_mismatch', camera=camera))
    if not cameras:
        missing.append('no_cameras_configured')
    return {'schema_version': 1, 'status': 'FAIL' if failures else 'INCOMPLETE' if missing else 'PASS',
            'config': config, 'cameras': cameras, 'failures': failures,
            'incomplete_reasons': sorted(set(missing)), 'pose_accuracy': 'NOT_EVALUATED',
            'observability': 'Metadata is downstream of driver synchronization; gaps alone do not prove USB or sensor loss.'}
