#!/usr/bin/env python3
"""Live input monitor and shared launcher lifecycle. Run with Jazzy/sam6d Python."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from camera_publish import topics
from input_integrity import (SequentialInput, content_hashes, read_rows, rgbd_identity,
                             stamp_ns, summarize, write_json)


class InputCheckSession:
    def __init__(self, output, domain_id, cameras, *, seconds=0, record=False,
                 capture=None, consumers=None, baseline=None, is_baseline=False,
                 compatibility=False, settings=None, environment='sam6d'):
        from rgbd_capture import check_space
        self.output, self.domain_id = Path(output), domain_id
        self.environment = environment
        self.output.mkdir(parents=True, exist_ok=True)
        self.config = dict(cameras=cameras, seconds=seconds, record=record,
            capture=str(capture) if capture else None, consumers=consumers or {},
            baseline=str(baseline) if baseline else None, is_baseline=is_baseline,
            rmw='rmw_fastrtps_cpp', qos_depth=120, queue_size=60, warmup_seconds=10,
            profile={'rgb':'640x480x30', 'depth':'848x480x30', 'aligned_depth':'640x480'},
            settings=settings or {}, window_closed=False, drained=False)
        import hashlib
        profile_path=Path(__file__).with_name('fastdds_input.xml')
        profile=profile_path.read_bytes()
        (self.output/'fastdds_input.xml').write_bytes(profile)
        self.config['publisher_profile_sha256']=hashlib.sha256(profile).hexdigest()
        self.budget = seconds+30 if seconds else 300
        if record:
            self.config['disk_budget'] = check_space(output, self.budget, cameras=len(cameras),
                                                    compatibility=compatibility)
        self.config_path = self.output/'input_check_config.json'
        write_json(self.config_path, self.config)
        self.log = (self.output/'input_check.log').open('x')
        self.process = subprocess.Popen(self._command([
            'python', str(Path(__file__).resolve()), '--monitor', str(self.config_path)]),
            stdout=self.log, stderr=subprocess.STDOUT,
            start_new_session=True)

    def _command(self, command):
        # A worker already activated by its controller must retain that environment.
        if self.environment is None:
            return [sys.executable, *command[1:]]
        from run_object_memory_rosbag import _conda_command
        return _conda_command(self.environment, command, domain_id=self.domain_id)

    def wait_ready(self, processes, timeout):
        deadline = time.monotonic()+timeout
        while not (self.output/'input_check.ready').exists():
            failed = [(name,p.returncode) for name,p in [*processes,('input monitor',self.process)] if p.poll() is not None]
            if failed:
                raise RuntimeError(f'input readiness failed: {failed}')
            if time.monotonic()>deadline:
                raise TimeoutError(f'RGBD/metadata/QoS/subscriber readiness timeout; see {self.log.name}')
            time.sleep(.1)

    def arm(self):
        write_json(self.output/'input_check.arm', {'start_ns':time.monotonic_ns()+10_000_000_000})

    def done(self):
        if self.process.poll() is not None:
            raise RuntimeError(f'input monitor exited ({self.process.returncode}); see {self.log.name}')
        if self.config['record']:
            from rgbd_capture import RESERVE_BYTES
            import shutil
            if shutil.disk_usage(self.output).free < RESERVE_BYTES+2*1024**3:
                raise RuntimeError('recording reached disk reserve; close input and preserve capture')
        return (self.output/'input_check.window_done').exists()

    def close_window(self):
        write_json(self.output/'input_check.close', {'end_ns':time.monotonic_ns()})

    def drain_consumers(self):
        services = []
        stages = {stage for values in self.config['consumers'].values() for stage in values}
        for stage, service in [('orb','/orbslam3/drain_input'), ('sam','/sam6d/drain_input')]:
            if stage in stages:
                services.append(service)
        for service in services:
            subprocess.run(self._command(['python', str(Path(__file__).resolve()),
                '--drain-service', service]), check=True, timeout=120)

    def finish(self, *, drained, error=None, keep_publishers=False):
        from run_object_memory_rosbag import _stop
        if keep_publishers and (not self.config['is_baseline'] or self.config['record']
                                or any(self.config['consumers'].values())):
            raise ValueError('only a camera-only baseline may keep publishers running')
        write_json(self.output/'input_check.stop', {'requested_ns':time.monotonic_ns(),
                                                  'keep_publishers':keep_publishers})
        try:
            self.process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            _stop(self.process)
            error = str(error or '')+' monitor drain timeout'
        self.log.close()
        self.config.update(drained=drained and self.process.returncode==0, error=error)
        write_json(self.config_path, self.config)
        subprocess.run(self._command(['python', str(Path(__file__).resolve()),
            '--finalize', str(self.config_path)]), check=True)
        return json.loads((self.output/'input_integrity.json').read_text())


def ensure_camera_baseline(baseline, output, domain_id, cameras, processes, timeout, *, settings,
                           environment='sam6d'):
    if baseline is not None:
        return baseline
    output = Path(output)/'input_baseline'
    print(f'[input] measuring 30-second camera-only baseline: {output}', flush=True)
    session = InputCheckSession(output, domain_id, cameras, seconds=30,
                                is_baseline=True, settings=settings,
                                **({'environment': environment} if environment != 'sam6d' else {}))
    error = None
    try:
        session.wait_ready(processes, timeout)
        session.arm()
        while not session.done():
            failed = [(name,p.returncode) for name,p in processes if p.poll() is not None]
            if failed:
                raise RuntimeError(f'camera baseline process exited: {failed}')
            time.sleep(.1)
    except BaseException as exc:
        error = repr(exc)
        raise
    finally:
        session.close_window()
        # Allow measurement-tail DDS delivery; the report still rejects missing pairs.
        time.sleep(2)
        result = session.finish(drained=True, error=error, keep_publishers=True)
    if result['status'] != 'PASS':
        raise RuntimeError(f'camera-only baseline is {result["status"]}; see {output / "input_integrity.json"}')
    return output/'input_integrity.json'


def run_monitor(config_path):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from realsense2_camera_msgs.msg import Metadata, RGBD
    cfg = json.loads(config_path.read_text())
    output = config_path.parent
    rclpy.init()
    node = rclpy.create_node('input_integrity_monitor')
    qos = QoSProfile(depth=120, reliability=ReliabilityPolicy.RELIABLE)
    workers, subscriptions, checked_topics = [], [], {}
    state = dict(qos_verified=False, window_closed=False, monitor_drained=False,
                 observed_publishers={}, observed_subscribers={})
    def describe_rgbd(msg, identity_only=False):
        result = dict(kind='rgbd', **rgbd_identity(msg))
        if not identity_only:
            result['hashes'] = content_hashes(msg)
            result['shape'] = [msg.rgb.width,msg.rgb.height,msg.depth.width,msg.depth.height]
        return result
    def describe_meta(payload, identity_only=False):
        stream,msg = payload
        row = dict(kind='metadata',stream=stream,stamp_ns=stamp_ns(msg))
        if not identity_only:
            metadata=json.loads(msg.json_data)
            row.update(metadata=metadata,frame_number=metadata.get('frame_counter'),
                sdk_frame_number=metadata.get('frame_number'),
                sensor_time_ms=metadata.get('frame_timestamp'),
                hardware_counter=type(metadata.get('frame_counter')) is int and metadata['frame_counter']>=0 and
                    metadata.get('clock_domain') in ('hardware_clock','global_time'))
        return row
    latest_metadata = {}
    def consume_metadata(msg,row):
        latest_metadata[(row['camera'],row['stream'])] = row['received_ns']
    def consume(msg,row):
        if row['kind']=='rgbd' and row['shape'] != [640,480,640,480]:
            raise ValueError(f'active profile differs from configured 640x480 RGB/aligned-depth: {row["shape"]}')
    for camera,serial in cfg['cameras'].items():
        frame_worker=SequentialInput(output/f'{camera}_rgbd_health.jsonl',consume,describe_rgbd,
            stage='monitor',camera=camera,serial=serial)
        metadata_worker=SequentialInput(output/f'{camera}_metadata_health.jsonl',consume_metadata,describe_meta,
            stage='monitor',camera=camera,serial=serial,capacity=240)
        workers += [frame_worker,metadata_worker]
        t=topics(camera)
        subscriptions.append(node.create_subscription(RGBD,t['rgbd'],frame_worker.put,qos))
        expected=['input_integrity_monitor']
        expected += [{'orb':'orbslam3_rgbd_node','sam':'sam6d_receiver'}[stage]
                     for stage in cfg['consumers'].get(camera,[])]
        if cfg['record']: expected.append('input_recorder')
        checked_topics[t['rgbd']]=expected
        for stream in ('rgb','depth'):
            topic=t[stream+'_metadata']
            subscriptions.append(node.create_subscription(Metadata,topic,
                lambda msg,s=stream,w=metadata_worker:w.put((s,msg)),qos))
            checked_topics[topic]=['input_integrity_monitor']+(['input_recorder'] if cfg['record'] else [])
    deadline=time.monotonic()+1200
    next_qos_check=0.0
    try:
        while True:
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()>=next_qos_check:
                next_qos_check=time.monotonic()+1.0
                ready=all(w.received for w in workers) and all(
                    (camera,stream) in latest_metadata for camera in cfg['cameras'] for stream in ('rgb','depth'))
                for topic,expected in checked_topics.items():
                    pubs=node.get_publishers_info_by_topic(topic)
                    subs=node.get_subscriptions_info_by_topic(topic)
                    state['observed_publishers'][topic]=[
                        {'name':p.node_name,'reliability':str(p.qos_profile.reliability),'depth':p.qos_profile.depth} for p in pubs]
                    state['observed_subscribers'][topic]=[
                        {'name':s.node_name,'reliability':str(s.qos_profile.reliability),'depth':s.qos_profile.depth} for s in subs]
                    reliable=[s.node_name for s in subs if s.qos_profile.reliability==ReliabilityPolicy.RELIABLE]
                    ready &= len(pubs)==1 and all(p.qos_profile.reliability==ReliabilityPolicy.RELIABLE for p in pubs)
                    ready &= all(name in reliable for name in expected)
                if not ready and state['qos_verified'] and state.get('start_ns',math.inf)<=time.monotonic_ns() and not state['window_closed']:
                    state.setdefault('runtime_events',[]).append({'event':'endpoint_or_qos_changed',
                                                                  'received_ns':time.monotonic_ns()})
                if ready and not state['qos_verified']:
                    state['qos_verified']=True
                    write_json(output/'input_check.ready',state)
                    print('input publishers, subscribers and reliable QoS READY',flush=True)
                elif not state['qos_verified'] and time.monotonic()>deadline:
                    raise TimeoutError('input QoS readiness timed out')
            arm=output/'input_check.arm'
            if 'start_ns' not in state and arm.exists():
                state.update(json.loads(arm.read_text()))
                if cfg['seconds']:
                    state['end_ns']=state['start_ns']+int(cfg['seconds']*1e9)
            close=output/'input_check.close'
            if not state['window_closed'] and close.exists():
                requested=json.loads(close.read_text())['end_ns']
                state['end_ns']=min(state.get('end_ns',requested),requested)
            if state.get('end_ns') and time.monotonic_ns()>=state['end_ns'] and not state['window_closed']:
                state['source_watermarks'] = all(latest_metadata.get((camera,stream),0)>=state['end_ns']
                    for camera in cfg['cameras'] for stream in ('rgb','depth'))
                if state['source_watermarks'] or time.monotonic_ns()>state['end_ns']+5_000_000_000:
                    state['window_closed']=True
                    write_json(output/'input_check.window_done',state)
            if (output/'input_check.stop').exists():
                if json.loads((output/'input_check.stop').read_text()).get('keep_publishers'):
                    if not cfg['is_baseline'] or cfg['record'] or any(cfg['consumers'].values()):
                        raise ValueError('only a camera-only baseline may keep publishers running')
                    # Seal this finite baseline's subscriptions and drain their workers.
                    # Source-window metadata/RGBD equality must still pass at finalization.
                    state['publishers_kept_running']=True
                    state['monitor_drained']=True
                    break
                last=max(w.last_received_ns for w in workers)
                # Publishers must be gone, and DDS delivery quiet before sealing the tail.
                no_publishers=all(not node.get_publishers_info_by_topic(t) for t in checked_topics)
                if no_publishers and time.monotonic_ns()-last>2_000_000_000:
                    state['monitor_drained']=True
                    break
    except BaseException as exc:
        state['monitor_error']=repr(exc)
        raise
    finally:
        for sub in subscriptions: node.destroy_subscription(sub)
        for worker in workers: worker.close()
        state['monitor_failures']=sum(w.failures for w in workers)
        write_json(output/'monitor_state.json',state)
        node.destroy_node()
        rclpy.shutdown()


def finalize(config_path):
    from rgbd_capture import bag_rows
    cfg=json.loads(config_path.read_text()); output=config_path.parent
    state_path=output/'monitor_state.json'
    if state_path.exists():
        state=json.loads(state_path.read_text()); cfg.update(state)
        cfg['drained']=cfg['drained'] and state.get('monitor_drained',False)
    paths=[*output.glob('*_health.jsonl'),*output.glob('orbslam3/*_health.jsonl'),
           *output.glob('live_map_preview/*_health.jsonl')]
    paths=[p for p in paths if p.name not in ('input_health.jsonl','bag_replay_health.jsonl')]
    rows=[]
    for path in paths:
        try:
            with path.open() as handle:
                for number,line in enumerate(handle,1):
                    try:
                        row=json.loads(line)
                        if not isinstance(row,dict): raise ValueError('row is not an object')
                        rows.append(row)
                    except (ValueError,TypeError) as exc:
                        cfg.setdefault('evidence_errors',[]).append(f'{path}:{number}: {exc}')
        except OSError as exc:
            cfg.setdefault('evidence_errors',[]).append(str(exc))
    baseline=None
    if cfg.get('baseline'):
        try:
            baseline=json.loads(Path(cfg['baseline']).read_text())
            if not isinstance(baseline,dict): raise ValueError('baseline is not an object')
        except (OSError,ValueError) as exc:
            cfg.setdefault('evidence_errors',[]).append('baseline: '+str(exc))
            baseline=None
    recorded=None
    if cfg['record']:
        try:
            recorded=list(bag_rows(cfg['capture']))
            with (output/'bag_replay_health.jsonl').open('w') as handle:
                for row in recorded: handle.write(json.dumps(row)+'\n')
        except Exception as exc:
            cfg['error']=str(cfg.get('error') or '')+' bag read failed: '+repr(exc)
    try:
        result=summarize(rows,cfg,baseline=baseline,bag_rows=recorded)
    except (KeyError,TypeError,ValueError,IndexError) as exc:
        cfg.setdefault('evidence_errors',[]).append('invalid input evidence: '+repr(exc))
        result=dict(schema_version=1,status='INCOMPLETE',config=cfg,cameras={},failures=[],
                    incomplete_reasons=cfg['evidence_errors'],pose_accuracy='NOT_EVALUATED')
    if 'orb' in {s for stages in cfg['consumers'].values() for s in stages}:
        # Static dataset names do not prove the currently active Atlas map identity.
        measured_orb = [r for r in rows if r.get('stage')=='orb' and r.get('kind')=='rgbd'
                        and cfg.get('start_ns',math.inf)<=r.get('received_ns',0)<cfg.get('end_ns',-1)]
        if not measured_orb or any(not r.get('map_id') or r.get('atlas_map_id',-1)<0 for r in measured_orb):
            result['incomplete_reasons'].append('active_orb_map_id_not_observed')
            if result['status']=='PASS': result['status']='INCOMPLETE'
    with (output/'input_health.jsonl').open('w') as handle:
        for row in sorted(rows,key=lambda r:r.get('received_ns',0)):
            row['measurement']=bool(cfg.get('start_ns',math.inf)<=row.get('received_ns',0)<cfg.get('end_ns',-1))
            handle.write(json.dumps(row)+'\n')
        for event in result['failures']:
            handle.write(json.dumps(dict(kind='integrity_event',**event))+'\n')
    write_json(output/'input_integrity.json',result)
    print(f'input integrity: {result["status"]} ({output})',flush=True)
    return result


def drain_service(name):
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init(); node=rclpy.create_node('input_drain_client')
    client=node.create_client(Trigger,name)
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise TimeoutError(f'drain service absent: {name}')
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=100)
        if not future.done() or not future.result().success:
            raise RuntimeError(f'drain failed: {name}: {future.result() if future.done() else "timeout"}')
        print(f'{name}: {future.result().message}')
    finally:
        node.destroy_node(); rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--monitor',type=Path)
    parser.add_argument('--finalize',type=Path)
    parser.add_argument('--drain-service')
    args=parser.parse_args()
    if args.monitor: run_monitor(args.monitor)
    elif args.finalize: finalize(args.finalize)
    elif args.drain_service: drain_service(args.drain_service)
    else: parser.error('provide --monitor, --finalize or --drain-service')


if __name__=='__main__': main()
