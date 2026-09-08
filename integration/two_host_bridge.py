#!/usr/bin/env python3
"""Relay only ORB control/pose topics across private and LAN DDS domains.

One ROS participant per process: Fast DDS caches default XML profiles globally,
so switching environment variables between rclpy Contexts is not isolation.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

from two_host import atomic_json, load_config

TOPICS = ('/orbslam3/pose', '/orbslam3/map_id', '/orbslam3/tracking_state', '/orbslam3/ready')
ROOT = Path(__file__).resolve().parent



class BridgeHealth:
    def __init__(self):
        self.map_id = None
        self.map_changed = False
        self.ready = False
        self.last_pose = self.last_state = 0.0
        self.state_monotonic = 0.0
        self.counts = Counter()
        # ponytail: last 100k poses bound RAM; full-run quantiles can use the JSONL log.
        self.latencies = deque(maxlen=100000)

    def observe(self, index, message, unix_s, monotonic_s):
        self.counts[TOPICS[index]] += 1
        if index == 0:
            self.last_pose = unix_s
            stamp = message.header.stamp
            latency_ms = (unix_s * 1e9 - stamp.sec * 1e9 - stamp.nanosec) / 1e6
            self.latencies.append(latency_ms)
            self.counts['negative_latency'] += latency_ms < 0
            return latency_ms
        if index == 1:
            value = message.data.strip()
            if not value or value == 'unknown':
                self.map_changed |= self.map_id is not None
            elif self.map_id is None:
                self.map_id = value
            elif value != self.map_id:
                self.map_changed = True
        elif index == 2:
            self.last_state = unix_s
            self.state_monotonic = monotonic_s
            self.counts['tracking'] += message.data == 'tracking'
        else:
            self.ready = message.data == 'ready'
        return None

    def status(self, unix_s, monotonic_s, failure=''):
        reason = failure or ('map_changed' if self.map_changed else
                            'not_ready' if not self.ready else
                            'map_unknown' if self.map_id is None else
                            'state_timeout' if not self.last_state or monotonic_s - self.state_monotonic > 2 else '')
        return dict(map_id=self.map_id, healthy=not reason, last_pose_unix_s=self.last_pose,
                    updated_unix_s=unix_s, reason=reason, ready=self.ready,
                    last_state_unix_s=self.last_state)

    def metrics(self):
        values = sorted(self.latencies)
        percentile = lambda p: values[max(0, math.ceil(len(values) * p) - 1)] if values else None
        states = self.counts[TOPICS[2]]
        return dict(counts=dict(self.counts), capture_to_arrival_ms_p50=percentile(.5),
                    capture_to_arrival_ms_p95=percentile(.95), pure_lan_latency_ms=None, latency_window_count=len(values),
                    tracking_ratio=self.counts['tracking'] / states if states else None)


def endpoint_environment(config, role, lan):
    network = config['network']
    peer = network['remote_ip' if role == 'local' else 'local_ip']
    return dict(ROS_DOMAIN_ID=str(int(network['ros_domain_id']) + (not lan)),
                ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS=peer if lan else '',
                RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT / ('fastdds_two_host.xml' if lan else 'fastdds_input.xml')),
                FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT / ('fastdds_two_host.xml' if lan else 'fastdds_input.xml')),
                RCUTILS_LOGGING_USE_STDOUT='0')


def ros_node(config, role, lan):
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ.update(endpoint_environment(config, role, lan))
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    class RelayNode(Node):
        def create_publisher(self, message_type, topic, *args, **kwargs):
            # Jazzy unconditionally creates this publisher during Node.__init__.
            if topic == '/parameter_events':
                return SimpleNamespace(publish=lambda message: None)
            return super().create_publisher(message_type, topic, *args, **kwargs)

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = RelayNode('two_host_' + role + ('_lan' if lan else '_private'),
                     enable_rosout=False, start_parameter_services=False,
                     parameter_overrides=[Parameter('start_type_description_service', value=False)])
    specs = [(PoseStamped if i == 0 else String, topic,
              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if i in (1, 3)
              else QoSProfile(depth=10)) for i, topic in enumerate(TOPICS)]
    return rclpy, node, specs


def sender(config, role, fd):
    from rclpy.serialization import deserialize_message
    channel = socket.socket(fileno=fd)
    rclpy, node, specs = ros_node(config, role, lan=role == 'remote')
    try:
        publishers = [node.create_publisher(*spec) for spec in specs]
        channel.send(b'READY')
        while rclpy.ok():
            if select.select([channel], [], [], .02)[0]:
                data = channel.recv(65536)
                if not data:
                    break
                index = data[0]
                if index >= len(specs):
                    raise ValueError('invalid bridge topic index')
                publishers[index].publish(deserialize_message(data[1:], specs[index][0]))
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        channel.close()


def run(config, role, output, config_path):
    from rclpy.serialization import serialize_message
    output.mkdir(parents=True, exist_ok=True)
    health = BridgeHealth()
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--config', str(config_path),
                                '--role', role, '--output', str(output), '--sender-fd', str(child.fileno())],
                               pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL, stdout=sys.stderr)
    child.close()
    rclpy = node = None
    failure = 'bridge_stopped'
    stop = False
    def request_stop(signum, frame):
        nonlocal stop
        stop = True
    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        if not select.select([parent], [], [], 30)[0] or parent.recv(65536) != b'READY':
            raise RuntimeError('bridge publisher failed to start')
        parent.setblocking(False)
        rclpy, node, specs = ros_node(config, role, lan=role == 'local')
        source_gids = {}
        with (output / 'pose_lan_arrivals.jsonl').open('a', buffering=1) as arrivals:
            def relay(index, message):
                nonlocal failure, stop
                publishers = node.get_publishers_info_by_topic(TOPICS[index])
                if len(publishers) != 1:
                    if len(publishers) > 1:
                        failure, stop = 'multiple_pose_publishers', True
                    return
                gid = bytes(publishers[0].endpoint_gid)
                if index in source_gids and source_gids[index] != gid:
                    failure, stop = 'pose_source_changed', True
                    return
                source_gids[index] = gid
                now, monotonic = time.time(), time.monotonic()
                latency = health.observe(index, message, now, monotonic)
                if latency is not None:
                    arrivals.write(json.dumps(dict(arrival_unix_s=now, latency_ms=latency)) + '\n')
                try:
                    parent.send(bytes([index]) + serialize_message(message))
                except (BlockingIOError, BrokenPipeError, ConnectionResetError):
                    # Fail closed instead of queuing stale poses or losing a map-change event.
                    health.counts['relay_failures'] += 1
                    failure, stop = 'relay_backpressure_or_disconnect', True
            subscriptions = [node.create_subscription(typ, topic, lambda msg, i=i: relay(i, msg), qos)
                             for i, (typ, topic, qos) in enumerate(specs)]
            atomic_json(output / 'bridge_status.json', health.status(time.time(), time.monotonic()))
            print('TWO_HOST_READY ' + json.dumps(dict(role=role, topics=list(TOPICS))), flush=True)
            heartbeat = time.monotonic()
            last_report = 0.0
            stdin_buffer = b''
            while rclpy.ok() and not stop:
                rclpy.spin_once(node, timeout_sec=.02)
                now = time.monotonic()
                if process.poll() is not None:
                    failure = 'publisher_exited'
                    break
                if select.select([sys.stdin], [], [], 0)[0]:
                    data = os.read(sys.stdin.fileno(), 4096)
                    if not data:
                        break
                    stdin_buffer += data
                    if len(stdin_buffer) > 65536:
                        raise ValueError('oversized bridge control input')
                    while b'\n' in stdin_buffer:
                        line, stdin_buffer = stdin_buffer.split(b'\n', 1)
                        if line.strip() == b'STOP':
                            stop = True
                        elif line.strip() in (b'HEARTBEAT', b'PING'):
                            heartbeat = now
                if now - heartbeat > 8:
                    failure = 'controller_lease_expired'
                    break
                if now - last_report >= .2:
                    if any(node.count_publishers(topic) > 1 for topic in TOPICS):
                        failure, stop = 'multiple_pose_publishers', True
                        break
                    atomic_json(output / 'bridge_status.json', health.status(time.time(), now))
                    atomic_json(output / 'two_host_bridge_metrics.json', health.metrics())
                    last_report = now
    except Exception as error:
        failure = 'bridge_error: ' + str(error)
        raise
    finally:
        parent.close()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        atomic_json(output / 'bridge_status.json', health.status(time.time(), time.monotonic(), failure))
        atomic_json(output / 'two_host_bridge_metrics.json', health.metrics())
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def self_test():
    h = BridgeHealth()
    msg = lambda value: SimpleNamespace(data=value)
    assert not h.status(100, 10)['healthy']
    h.observe(3, msg('ready'), 100, 10)
    h.observe(1, msg('unknown'), 100, 10)
    h.observe(2, msg('not_tracking'), 100, 10)
    assert not h.status(100, 10)['healthy']
    h.observe(1, msg('map/1'), 100, 10)
    assert h.status(100, 10)['healthy']  # state is display-only, pose gates fusion.
    assert not h.status(104, 14)['healthy']
    h.observe(2, msg('tracking'), 105, 15)
    assert h.metrics()['tracking_ratio'] == .5
    pose = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=105, nanosec=0)))
    assert abs(h.observe(0, pose, 105.05, 15) - 50) < .001
    h.observe(0, pose, 104.99, 15)
    assert h.metrics()['counts']['negative_latency'] == 1
    h.observe(1, msg('map/2'), 105, 15)
    h.observe(1, msg('map/1'), 105, 15)
    assert h.status(105, 15)['reason'] == 'map_changed'
    assert h.status(105, 15, 'PTP')['reason'] == 'PTP'
    print('two_host_bridge self-test PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--role', choices=('local', 'remote'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--sender-fd', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.config is None or args.role is None or args.output is None:
        parser.error('--config, --role and --output are required')
    config = load_config(args.config)
    if args.sender_fd is not None:
        sender(config, args.role, args.sender_fd)
    else:
        run(config, args.role, args.output, args.config.resolve())


if __name__ == '__main__':
    main()
