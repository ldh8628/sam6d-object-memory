#!/usr/bin/env python3
"""Exercise the actual ORB executable using synthetic native and legacy ROS input.

Run in the realsense ROS environment, with --executable, --vocabulary, --settings.
Logs and trajectories are retained below --output. This tests input mechanics,
not camera completeness or pose accuracy.
"""
import argparse
from array import array
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


def wait(node, predicate, seconds=30):
    until = time.monotonic() + seconds
    while not predicate():
        assert time.monotonic() < until, 'Timed out waiting for ORB input'
        rclpy.spin_once(node, timeout_sec=0.02)


def run(args, mode):
    output = args.output / mode
    output.mkdir(parents=True, exist_ok=True)
    health = output / 'orb_input_health.jsonl'
    topic = '/input_queue_test/' + mode
    node = rclpy.create_node('input_queue_test_' + mode)
    reliable = QoSProfile(depth=120, reliability=(ReliabilityPolicy.BEST_EFFORT
                          if mode in ('qos', 'late_qos') else ReliabilityPolicy.RELIABLE))
    legacy = mode.startswith('legacy')
    if mode in ('no_publishers', 'late_qos'):
        publishers = []
    elif legacy:
        publishers = [node.create_publisher(Image, topic + '/' + stream, reliable)
                      for stream in ('rgb', 'depth')]
    else:
        publishers = [node.create_publisher(RGBD, topic, reliable)]
    ready = []
    map_ids = []
    map_sub = node.create_subscription(String, '/orbslam3/map_id', map_ids.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    ready_sub = node.create_subscription(String, '/orbslam3/ready', ready.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    params = {'vocabulary_path': args.vocabulary, 'settings_path': args.settings,
              'rgbd_topic': '' if legacy else topic,
              'rgb_topic': topic + '/rgb', 'depth_topic': topic + '/depth',
              'input_health_path': health, 'input_queue_size': 1 if mode == 'overflow' else 60,
              'input_require_publishers': 'false' if mode in ('no_publishers', 'late_qos') else 'true',
              'publish_map_points': 'false', 'dense_map_enabled': 'false',
              'input_serial': 'synthetic', 'enable_viewer': 'false'}
    command = [str(args.executable), '--ros-args']
    for key, value in params.items():
        if value != '':
            command += ['-p', f'{key}:={value}']
    with (output / 'node.log').open('w') as log:
        process = subprocess.Popen(command, cwd=output, stdout=log, stderr=subprocess.STDOUT)
        try:
            if mode == 'qos':
                drain = node.create_client(Trigger, '/orbslam3/drain_input')
                assert drain.wait_for_service(timeout_sec=60)
                wait(node, lambda: health.exists() and 'incompatible_qos' in health.read_text())
                assert not ready
                result = drain.call_async(Trigger.Request())
                wait(node, result.done)
                assert not result.result().success
                summary = {'mode': mode, 'test': 'PASS'}
                print(json.dumps(summary), flush=True)
                return summary
            wait(node, lambda: ready and all(p.get_subscription_count() for p in publishers), 60)
            if mode == 'replaced_qos':
                node.destroy_publisher(publishers.pop())
                publishers.append(node.create_publisher(RGBD, topic,
                    QoSProfile(depth=120, reliability=ReliabilityPolicy.BEST_EFFORT)))
                wait(node, lambda: 'incompatible_qos' in health.read_text())
                drain = node.create_client(Trigger, '/orbslam3/drain_input')
                assert drain.wait_for_service(timeout_sec=10)
                result = drain.call_async(Trigger.Request())
                wait(node, result.done)
                assert not result.result().success
                summary = {'mode': mode, 'test': 'PASS'}
                print(json.dumps(summary), flush=True)
                return summary
            if mode in ('no_publishers', 'late_qos'):
                # A bag player can start after Atlas readiness; QoS discovery stays active.
                publishers.append(node.create_publisher(RGBD, topic, reliable))
                if mode == 'late_qos':
                    wait(node, lambda: 'incompatible_qos' in health.read_text())
                    drain = node.create_client(Trigger, '/orbslam3/drain_input')
                    assert drain.wait_for_service(timeout_sec=10)
                    result = drain.call_async(Trigger.Request())
                    wait(node, result.done)
                    assert not result.result().success
                    summary = {'mode': mode, 'test': 'PASS'}
                    print(json.dumps(summary), flush=True)
                    return summary
                wait(node, lambda: publishers[0].get_subscription_count() > 0)
            wait(node, lambda: bool(map_ids))
            assert '/slam_camera/' in map_ids[0].data and '/atlas_' in map_ids[0].data
            def send(index, malformed=False):
                msg = RGBD()
                for image, channels, encoding in ((msg.rgb, 3, 'rgb8'), (msg.depth, 2, '16UC1')):
                    image.header.stamp.sec = 100 + index // 30
                    image.header.stamp.nanosec = (index % 30) * 33333333
                    image.width, image.height = 640, 480
                    image.encoding, image.step = encoding, 640 * channels
                    image.data = rgb_data if channels == 3 else depth_data
                # Legacy ApproximateTime retains a nonzero-offset tail candidate;
                # lossless claims apply to native input, whose two stamps differ.
                if mode != 'legacy':
                    msg.depth.header.stamp.nanosec += 10
                if malformed:
                    msg.depth.data = array('B', [0])
                if mode == 'encodings':
                    msg.rgb.encoding = ('bgr8', 'rgb8', 'bgra8', 'rgba8')[index % 4]
                    msg.rgb.step = 640 * (4 if 'a' in msg.rgb.encoding else 3)
                    msg.rgb.data = array('B', bytes(msg.rgb.step * 480))
                    if index % 2:
                        msg.depth.encoding, msg.depth.step = '32FC1', 640 * 4
                        msg.depth.data = array('B', array('f', [1000.] * (640 * 480)).tobytes())
                if legacy:
                    publishers[0].publish(msg.rgb)
                    publishers[1].publish(msg.depth)
                else:
                    publishers[0].publish(msg)
            rgb_data = array('B', random.Random(3).randbytes(640 * 480 * 3)
                             if mode == 'overflow' else bytes(640 * 480 * 3))
            depth_data = array('B', b'\xe8\x03' * (640 * 480))
            count = 80 if mode == 'overflow' else 8
            for i in range(count):
                send(i)
                if mode != 'overflow':
                    until = time.monotonic() + 0.06
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.005)
            if mode == 'errors':
                send(count - 1)  # Duplicate input must fail without disappearing.
                send(count, malformed=True)
                count += 2
            for publisher in publishers:
                assert publisher.wait_for_all_acked(rclpy.duration.Duration(seconds=10))
            drain = node.create_client(Trigger, '/orbslam3/drain_input')
            assert drain.wait_for_service(timeout_sec=10)
            result = drain.call_async(Trigger.Request())
            wait(node, result.done, 60)
            rows = [json.loads(line) for line in health.read_text().splitlines()]
            frames = [row for row in rows if row['kind'] == 'rgbd']
            identities = [row for row in rows if row['kind'] == 'map_id']
            assert identities and identities[0]['map_id'] == map_ids[0].data
            assert all(row['map_id'] and row['atlas_map_id'] >= 0 for row in frames)
            if mode == 'legacy_offset':
                # Regression evidence: legacy approximate pairing is not lossless.
                assert len(frames) == count - 1
                summary = {'mode': mode, 'test': 'PASS', 'lossless': False,
                           'sent': count, 'received': len(frames), 'unmatched_tail': 1}
                print(json.dumps(summary), flush=True)
                return summary
            if mode == 'overflow':
                assert len(frames) + rows[-1]['rejected_after_abort'] == count
            else:
                assert len(frames) == count, (mode, len(frames), count)
            assert [row['sequence'] for row in frames] == list(range(1, len(frames) + 1))
            assert all(row['received_ns'] > 0 and row['callback_ms'] >= 0 and row['queue_ms'] >= 0 for row in frames)
            assert rows[-1]['kind'] == 'drain' and rows[-1]['context_valid']
            if mode in ('native', 'legacy', 'no_publishers', 'encodings'):
                assert result.result().success
                assert all(row['consumed'] and not row['event'] for row in frames)
                assert [row['stamp_ns'] for row in frames] == sorted(row['stamp_ns'] for row in frames)
                assert sorted(row['callback_ms'] for row in frames)[math.ceil(.95 * count) - 1] <= 2
            elif mode == 'errors':
                assert not result.result().success
                assert frames[-2]['event'] == 'duplicate_or_out_of_order'
                assert frames[-1]['event'] == 'consume_error' and not frames[-1]['consumed']
            else:
                assert not result.result().success
                assert rows[-1]['aborted'] and rows[-1]['rejected_after_abort'] > 0
                assert frames[-1]['event'] == 'queue_overflow' and not frames[-1]['consumed']
                assert all(row['consumed'] and not row['event'] for row in frames[:-1])
                first_rejected = len(frames) - 1
                assert frames[-1]['stamp_ns'] == (100 + first_rejected // 30) * 10**9 + (first_rejected % 30) * 33333333
            summary = {'mode': mode, 'received': len(frames),
                       'consumed': sum(row['consumed'] for row in frames), 'test': 'PASS',
                       'callback_p95_ms': sorted(row['callback_ms'] for row in frames)[math.ceil(.95 * len(frames)) - 1],
                       'overflow_count': sum(row['event'] == 'queue_overflow' for row in frames),
                       'rejected_after_abort': rows[-1]['rejected_after_abort']}
            print(json.dumps(summary), flush=True)
            return summary
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            assert process.returncode == 0, (mode, process.returncode, output / 'node.log')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--vocabulary', type=Path, required=True)
    parser.add_argument('--settings', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--modes', nargs='+', default=[
        'native', 'legacy', 'legacy_offset', 'errors', 'overflow', 'encodings', 'qos', 'no_publishers', 'late_qos', 'replaced_qos'])
    args = parser.parse_args()
    for name in ('executable', 'vocabulary', 'settings', 'output'):
        setattr(args, name, getattr(args, name).resolve())
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID', '191')
    rclpy.init()
    try:
        results = [run(args, mode) for mode in args.modes]
        (args.output / 'test_results.json').write_text(json.dumps(results, indent=2) + '\n')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
