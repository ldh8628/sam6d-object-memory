"""Runnable with Jazzy Python: actual CDR relay, topic isolation and fail-closed health."""
import json
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import time

from two_host_bridge import TOPICS, ros_node


def endpoint(config, mode, report, role):
    from sensor_msgs.msg import Image
    rclpy, node, specs = ros_node(config, 'remote' if mode == 'source' else 'local', lan=(mode == 'sink') == (role == 'remote'))
    try:
        if mode == 'source':
            publishers = [node.create_publisher(*spec) for spec in specs]
            image_pub = node.create_publisher(Image, '/camera/test/rgbd', 10) if role == 'remote' else None
            for i, value in ((1, 'map/first'), (3, 'ready')):
                message = specs[i][0]()
                message.data = value
                publishers[i].publish(message)
            start = time.monotonic()
            while time.monotonic() - start < 12:
                for i in (0, 2):
                    message = specs[i][0]()
                    if i == 0:
                        now = time.time_ns()
                        message.header.stamp.sec, message.header.stamp.nanosec = divmod(now, 10**9)
                        message.pose.position.x = 1.25
                    else:
                        message.data = 'tracking'
                    publishers[i].publish(message)
                if image_pub is not None:
                    image_pub.publish(Image())
                if time.monotonic() - start > 7:
                    message = specs[1][0]()
                    message.data = 'map/second'
                    publishers[1].publish(message)
                rclpy.spin_once(node, timeout_sec=.03)
        else:
            received = {}
            subscriptions = [node.create_subscription(typ, topic, lambda msg, i=i: received.setdefault(i, msg), qos)
                             for i, (typ, topic, qos) in enumerate(specs)]
            deadline = time.monotonic() + 8
            while len(received) < 4 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            assert set(received) == {0, 1, 2, 3}, list(received)
            assert received[0].pose.position.x == 1.25
            assert received[1].data == 'map/first'
            assert received[3].data == 'ready'
            topics = dict(node.get_topic_names_and_types())
            assert set(topics) == set(TOPICS), topics
            Path(report).write_text(json.dumps(topics))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test(role):
    config = dict(ssh=dict(target='localhost', remote_root='/tmp/bridge-test'),
                  network=dict(local_ip='127.0.0.1', remote_ip='127.0.0.2', ros_domain_id=174 if role == 'remote' else 176,
                               local_ptp_interface='lo', remote_ptp_interface='lo', ptp_max_offset_us=1000),
                  cameras=dict(slam_serial='123', sam_serial='456', slam_sync_mode=1, sam_sync_mode=3))
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        path = root / 'config.json'
        path.write_text(json.dumps(config))
        commands = [sys.executable, str(Path(__file__).resolve())]
        source = subprocess.Popen(commands + ['source', str(path), 'unused', role])
        bridge = subprocess.Popen([sys.executable, str(Path(__file__).with_name('two_host_bridge.py')),
                                   '--config', str(path), '--role', role, '--output', str(root / 'output')],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        sink = None
        try:
            assert select.select([bridge.stdout], [], [], 30)[0], 'bridge startup timeout'
            line = bridge.stdout.readline()
            assert line.startswith('TWO_HOST_READY '), line
            bridge.stdin.write('HEARTBEAT\n'); bridge.stdin.flush()
            time.sleep(1)
            sink = subprocess.Popen(commands + ['sink', str(path), str(root / 'received.json'), role])
            deadline = time.monotonic() + 14
            saw_healthy = saw_change = False
            while time.monotonic() < deadline:
                assert bridge.poll() is None, 'bridge died'
                bridge.stdin.write('HEARTBEAT\n'); bridge.stdin.flush()
                status = json.loads((root / 'output/bridge_status.json').read_text())
                saw_healthy |= status['healthy']
                saw_change |= status['reason'] == 'map_changed'
                if saw_change and sink.poll() is not None:
                    break
                time.sleep(.1)
            assert sink.wait(timeout=2) == 0
            assert saw_healthy and saw_change, (saw_healthy, saw_change)
            bridge.stdin.write('STOP\n'); bridge.stdin.flush()
            assert bridge.wait(timeout=8) == 0
            status = json.loads((root / 'output/bridge_status.json').read_text())
            assert not status['healthy'] and status['reason'] == 'bridge_stopped', status
            metrics = json.loads((root / 'output/two_host_bridge_metrics.json').read_text())
            assert metrics['counts']['/orbslam3/pose'] > 0 and metrics['pure_lan_latency_ms'] is None
        finally:
            for process in (sink, source, bridge):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
    print(f'two_host_bridge {role} ROS CDR/isolation/map-change/shutdown PASS')


if __name__ == '__main__':
    if len(sys.argv) > 1:
        endpoint(json.loads(Path(sys.argv[2]).read_text()), sys.argv[1], sys.argv[3], sys.argv[4])
    else:
        test('remote')
        test('local')
