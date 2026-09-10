"""Replay converted RGB-D bags with one outstanding ORB frame, preserving stamps."""
import argparse
import json
import math
from pathlib import Path
import time


def completion(handle, rgb_stamp, depth_stamp):
    while True:
        position = handle.tell()
        line = handle.readline()
        if not line.endswith('\n'):
            handle.seek(position)
            return False
        row = json.loads(line)
        if row.get('event'):
            raise RuntimeError(f'ORB input failure: {row}')
        if row.get('kind') != 'rgbd':
            continue
        if (row.get('stamp_ns'), row.get('depth_stamp_ns')) != (rgb_stamp, depth_stamp):
            raise RuntimeError('ORB completion does not match the outstanding RGB-D pair')
        if row.get('consumed') is not True:
            raise RuntimeError('ORB did not consume the outstanding frame')
        return True


def run(config):
    import yaml
    import rclpy
    import rosbag2_py as bag
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from input_integrity import stamp_ns

    cfg = yaml.safe_load(config.read_text())
    rate = float(cfg['bag']['rate'])
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('playback rate must be finite and positive')
    output = Path(cfg['output']['dir'])
    result = dict(status='FAIL', frames=0, mode='wait_for_ORB_completion')
    rclpy.init()
    node = rclpy.create_node('paced_orb_player')
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    names = [cfg['topics']['rgb'], cfg['topics']['depth']]
    publishers = {name: node.create_publisher(Image, name, qos) for name in names}
    clock = node.create_publisher(Clock, '/clock', 10) if cfg['bag'].get('clock') else None
    def wait_until(predicate, description):
        deadline = time.monotonic() + 60
        while not predicate():
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise TimeoutError(description)
            rclpy.spin_once(node, timeout_sec=.001)
    try:
        wait_until(lambda: all(p.get_subscription_count() for p in publishers.values()), 'ORB subscribers not ready')
        reader = bag.SequentialReader()
        reader.open(bag.StorageOptions(uri=cfg['bag']['path'], storage_id='sqlite3'), bag.ConverterOptions('', ''))
        reader.set_filter(bag.StorageFilter(topics=names))
        pending = {}
        previous_stamp = previous_sent = None
        with (output/'orb_input_health.jsonl').open() as health:
            while reader.has_next():
                topic, data, _ = reader.read_next()
                if topic in pending:
                    raise RuntimeError('Converted bag has an unpaired RGB/depth image')
                pending[topic] = deserialize_message(data, Image)
                if len(pending) != 2:
                    continue
                rgb, depth = (pending[name] for name in names)
                stamp = stamp_ns(rgb)
                if previous_stamp is not None:
                    if stamp <= previous_stamp:
                        raise RuntimeError('Non-increasing source image timestamp')
                    target = previous_sent + (stamp-previous_stamp)/1e9/rate
                    while time.monotonic() < target:
                        rclpy.spin_once(node, timeout_sec=max(0., min(.01, target-time.monotonic())))
                if clock:
                    msg = Clock(); msg.clock = rgb.header.stamp; clock.publish(msg)
                previous_sent = time.monotonic()
                for name in names:
                    publishers[name].publish(pending[name])
                wait_until(lambda: completion(health, stamp, stamp_ns(depth)), 'ORB frame completion timeout')
                previous_stamp = stamp
                pending.clear()
                result['frames'] += 1
            if pending or not result['frames']:
                raise RuntimeError('Empty or incomplete RGB-D replay')
        result['status'] = 'PASS'
    except BaseException as exc:
        result['error'] = repr(exc)
        raise
    finally:
        (output/'paced_replay.json').write_text(json.dumps(result, indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    run(parser.parse_args().config)
