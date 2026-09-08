"""Shared raw MCAP recording and lossless split-bag compatibility."""
from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import yaml

from camera_publish import SAM_CAMERA, SLAM_CAMERA, topics
from input_integrity import content_digest, content_hashes, rgbd_identity, write_json

CAMERAS = (SLAM_CAMERA, SAM_CAMERA)
CACHE_BYTES = 512 * 1024**2
RESERVE_BYTES = 10 * 1024**3
MCAP_CONFIG = {'compression': 'None', 'chunkSize': 4 * 1024**2,
               'noChunkCRC': False, 'noAttachmentCRC': False, 'noSummaryCRC': False,
               'enableDataCRC': True, 'noMessageIndex': False,
               'noChunkIndex': False, 'noSummary': False, 'noChunking': False}


def record_topics(cameras=CAMERAS):
    return [topics(camera)[key] for camera in cameras
            for key in ('rgbd', 'rgb_metadata', 'depth_metadata')] + [
                '/diagnostics', '/rosout', '/orbslam3/pose',
                '/orbslam3/tracking_state', '/orbslam3/map_id']


def legacy_topics(cameras=CAMERAS):
    return [topics(camera)[key] for camera in cameras
            for key in ('rgb', 'depth', 'caminfo', 'depth_caminfo')]


def recorder_args(capture, cameras=CAMERAS):
    capture = Path(capture)
    capture.parent.mkdir(parents=True, exist_ok=True)
    storage, qos = capture.parent/'mcap_storage.yaml', capture.parent/'record_qos.yaml'
    storage.write_text(yaml.safe_dump(MCAP_CONFIG, sort_keys=False))
    policies = {t: {'reliability': 'reliable', 'history': 'keep_last', 'depth': 120,
                    'durability': 'volatile'} for t in record_topics(cameras)}
    policies['/orbslam3/map_id']['durability'] = 'transient_local'
    qos.write_text(yaml.safe_dump(policies, sort_keys=False))
    return ['ros2', 'bag', 'record', '--storage', 'mcap', '--output', str(capture),
            '--storage-config-file', str(storage), '--qos-profile-overrides-path', str(qos),
            '--max-cache-size', str(CACHE_BYTES), '--node-name', 'input_recorder',
            '--disable-keyboard-controls', *record_topics(cameras)]


def check_space(path, seconds, *, cameras=2, compatibility=False):
    if not math.isfinite(seconds) or seconds <= 0 or cameras < 1:
        raise ValueError('recording budget must have positive duration and camera count')
    path = Path(path)
    while not path.exists():
        path = path.parent
    # Configured aligned payload: RGB8 + Z16 at 640x480x30. 15% covers headers/indexes.
    raw = math.ceil(cameras * 640 * 480 * 5 * 30 * seconds * 1.15)
    required = raw * (2 if compatibility else 1) + RESERVE_BYTES + 2*CACHE_BYTES
    free = shutil.disk_usage(path).free
    result = dict(raw_estimate_bytes=raw, compatibility_estimate_bytes=raw if compatibility else 0,
                  cache_budget_bytes=2*CACHE_BYTES, reserve_bytes=RESERVE_BYTES,
                  required_bytes=required, free_bytes=free, seconds=seconds)
    if free < required:
        raise RuntimeError('insufficient recording/conversion space: ' + json.dumps(result))
    return result


def open_reader(path):
    import rosbag2_py as bag
    path = Path(path)
    metadata = yaml.safe_load((path/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                bag.ConverterOptions('', ''))
    return reader


def bag_rows(path):
    """Read native combined frames directly; never re-pair or rewrite their timestamps."""
    from rclpy.serialization import deserialize_message
    from realsense2_camera_msgs.msg import RGBD, Metadata
    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    combined = {name for name,typ in types.items() if typ == 'realsense2_camera_msgs/msg/RGBD'}
    sequence = Counter()
    while reader.has_next():
        topic, data, recorded_ns = reader.read_next()
        if types[topic] == 'realsense2_camera_msgs/msg/Metadata':
            message = deserialize_message(data, Metadata)
            camera, stream = topic.rsplit('/', 3)[-3:-1]
            yield dict(kind='metadata',stage='bag',camera=camera,
                stream='rgb' if stream=='color' else stream,
                stamp_ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
                metadata=json.loads(message.json_data),recorded_ns=recorded_ns)
            continue
        if topic not in combined:
            continue
        message = deserialize_message(data, RGBD)
        camera = topic.rsplit('/', 2)[-2]
        yield dict(kind='rgbd', stage='bag', camera=camera, sequence=sequence[camera],
                   recorded_ns=recorded_ns, **rgbd_identity(message), hashes=content_hashes(message))
        sequence[camera] += 1


def split_records(reader):
    """Yield unchanged nested messages at the original bag timestamp, in input order."""
    from rclpy.serialization import deserialize_message, serialize_message
    from realsense2_camera_msgs.msg import RGBD
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if types[topic] != 'realsense2_camera_msgs/msg/RGBD':
            continue
        camera = topic.rsplit('/', 2)[-2]
        message = deserialize_message(data, RGBD)
        for key, field, typ in [('rgb', 'rgb', 'Image'), ('depth', 'depth', 'Image'),
                                ('caminfo', 'rgb_camera_info', 'CameraInfo'),
                                ('depth_caminfo', 'depth_camera_info', 'CameraInfo')]:
            yield topics(camera)[key], 'sensor_msgs/msg/'+typ, serialize_message(getattr(message, field)), timestamp


def convert_to_sqlite(source, destination):
    import rosbag2_py as bag
    source, destination = Path(source), Path(destination)
    if destination.exists():
        raise ValueError(f'refusing to replace compatibility bag: {destination}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    size = sum(p.stat().st_size for p in source.iterdir() if p.is_file())
    if shutil.disk_usage(destination.parent).free < size*1.15 + RESERVE_BYTES:
        raise RuntimeError('compatibility conversion would leave less than 10 GiB free')
    reader = open_reader(source)
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(destination), storage_id='sqlite3'),
                bag.ConverterOptions('', ''))
    known, count = set(), Counter()
    try:
        for topic, typ, data, timestamp in split_records(reader):
            if topic not in known:
                writer.create_topic(bag.TopicMetadata(id=len(known)+1, name=topic, type=typ,
                    serialization_format='cdr', offered_qos_profiles=[bag._storage.QoS(120).reliable()]))
                known.add(topic)
            writer.write(topic, data, timestamp)
            count[topic] += 1
    finally:
        writer.close()
    if not count:
        raise ValueError('no combined RGBD messages in source bag')
    verify_conversion(source, destination)
    result = dict(status='PASS', source=str(source), destination=str(destination), counts=dict(count),
                  comparison='every RGB/depth buffer byte, ROS message field and bag timestamp in order')
    write_json(destination/'conversion_integrity.json', result)
    return result


def verify_conversion(source, destination):
    # Independently reopen output; compare every ROS field, time and order.
    # CDR alignment padding is not part of message content.
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    actual = open_reader(destination)
    for topic, typ, data, timestamp in split_records(open_reader(source)):
        if not actual.has_next():
            raise RuntimeError('compatibility conversion truncated')
        actual_topic, actual_data, actual_stamp = actual.read_next()
        message_type = get_message(typ)
        if ((actual_topic, actual_stamp) != (topic, timestamp) or
                content_digest(deserialize_message(data, message_type)) !=
                content_digest(deserialize_message(actual_data, message_type))):
            raise RuntimeError('compatibility conversion content/order/timestamp mismatch')
    if actual.has_next():
        raise RuntimeError('compatibility conversion contains extra messages')
    return True


def prepare_replay(source, destination):
    from rclpy.serialization import deserialize_message
    from realsense2_camera_msgs.msg import RGBD
    source, destination = Path(source).resolve(), Path(destination)
    reader = open_reader(source)
    combined = {t.name for t in reader.get_all_topics_and_types()
                if t.type == 'realsense2_camera_msgs/msg/RGBD'}
    metadata = yaml.safe_load((source/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    counts = {t['topic_metadata']['name']: t['message_count'] for t in metadata['topics_with_message_count']}
    first = {}
    while reader.has_next() and len(first) < len(combined):
        topic, data, timestamp = reader.read_next()
        if topic in combined and topic not in first:
            first[topic] = (deserialize_message(data, RGBD), timestamp)
    result = {'source_format': 'native_rgbd', 'source': str(source)}
    for role,camera in [('SLAM',SLAM_CAMERA), ('SAM',SAM_CAMERA)]:
        topic = topics(camera)['rgbd']
        if topic not in first:
            raise ValueError(f'native replay requires both camera topics; missing {topic}')
        msg, timestamp = first[topic]
        info = msg.rgb_camera_info
        result[role] = dict(path=role, rgbd_topic=topic, color_topic=topics(camera)['rgb'],
            depth_topic=topics(camera)['depth'], caminfo_topic=topics(camera)['caminfo'],
            t_start_ns=timestamp, color_count=counts[topic], color_hz=30.0,
            camera=dict(width=info.width,height=info.height,K=list(info.k),D=list(info.d),
                        distortion_model=info.distortion_model))
    destination.mkdir(parents=True,exist_ok=True)
    write_json(destination/'info.json',result)
    for role in ('SLAM','SAM'):
        link=destination/role
        if not link.exists(): link.symlink_to(source,target_is_directory=True)
        elif link.resolve()!=source: raise ValueError(f'replay layout points at another bag: {link}')
    return result


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--to-sqlite', type=Path)
    parser.add_argument('--index', type=Path)
    parser.add_argument('--prepare-replay', type=Path)
    parser.add_argument('--verify-sqlite', type=Path)
    args = parser.parse_args()
    if args.verify_sqlite:
        verify_conversion(args.source,args.verify_sqlite)
    if args.prepare_replay:
        prepare_replay(args.source, args.prepare_replay)
    if args.to_sqlite:
        print(json.dumps(convert_to_sqlite(args.source, args.to_sqlite), indent=2))
    if args.index:
        with args.index.open('x') as handle:
            for row in bag_rows(args.source):
                handle.write(json.dumps(row)+'\n')
    if not args.to_sqlite and not args.index and not args.prepare_replay and not args.verify_sqlite:
        parser.error('provide --to-sqlite or --index')


if __name__ == '__main__':
    main()
