"""Actual installed ROS messages and MCAP/SQLite roundtrip; no sensor needed."""
from array import array
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import rosbag2_py as bag
from rclpy.serialization import serialize_message
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import CameraInfo, Image

from camera_publish import topics
from input_integrity import content_digest, content_hashes, rgbd_identity
from rgbd_capture import (MCAP_CONFIG, bag_rows, convert_to_sqlite, open_reader,
                          prepare_replay, recorder_args, split_records, verify_conversion)


def message(index):
    msg=RGBD()
    for field,offset in [('rgb',0),('depth',7)]:
        image=getattr(msg,field)
        image.header.frame_id='optical_'+field
        image.header.stamp.sec=100+index
        image.header.stamp.nanosec=1000+offset
        image.height=2;image.width=3;image.is_bigendian=1
        image.encoding='rgb8' if field=='rgb' else '16UC1'
        image.step=12 if field=='rgb' else 8  # deliberate row padding
        image.data=array('B',[(index+offset)%256]*(image.step*image.height))
    for field,offset in [('rgb_camera_info',11),('depth_camera_info',19)]:
        info=getattr(msg,field)
        info.header.frame_id=field;info.header.stamp.sec=100+index;info.header.stamp.nanosec=offset
        info.width=3;info.height=2;info.distortion_model='plumb_bob'
        info.d=[.01,.02,0.,0.,0.]
        info.k=[2.,0.,1.,0.,2.,1.,0.,0.,1.]
        info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        info.p=[2.,0.,1.,0.,0.,2.,1.,0.,0.,0.,1.,0.]
        info.binning_x=2;info.roi.x_offset=1;info.roi.do_rectify=True
    msg.header=msg.rgb.header
    return msg


class CaptureChecks(unittest.TestCase):
    def test_full_ros_roundtrip_native_and_sqlite(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'raw';dest=root/'split'
            command=recorder_args(source)
            self.assertIn('--storage-config-file',command)
            self.assertFalse(any(t.endswith('image_raw') for t in command))
            writer=bag.SequentialWriter()
            writer.open(bag.StorageOptions(uri=str(source),storage_id='mcap',
                storage_config_uri=str(root/'mcap_storage.yaml')),bag.ConverterOptions('',''))
            expected=[]
            for idx,camera in enumerate(('slam_camera','sam_camera')):
                writer.create_topic(bag.TopicMetadata(id=idx+1,name=topics(camera)['rgbd'],
                    type='realsense2_camera_msgs/msg/RGBD',serialization_format='cdr'))
            for i in range(4):
                for camera in ('slam_camera','sam_camera'):
                    msg=message(i)
                    writer.write(topics(camera)['rgbd'],serialize_message(msg),int(1e12+i*1e9))
                    expected.append(dict(camera=camera,**rgbd_identity(msg),hashes=content_hashes(msg)))
            writer.close()
            actual=list(bag_rows(source))
            self.assertEqual([{k:r[k] for k in ('camera','stamp_ns','depth_stamp_ns','hashes')} for r in actual],expected)
            result=convert_to_sqlite(source,dest)
            self.assertEqual(result['status'],'PASS')
            raw_records=list(split_records(open_reader(source)))
            reader=open_reader(dest);split=[]
            while reader.has_next(): split.append(reader.read_next())
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
            for (topic,typ,data,stamp),(topic2,data2,stamp2) in zip(raw_records,split):
                self.assertEqual((topic,stamp),(topic2,stamp2))
                self.assertEqual(content_digest(deserialize_message(data,get_message(typ))),
                                 content_digest(deserialize_message(data2,get_message(typ))))
            layout=root/'layout';info=prepare_replay(source,layout)
            self.assertEqual((layout/'SLAM').resolve(),(layout/'SAM').resolve())
            self.assertEqual(info['SAM']['color_count'],4)
            self.check_replay(layout, expected)
            legacy_layout=root/'legacy_layout';legacy_layout.mkdir()
            for role in ('SLAM','SAM'):
                info[role].pop('rgbd_topic')
                (legacy_layout/role).symlink_to(dest,target_is_directory=True)
            (legacy_layout/'info.json').write_text(json.dumps(info))
            self.check_replay(legacy_layout, expected, legacy=True)
            with self.assertRaises(ValueError): convert_to_sqlite(source,dest)
            # A truncated compatibility bag must fail comparison, not be repaired by duplication.
            import sqlite3
            db=next(dest.glob('*.db3'))
            with sqlite3.connect(db) as conn: conn.execute('DELETE FROM messages WHERE id=(SELECT MAX(id) FROM messages)')
            self.assertEqual(len(split),32)
            reader=open_reader(dest);count=0
            while reader.has_next(): reader.read_next();count+=1
            self.assertEqual(count,31)
            with self.assertRaisesRegex(RuntimeError,'truncated'):
                verify_conversion(source,dest)

    def check_replay(self, layout, expected, legacy=False):
        import os
        import time
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from run_object_memory_rosbag import _play_pair, _stop
        os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
        os.environ['ROS_DOMAIN_ID']='191'
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
        rclpy.init();node=rclpy.create_node('native_replay_verifier');received=[]
        qos=QoSProfile(depth=120,reliability=ReliabilityPolicy.RELIABLE)
        if legacy:
            expected = [dict(camera=camera, field=field, digest=content_digest(getattr(message(i),field)))
                        for i in range(4) for camera in ('slam_camera','sam_camera')
                        for field in ('rgb','depth','rgb_camera_info','depth_camera_info')]
        for camera,prefix in [('slam_camera','/slam_camera'),('sam_camera','/camera/camera')]:
            if not legacy:
                node.create_subscription(RGBD,prefix+'/rgbd',lambda msg,camera=camera:received.append(
                    dict(camera=camera,**rgbd_identity(msg),hashes=content_hashes(msg))),qos)
            else:
                for field,suffix,typ in [('rgb','color/image_raw',Image),('depth','aligned_depth_to_color/image_raw',Image),
                    ('rgb_camera_info','color/camera_info',CameraInfo),('depth_camera_info','aligned_depth_to_color/camera_info',CameraInfo)]:
                    node.create_subscription(typ,prefix+'/'+suffix,lambda msg,camera=camera,field=field:received.append(
                        dict(camera=camera,field=field,digest=content_digest(msg))),qos)
        process=_play_pair(layout,{'SLAM':1.,'SAM':1.},1.,191)
        try:
            deadline=time.monotonic()+20
            while time.monotonic()<deadline and (process.poll() is None or len(received)<len(expected)):
                rclpy.spin_once(node,timeout_sec=.05)
            self.assertEqual(process.poll(),0)
            for camera in ('slam_camera','sam_camera'):
                # ROS preserves order per topic; inter-topic delivery order is unspecified.
                for field in ('rgb','depth','rgb_camera_info','depth_camera_info') if legacy else (None,):
                    self.assertEqual([r for r in received if r['camera']==camera and r.get('field')==field],
                                     [r for r in expected if r['camera']==camera and r.get('field')==field])
        finally:
            _stop(process);node.destroy_node();rclpy.shutdown()


if __name__=='__main__': unittest.main()
