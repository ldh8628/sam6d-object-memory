"""Exercise real SAM receiver subscription -> single worker -> shared memory -> drain."""
from array import array
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'sam6d_ws/realtime'))
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from realsense2_camera_msgs.msg import RGBD
from std_srvs.srv import Trigger
from sam6d_receiver_node import Receiver


class ReceiverChecks(unittest.TestCase):
    def test_native_consumption_and_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            rclpy.init()
            node=Receiver(dict(topics={'rgbd':'/receiver_test/rgbd'},
                runtime={'use_sim_time':False,'qos_depth':120},
                output={'dir':directory,'overlay_topic':''},
                slam={'timestamp_tolerance_ms':0},input={'sam_camera':{'name':'sam_camera','serial':'test'}}))
            sender=rclpy.create_node('receiver_test')
            pub=sender.create_publisher(RGBD,'/receiver_test/rgbd',QoSProfile(depth=120,reliability=ReliabilityPolicy.RELIABLE))
            executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node);executor.add_node(sender)
            def wait(condition,seconds=10):
                deadline=time.monotonic()+seconds
                while not condition():
                    if time.monotonic()>deadline: raise AssertionError('receiver timeout')
                    executor.spin_once(timeout_sec=.01)
            try:
                wait(lambda:pub.get_subscription_count()>0)
                for i in range(10):
                    msg=RGBD()
                    for image,size,encoding in [(msg.rgb,3,'rgb8'),(msg.depth,2,'16UC1')]:
                        image.width=640;image.height=480;image.step=640*size;image.encoding=encoding
                        image.header.stamp.sec=100+i
                        image.header.stamp.nanosec=3 if size==2 else 0
                        image.data=array('B',bytes(640*480*size))
                    msg.rgb_camera_info.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.]
                    msg.rgb_camera_info.width=640;msg.rgb_camera_info.height=480
                    pub.publish(msg)
                    if i < 8:
                        wait(lambda:node._input.received==i+1)
                # Two acknowledged frames remain in DDS while the executor is idle.
                self.assertTrue(pub.wait_for_all_acked(Duration(seconds=5)))
                self.assertEqual(node._input.received,8)
                response=node._drain_input(Trigger.Request(),Trigger.Response())
                self.assertTrue(response.success,response.message)
                rows=[json.loads(line) for line in (Path(directory)/'sam_input_health.jsonl').read_text().splitlines()]
                self.assertEqual(len(rows),10)
                self.assertTrue(all(r['consumed'] for r in rows))
                self.assertEqual([r['sequence'] for r in rows],list(range(10)))
                self.assertEqual(node.n_written,10)
                self.assertTrue(all(r['depth_stamp_ns']-r['stamp_ns']==3 for r in rows))
            finally:
                executor.shutdown();node.close();node.destroy_node();sender.destroy_node();rclpy.shutdown()


if __name__=='__main__': unittest.main()
