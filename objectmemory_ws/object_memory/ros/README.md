# Phase B — live ROS2 object_memory node

`object_memory_node.py` is the real-time deployment of the Phase A streaming
engine (`core/pose_buffer.py` + `pipeline/object_memory_runner.py`
`StreamingObjectMemory`). The algorithm is byte-identical to the offline/replay
path (verified: `scripts_test/eval_async.py` PARITY); only the event source is
ROS topics instead of file replay.

## Topics

| dir | topic (param) | type | rate |
|-----|---------------|------|------|
| in  | `pose_topic` = `/orbslam3/pose` | `geometry_msgs/PoseStamped` | ~30 Hz |
| in  | `detections_topic` = `/sam6d/detections` | `vision_msgs/Detection3DArray` | few Hz, **late** |
| in  | `caminfo_topic` | `sensor_msgs/CameraInfo` | once |
| out | `~/landmarks` | `geometry_msgs/PoseArray` (map frame) | per detection msg |

The detection message `header.stamp` MUST be the **capture** stamp of the frame
SAM ran on — the node looks the camera pose up at that past instant (time
alignment). Each `Detection3D.results[0]` carries `hypothesis.class_id`
(object_name), `hypothesis.score`, and `pose.pose` (`T_cam_obj`).

## Required change to the SAM node (one gate)

`sam6d_multiobject_node.py` currently publishes `geometry_msgs/PoseArray`, which
**drops object identity** (names live only in the overlay). To drive this node it
must publish `vision_msgs/Detection3DArray` — it already has name + score + pose
in hand at the publish block (`drawn` / `p`); wrap each into a `Detection3D` with
one `ObjectHypothesisWithPose`. No custom interface package is needed
(`vision_msgs` is already in the env).

## Run (3 terminals, 3 conda envs — nodes need different envs)

```bash
# T1 — SLAM (C++ ORB-SLAM3 ROS2; build once with colcon)
conda activate orbslam3 && source <orbslam_ws>/install/setup.bash
ros2 run orbslam3_ros2 rgbd   # publishes /orbslam3/pose

# T2 — SAM-6D (after the Detection3DArray change above)
conda activate sam6d_ros_humble
ros2 run sam6d_ros sam6d_multiobject_node   # publishes /sam6d/detections

# T3 — object memory (this node)
conda activate sam6d_ros_humble
python object_memory_node.py --ros-args \
    -p pose_topic:=/orbslam3/pose \
    -p detections_topic:=/sam6d/detections \
    -p caminfo_topic:=/camera/camera/color/camera_info

# T4 — drive the bag
ros2 bag play <bag>            # replays RGB-D + camera_info
```

`MultiThreadedExecutor` + `ReentrantCallbackGroup` (already set) let the fast
pose stream keep filling the buffer while a slow detection message is processed.
