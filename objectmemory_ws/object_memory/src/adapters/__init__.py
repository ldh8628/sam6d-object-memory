"""Offline file adapters for object_memory.

These read only the three inputs the pipeline needs and convert them into the
Story 0 core contracts. They use the Python standard library only (no ROS2):

    slam_trajectory   - SLAM CameraTrajectory.txt (TUM) -> [SlamCameraPose]
    sam6d_pem         - SAM-6D detection_pem.json tree -> {frame_idx: [Sam6DDetection]}
    bag_frame_index   - ros2 bag .db3 color topic       -> frame_idx -> timestamp
"""
