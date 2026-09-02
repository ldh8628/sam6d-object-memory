"""Offline object_memory pipeline (Story 1+2 minimal integration).

Combines three inputs into persistent object memory:

    1. SLAM CameraTrajectory  -> T_map_cam per timestamp
    2. SAM-6D PEM detections  -> T_cam_obj per frame
    3. ros2 bag color topic   -> frame index -> timestamp

and applies T_map_obj = T_map_cam * T_cam_obj with an object_name-keyed
association MVP.
"""
