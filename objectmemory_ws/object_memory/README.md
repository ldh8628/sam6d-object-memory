# object_memory

Standalone Python skeleton for SLAM pose assisted SAM-6D object ID association.

This module is intentionally independent from SAM-6D, SLAM backends, Unity, ROS2
custom messages, launch files, and runtime nodes. Story 0 only establishes core
data contracts, transform utilities, lifecycle transitions, memory storage, and
SAM_circle ros2 bag usability checking.

## Quick Checks

```bash
python3 -m compileall src scripts_test
pytest scripts_test -q
python3 scripts_test/check_sam_circle_bag.py \
  --bag-path ~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle \
  --output-report reports/story0_sam_circle_bag_check.md
```

If the SAM_circle bag is unavailable, tests use only in-memory synthetic data.
No fake bag, dataset, image, depth, or SAM output folders are created.

> Path note (this machine): the real SAM_circle bag lives at
> `../../data_slam/rgbd_imu_sdk_bag/SAM_circle`. Pass it explicitly via
> `--bag-path` when generating the real usability report.
