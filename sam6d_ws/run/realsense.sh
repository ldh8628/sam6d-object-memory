#!/usr/bin/env bash
set -eo pipefail

unset PYTHONPATH
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate realsense
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"

exec ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  enable_rgbd:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=848x480x30 \
  "$@"
