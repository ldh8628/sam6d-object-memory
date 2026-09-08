#!/usr/bin/env bash
set -e
orb_publish_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$orb_publish_root/wsl2/env.sh"
cd "$orb_publish_root/orbslam_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --executor sequential --packages-select orbslam3_ros2 --build-base build_jazzy --install-base install_jazzy --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
