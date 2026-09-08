#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "$0")/env.sh"
set -u
[[ -z ${CONDA_PREFIX:-} ]] || { echo 'Deactivate Conda first' >&2; exit 1; }
cd "$ORB_LIVE_WORKSPACE"
[[ -f src/ORB_SLAM3/package.xml ]] || { echo 'Repository checkout missing' >&2; exit 1; }
[[ -s src/ORB_SLAM3/Vocabulary/ORBvoc.txt ]] || tar -xzf src/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz -C src/ORB_SLAM3/Vocabulary
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
packages=(distributed_slam_interfaces orbslam3_core orbslam3_ros2)
if [[ ${1:-} == --core-only ]]; then packages=(orbslam3_core); fi
for package in "${packages[@]}"; do
  colcon build --executor sequential --packages-select "$package" \
    --build-base build_jazzy --install-base install_jazzy \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_ORB_SLAM3_EXAMPLES=OFF -DBUILD_TESTING=OFF
  set +u
  source install_jazzy/setup.bash
  set -u
done
if [[ ${1:-} == --core-only ]]; then exit 0; fi
ros2 pkg executables orbslam3_ros2
ldd install_jazzy/orbslam3_ros2/lib/orbslam3_ros2/rgbd_node > build_jazzy/rgbd-node-ldd.txt
if grep -q 'not found' build_jazzy/rgbd-node-ldd.txt; then cat build_jazzy/rgbd-node-ldd.txt; exit 1; fi
