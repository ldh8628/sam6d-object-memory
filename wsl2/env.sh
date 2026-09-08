#!/usr/bin/env bash
# Source from a new Ubuntu-22.04 terminal.
orb_live_root=$(cd -P -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
export ORB_LIVE_WORKSPACE="$orb_live_root/orbslam_ws"
# RGB-D frames need more space than Fast DDS's default 512 KiB SHM segment.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$orb_live_root/wsl2/fastdds_live.xml}"
if [[ ! -f /opt/orb-live/ros2_jazzy/install/setup.bash ]]; then
  echo 'Jazzy has not finished building: /opt/orb-live/ros2_jazzy/install/setup.bash' >&2
  return 1
fi
source /opt/orb-live/ros2_jazzy/install/setup.bash
export CMAKE_PREFIX_PATH="/opt/orb-live:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/opt/orb-live/lib:${LD_LIBRARY_PATH:-}"
export PATH="/opt/orb-live/bin:$PATH"
for orb_live_python_dir in /opt/orb-live/lib/python*/{site,dist}-packages; do
  [[ ! -d $orb_live_python_dir ]] || export PYTHONPATH="$orb_live_python_dir:${PYTHONPATH:-}"
done
[[ ! -f /opt/orb-live/realsense_ws/install/setup.bash ]] || source /opt/orb-live/realsense_ws/install/setup.bash
[[ ! -f $ORB_LIVE_WORKSPACE/install_jazzy/setup.bash ]] || source "$ORB_LIVE_WORKSPACE/install_jazzy/setup.bash"
unset orb_live_root orb_live_python_dir
