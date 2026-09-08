#!/usr/bin/env bash
set -e
orb_publish_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$orb_publish_root/wsl2/env.sh"
export ORB3_PUBLISH_ENV=1 ROS_DOMAIN_ID=78 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
exec python3 "$orb_publish_root/orb3_publish/benchmark.py" "$@"
