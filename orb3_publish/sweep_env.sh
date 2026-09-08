#!/usr/bin/env bash
set -e
orb_publish_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$orb_publish_root/wsl2/env.sh"
export ORB3_PUBLISH_ENV=1
exec python3 "$orb_publish_root/orb3_publish/sweep.py" "$@"
