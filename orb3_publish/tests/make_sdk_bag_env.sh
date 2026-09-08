#!/usr/bin/env bash
set -e
orb_publish_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source "$orb_publish_root/wsl2/env.sh"
exec python3 "$orb_publish_root/orb3_publish/tests/make_sdk_bag.py" "$@"
