#!/usr/bin/env bash
set -e
orb_publish_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$orb_publish_root/wsl2/env.sh"
cd "$orb_publish_root"
exec python3 -m unittest discover -s orb3_publish/tests -v
