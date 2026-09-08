#!/usr/bin/env bash
set -euo pipefail
benchmark_dir=${1:-/opt/orb-live/benchmarks}
mkdir -p "$benchmark_dir"
cd "$benchmark_dir"
for sequence in xyz rpy; do
  archive="rgbd_dataset_freiburg1_${sequence}.tgz"
  if [[ ! -d "${archive%.tgz}" ]]; then
    curl -fL --retry 3 --connect-timeout 20 -C - -o "$archive" "https://cvg.cit.tum.de/rgbd/dataset/freiburg1/$archive"
    tar -xzf "$archive"
  fi
done
