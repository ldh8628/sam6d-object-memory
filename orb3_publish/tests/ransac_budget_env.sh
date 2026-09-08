#!/usr/bin/env bash
set -eo pipefail
orb_publish_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source "$orb_publish_root/wsl2/env.sh"
set -u
orb_test_core="$orb_publish_root/orbslam_ws/src/ORB_SLAM3"
orb_test_output=$(mktemp -d)
trap 'rm -rf "$orb_test_output"' EXIT
read -r -a orb_test_opencv <<< "$(pkg-config --cflags --libs opencv4)"
# Match the core's Eigen alignment ABI (-march=native in its CMake flags).
g++ -std=c++17 -O2 -g -march=native -DHAVE_GLEW -DCOMPILEDWITHC11 "$orb_publish_root/orb3_publish/tests/ransac_budget.cpp" \
  -I"$orb_test_core" -I"$orb_test_core/include" -I"$orb_test_core/Thirdparty/Sophus" \
  -I"$orb_test_core/include/CameraModels" \
  -I/usr/include/eigen3 -I/opt/orb-live/include "${orb_test_opencv[@]}" -L"$orb_test_core/lib" -lORB_SLAM3 -lDBoW2 \
  -L/opt/orb-live/lib -lpango_display -lpango_opengl -lpango_windowing -lpango_vars -lpango_core \
  -lGLEW -lGL -lboost_serialization \
  -Wl,-rpath,"$orb_test_core/lib" -o "$orb_test_output/ransac_budget"
"$orb_test_output/ransac_budget"
