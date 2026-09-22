#!/usr/bin/env bash
set -eo pipefail
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(realpath -m "${1:-$package_dir/../..}")"
cd "$workspace_dir"
source /opt/ros/noetic/setup.bash
catkin_make install -DCMAKE_BUILD_TYPE=Release -j"${BUILD_JOBS:-2}"
source "$workspace_dir/install/setup.bash"
test "$(rospack find cc_mls_slam)" = "$workspace_dir/install/share/cc_mls_slam"
rosmsg show cc_mls_slam/Pose6D >/dev/null
rosmsg show livox_ros_driver/CustomMsg >/dev/null
for binary in "$workspace_dir/install/lib/cc_mls_slam/cc_mls_frontend" \
              "$workspace_dir/install/lib/cc_mls_slam/cc_mls_backend" \
              "$workspace_dir/install/lib/libcc_mls_dynamic.so"; do
  test -f "$binary"
  linkage="$(ldd "$binary")"
  printf '%s\n%s\n' "$binary" "$linkage"
  if grep -q 'not found' <<< "$linkage"; then exit 1; fi
done
for launch_file in mapping_ouster128.launch mapping_horizon.launch mapping_hap.launch; do
  roslaunch --nodes cc_mls_slam "$launch_file" rviz:=false
done
catkin_make run_tests_cc_mls_slam -j"${BUILD_JOBS:-2}"
catkin_test_results "$workspace_dir/build/test_results"
echo "Install paths, shared-library resolution, launch parsing and gtests passed."
echo "Sensor input, HAP adaptation and rosbag behavior still require separate validation."
