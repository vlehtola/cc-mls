#!/usr/bin/env bash
# Run with bash; requires an existing Ubuntu 20.04 ROS Noetic installation.
set -eo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 20.04 ]]; then
  echo "This installer targets Ubuntu 20.04 + ROS Noetic. See docs/新电脑安装.md." >&2
  exit 1
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "Install ROS Noetic first; this script does not configure the ROS apt repository." >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
set -u
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(realpath -m "${1:-$package_dir/../..}")"
if [[ "$package_dir" != "$workspace_dir/src/cc_mls_slam" ]]; then
  echo "Put cc_mls_slam in WORKSPACE/src/cc_mls_slam, then pass WORKSPACE as argument." >&2
  exit 1
fi
build_jobs="${BUILD_JOBS:-2}"
sudo apt-get update
sudo apt-get install -y build-essential cmake git pkg-config libeigen3-dev \
  libpcl-dev libopencv-dev libtbb-dev libboost-all-dev libapr1-dev libgtest-dev \
  python3-rosdep python3-catkin-pkg python3-rospkg \
  ros-noetic-roscpp ros-noetic-rospy ros-noetic-rosbag ros-noetic-nav-msgs \
  ros-noetic-sensor-msgs ros-noetic-geometry-msgs ros-noetic-std-msgs \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf \
  ros-noetic-eigen-conversions ros-noetic-visualization-msgs \
  ros-noetic-message-generation ros-noetic-message-runtime \
  ros-noetic-message-filters ros-noetic-rviz ros-noetic-rostest

dependency_dir="$workspace_dir/.cc_mls_dependencies"
mkdir -p "$dependency_dir"
clone_if_missing() {
  local repository_url="$1" destination="$2" revision="$3"
  if [[ -e "$destination" ]]; then
    git -C "$destination" rev-parse --is-inside-work-tree >/dev/null
    echo "Reusing $destination; existing checkout will not be changed."
  elif [[ "$revision" == master ]]; then
    git clone --depth 1 --branch master "$repository_url" "$destination"
  else
    git clone --depth 1 --branch "$revision" "$repository_url" "$destination"
  fi
}
clone_if_missing https://github.com/Livox-SDK/Livox-SDK.git "$dependency_dir/Livox-SDK" master
cmake -S "$dependency_dir/Livox-SDK" -B "$dependency_dir/Livox-SDK/build-linux" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build "$dependency_dir/Livox-SDK/build-linux" --parallel "$build_jobs"
sudo cmake --install "$dependency_dir/Livox-SDK/build-linux"
# Install the SDK first so the vendor driver never enters its download fallback.
test -f /usr/local/lib/liblivox_sdk_static.a

clone_if_missing https://github.com/borglab/gtsam.git "$dependency_dir/gtsam" 4.2
if [[ "$(git -C "$dependency_dir/gtsam" describe --tags --exact-match 2>/dev/null || true)" != 4.2 ]]; then
  echo "Expected the GTSAM 4.2 tag; use a fresh dependency directory or inspect existing checkout." >&2
  exit 1
fi
cmake -S "$dependency_dir/gtsam" -B "$dependency_dir/gtsam/build-linux" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_TBB=OFF
cmake --build "$dependency_dir/gtsam/build-linux" --parallel "$build_jobs"
sudo cmake --install "$dependency_dir/gtsam/build-linux"
sudo ldconfig

clone_if_missing https://github.com/Livox-SDK/livox_ros_driver.git \
  "$workspace_dir/src/livox_ros_driver" master
test -f "$workspace_dir/src/livox_ros_driver/livox_ros_driver/package.xml"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update --include-eol-distros
# gtsam is a manually installed dependency; the base rosdep database has no gtsam key.
rosdep install --from-paths "$workspace_dir/src" --ignore-src --rosdistro noetic \
  --skip-keys gtsam -y
{
  echo "GTSAM tag: 4.2; GTSAM_USE_SYSTEM_EIGEN=ON"
  for repository in "$dependency_dir/Livox-SDK" "$dependency_dir/gtsam" "$workspace_dir/src/livox_ros_driver"; do
    printf '%s ' "$repository"
    git -C "$repository" rev-parse HEAD
  done
  g++ --version | head -n 1
  cmake --version | head -n 1
  dpkg-query -W libeigen3-dev libpcl-dev libopencv-dev libtbb-dev
} > "$dependency_dir/installed_versions.txt"
cd "$workspace_dir"
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$build_jobs"
echo "Build completed. Run: bash src/cc_mls_slam/scripts/verify_installation.sh $workspace_dir"
