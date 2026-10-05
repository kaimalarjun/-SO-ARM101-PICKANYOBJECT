#!/usr/bin/env bash
set -euo pipefail

# Ubuntu 24.04 only. Run on the physically connected robot host.
if [[ $(. /etc/os-release; echo "$ID:$VERSION_ID") != ubuntu:24.04 ]]; then
  echo 'ROS 2 Jazzy binary install requires Ubuntu 24.04' >&2; exit 2
fi
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace=${ROBOT_WS:-"$HOME/so101_ws"}
if [[ ! -f /etc/apt/sources.list.d/ros2.sources ]]; then
  sudo apt-get update
  sudo apt-get install -y curl ca-certificates
  version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -1)
  [[ -n $version ]] || { echo 'Could not find ros-apt-source release' >&2; exit 3; }
  deb="ros2-apt-source_${version#v}.noble_all.deb"
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT
  curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$version/$deb" -o "$tmp/$deb"
  sudo dpkg -i "$tmp/$deb"
fi
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-rviz2 ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-moveit ros-jazzy-moveit-py \
  ros-jazzy-usb-cam ros-jazzy-xacro ros-jazzy-nav2-common ros-jazzy-cv-bridge \
  python3-colcon-common-extensions python3-vcstool python3-rosdep \
  python3-opencv python3-yaml python3-pytest python3-venv
if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then sudo rosdep init; fi
rosdep update
mkdir -p "$workspace/src"
# ROS setup scripts reference optional variables that are unset under `set -u`.
set +u
source /opt/ros/jazzy/setup.bash
set -u
vcs import --skip-existing "$workspace/src" < "$repo/sources.repos"
# Verify existing checkouts rather than silently drifting from the published pins.
for item in 'ros2_so_arm e166df9d51f43b24da9b99047c6c51c306bda74f' \
            'feetech_ros2_driver 18aed7fb26d3e2b4c0b47762f39d8698b7032422'; do
  read -r dir expected <<< "$item"
  actual=$(git -C "$workspace/src/$dir" rev-parse HEAD)
  [[ $actual == "$expected" ]] || { echo "$dir is at $actual, expected $expected" >&2; exit 4; }
done
# Apply the documented startup, serial diagnostics and angle-alignment fixes.
driver_dir="$workspace/src/feetech_ros2_driver"
driver_patch="$repo/patches/feetech-driver.patch"
if git -C "$driver_dir" apply --check "$driver_patch"; then
  git -C "$driver_dir" apply "$driver_patch"
elif ! git -C "$driver_dir" apply --reverse --check "$driver_patch"; then
  echo 'Driver has partial or conflicting edits. Preserve them before updating the patch.' >&2
  exit 5
fi
ln -sfn "$repo/ros" "$workspace/src/kendra_robot"
cd "$workspace"
rosdep install --from-paths \
  src/kendra_robot src/ros2_so_arm/so_arm101_description \
  src/ros2_so_arm/so_arm_utils src/feetech_ros2_driver \
  --ignore-src -r -y
colcon build --symlink-install --packages-up-to \
  kendra_robot so_arm101_description feetech_ros2_driver
echo 'Installed ROS and built workspace. Launch remains mock-only by default.'
