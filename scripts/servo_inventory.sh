#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 ]]; then
  echo 'usage: bash scripts/servo_inventory.sh /dev/serial/by-id/DEVICE' >&2
  exit 2
fi

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace=${ROBOT_WS:-"$HOME/so101_ws"}
driver_src="$workspace/src/feetech_ros2_driver/feetech_driver"
driver_lib="$workspace/install/feetech_ros2_driver/lib"
binary_dir="$HOME/.cache/so101"
mkdir -p "$binary_dir"

g++ -std=c++20 -O0 -I"$driver_src/include" \
  "$repo/scripts/servo_inventory.cpp" -L"$driver_lib" \
  "-Wl,-rpath,$driver_lib" -lcommunication_protocol -lserial_port \
  -lfmt -lspdlog -o "$binary_dir/servo_inventory"
exec "$binary_dir/servo_inventory" "$1"
