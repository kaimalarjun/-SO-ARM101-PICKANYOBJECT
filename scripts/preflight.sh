#!/usr/bin/env bash
set -euo pipefail
echo "OS: $(. /etc/os-release; echo "$PRETTY_NAME")"
echo "ROS: $([[ -f /opt/ros/jazzy/setup.bash ]] && echo installed || echo absent)"
echo "Follower stable port: $([[ -e /dev/LeRobotFollower ]] && echo present || echo absent)"
echo "Camera nodes:"
find /dev -maxdepth 1 -name 'video*' -printf '%f\n' 2>/dev/null || true
echo "Serial nodes:"
find /dev -maxdepth 1 \( -name 'ttyUSB*' -o -name 'ttyACM*' \) -printf '%f\n' 2>/dev/null || true
echo "Physical label, supply rating, servo IDs, and cutoff require operator inspection."
echo 'No motion or serial write was performed.'
