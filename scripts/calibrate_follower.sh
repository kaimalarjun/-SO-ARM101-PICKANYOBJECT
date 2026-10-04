#!/usr/bin/env bash
set -euo pipefail
umask 077
if [[ $# != 1 ]]; then
  echo 'usage: bash scripts/calibrate_follower.sh /dev/serial/by-id/DEVICE' >&2
  exit 2
fi
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
venv=${CALIBRATION_VENV:-"$HOME/.venvs/so101-calibration"}
state=${CALIBRATION_STATE:-"$HOME/.config/so101"}
[[ -x "$venv/bin/lerobot-calibrate" ]] || {
  echo 'Run scripts/install_calibration_helper.sh first.' >&2; exit 3
}
command -v fuser >/dev/null || { echo 'Install psmisc for the port ownership check.' >&2; exit 3; }
if pgrep -f '[r]os2_control_node' >/dev/null || fuser "$1" >/dev/null 2>&1; then
  echo 'Stop ROS and every other program using the arm before calibration.' >&2
  exit 4
fi
mkdir -p "$state/calibration"
"$venv/bin/python" "$repo/scripts/backup_servo_settings.py" "$1" \
  "$state/follower-settings-$(date -u +%Y%m%dT%H%M%S).json"
if [[ -f "$state/calibration/follower.json" ]]; then
  cp "$state/calibration/follower.json" "$state/calibration/follower-$(date -u +%Y%m%dT%H%M%S).json"
fi
echo 'Support the arm, clear its sweep and keep the motor power cutoff accessible.'
echo 'Follow the helper prompts. It changes stored motor calibration with torque disabled.'
exec "$venv/bin/python" "$repo/scripts/calibrate_follower.py" "$1" "$state/calibration"
