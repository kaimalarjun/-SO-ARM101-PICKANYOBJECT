#!/usr/bin/env bash
set -euo pipefail

# Optional one-time calibration environment; ROS remains the runtime controller.
venv=${CALIBRATION_VENV:-"$HOME/.venvs/so101-calibration"}
python3 -m venv "$venv" || {
  echo 'Install python3.12-venv on Ubuntu 24.04, then rerun this script.' >&2
  exit 2
}
"$venv/bin/python" -m pip install --upgrade pip
"$venv/bin/python" -m pip install 'torch==2.11.0+cpu' 'torchvision==0.26.0+cpu' \
  --index-url https://download.pytorch.org/whl/cpu
"$venv/bin/python" -m pip install 'lerobot[feetech,hardware]==0.6.1'
"$venv/bin/python" -m pip check
"$venv/bin/lerobot-calibrate" --help >/dev/null
echo "Calibration helper ready: $venv/bin/lerobot-calibrate"
