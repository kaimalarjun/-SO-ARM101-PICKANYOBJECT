#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
venv=${VISION_VENV:-"$HOME/.local/share/kendra-robot-vision/venv"}
python3 -m venv "$venv"
"$venv/bin/python" -m pip install --upgrade pip
"$venv/bin/python" -m pip install -r "$repo/vision/requirements.lock"
echo 'Vision dependencies installed. Models download on first inference; service is not started.'
