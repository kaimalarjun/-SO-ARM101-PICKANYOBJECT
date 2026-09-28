#!/usr/bin/env bash
set -euo pipefail
dataset=${1:?Usage: train.sh LOCAL_DATASET_DIR OUTPUT_DIR}
output=${2:?Usage: train.sh LOCAL_DATASET_DIR OUTPUT_DIR}
[[ -d $dataset ]] || { echo 'Local dataset missing' >&2; exit 2; }
[[ ! -e $output ]] || { echo 'Output exists; refusing overwrite' >&2; exit 2; }
lerobot-train \
  --dataset.repo_id=local/so101_tabletop \
  --dataset.root="$dataset" \
  --policy.path=lerobot/smolvla_base \
  --policy.push_to_hub=false \
  --output_dir="$output"
