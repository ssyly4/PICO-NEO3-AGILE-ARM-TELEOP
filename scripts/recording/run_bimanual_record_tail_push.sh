#!/usr/bin/env bash
set -euo pipefail

# Tail-only recollection for the final right-arm towel-flattening motion.
# These are the operator-selected tail start poses, captured from live CAN
# feedback on 2026-08-27. The cropped full-flow data ends at its release
# boundary; the new tail demonstration begins from this deliberately selected
# physical configuration. Move the left arm away from the towel, then use the
# right arm to flatten it forward. Both grippers are opened before each episode.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

export NERO_LEFT_HOME_DEG="${NERO_LEFT_HOME_DEG:--45.166000,39.730000,19.440000,87.785000,-9.381000,-8.141000,61.037000}"
export NERO_RIGHT_HOME_DEG="${NERO_RIGHT_HOME_DEG:-32.760000,53.901000,6.157000,77.782000,-9.693000,-1.903000,59.070000}"

export NERO_BIMANUAL_DATASET_BASE="${NERO_BIMANUAL_DATASET_BASE:-nero_towel_tail_push_30_command_v1}"
export NERO_BIMANUAL_EPISODES="${NERO_BIMANUAL_EPISODES:-30}"
export NERO_BIMANUAL_TASK="${NERO_BIMANUAL_TASK:-fold the towel}"
export NERO_BIMANUAL_ACTION_SOURCE="${NERO_BIMANUAL_ACTION_SOURCE:-controller_command}"
export NERO_BIMANUAL_RELEASE_AUTO_STOP_MODE="${NERO_BIMANUAL_RELEASE_AUTO_STOP_MODE:-idle}"
export NERO_BIMANUAL_RELEASE_STATIONARY_SECONDS="${NERO_BIMANUAL_RELEASE_STATIONARY_SECONDS:-0.8}"
export NERO_BIMANUAL_RETURN_DELAY_SECONDS="${NERO_BIMANUAL_RETURN_DELAY_SECONDS:-0}"
export NERO_BIMANUAL_RETURN_SPEED_PERCENT="${NERO_BIMANUAL_RETURN_SPEED_PERCENT:-10}"

exec "$SCRIPT_DIR/run_bimanual_record.sh" --workflow custom "$@"
