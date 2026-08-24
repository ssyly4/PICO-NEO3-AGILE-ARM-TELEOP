#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"
CAN_DIR="$PROJECT_ROOT/scripts/can"
CONTROL_DIR="$PROJECT_ROOT/scripts/control"
PYTHON="$NERO_LEROBOT_PYTHON"
WORKFLOW="${NERO_BIMANUAL_WORKFLOW:-custom}"
passthrough=()
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --workflow)
      [[ "$#" -ge 2 ]] || { echo "[FAIL] --workflow requires a value" >&2; exit 2; }
      WORKFLOW="$2"
      shift 2
      ;;
    --workflow=*)
      WORKFLOW="${1#--workflow=}"
      shift
      ;;
    *)
      passthrough+=("$1")
      shift
      ;;
  esac
done

case "$WORKFLOW" in
  custom|fullflow|stage1|stage23) ;;
  *)
    echo "[FAIL] unknown workflow '$WORKFLOW'; use custom, fullflow, stage1, or stage23" >&2
    exit 2
    ;;
esac

LEFT_CAN="${PICO_LEFT_CAN_PORT:-can_left}"
RIGHT_CAN="${PICO_RIGHT_CAN_PORT:-can_right}"
LEFT_USB="${PICO_LEFT_CAN_USB_BUS:-1-2.3:1.0}"
RIGHT_USB="${PICO_RIGHT_CAN_USB_BUS:-3-1.2:1.0}"
WORLD_CAMERA="${NERO_WORLD_CAMERA:-}"
LEFT_WRIST_CAMERA="${NERO_LEFT_WRIST_CAMERA:-}"
RIGHT_WRIST_CAMERA="${NERO_RIGHT_WRIST_CAMERA:-}"
DATA_ROOT="$NERO_BIMANUAL_DATA_DIR"
for camera_var in WORLD_CAMERA LEFT_WRIST_CAMERA RIGHT_WRIST_CAMERA; do
  if [[ -z "${!camera_var}" ]]; then
    echo "[FAIL] ${camera_var} is unset; configure the corresponding NERO_* variable in .env" >&2
    exit 2
  fi
done
case "$WORKFLOW" in
  fullflow)
    DATASET_BASE="${NERO_BIMANUAL_DATASET_BASE:-nero_towel_fullflow_70_command_v1}"
    EPISODES="${NERO_BIMANUAL_EPISODES:-70}"
    TASK="${NERO_BIMANUAL_TASK:-fold the towel}"
    RELEASE_AUTO_STOP_MODE="${NERO_BIMANUAL_RELEASE_AUTO_STOP_MODE:-idle}"
    RELEASE_STATIONARY_SECONDS="${NERO_BIMANUAL_RELEASE_STATIONARY_SECONDS:-0.8}"
    RETURN_DELAY_SECONDS="${NERO_BIMANUAL_RETURN_DELAY_SECONDS:-1}"
    ACTION_SOURCE="${NERO_BIMANUAL_ACTION_SOURCE:-controller_command}"
    controller_default="cd '$CONTROL_DIR' && PICO_SKIP_DUAL_HOME=1 ./run_dual_servo_v3_experiment.sh --duration 3600 --execute"
    ;;
  stage1)
    DATASET_BASE="${NERO_BIMANUAL_DATASET_BASE:-nero_towel_stage1_reposition_50_v1}"
    EPISODES="${NERO_BIMANUAL_EPISODES:-50}"
    TASK="${NERO_BIMANUAL_TASK:-grasp the middle of the towel and place it at the staging position}"
    RELEASE_AUTO_STOP_MODE="${NERO_BIMANUAL_RELEASE_AUTO_STOP_MODE:-right}"
    RETURN_DELAY_SECONDS="${NERO_BIMANUAL_RETURN_DELAY_SECONDS:-0}"
    ACTION_SOURCE="${NERO_BIMANUAL_ACTION_SOURCE:-next_feedback}"
    export PICO_TRANSLATION_SCALE="${PICO_TRANSLATION_SCALE:-0.70}"
    export PICO_ROTATION_SCALE="${PICO_ROTATION_SCALE:-1.20}"
    export PICO_POSITION_GAIN_S="${PICO_POSITION_GAIN_S:-7}"
    export PICO_ROTATION_GAIN_S="${PICO_ROTATION_GAIN_S:-7}"
    export PICO_MAX_LINEAR_SPEED_MM_S="${PICO_MAX_LINEAR_SPEED_MM_S:-200}"
    export PICO_MAX_ANGULAR_SPEED_DEG_S="${PICO_MAX_ANGULAR_SPEED_DEG_S:-150}"
    export PICO_MAX_VELOCITY_DEG_S="${PICO_MAX_VELOCITY_DEG_S:-35}"
    export PICO_MAX_ACCELERATION_DEG_S2="${PICO_MAX_ACCELERATION_DEG_S2:-300}"
    controller_default="cd '$CONTROL_DIR' && PICO_CAN_USB_BUS='$RIGHT_USB' PICO_SKIP_HOME=1 ./run_servo_v3_experiment.sh --can-port '$RIGHT_CAN' --duration 3600 --execute"
    ;;
  stage23)
    DATASET_BASE="${NERO_BIMANUAL_DATASET_BASE:-nero_towel_stage23_fold_50_command_v1}"
    EPISODES="${NERO_BIMANUAL_EPISODES:-50}"
    TASK="${NERO_BIMANUAL_TASK:-grasp both sides of the towel from the staging position, fold it, and release it}"
    RELEASE_AUTO_STOP_MODE="${NERO_BIMANUAL_RELEASE_AUTO_STOP_MODE:-dual}"
    RETURN_DELAY_SECONDS="${NERO_BIMANUAL_RETURN_DELAY_SECONDS:-0}"
    ACTION_SOURCE="${NERO_BIMANUAL_ACTION_SOURCE:-controller_command}"
    controller_default="cd '$CONTROL_DIR' && PICO_SKIP_DUAL_HOME=1 ./run_dual_servo_v3_experiment.sh --duration 3600 --execute"
    echo "[stage23] Prepare the towel in a valid stage-1 staging layout before each Enter."
    ;;
  custom)
    DATASET_BASE="${NERO_BIMANUAL_DATASET_BASE:-nero_towel_bimanual}"
    EPISODES="${NERO_BIMANUAL_EPISODES:-3}"
    TASK="${NERO_BIMANUAL_TASK:-fold the towel}"
    RELEASE_AUTO_STOP_MODE="${NERO_BIMANUAL_RELEASE_AUTO_STOP_MODE:-dual}"
    RETURN_DELAY_SECONDS="${NERO_BIMANUAL_RETURN_DELAY_SECONDS:-1}"
    ACTION_SOURCE="${NERO_BIMANUAL_ACTION_SOURCE:-next_feedback}"
    controller_default="cd '$CONTROL_DIR' && PICO_SKIP_DUAL_HOME=1 ./run_dual_servo_v3_experiment.sh --duration 3600 --execute"
    ;;
esac
RELEASE_STATIONARY_SECONDS="${RELEASE_STATIONARY_SECONDS:-${NERO_BIMANUAL_RELEASE_STATIONARY_SECONDS:-0.5}}"
EPISODE_SECONDS="${NERO_BIMANUAL_EPISODE_SECONDS:-0}"
MANAGED="${NERO_BIMANUAL_MANAGED:-1}"
RETURN_SPEED_PERCENT="${NERO_BIMANUAL_RETURN_SPEED_PERCENT:-10}"
if [[ ! "$RETURN_SPEED_PERCENT" =~ ^[0-9]+$ ]] \
  || (( RETURN_SPEED_PERCENT < 1 || RETURN_SPEED_PERCENT > 10 )); then
  echo "[FAIL] NERO_BIMANUAL_RETURN_SPEED_PERCENT must be an integer in [1, 10]" >&2
  exit 2
fi

prepare_can() {
  local name="$1"
  local usb="$2"
  local attempt
  for attempt in 1 2 3; do
    echo "[CAN] recorder preflight ${name} ${attempt}/3: USB=${usb}"
    if "$CAN_DIR/ensure_can_interface.sh" "$name" "$usb" \
      && "$NERO_TELEOP_PYTHON" -B -c \
        "from nero_vla.dual_can import require_can_role; require_can_role('${name}', 'follower', recovery_timeout_sec=3.0)"; then
      return 0
    fi
    if [[ "$attempt" -lt 3 ]]; then
      "$CAN_DIR/reset_gs_usb_adapter.sh" "$usb" || true
      sleep 1
    fi
  done
  echo "[FAIL] no healthy follower feedback from ${name} at ${usb}" >&2
  return 1
}

parquet_ok() {
  "$PYTHON" - "$1" <<'PY'
from pathlib import Path
import sys
import pyarrow.parquet as pq

root = Path(sys.argv[1])
paths = [root / "meta/tasks.parquet"]
paths += sorted((root / "meta/episodes").rglob("*.parquet"))
paths += sorted((root / "data").rglob("*.parquet"))
if not paths[0].is_file() or not any((root / "meta/episodes").rglob("*.parquet")):
    raise SystemExit(1)
for path in paths:
    pq.ParquetFile(path)
print(f"[record] parquet health check passed: {len(paths)} files")
PY
}

mkdir -p "$DATA_ROOT"
resume_args=()
dataset_name=""
dataset_root=""
for version in $(seq 1 99); do
  if [[ "$version" == 1 ]]; then
    candidate="$DATASET_BASE"
  else
    candidate="${DATASET_BASE}_v${version}"
  fi
  candidate_root="${DATA_ROOT}/${candidate}"
  if [[ ! -e "$candidate_root" ]]; then
    dataset_name="$candidate"
    dataset_root="$candidate_root"
    break
  fi
  if parquet_ok "$candidate_root"; then
    dataset_name="$candidate"
    dataset_root="$candidate_root"
    resume_args=(--resume)
    echo "[record] resuming complete dataset: ${dataset_root}"
    break
  fi
  echo "[record] ignoring incomplete dataset root: ${candidate_root}" >&2
done

if [[ -z "$dataset_root" ]]; then
  echo "[FAIL] could not allocate or resume a dataset root" >&2
  exit 1
fi

prepare_can "$LEFT_CAN" "$LEFT_USB"
prepare_can "$RIGHT_CAN" "$RIGHT_USB"

echo "[record] dataset=${dataset_root}"
echo "[record] successful target=${EPISODES}; discarded or failed attempts do not count"

action_args=(--action-source "$ACTION_SOURCE")
if [[ "$ACTION_SOURCE" == "controller_command" ]]; then
  action_socket_dir="${NERO_ACTION_SOCKET_DIR:-/tmp/nero_bimanual_actions_$$}"
  export NERO_ACTION_SOCKET_DIR="$action_socket_dir"
  action_args+=(--action-socket-dir "$action_socket_dir")
  echo "[record] action labels=timestamped executed controller targets"
else
  echo "[record] action labels=next feedback sample (legacy mode)"
fi

managed_args=()
if [[ "$MANAGED" == 1 ]]; then
  echo "[record] managed mode: controller stop -> dual Home -> save/discard -> next attempt"
  "$CONTROL_DIR/run_dual_home.sh" \
    --execute \
    --confirm 'MOVE BOTH NERO ARMS TO COMMUNITY HOME'
  controller_command="${NERO_BIMANUAL_CONTROLLER_COMMAND:-$controller_default}"
  home_command="${NERO_BIMANUAL_HOME_COMMAND:-cd '$CONTROL_DIR' && ./run_dual_home.sh --speed-percent '$RETURN_SPEED_PERCENT' --execute --confirm 'MOVE BOTH NERO ARMS TO COMMUNITY HOME'}"
  echo "[record] automatic return speed=${RETURN_SPEED_PERCENT}%"
  managed_args=(
    --managed-controller-command "$controller_command"
    --managed-controller-startup-sec 2
    --return-home-command "$home_command"
    --return-delay-seconds "$RETURN_DELAY_SECONDS"
    --return-timeout-sec 75
  )
else
  echo "[record] passive sidecar mode: start dual Servo v3 in another terminal"
fi

exec "$PYTHON" -m nero_neo_teleop.recording.bimanual_lerobot_recorder \
  --left-can "$LEFT_CAN" \
  --right-can "$RIGHT_CAN" \
  --world-camera "$WORLD_CAMERA" \
  --left-wrist-camera "$LEFT_WRIST_CAMERA" \
  --right-wrist-camera "$RIGHT_WRIST_CAMERA" \
  --successful-episodes "$EPISODES" \
  --episode-seconds "$EPISODE_SECONDS" \
  --fps 30 \
  --width 1280 \
  --height 720 \
  --task "$TASK" \
  "${action_args[@]}" \
  --release-auto-stop-mode "$RELEASE_AUTO_STOP_MODE" \
  --release-stationary-seconds "$RELEASE_STATIONARY_SECONDS" \
  --repo-id "local/${dataset_name}" \
  --root "$dataset_root" \
  "${managed_args[@]}" \
  "${resume_args[@]}" \
  "${passthrough[@]}"
