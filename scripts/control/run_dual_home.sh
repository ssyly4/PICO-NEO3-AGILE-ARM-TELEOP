#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"
CAN_DIR="$PROJECT_ROOT/scripts/can"
"$CAN_DIR/ensure_can_interface.sh" "$PICO_LEFT_CAN_PORT" "$PICO_LEFT_CAN_USB_BUS"
"$CAN_DIR/ensure_can_interface.sh" "$PICO_RIGHT_CAN_PORT" "$PICO_RIGHT_CAN_USB_BUS"

args=("$@")
for arg in "$@"; do
  if [[ "$arg" == "--execute" ]]; then
    args+=(--confirm "MOVE BOTH NERO ARMS TO COMMUNITY HOME")
    break
  fi
done

exec "$NERO_TELEOP_PYTHON" -m nero_neo_teleop.robot.dual_home \
  --left-can "$PICO_LEFT_CAN_PORT" \
  --right-can "$PICO_RIGHT_CAN_PORT" \
  --speed-percent 5 \
  "${args[@]}"
