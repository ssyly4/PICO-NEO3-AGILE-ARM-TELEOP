#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"
CAN_DIR="$PROJECT_ROOT/scripts/can"
PYTHON="$NERO_TELEOP_PYTHON"

can_port="${PICO_CAN_PORT:-$PICO_RIGHT_CAN_PORT}"
max_velocity_deg_s="${PICO_MAX_VELOCITY_DEG_S:-35}"
max_acceleration_deg_s2="${PICO_MAX_ACCELERATION_DEG_S2:-300}"
translation_scale="${PICO_TRANSLATION_SCALE:-0.50}"
rotation_scale="${PICO_ROTATION_SCALE:-1.25}"
position_gain_s="${PICO_POSITION_GAIN_S:-10}"
rotation_gain_s="${PICO_ROTATION_GAIN_S:-10}"
max_linear_speed_mm_s="${PICO_MAX_LINEAR_SPEED_MM_S:-200}"
max_angular_speed_deg_s="${PICO_MAX_ANGULAR_SPEED_DEG_S:-150}"
max_command_lead_deg="${PICO_MAX_COMMAND_LEAD_DEG:-2.50}"
max_cpv_step_deg="${PICO_MAX_CPV_STEP_DEG:-0.95}"
max_position_lead_mm="${PICO_MAX_EXECUTABLE_POSITION_LEAD_MM:-50}"
max_rotation_lead_deg="${PICO_MAX_EXECUTABLE_ROTATION_LEAD_DEG:-16}"

# The launcher must use the same interface name as the Python process during
# CAN preparation, Home, and the post-Home handoff.
arguments=("$@")
show_selection=false
for ((index = 0; index < ${#arguments[@]}; index++)); do
  case "${arguments[index]}" in
    --show-selection)
      show_selection=true
      ;;
    --can-port)
      if ((index + 1 >= ${#arguments[@]})); then
        echo "[FAIL] --can-port requires an interface name" >&2
        exit 2
      fi
      can_port="${arguments[index + 1]}"
      ((index += 1))
      ;;
    --can-port=*)
      can_port="${arguments[index]#--can-port=}"
      ;;
  esac
done

if [[ "$can_port" == "$PICO_LEFT_CAN_PORT" ]]; then
  hand=left
  home_side=left
  expected_usb_bus="$PICO_LEFT_CAN_USB_BUS"
elif [[ "$can_port" == "$PICO_RIGHT_CAN_PORT" ]]; then
  hand=right
  home_side=right
  expected_usb_bus="$PICO_RIGHT_CAN_USB_BUS"
else
  echo "[FAIL] ${can_port} is not a configured left or right CAN interface" >&2
  exit 2
fi
can_usb_bus="${PICO_CAN_USB_BUS:-$expected_usb_bus}"
if [[ "$show_selection" == true ]]; then
  printf 'can=%s usb=%s hand=%s home=%s\n' \
    "$can_port" "$can_usb_bus" "$hand" "$home_side"
  exit 0
fi

follower_traffic_ready() {
  "$NERO_TELEOP_PYTHON" -B -c \
    "from nero_vla.dual_can import require_can_role; require_can_role('${can_port}', 'follower', recovery_timeout_sec=3.0)"
}

can_ready=false
for attempt in 1 2 3; do
  echo "[CAN] Servo v3 prepare ${attempt}/3: name=${can_port} USB=${can_usb_bus}"
  if "$CAN_DIR/ensure_can_interface.sh" "$can_port" "$can_usb_bus" && follower_traffic_ready; then
    can_ready=true
    break
  fi
  if [[ "$attempt" -lt 3 ]]; then
    "$CAN_DIR/reset_gs_usb_adapter.sh" "$can_usb_bus" || true
  fi
  sleep 1
done
if [[ "$can_ready" != true ]]; then
  echo "[FAIL] could not prepare ${can_port} at USB ${can_usb_bus}" >&2
  exit 1
fi

execute_requested=false
for argument in "$@"; do
  if [[ "$argument" == "--execute" ]]; then
    execute_requested=true
    break
  fi
done

if [[ "$execute_requested" == true && "${PICO_SKIP_HOME:-0}" != "1" ]]; then
  echo "[home] returning ${can_port} to the ${home_side}-arm PICO Home"
  "$PYTHON" -m nero_neo_teleop.robot.single_home \
    --can-port "$can_port" \
    --home-side "$home_side" \
    --speed-percent 5 \
    --execute \
    --confirm 'MOVE NERO ARM TO PICO HOME'

  handoff_ready=false
  for attempt in 1 2 3; do
    echo "[CAN] Servo v3 post-Home handoff ${attempt}/3"
    if "$CAN_DIR/ensure_can_interface.sh" "$can_port" "$can_usb_bus" && follower_traffic_ready; then
      handoff_ready=true
      break
    fi
    if [[ "$attempt" -lt 3 ]]; then
      "$CAN_DIR/reset_gs_usb_adapter.sh" "$can_usb_bus" || true
    fi
    sleep 1
  done
  if [[ "$handoff_ready" != true ]]; then
    echo "[FAIL] follower feedback did not recover after Home" >&2
    exit 1
  fi
fi

exec "$PYTHON" -m nero_neo_teleop.control.servo_v3_controller \
  --can-port "$can_port" \
  --hand "$hand" \
  --duration 120 \
  --rate-hz 40 \
  --translation-scale "$translation_scale" \
  --max-translation-mm 300 \
  --rotation-scale "$rotation_scale" \
  --max-rotation-deg 40 \
  --position-filter-hz 10 \
  --rotation-filter-hz 15 \
  --position-gain-s "$position_gain_s" \
  --rotation-gain-s "$rotation_gain_s" \
  --max-linear-speed-mm-s "$max_linear_speed_mm_s" \
  --max-angular-speed-deg-s "$max_angular_speed_deg_s" \
  --max-velocity-deg-s "$max_velocity_deg_s" \
  --max-acceleration-deg-s2 "$max_acceleration_deg_s2" \
  --command-lead-ms 67 \
  --max-command-lead-deg "$max_command_lead_deg" \
  --max-cpv-step-deg "$max_cpv_step_deg" \
  --nullspace-gain-s 0.30 \
  --orientation-limit-soft-margin-deg 12 \
  --orientation-limit-hard-margin-deg 3 \
  --max-executable-position-lead-mm "$max_position_lead_mm" \
  --max-executable-rotation-lead-deg "$max_rotation_lead_deg" \
  --grip-engage-threshold 0.30 \
  --grip-release-threshold 0.10 \
  --max-packet-age-ms 120 \
  --network-prediction-ms 120 \
  --clutch-reset-gap-ms 500 \
  --gripper-open-width-mm 90 \
  --gripper-closed-width-mm 0 \
  --gripper-force-n 1.0 \
  --invert-forward \
  --invert-lateral \
  "$@"
