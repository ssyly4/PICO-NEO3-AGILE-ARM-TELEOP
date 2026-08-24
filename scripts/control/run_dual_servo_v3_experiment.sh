#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"
CAN_DIR="$PROJECT_ROOT/scripts/can"
PYTHON="$NERO_TELEOP_PYTHON"

left_can="${PICO_LEFT_CAN_PORT:-can_left}"
right_can="${PICO_RIGHT_CAN_PORT:-can_right}"
left_usb_bus="${PICO_LEFT_CAN_USB_BUS:-1-2.3:1.0}"
right_usb_bus="${PICO_RIGHT_CAN_USB_BUS:-3-1.2:1.0}"
translation_scale="${PICO_TRANSLATION_SCALE:-0.80}"
max_velocity_deg_s="${PICO_MAX_VELOCITY_DEG_S:-32}"
max_acceleration_deg_s2="${PICO_MAX_ACCELERATION_DEG_S2:-220}"
action_socket_dir="${NERO_ACTION_SOCKET_DIR:-}"

if [[ "$left_can" == "$right_can" || "$left_usb_bus" == "$right_usb_bus" ]]; then
  echo "[FAIL] left and right CAN bindings must be different" >&2
  exit 1
fi

prepare_can() {
  local name="$1"
  local usb_bus="$2"
  local attempt
  for attempt in 1 2 3; do
    echo "[CAN] Servo v3 prepare ${name} ${attempt}/3: USB=${usb_bus}"
    if "$CAN_DIR/ensure_can_interface.sh" "$name" "$usb_bus" \
      && "$NERO_TELEOP_PYTHON" -B -c \
        "from nero_vla.dual_can import require_can_role; require_can_role('${name}', 'follower', recovery_timeout_sec=3.0)"; then
      return 0
    fi
    if [[ "$attempt" -lt 3 ]]; then
      "$CAN_DIR/reset_gs_usb_adapter.sh" "$usb_bus" || true
    fi
    sleep 1
  done
  echo "[FAIL] could not prepare ${name} at USB ${usb_bus}" >&2
  return 1
}

execute_requested=false
forward_args=()
while (( $# > 0 )); do
  case "$1" in
    --execute)
      execute_requested=true
      forward_args+=("$1")
      shift
      ;;
    --confirm)
      if (( $# < 2 )); then
        echo "[FAIL] --confirm requires a value" >&2
        exit 2
      fi
      echo "[compat] dual wrapper handles guarded Home confirmation"
      shift 2
      ;;
    *)
      forward_args+=("$1")
      shift
      ;;
  esac
done

prepare_can "$left_can" "$left_usb_bus"
prepare_can "$right_can" "$right_usb_bus"

ready_file="$(mktemp /tmp/nero_pico_udp_ready.XXXXXX)"
rm -f "$ready_file"
"$PYTHON" -m nero_neo_teleop.pico.udp_fanout \
  --port 50150 \
  --left-port 50151 \
  --right-port 50152 \
  --ready-file "$ready_file" &
fanout_pid=$!
left_pid=""
right_pid=""

cleanup() {
  trap - INT TERM EXIT
  [[ -z "$left_pid" ]] || kill -INT "$left_pid" 2>/dev/null || true
  [[ -z "$right_pid" ]] || kill -INT "$right_pid" 2>/dev/null || true
  kill "$fanout_pid" 2>/dev/null || true
  [[ -z "$left_pid" ]] || wait "$left_pid" 2>/dev/null || true
  [[ -z "$right_pid" ]] || wait "$right_pid" 2>/dev/null || true
  wait "$fanout_pid" 2>/dev/null || true
  rm -f "$ready_file"
}

stop_requested() {
  echo "[STOP] dual Servo v3 controlled shutdown requested"
  cleanup
  exit 0
}

trap stop_requested INT TERM
trap cleanup EXIT

echo "[PICO] waiting for left+right controller stream on UDP :50150"
input_wait_sec="${PICO_INPUT_WAIT_SEC:-30}"
input_wait_attempts=$((input_wait_sec * 10))
for ((_attempt = 1; _attempt <= input_wait_attempts; _attempt++)); do
  if [[ -s "$ready_file" ]]; then
    echo "[PICO] input ready: $(cat "$ready_file")"
    break
  fi
  if ! kill -0 "$fanout_pid" 2>/dev/null; then
    echo "[FAIL] UDP fanout exited before receiving PICO input" >&2
    exit 1
  fi
  sleep 0.1
done
if [[ ! -s "$ready_file" ]]; then
  echo "[FAIL] no PICO UDP packet within ${input_wait_sec} seconds" >&2
  exit 1
fi

if [[ "$execute_requested" == true && "${PICO_SKIP_DUAL_HOME:-0}" != "1" ]]; then
  echo "[home] returning both arms to the captured mirrored Home at 5% speed"
  "$PYTHON" -m nero_neo_teleop.robot.dual_home \
    --left-can "$left_can" \
    --right-can "$right_can" \
    --speed-percent 5 \
    --execute \
    --confirm 'MOVE BOTH NERO ARMS TO COMMUNITY HOME'

  echo "[CAN] validating both SDK-to-CPV handoffs after Home"
  prepare_can "$left_can" "$left_usb_bus"
  prepare_can "$right_can" "$right_usb_bus"
fi

common_args=(
  --bind 127.0.0.1
  --duration 120
  --rate-hz 40
  --translation-scale "$translation_scale"
  --max-translation-mm 300
  --rotation-scale 1.25
  --max-rotation-deg 40
  --position-filter-hz 10
  --rotation-filter-hz 15
  --position-gain-s 8
  --rotation-gain-s 8
  --max-linear-speed-mm-s 160
  --max-angular-speed-deg-s 120
  --max-velocity-deg-s "$max_velocity_deg_s"
  --max-acceleration-deg-s2 "$max_acceleration_deg_s2"
  --command-lead-ms 67
  --max-command-lead-deg 1.80
  --max-cpv-step-deg 0.85
  --nullspace-gain-s 0.30
  --orientation-limit-soft-margin-deg 12
  --orientation-limit-hard-margin-deg 3
  --max-executable-position-lead-mm 35
  --max-executable-rotation-lead-deg 12
  --grip-engage-threshold 0.30
  --grip-release-threshold 0.10
  --max-packet-age-ms 120
  --network-prediction-ms 250
  --clutch-reset-gap-ms 500
  --gripper-open-width-mm 90
  --gripper-closed-width-mm 0
  --gripper-force-n 1.0
  --invert-forward
  --invert-lateral
)

left_output="$NERO_TELEOP_ARTIFACTS_DIR/logs/servo_v3_dual/left"
right_output="$NERO_TELEOP_ARTIFACTS_DIR/logs/servo_v3_dual/right"
mkdir -p "$left_output" "$right_output"

left_action_args=()
right_action_args=()
if [[ -n "$action_socket_dir" ]]; then
  left_action_args=(--action-socket "$action_socket_dir/left.sock")
  right_action_args=(--action-socket "$action_socket_dir/right.sock")
  echo "[record] publishing executed action targets to ${action_socket_dir}"
fi

"$PYTHON" -m nero_neo_teleop.control.servo_v3_controller \
  "${common_args[@]}" \
  --port 50151 \
  --hand left \
  --can-port "$left_can" \
  "${left_action_args[@]}" \
  --output-dir "$left_output" \
  "${forward_args[@]}" &
left_pid=$!

"$PYTHON" -m nero_neo_teleop.control.servo_v3_controller \
  "${common_args[@]}" \
  --port 50152 \
  --hand right \
  --can-port "$right_can" \
  "${right_action_args[@]}" \
  --output-dir "$right_output" \
  "${forward_args[@]}" &
right_pid=$!

set +e
finished_pid=""
wait -n -p finished_pid "$left_pid" "$right_pid"
first_status=$?
finished_pid="${finished_pid:-}"
if [[ -z "$finished_pid" ]]; then
  echo "[FAIL] wait was interrupted without identifying a Servo process" >&2
  kill -INT "$left_pid" "$right_pid" 2>/dev/null || true
  wait "$left_pid" 2>/dev/null
  left_status=$?
  wait "$right_pid" 2>/dev/null
  right_status=$?
elif [[ "$finished_pid" == "$left_pid" ]]; then
  left_status=$first_status
  if (( first_status != 0 )); then
    kill -INT "$right_pid" 2>/dev/null || true
  fi
  wait "$right_pid"
  right_status=$?
else
  right_status=$first_status
  if (( first_status != 0 )); then
    kill -INT "$left_pid" 2>/dev/null || true
  fi
  wait "$left_pid"
  left_status=$?
fi
set -e
left_pid=""
right_pid=""

if (( first_status != 0 || left_status != 0 || right_status != 0 )); then
  echo "[FAIL] dual Servo v3 stopped: left=${left_status} right=${right_status}" >&2
  exit 1
fi
echo "[PASS] dual PICO Servo v3 experiment complete"
