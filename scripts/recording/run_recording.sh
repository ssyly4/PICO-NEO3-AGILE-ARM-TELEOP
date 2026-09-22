#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"
CAN_DIR="$PROJECT_ROOT/scripts/can"
CONTROL_DIR="$PROJECT_ROOT/scripts/control"
PYTHON="$NERO_LEROBOT_PYTHON"

usage() {
  cat <<'EOF'
用法：
  run_recording.sh --task TEXT --dataset NAME [options]

必填参数：
  --task TEXT                  每帧保存的自然语言任务标签。
  --dataset NAME               数据集目录或仓库名称。

可选参数：
  --episodes N                 成功保存的 episode 数量，默认 10。
  --data-root PATH             数据集父目录。
  --episode-seconds SEC        单集最长时间；0 表示不限制。
  --auto-stop MODE             dual、left、right、idle 或 off。
  --action-source SOURCE       controller_command 或 next_feedback。
  --passive                    不托管遥操和 Home 子进程。
  --execute                    启动相机、CAN 和机器人进程。
  --help                       显示本帮助。
  -- ARGS...                   传递给录制器的额外参数。

机器人、相机和控制器配置从 .env 读取。上述选项也可通过 .env.example 中列出的
NERO_RECORD_* 环境变量提供。
EOF
}

TASK="${NERO_RECORD_TASK:-}"
DATASET_BASE="${NERO_RECORD_DATASET:-}"
EPISODES="${NERO_RECORD_EPISODES:-10}"
DATA_ROOT="${NERO_RECORD_DATA_DIR:-$HOME/nero_data/raw}"
EPISODE_SECONDS="${NERO_RECORD_EPISODE_SECONDS:-0}"
RELEASE_AUTO_STOP_MODE="${NERO_RECORD_AUTO_STOP:-off}"
RELEASE_STATIONARY_SECONDS="${NERO_RECORD_STATIONARY_SECONDS:-0.5}"
ACTION_SOURCE="${NERO_RECORD_ACTION_SOURCE:-controller_command}"
MANAGED="${NERO_RECORD_MANAGED:-1}"
RETURN_DELAY_SECONDS="${NERO_RECORD_RETURN_DELAY_SECONDS:-1}"
RETURN_SPEED_PERCENT="${NERO_RECORD_RETURN_SPEED_PERCENT:-10}"
execute=0
passthrough=()
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --task|--dataset|--episodes|--data-root|--episode-seconds|--auto-stop|--action-source)
      [[ "$#" -ge 2 ]] || { echo "[FAIL] $1 requires a value" >&2; exit 2; }
      option="$1"
      value="$2"
      case "$option" in
        --task) TASK="$value" ;;
        --dataset) DATASET_BASE="$value" ;;
        --episodes) EPISODES="$value" ;;
        --data-root) DATA_ROOT="$value" ;;
        --episode-seconds) EPISODE_SECONDS="$value" ;;
        --auto-stop) RELEASE_AUTO_STOP_MODE="$value" ;;
        --action-source) ACTION_SOURCE="$value" ;;
      esac
      shift 2
      ;;
    --passive)
      MANAGED=0
      shift
      ;;
    --execute)
      execute=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      passthrough+=("$@")
      break
      ;;
    *)
      echo "[FAIL] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$TASK" ]] || { echo "[FAIL] --task is required" >&2; exit 2; }
[[ -n "$DATASET_BASE" ]] || { echo "[FAIL] --dataset is required" >&2; exit 2; }
[[ "$EPISODES" =~ ^[1-9][0-9]*$ ]] || { echo "[FAIL] --episodes must be positive" >&2; exit 2; }
case "$RELEASE_AUTO_STOP_MODE" in dual|left|right|idle|off) ;; *)
  echo "[FAIL] --auto-stop must be dual, left, right, idle, or off" >&2; exit 2;; esac
case "$ACTION_SOURCE" in controller_command|next_feedback) ;; *)
  echo "[FAIL] --action-source must be controller_command or next_feedback" >&2; exit 2;; esac
if [[ ! "$RETURN_SPEED_PERCENT" =~ ^[0-9]+$ ]] \
  || (( RETURN_SPEED_PERCENT < 1 || RETURN_SPEED_PERCENT > 10 )); then
  echo "[FAIL] NERO_RECORD_RETURN_SPEED_PERCENT must be an integer in [1, 10]" >&2
  exit 2
fi

LEFT_CAN="${PICO_LEFT_CAN_PORT:-can_left}"
RIGHT_CAN="${PICO_RIGHT_CAN_PORT:-can_right}"
LEFT_USB="${PICO_LEFT_CAN_USB_BUS:-1-2.2:1.0}"
RIGHT_USB="${PICO_RIGHT_CAN_USB_BUS:-3-1.2:1.0}"
WORLD_CAMERA="${NERO_WORLD_CAMERA:-}"
LEFT_WRIST_CAMERA="${NERO_LEFT_WRIST_CAMERA:-}"
RIGHT_WRIST_CAMERA="${NERO_RIGHT_WRIST_CAMERA:-}"
controller_default="cd '$CONTROL_DIR' && PICO_SKIP_DUAL_HOME=1 ./run_dual_servo_v3_experiment.sh --duration 3600 --execute"

echo "[record] task=$TASK"
echo "[record] dataset=$DATA_ROOT/$DATASET_BASE episodes=$EPISODES"
echo "[record] action_source=$ACTION_SOURCE auto_stop=$RELEASE_AUTO_STOP_MODE"
if [[ "$execute" != 1 ]]; then
  echo "[PREVIEW ONLY] add --execute to start cameras, CAN and robot processes"
  exit 0
fi

for camera_var in WORLD_CAMERA LEFT_WRIST_CAMERA RIGHT_WRIST_CAMERA; do
  if [[ -z "${!camera_var}" ]]; then
    echo "[FAIL] ${camera_var} is unset; configure the corresponding NERO_* variable in .env" >&2
    exit 2
  fi
done

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
  controller_command="${NERO_RECORD_CONTROLLER_COMMAND:-$controller_default}"
  home_command="${NERO_RECORD_HOME_COMMAND:-cd '$CONTROL_DIR' && ./run_dual_home.sh --speed-percent '$RETURN_SPEED_PERCENT' --execute --confirm 'MOVE BOTH NERO ARMS TO COMMUNITY HOME'}"
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
