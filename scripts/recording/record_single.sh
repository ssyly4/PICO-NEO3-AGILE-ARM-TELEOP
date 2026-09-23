#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"

usage() {
  cat <<'EOF'
用法：
  record_single.sh --task TEXT --dataset NAME [options]

必填参数：
  --task TEXT                  每帧保存的自然语言任务标签。
  --dataset NAME               数据集目录或仓库名称。

可选参数：
  --episodes N                 成功保存的 episode 数量，默认 10。
  --data-root PATH             数据集父目录，默认 ~/nero_data/raw。
  --episode-seconds SEC        单集最长时间，默认 30 秒。
  --action-source SOURCE       controller_command 或 next_feedback。
  --current-home               将启动时姿态作为本次数据集的回位姿态。
  --execute                    启动相机、CAN 和机器人进程。
  --help                       显示本帮助。
EOF
}

TASK="${NERO_RECORD_TASK:-}"
DATASET_BASE="${NERO_RECORD_DATASET:-}"
EPISODES="${NERO_RECORD_EPISODES:-10}"
DATA_ROOT="${NERO_RECORD_DATA_DIR:-$HOME/nero_data/raw}"
EPISODE_SECONDS="${NERO_RECORD_EPISODE_SECONDS:-30}"
ACTION_SOURCE="${NERO_RECORD_ACTION_SOURCE:-controller_command}"
CURRENT_HOME=0
execute=0

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --task|--dataset|--episodes|--data-root|--episode-seconds|--action-source)
      [[ "$#" -ge 2 ]] || { echo "[FAIL] $1 requires a value" >&2; exit 2; }
      option="$1"
      value="$2"
      case "$option" in
        --task) TASK="$value" ;;
        --dataset) DATASET_BASE="$value" ;;
        --episodes) EPISODES="$value" ;;
        --data-root) DATA_ROOT="$value" ;;
        --episode-seconds) EPISODE_SECONDS="$value" ;;
        --action-source) ACTION_SOURCE="$value" ;;
      esac
      shift 2
      ;;
    --current-home) CURRENT_HOME=1; shift ;;
    --execute) execute=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[FAIL] unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$TASK" ]] || { echo "[FAIL] --task is required" >&2; exit 2; }
[[ -n "$DATASET_BASE" ]] || { echo "[FAIL] --dataset is required" >&2; exit 2; }
[[ "$EPISODES" =~ ^[1-9][0-9]*$ ]] || { echo "[FAIL] --episodes must be positive" >&2; exit 2; }
case "$ACTION_SOURCE" in controller_command|next_feedback) ;; *)
  echo "[FAIL] --action-source must be controller_command or next_feedback" >&2; exit 2;; esac

CAN_PORT="${PICO_RIGHT_CAN_PORT:-can_right}"
CAN_USB="${PICO_RIGHT_CAN_USB_BUS:-3-1.2:1.0}"
WORLD_CAMERA="${NERO_WORLD_CAMERA:-}"
WRIST_CAMERA="${NERO_RIGHT_WRIST_CAMERA:-}"

echo "[record single] arm=right task=$TASK"
echo "[record single] dataset=$DATA_ROOT/$DATASET_BASE episodes=$EPISODES"
echo "[record single] action_source=$ACTION_SOURCE current_home=$CURRENT_HOME"
if [[ "$execute" != 1 ]]; then
  echo "[PREVIEW ONLY] add --execute to start camera, CAN and robot processes"
  exit 0
fi

[[ -n "$WORLD_CAMERA" ]] || { echo "[FAIL] NERO_WORLD_CAMERA is unset in .env" >&2; exit 2; }
[[ -n "$WRIST_CAMERA" ]] || { echo "[FAIL] NERO_RIGHT_WRIST_CAMERA is unset in .env" >&2; exit 2; }

"$PROJECT_ROOT/scripts/can/ensure_can_interface.sh" "$CAN_PORT" "$CAN_USB"
"$NERO_TELEOP_PYTHON" -B -c \
  "from nero_vla.dual_can import require_can_role; require_can_role('${CAN_PORT}', 'follower', recovery_timeout_sec=3.0)"

resume_args=()
dataset_name=""
dataset_root=""
for version in $(seq 1 99); do
  [[ "$version" == 1 ]] && candidate="$DATASET_BASE" || candidate="${DATASET_BASE}_v${version}"
  candidate_root="$DATA_ROOT/$candidate"
  if [[ ! -e "$candidate_root" ]]; then
    dataset_name="$candidate"
    dataset_root="$candidate_root"
    break
  fi
  if "$NERO_LEROBOT_PYTHON" - "$candidate_root" >/dev/null 2>&1 <<'PY'
from pathlib import Path
import sys
from nero_neo_teleop.recording.bimanual_lerobot_recorder import validate_existing_dataset
validate_existing_dataset(Path(sys.argv[1]))
PY
  then
    dataset_name="$candidate"
    dataset_root="$candidate_root"
    resume_args=(--resume)
    echo "[record single] resuming complete dataset: $dataset_root"
    break
  fi
done
[[ -n "$dataset_root" ]] || { echo "[FAIL] could not allocate dataset root" >&2; exit 1; }

start_args=()
if [[ "$CURRENT_HOME" == 1 ]]; then
  config_path="$dataset_root/recording_config.json"
  if [[ -f "$config_path" ]]; then
    home_deg="$($NERO_TELEOP_PYTHON -c "import json; print(','.join(str(x) for x in json.load(open('$config_path'))['start_joints_deg']))")"
    echo "[record single] reusing captured Home from $config_path"
  else
    home_deg="$($NERO_TELEOP_PYTHON -m nero_neo_teleop.recording.capture_single_arm_start --can "$CAN_PORT")"
    echo "[record single] captured current Home: $home_deg deg"
  fi
  start_args=("--start-joints-deg=$home_deg")
  home_command="cd '$PROJECT_ROOT' && NERO_RIGHT_HOME_DEG='$home_deg' '$NERO_TELEOP_PYTHON' -m nero_neo_teleop.robot.single_home --can-port '$CAN_PORT' --home-side right --speed-percent 5 --execute --confirm 'MOVE NERO ARM TO PICO HOME'"
else
  home_command="cd '$PROJECT_ROOT' && '$NERO_TELEOP_PYTHON' -m nero_neo_teleop.robot.single_home --can-port '$CAN_PORT' --home-side right --speed-percent 5 --execute --confirm 'MOVE NERO ARM TO PICO HOME'"
fi

action_socket="/tmp/nero_single_actions_$$.sock"
action_args=(--action-source "$ACTION_SOURCE")
if [[ "$ACTION_SOURCE" == controller_command ]]; then
  action_args+=(--action-socket "$action_socket")
else
  action_args+=(--activity-socket "$action_socket")
fi

controller_command="cd '$PROJECT_ROOT/scripts/control' && PICO_SKIP_HOME=1 PICO_TRANSLATION_SCALE=0.80 PICO_ROTATION_SCALE=1.25 PICO_POSITION_GAIN_S=8 PICO_ROTATION_GAIN_S=8 PICO_MAX_LINEAR_SPEED_MM_S=160 PICO_MAX_ANGULAR_SPEED_DEG_S=120 PICO_MAX_VELOCITY_DEG_S=32 PICO_MAX_ACCELERATION_DEG_S2=220 PICO_MAX_COMMAND_LEAD_DEG=1.80 PICO_MAX_CPV_STEP_DEG=0.85 PICO_MAX_EXECUTABLE_POSITION_LEAD_MM=35 PICO_MAX_EXECUTABLE_ROTATION_LEAD_DEG=12 ./run_servo_v3_experiment.sh --can-port '$CAN_PORT' --duration 3600 --rate-hz 40 --max-rotation-deg 40 --position-filter-hz 10 --rotation-filter-hz 15 --translation-feedforward-gain 0.0 --nullspace-gain-s 0.30 --network-prediction-ms 250 --grip-engage-threshold 0.30 --grip-release-threshold 0.10 --action-socket '$action_socket' --execute"

echo "[record single] moving right arm Home before episode 1"
/bin/bash -c "$home_command"

exec "$NERO_LEROBOT_PYTHON" -m nero_neo_teleop.recording.single_arm_lerobot_recorder \
  --can "$CAN_PORT" \
  --world-camera "$WORLD_CAMERA" \
  --right-wrist-camera "$WRIST_CAMERA" \
  --successful-episodes "$EPISODES" \
  --episode-seconds "$EPISODE_SECONDS" \
  --task "$TASK" \
  --control-profile nero_single_servo_v3_v1 \
  --repo-id "local/$dataset_name" \
  --root "$dataset_root" \
  "${action_args[@]}" \
  --controller-start-detection \
  --managed-controller-command "$controller_command" \
  --return-home-command "$home_command" \
  "${start_args[@]}" \
  "${resume_args[@]}"
