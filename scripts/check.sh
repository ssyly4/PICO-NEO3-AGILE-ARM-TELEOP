#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

"$NERO_TELEOP_PYTHON" -m compileall -q "$PROJECT_ROOT/src" "$PROJECT_ROOT/tests"
for file in "$PROJECT_ROOT"/scripts/*.sh "$PROJECT_ROOT"/scripts/*/*.sh; do
  bash -n "$file"
done

"$NERO_TELEOP_PYTHON" -m unittest discover -s "$PROJECT_ROOT/tests" -p 'test_pose_mapper.py' -q
"$NERO_TELEOP_PYTHON" -m unittest discover -s "$PROJECT_ROOT/tests" -p 'test_dual_home_config.py' -q
"$NERO_TELEOP_PYTHON" -m unittest discover -s "$PROJECT_ROOT/tests" -p 'test_servo_v3_core.py' -q
"$NERO_LEROBOT_PYTHON" -m unittest discover \
  -s "$PROJECT_ROOT/tests" -p 'test_bimanual_lerobot_recorder.py' -q

echo "[PASS] static checks and unit tests completed"
