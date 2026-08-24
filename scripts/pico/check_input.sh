#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"
exec "$NERO_TELEOP_PYTHON" -m nero_neo_teleop.diagnostics.pico_input_probe "$@"
