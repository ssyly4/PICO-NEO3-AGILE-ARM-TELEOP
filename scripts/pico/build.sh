#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"
PICO_PACKAGE="$PROJECT_ROOT/pico_client/LocalPackages/com.unity.xr.openxr.picoxr"
if [[ ! -f "$PICO_PACKAGE/package.json" ]]; then
  echo "[FAIL] PICO Unity OpenXR SDK is not installed at:" >&2
  echo "       $PICO_PACKAGE" >&2
  echo "Download it from https://developer.picoxr.com/document/unity-openxr/" >&2
  echo "and place the package directory at the path above." >&2
  exit 1
fi
UNITY="${UNITY_EDITOR:-}"
if [[ -z "$UNITY" ]]; then
  UNITY="$(find "$HOME/Unity/Hub/Editor" -path '*/Editor/Unity' -type f 2>/dev/null | sort -V | tail -1)"
fi
if [[ ! -x "$UNITY" ]]; then
  echo "[FAIL] Unity Editor not found; set UNITY_EDITOR in .env" >&2
  exit 1
fi
DEFAULT_HOST_IP="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
HOST_IP="${NERO_PICO_HOST:-$DEFAULT_HOST_IP}"
if [[ -z "$HOST_IP" ]]; then
  echo "[FAIL] host IPv4 address was not detected; set NERO_PICO_HOST in .env" >&2
  exit 1
fi
BUILD_DIR="$NERO_TELEOP_ARTIFACTS_DIR/builds"
LOG="$BUILD_DIR/unity_build_pico.log"
APK="$BUILD_DIR/NeroPicoControllerProbe.apk"
mkdir -p "$BUILD_DIR"

export NERO_PICO_HOST="$HOST_IP"
export NERO_PICO_APK="$APK"
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy

"$UNITY" \
  -batchmode \
  -quit \
  -projectPath "$PROJECT_ROOT/pico_client" \
  -executeMethod PicoProjectBuilder.ConfigureAndBuild \
  -logFile "$LOG"

echo "PICO APK: $APK"
echo "UDP destination: ${HOST_IP}:50150"
echo "Unity log: $LOG"
