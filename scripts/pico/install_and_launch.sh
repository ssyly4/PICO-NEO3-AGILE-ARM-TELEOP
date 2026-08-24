#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../common.sh
source "$SCRIPT_DIR/../common.sh"
ADB=${ADB:-$(command -v adb || true)}
APK=${APK:-$NERO_TELEOP_ARTIFACTS_DIR/builds/NeroPicoControllerProbe.apk}
PACKAGE=com.nero.teleop.picocontrollerprobe

if [[ ! -x "$ADB" ]]; then
  echo "ERROR: adb not found: $ADB" >&2
  exit 1
fi
if [[ ! -f "$APK" ]]; then
  echo "ERROR: APK not found: $APK" >&2
  exit 1
fi

"$ADB" start-server >/dev/null
device_count=$("$ADB" devices | awk 'NR > 1 && $2 == "device" { count++ } END { print count + 0 }')
if [[ "$device_count" -ne 1 ]]; then
  echo "ERROR: expected exactly one authorized PICO, found $device_count" >&2
  "$ADB" devices -l >&2
  exit 1
fi

model=$("$ADB" shell getprop ro.product.model | tr -d '\r')
build=$("$ADB" shell getprop ro.build.display.id | tr -d '\r')
echo "PICO connected: model=$model build=$build"

"$ADB" install -r "$APK"
"$ADB" shell am force-stop "$PACKAGE"
"$ADB" shell monkey -p "$PACKAGE" -c android.intent.category.LAUNCHER 1 >/dev/null

echo "PICO probe launched: $PACKAGE"
echo "On the host, inspect packets with:"
echo "  $PROJECT_ROOT/scripts/pico/check_input.sh"
