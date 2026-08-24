#!/usr/bin/env bash
set -euo pipefail

target_name="${1:?usage: ensure_can_interface.sh TARGET_NAME USB_BUS_PATH}"
target_bus="${2:?usage: ensure_can_interface.sh TARGET_NAME USB_BUS_PATH}"
bitrate=1000000

interface_bus() {
  local interface="$1"
  local device_path
  device_path="$(readlink -f "/sys/class/net/${interface}/device")"
  basename "$device_path"
}

find_interface_by_bus() {
  local interface
  for interface_path in /sys/class/net/*; do
    interface="$(basename "$interface_path")"
    [[ -e "/sys/class/net/${interface}/type" ]] || continue
    [[ "$(cat "/sys/class/net/${interface}/type")" == "280" ]] || continue
    if [[ "$(interface_bus "$interface")" == "$target_bus" ]]; then
      printf '%s\n' "$interface"
      return 0
    fi
  done
  return 1
}

if ip link show "$target_name" >/dev/null 2>&1; then
  current_name="$target_name"
  current_bus="$(interface_bus "$current_name")"
  if [[ "$current_bus" != "$target_bus" ]]; then
    echo "[CAN] ERROR: ${target_name} is on ${current_bus}, expected ${target_bus}" >&2
    exit 1
  fi
else
  current_name="$(find_interface_by_bus || true)"
  if [[ -z "$current_name" ]]; then
    echo "[CAN] ERROR: no CAN adapter found at USB ${target_bus}" >&2
    exit 1
  fi
  echo "[CAN] restoring ${current_name} (${target_bus}) as ${target_name}"
  sudo ip link set "$current_name" down
  sudo ip link set "$current_name" name "$target_name"
  current_name="$target_name"
fi

current_bitrate="$(ip -details link show "$current_name" | awk '/bitrate/ {for (i=1; i<=NF; i++) if ($i == "bitrate") {print $(i+1); exit}}')"
if ip link show "$current_name" | grep -q '<[^>]*UP' \
  && [[ "$current_bitrate" == "$bitrate" ]]; then
  echo "[CAN] ${current_name} ready: USB=${target_bus} bitrate=${bitrate}"
  exit 0
fi

echo "[CAN] configuring ${current_name}: USB=${target_bus} bitrate=${bitrate}"
sudo ip link set "$current_name" down
sudo ip link set "$current_name" type can bitrate "$bitrate"
sudo ip link set "$current_name" up

current_bitrate="$(ip -details link show "$current_name" | awk '/bitrate/ {for (i=1; i<=NF; i++) if ($i == "bitrate") {print $(i+1); exit}}')"
if ! ip link show "$current_name" | grep -q '<[^>]*UP' \
  || [[ "$current_bitrate" != "$bitrate" ]]; then
  echo "[CAN] ERROR: ${current_name} did not become ready" >&2
  exit 1
fi
echo "[CAN] ${current_name} ready: USB=${target_bus} bitrate=${bitrate}"
