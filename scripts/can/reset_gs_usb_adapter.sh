#!/usr/bin/env bash
set -euo pipefail

usb_interface="${1:?usage: reset_gs_usb_adapter.sh USB_INTERFACE_PATH}"
driver=/sys/bus/usb/drivers/gs_usb
usb_device="${usb_interface%%:*}"
usb_driver=/sys/bus/usb/drivers/usb

if [[ ! -e "/sys/bus/usb/devices/${usb_interface}" ]]; then
  echo "[CAN] ERROR: USB interface ${usb_interface} is not present" >&2
  exit 1
fi
if [[ ! -e "${driver}/${usb_interface}" ]]; then
  echo "[CAN] ERROR: ${usb_interface} is not bound to gs_usb" >&2
  exit 1
fi

echo "[CAN] resetting stalled gs_usb adapter at ${usb_interface}"
printf '%s\n' "$usb_interface" | sudo tee "${driver}/unbind" >/dev/null
sleep 1
if printf '%s\n' "$usb_interface" | sudo tee "${driver}/bind" >/dev/null 2>&1; then
  sleep 2
fi

if [[ -e "${driver}/${usb_interface}" ]] \
  && find "/sys/bus/usb/devices/${usb_interface}/net" -mindepth 1 -maxdepth 1 \
      -type d -print -quit 2>/dev/null | grep -q .; then
  echo "[CAN] gs_usb adapter rebound at ${usb_interface}"
  exit 0
fi

echo "[CAN] interface rebind failed; re-enumerating USB device ${usb_device}" >&2
if [[ -e "/sys/bus/usb/devices/${usb_device}" ]]; then
  printf '%s\n' "$usb_device" | sudo tee "${usb_driver}/unbind" >/dev/null
  sleep 1
  printf '%s\n' "$usb_device" | sudo tee "${usb_driver}/bind" >/dev/null
  sleep 3
fi

if [[ ! -e "${driver}/${usb_interface}" ]] \
  || ! find "/sys/bus/usb/devices/${usb_interface}/net" -mindepth 1 -maxdepth 1 \
      -type d -print -quit 2>/dev/null | grep -q .; then
  echo "[CAN] ERROR: ${usb_device} did not re-enumerate as gs_usb; physically unplug/replug it" >&2
  exit 1
fi
echo "[CAN] gs_usb adapter recovered after USB re-enumeration at ${usb_interface}"
