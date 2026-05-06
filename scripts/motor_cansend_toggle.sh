#!/usr/bin/env bash
# Alternate two cansend payloads (on / off) at a fixed rate.
# Requires: cansend (from can-utils / iproute2), awk
#
# Usage:
#   ./motor_cansend_toggle.sh <hz> [can_interface]
#
# Period between successive commands is 1/hz seconds (on, then off, then on, …).
#
# Set the payloads below, or override at runtime:
#   CMD_ON='1081848C#0000803F' CMD_OFF='1081848C#00000000' ./motor_cansend_toggle.sh 5 can0

set -euo pipefail

usage() {
  echo "Usage: $0 <hz> [can_interface]" >&2
  echo "  hz: positive number; wait 1/hz seconds between each cansend." >&2
  echo "  can_interface: default can0 (or set CAN_IF)." >&2
  echo "  Edit CMD_ON / CMD_OFF in this script, or export them." >&2
}

HZ="${1:-}"
CAN_IF="${2:-${CAN_IF:-can0}}"

if [[ -z "$HZ" ]] || ! awk -v h="$HZ" 'BEGIN { exit !(h > 0) }'; then
  usage
  exit 1
fi

SLEEP_SEC=$(awk -v hz="$HZ" 'BEGIN { printf "%.6f", 1.0 / hz }')

# ========== Put your cansend lines here (id#data, extended ID is 8 hex digits) ==========
CMD_ON="${CMD_ON:-00000000#00000000}"
CMD_OFF="${CMD_OFF:-00000000#00000000}"
# ========================================================================================

if ! command -v cansend >/dev/null 2>&1; then
  echo "error: cansend not found (install can-utils or iproute2)" >&2
  exit 1
fi

echo "Interface: $CAN_IF  |  step every ${SLEEP_SEC}s (${HZ} Hz)  |  Ctrl+C to stop" >&2
echo "  ON : $CMD_ON" >&2
echo "  OFF: $CMD_OFF" >&2

while true; do
  cansend "$CAN_IF" "$CMD_ON"
  sleep "$SLEEP_SEC"
  cansend "$CAN_IF" "$CMD_OFF"
  sleep "$SLEEP_SEC"
done
