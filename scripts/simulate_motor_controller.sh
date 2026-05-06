#!/usr/bin/env bash
# Desk simulation of motor PWM compare + DLC-4 float payload per scripts/SIMULATION.md:
#   compare = min(Period, (|rads| / MAX_RADS) * Period)
# Uses Period=799, MAX_RADS=1024 unless overridden.
#
# One-shot (print math + little-endian payload hex):
#   ./simulate_motor_controller.sh 512
#   ./simulate_motor_controller.sh --max-rads 1024 --period 799 -512
#
# Loop two commands (on/off rads) at --hz; optional SocketCAN send:
#   ./simulate_motor_controller.sh --hz 5 --on 512 --off 0
#   ./simulate_motor_controller.sh --hz 10 --on 1024 --off 0 --can-if vcan0 --can-id 0001848C
#
# Requires: awk, python3 (for IEEE float → hex). Optional: cansend for --can-if.

set -euo pipefail

MAX_RADS_DEFAULT=1024
PERIOD_DEFAULT=799

usage() {
  cat >&2 <<EOF
Desk simulation per scripts/SIMULATION.md (compare + float payload).
Usage:
  $0 [options] <rads>           # one-shot
  $0 --hz N --on A --off B ...  # alternate on/off rads every 1/N s

Options:
  --period N     timer Period (default $PERIOD_DEFAULT)
  --max-rads M   MAX_RADS (default $MAX_RADS_DEFAULT)
  --can-if IF    with --can-id, run cansend each step
  --can-id ID    8 hex digits extended, e.g. 0001848C
  -h, --help
EOF
}

motor_payload_hex() {
  python3 -c 'import struct,sys; print(struct.pack("<f", float(sys.argv[1])).hex())' "$1"
}

# Prints one line of simulation to stdout; optionally sends CAN frame.
simulate_line() {
  local rads=$1
  local max_r=$2
  local period=$3
  local label=${4:-}

  awk -v r="$rads" -v mx="$max_r" -v p="$period" -v lb="$label" 'BEGIN {
    if (r == 0) mag = 0; else mag = (r < 0) ? -r : r
    ratio = mag / mx
    c = ratio * p
    if (c > p) c = p
    cu = int(c)
    if (r > 0) dir = "+"
    else if (r < 0) dir = "-"
    else dir = "0"
    if (lb != "") printf "%s\t", lb
    printf "rads=%s\t|mag|/MAX=%.6f\tcompare_f=%.4f\tcompare_u16=%d\tdir=%s\n", r, ratio, c, cu, dir
  }'
}

simulate_and_maybe_send() {
  local rads=$1
  local max_r=$2
  local period=$3
  local label=$4
  local can_if=${5:-}
  local can_id=${6:-}

  local hex
  hex=$(motor_payload_hex "$rads")
  simulate_line "$rads" "$max_r" "$period" "$label"
  if [[ -n "$can_id" ]]; then
    printf "\tpayload_le=%s\tcansend … %s#%s\n" "$hex" "$can_id" "$hex"
  else
    printf "\tpayload_le=%s\n" "$hex"
  fi

  if [[ -n "$can_if" && -n "$can_id" ]]; then
    if ! command -v cansend >/dev/null 2>&1; then
      echo "error: cansend not found but --can-if/--can-id set" >&2
      exit 1
    fi
    cansend "$can_if" "${can_id}#${hex}"
  fi
}

MAX_RADS=$MAX_RADS_DEFAULT
PERIOD=$PERIOD_DEFAULT
HZ=""
ON_RADS=""
OFF_RADS=""
CAN_IF=""
CAN_ID=""
RADS_SINGLE=""
positional=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --period) PERIOD=$2; shift 2 ;;
    --max-rads) MAX_RADS=$2; shift 2 ;;
    --hz) HZ=$2; shift 2 ;;
    --on) ON_RADS=$2; shift 2 ;;
    --off) OFF_RADS=$2; shift 2 ;;
    --can-if) CAN_IF=$2; shift 2 ;;
    --can-id) CAN_ID=$2; shift 2 ;;
    *)
      positional+=("$1")
      shift
      ;;
  esac
done

if [[ -n "$HZ" ]]; then
  if [[ -z "$ON_RADS" || -z "$OFF_RADS" ]]; then
    echo "error: --hz requires --on and --off (rads values)" >&2
    usage
    exit 1
  fi
  if ! awk -v h="$HZ" 'BEGIN { exit !(h > 0) }'; then
    echo "error: --hz must be positive" >&2
    exit 1
  fi
  SLEEP_SEC=$(awk -v hz="$HZ" 'BEGIN { printf "%.6f", 1.0 / hz }')
  echo "Simulating (SIMULATION.md): MAX_RADS=$MAX_RADS Period=$PERIOD  |  step ${SLEEP_SEC}s (${HZ} Hz)  |  Ctrl+C to stop" >&2
  [[ -n "$CAN_IF" && -n "$CAN_ID" ]] && echo "  cansend: $CAN_IF ${CAN_ID}#<payload>" >&2
  while true; do
    echo "--- ON ---" >&2
    simulate_and_maybe_send "$ON_RADS" "$MAX_RADS" "$PERIOD" "ON" "$CAN_IF" "$CAN_ID"
    sleep "$SLEEP_SEC"
    echo "--- OFF ---" >&2
    simulate_and_maybe_send "$OFF_RADS" "$MAX_RADS" "$PERIOD" "OFF" "$CAN_IF" "$CAN_ID"
    sleep "$SLEEP_SEC"
  done
fi

if [[ ${#positional[@]} -eq 1 ]]; then
  RADS_SINGLE=${positional[0]}
elif [[ ${#positional[@]} -gt 1 ]]; then
  echo "error: expected a single <rads> for one-shot mode" >&2
  exit 1
else
  echo "error: provide <rads> or --hz with --on/--off" >&2
  usage
  exit 1
fi

echo "SIMULATION.md model: MAX_RADS=$MAX_RADS Period=$PERIOD" >&2
simulate_and_maybe_send "$RADS_SINGLE" "$MAX_RADS" "$PERIOD" "1shot" "$CAN_IF" "$CAN_ID"
