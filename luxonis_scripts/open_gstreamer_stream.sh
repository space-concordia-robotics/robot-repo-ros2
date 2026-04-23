#!/usr/bin/env bash
# Runs luxonis_viewer.py (tiled RTSP UI). Encoder must be running on HOST:8554.
# Usage: ./open_gstreamer_stream.sh [HOST] [-- viewer args…]
#   ./open_gstreamer_stream.sh
#   ./open_gstreamer_stream.sh 192.168.1.5 -- --sources ffc

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST="${LUXONIS_RTSP_HOST:-localhost}"
if [[ "${1:-}" != "" && "${1:-}" != --* ]]; then
  HOST="$1"
  shift
fi
exec python3 "${SCRIPT_DIR}/luxonis_viewer.py" --host "${HOST}" "$@"
