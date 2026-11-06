#!/bin/bash

HOST=10.240.0.10
PORT=8554
RTSP="rtsp://${HOST}:${PORT}"

FFPLAY_FLAGS=(
    -fflags nobuffer
    -flags low_delay
    -rtsp_transport udp
    -framedrop
    -probesize 32
    -analyzeduration 0
    -sync ext
    -reconnect 1
    -reconnect_at_eof 1
    -reconnect_streamed 1
    -reconnect_delay_max 5
)

play() {
    local name=$1
    ffplay "${FFPLAY_FLAGS[@]}" -window_title "$name" "$RTSP/$name" >/dev/null 2>&1 &
}

mode=$1
shift

case "$mode" in
    ffc)
        if [ "$#" -eq 0 ]; then
            echo "usage: $0 ffc <cam> [cam ...]   (cams: front right left back)" >&2
            exit 1
        fi
        for c in "$@"; do play "$(echo "$c" | tr '[:lower:]' '[:upper:]')"; done
        ;;
    ffc_all)            play FRONT; play RIGHT; play LEFT; play BACK ;;
    ffc_front)          play FRONT ;;
    ffc_back)           play BACK ;;
    ffc_right)          play RIGHT ;;
    ffc_left)           play LEFT ;;
    oakd_rgb|oakd_yolo) play RGB ;;
    all)
        play FRONT; play RIGHT; play LEFT; play BACK; play RGB
        ;;
    *)
        echo "$0: unknown mode '$mode'" >&2
        echo "valid: ffc <cams...> | ffc_all | ffc_front | ffc_back | ffc_right | ffc_left | oakd_rgb | oakd_yolo | all" >&2
        exit 1
        ;;
esac

wait
