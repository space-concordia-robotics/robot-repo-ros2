#!/bin/bash

mode=$1

if [ "$mode" = "ffc_all" ]; then
    vlc --no-one-instance rtsp://localhost:8554/FRONT &
    vlc --no-one-instance rtsp://localhost:8554/LEFT &
    vlc --no-one-instance rtsp://localhost:8554/RIGHT &
    vlc --no-one-instance rtsp://localhost:8554/BACK
elif [ "$mode" = "ffc_front" ]; then
    vlc --no-one-instance rtsp://localhost:8554/FRONT 
elif [ "$mode" = "ffc_left" ]; then
    vlc --no-one-instance rtsp://localhost:8554/LEFT 
elif [ "$mode" = "ffc_right" ]; then
    vlc --no-one-instance rtsp://localhost:8554/RIGHT 
elif [ "$mode" = "ffc_back" ]; then
    vlc --no-one-instance rtsp://localhost:8554/BACK
elif [ "$mode" = "oakd_rgb" ]; then
    vlc --no-one-instance rtsp://localhost:8554/RGB
fi