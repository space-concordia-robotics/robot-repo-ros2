#!/bin/bash

gst-launch-1.0 rtspsrc location=rtsp://10.240.0.10:8554/cam0 latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &

gst-launch-1.0 rtspsrc location=rtsp://10.240.0.10:8554/cam1 latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &

gst-launch-1.0 rtspsrc location=rtsp://10.240.0.10:8554/cam2 latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &
