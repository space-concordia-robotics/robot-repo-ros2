#!/bin/bash

HOST="${1:-localhost}"

# FFC 4P cameras
gst-launch-1.0 rtspsrc location=rtsp://$HOST:8554/ffc0 latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &

gst-launch-1.0 rtspsrc location=rtsp://$HOST:8554/ffc1 latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &

gst-launch-1.0 rtspsrc location=rtsp://$HOST:8554/ffc2 latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &

gst-launch-1.0 rtspsrc location=rtsp://$HOST:8554/ffc3 latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &

# OAK-D Pro W cameras
gst-launch-1.0 rtspsrc location=rtsp://$HOST:8554/oakd_rgb latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &

gst-launch-1.0 rtspsrc location=rtsp://$HOST:8554/oakd_left latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &

gst-launch-1.0 rtspsrc location=rtsp://$HOST:8554/oakd_right latency=0 ! \
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false &
