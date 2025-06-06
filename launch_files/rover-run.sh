#!/bin/bash

# Run setup.sh and wait for it to complete
./setup.sh

wait

# Launch rover-start in a new terminal
gnome-terminal -- bash -c "ros2 launch  rover-start.py; exec bash"

# Launch aruco_tracker in a new terminal
gnome-terminal -- bash -c "ros2 launch aruco_tracker.py; exec bash"