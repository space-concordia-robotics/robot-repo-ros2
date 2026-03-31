#!/bin/bash

# Grant permissions to serial devices
sudo chmod 777 /dev/tty*

# Configure memory-mapped registers using busybox devmem
sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400

# Load CAN-related kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Set up the CAN interface
sudo ip link set can0 up type can bitrate 1000000
