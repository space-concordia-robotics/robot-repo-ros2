# SIL Interface

This package provides both a hardware interface and a controller interface for SIL (Status Light Indicator).

## Controller Interface

The controller interface creates a service at `~/set_status` for `rover_msgs/SetSILStatus`.
This service is what is used to control SIL.

## Hardware Interface

The hardware interface interfaces with SIL over CAN Bus.
