# Swerve Drive

The packages in this folder are used for implementing swerve drive.

Packages:

- swerve-controller: implements the ros2_control controller interface for swerve drive.
- swerve-simulation: simple simulation for testing swerve drive

To run the swerve drive simulation, you can do

source opt/ros/jazzy/setup.bash
colcon build --mixing --packages-up-to swerve_simulation
source install/setup.bash
ros2 launch swerve_simulation sim.launch.py
