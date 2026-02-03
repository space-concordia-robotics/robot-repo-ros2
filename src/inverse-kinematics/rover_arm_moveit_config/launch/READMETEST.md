# Launching Arm in RViz - controller_test_launch.py

Follow these steps in order to launch the arm visualization in RViz:

## Step 1: Install Required ROS2 Packages

```bash
sudo apt update
sudo apt install ros-humble-joint-state-publisher-gui
```

## Step 2: Navigate to Workspace Root

```bash
cd ~/robot-repo-ros2
```

## Step 3: Build Required Packages
```bash
colcon build --packages-select \
  rover_arm_description \
  rover_arm_moveit_config
```

## Step 4: Source the Workspace

```bash
source install/setup.bash
```

## Step 5: Launch RViz Visualization

```bash
ros2 launch rover_arm_moveit_config controller_test_launch.py
```

This will start:
- Robot state publisher with the arm URDF
- RViz2 with the arm configuration
- Joint state publisher GUI for manual joint control