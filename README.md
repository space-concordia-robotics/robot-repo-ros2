<<<<<<< HEAD
<<<<<<< HEAD
# Inverse Kinematics Hardware_interface 
=======
=======
# Inverse Kinematics Hardware_interface 
>>>>>>> origin/main
# Space Concordia Robotics - Ceres Rover ROS2 Workspace

This repository contains the ROS2 workspace for the Ceres rover developed by Space Concordia Robotics. The rover is designed for autonomous navigation, manipulation, and data collection using various sensors and actuators.

## 🤖 Project Overview

The Ceres rover is a six-wheeled robotic platform equipped with:
- **6-DOF Robotic Arm** - For object manipulation and sample collection
- **Ouster LiDAR** - For 3D mapping and obstacle detection  
- **ZED2 Stereo Camera** - For computer vision and depth perception
- **RTSP Cameras** - For teleoperation and monitoring
- **GPS Module (u-blox)** - For absolute positioning
- **CAN Bus Communication** - For motor control and sensor integration
- **Joystick Control** - For manual teleoperation

## 📋 Prerequisites

Before setting up this workspace, ensure you have:

- **Ubuntu 22.04** (recommended)
- **ROS2 Humble** - [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Python 3.8+**
- **colcon** build tool: `sudo apt install python3-colcon-common-extensions`
- **Git** for version control

### Additional Dependencies

Install the following ROS2 packages:
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-ublox-gps
sudo apt install ros-humble-aruco-opencv
sudo apt install ros-humble-zed-wrapper
sudo apt install ros-humble-rtsp-camera
```

## 🚀 Quick Start

### 1. Clone and Build the Workspace

```bash
# Clone the repository
git clone https://github.com/space-concordia-robotics/robot-repo-ros2.git
cd robot-repo-ros2

# Install dependencies using rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Hardware Setup (On Rover Jetson)

If running on the actual rover hardware:

```bash
# Run the hardware setup script (requires sudo)
cd launch_files
sudo ./setup.sh
```

This script will:
- Configure serial device permissions
- Set up memory-mapped registers for hardware communication
- Load CAN bus kernel modules
- Initialize CAN interface at 1Mbps

### 3. Launch the Rover

#### Full System Launch
```bash
# Launch all rover subsystems
ros2 launch launch_files/Full_Launch_File.py
```

#### Individual Component Launch
```bash
# Launch basic rover functionality (without cameras)
ros2 launch launch_files/rover-start.py

# Launch only LiDAR
ros2 launch launch_files/lidar_launch.py

# Test individual components
ros2 launch launch_files/wheel_test.py    # Wheel control test
ros2 launch launch_files/arm_test.py      # Arm control test
```

## 🔧 Package Overview

### Core Packages

| Package | Description |
|---------|-------------|
| `wheels_controller` | Controls the 6-wheel drive system via CAN bus |
| `arm_controller` | Manages the 6-DOF robotic arm |
| `gps_py` | GPS data processing and marker publishing |
| `joy_mux_controller` | Joystick input multiplexing and command routing |
| `ceres_urdf` | Robot description files (URDF/XACRO) |
| `absenc_interface` | Sensor interface abstraction |

### Third-Party Packages

| Package | Description |
|---------|-------------|
| `ouster-ros` | Ouster LiDAR driver and data processing |
| `usb_cam` | USB camera driver for additional sensors |

## 🎮 Remote Control

### Connecting to Rover
```bash
ssh nvidia@10.240.0.10
```

### Joystick Control
The rover supports joystick control through the `joy_mux_controller` package. Connect a compatible gamepad and the controller will automatically detect and configure it.

## 🛠️ Development

### Building Individual Packages
```bash
# Build specific packages only
colcon build --packages-select wheels_controller arm_controller

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Testing
```bash
# Run tests for all packages
colcon test

# Run tests for specific package
colcon test --packages-select wheels_controller

# View test results
colcon test-result --verbose
```

### Code Style
This project follows ROS2 coding standards. Use the following tools:

```bash
# C++ linting (from ouster-ros)
./src/ouster-ros/ouster-ros/ouster-sdk/clang-linting.sh

# Python linting
flake8 src/*/python/
```

## 🗺️ Robot Description

The robot's physical structure is defined in `src/ceres_urdf/urdf/ceres.xacro`. To visualize the robot:

```bash
# Launch robot state publisher and RViz
ros2 launch ceres_urdf display.launch.py
```

## 📡 Sensors and Actuators

### LiDAR (Ouster)
- **Model**: Ouster OS1/OS2 series
- **Interface**: Ethernet
- **Driver**: [ouster-ros](https://github.com/ouster-lidar/ouster-ros/tree/ros2)
- **Topics**: `/os_cloud_node/points`, `/os_cloud_node/range_image`

### Cameras
- **ZED2**: Stereo camera for depth perception
- **RTSP Cameras**: Network cameras for teleoperation
- **Configuration**: `launch_files/rtsp_cameras.yaml`

### GPS
- **Model**: u-blox GPS module
- **Interface**: Serial/USB
- **Topics**: `/fix`, `/navsat/fix`

### Motors
- **Drive System**: 6-wheel independent drive
- **Interface**: CAN bus (socketcan)
- **Protocol**: Custom CAN frames for motor control

## 🚨 Troubleshooting

### Common Issues

**Build Failures:**
```bash
# Clear build artifacts and rebuild
rm -rf build/ install/ log/
colcon build
```

**CAN Bus Issues:**
```bash
# Check CAN interface status
ip link show can0

# Restart CAN interface
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

**Permission Errors:**
```bash
# Fix serial device permissions
sudo chmod 777 /dev/tty*

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

**Missing Dependencies:**
```bash
# Reinstall dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Make your changes and test thoroughly
4. Follow the existing code style and ROS2 conventions
5. Submit a pull request with a clear description

### Development Workflow
- Test all changes in simulation before deploying to hardware
- Update documentation for any new features or API changes
- Ensure all tests pass before submitting PRs

## 📄 License

This project is licensed under the terms specified in the [LICENSE](LICENSE) file.

## 🆘 Support

For questions, issues, or contributions:
- Open an issue on GitHub
- Contact Space Concordia Robotics team
- Refer to [ROS2 Documentation](https://docs.ros.org/en/humble/) for general ROS2 help

---

**Space Concordia Robotics** - Building the future of space exploration, one rover at a time.
>>>>>>> main
