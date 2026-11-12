# Swerve Drive (`hardware/swerve`)

This directory contains the ROS 2 packages implementing and simulating a **4-Wheel Independent Steering (4WIS) Swerve Drive** system.

---

## Packages Overview

| Package | Path | Description |
| :--- | :--- | :--- |
| **`swerve_controller`** | `hardware/swerve/swerve-controller` | A `ros2_control` (`controller_interface::ControllerInterface`) lifecycle plugin that converts chassis velocity commands (`~/cmd_vel`) into individual steering angle (position) and drive wheel speed (velocity) commands for 4 swerve modules. |
| **`swerve_simulation`** | `hardware/swerve/swerve-simulation` | A Gazebo Sim (`gz-sim`) simulation package containing the `swervebot` URDF/Xacro model, `ros2_control` configuration, flat test world, and launch script. |

---

## File-by-File Documentation

### 1. Root Directory (`hardware/swerve/`)
* **`README.md`**: Main documentation and architectural overview of the swerve drive packages and files.
* **`ahmet.README.md`**: Quick-start terminal commands for launching the simulation and running `teleop_twist_keyboard`.
* **`TODO.md`**: Project roadmap tracking pending features (unit tests, `cmd_vel` timeout, per-wheel velocity scaling, deactivation safety stops, open/closed-loop odometry, and Nav2 swerve critics).

### 2. `swerve_controller` (`hardware/swerve/swerve-controller/`)
* **`package.xml`**: ROS 2 package manifest declaring build and runtime dependencies (`ament_cmake_ros`, `scrb_cmake_utils`, `scrb_common_util`).
* **`CMakeLists.txt`**: Build script that generates C++ parameter headers from `src/swerve_controller_parameters.yaml` via `generate_parameter_library`, compiles `swerve_drive.cpp` and `swerve_kinematics.cpp` into the `libswerve_controller.so` shared library, and exports the `pluginlib` description.
* **`swerve_controller.xml`**: `pluginlib` registration file exposing `swerve_controller::SwerveController` under the plugin name `swerve_controller/SwerveController`.
* **`src/swerve_controller_parameters.yaml`**: Declarative parameter schema defining:
  * Module joint names (`modules.{front_left,front_right,rear_left,rear_right}.{wheel_joint,steering_joint}`)
  * Robot geometry (`wheel_radius`, `wheelbase`, `track_width`)
  * Odometry & TF parameters (`tf_frame_prefix`, `odom_frame_id`, `base_frame_id`, pose/twist covariances, `enable_odom_tf`)
  * Safety & rate-limiting parameters (`cmd_vel_timeout`, `linear.x`, `linear.y`, `angular.z` velocity/acceleration/jerk bounds)
* **`include/swerve_controller/swerve_kinematics.hpp` & `src/swerve_kinematics.cpp`**:
  * Defines `ModulePosition` $(x, y)$, `ModuleState` $(\text{speed}, \text{angle})$, and the `SwerveKinematics` class.
  * Precomputes the $2 \times 4$ tangential rotation matrix $\begin{bmatrix}-y_i \\ x_i\end{bmatrix}$ from the $4 \times 2$ module coordinates.
  * `inverse()`: Computes each wheel's 2D velocity vector $\mathbf{v}_i = \mathbf{v}_{\text{linear}} + \omega_z \begin{bmatrix}-y_i \\ x_i\end{bmatrix}$, converts it to polar speed and steering angle, holds the current steering angle when speed $\le 10^{-9}\text{ m/s}$, and calls `optimize()`.
  * `optimize()`: Minimizes steering movement by flipping the steering target by $\pi\text{ rad}$ and reversing wheel speed whenever the shortest angular distance exceeds $\pi/2\text{ rad}$ ($90^\circ$).
* **`include/swerve_controller/swerve_drive.hpp` & `src/swerve_drive.cpp`**:
  * Implements the `SwerveController` lifecycle plugin:
    * `on_init()`: Initializes `ros2_fmt_logger` and `ParamListener`.
    * `command_interface_configuration()` / `state_interface_configuration()`: Claims `<steering_joint>/position` and `<wheel_joint>/velocity` command and state interfaces for all 4 modules.
    * `on_configure()`: Computes the 4 corner module positions from `wheelbase` and `track_width`, initializes `SwerveKinematics`, subscribes to `~/cmd_vel` (`geometry_msgs::msg::TwistStamped`), and configures `control_toolbox::RateLimiter` instances.
    * `on_activate()` / `on_deactivate()`: Binds loaned hardware interfaces into `SwerveModuleHandle` structs and toggles command reception.
    * `update()`: Real-time control loop step that reads the latest command from `RealtimeBuffer`, rate-limits velocities, reads current steering angles, computes inverse kinematics, and writes target steering angles ($\text{rad}$) and wheel angular velocities ($v / r$, in $\text{rad/s}$) to hardware.

### 3. `swerve_simulation` (`hardware/swerve/swerve-simulation/`)
* **`package.xml` & `CMakeLists.txt`**: Package manifest and CMake install rules that copy `config`, `launch`, `urdf`, and `worlds` into `share/swerve_simulation/`.
* **`config/controllers.yaml`**: Configures `controller_manager` at $200\text{ Hz}$ with `joint_state_broadcaster` and `swerve_controller`, setting `wheelbase: 1.6`, `track_width: 1.2`, `wheel_radius: 0.25`, and the 8 joint names.
* **`urdf/swervebot.urdf`**: Xacro/URDF robot description for `swervebot` ($20\text{ kg}$ chassis + 4 corner modules with revolute steering joints and continuous wheel joints) integrated with `gz_ros2_control::GazeboSimROS2ControlPlugin`.
* **`worlds/flat.world`**: SDF 1.9 Gazebo world with $200\text{ Hz}$ ODE physics (`max_step_size: 0.005`), sun light, and a $1000\times 1000\text{ m}$ ground plane.
* **`launch/sim.launch.py`**: Launch file using `launch_util.SimpleLauncher` to start `robot_state_publisher`, launch Gazebo with `flat.world`, spawn `swervebot` at $z = 0.5\text{ m}$, sequentially spawn `joint_state_broadcaster` and `swerve_controller`, and bridge `/clock`, `/joint_states`, and `/tf`.

---

## Quick Start

### 1. Build the Packages
```bash
colcon build --packages-select swerve_controller swerve_simulation
source install/setup.bash
```

### 2. Launch the Simulation
```bash
ros2 launch swerve_simulation sim.launch.py
```

### 3. Teleoperate with Keyboard
In a second terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -r cmd_vel:=/swerve_controller/cmd_vel
```

