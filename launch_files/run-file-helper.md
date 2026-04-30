# FK + MoveIt Run Commands

Use one of the following flows (do not run overlapping launch files together unless noted).

## 0) Build (run before launching)

Run this once after code changes:

```bash
cd /home/mach/projects/robot-repo-ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Faster package-only build (optional):

```bash
cd /home/mach/projects/robot-repo-ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select joy_mux_controller_py rover_arm_moveit_config arm_controller ik_mux_controller
```

---

## 1) Environment (new terminal)

```bash
cd /home/mach/projects/robot-repo-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## 2) Single-command FK + MoveIt + RViz + keyboard (recommended)

This launch already starts:
- `moveit_launch.py` (ros2_control + controllers + servo)
- `rover_arm_moveit_config/launch/move_group.launch.py`
- `rover_arm_moveit_config/launch/moveit_rviz.launch.py`
- `fk_moveit_keyboard_controller`

```bash
ros2 launch launch_files/fk_moveit_rviz_keyboard.launch.py
```

---

## 3) Manual split run (same stack, separate commands)

Terminal A:
```bash
cd /home/mach/projects/robot-repo-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch launch_files/moveit_launch.py use_fake_hardware:=true
```

Terminal B:
```bash
cd /home/mach/projects/robot-repo-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_arm_moveit_config move_group.launch.py use_fake_hardware:=true
```

Terminal C:
```bash
cd /home/mach/projects/robot-repo-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_arm_moveit_config moveit_rviz.launch.py use_fake_hardware:=true use_joint_state_publisher_gui:=false
```

Terminal D:
```bash
cd /home/mach/projects/robot-repo-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run joy_mux_controller_py fk_moveit_keyboard_controller
```

---

## 4) FK keyboard + arm controller local launch (non-MoveIt path)

```bash
ros2 launch joy_mux_controller_py fk_keyboard.launch.py local_mode:=true
```

---

## 5) Controller test launch

Starts: `ros2_control`, `move_group`, `rviz2`, `joy_node`, `ik_mux`, `servo_node`, and controller spawners.

```bash
ros2 launch launch_files/controller_test_launch.py
```

---

## 6) Arm hardware test launch

```bash
ros2 launch launch_files/arm_test.py
```

---

## 7) Individual controller nodes (if needed)

```bash
ros2 run joy_mux_controller_py fk_keyboard_controller
ros2 run joy_mux_controller_py fk_moveit_keyboard_controller
ros2 run joy_mux_controller_py joy_mux_controller
```

---

## Duplicate / Overlap Notes

- `fk_moveit_rviz_keyboard.launch.py` already includes `moveit_launch.py`, `move_group.launch.py`, `moveit_rviz.launch.py`, and `fk_moveit_keyboard_controller`.
- `controller_test_launch.py` also launches core MoveIt/control stack pieces (`move_group`, `rviz`, `servo`, controllers). Avoid running it together with `fk_moveit_rviz_keyboard.launch.py`.
- `fk_keyboard.launch.py` already starts `arm_controller_node` + `fk_keyboard_controller`; do not separately run `ros2 run joy_mux_controller_py fk_keyboard_controller` unless you intentionally want a second instance.
- `arm_test.py` launches `arm_controller_node` and `joy_mux_controller`; avoid parallel duplicate launches of those same nodes.
