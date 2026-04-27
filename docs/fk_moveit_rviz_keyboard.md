# FK Keyboard Control in RViz/MoveIt

This guide documents how to control the rover arm with forward kinematics (FK) from the keyboard while visualizing in RViz + MoveIt.

## What this setup does

- Starts MoveIt Servo and ros2_control (velocity arm controller)
- Starts Move Group and RViz MoveIt interface
- Starts a keyboard teleop node that publishes JointJog commands to MoveIt Servo
- Lets you jog one joint at a time from the keyboard

## Files added/updated

- launch_files/fk_moveit_rviz_keyboard.launch.py
- src/joy_mux_controller_py/joy_mux_controller_py/fk_moveit_keyboard_controller.py
- src/joy_mux_controller_py/setup.py
- src/joy_mux_controller_py/package.xml
- src/inverse-kinematics/rover_arm_moveit_config/launch/moveit_rviz.launch.py

## Prerequisites

- ROS 2 Humble installed
- Workspace dependencies installed (moveit, ros2_control stack)
- Workspace built successfully

## Build

From workspace root:

```bash
colcon build --packages-select rover_arm_moveit_config joy_mux_controller_py
source install/setup.bash
```

## Launch FK keyboard + MoveIt + RViz

From workspace root:

```bash
source install/setup.bash
ros2 launch launch_files/fk_moveit_rviz_keyboard.launch.py
```

This launch uses fake ros2_control hardware by default so it works on a dev machine without arm serial devices.

For real hardware testing, use the base MoveIt launch directly with real hardware enabled:

```bash
source install/setup.bash
ros2 launch launch_files/moveit_launch.py use_fake_hardware:=false
```

## Keyboard controls

Joint selection:
- 1 -> joint1
- 2 -> joint2
- 3 -> joint3
- 4 -> joint4
- 5 -> joint5
- 6 -> joint7

Motion:
- l -> positive velocity on selected joint
- j -> negative velocity on selected joint
- k -> stop selected joint command

Speed:
- + or = -> increase speed scale
- - or _ -> decrease speed scale

Quit:
- q -> stop keyboard controller
- Ctrl+C in launch terminal -> stop entire stack

## Topics and interfaces

Keyboard node publishes:
- /servo_node/delta_joint_cmds (control_msgs/msg/JointJog)

MoveIt Servo consumes:
- ~/delta_joint_cmds (resolved as /servo_node/delta_joint_cmds)

MoveIt Servo outputs trajectory commands to:
- /arm_controller/joint_trajectory

Joint states come from:
- /joint_states via joint_state_broadcaster

## Notes

- The keyboard node automatically tries to call /servo_node/start_servo.
- RViz launch now supports disabling joint_state_publisher_gui to avoid /joint_states conflicts during servo control.
- The combined launch disables joint_state_publisher_gui for correct servo-driven motion visualization.
- The combined launch passes use_fake_hardware:=true to moveit_launch.py.

## Troubleshooting

1. No motion in RViz

- Check servo topic traffic:

```bash
ros2 topic echo /servo_node/delta_joint_cmds
```

- Check that Servo started:

```bash
ros2 service list | grep start_servo
```

2. Keyboard input not captured

- Run from an interactive terminal.
- Node uses /dev/tty fallback when stdin is not a TTY.

3. Controller not active

- Verify controllers:

```bash
ros2 control list_controllers
```

Expected active controllers include:
- joint_state_broadcaster
- arm_controller

4. Build/install mismatch

- Rebuild and re-source:

```bash
colcon build --packages-select rover_arm_moveit_config joy_mux_controller_py
source install/setup.bash
```

## When it does not work: exact check sequence

If launch fails intermittently or controllers fail to configure, follow this sequence exactly.

1. Clean stale ROS processes and graph cache

```bash
pkill -f "ros2 launch|move_group|servo_node_main|ros2_control_node|controller_manager|fk_moveit_keyboard_controller|rviz2" || true
ros2 daemon stop
ros2 daemon start
```

2. Start one stack only

```bash
source install/setup.bash
ros2 launch launch_files/fk_moveit_rviz_keyboard.launch.py
```

3. In a second terminal, verify node graph is single-instance

```bash
source install/setup.bash
ros2 node list
```

Expected core nodes include one of each:
- /controller_manager
- /servo_node
- /move_group
- /fk_moveit_keyboard_controller

4. Verify controllers are active

```bash
source install/setup.bash
ros2 control list_controllers
```

Expected:
- joint_state_broadcaster -> active
- arm_controller -> active

5. Verify FK command path

```bash
source install/setup.bash
ros2 topic info /servo_node/delta_joint_cmds
ros2 topic echo /servo_node/delta_joint_cmds --once
```

Expected:
- Type is control_msgs/msg/JointJog
- Publisher count is 1
- Subscription count is 1

6. Functional keyboard test

- Press 1 then hold l: positive velocity appears on joint1.
- Press k: velocities return to 0.
- Press j: negative velocity appears.

## Common log/exit notes

- "No 3D sensor plugin(s) defined for octomap updates" is usually non-blocking for FK jogging.
- Exit code 143 means SIGTERM (process was terminated), commonly seen when launch is stopped externally; it is not by itself a root-cause error.

## Quick validation checklist

- RViz opens with MoveIt panel
- /servo_node/delta_joint_cmds receives JointJog messages
- Selected joint index changes with 1..6
- Holding j/l produces visible joint movement in RViz
- Releasing with k stops movement

## How Cartesian IK Mapping Works Here

For Cartesian commands, the joystick path publishes TwistStamped to MoveIt Servo (`/servo_node/delta_twist_cmds`).
Servo then computes joint velocities from the arm Jacobian at each control cycle.

Important:
- This is a single Jacobian for the active arm chain (group `arm`), not one inverse Jacobian per motor.
- JointJog input (`/servo_node/delta_joint_cmds`) does not need Jacobian inversion because it is already joint-space velocity.

Relevant config points:
- MoveIt Servo group: `move_group_name: arm`
- Servo inputs: `cartesian_command_in_topic`, `joint_command_in_topic`
- Servo output: `command_out_topic: /arm_controller/joint_trajectory`
- Kinematics plugin for planning: `kdl_kinematics_plugin/KDLKinematicsPlugin`

## Why the Arm Can "Drop" In IK/Servo Mode

After fixing the serial end byte issue, these are still common causes in this stack:

1. Activation command initialization bug in hardware interface

- In `arm_interface.cpp`, `on_activate()` currently checks velocity state NaN but writes position into velocity command buffer:
	- `if (!std::isnan(hw_states_velocity_[i]))`
	- `hw_commands_velocity_[i] = hw_states_position_[i];`
- This can inject unintended nonzero velocity commands at activation.

2. Joint state feedback does not fully cover commanded joints

- Controller commands 6 joints (`joint1, joint2, joint3, joint4, joint5, joint7`), but the encoder read path updates only 4 joint state slots.
- Missing or stale state on commanded joints can degrade controller behavior and produce unexpected motion/settling.

3. Servo command timeout can trigger frequent halt behavior

- `incoming_command_timeout: 0.1` is strict.
- If command stream jitter exceeds this timeout, Servo sends halt behavior, which can look like sudden drop/stop cycles.

4. Model/controller joint mismatch risk

- Ensure the SRDF arm group and ros2_control joint list are aligned.
- Any mismatch in active joints between planning, servo, and controller layers can cause unstable IK execution.

## Should We Switch To Position Command Interface?

Short answer: not yet.

Because your motor firmware accepts velocity-style commands, changing ros2_control hardware command interface to position now is likely to worsen behavior unless you also add a robust inner position loop in firmware.

Recommended path:
- Keep hardware interface velocity-based.
- Keep outer position behavior at controller/planner level (JointTrajectoryController + MoveIt).
- Fix state/activation issues first, then tune gains.

## Priority Fix Order

1. Fix `on_activate()` velocity command initialization bug.
2. Make state feedback consistent with all commanded joints.
3. Align SRDF/MoveIt joint set with ros2_control joint set.
4. Increase `incoming_command_timeout` (for example to 0.2-0.3) and retest.
5. Re-tune per-joint velocity-loop gains after the above fixes.

changes to be done 
ctrl + shift + F
in include: arm_interface.cpp, ros2_controllers.yaml, servo_config.yaml