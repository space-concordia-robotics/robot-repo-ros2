# Controller Controls Guide

This guide documents the current joystick mappings used by `joy_mux_controller_py` for rover and arm teleoperation.

## Safety First

- Hold **LEFT_BUMPER** (deadman) to allow command output.
- Releasing deadman sends an immediate all-stop and a short zero-command burst (`stop_burst_duration_s`, default `0.5s`) to improve motor stop reliability.

## Mode Switching

- Press **HOME** (`Buttons.TOGGLE`, index `10`) to toggle modes:
  - `Rover` mode
  - `Arm` mode

## Rover Mode (current_mode = 0)

When deadman is held, these fields are published to `/cmd_vel`:

- `linear.x` <- `LEFT_STICK_X`
- `angular.z` <- `LEFT_STICK_Y`
- `linear.y` <- `RIGHT_STICK_X`
- `linear.z` <- `RIGHT_STICK_Y`

## Arm Mode (current_mode = 1)

When deadman is held, a 7-value `JointState.velocity` is published to `/arm_xyz_cmd`.

- `joint1` (index 0) <- `D_PAD_X`
- `joint2` (index 1) <- `D_PAD_Y`
- `joint3` (index 2) <- `RIGHT_STICK_X`
- `joint4` (index 3) <- `RIGHT_TRIGGER - LEFT_TRIGGER` (normalized to signed range)
- `joint5` (index 4) <- `LEFT_STICK_Y`
- `joint6` (index 5) <- `TRIANGLE - X` (servo placeholder: gripper rotate)
- `joint7` (index 6) <- `CIRCLE - SQUARE` (servo placeholder: gripper grip)

## Trigger Behavior

Controller trigger axes rest at `+1.0` and go to `-1.0` when fully pressed.  
The node normalizes each trigger to `0..1` and computes:

- `joint4 = right_trigger_norm - left_trigger_norm`

This gives smooth analog control and avoids button-sticking behavior previously seen with binary button mappings.

## Published Topics

- Input: `/joy` (`sensor_msgs/Joy`)
- Rover output: `/cmd_vel` (`geometry_msgs/Twist`)
- Arm output: `/arm_xyz_cmd` (`sensor_msgs/JointState`)

## Relevant Parameters (`joy_mux_controller`)

- `max_cmd_publish_hz` (default `100.0`)
- `skip_identical` (default `true`)
- `identical_epsilon` (default `1e-3`)
- `stop_burst_duration_s` (default `0.5`)

