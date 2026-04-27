## PID Tuning, Calibration, and Expected-vs-Actual Encoder Diagnostics

Yes, this is a great idea, and you already have most of the plumbing in place.

### Best way to tune PID safely in your stack

1. Start with one joint at a time
- Lock or park the other joints.
- Use small velocity commands and short tests.
- Keep a physical kill switch and low speed limits during tuning.

2. Tune in this order: P, then D, then I
- Set I = 0 and D = 0.
- Increase P until you see oscillation, then back off about 30 to 50%.
- Increase D until overshoot/ringing is damped.
- Add small I only if you still have steady-state bias (gravity/friction drift).

3. Use repeatable test motions
- Step tests: command small fixed velocity changes.
- Ramp tests: slowly increase command.
- Hold tests: command zero and check drift.

4. Tune where your current gains live
- Your per-joint gains are in ros2_controllers.yaml.
- Controller update rate is also set there.

5. Reduce false “bad tuning” during tests
- Servo timeout is strict in servo_config.yaml.
- If commands jitter, you may see stop/drop behavior that looks like poor PID. Temporarily relaxing this while tuning helps.

### Resources worth using

1. ROS 2 control JointTrajectoryController docs (PID behavior and state topic details)
2. control_toolbox PID docs (anti-windup, clamp behavior)
3. Practical method: Ziegler-Nichols as a rough start, then manual refinement per joint
4. rosbag2 + PlotJuggler for error plots over time

### About publishing expected encoder values vs current

Yes, absolutely, and you may already get much of this from the controller state topic.

1. Check for existing controller state
- Topic usually: /arm_controller/controller_state
- Message includes desired, actual, and error trajectories per joint
- If present, this is the fastest path and usually better than rolling your own first

2. If you want custom diagnostics, build a small node that
- Subscribes to /arm_controller/joint_trajectory (expected command)
- Subscribes to /joint_states (measured encoder-derived state)
- Matches by joint name and timestamp
- Publishes per-joint error:
  - position_error = expected_position - actual_position
  - velocity_error = expected_velocity - actual_velocity
- Publishes RMS error over a sliding window for each joint (very useful for tuning decisions)

### Important caution for current hardware interface

- Read feedback currently fills only 4 encoder-backed joints.
- So expected-vs-actual plots for joint4 and joint7 may be misleading unless those states are populated reliably.

---

## Are PID Gains Locked After Build?

They are not locked after build.

For this setup, PID values are runtime parameters loaded when the controller starts from ros2_controllers.yaml.  
So rebuild is usually not required for gain changes.

What matters is controller support for dynamic parameter updates:

1. If dynamic updates are allowed, you can change gains live with `ros2 param set`.
2. If not, you must restart or respawn the arm controller (or controller_manager process) to apply new gains.

### Quick way to verify on your robot

1. `ros2 param list /arm_controller`
2. `ros2 param describe /arm_controller gains.joint1.p`
3. Try a live update: `ros2 param set /arm_controller gains.joint1.p 45.0`

If that set command is rejected, gains are effectively fixed after controller startup, not after build.  
In that case: edit YAML, restart controller stack, retest.


we can read fk for 2 servos
ik doesnt it let us because cant send states connected, PID

to correct for backlash
to have the joint on the 
error correction for aruCO tags
the path from moveit is position trajectories, current - desired 
torque for more complex torque control to take dynamics into account
pos/vel control stresses the motor more
