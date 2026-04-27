# Keyboard Pressing with IK and Vision-Based Pose Estimation

## Overview

This document describes the architecture for using the rover arm to press keys on a wall-mounted keyboard using camera-based keyboard frame detection, pose estimation, and inverse kinematics. This architecture is adapted from the IFRA-Cranfield irb120_PoseEstimation cube pick-and-place pattern for keyboard pressing.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Vision & Perception Layer                    │
│                                                                     │
│  USB Camera → Image Processing → Keyboard Detection → Frame TF      │
│  (raw image)   (calibration,      (fiducial/contour)  (pub on       │
│                 undistortion)                          camera→arm)  │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ /tf topic
                                    ↓
┌─────────────────────────────────────────────────────────────────────┐
│                     Task Planning Layer                             │
│                                                                     │
│  Keyboard Frame → Key Pose Generator → MoveIt IK Client             │
│  (stable frame)   (offset from frame)   (pose →                     │
│                                          joint angles)              │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ /move_group/plan
                                    ↓
┌─────────────────────────────────────────────────────────────────────┐
│                   Motion Execution Layer                            │
│                                                                     │
│  MoveIt arm_controller → JointTrajectoryController → Hardware       │
│  (trajectory)           (velocity commands)        (arm_interface)  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Key Components

### 1. Vision & Perception Layer

**Purpose:** Detect and localize the keyboard in 3D space, establishing a stable reference frame.

**Inputs:**
- USB camera image stream: `/camera/image_raw` (or USB camera topic)

**Outputs:**
- TensorFlow: `/tf` broadcasting `camera_frame → keyboard_frame` transform
- Diagnostic: `/keyboard_frame/pose` (geometry_msgs/PoseStamped)

**Responsibilities:**

1. **Camera Calibration & Undistortion**
   - Load camera intrinsics (via ros2 param or calibration file).
   - Undistort raw images.

2. **Keyboard Detection**
   - Option A: Fiducial marker on keyboard frame (ArUco/AprilTag).
   - Option B: Keyboard contour via OpenCV (edge detection, corner finding).
   - Option C: YOLO-based keyboard bounding box (like irb120_PoseEstimation but for keyboard).

3. **6DoF Pose Estimation**
   - Compute 3D position and orientation of keyboard frame from 2D detections.
   - Use OpenCV solvePnP() or similar.
   - Publish as Transform broadcaster: `keyboard_frame` relative to `camera_frame`.

4. **Frame Stability**
   - Apply low-pass filter or median to smooth jittering poses.
   - Only update MoveIt goals when confidence is high.

**ROS 2 Node:** `keyboard_detector_node` (Python or C++)

**Topics:**
- Subscribe: `/camera/image_raw`
- Publish: `/tf` (via broadcaster), `/keyboard_frame/pose`, `/keyboard_frame/debug` (annotated images)

---

### 2. Task Planning Layer

**Purpose:** Convert detected keyboard frame into per-key target poses, then plan arm trajectories.

**Inputs:**
- Keyboard frame transform (from perception layer).
- Key map (array of key positions relative to keyboard frame origin).
- Press parameters: approach height, press depth, dwell time.

**Outputs:**
- MoveIt motion plans: `/move_group/plan` responses.
- Target poses for each key.

**Responsibilities:**

1. **Key Pose Generator**
   - Maintain a hardcoded or calibrated map of key positions in the keyboard frame.
   - Example: `joint_positions = {('A', 0): [0.015, -0.020, 0.002], ('B', 0): [0.045, -0.020, 0.002], ...}` (meters, relative to keyboard frame origin).
   - Transform each key position from keyboard frame → end-effector approach pose in world frame.
   - Compute approach pose (e.g., 5 cm above key) and press pose (e.g., 1 cm depth).

2. **MoveIt IK Planning**
   - Call MoveIt IK service `/compute_ik` (via MoveItCpp Action Client or service).
   - Provide desired end-effector pose → get joint trajectory.
   - Handle IK failures: retry with nearby approach angles, or skip key.

3. **Collision Checking** (optional)
   - Use MoveIt's collision checking to validate trajectories touch only the key.
   - Reject plans that collide with arm/gripper outside intended contact area.

**ROS 2 Node:** `keyboard_task_planner_node` (Python)

**Topics:**
- Subscribe: `/keyboard_frame/pose` (or TF listener)
- Publish: `/keyboard_task/target_key`, `/keyboard_task/approach_pose`, `/keyboard_task/press_trajectory` (diagnostic)
- Use Actions: `/move_group/plan`, `/move_group/execute`

**Parameters:**
```yaml
keyboard_task_planner:
  # Key positions relative to keyboard frame [x (right), y (up), z (forward)] in meters
  key_positions:
    'A': [0.015, -0.020, 0.002]
    'B': [0.045, -0.020, 0.002]
    # ... more keys
  # Press profile
  approach_height: 0.05  # meters, clearance above key
  press_depth: 0.010     # meters, how far to depress
  press_speed: 0.01      # m/s
  return_height: 0.05    # meters, retract after press
  dwell_time: 0.5        # seconds, hold key down
```

---

### 3. Motion Execution Layer

**Purpose:** Execute planned trajectories on the physical robot.

**Inputs:**
- Motion plans from MoveIt (joint trajectory messages).
- Desired key sequence / press commands.

**Outputs:**
- Joint commands to hardware interface.
- Feedback on execution success.

**Responsibilities:**

1. **Trajectory Execution**
   - Use MoveIt arm_controller (already configured in your stack).
   - Publish trajectory commands to `/arm_controller/joint_trajectory`.
   - Monitor execution via `/arm_controller/controller_state`.

2. **Feedback Loop**
   - Monitor `/joint_states` to verify arm follows trajectory.
   - Detect failures: large position errors, timeout, collision stop.
   - Log press events: timestamp, key, success/failure.

3. **Safe Press Profile**
   - Approach move (relative slow): 0.05 m/s.
   - Press move (very slow): 0.01 m/s, apply force feedback if gripper has sensors.
   - Dwell: hold position for 0.5 s (key registration).
   - Retract: 0.05 m/s back to clearance.

**ROS 2 Node:** `keyboard_executor_node` (Python)

**Topics:**
- Subscribe: `/keyboard_task/press_trajectory`, `/joint_states`
- Publish: `/keyboard_executor/status`, `/keyboard_executor/press_log`
- Use Actions: `/arm_controller/follow_joint_trajectory`

---

## Data Flow Example: Pressing Key 'A'

1. **Perception (t = 0 ms)**
   - Camera captures keyboard image.
   - Detector finds ArUco marker on keyboard.
   - solvePnP computes keyboard frame pose.
   - Broadcaster publishes `tf: camera_frame → keyboard_frame`.

2. **Planning (t = 50 ms)**
   - Task planner receives keyboard frame via TF.
   - Looks up key 'A' position in keyboard frame: (0.015, -0.020, 0.002).
   - Computes world-frame approach pose: (0.5, 0.3, 0.7) with orientation toward key.
   - Calls MoveIt IK: desired pose → joint angles [q1, q2, q3, q4, q5, q7].
   - Plans trajectory from current pose to approach pose to press pose.

3. **Execution (t = 200 ms)**
   - Executor receives validated trajectory.
   - Publishes to `arm_controller/joint_trajectory`.
   - JointTrajectoryController interpolates and sends velocity commands to `arm_interface`.
   - Hardware interface (on real robot) drives motors.
   - Joint state feedback streams back from encoders.
   - After dwell time, retract trajectory executes.
   - Press logged with timestamp and status.

---

## Integration with Existing Stack

### Controllers (Already in place)

```
ros2_controllers.yaml:
  - arm_controller (JointTrajectoryController)
    ├─ command_interface: velocity
    └─ state_interface: position, velocity
  - joint_state_broadcaster
```

**No changes needed** if you keep JointTrajectoryController.

### Hardware Interface

Your current `arm_interface.cpp` handles:
- Reading 4 encoders → joint state (joint1, joint2, joint3, joint5).
- Writing velocity commands to 6 motor channels.

**One noted issue:** joint4 and joint7 lack encoder feedback. Until addressed:
- IK solutions cannot verify joint4/joint7 positions reliably.
- Use joint limits and collision checking conservatively.

### Launch Files

Add keyboard keyboard nodes to your existing FK/IK launch:

```python
# fk_moveit_rviz_keyboard.launch.py (modified)

keyboard_detector = Node(
    package='keyboard_detector',
    executable='keyboard_detector_node',
    name='keyboard_detector',
    output='screen',
    parameters=[...camera_params...]
)

keyboard_planner = Node(
    package='keyboard_task_planner',
    executable='keyboard_task_planner_node',
    name='keyboard_planner',
    output='screen',
    parameters=['config/keyboard_task_params.yaml']
)

keyboard_executor = Node(
    package='keyboard_executor',
    executable='keyboard_executor_node',
    name='keyboard_executor',
    output='screen'
)

# Add to ld.LaunchDescription([ ... ])
```

---

## Packages to Create or Adapt

### 1. `keyboard_detector` (New)

**Purpose:** Vision-based keyboard frame detection.

**Files:**
```
keyboard_detector/
├── launch/
│   └── keyboard_detector.launch.py
├── config/
│   ├── camera_calibration.yaml
│   └── keyboard_detector_params.yaml
├── src/
│   └── keyboard_detector_node.py
├── package.xml
└── setup.py
```

**Key Logic:**
- Subscribe to camera image.
- Detect fiducial marker (ArUco) or keyboard contour.
- Estimate 6DoF pose.
- Broadcast TF.

### 2. `keyboard_task_planner` (New)

**Purpose:** Task-level motion planning for key pressing.

**Files:**
```
keyboard_task_planner/
├── launch/
│   └── keyboard_task_planner.launch.py
├── config/
│   └── keyboard_task_params.yaml
├── src/
│   └── keyboard_task_planner_node.py
├── package.xml
└── setup.py
```

**Key Logic:**
- Listen to keyboard frame TF.
- Maintain key map.
- Generate per-key target poses.
- Call MoveIt IK and plan.

### 3. `keyboard_executor` (New)

**Purpose:** Execution and monitoring.

**Files:**
```
keyboard_executor/
├── launch/
│   └── keyboard_executor.launch.py
├── src/
│   └── keyboard_executor_node.py
├── package.xml
└── setup.py
```

**Key Logic:**
- Wait for press commands.
- Execute approach → press → retract trajectory.
- Monitor joint state feedback.
- Log outcomes.

---

## Key Calibration Steps

### 1. Camera Calibration
- Capture checkerboard images at different angles.
- Run OpenCV calibration: intrinsics matrix K, distortion coefficients.
- Save to `camera_calibration.yaml`.

### 2. Keyboard Frame Registration
- Place known markers (ArUco/AprilTag) on keyboard frame corners.
- Measure marker positions relative to keyboard origin (in CAD model).
- In detector, compute keyboard frame → marker positions using solvePnP.

### 3. Key Position Mapping
- Manually measure each key position relative to keyboard frame origin (CAD or physical measurement).
- Populate `keyboard_task_params.yaml` with key map.
- Validate in simulation first (Gazebo + keyboard model).

---

## Safety Considerations

1. **Hard Stops:** Disable Servo `collision_check` initially; rely on joint limits only.
2. **Speed Limits:** Set conservative velocity gains; test on FK mode first.
3. **Approach Distance:** Keep `approach_height` large (≥5 cm) until validated.
4. **Watchdog:** If no joint state update for >1 s, emergency stop.
5. **Manual Override:** Provide keyboard command to stop arm immediately (separate from press logic).

---

## Testing Strategy

### Phase 1: Simulation (Gazebo)
- Launch with fake hardware.
- Test keyboard detector with synthetic camera feed (or test image).
- Validate IK solutions for all keys.
- Verify approach → press → retract trajectories.

### Phase 2: Simulation + Real Perception
- Use real camera feed into detector.
- Plan trajectories, but **do not execute** (log only).
- Visually verify poses match keyboard in RViz.

### Phase 3: Single Key, Real Execution
- Select safest key (middle, easily accessible).
- Execute approach only (stop before press).
- Verify end-effector reaches correct position.
- Then add press motion with low depth (1 mm, not 10 mm).
- Gradually increase depth.

### Phase 4: Multiple Keys
- Test all keys one at a time.
- Build confidence on keyboard geometry.
- Then test sequences (e.g., type a word).

---

## ROS 2 Param Example

```yaml
# keyboard_detector/config/keyboard_detector_params.yaml
keyboard_detector:
  camera_info_topic: /camera/camera_info
  image_topic: /camera/image_raw
  
  # Fiducial/ArUco detection
  marker_type: 'aruco_4x4_100'  # or 'apriltag' or 'contour'
  marker_ids: [0, 1, 2, 3]       # corner markers on keyboard
  
  # TF
  camera_frame: 'camera_link'
  keyboard_frame: 'keyboard_frame'
  
  # Smoothing
  pose_history_size: 5
  publish_rate: 10  # Hz

---

# keyboard_task_planner/config/keyboard_task_params.yaml
keyboard_task_planner:
  move_group_name: 'arm'
  ee_frame: 'gripper_claw_link'
  
  # Keyboard frame from perception
  keyboard_frame: 'keyboard_frame'
  
  # Key map (relative to keyboard frame origin, in meters)
  key_positions:
    'A': [0.015, -0.020, 0.002]
    'B': [0.045, -0.020, 0.002]
    'C': [0.075, -0.020, 0.002]
    # ... more rows
  
  # Press profile
  approach_height: 0.05
  press_depth: 0.010
  press_speed: 0.01
  dwell_time: 0.5
  
  ik_timeout: 2.0  # seconds
```

---

## Migration from Current Stack

**Your current FK/IK paths:**
- FK: Keyboard → JointJog → Servo → arm_controller.
- IK (if available): MoveIt Plan → arm_controller.

**New keyboard path:**
- Keyboard detector (vision) → Task planner (IK) → Executor (follow trajectory).

**Coexistence:**
- Keep FK keyboard mode for manual testing / jog mode.
- Add keyboard pressing as a separate mode.
- Toggle between FK and pressing via launch flags or runtime parameter.

---

## References

- IFRA-Cranfield `irb120_PoseEstimation`: https://github.com/IFRA-Cranfield/irb120_PoseEstimation
- MoveIt 2 motion planning: https://moveit.picknik.ai/humble/index.html
- OpenCV solvePnP: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
- ArUco markers: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

