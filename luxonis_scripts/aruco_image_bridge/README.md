# ArUco image bridge: Luxonis camera → ROS 2 → ArUco tracker

Publishes a low-rate raw image stream and matching calibration from the Luxonis
cameras, so the existing ArUco tracker (`aruco/ros-aruco-opencv`) can detect markers.

```text
Luxonis camera ──> DepthAI pipeline (main.py)
                     ├── 1080p H.264 ──> RTSP ──> base station        (existing, unchanged)
                     └── 1280x720 @ 5 FPS raw (--ros)
                              └──> sensor_msgs/Image + sensor_msgs/CameraInfo
                                        └──> ArUco tracker ──> /aruco_detections, TF
```

The raw image topics are meant to stay on the rover computer; they are not a
video feed for the base station.

## Status

- Verified without hardware: message building, calibration conversion, rate
  limiting, capture timestamps, error isolation (unit tests), and synthetic images
  → team ArUco tracker → correct marker ID, corners and distance for 4X4_100.
- **Pending hardware validation**: real DepthAI frames, real calibration,
  per-output frame rate on the device, Jetson performance. Nothing here is
  rover-ready until the lab checklist at the bottom is complete.
- **Depends on the separate `ros_aruco_opencv` fix PR** until it is merged: without it,
  the tracker does not build on Ubuntu 24.04 and publishes wrong corners. Merge that
  branch locally to test end to end (see "Build the ArUco tracker").

## Files

| File | Role |
|---|---|
| `luxonis_adapter.py` | Glue for `main.py`: options, default topics/frames, rate limit, capture timestamps, calibration caching. No ROS/DepthAI imports. |
| `calibration.py` | DepthAI calibration → CameraInfo fields, with plausibility checks. No ROS/DepthAI imports. |
| `ros_bridge.py` | Builds `Image`/`CameraInfo` (no `cv_bridge`) and owns the rclpy node and publishers. |
| `synthetic_frames.py` | Test fixtures: marker/blank frames and a **synthetic** calibration. |
| `publish_test_images.py` | Test tool: publishes synthetic frames through the same `ros_bridge` code as `main.py`. |
| `check_test_stream.py` | Test tool: independently checks the synthetic stream (not a detector). |
| `test/` | pytest unit tests; `test/fakes.py` holds the fake DepthAI objects they share. |
| `ros-constraints.txt` | pip constraints that keep the camera venv compatible with ROS 2 Jazzy. |

`main.py` changes are small: the `--ros*` options, one extra camera output per
selected camera in `build_pipeline()`, and a few lines in the session loop.

## Setup

Target: Ubuntu 24.04 + ROS 2 Jazzy (the Jetson's setup; WSL 2 also works for development).

### 1. Build the ArUco tracker

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install -y python3-colcon-mixin clang ninja-build ccache ros-jazzy-ros2-fmt-logger ros-jazzy-xacro python3-pytest
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
rosdep install --from-paths aruco cmake-utils --ignore-src -y
colcon build --packages-up-to ros_aruco_opencv launch_util --mixin compile-commands ninja clang ccache release
source install/setup.bash
```

`launch_util` is needed by the tracker launch file. Use the `release` mixin for now
(the whole list must be given, since command-line mixins replace the defaults): with
the repository default `rel-with-deb-info` the tracker aborts on start with
`undefined symbol: __ubsan_vptr_type_cache`. This is a known issue with the sanitizer
setup that is being fixed separately.

### 2. Camera virtualenv that can also import ROS

`--ros` needs `rclpy` from the ROS install, so the venv must see system packages,
and NumPy/OpenCV must stay compatible with ROS (see `ros-constraints.txt`):

```bash
cd luxonis_scripts
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt -c aruco_image_bridge/ros-constraints.txt
.venv/bin/python -c "import depthai, rclpy, sensor_msgs.msg, numpy; print('ok', numpy.__version__)"
```

The last line must print `ok 1.26.x`. An error mentioning NumPy 1.x/2.x means the
constraints were not applied.

## Tests (no camera needed)

From `luxonis_scripts/`, with ROS sourced:

```bash
python3 -m pytest -v aruco_image_bridge
```

All tests should pass. Without ROS sourced, the message tests are skipped rather than failing.

Lint and type-check (see the team's code standards), from the repository root:

```bash
ruff check luxonis_scripts/aruco_image_bridge rover-bringup/launch/aruco_tracker.py
ruff format --check luxonis_scripts/aruco_image_bridge
ty check --python luxonis_scripts/.venv-ros --extra-search-path luxonis_scripts \
    --extra-search-path /opt/ros/jazzy/lib/python3.12/site-packages \
    luxonis_scripts/aruco_image_bridge
```

`--python` points `ty` at the camera venv: Ubuntu's apt OpenCV is a compiled module
without type information, so without it `ty` reports `cv2` as unresolved.

Newer ruff versions also report `CPY001` (missing copyright notice) on every file,
including existing team code; the repository does not use copyright headers.

## Synthetic end-to-end check (no camera needed)

Use three terminals, each with ROS and the workspace sourced
(`source /opt/ros/jazzy/setup.bash && source <repo>/install/setup.bash`).
Optionally isolate the test with `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
in all three (Jazzy replacement for `ROS_LOCALHOST_ONLY`; never set it on the rover).

```bash
# Terminal 1 (from luxonis_scripts/): synthetic camera
python3 -m aruco_image_bridge.publish_test_images --dictionary 4X4_50

# Terminal 2 (from the repository root): the team tracker on the test topic
ros2 launch rover-bringup/launch/aruco_tracker.py \
    cam_base_topic:=/test/camera/front/image_raw marker_dict:=4X4_50 marker_size:=0.15

# Terminal 3: evidence
ros2 topic echo /aruco_detections --once
ros2 topic hz /test/camera/front/camera_info
ros2 topic info /test/camera/front/image_raw --verbose
(cd luxonis_scripts && python3 -m aruco_image_bridge.check_test_stream --dictionary 4X4_50)
```

Expected from `/aruco_detections` (repeat if you catch a blank frame, `markers: []`):
`id: 7`; corners (540, 260), (739, 260), (739, 459), (540, 459); `pose.position.z`
≈ 0.603 (800 × 0.15 / 199 px); x, y ≈ 0; orientation w ≈ 1. The checker prints `PASS`.

To test grayscale output, add `--encoding mono8` to both the publisher and the checker;
the tracker needs no change.

Repeat with `4X4_100`, `5X5_50` and `5X5_100` (restart terminals 1 and 2 with the new dictionary;
the tracker's dictionary cannot change while it runs). For the mismatch test, publish a 5X5
dictionary while the tracker runs `4X4_50`: it must give `markers: []` on every frame. (A
`4X4_100` tracker would still detect `4X4_50` markers, since those are its first 50.)

## Running on the rover

```bash
cd luxonis_scripts
source /opt/ros/jazzy/setup.bash
./run.sh --mode ffc_front --ros                # front FFC camera, 1280x720 @ 5 FPS
./run.sh --mode ffc_all --ros --ros-cameras FRONT,BACK
```

Then start the tracker (default topic `/ffc/front/image_raw`):

```bash
ros2 launch rover_bringup aruco_tracker.py marker_dict:=4X4_50
```

At start-up the camera script logs, per camera, the resolution, rate, topics and
frame ID, then the calibration it will use and where it came from. Without `--ros`,
`main.py` behaves exactly as before and does not need ROS installed.

| Option | Default | Meaning |
|---|---|---|
| `--ros` | off | Enable the ROS image output |
| `--ros-cameras` | `FRONT` | Any of `FRONT,RIGHT,LEFT,BACK,RGB`; only cameras in the running mode publish |
| `--ros-fps` | `5` | Output rate (device-side, with a host-side guard) |
| `--ros-size` | `1280x720` | Output resolution |
| `--ros-encoding` | `bgr8` | `bgr8`, `rgb8` or `mono8`; `mono8` is a third of the size |

| Camera | Image / CameraInfo topics | `frame_id` (placeholder) |
|---|---|---|
| FRONT | `/ffc/front/image_raw`, `/ffc/front/camera_info` | `ffc_front_camera_optical_frame` |
| RIGHT | `/ffc/right/...` | `ffc_right_camera_optical_frame` |
| LEFT | `/ffc/left/...` | `ffc_left_camera_optical_frame` |
| BACK | `/ffc/rear/...` | `ffc_rear_camera_optical_frame` |
| RGB (OAK-D, `oakd_rgb` mode) | `/oakd/rgb/...` | `oakd_rgb_camera_optical_frame` |

Topic names follow the simulated cameras in `rover-description/urdf/sensors/ffc-module.urdf`
(the back camera is called `rear` there).

## Design notes

- **One process owns the camera.** Output is added inside `main.py`'s existing pipeline
  instead of a second program opening the device.
- **RTSP is protected.** The ArUco output uses a 2-frame non-blocking queue, and every
  ROS or calibration error is caught and logged once per distinct message; RTSP keeps running.
- **NV12 on the link.** Frames cross the PoE link as NV12 (half the size of BGR) and are
  converted with `getCvFrame()` on the host.
- **Calibration source.** Preferred: the frame's own `getTransformation()`, whose intrinsics
  already include the crop/scale the camera applied. Fallback: the device EEPROM,
  rescaled with `getCameraIntrinsics(socket, width, height)`. Uncalibrated frames report
  an identity matrix, which is rejected. Only the Perspective (pinhole) model is accepted,
  because the tracker passes `CameraInfo.d` straight to OpenCV `solvePnP`. Its 8 leading
  coefficients are published as `rational_polynomial`; a warning is logged if the
  thin-prism/tilt terms are non-zero.
- **Timestamps.** Image and CameraInfo share one stamp: the capture time, computed as
  ROS now minus the frame's age on DepthAI's host-synchronised clock. Ages outside
  0–1 s fall back to the receive time.
- **No `cv_bridge`.** The Image is built directly from the NumPy buffer, so the venv's
  OpenCV and ROS's OpenCV never need to agree.
- **Encodings.** `ros_bridge.image_msg` supports `mono8`, `mono16`, `bgr8`, `rgb8`,
  `bgra8` and `rgba8`, inferring one from the array when none is given. The camera path
  offers `bgr8` (default), `rgb8` and `mono8` (`--ros-encoding`): all three are accepted
  by the tracker, and `mono8` is a third of the size while ArUco detection works on
  grayscale anyway.

## Competition specs (from Mathieu and the mission requirements)

- **Markers:** `4X4_50` tags on 20 x 20 cm faces with a 1-cell white border, so cells are
  2.5 cm and the black square (`marker_size`) is 6 cells = **0.15 m**. Three-sided posts,
  markers 0.5 to 1.5 m off the ground.
- **Range:** detection from at least 3 m, ideally 5 m or more.
- **Distance accuracy:** about +/- 30 cm.
- `4X4_50` markers are the first 50 of `4X4_100`, so a tracker set to `4X4_100` also detects
  them; a dictionary-mismatch test must therefore use a 5X5 marker.
- **Resolution vs range (estimate):** with the IMX378W's 95 degree HFOV, fx is about 586 px at
  1280 px width and 880 px at 1920 px. A 0.15 m marker is then about 29 px at 3 m and 18 px at
  5 m in 720p, versus 44 px and 26 px in 1080p. In a synthetic OpenCV test, markers of 18 px and
  above decoded every time and 15 px missed 20% of the time, so 5 m at 720p has little margin,
  and none when a face is seen at an angle. `--ros-size 1920x1080 --ros-encoding mono8` is still
  smaller per frame than 720p `bgr8` (2.1 MB vs 2.8 MB); compare both in the lab.

## Known limitations and open questions

- **Hardware validation pending** (see checklist below).
- **Optical frames.** The URDF defines `ffc_<side>_camera` links in body convention
  (x forward) but no optical frames (z forward, x right, y down), which camera images
  and tracker poses use. The `*_optical_frame` names above are placeholders; the URDF
  needs a fixed child per camera, rotated `rpy="-pi/2 0 -pi/2"`. To confirm with the team.
- **Marker size.** Confirmed as 0.15 m (see above); the tracker package's own config still
  says 0.0742 m, which the launch file overrides.
- **Dictionary switching.** `marker_dict` is read-only in the tracker, so switching
  means restarting it (or running one tracker per dictionary). Whether live switching is
  required is an open question for the team.
- **Per-output frame rate.** `requestOutput(..., fps=5)` is DepthAI v3's API for this;
  that the device really sends 5 FPS alongside the 30 FPS encoder output must be confirmed
  on the camera (`ros2 topic hz`, and the `bw` command in `main.py`).

## Lab checklist (before calling it rover-ready)

- [ ] `./run.sh --mode ffc_front --ros` starts; the calibration log line shows plausible
      fx/fy/cx/cy and `source=frame transformation`.
- [ ] `ros2 topic hz /ffc/front/image_raw` ≈ 5 Hz for several minutes; RTSP still smooth.
- [ ] Image and CameraInfo stamps and frame IDs match (`ros2 topic echo ... --once`).
- [ ] A printed marker of each dictionary (4X4_50 for competition; also 4X4_100, 5X5_50, 5X5_100) gives the right ID.
- [ ] A marker at a measured distance (e.g. 1.00 m and 3.00 m) gives `pose.position.z`
      within a few percent, which validates the calibration and the marker size.
- [ ] Autonomy receives the detections it expects.
