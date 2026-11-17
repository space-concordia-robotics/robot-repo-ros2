# Luxonis Scripts

We have two Luxonis cameras, and OAK-FFC-4P (4 directional cameras) and an OAK-D-PRO (depth camera)

The `oakd_yolo` mode runs a custom-trained YOLO model (`best_190_Epoch.rvc2.tar.xz`) with on-device stereo depth for object detection.

## Setup

The current setup requires DepthAI v3 (`requirements.txt` pins `depthai==3.6.1`) and python3 (`sudo apt install python3`). On the Jetson, PyGObject is built from source and needs some apt packages first; see [Camera virtualenv](aruco_image_bridge/README.md#setup-on-the-jetson).

```
python3 -m venv venv
source venv/bin/activate   # venv/bin/activate.fish in fish

# Installs library and requirements
pip install -r requirements.txt
```

### Set usb rules

```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### How to run the program

To run the camera:
```
python3 main.py --mode [PipelineType] -d [MXID or IP]
```

IPs* OAK-D Pro `10.240.0.67`, FFC PoE `10.240.0.69`.

**Modes**

| mode | bitrate | fps | RTSP paths |
|------|---------|-----|------------|
| ffc_all | 2 Mbps | 30 | `/FRONT` `/RIGHT` `/LEFT` `/BACK` |
| ffc_front / ffc_back / ffc_right / ffc_left | 7 Mbps | 30 | one of the above |
| ffc (REPL only: `mode ffc front,right`) | 4 Mbps | 30 | the chosen cameras |
| oakd_rgb | 7 Mbps | 30 | `/RGB` |
| oakd_yolo | 4 Mbps | 15 | `/RGB` + REPL `detections` |

The FFC and the OAK-D run in separate sessions, so one FFC mode and one OAK-D mode can run at the same time (start the second with `mode` at the prompt).

## ROS image output (`--ros`)

`--ros` also publishes a low-rate raw image and `CameraInfo` for the ArUco tracker (see `aruco_image_bridge/`). It is off by default; without it `main.py` does not import ROS.

For the full procedure (camera, tracker, viewing detections) and troubleshooting, see [Running ArUco detection on the rover](aruco_image_bridge/README.md#running-aruco-detection-on-the-rover). On the PoE cameras use `--ros-encoding mono8`; raw `bgr8` frames use most of the link.

```
python3 main.py --mode oakd_rgb -d 10.240.0.67 --ros --ros-cameras RGB --ros-encoding mono8 --ros-size 640x480
```

`--ros` needs ROS 2 sourced (`source /opt/ros/jazzy/setup.bash`) in the same terminal as the virtualenv.

| Option | Default | Meaning |
|---|---|---|
| `--ros` | off | Enable the ROS image output |
| `--ros-cameras` | `FRONT` | Any of `FRONT,RIGHT,LEFT,BACK,RGB`; only cameras in the running mode publish |
| `--ros-fps` | `5` | Output rate |
| `--ros-size` | `1280x720` | Output resolution |
| `--ros-encoding` | `bgr8` | `bgr8`, `rgb8` or `mono8` (`mono8` is a third of the size) |
| `--ros-drop` | `host` | Where frames above `--ros-fps` are dropped: `host` (Jetson) or `camera` (keeps them off the PoE link; not yet validated on the rover) |

| Camera | Image / CameraInfo topics | `frame_id` |
|---|---|---|
| FRONT | `/ffc/front/image_raw`, `/ffc/front/camera_info` | `ffc_front_camera_optical_frame` |
| RIGHT | `/ffc/right/...` | `ffc_right_camera_optical_frame` |
| LEFT | `/ffc/left/...` | `ffc_left_camera_optical_frame` |
| BACK | `/ffc/rear/...` | `ffc_rear_camera_optical_frame` |
| RGB (`oakd_rgb` mode) | `/oakd/rgb/...` | `oakd_rgb_camera_optical_frame` |

## REPL commands

`main.py` reads commands at its `>` prompt:

| Command | What it does |
|---|---|
| `mode <name> [device]` | Start a mode; restarts that camera's session if it is already running |
| `mode ffc <cams> [device]` | FFC with a chosen set of cameras, e.g. `front,right` |
| `stop [ffc\|oakd]` | Stop one session, or both without an argument |
| `status` | Show which mode each session runs |
| `bw` | Bandwidth and FPS per stream; `no traffic` means the camera stopped |
| `watch [interval]` | `bw` repeatedly (Ctrl+C to stop) |
| `detections` / `watch_detections [interval]` | Latest YOLO detections (`oakd_yolo` only) |
| `panorama [filename]` | Stitch a panorama from the running FFC streams |
| `help`, `quit` | `quit` (or `exit`, `q`) shuts down |

### Panorama

`panorama [filename]` needs all four FFC cameras streaming (`mode ffc_all`).
It grabs one frame per camera from the local RTSP streams with `ffmpeg`,
stitches them with a cardinal-direction compass overlay, and saves the
result in `panoramas/`. The streams keep running. The REPL command does
not read the OAK-D IMU, so the compass uses `yaw=0`.

To get the compass from the OAK-D IMU, stop the streams and run the
capture by itself; it opens both cameras directly:

```
python3 panorama.py                              # uses default MxIds
python3 panorama.py --ffc 10.240.0.69 \
                    --oakd 10.240.0.67 \
                    --filename garden_north
```

If the OAK-D IMU isn't reachable (or with `--no-imu`) the panorama still builds with `yaw=0`.


## main.py examples (To launch all cameras)

  python3 main.py --mode oakd_rgb -d 10.240.0.67
```
  > mode ffc front,right 10.240.0.69      # OAK keeps running; FFC starts with 2 cams
  > mode ffc front,right,left 10.240.0.69 # FFC restarts with 3 cams; OAK untouched
  > mode ffc_all 10.240.0.69              # FFC restarts with all 4
  > stop ffc                              # OAK keeps running
  > stop                                  # stops both
```
  Valid ffc cameras: front, right, left, back (case-insensitive). Without a device argument, `mode` uses `-d` if it was given, even for the other camera, otherwise the built-in MxId for that camera. So when `main.py` was started with `-d`, give the device explicitly when switching to the other camera, as above.

## launch_cameras.sh examples

```
  bash launch_cameras.sh ffc front right       # 2 ffplay windows
  bash launch_cameras.sh ffc front left back   # 3 windows
  bash launch_cameras.sh ffc_front             # 1 (alias still works)
  bash launch_cameras.sh oakd_rgb              # OAK
  bash launch_cameras.sh all                    # FFC 4-up + OAK RGB
```