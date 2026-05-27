# Luxonis Scripts

We have two Luxonis cameras: an OAK-FFC-4P (4 directional cameras) and an OAK-D-PRO (depth camera).

We are running [this depth model on the PRO](https://models.luxonis.com/luxonis/crestereo/4729a8bd-54df-467a-92ca-a8a5e70b52ab) and a custom trained YOLO model for object detection.

## ROS 2 bridge

From the **colcon workspace root** (e.g. `/ws` in the Jazzy dev container), source ROS first:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rover_msgs luxonis_scripts
source install/setup.bash
```

One node (`luxonis_camera_node`) drives the FFC and/or OAK-D pipelines. YOLO detections from every active camera are combined into a single `rover_msgs/ImageDetectionArray` on `/detections`.

Camera `frame_id` values match `rover-description` URDF links (`ffc_front_camera`, `ffc_rear_camera`, …, `forward_camera` for OAK-D RGB).

### Run

FFC only (namespace `ffc`):

```bash
ros2 launch luxonis_scripts ffc_quad.launch.py device_mxid:=10.240.0.69
```

OAK-D only (namespace `oak`):

```bash
ros2 launch luxonis_scripts spatial_oak.launch.py device_mxid:=10.240.0.67
```

Both cameras, all detections in one message:

```bash
ros2 launch luxonis_scripts luxonis_cameras.launch.py \
  ffc_device_mxid:=10.240.0.69 oak_device_mxid:=10.240.0.67
```

### Topics

Under namespace `ffc` (per active camera slug):

| Topic | Type |
| ----- | ---- |
| `/<slug>/image_raw` | `sensor_msgs/Image` |
| `/<slug>/camera_info` | `sensor_msgs/CameraInfo` |

Under namespace `oak`:

| Topic | Type |
| ----- | ---- |
| `/rgb/image_raw` | `sensor_msgs/Image` |
| `/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `/depth/image_raw` | `sensor_msgs/Image` (16UC1, mm) |
| `/depth/camera_info` | `sensor_msgs/CameraInfo` |

Global (absolute topic):

| Topic | Type |
| ----- | ---- |
| `/detections` | `rover_msgs/ImageDetectionArray` |

Quick check:

```bash
ros2 topic echo /detections --once
ros2 topic hz /oak/depth/image_raw
```
