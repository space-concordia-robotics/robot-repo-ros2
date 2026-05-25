# luxonis_ros

ROS 2 bridge for the OAK-D Pro and OAK-FFC-4P. Publishes images, `CameraInfo`,
and YOLO detections. The OAK-D node also publishes depth.

## Build

From your colcon workspace root:

```bash
colcon build --packages-select luxonis_ros
source install/setup.bash
```

## Run

OAK-D Pro (RGB + depth + 3D YOLO):

```bash
ros2 launch luxonis_ros spatial_oak.launch.py [device_mxid:=10.240.0.67]*optional
```

OAK-FFC-4P (4× RGB + 2D YOLO, PoE):
x.x.x
```bash
ros2 launch luxonis_ros ffc_quad.launch.py device_mxid:=10.240.0.69
```

Options:

- `active_cameras:=front`
- `fps:=4.0` (FFC default) / `fps:=15.0` (OAK-D defualt)
- `publish_depth:=false` (OAK-D) to turn off the depth topic

## Topics

OAK-D (`spatial_oak.launch.py`):

| Topic              | Type                              |
| ------------------ | --------------------------------- |
| `image_raw`        | `sensor_msgs/Image` (bgr8)        |
| `camera_info`      | `sensor_msgs/CameraInfo`          |
| `depth/image_raw`  | `sensor_msgs/Image` (16UC1, mm)   |
| `depth/camera_info`| `sensor_msgs/CameraInfo`          |
| `detections_3d`    | `vision_msgs/Detection3DArray`    |
| `detections_2d`    | `vision_msgs/Detection2DArray`    |

FFC (`ffc_quad.launch.py`), one set per active camera (`front`/`right`/`left`/`back`):

| Topic                          | Type                           |
| ------------------------------ | ------------------------------ |
| `ffc/<slug>/image_raw`         | `sensor_msgs/Image` (bgr8)     |
| `ffc/<slug>/camera_info`       | `sensor_msgs/CameraInfo`       |
| `ffc/<slug>/detections_2d`     | `vision_msgs/Detection2DArray` |

## Detection format

Each `Detection3D` on `detections_3d` gives you the YOLO class and the object
position in meters (RGB optical frame: +x right, +y down, +z forward):

- `id` → label (e.g. `"rock"`)
- `results[0].hypothesis.score` → confidence
- `bbox.center.position.{x,y,z}` → distance in meters

Quick check:

```bash
ros2 topic echo /detections_3d --once
ros2 topic hz /depth/image_raw
```
