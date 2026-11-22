"""
Launch the ArUco tracker on a camera image topic.

By default it listens to the front FFC camera published by
``luxonis_scripts/main.py --ros`` (/ffc/front/image_raw and /ffc/front/camera_info).

Examples::

    ros2 launch rover_bringup aruco_tracker.py
    ros2 launch rover_bringup aruco_tracker.py marker_dict:=5X5_100
    ros2 launch rover_bringup aruco_tracker.py cam_base_topic:=/test/camera/front/image_raw

Supported marker_dict values include 4X4_50/100/250/1000, 5X5_*, 6X6_*, 7X7_*,
ARUCO_ORIGINAL and APRILTAG_16h5/25h9/36h10/36h11 (without a DICT_ prefix).

The dictionary is read-only in the tracker: changing it means restarting this launch.
The tuned detector settings come from ros_aruco_opencv's config/aruco_tracker.yaml;
the arguments below override only what differs per camera or mission.
"""

from launch_ros.parameter_descriptions import ParameterValue
from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    cam_base_topic = sl.declare_arg(
        "cam_base_topic",
        default_value="/ffc/front/image_raw",
        description="Image topic; CameraInfo is read from the sibling camera_info topic",
    )
    # Competition markers (mission requirements): 4x4_50 tags on 20 x 20 cm faces with a
    # 1-cell white border, so 8 cells of 2.5 cm; the black square is 6 cells = 0.15 m.
    marker_dict = sl.declare_arg("marker_dict", default_value="4X4_50", description="ArUco dictionary, e.g. 4X4_50, 4X4_100, 5X5_50, 5X5_100")
    marker_size = sl.declare_arg("marker_size", default_value="0.15", description="Black-square side length of the marker, in meters")
    image_is_rectified = sl.declare_arg(
        "image_is_rectified",
        default_value="false",
        choices=["true", "false"],
        description="true only if the images are already undistorted",
    )

    sl.node(
        "ros_aruco_opencv",
        "aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[
            sl.params(package="ros_aruco_opencv", file="aruco_tracker.yaml"),
            {
                "cam_base_topic": cam_base_topic,
                "marker_dict": marker_dict,
                "marker_size": ParameterValue(marker_size, value_type=float),
                "image_is_rectified": ParameterValue(image_is_rectified, value_type=bool),
            },
        ],
    )

    return sl.launch_description()
