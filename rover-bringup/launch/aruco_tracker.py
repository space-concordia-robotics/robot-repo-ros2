"""
Launch the ArUco tracker on a camera image topic.

By default it listens to the front FFC camera published by
``luxonis_scripts/main.py --ros`` (/ffc/front/image_raw and /ffc/front/camera_info).

Examples::

    ros2 launch rover_bringup aruco_tracker.py
    ros2 launch rover_bringup aruco_tracker.py marker_dict:=5X5_50
    ros2 launch rover_bringup aruco_tracker.py cam_base_topic:=/test/camera/front/image_raw

Supported marker_dict values include 4X4_50/100/250/1000, 5X5_*, 6X6_*, 7X7_*,
ARUCO_ORIGINAL and APRILTAG_16h5/25h9/36h10/36h11 (without a DICT_ prefix).

The dictionary is read-only in the tracker: changing it means restarting this launch.
The tuned detector settings come from ros_aruco_opencv's config/aruco_tracker.yaml;
the arguments below override only what differs per camera or mission.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Tracker node with the package defaults plus per-launch overrides."""
    arguments = [
        DeclareLaunchArgument(
            "cam_base_topic",
            default_value="/ffc/front/image_raw",
            description="Image topic; CameraInfo is read from the sibling camera_info topic",
        ),
        DeclareLaunchArgument("marker_dict", default_value="4X4_100", description="ArUco dictionary, e.g. 4X4_100, 5X5_50, 5X5_100"),
        # TODO: confirm the physical competition marker size with the team (tracker config says 0.0742, mission config 0.15).
        DeclareLaunchArgument("marker_size", default_value="0.15", description="Black-square side length of the marker, in meters"),
        DeclareLaunchArgument("image_is_rectified", default_value="false", description="true only if the images are already undistorted"),
    ]

    tracker = Node(
        package="ros_aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("ros_aruco_opencv"), "config", "aruco_tracker.yaml"]),
            {
                "cam_base_topic": LaunchConfiguration("cam_base_topic"),
                "marker_dict": LaunchConfiguration("marker_dict"),
                "marker_size": ParameterValue(LaunchConfiguration("marker_size"), value_type=float),
                "image_is_rectified": ParameterValue(LaunchConfiguration("image_is_rectified"), value_type=bool),
            },
        ],
    )

    return LaunchDescription([*arguments, tracker])
