from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    rtsp_camera_params = PathJoinSubstitution([FindPackageShare("rtsp_camera"), "config", "rtsp_camera_parameters.test.yaml"])

    ld.add_action(
        Node(
            package="rtsp_camera",
            executable="rtsp_camera_node",
            parameters=[rtsp_camera_params],
        ),
    )

    return ld
