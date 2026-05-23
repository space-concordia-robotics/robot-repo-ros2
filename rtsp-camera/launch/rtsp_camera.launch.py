import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    rtsp_camera_params = os.path.join(
        get_package_share_directory('rtsp_camera'),
        'config',
        'rtsp_camera_parameters.test.yaml'
    )

    ld.add_action(
        Node(
            package='rtsp_camera',
            executable='rtsp_camera_node',
            parameters=[rtsp_camera_params],
        )
    )

    return ld
