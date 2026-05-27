"""Launch OAK-D spatial camera bridge under the ``oak`` ROS namespace."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'device_mxid',
                default_value='',
                description='DepthAI OAK-D device MxID or Ethernet IP.',
            ),
            DeclareLaunchArgument(
                'nn_archive_path',
                default_value='',
                description='Path to *.rvc2.tar.xz YOLO archive; empty uses package share.',
            ),
            Node(
                package='luxonis_scripts',
                executable='luxonis_camera_node',
                name='luxonis_camera',
                namespace='oak',
                output='screen',
                parameters=[
                    {
                        'ffc_device_mxid': '',
                        'oak_device_mxid': LaunchConfiguration('device_mxid'),
                        'nn_archive_path': LaunchConfiguration('nn_archive_path'),
                        'oak_fps': 15.0,
                        'confidence_threshold': 0.5,
                        'publish_rate_hz': 30.0,
                        'publish_depth': True,
                        'align_depth_to_rgb': True,
                        'detections_topic': '/detections',
                    }
                ],
            ),
        ]
    )
