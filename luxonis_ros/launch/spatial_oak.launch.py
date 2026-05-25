"""Launch OAK-D spatial camera bridge (DepthAI + YOLO NN archive)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='luxonis_ros',
                executable='spatial_camera_node',
                name='oak_spatial',
                output='screen',
                parameters=[
                    {
                        'use_depthai': True,
                        'use_placeholder_image': False,
                        'device_mxid': '',
                        'nn_archive_path': '',
                        'fps': 15.0,
                        'confidence_threshold': 0.5,
                        'frame_id': 'oak_rgb_optical_frame',
                        'camera_name': 'oak_pro',
                        'publish_rate_hz': 30.0,
                        'publish_depth': True,
                        'align_depth_to_rgb': True,
                        'depth_image_topic': 'depth/image_raw',
                        'depth_camera_info_topic': 'depth/camera_info',
                    }
                ],
            )
        ]
    )
