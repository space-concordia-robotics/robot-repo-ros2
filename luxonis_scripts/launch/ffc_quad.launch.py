"""Launch OAK-FFC-4P quad bridge under the ``ffc`` ROS namespace."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'device_mxid',
                default_value='',
                description='DepthAI FFC device MxID or Ethernet IP (e.g. 10.240.0.170).',
            ),
            DeclareLaunchArgument(
                'nn_archive_path',
                default_value='',
                description='Path to *.rvc2.tar.xz YOLO archive; empty uses LUXONIS_NN_ARCHIVE / package share.',
            ),
            DeclareLaunchArgument(
                'fps',
                default_value='4.0',
                description='Camera / NN FPS per branch; four nets over PoE are heavy.',
            ),
            DeclareLaunchArgument(
                'publish_rate_hz',
                default_value='15.0',
                description='ROS timer rate for draining queues and publishing.',
            ),
            DeclareLaunchArgument(
                'active_cameras',
                default_value='all',
                description='all or comma-separated subset: front,right,left,back',
            ),
            DeclareLaunchArgument(
                'depthai_watchdog',
                default_value='4500',
                description='Ms; Luxonis Ethernet keepalive budget.',
            ),
            DeclareLaunchArgument(
                'depthai_watchdog_initial_delay',
                default_value='60000',
                description='Ms; initial watchdog delay over Ethernet.',
            ),
            DeclareLaunchArgument(
                'depthai_bootup_timeout',
                default_value='60000',
                description='Ms; device boot/connect timeout.',
            ),
            SetEnvironmentVariable('DEPTHAI_WATCHDOG', LaunchConfiguration('depthai_watchdog')),
            SetEnvironmentVariable(
                'DEPTHAI_WATCHDOG_INITIAL_DELAY', LaunchConfiguration('depthai_watchdog_initial_delay')
            ),
            SetEnvironmentVariable('DEPTHAI_BOOTUP_TIMEOUT', LaunchConfiguration('depthai_bootup_timeout')),
            Node(
                package='luxonis_scripts',
                executable='luxonis_camera_node',
                name='luxonis_camera',
                namespace='ffc',
                output='screen',
                parameters=[
                    {
                        'ffc_device_mxid': LaunchConfiguration('device_mxid'),
                        'oak_device_mxid': '',
                        'nn_archive_path': LaunchConfiguration('nn_archive_path'),
                        'ffc_fps': LaunchConfiguration('fps'),
                        'confidence_threshold': 0.5,
                        'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                        'active_cameras': LaunchConfiguration('active_cameras'),
                        'detections_topic': '/detections',
                    }
                ],
            ),
        ]
    )
