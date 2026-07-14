"""Launch FFC + OAK-D on one node; all detections published together on ``/detections``."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('ffc_device_mxid', default_value=''),
            DeclareLaunchArgument('oak_device_mxid', default_value=''),
            DeclareLaunchArgument('nn_archive_path', default_value=''),
            DeclareLaunchArgument('ffc_fps', default_value='4.0'),
            DeclareLaunchArgument('oak_fps', default_value='15.0'),
            DeclareLaunchArgument('active_cameras', default_value='all'),
            DeclareLaunchArgument('depthai_watchdog', default_value='4500'),
            DeclareLaunchArgument('depthai_watchdog_initial_delay', default_value='60000'),
            DeclareLaunchArgument('depthai_bootup_timeout', default_value='60000'),
            SetEnvironmentVariable('DEPTHAI_WATCHDOG', LaunchConfiguration('depthai_watchdog')),
            SetEnvironmentVariable(
                'DEPTHAI_WATCHDOG_INITIAL_DELAY', LaunchConfiguration('depthai_watchdog_initial_delay')
            ),
            SetEnvironmentVariable('DEPTHAI_BOOTUP_TIMEOUT', LaunchConfiguration('depthai_bootup_timeout')),
            Node(
                package='luxonis_scripts',
                executable='luxonis_camera_node',
                name='luxonis_camera',
                output='screen',
                parameters=[
                    {
                        'ffc_device_mxid': LaunchConfiguration('ffc_device_mxid'),
                        'oak_device_mxid': LaunchConfiguration('oak_device_mxid'),
                        'nn_archive_path': LaunchConfiguration('nn_archive_path'),
                        'ffc_fps': LaunchConfiguration('ffc_fps'),
                        'oak_fps': LaunchConfiguration('oak_fps'),
                        'active_cameras': LaunchConfiguration('active_cameras'),
                        'detections_topic': '/detections',
                    }
                ],
            ),
        ]
    )
