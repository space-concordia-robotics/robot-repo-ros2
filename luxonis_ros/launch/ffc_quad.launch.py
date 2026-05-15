"""Launch OAK-FFC-4P quad bridge: four RGB streams + on-device YOLO (lower FPS may be needed for bandwidth).

Pass the camera over Ethernet, e.g.::

  ros2 launch luxonis_ros ffc_quad.launch.py device_mxid:=10.240.0.170

PoE over high-latency or congested routing often hits ``ping was missed`` / X_LINK_ERROR
with four networks; lower ``fps``, use ``active_cameras:=front`` to test one branch, or
relax DepthAI watchdog env (also set via node parameters depthai_watchdog_ms, etc.).
"""

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
                description='DepthAI device MxID or Ethernet IP (use IP for PoE, e.g. 10.240.0.170). '
                'Empty uses getAllAvailableDevices() (often fails inside Docker/LAN unless discovery works).',
            ),
            DeclareLaunchArgument(
                'nn_archive_path',
                default_value='',
                description='Path to *.rvc2.tar.xz YOLO archive; empty uses LUXONIS_NN_ARCHIVE / package share.',
            ),
            DeclareLaunchArgument(
                'fps',
                default_value='4.0',
                description='Camera / NN FPS per branch; four nets over PoE are heavy—raise only after stable.',
            ),
            DeclareLaunchArgument(
                'publish_rate_hz',
                default_value='15.0',
                description='ROS timer rate for draining queues and publishing images.',
            ),
            DeclareLaunchArgument(
                'active_cameras',
                default_value='all',
                description='all or comma-separated subset: front,right,left,back',
            ),
            DeclareLaunchArgument(
                'topic_namespace',
                default_value='ffc',
                description='ROS topic prefix: <namespace>/<front|right|left|back>/....',
            ),
            DeclareLaunchArgument(
                'frame_prefix',
                default_value='ffc',
                description='TF frame_id pattern: <prefix>_<camera>_optical_frame.',
            ),
            DeclareLaunchArgument(
                'depthai_watchdog',
                default_value='4500',
                description='Ms; Luxonis Ethernet keepalive budget (exported before node starts).',
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
                package='luxonis_ros',
                executable='ffc_camera_node',
                name='ffc_quad',
                output='screen',
                parameters=[
                    {
                        'device_mxid': LaunchConfiguration('device_mxid'),
                        'nn_archive_path': LaunchConfiguration('nn_archive_path'),
                        'fps': LaunchConfiguration('fps'),
                        'confidence_threshold': 0.5,
                        'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                        'active_cameras': LaunchConfiguration('active_cameras'),
                        'topic_namespace': LaunchConfiguration('topic_namespace'),
                        'frame_prefix': LaunchConfiguration('frame_prefix'),
                    }
                ],
            ),
        ]
    )
