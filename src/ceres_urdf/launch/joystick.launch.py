from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_util import add_mode_argument, sim_time_parameter


def generate_launch_description():
    ld = LaunchDescription()

    add_mode_argument(ld)

    ceres_urdf = FindPackageShare('ceres_urdf')
    joy_params = PathJoinSubstitution([ceres_urdf, 'config', 'joystick.yml'])

    ld.add_action(
        Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': sim_time_parameter}],
        )
    )

    ld.add_action(
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': sim_time_parameter}],
            remappings=[('/cmd_vel', '/cmd_vel_joy')]
        )
    )

    ld.add_action(
        Node(
            package='twist_stamper',
            executable='twist_stamper',
            parameters=[{'use_sim_time': sim_time_parameter}],
            remappings=[('/cmd_vel_in', '/diff_drive_base_controller/cmd_vel_unstamped'),
                        ('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')]
        )
    )

    return ld
