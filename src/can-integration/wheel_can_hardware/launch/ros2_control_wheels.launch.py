"""Bring up ros2_control for the six-wheel CAN stack.

Passes robot_description into ros2_control_node (required). Without this,
controller_manager logs forever::

    Waiting for data on 'robot_description' topic to finish initialization

Usage::

    source install/setup.bash
    ros2 launch wheel_can_hardware ros2_control_wheels.launch.py can_interface:=can0

With odometry node (needs /joint_states from joint_state_broadcaster)::

    ros2 launch wheel_can_hardware ros2_control_wheels.launch.py \\
        can_interface:=can0 launch_odometry:=true

Send velocity commands (six wheels, rad/s)::

    ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \\
        "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

Arm control lives in the arm / MoveIt launch (arm_hardware.launch.py with
arm_can_hardware plugin). Safety stop/resume for wheels is can_safety_node,
not part of ros2_control.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('wheel_can_hardware')
    urdf_path = os.path.join(pkg_share, 'urdf', 'test_rover_wheels.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'wheel_controllers.yaml')

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN interface passed into WheelCanInterface URDF.',
    )
    launch_odom_arg = DeclareLaunchArgument(
        'launch_odometry',
        default_value='false',
        description='If true, also start wheel_odometry_node.',
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.15',
        description='Wheel radius (m) for wheel_odometry_node; autonomy diff_drive uses 0.15.',
    )
    track_width_arg = DeclareLaunchArgument(
        'track_width',
        default_value='1.25',
        description='Track width (m) for wheel_odometry_node; autonomy wheel_separation is 1.25.',
    )
    spawn_after_arg = DeclareLaunchArgument(
        'spawn_controller_delay',
        default_value='8.0',
        description=(
            'Seconds after ros2_control_node starts before spawning controllers '
            '(hardware plugins + CAN need time before list_controllers exists).'
        ),
    )

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                urdf_path,
                ' can_interface:=',
                LaunchConfiguration('can_interface'),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_yaml,
            {'robot_description': robot_description},
        ],
        output='screen',
    )

    # Give controller_manager time to load URDF plugins and open CAN before
    # list_controllers is advertised (matches arm_hardware.launch.py pattern).
    delayed_control_node = TimerAction(
        period=3.0,
        actions=[control_node],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )

    vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'velocity_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=LaunchConfiguration('spawn_controller_delay'),
                    actions=[jsb_spawner, vel_spawner],
                ),
            ],
        )
    )

    wheel_odometry_node = Node(
        package='wheel_can_hardware',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[
            {
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'track_width': LaunchConfiguration('track_width'),
                'publish_tf': True,
            }
        ],
        condition=IfCondition(LaunchConfiguration('launch_odometry')),
    )

    return LaunchDescription(
        [
            can_interface_arg,
            launch_odom_arg,
            wheel_radius_arg,
            track_width_arg,
            spawn_after_arg,
            robot_state_publisher,
            delayed_control_node,
            delayed_spawners,
            wheel_odometry_node,
        ]
    )
