"""Bring up arm (ArmCanInterface) + six-wheel drive (WheelCanInterface) together.

Both hardware plugins share the same SocketCAN socket via getSharedCanController().
Do NOT also start a legacy can_node on the same CAN interface.

Usage::

    source install/setup.bash

    # Minimal (CAN, arm + wheels, no odometry or safety)
    ros2 launch wheel_can_hardware rover_arm_and_wheels.launch.py can_interface:=can0

    # With wheel odometry
    ros2 launch wheel_can_hardware rover_arm_and_wheels.launch.py \\
        can_interface:=can0 launch_odometry:=true

    # With can_safety_node (F1/F2 defaults; verify with joy_button_probe first)
    ros2 launch wheel_can_hardware rover_arm_and_wheels.launch.py \\
        can_interface:=can0 launch_safety:=true \\
        wheel_force_stop_button:=26 wheel_resume_button:=27

See docs/testing/09-rover-arm-and-wheels-combined.md for the full bring-up checklist.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_wheel = get_package_share_directory('wheel_can_hardware')
    pkg_arm   = get_package_share_directory('rover_arm_moveit_config')

    urdf_path       = os.path.join(pkg_wheel, 'urdf', 'rover_arm_and_wheels.urdf.xacro')
    controllers_yaml = os.path.join(pkg_wheel, 'config', 'rover_arm_and_wheels_controllers.yaml')
    initial_pos_file = os.path.join(pkg_arm, 'config', 'initial_positions.yaml')

    # ── Launch arguments ────────────────────────────────────────────────────────
    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN interface used by both ArmCanInterface and WheelCanInterface.',
    )
    launch_odom_arg = DeclareLaunchArgument(
        'launch_odometry', default_value='false',
        description='If true, start wheel_odometry_node (needs joint_state_broadcaster active).',
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.15',
        description='Wheel radius (m) for wheel_odometry_node.',
    )
    track_width_arg = DeclareLaunchArgument(
        'track_width', default_value='1.25',
        description='Track width (m) for wheel_odometry_node.',
    )
    launch_safety_arg = DeclareLaunchArgument(
        'launch_safety', default_value='false',
        description='If true, start can_safety_node for wheel force-stop / resume.',
    )
    fs_button_arg = DeclareLaunchArgument(
        'wheel_force_stop_button', default_value='26',
        description='Joy button index for wheel force stop. Default = VKBButtonLayout.F1 (26).',
    )
    rs_button_arg = DeclareLaunchArgument(
        'wheel_resume_button', default_value='27',
        description='Joy button index for wheel resume. Default = VKBButtonLayout.F2 (27).',
    )
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_controller_delay', default_value='8.0',
        description='Seconds to wait after ros2_control_node before spawning controllers.',
    )

    # ── Robot description (combined arm + wheels URDF) ───────────────────────
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' can_interface:=', LaunchConfiguration('can_interface'),
            ' initial_positions_file:=', initial_pos_file,
        ]),
        value_type=str,
    )

    # ── Nodes ────────────────────────────────────────────────────────────────
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

    # Delay start so driver libraries and CAN are initialised before spawners try
    delayed_control_node = TimerAction(period=3.0, actions=[control_node])

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )
    vel_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'velocity_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=LaunchConfiguration('spawn_controller_delay'),
                    actions=[jsb_spawner, arm_spawner, vel_spawner],
                ),
            ],
        )
    )

    wheel_odometry_node = Node(
        package='wheel_can_hardware',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'track_width': LaunchConfiguration('track_width'),
            'publish_tf': True,
        }],
        condition=IfCondition(LaunchConfiguration('launch_odometry')),
    )

    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('can_safety_node'),
                'launch', 'safety.launch.py',
            )
        ),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'wheel_force_stop_button': LaunchConfiguration('wheel_force_stop_button'),
            'wheel_resume_button': LaunchConfiguration('wheel_resume_button'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_safety')),
    )

    return LaunchDescription([
        can_interface_arg,
        launch_odom_arg,
        wheel_radius_arg,
        track_width_arg,
        launch_safety_arg,
        fs_button_arg,
        rs_button_arg,
        spawn_delay_arg,
        robot_state_publisher,
        delayed_control_node,
        delayed_spawners,
        wheel_odometry_node,
        safety_launch,
    ])
