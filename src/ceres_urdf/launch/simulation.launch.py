from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ceres_urdf = FindPackageShare('ceres_urdf')

    mode = 'simulation'
    control = 'ros'

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([ceres_urdf, 'launch', 'rsp.launch.py']),
            launch_arguments={'mode': mode, 'control': control}.items()
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([ceres_urdf, 'launch', 'joystick.launch.py']),
            launch_arguments={'mode': mode}.items()
        )
    )

    twist_mux_params = PathJoinSubstitution([ceres_urdf, 'config', 'twist_mux.yml'])
    ld.add_action(
        Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel_unstamped')]
        )
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
            # launch_arguments={'extra_gazebo_args': f'--ros-args --params-file {gazebo_params_file}'}.items()
            launch_arguments={'extra_gazebo_args': f'--verbose'}.items()
        )
    )

    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'ceres'],
            output='screen'
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_base_controller"],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        )
    )

    return ld
