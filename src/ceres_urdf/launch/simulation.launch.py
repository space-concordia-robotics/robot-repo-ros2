from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ceres_urdf = FindPackageShare('ceres_urdf')
    autonomy = FindPackageShare('autonomy')  # TODO: get rid of this
    controllers = PathJoinSubstitution([ceres_urdf, 'config', 'controllers.yaml'])

    mode = 'simulation'
    control = 'ros'

    # Declare the launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='The starting X coordinate of the robot'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='The starting Y coordinate of the robot'
        )
    )

    world = PathJoinSubstitution([autonomy, 'worlds', 'maze.out.world'])
    # world = ""

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

    ld.add_action(
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([autonomy, 'models'])
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r -s -v 4 ', world], 'on_exit_shutdown': 'true'}.items()
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': '-g -v4 '}.items()
        )
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-x', LaunchConfiguration('x_pose', default='0.0'),
            '-y', LaunchConfiguration('y_pose', default='0.0'),
            '-z', '0.5',  # robot seems to be ~0.49m tall
            '-R', '0.0',  # roll
            '-P', '0.0',  # pitch
            '-Y', '0.0',  # yaw
            '-name', 'ceres',
            '-topic', 'robot_description',
        ],
        output='screen'
    )

    ld.add_action(gz_spawn_entity)

    bridge_params = PathJoinSubstitution([ceres_urdf, 'params', 'gazebo_bridge_params.yaml'])

    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                ['config_file:=', bridge_params],
            ],
            parameters=[{
                "qos_overrides./tf_static.publisher.durability": "transient_local"
            }],
            output='screen',
        )
    )

    diff_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            '--param-file',
            controllers,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
    )

    ekf_params = PathJoinSubstitution([ceres_urdf, 'config', 'ekf.yaml'])

    # ld.add_action(
    #     Node(
    #         package='imu_processors',
    #         executable='imu_integrator_node',
    #         name='imu_integrator_node',
    #         output='screen',
    #         remappings=[
    #             ('/ceres/imu', '/imu'),
    #             ('/imu_integrated', '/ceres/imu_integrated'),
    #         ]
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package='imu_processors',
    #         executable='imu_integrator_node',
    #         name='lidar_imu_integrator_node',
    #         output='screen',
    #         remappings=[
    #             ('/ceres/lidar_imu', '/imu'),
    #             ('/imu_integrated', '/ceres/lidar_imu_integrated'),
    #         ]
    #     )
    # )

    ld.add_action(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': True}]
        )
    )

    return ld
