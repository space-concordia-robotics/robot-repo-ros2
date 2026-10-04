from launch.event_handlers import OnProcessExit
from launch_util import SimpleLauncher, SomeSubstitutionsValueTypeDict


def generate_launch_description():
    sl = SimpleLauncher(mode="simulation", control="ros")

    mappings: SomeSubstitutionsValueTypeDict = {
        "mode": sl.mode_arg,
        "control": sl.control_arg,
    }

    sl.robot_state_publisher(package="swerve_simulation", file="swervebot.urdf", xacro_args=mappings, strip_comments=True)

    sl.gz_launch(package="swerve_simulation", file="flat.world")

    sl.declare_gazebo_axes(z="0.5")

    gz_spawn_ceres = sl.spawn_gz_model(name="swervebot", spawn_args=sl.gazebo_axes_args())

    controllers = sl.params(package="swerve_simulation", file="controllers.yaml")

    # steering & wheel spawners are temporary
    steering_spawner = sl.node(
        "controller_manager",
        "spawner",
        exec_name="steering_spawner",
        arguments=["steering_controller", "--param-file", controllers],
        add=False,
    )

    wheel_spawner = sl.node(
        "controller_manager",
        "spawner",
        exec_name="wheel_spawner",
        arguments=["wheel_controller", "--param-file", controllers],
        add=False,
    )

    joint_state_broadcaster_spawner = sl.node(
        "controller_manager",
        "spawner",
        exec_name="jsp_spawner",
        arguments=["joint_state_broadcaster"],
        add=False,
    )

    sl.add_event(
        OnProcessExit(
            target_action=gz_spawn_ceres,
            on_exit=[joint_state_broadcaster_spawner],
        ),
    )

    sl.add_event(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[steering_spawner],
        ),
    )

    sl.add_event(
        OnProcessExit(
            target_action=steering_spawner,
            on_exit=[wheel_spawner],
        ),
    )

    with sl.gz_bridge() as bridge:
        bridge.add_clock()
        bridge.add_joint_states()
        bridge.add_tf()
        # bridge.add_topic("/odom", "/diff_drive_base_controller/odom", BridgeDirection.GZ_TO_ROS, ros_msg="nav_msgs/Odometry")

    return sl.launch_description()
