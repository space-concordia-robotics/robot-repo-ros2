"""
ROS 2 launch file for the Swerve Drive Gazebo simulation (`swerve_simulation`).

Starts:
1. `robot_state_publisher` with the processed `swervebot.urdf` Xacro model.
2. Gazebo Sim (`gz sim`) with the `flat.world` environment.
3. Entity spawner to spawn `swervebot` slightly above the ground plane (z = 0.5 m).
4. `joint_state_broadcaster` and `swerve_controller` spawners chained via process exit events.
5. `ros_gz_bridge` for `/clock`, `/joint_states`, and `/tf`.
"""

from launch.event_handlers import OnProcessExit

from launch_util import SimpleLauncher, SomeSubstitutionsValueTypeDict


def generate_launch_description():
    # Initialize Space Concordia's SimpleLauncher helper configured for Gazebo simulation with ros2_control
    sl = SimpleLauncher(mode="simulation", control="ros")

    mappings: SomeSubstitutionsValueTypeDict = {
        "mode": sl.mode_arg,
        "control": sl.control_arg,
    }

    # Process swervebot.urdf with xacro (stripping comments for gz_ros2_control compatibility) and publish /robot_description
    sl.robot_state_publisher(package="swerve_simulation", file="swervebot.urdf", xacro_args=mappings, strip_comments=True)

    # Launch Gazebo Sim with the flat ground plane world
    sl.gz_launch(package="swerve_simulation", file="flat.world")

    # Declare initial spawn pose arguments (spawn at z=0.5m so wheels don't clip into the ground)
    sl.declare_gazebo_axes(z="0.5")

    # Spawn the swervebot entity in Gazebo from /robot_description
    gz_spawn_ceres = sl.spawn_gz_model(name="swervebot", spawn_args=sl.gazebo_axes_args())

    # Locate the ros2_control controller configuration YAML file
    controllers = sl.params(package="swerve_simulation", file="controllers.yaml")

    # Define spawner node for the custom swerve_controller (deferred until joint_state_broadcaster exits)
    # steering & wheel spawners are temporary
    swerve_spawner = sl.node(
        "controller_manager",
        "spawner",
        exec_name="swerve_spawner",
        arguments=["swerve_controller", "--param-file", controllers],
        add=False,
    )

    # Define spawner node for joint_state_broadcaster (deferred until robot model finishes spawning)
    joint_state_broadcaster_spawner = sl.node(
        "controller_manager",
        "spawner",
        exec_name="jsp_spawner",
        arguments=["joint_state_broadcaster"],
        add=False,
    )

    # Event 1: Once swervebot is spawned in Gazebo, activate joint_state_broadcaster
    sl.add_event(
        OnProcessExit(
            target_action=gz_spawn_ceres,
            on_exit=[joint_state_broadcaster_spawner],
        ),
    )

    # Event 2: Once joint_state_broadcaster is active, activate swerve_controller
    sl.add_event(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_spawner],
        ),
    )

    # Bridge simulation clock, joint states, and transforms between Gazebo Transport and ROS 2
    with sl.gz_bridge() as bridge:
        bridge.add_clock()
        bridge.add_joint_states()
        bridge.add_tf()

    return sl.launch_description()
