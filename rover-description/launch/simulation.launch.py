from launch.actions import AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_util import SimpleLauncher, BridgeDirection


def generate_launch_description():
    sl = SimpleLauncher(mode="simulation", control="ros")

    twist_mux_params = sl.params(package="rover_description", file="twist_mux.yml")
    sl.node(
        package="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/diff_drive_base_controller/cmd_vel_unstamped")],
    )

    sl.add_action(
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("autonomy"), "models"])
        )
    )

    # robot seems to be ~0.49m tall
    sl.declare_gazebo_axes(z="0.5")

    sl.gz_launch(package="autonomy", file="maze.world")

    gz_spawn_ceres = sl.spawn_gz_model(name="ceres", spawn_args=sl.gazebo_axes_args())

    controllers = sl.params(package="rover_description", file="controllers.yaml")

    diff_drive_spawner = sl.node(
        "controller_manager",
        "spawner",
        exec_name="diff_drive_spawner",
        arguments=["diff_drive_base_controller", "--param-file", controllers],
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
        )
    )

    sl.add_event(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    with sl.gz_bridge() as bridge:
        bridge.add_clock()
        bridge.add_joint_states()
        bridge.add_tf()
        bridge.add_topic("/odom", "/diff_drive_base_controller/odom", BridgeDirection.GZ_TO_ROS, ros_msg="nav_msgs/Odometry")
        bridge.add_topic("/cmd_vel", "rover/cmd_vel", BridgeDirection.ROS_TO_GZ, ros_msg="geometry_msgs/Twist")
        bridge.add_topic("/imu/data", "rover/imu/data", BridgeDirection.GZ_TO_ROS, ros_msg="sensor_msgs/Imu")
        bridge.add_topic("/lidar/imu", "rover/lidar/imu/data", BridgeDirection.GZ_TO_ROS, ros_msg="sensor_msgs/Imu")
        bridge.add_topic("/lidar/scan", "rover/lidar/scan", BridgeDirection.GZ_TO_ROS, ros_msg="sensor_msgs/LaserScan")
        bridge.add_topic("/lidar/scan/points", "rover/lidar/scan/points", BridgeDirection.GZ_TO_ROS, ros_msg="sensor_msgs/PointCloud2")
        bridge.add_topic("/gps/fix", "rover/gps/fix", BridgeDirection.GZ_TO_ROS, ros_msg="sensor_msgs/NavSatFix")

    return sl.launch_description()
