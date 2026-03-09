from launch_ros.actions import SetRemap

from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(mode="production", control="ros")

    # sl.add_action(PushROSNamespace("rover"))
    # sl.add_action(SetRemap("tf", "/tf"))
    # sl.add_action(SetRemap("/tf", "tf"))
    # sl.add_action(SetRemap("tf_static", "/tf_static"))
    # sl.add_action(SetRemap("/tf_static", "tf_static"))
    # sl.add_action(SetRemap("~/robot_description", "/robot_description"))

    with sl.group():
        sl.add_action(
            SetRemap(src="/ouster/points", dst="rover/lidar/scan/points"),
        )
        sl.add_action(
            SetRemap(src="/ouster/scan", dst="rover/lidar/scan"),
        )
        sl.add_action(
            SetRemap(src="/ouster/imu", dst="rover/lidar/imu/data"),
        )
        sl.include(
            package="ouster_ros",
            launch_file="sensor.launch.xml",
            launch_arguments={
                "sensor_hostname": "os1-992005000098.local",
                "sensor_frame": "lidar_link",
                # "imu_frame": "lidar_link",
                # "lidar_frame": "lidar_link"
                "timestamp_mode": "TIME_FROM_ROS_TIME",
                "viz": "False",
            }.items(),
        )

    sl.node(
        package="tm_imu",
        executable="transducer_m_imu",
        parameters=[sl.params(package="rover_description", file="transducer.yaml")],
        remappings=[
            ("/imu_data", "rover/imu/data"),
            ("/imu_data_rpy", "rover/imu/rpy"),
            ("/imu_data_mag", "rover/imu/mag"),
        ],
    )

    sl.include(
        package="autonomy",
        launch_file="gps.launch.py",
    )

    sl.include(package="rover_description", launch_file="rsp.launch.py")
    sl.include(package="rover_description", launch_file="joystick.launch.py")

    twist_mux_params = sl.params(package="rover_description", file="twist_mux.yml")
    sl.node(
        package="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "rover/diff_drive_base_controller/cmd_vel_unstamped")],
    )

    controllers = sl.params(package="rover_description", file="controllers.yaml")

    # TODO 2026-02-17 (Will Free): add parameters for controller_manager: https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html
    sl.node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        remappings={
            "~/robot_description": "rover/robot_description",
        }.items(),
    )

    sl.node(
        "controller_manager",
        "spawner",
        namespace="rover",
        exec_name="diff_drive_spawner",
        arguments=["diff_drive_base_controller", "--param-file", controllers],
    )

    sl.node(
        "controller_manager",
        "spawner",
        namespace="rover",
        exec_name="jsp_spawner",
        arguments=["joint_state_broadcaster"],
    )

    sl.include(package="autonomy", launch_file="ekf_navsat.launch.py")

    rviz_config = sl.declare_arg("rviz_config", default_value=sl.find(package="rover_description", directory="rviz", file="urdf.rviz"))

    sl.node(package="rviz2", arguments=["-d", rviz_config])

    sl.include(
        package="nav2_bringup",
        launch_file="navigation_launch.py",
        launch_arguments={
            "autostart": 'True',
            "use_composition": 'False',  # TODO 2026-02-09 (Will Free): consider using composition?
            "use_respawn": 'True',  # only relevant when composition is disabled
            "params_file": sl.params(package="autonomy", file="nav2_params.yaml"),
        }.items(),
    )

    return sl.launch_description()
