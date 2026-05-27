from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    ekf_navsat_params = sl.params(package="autonomy", file="ekf.yaml")

    # sl.node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_node",
    #     parameters=[ekf_navsat_params]
    # )

    sl.arg("output_final_position", default="false")
    sl.arg("output_location", default=sl.find(package="autonomy", directory="logs", file="dual_ekf_navsat_example_debug.txt"))

    sl.node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="rover_imu_filter",
        output="screen",
        parameters=[ekf_navsat_params],
        remappings=[
            ("imu/data_raw", "rover/imu/data"),
            ("imu/mag", "rover/imu/mag"),
            ("imu/data", "rover/imu/filtered"),
        ]
    )

    sl.node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node_odom",
        parameters=[ekf_navsat_params],
        remappings=[("odometry/filtered", "rover/odometry/local")],
    )

    sl.node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node_map",
        output="screen",
        parameters=[ekf_navsat_params],
        remappings=[("odometry/filtered", "rover/odometry/global")],
    )

    sl.node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[ekf_navsat_params],
        remappings=[
            ("imu", "rover/imu/data"),
            ("gps/fix", "rover/gps/fix"),
            ("gps/filtered", "rover/gps/filtered"),
            ("odometry/gps", "rover/odometry/gps"),
            ("odometry/filtered", "rover/odometry/global"),
        ],
    )

    # sl.node(
    #     package="imu_processors",
    #     executable="imu_integrator_node",
    #     name="imu_integrator_node",
    #     output="screen",
    #     remappings=[
    #         ("rover/imu", "/imu"),
    #         ("/imu_integrated", "rover/imu/integrated"),
    #     ]
    # )
    #
    # sl.node(
    #     package="imu_processors",
    #     executable="imu_integrator_node",
    #     name="lidar_imu_integrator_node",
    #     output="screen",
    #     remappings=[
    #         ("rover/lidar/imu/data", "/imu"),
    #         ("/imu_integrated", "rover/lidar/imu/integrated"),
    #     ]
    # )

    return sl.launch_description()
