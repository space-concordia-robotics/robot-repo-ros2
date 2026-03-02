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
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node_odom",
        parameters=[ekf_navsat_params],
        remappings=[("odometry/filtered", "ceres/odometry/local")]
    )

    sl.node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node_map",
        output="screen",
        parameters=[ekf_navsat_params],
        remappings=[("odometry/filtered", "ceres/odometry/global")],
    )

    # GPS (/fix) ──┐
    #              ├─→ navsat_transform ─→ /odometry/gps ──┐
    # IMU (/imu/data_raw) ──┤                              │
    #              └─→ ekf_local ─→ /odometry/local ───────┼─→ ekf_global ─→ /odometry/global
    #  ODOM (/odom) ────────┘                              │
    #                                                      │
    #                                                (map ← odom TF)

    sl.node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[ekf_navsat_params],
        remappings=[
            # ("imu/data", "ceres/imu"),
            ("imu/data", "ceres/lidar_imu"),
            ("gps/fix", "ceres/gps/fix"),
            ("gps/filtered", "ceres/gps/filtered"),
            ("odometry/gps", "ceres/odometry/gps"),
            ("odometry/filtered", "ceres/odometry/global"),
        ],
    )

    # sl.node(
    #     package="imu_processors",
    #     executable="imu_integrator_node",
    #     name="imu_integrator_node",
    #     output="screen",
    #     remappings=[
    #         ("/ceres/imu", "/imu"),
    #         ("/imu_integrated", "/ceres/imu_integrated"),
    #     ]
    # )
    #
    # sl.node(
    #     package="imu_processors",
    #     executable="imu_integrator_node",
    #     name="lidar_imu_integrator_node",
    #     output="screen",
    #     remappings=[
    #         ("/ceres/lidar_imu", "/imu"),
    #         ("/imu_integrated", "/ceres/lidar_imu_integrated"),
    #     ]
    # )

    return sl.launch_description()
