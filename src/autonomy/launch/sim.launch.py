from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(mode="simulation")

    sl.include(package="rover_description", launch_file="simulation.launch.py")

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
