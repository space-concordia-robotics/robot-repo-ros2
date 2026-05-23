from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    # sl.include(package="autonomy", launch_file="ekf_navsat.launch.py")

    # use_composition = sl.declare_arg("use_composition", default_value="False")
    #
    # sl.composable_node_container(
    #     multithreaded=True,
    #     name="nav2_container",
    #     condition=IfCondition(use_composition)
    # )

    sl.include(
        package="nav2_bringup",
        # package="autonomy",
        launch_file="navigation_launch.py",
        launch_arguments={
            "autostart": sl.declare_arg("autostart"),
            # "use_composition": use_composition,
            "namespace": sl.declare_arg("namespace", default_value=""),
            "use_respawn": sl.declare_arg("use_respawn"),
            "params_file": sl.params(package="autonomy", file="nav2_params.yaml"),
        }.items(),
    )

    return sl.launch_description()
