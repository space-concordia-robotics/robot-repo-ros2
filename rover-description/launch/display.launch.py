from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    gui = sl.declare_arg("gui", default_value="true", choices=["true", "false"], description="Flag to enable joint_state_publisher_gui")

    sl.joint_state_publisher(use_gui=gui)

    sl.include(package="rover_description", launch_file="rsp.launch.py")

    rviz_config = sl.declare_arg(
        name="rviz_config",
        default_value=sl.find("rover_description", "rviz", "urdf.rviz"),
        description="Absolute path to rviz config file"
    )

    sl.include(
        package='urdf_launch',
        launch_file='display.launch.py',
        launch_arguments={
            "urdf_package": "rover_description",
            "urdf_package_path": sl.find(package="rover_description", directory="urdf", file="robot.urdf"),
            "rviz_config": rviz_config,
            "jsp_gui": "false",
        }.items()
    )

    return sl.launch_description()
