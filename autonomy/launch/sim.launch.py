from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(mode="simulation")

    sl.include(package="rover_description", launch_file="joystick.launch.py")

    sl.include(
        package="autonomy",
        launch_file="navigation.launch.py",
        launch_arguments={
            "autostart": "True",
            "use_composition": "False",  # TODO 2026-02-09 (Will Free): consider using composition?
            "use_respawn": "True",  # only relevant when composition is disabled
            # "namespace": "rover",
        }.items(),
    )

    sl.include(package="rover_description", launch_file="rsp.launch.py")

    # sl.add_action(PushROSNamespace("rover"))
    # sl.add_action(SetRemap("tf", "/tf"))
    # sl.add_action(SetRemap("tf_static", "/tf_static"))
    # sl.add_action(SetRemap("joint_states", "/joint_states"))
    # sl.add_action(SetRemap("robot_description", "/robot_description"))
    # # sl.add_action(SetRemap("bond", "/bond"))
    # sl.add_action(SetRemap("cmd_vel_joy", "/cmd_vel_joy"))

    sl.include(package="rover_description", launch_file="simulation.launch.py")

    sl.include(package="autonomy", launch_file="ekf_navsat.launch.py")

    rviz_config = sl.declare_arg("rviz_config", default_value=sl.find(package="rover_description", directory="rviz", file="urdf.rviz"))

    sl.node(package="rviz2", arguments=["-d", rviz_config])

    return sl.launch_description()
