from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    gui = sl.declare_arg("gui", default_value="true", choices=["true", "false"], description="Flag to enable joint_state_publisher_gui")

    sl.joint_state_publisher(use_gui=gui)

    sl.include(package="rover_description", launch_file="rsp.launch.py")

    # rviz_config = sl.declare_arg(
    #     name="rviz_config",
    #     default_value=sl.find("rover_description", "rviz", "urdf.rviz"),
    #     description="Absolute path to rviz config file"
    # )
    #
    # sl.declare_arg()

    return sl.launch_description()

    ld = LaunchDescription()

    rover_description = FindPackageShare("rover_description")

    ld.add_action(
        DeclareLaunchArgument(
            name="gui",
            default_value="true",
            choices=["true", "false"],
            description="Flag to enable joint_state_publisher_gui"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="rviz_config",
            default_value=PathJoinSubstitution([rover_description, "rviz", "urdf.rviz"]),
            description="Absolute path to rviz config file"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="model",
            default_value=PathJoinSubstitution(["urdf", "robot.urdf"]),
            description="Path to robot urdf file relative to urdf_tutorial package"
        ),
    )

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("urdf_launch"), "launch", "display.launch.py"]),
            launch_arguments={
                "urdf_package": "rover_description",
                "urdf_package_path": LaunchConfiguration("model"),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "jsp_gui": LaunchConfiguration("gui"),
            }.items()
        )
    )

    return ld
