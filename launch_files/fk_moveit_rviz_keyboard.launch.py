import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    workspace_moveit_launch = os.path.join(os.path.dirname(__file__), "moveit_launch.py")

    move_group_launch = os.path.join(
        get_package_share_directory("rover_arm_moveit_config"),
        "launch",
        "move_group.launch.py",
    )

    moveit_rviz_launch = os.path.join(
        get_package_share_directory("rover_arm_moveit_config"),
        "launch",
        "moveit_rviz.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(workspace_moveit_launch),
                launch_arguments={"use_fake_hardware": "true"}.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_rviz_launch),
                launch_arguments={"use_joint_state_publisher_gui": "false"}.items(),
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "joy_mux_controller_py", "fk_moveit_keyboard_controller"],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
