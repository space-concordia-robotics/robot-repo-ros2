import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    base_link_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_alias_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_structure_link", "base_link"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use mock ros2_control hardware (set false for real arm)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(workspace_moveit_launch),
                launch_arguments={
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                    "launch_servo": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(move_group_launch),
                launch_arguments={
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_rviz_launch),
                launch_arguments={
                    "use_joint_state_publisher_gui": "false",
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                }.items(),
            ),
            base_link_alias_tf,
            ExecuteProcess(
                cmd=["ros2", "run", "joy_mux_controller_py", "arm_pid_calibration_keyboard"],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
