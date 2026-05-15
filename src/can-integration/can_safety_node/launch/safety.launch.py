"""Launch the can_safety_node with the legacy joystick mappings."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface_arg = DeclareLaunchArgument(
        "can_interface", default_value="can0",
        description="SocketCAN interface name (passed to CanSafetyNode).",
    )
    fs_arg = DeclareLaunchArgument(
        "wheel_force_stop_button", default_value="6",
        description="Joy button index for wheel force stop (rising edge triggered).",
    )
    rs_arg = DeclareLaunchArgument(
        "wheel_resume_button", default_value="7",
        description="Joy button index for wheel resume (rising edge triggered).",
    )

    safety = Node(
        package="can_safety_node",
        executable="can_safety_node",
        name="can_safety_node",
        output="screen",
        parameters=[{
            "can_interface": LaunchConfiguration("can_interface"),
            "wheel_force_stop_button": LaunchConfiguration("wheel_force_stop_button"),
            "wheel_resume_button": LaunchConfiguration("wheel_resume_button"),
        }],
    )

    return LaunchDescription([can_interface_arg, fs_arg, rs_arg, safety])
