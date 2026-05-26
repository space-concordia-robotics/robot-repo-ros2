"""Launch the can_safety_node.

Default button indices are VKBButtonLayout F1 (26) and F2 (27), which are
unassigned in joy_mux_controller_py and do not conflict with arm-joint axes.

Verify on your hardware first:
    ros2 run joy_mux_controller_py joy_button_probe
Then override if measured indices differ from the defaults:
    ros2 launch can_safety_node safety.launch.py \\
        wheel_force_stop_button:=<F1_index> wheel_resume_button:=<F2_index>
"""

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
        "wheel_force_stop_button", default_value="26",
        description=(
            "Joy button index for wheel force stop (rising edge triggered). "
            "Default = VKBButtonLayout.F1 (26). Verify with joy_button_probe."
        ),
    )
    rs_arg = DeclareLaunchArgument(
        "wheel_resume_button", default_value="27",
        description=(
            "Joy button index for wheel resume (rising edge triggered). "
            "Default = VKBButtonLayout.F2 (27). Verify with joy_button_probe."
        ),
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
