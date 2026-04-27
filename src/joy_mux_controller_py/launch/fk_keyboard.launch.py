from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_mode = LaunchConfiguration("local_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "local_mode",
                default_value="true",
                description="Run arm controller without serial hardware access",
            ),
            Node(
                package="arm_controller",
                executable="arm_controller_node",
                name="arm_controller_node",
                output="screen",
                parameters=[{"local_mode": local_mode}],
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "joy_mux_controller_py", "fk_keyboard_controller"],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
