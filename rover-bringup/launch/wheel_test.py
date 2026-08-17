from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # execute process of the nodes
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            # output='screen',
        ),
        Node(
            package="joy_mux_controller",
            executable="joy_mux_controller",
            name="joy_mux_controller",
            # output='screen',
        ),
        Node(
            package="wheels_controller",
            executable="wheels_controller_node",
            name="wheels_controller_node",
            output="screen",
            parameters=[
                {"can_path": "can0"},  # Default CAN interface path
                {"multiplier": 900},  # Default multiplier for RPM conversion
            ],
        ),
    ])
