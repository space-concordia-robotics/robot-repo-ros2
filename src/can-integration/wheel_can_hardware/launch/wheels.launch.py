"""Bring up the rover wheel stack: WheelCanInterface + diff_drive_controller +
joint_state_broadcaster + WheelOdometryNode.

This launch file expects a top-level URDF/xacro that includes
`urdf/rover_wheels.ros2_control.xacro` (provided by this package) and a
controller-manager YAML with a diff_drive_controller entry. See the README
for an example.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface name (passed to WheelCanInterface).",
    )
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="true",
        description="Whether wheel_odometry_node should broadcast odom->base_link.",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.165",
        description="Driven wheel radius in meters.",
    )
    track_width_arg = DeclareLaunchArgument(
        "track_width",
        default_value="1.20",
        description="Distance between left and right wheel centers in meters.",
    )

    odometry_node = Node(
        package="wheel_can_hardware",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
        parameters=[{
            "wheel_radius": LaunchConfiguration("wheel_radius"),
            "track_width": LaunchConfiguration("track_width"),
            "publish_tf": LaunchConfiguration("publish_tf"),
        }],
    )

    return LaunchDescription([
        can_interface_arg,
        publish_tf_arg,
        wheel_radius_arg,
        track_width_arg,
        odometry_node,
    ])
