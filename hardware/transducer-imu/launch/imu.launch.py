from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("tm_imu"), "config", "params.yaml"])

    imu_node = Node(
        package="tm_imu",
        executable="transducer_m_imu",
        name="tm_imu",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([imu_node])
