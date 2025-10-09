from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('object_recognition')
    keymap_file = os.path.join(pkg_share, 'config', 'keymap.yaml')

    return LaunchDescription([
        Node(
            package='object_recognition',
            executable='keymap_node',
            name='keymap_node',
            output='screen',
            parameters=[{'keymap_file': keymap_file}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0.7', '1', '0', '0', '1.5708', 'base_structure_link', 'keyboard']
        )
    ])
