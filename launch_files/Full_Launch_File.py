from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    zed_launch_file = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py',
        {'camera_name':'zed2'}
    )
    
    return LaunchDescription([
        Node(
            package="ublox_gps",
            executable="ublox_gps_node",
            name="gps_node",
            output="screen",
        ),

        Node( 
            package='joy_mux_controller',
            executable='joy_mux_controller',
            name='joy_mux_controller',
            output='screen',
        ),

        Node(
            package='arm_controller',
            executable='arm_controller_node',
            name='arm_controller_node',
            output='screen',
            parameters=[{'local_mode': False}],
        ),

        Node(
            package='wheels_controller',
            executable='wheels_controller_node',
            name='wheels_controller_node',
            output='screen',
            parameters=[
                {'can_path': 'can0'},
                {'multiplier': 500},
            ],
        ),

        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            parameters=[
                {'aruco_dict': 'DICT_4X4_50'},
                {'cam_base_topic': '/zed/zed_node/rgb/image_rect_color'},
                {'image_is_rectified': True},
                {'marker_dict':'DICT_5X5_50'},
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch_file),
        ),
     ]) 

