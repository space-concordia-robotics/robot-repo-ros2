"""
Author: Felipe Lazcano and Rayan Raad
Description: This is a ros2 python launch file. 
This launch creates an aruco_tracker node from the aruco_opencv package.
The node will listen to the image topic and find aruco IDs.
"""

from launch import LaunchDescription # launch system component
from launch_ros.actions import Node # launch system component
import cv2 # -- not used
from cv2_enumerate_cameras import enumerate_cameras # -- not used

def generate_launch_description(): # launch system
    return LaunchDescription([
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            output='screen', # visualize logs in the terminal
            parameters=[{'cam_base_topic':'zed/zed_node/rgb/image_rect_color'},# subscriber to imgs
                        {'image_is_rectified':True}, # removing distortion
                        #{'marker_dict':'5x5_50'}
            ]
        )
    ])
