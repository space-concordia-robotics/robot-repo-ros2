"""
Author: Felipe Lazcano and Rayan Raad
Description: This is a ros2 python launch file. 
This launch creates an aruco_tracker node from the aruco_opencv package.
The node will listen to the image topic and find aruco IDs.

Usable values for marker_dict:
    4X4_50
    4X4_100
    4X4_250
    4X4_1000
    5X5_50
    5X5_100
    5X5_250
    5X5_1000
    6X6_50
    6X6_100
    6X6_250
    6X6_1000
    7X7_50
    7X7_100
    7X7_250
    7X7_1000
    ARUCO_ORIGINAL
    APRILTAG_16h5
    APRILTAG_25h9
    APRILTAG_36h10
    APRILTAG_36h11
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
                        {'marker_dict':'5X5_50'}
            ]
        )
    ])
