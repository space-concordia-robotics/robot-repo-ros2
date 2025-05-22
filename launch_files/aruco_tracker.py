"""
Author: Felipe Lazcano and Rayan Raad
Description: This is a ros2 python launch file. 
This launch creates an aruco_tracker node from the aruco_opencv package.
The node will listen to the image topic and find aruco IDs.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import cv2
from cv2_enumerate_cameras import enumerate_cameras

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            parameters=[{'cam_base_topic':'zed/zed_node/rgb/image_rect_color'},
            {'image_is_rectified':True},
            #{'marker_dict':'5x5_50'}
            ]
            )])
