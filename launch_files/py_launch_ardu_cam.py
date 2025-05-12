"""
Author: Felipe Lazcano
Description: This is a ros2 python launch file. 
A list of USB cameras is extracted using the cv2 library. 
A ros2 usb_cam node is created for each camera.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import cv2
from cv2_enumerate_cameras import enumerate_cameras

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            namespace=f'usb_cam_{cam_id.path[-1]}',
            executable='usb_cam_node_exe',
            parameters=[{
        	'video_device': str(cam_id.path),
        	'framerate': 10.0,
        	'io_method': str("mmap"),
        	'frame_id': str("camera"),
        	'pixel_format': str("yuyv2rgb"),
        	'av_device_format': str("YUV422P"),
        	'image_width': 960,
        	'image_height': 600,
        	'camera_name': str("test_camera"),
        	'camera_info_url': str("package://usb_cam/config/camera_info.yaml"),
        	'brightness': -1,
        	'contrast': -1,
        	'saturation': -1,
        	'sharpness': -1,
        	'gain': -1,
        	'auto_white_balance': True,
        	'white_balance': 4000,
        	'autoexposure': True,
        	'exposure': 100,
        	'autofocus': False,
        	'focus': -1}
         ])
        for cam_id in list(enumerate_cameras(cv2.CAP_V4L2))]
	)
