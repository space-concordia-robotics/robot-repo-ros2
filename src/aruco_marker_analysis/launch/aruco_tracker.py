"""
Author: Felipe Lazcano and Rayan Raad

Description:
This is a ROS 2 Python launch file that starts an ArUco marker tracking node
from the `aruco_opencv` package.

The node subscribes to a camera image topic, detects ArUco markers,
    and publishes their poses (position and orientation).

    Supported marker dictionaries include:
    4X4_50, 4X4_100, 4X4_250, 4X4_1000,
    5X5_50, 5X5_100, 5X5_250, 5X5_1000,
    6X6_50, 6X6_100, 6X6_250, 6X6_1000,
    7X7_50, 7X7_100, 7X7_250, 7X7_1000,
    ARUCO_ORIGINAL,
    APRILTAG_16h5, APRILTAG_25h9,
    APRILTAG_36h10, APRILTAG_36h11
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'video_file',
	   # Change to usb cam location
            default_value='/dev/video0',
	    description ='USB cam'
        ),
	
	# Image Publisher
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher',
            output='screen',
            arguments=[LaunchConfiguration('video_file')],
            parameters=[
                {'frame_id': 'camera_frame'},
                {'loop': True}
            ]
        ),
	
	# ArucoOpen CV (marker dictionary)
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            output='screen',
            remappings=[
                ('image', '/image_raw')
            ],
            parameters=[
                {'cam_base_topic': '/image_raw'},
                {'marker_dict': '5X5_50'},
                {'marker_size': 0.01},# m to cm
                {'draw_markers': True},
                {'frame_id': 'camera_frame'},
            ]
        ),
	
	# ArucoMarker Analysis
        Node(
            package='aruco_marker_analysis',
            executable='aruco_pose_analyzer',
            name='aruco_pose_analyzer',
            output='screen',
            parameters=[
                {'angle_threshold_deg': 15.0},
                {'pose_topic': '/aruco_detections'}
            ]
        ),
	
	# Video feed
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/image_raw']
        )

    ])
