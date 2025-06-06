from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
     
        # Launch the ublox_node ie the gps
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='gps_node',
            output='screen',       
        ),
     
        # Launch the joy_mux_controller
        Node(
            package='joy_mux_controller',
            executable='joy_mux_controller',
            name='joy_mux_controller',
            #output='screen',
        ),

        # Launch the arm_controller_node
        Node(
            package='arm_controller',
            executable='arm_controller_node',
            name='arm_controller_node',
            output='screen',
            parameters=[
                {'local_mode': False}  # Example parameter for the arm controller
            ]
        ),

        # Launch the wheels_controller_node
        Node(
            package='wheels_controller',
            executable='wheels_controller_node',
            name='wheels_controller_node',
            output='screen',
            parameters=[
                {'can_path': 'can0'},  # Default CAN interface path
                {'multiplier': 500}    # Default multiplier for RPM conversion
            ]
        )
    ])
