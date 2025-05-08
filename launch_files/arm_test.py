from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Execute shell commands to configure the CAN interface
        # Set permissions for the required devices
        ExecuteProcess(
            cmd=['sudo', 'chmod', '777', '/dev/ttyTHS1'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['sudo', 'chmod', '777', '/dev/ttyTHS2'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['sudo', 'chmod', '777', '/dev/ttyUSB0'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['sudo', 'chmod', '777', '/dev/ttyUSB1'],
            shell=True
        ),
        #excute processof the nodes 
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
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
        )
    ])