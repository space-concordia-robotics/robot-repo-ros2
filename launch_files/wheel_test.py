from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Execute shell commands to configure the CAN interface
        ExecuteProcess(
            cmd=['sudo', 'busybox', 'devmem', '0x0c303018', 'w', '0xc458'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['sudo', 'busybox', 'devmem', '0x0c303010', 'w', '0xc400'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['sudo', 'modprobe', 'can'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['sudo', 'modprobe', 'can_raw'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['sudo', 'modprobe', 'mttcan'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '1000000'],
            shell=True
        ),
        #excute processof the nodes 
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            #output='screen',
        ),
        Node(
            package='joy_mux_controller',
            executable='joy_mux_controller',
            name='joy_mux_controller',
            #output='screen',
        ),
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