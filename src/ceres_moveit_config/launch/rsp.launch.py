# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_rsp_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("ceres_rover", package_name="ceres_moveit_config").to_moveit_configs()
#     return generate_rsp_launch(moveit_config)


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('ceres_moveit_config'))
    xacro_file = os.path.join(pkg_path,'config','ceres_rover.ros2_control.xacro')

    robot_description_config = Command(['xacro ', xacro_file])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([

        node_robot_state_publisher
    ])
