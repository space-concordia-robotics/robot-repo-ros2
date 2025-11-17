import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ceres_rover", package_name="ceres_moveit_config")
        .to_moveit_configs()
    )

    # --- This is the fix ---
    # We manually find your top-level .xacro file
    xacro_file = os.path.join(
        get_package_share_directory("ceres_moveit_config"),
        "config",
        "ceres_rover.xacro",  # This is your top-level file that includes the ros2_control macro
    )
   
    # Process the xacro file to get the robot_description as a string
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    # -----------------------

    # This is the robot_state_publisher node, which is what
    # generate_rsp_launch() would have created.
    # We are now overriding its robot_description parameter.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            # Pass our manually-processed xacro content as the 'robot_description'
            {"robot_description": robot_description_content},
           
            # Pass the semantic description (SRDF) from MoveIt
            moveit_config.robot_description_semantic,
           
            # Pass the kinematics (kinematics.yaml) from MoveIt
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([robot_state_publisher_node])

