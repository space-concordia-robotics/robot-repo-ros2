import os

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    def launch_setup(context, *args, **kwargs):
        use_fake = LaunchConfiguration("use_fake_hardware").perform(context)
        moveit_config = (
            MoveItConfigsBuilder("rover_arm", package_name="rover_arm_moveit_config")
            .robot_description(
                file_path="config/rover_arm.urdf.xacro",
                mappings={"use_fake_hardware": use_fake},
            )
            .to_moveit_configs()
        )
        launch_description = generate_moveit_rviz_launch(moveit_config)

        static_virtual_joint_tfs = os.path.join(
            get_package_share_directory("rover_arm_moveit_config"),
            "launch",
            "static_virtual_joint_tfs.launch.py",
        )
        launch_description.add_action(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(static_virtual_joint_tfs))
        )

        use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        )

        joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            parameters=[moveit_config.robot_description],
            condition=IfCondition(use_joint_state_publisher_gui),
        )

        launch_description.add_action(robot_state_publisher)
        launch_description.add_action(joint_state_publisher_gui)
        return launch_description.entities

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_joint_state_publisher_gui",
                default_value="true",
                description="Publish /joint_states from GUI sliders",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Use mock ros2_control hardware in the published robot description",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
