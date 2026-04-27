from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rover_arm", package_name="rover_arm_moveit_config").to_moveit_configs()
    launch_description = generate_moveit_rviz_launch(moveit_config)

    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")

    launch_description.add_action(
        DeclareLaunchArgument(
            "use_joint_state_publisher_gui",
            default_value="true",
            description="Publish /joint_states from GUI sliders",
        )
    )

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
    return launch_description
