import os
from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch.actions import TimerAction


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rover_arm")
        .robot_description(file_path="config/rover_arm.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("rover_arm_moveit_config")
        .yaml("config/servo_config.yaml")
        .to_dict()
    }



    ros2_controllers_path = os.path.join(
        get_package_share_directory("rover_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    delayed_controller_manager = TimerAction(
        period = 3.0,
        actions=[ros2_control_node],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    rover_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[jsb_spawner, rover_arm_spawner], 
        )
    )


    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            delayed_controller_manager,
            delayed_spawners,
            servo_node,      
        ]
    )