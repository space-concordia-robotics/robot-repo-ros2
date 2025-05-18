import os
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

def generate_launch_description():
    moveit_config = (
    MoveItConfigsBuilder("ceres_servo_controller")
    .robot_description(file_path="config/ceres_rover.urdf.xacro")
    .joint_limits(file_path="config/joint_limits.yaml")
    .to_moveit_configs()
    )

    servo_params = {
        "ceres_servo" :ParameterBuilder("ceres_servo")
        .yaml("config/servo_params.yaml").to_dict()
    }

    # This set the update rate and planning group name for the celeration limits filter
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "ceres_arm"}


    #Rviz config (hardest part)

    rviz_config_file = (
        get_package_share_directory("ceres_servo_controller")
        + "/config/ceres_arm_servo.rviz"
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("ceres_servo_controller"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen"
    )

    joint_state_broadcaser_spawner = launch_ros.actions.Node(
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

    ceres_arm_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ceres_arm_controller", "-c", "/controller_manager"],       
    )

    container = launch_ros.actions.ComposableNodeContainer(
        name="ceres_servo_container", 
        namespace="/",
        package="rclcpp_components",
        composable_node_descriptions=[

            launch_ros.descriptions.ComposableNode(
                package="ceres_servo_controller",
                plugin="ceres_servo_controller::CeresServoController",
                name="ceres_servo_controller",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits, 
                ],
            ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[ moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcaster",
                name="static_tf_broadcaster",
                parameters=[{"child_frame_id": "/base_structure_link", "frame_id": "/origin"}],
            )
        ],
        output="screen",
    )

    servo_node = launch_ros.actions.Node(
        package="ceres_servo_controller",
        executable="ceres_servo_node",
        name="ceres_servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits, 
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [ 
            rviz_node,
            ros2_control_node,
            joint_state_broadcaser_spawner,
            ceres_arm_controller_spawner,
            container,
            servo_node,
        ]
    )