import os
import launch
import yaml
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch.conditions import IfCondition, UnlessCondition


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ceres")
        .robot_description(file_path="config/ceres_rover.urdf.xacro")
        .to_moveit_configs()
    )


    # Get parameters for the Servo node
    # servo_params =  {
    #     "moveit_servo": ParameterBuilder("ceres_moveit_config")
    #     .yaml("config/servo_params.yaml")
    #     .to_dict()
    # }
    
    servo_yaml = load_yaml("ceres_moveit_config", "config/servo_params.yaml")
    servo_params = {"moveit_servo": servo_yaml}


    # RViz
    rviz_config_file = (
        get_package_share_directory("ceres_moveit_config")
        + "/config/moveit.rviz"
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

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ceres_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
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

    #If ever we wanna launch move_group, we can do it here.

    # move_group_node = launch_ros.actions.Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #     moveit_config.to_dict(),
    #   ],
    # )

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="ceres_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[

            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_structure_link", "frame_id": "/origin"}],
            ),
        ],
        output="screen",
    )
    
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description, #Load urdf
            moveit_config.robot_description_semantic, #Load SRDF
            moveit_config.robot_description_kinematics, #Load kinematics.yaml (does not fkn work for some reason)
        ],
        output="screen",
    )

    IK_mux_node = launch_ros.actions.Node(
        package="joy_mux_controller",
        executable="IK_mux",
        name="IK_mux",
        output="screen",
    )

    joy_node = launch_ros.actions.Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )
    
#Issues to fix: /joint_states topic is not being published, so the robot state publisher is not working. 

    return launch.LaunchDescription(
        [
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            ceres_arm_controller_spawner,
            #move_group_node,
            servo_node,
            IK_mux_node,
            joy_node,
            container,
        ]
    )