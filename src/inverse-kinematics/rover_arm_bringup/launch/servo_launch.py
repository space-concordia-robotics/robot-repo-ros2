from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    urdf_model_path = PathJoinSubstitution([FindPackageShare("rover_arm_description"), "urdf", "arm.urdf.xacro"])
    rviz_config_path = PathJoinSubstitution([FindPackageShare("rover_arm_description"), "rviz", "urdf_config.rviz"])
    controller_config_path = PathJoinSubstitution([FindPackageShare("rover_arm_bringup"), "config", "ros2_controllers.yaml"])
    servo_config_path = PathJoinSubstitution([FindPackageShare("rover_arm_bringup"), "config", "servo_config.yaml"])

    robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_model_path,
    ]
)
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_path],
        output="screen",
    )

    joint_state_broadcaster_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    rviz_spawner= Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
        parameters=[robot_description],
    )

    servo_config_spawner = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[servo_config_path, robot_description],
        output="screen",
    )


    return LaunchDescription([

        robot_state_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rviz_spawner,
        servo_config_spawner,

    ])