import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Minimal launch for visualizing rover_arm model with joint sliders.
    
    Loads the rover arm URDF without hardware dependencies for simulation/visualization.
    - Shows the robot model in RViz
    - Provides joint sliders for manual control
    - Can optionally start ros2_control (requires packages to be built)
    """
    
    # === Configuration Paths ===
    moveit_config_dir = get_package_share_directory("rover_arm_moveit_config")
    
    # Use rover_arm_description's urdf directly (simpler, avoids hardware dependency)
    urdf_path = os.path.join(
        get_package_share_directory("rover_arm_description"), 
        "urdf", 
        "arm.urdf.xacro"
    )
    
    # Use MoveIt RViz config for visualization
    rviz_config_path = os.path.join(moveit_config_dir, "config", "moveit.rviz")
    
    # === Launch Arguments ===
    use_gui_arg = LaunchConfiguration("use_gui")
    use_rviz_arg = LaunchConfiguration("use_rviz")
    
    declare_use_gui = DeclareLaunchArgument(
        "use_gui", 
        default_value="true",
        description="Launch joint_state_publisher_gui for slider control"
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", 
        default_value="true",
        description="Launch RViz for visualization"
    )
    
    # === Robot Description ===
    # Load URDF from rover_arm_description package (not moveit_config)
    # This avoids the ros2_control hardware dependency
    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            urdf_path,
        ]),
        value_type=str,
    )
    
    # === Robot State Publisher ===
    # Publishes transform tree from URDF and joint_states
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    
    # === Joint State Publisher GUI ===
    # Provides sliders to manually publish joint states
    # Pass robot_description as parameter so GUI doesn't wait for topic
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[
            {"robot_description": robot_description},
            {"publish_frequency": 50.0},
        ],
        output="screen",
        condition=IfCondition(use_gui_arg),
    )
    
    # === RViz Visualization ===
    # Displays the model with all visual properties
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
        condition=IfCondition(use_rviz_arg),
    )
    
    # === Launch Description ===
    return LaunchDescription([
        declare_use_gui,
        declare_use_rviz,
        rsp_node,
        joint_state_publisher_gui,
        rviz_node,
    ])
