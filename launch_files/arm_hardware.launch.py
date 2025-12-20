import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description(): 
    
    package_name="ceres_moveit_config"

     # Declare the HW_mode launch argument
    declare_hw_mode_arg = DeclareLaunchArgument(
        "HW_mode",
        default_value="true",
        description="Flag to set hardware vs. simulation mode."
    )

    HW_mode = LaunchConfiguration("HW_mode")

    #launching robot_state_publisher 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]),
        launch_arguments={"HW_mode" : HW_mode}.items()
    )

    controller_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'ros2_controllers.yaml'
    )

    #running the ros2_control node to handle controller spawining and loading 
    #remap topic to /robot_description

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[("/controller_manager/robot_description", "robot_description")],
        output="screen",
    )

    #delaying controller manager to give time for everything to initialize
    delayed_controller_manager = TimerAction(
        period = 3.0,
        actions={controller_manager},
    )


    #spawining in the joint state broadcaster 

    jsb_spawner = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Spawner for the controller that executes trajectories
    # *** UPDATE "ceres_arm_controller" to match your ros2_controllers.yaml ***
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ceres_arm_controller"], # <-- Fixed controller name
        output="screen",
    )

    # Register event handlers to start spawners AFTER controller_manager starts
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            # Add both spawners here
            on_start=[jsb_spawner, arm_controller_spawner], 
        )
    )

    # Temporarily commented out ik_mux_node due to build issues
    # ik_mux_node = Node(
    #     package="ik_mux_controller",
    #     executable="ik_mux",
    #     name="ik_mux_controller",
    #     output="screen",
    # )

    return LaunchDescription([
        
        declare_hw_mode_arg,
        rsp,
        delayed_controller_manager,
        delayed_spawners,
        # ik_mux_node,  # Temporarily commented out
    ])