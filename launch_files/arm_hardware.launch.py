import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
<<<<<<< HEAD
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description(): 
    
    urdf_path = os.path.join(get_package_share_directory('rover_arm_description'), 'urdf', 'arm.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    controller_config = os.path.join(
        get_package_share_directory('rover_arm_bringup'), 'config', 'ros2_controllers.yaml'
    )
   

    rsp_launch = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
=======
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
>>>>>>> origin/main

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
<<<<<<< HEAD
        parameters=[controller_config, {'robot_description': robot_description}],
=======
        parameters=[controller_config],
        remappings=[("/controller_manager/robot_description", "robot_description")],
>>>>>>> origin/main
        output="screen",
    )

    #delaying controller manager to give time for everything to initialize
    delayed_controller_manager = TimerAction(
        period = 3.0,
<<<<<<< HEAD
        actions=[controller_manager],
=======
        actions={controller_manager},
>>>>>>> origin/main
    )


    #spawining in the joint state broadcaster 

    jsb_spawner = Node( 
        package="controller_manager",
        executable="spawner",
<<<<<<< HEAD
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
=======
        arguments=["joint_state_broadcaster"],
>>>>>>> origin/main
        output="screen",
    )

    # Spawner for the controller that executes trajectories
    # *** UPDATE "ceres_arm_controller" to match your ros2_controllers.yaml ***
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
<<<<<<< HEAD
        arguments=["arm_controller", "--controller-manager", "/controller_manager"], # <-- Fixed controller name
=======
        arguments=["ceres_arm_controller"], # <-- Fixed controller name
>>>>>>> origin/main
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
        
<<<<<<< HEAD
        rsp_launch,
=======
        declare_hw_mode_arg,
        rsp,
>>>>>>> origin/main
        delayed_controller_manager,
        delayed_spawners,
        # ik_mux_node,  # Temporarily commented out
    ])