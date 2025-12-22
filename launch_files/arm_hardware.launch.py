import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
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

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config, {'robot_description': robot_description}],
        output="screen",
    )

    #delaying controller manager to give time for everything to initialize
    delayed_controller_manager = TimerAction(
        period = 3.0,
        actions=[controller_manager],
    )


    #spawining in the joint state broadcaster 

    jsb_spawner = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner for the controller that executes trajectories
    # *** UPDATE "ceres_arm_controller" to match your ros2_controllers.yaml ***
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"], # <-- Fixed controller name
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
        
        rsp_launch,
        delayed_controller_manager,
        delayed_spawners,
        # ik_mux_node,  # Temporarily commented out
    ])