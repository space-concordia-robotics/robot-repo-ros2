from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration


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
        return generate_move_group_launch(moveit_config).entities

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Use mock ros2_control hardware in the MoveIt robot description",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
