from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, TextSubstitution

mode_parameter = LaunchConfiguration('mode')

control_parameter = LaunchConfiguration('control')

sim_time_parameter = EqualsSubstitution(mode_parameter, TextSubstitution(text='simulation'))


def add_mode_argument(ld: LaunchDescription):
    ld.add_action(
        DeclareLaunchArgument(
            'mode',
            default_value='production',
            choices=['production', 'simulation'],
            description='The operation mode to use'
        )
    )


def add_control_argument(ld: LaunchDescription):
    ld.add_action(
        DeclareLaunchArgument(
            'control',
            default_value='ros',
            choices=['ros', 'gazebo'],
            description='The control mode to use'
        ),
    )
