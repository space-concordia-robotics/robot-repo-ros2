from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ceres_urdf = FindPackageShare('ceres_urdf')

    ld.add_action(
        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=PathJoinSubstitution([ceres_urdf, 'rviz', 'urdf.rviz']),
            description='Absolute path to rviz config file'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name='model',
            default_value=PathJoinSubstitution(['urdf', 'robot.urdf']),
            description='Path to robot urdf file relative to urdf_tutorial package'
        ),
    )

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'ceres_urdf',
                'urdf_package_path': LaunchConfiguration('model'),
                'rviz_config': LaunchConfiguration('rviz_config'),
                'jsp_gui': LaunchConfiguration('gui'),
            }.items()
        )
    )

    return ld
