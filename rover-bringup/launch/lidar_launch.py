#we want to launch ros2 launch ouster_ros sensor.launch.xml    \
#    sensor_hostname:=<sensor host name>
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch',
                'sensor.launch.xml'
            ]),
            launch_arguments={
                'sensor_hostname': 'os1-992005000098.local',
            }.items()
        )
    ])
