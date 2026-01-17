from xml.dom import minidom

import xacro
from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_util import add_mode_argument, add_control_argument, control_parameter, mode_parameter, sim_time_parameter


def generate_launch_description():
    """Robot state publisher"""

    ld = LaunchDescription()

    add_mode_argument(ld)
    add_control_argument(ld)

    ld.add_action(
        OpaqueFunction(function=robot_state_publisher_node)
    )

    return ld


def remove_comments(parent: minidom.Node):
    node: minidom.Node
    for node in parent.childNodes:
        if isinstance(node, minidom.Comment):
            parent: minidom.Node = node.parentNode
            parent.removeChild(node)
        else:
            remove_comments(node)


def robot_state_publisher_node(context: LaunchContext, *args: object, **kwargs: object) -> list[Node]:
    """Disgusting hack to work around ros2/launch_ros#214 & ros-controls/gazebo_ros2_control#295"""
    print(control_parameter.__class__)
    control = control_parameter.perform(context)
    mode = mode_parameter.perform(context)

    ceres_urdf = FindPackageShare('ceres_urdf')
    urdf_dir = PathJoinSubstitution([ceres_urdf, 'urdf']).perform(context)
    urdf_file = PathJoinSubstitution([urdf_dir, 'robot.urdf']).perform(context)

    robot_description = xacro.parse(open(urdf_file))

    xacro.root_dir = urdf_dir

    xacro.process_doc(robot_description, mappings={
        'control': control,
        'mode': mode,
    })

    remove_comments(robot_description)

    print(robot_description.toprettyxml())  # this print statement is load bearing, do not remove. it will break.

    robot_description_param = {"robot_description": robot_description.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': sim_time_parameter}],
    )

    return [
        node_robot_state_publisher,
    ]
