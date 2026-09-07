#!/usr/bin/env python3

from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "urdf_viewer_pkg"


def start_viewer(_context):
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    package_install = package_share.parent.parent
    processor = package_install / "lib" / PACKAGE_NAME / "urdf_preprocessor.py"
    imports = package_share / "URDF_Imports"
    processed = Path.home() / ".ros" / PACKAGE_NAME / "processed_rover.xacro"

    subprocess.run(
        [
            sys.executable,
            str(processor),
            "--imports",
            str(imports),
            "--output",
            str(processed),
        ],
        check=True,
    )

    robot_description = ParameterValue(
        Command(["xacro ", str(processed)]),
        value_type=str,
    )
    rviz_config = package_share / "rviz_configuration" / "urdf_config.rviz"

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=start_viewer)])
