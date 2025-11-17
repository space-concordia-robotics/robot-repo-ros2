import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ceres_rover", package_name="ceres_moveit_config")
        .robot_description_file(
            # This is the fix:
            # Point to your top-level .xacro file instead of the default .urdf
            os.path.join(
                get_package_share_directory("ceres_moveit_config"),
                "config",
                "ceres_rover.xacro", # This is the file you showed me that includes the ros2_control macro
            )
        )
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config)

