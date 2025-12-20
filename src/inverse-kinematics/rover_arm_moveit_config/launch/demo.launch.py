from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
<<<<<<< HEAD:src/ceres_moveit_config/launch/demo.launch.py
    moveit_config = MoveItConfigsBuilder("ceres_rover", package_name="ceres_moveit_config").to_moveit_configs()
=======
    moveit_config = MoveItConfigsBuilder("rover_arm", package_name="rover_arm_moveit_config").to_moveit_configs()
>>>>>>> main:src/inverse-kinematics/rover_arm_moveit_config/launch/demo.launch.py
    return generate_demo_launch(moveit_config)
