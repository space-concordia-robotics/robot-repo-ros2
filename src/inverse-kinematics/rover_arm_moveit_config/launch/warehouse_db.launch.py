from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
<<<<<<< HEAD:src/ceres_moveit_config/launch/warehouse_db.launch.py
    moveit_config = MoveItConfigsBuilder("ceres_rover", package_name="ceres_moveit_config").to_moveit_configs()
=======
    moveit_config = MoveItConfigsBuilder("rover_arm", package_name="rover_arm_moveit_config").to_moveit_configs()
>>>>>>> main:src/inverse-kinematics/rover_arm_moveit_config/launch/warehouse_db.launch.py
    return generate_warehouse_db_launch(moveit_config)
