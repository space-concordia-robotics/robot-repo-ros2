from launch_util import SimpleLauncher


def generate_launch_description():
    """Robot state publisher"""

    sl = SimpleLauncher()

    mappings = {
        "mode": sl.mode_arg,
        "control": sl.control_arg,
    }

    sl.robot_state_publisher(package="rover_description", file="robot.urdf", xacro_args=mappings, strip_comments=True)

    return sl.launch_description()
