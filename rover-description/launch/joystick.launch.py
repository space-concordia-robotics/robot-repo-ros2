from launch_util import SimpleLauncher


# TODO 2026-02-09 (Will Free): move this to a better package
def generate_launch_description():
    sl = SimpleLauncher()

    joy_params = sl.params(package="rover_description", file="joystick.yml")

    sl.node("joy", "joy_node", parameters=[joy_params])

    sl.node("teleop_twist_joy", "teleop_node", name="teleop_node", parameters=[joy_params], remappings=[("/cmd_vel", "/cmd_vel_joy")])

    # TODO 2026-03-09 (Will Free): move this somewhere better
    sl.node(
        "twist_stamper",
        remappings=[
            ("/cmd_vel_in", "/diff_drive_base_controller/cmd_vel_unstamped"),
            ("/cmd_vel_out", "/diff_drive_base_controller/cmd_vel"),
        ],
    )

    return sl.launch_description()
