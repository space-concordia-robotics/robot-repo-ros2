# Launch Files

ROS launch files are used to launch multiple nodes simultaneously.
You can read more about launch files in the [ROS documentation](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
(Note: please switch to the "Python" tab, as we use Python launch files).

In order to simplify writing launch files, we have written the `launch_util` package.

If you know how to use ROS launch files, then it should be relatively straightforward to use `launch_util`.

Here is an example using `launch_util`:

```py
from launch_util import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    joy_params = sl.params(package="rover_description", file="joystick.yml")

    sl.node("joy", "joy_node", parameters=[joy_params])

    sl.node("teleop_twist_joy", "teleop_node", name="teleop_node", parameters=[joy_params], remappings=[("/cmd_vel", "/cmd_vel_joy")])

    sl.node(
        "twist_stamper",
        remappings=[
            ("/cmd_vel_in", "/diff_drive_base_controller/cmd_vel_unstamped"),
            ("/cmd_vel_out", "/diff_drive_base_controller/cmd_vel"),
        ]
    )

    return sl.launch_description()
```
