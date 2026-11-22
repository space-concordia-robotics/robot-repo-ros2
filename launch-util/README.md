# launch_util

The code in this directory is largely based off of [simple_launch](https://github.com/oKermorgant/simple_launch), as well as some code
from [launch_ext](https://github.com/Greenroom-Robotics/launch_ext).

## Simple Launcher

The `SimpleLauncher` can help significantly shorten any launch files you need to write.

### Without `SimpleLauncher`

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_params = PathJoinSubstitution([FindPackageShare("rover_description"), "config", "joystick.yml"])

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params],
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params],
        remappings=[("/cmd_vel", "/cmd_vel_joy")],
    )

    # TODO 2026-03-09 (Will Free): move this somewhere better
    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        remappings=[
            ("/cmd_vel_in", "/diff_drive_base_controller/cmd_vel_unstamped"),
            ("/cmd_vel_out", "/diff_drive_base_controller/cmd_vel"),
        ],
    )

    return LaunchDescription([joy_node, teleop_twist_joy_node, twist_stamper_node])

```

### With `SimpleLauncher`

```python
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

<!-- TODO 2026-11-21 (Will Free): Add usage section -->

## Substitutions

launch_util also offers the following substitutions for convenience:

- `Xacro`: Processes a file with xacro and returns the result as a string, optionally stripping any comments.
- `WriteTempFile`: Writes a substitution to a temporary file, deleting it on exit.
- `OpaqueFunctionSubstitution`: Executes an opaque function.
- `Templated`: Performs template-based substitution on the provided substitutions.
