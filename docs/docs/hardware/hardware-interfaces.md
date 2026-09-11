# ROS2 Control: Hardware Interfaces & Controllers

It is recommended to read through the [ros2 control documentation][ros2 control documentation] along side this.
This is just meant to be an introduction.

## Why

Hardware interfaces & controllers help abstract over interactions with hardware, and also help with organizing our code keeping it clean and modularized.

The general idea behind them is that you want to split the logic and the control of the hardware into two.
The hardware interface handles communicating with the hardware, and the controller handles the logic.

An example would be a light which is able to take different commands like "blink", "rainbow", etc.\
The hardware interface only sends RGB values directly to the light, and then the controller will handle the logic like deciding when to turn on/off, when to
change colours, etc.

Hardware interfaces also allow us to describe the hardware itself inside the URDF, as well as provide parameters to the hardware interface like the min/max
values for a joint, etc.
Controllers can also be provided parameters via a YAML file.

## Writing

You will primarily need to write hardware interfaces, however it may sometimes also be necessary to write hardware controllers.

### Hardware Interfaces

Writing a hardware interface is relatively straight forward.
It is recommended you read through the [ros2 control documentation on creating a hardware interface][writing a hardware component].

==TODO: finish me==

### Controllers

It is less frequent that you will be writing a ROS controller, as most of the ones we need are written for us,

==TODO: finish me==

## Debugging

Debugging hardware interfaces & controller interfaces is a metric pain in the ass.

Try reading this, maybe it will help.
https://control.ros.org/master/doc/ros2_control/doc/debugging.html

One day, I (Will Free) will write down more documentation to help with debugging. Until then, if you encounter crashes, have fun :)


[ros2 control documentation]: https://control.ros.org/master/doc/getting_started/getting_started.html

[writing a hardware component]: https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html "Writing a new hardware component"
