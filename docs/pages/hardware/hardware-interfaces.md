# ROS2 Control: Hardware Interfaces & Controllers

It is recommended to read through the [ros2 control documentation][ros2 control documentation] along side this.
This is just meant to be an introduction.

## Why

Hardware interfaces & controller interfaces help abstract over interactions with hardware,
while helping to organizing our code to keep it clean and modularized.

The general idea behind them is that you want to split the logic and the control of the hardware into two.
The hardware interface handles communicating with the hardware, and the controller handles the logic.

An example would be a light, which is able to take different commands like "set colour", "blink", "rainbow", etc.\
The hardware interface only takes RGB values from the controller and sends them to the light,
and then the controller will handle the logic like deciding when to turn on/off,
when to change colours, etc. and then send RGB values to the hardware interface.

Hardware interfaces also allow us to describe the hardware itself inside the URDF, as well as provide parameters to the hardware interface like the min/max
values for a joint, etc.
Controllers can also be provided parameters via a YAML file.

## Writing

You will primarily need to write hardware interfaces, however it may sometimes also be necessary to write hardware controllers.

When writing either a hardware interface or a controller interface, you must keep in mind that this code is realtime code.
That means:

- Do not block, ever. Avoid mutexes, locks, and other things that can block.
  In order to avoid blocking, take advantage of the :package:`realtime_tools` package: use `#!cpp realtime_tools::RealtimePublisher<T>`,
  `#!cpp realtime_tools::RealtimeThreadSafeBox<T>`, etc.
- Write code with performance in mind.
- Don't forget to add *useful* log messages. These can help significantly if you need to debug something later. (But don't overdo it)

It is also generally good-practice to run the hardware interface & controller interface with asan & ubsan in development to ensure you did not do anything you
shouldn't have.
By default, hardware interfaces and controller interfaces must be compiled *without* any sanitizers enabled.
This is because, by default, if you compile them with a sanitizer, the controller manager will fail because asan was not the first library loaded.\
In order to be able to run it with sanitizers enabled, first compile with the `FORCE_SANITIZERS` CMake option.
Then, you will need to `LD_PRELOAD` asan in controller manager.

It is recommended that you reference existing implementations in this repository while writing a new hardware interface or controller interface.

### Hardware Interfaces

Writing a hardware interface is relatively straight forward.
It is recommended you read through the [ros2 control documentation on creating a hardware interface][writing a hardware component].

!!! note inline end "Diagnostics"

    Depending on what the hardware interface is for, you might want to add a `#!cpp diagnostic_updater::Updater`.\
    You can use the diagnostic updater to report things such as temperature, voltage, etc.
    This can be recorded and later analyzed if something goes wrong.
    <!--
    ~~In addition to `diagnostic_updater::Updater`, you can also publish a :msg:`control_msgs/HardwareStatus` by using
    a [framework-managed publisher][framework-managed publisher]~~.
    Framework-managed publishers are not available in the ROS distro we are currently using.
    They are only available starting in ROS2 Kilted. When we eventually move to Lyrical, this can be uncommented.
    -->

The primary goal of hardware interfaces is to act as a bridge between the controller and the hardware.
Hardware interfaces should not have any complex logic, other than what is strictly necessary to communicate with the hardware.
They should essentially just take commands from the controller and then send them to the hardware.

Here are some additional things to keep in mind:

- When deactivating, make sure to stop things as necessary (e.g. if you are writing a hardware interface for a motor or other things that move, make sure to
  stop them)
- Make sure to check for and handle any hardware errors. You don't want scenarios where your hardware enters into a bad state,
  but you continue like it's perfectly fine.

#### Parameters

If you have any things that you would like to be configurable for the hardware interface, you want it to be able to be configured from the URDF.\
This can be done by using `#!cpp params.hardware_info.hardware_parameters` from `#!cpp hardware_interface::HardwareComponentInterfaceParams`,
which gives you an `#!cpp std::unordered_map<std::string, std::string>` of all the `#!xml <param>` tags specified on the `#!xml <hardware>` tag in the URDF.
Each `#!xml <joint>` tag can also have `#!xml <param>` tags, which you can get by iterating through `#!cpp params.hardware_info.joints` and using
`#!cpp joint.parameters`.\
It's best to use parameters to avoid hardcoding things.
Parameters are useful for things that might change at some point in the future, such as:

- the CAN Bus path used for SocketCAN
- the CAN Bus device ID
- etc.

For parsing parameters, the `scrb_common_util` package provides several utility functions, which you can use:

- Float parsing:
    - `#!cpp template <Stringish S> double scrb::common_util::parse_double(const S& input)`
    - `#!cpp template <Stringish S> float scrb::common_util::parse_float(const S& input)`
- Integer parsing:
    - `#!cpp template <Stringish S> int8_t scrb::common_util::parse_int8(S& input, int base = 10)`
    - `#!cpp template <Stringish S> uint8_t scrb::common_util::parse_uint8(S& input, int base = 10)`
    - `#!cpp template <Stringish S> int16_t scrb::common_util::parse_int16(S& input, int base = 10)`
    - `#!cpp template <Stringish S> uint16_t scrb::common_util::parse_uint16(S& input, int base = 10)`
    - `#!cpp template <Stringish S> int32_t scrb::common_util::parse_int32(S& input, int base = 10)`
    - `#!cpp template <Stringish S> uint32_t scrb::common_util::parse_uint32(S& input, int base = 10)`
    - `#!cpp template <Stringish S> int64_t scrb::common_util::parse_int64(S& input, int base = 10)`
    - `#!cpp template <Stringish S> uint64_t scrb::common_util::parse_uint64(S& input, int base = 10)`
- Other:
    - `#!cpp template <Stringish S> bool scrb::common_util::parse_bool(const S& input)`
    - `#!cpp template <Stringish S> std::chrono::nanoseconds scrb::common_util::parse_duration(const S& input)`
    - `#!cpp template <typename T> std::vector<T> scrb::common_util::parse_array(const std::string& array_string)`

### Controller Interfaces

!!! tip inline start "Using Topics"

    Not all controller interfaces need to subscribe to a topic!\
    Consider what your controller is going to be used for.\
    Depending on the usecase, it might be more appropriate for your controller to instead use a service (or maybe even an action!) instead of a topic.

It is less frequent that you will be writing a ROS controller, as most of the ones we need are written for us,
however there are a few scenarios where it will be necessary for us to write our own.
These occur primarily when we want some non-standard controller, such as a controller for the science payload or a controller for the status indicator light.

When implementing the controller interface, your goal should be to handle all of the high-level logic and then forward the commands to the hardware interface.
This allows us to decouple the logic from the hardware and be able to change the logic without needing to change the hardware interface and vice-versa.

Here are some additional things to keep in mind when writing a controller interface:

- When deactivating, make sure to stop things as necessary (e.g. if you are writing a hardware interface for a motor or other things that move,
  make sure to stop them) and reset any internal state
- When you receive a message in a callback, that is not necessarily on the main thread. Make sure you don't have any threading issues.
- Do not assume that the interfaces are in the order you expect them to be.
- Try to avoid making assumptions about what kind of hardware will use the controller interface.
  The goal of writing a controller interface is that if we ever need to change how it communicates with the hardware (e.g. use a different motor controller,
  change the CAN Bus message format, switch to PWM or serial instead of CAN Bus, etc.),
  then the controller interface does not need to have any changes made to it.\
  Try to keep things decoupled from what the hardware is doing.

## Debugging

Debugging hardware interfaces & controller interfaces is a metric pain in the ass.

Try reading this, maybe it will help.
https://control.ros.org/master/doc/ros2_control/doc/debugging.html

One day, I (Will Free) will write down more documentation to help with debugging. Until then, if you encounter crashes, have fun :)


[ros2 control documentation]: https://control.ros.org/master/doc/getting_started/getting_started.html
[writing a hardware component]: https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html "Writing a new hardware component"
[framework-managed publisher]: https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html#framework-managed-publisher
