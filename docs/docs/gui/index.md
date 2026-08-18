# GUI

For the GUI, we built our own custom solution in-house called FOC2, found in the `foc2-gui/` directory.\
This is our C2 station from which we control the robot during missions, but also during testing.

We use [imgui](https://github.com/ocornut/imgui/) with the [SDL3](https://wiki.libsdl.org/SDL3/FrontPage) & OpenGL backend.\
imgui does most of the work in terms of abstracting over OpenGL, however there in some cases such as for the [Map Widget](./map.md) or
the [Video Stream Widget](./video.md).

*[C2]: Command & Control
