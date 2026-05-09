# URDF

The URDF is used to define the model of the robot, as well as the links between different components.

This is used for things such as [tf2](https://wiki.ros.org/tf2) to let you do things like
"transform this point that is relative to the front camera to a point that is relative to the base link",
which is something that is extremely useful.

The URDF is split across many different files in the following tree:

```
urdf/
├── robot.urdf
├── utils.urdf
├── body/
│   ├── base.urdf
│   ├── materials.urdf
│   ├── wheels.urdf
│   ├── arm.urdf
│   ├── autonomy-module.urdf
│   └── ffc-mount.urdf
├── sensors/
│   ├── lidar.urdf
│   ├── cameras.urdf
│   └── misc.urdf
└── control/
    ├── gazebo.urdf
    └── ros.urdf
```

Everything under `body/` forms all the physical joints & links for our robot, while everything under `sensors/` is all the sensor definitions for our robot.
Under `control/` is urdf for ROS control, currently `gazebo.urdf` is unused and only `ros.urdf` is ever used.
