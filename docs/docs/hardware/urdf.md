# URDF

The URDF is used to define the model of the robot, as well as the links between different components.

This is used for things such as [tf2](https://wiki.ros.org/tf2) to let you do things like
"transform this point that is relative to the front camera to a point that is relative to the base link",
which is something that is extremely useful.

In order to generate the URDF, we use [xacro](https://github.com/ros/xacro), an XML macro language.
It allows us to split the URDF across multiple files, as well as to use macros to reuse repeated portions.  
We pass several parameters to xacro, which allows the URDF to be different for different scenarios (e.g. simulation vs production).  
Currently, the two arguments are:

<div class="annotate" markdown>

- `mode`, which is one of:
    - `simulation`: when running in a gazebo simulation
    - `production`: when running on the actual rover
- `control`(1), which is one of:
    - `ros`: when using ros2 control
    - `gazebo`: when using gazebo control (never used)

</div>

1. This should eventually be removed.

The URDF is split across many different files in the following tree:

```text { title="URDF file tree" .annotate }
urdf/
├── robot.urdf(1)
├── utils.urdf(2)
├── body/(3)
│   ├── base.urdf
│   ├── materials.urdf
│   ├── wheels.urdf
│   ├── arm.urdf
│   ├── autonomy-module.urdf
│   └── ffc-mount.urdf
├── sensors/(4)
│   ├── sensors.urdf
│   ├── lidar.urdf
│   ├── depth-camera.urdf
│   ├── cameras.yml(5)
│   ├── ffc-module.urdf
│   └── misc.urdf
└── control/(6)
    ├── gazebo.urdf
    └── ros.urdf
```

1. This is the main entrypoint for the URDF. It doesn't do anything on its own, it just loads other files based on the parameters passed to xacro.
2. Miscellaneous utilities are kept here.
3. The files in this folder model the physical joints & links of the robot, e.g. the wheels, the arm, etc.  
   The `base.urdf` file is the primary entrypoint and includes all the other files.
4. The files in this folder defines the sensors present on the robot.  
   The `sensors.urdf` file is the primary entrypoint and includes all the other files.
5. This file is used to define properties for different camera sensors for the ffc module.
6. The files in this folder define the control system used for the robot.  
   Note that currently, ros control (`ros.urdf`) is the only one ever used, gazebo control is never used.

Everything under `body/` forms all the physical joints & links for our robot, while everything under `sensors/` is all the sensor definitions for our robot.
Under `control/` is urdf for ROS control, currently `gazebo.urdf` is unused and only `ros.urdf` is ever used.
