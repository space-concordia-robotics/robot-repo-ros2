# ROS workspace Repo for the Rover
## Connecting to Rover Jetson
`ssh nvidia@10.240.0.10`
## LiDAR
The make of our LiDAR is Ouster and they provide us with the ros ouster driver [ouster-ros](https://github.com/ouster-lidar/ouster-ros/tree/ros2). The Python launch file [lidar_launch](./launch_files/lidar_launch.py) will create a the ouster driver node.
