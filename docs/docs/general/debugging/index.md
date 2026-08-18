# Debugging

## General

When debugging in ROS, there are a couple of tools that are generally helpful:

- `rqt`: lets you visualize some information like:
    - tf trees
    - node/topic graph
    - plot data from a topic
    - & more
- `ros2 topic` commands: list topics, show topic output, get topic type, public topic, get topic bandwidth, etc.
- `ros2 node list` and `ros2 node info`: show which nodes are running and get some basic information about a node
  (subscribers, publishers, services, actions, etc.)
- `ros2 service` & `ros2 action` commands: interact with services & actions from the command line

## Python

See [Python Debugging](./python.md)

## C++

See [C++ Debugging](./cpp.md)
