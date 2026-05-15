cmake_minimum_required(VERSION 3.22)
project(encoder-boards)

# Placeholder package — encoder-board ROS glue is not implemented yet.
#
# Common mistake that breaks colcon: copying can-utils CMake snippets here.
# The library target `can_util` is defined ONLY in the can-utils package.
# Never call target_include_directories / ament_target_dependencies on `can_util`
# from this CMakeLists.txt.

find_package(ament_cmake REQUIRED)
ament_package()
