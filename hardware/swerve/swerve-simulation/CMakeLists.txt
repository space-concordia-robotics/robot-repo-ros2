# CMake build file for swerve_simulation
# Installs simulation configuration, launch files, URDF/Xacro models, and Gazebo worlds
cmake_minimum_required(VERSION 3.22)
project(swerve_simulation)

find_package(ament_cmake REQUIRED)

# Copy simulation assets into share/swerve_simulation/ for ros2 launch and Gazebo lookup
install(
        DIRECTORY config launch urdf worlds
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
