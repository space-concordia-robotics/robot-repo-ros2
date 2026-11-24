cmake_minimum_required(VERSION 3.22)
project(rover_arm_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
        DIRECTORY urdf launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()

