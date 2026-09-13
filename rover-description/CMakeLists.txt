cmake_minimum_required(VERSION 3.22)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY config launch rviz urdf
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()

