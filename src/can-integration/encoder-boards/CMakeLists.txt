cmake_minimum_required(VERSION 3.22)
project(encoder-boards)

# Placeholder package -- actual encoder-board firmware glue is not implemented
# yet. The previous skeleton referenced src/can_interface.cpp and
# src/system_controller.cpp which don't exist in this directory, breaking the
# whole workspace build. Until the encoder-side work lands, this package
# installs nothing and only declares its dependencies so colcon picks it up
# in topological order.

set(CMAKE_CXX_STANDARD 23)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(DEPENDENCIES
        rclcpp
        sensor_msgs
        geometry_msgs
        ros2_fmt_logger
)

find_package(ament_cmake REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

ament_export_dependencies(${DEPENDENCIES})

ament_package()
