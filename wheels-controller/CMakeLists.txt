cmake_minimum_required(VERSION 3.22)
project(wheels_controller)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp
        std_msgs
        geometry_msgs
        action_msgs
        builtin_interfaces
        ros2_fmt_logger
)

## COMPILE

include_directories(include)

scrb_add_executable(${PROJECT_NAME}_node
        src/wheels_controller_node.cpp
        src/rev_motor_controller.cpp
        src/can_controller.cpp
        src/can_controller.h
        src/rev_motor_controller.h
        src/wheels_controller_node.h
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE ${DEPENDENCY_TARGETS})

target_precompile_headers(${PROJECT_NAME}_node PRIVATE
        <rclcpp/rclcpp.hpp>
        <ros2_fmt_logger/logger.hpp>
)

## INSTALL

install(
        TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORT

ament_package()
