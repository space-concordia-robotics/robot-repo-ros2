cmake_minimum_required(VERSION 3.22)
project(autonomy)

find_package(scrb_cmake_utils REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-fcoroutines)
endif ()

## DEPENDENCIES

# Geographiclib installs FindGeographicLib.cmake to this non-standard location on Ubuntu
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")

cpm()

CPMAddPackage("gh:Neargye/magic_enum#v0.9.7")

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp
        rclcpp_components
        rclcpp_action
        rclcpp_async

        geometry_msgs
        sensor_msgs
        nav2_msgs
        action_msgs
        lifecycle_msgs
        ros_aruco_opencv_msgs
        tf2_geometry_msgs
        rover_msgs

        GeographicLib
        magic_enum

        ros2_fmt_logger
        scrb_common_util
)

list(APPEND DEPENDENCY_TARGETS magic_enum::magic_enum)

## COMPILE

scrb_add_executable(${PROJECT_NAME}
        src/autonomy.cpp
        src/targets/ar_target.cpp
        src/targets/gps_target.cpp
        src/targets/object_target.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCY_TARGETS})

target_precompile_headers(${PROJECT_NAME} PRIVATE
        <rclcpp/rclcpp.hpp>
        <rclcpp_action/rclcpp_action.hpp>
        <rclcpp_async/rclcpp_async.hpp>
        <fmt/format.h>
        # I (Will) profiled it, and these ones are the messages that contribute most significantly to the compile time
        <nav2_msgs/action/compute_path_through_poses.hpp>
        <nav2_msgs/action/follow_gps_waypoints.hpp>
        <nav2_msgs/action/follow_waypoints.hpp>
        <nav2_msgs/action/navigate_through_poses.hpp>
        <nav2_msgs/action/follow_path.hpp>
        <nav2_msgs/action/navigate_to_pose.hpp>
        <nav2_msgs/action/compute_path_to_pose.hpp>
        <ros_aruco_opencv_msgs/msg/aruco_detections.hpp>
)

## INSTALL

install(
        DIRECTORY behaviour_trees config launch worlds models
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORTS

ament_package()
