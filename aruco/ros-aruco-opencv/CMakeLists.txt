cmake_minimum_required(VERSION 3.22)
project(ros_aruco_opencv)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

cpm()

CPMAddPackage("gh:Neargye/magic_enum#v0.9.7")

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp_components
        rclcpp_lifecycle

        ros_aruco_opencv_msgs
        tf2_geometry_msgs

        "OpenCV ${OpenCV_VERSION} REQUIRED COMPONENTS aruco"

        cv_bridge
        tf2_ros
        image_transport
        ros2_fmt_logger
        yaml-cpp
        magic_enum
)

list(APPEND DEPENDENCY_TARGETS ${YAML_CPP_LIBRARIES})
list(APPEND DEPENDENCY_TARGETS opencv_aruco)
list(APPEND DEPENDENCY_TARGETS magic_enum::magic_enum)

## COMPILE

scrb_add_library(${PROJECT_NAME} SHARED
        src/aruco_tracker.cpp
        src/detector.cpp
        src/board_loader.cpp
        src/parameters.cpp
        src/utils.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCY_TARGETS})

target_precompile_headers(${PROJECT_NAME} PRIVATE
        <rclcpp/rclcpp.hpp>
        <opencv2/opencv.hpp>
        <yaml-cpp/yaml.h>
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "ros_aruco_opencv::ArucoTracker"
        EXECUTABLE "aruco_tracker"
)
rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "ros_aruco_opencv::ArucoTrackerAutostart"
        EXECUTABLE "aruco_tracker_autostart"
)

## INSTALL

install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        PROGRAMS scripts/create_board scripts/create_marker
        DESTINATION lib/${PROJECT_NAME}
)

## TESTING

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

## EXPORTS

ament_package()
