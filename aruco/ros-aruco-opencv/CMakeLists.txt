cmake_minimum_required(VERSION 3.5)
project(ros_aruco_opencv)

if (POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(DEPENDENCIES
        rclcpp
        rclcpp_components
        rclcpp_lifecycle

        ros_aruco_opencv_msgs
        tf2_geometry_msgs

        cv_bridge
        tf2_ros
        image_transport
        ros2_fmt_logger
        yaml-cpp
        magic_enum
)

include(../../cmake/CPM.cmake)

# find dependencies
CPMAddPackage("gh:Neargye/magic_enum#v0.9.7")

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
#find_package(magic_enum REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

find_package(OpenCV ${OpenCV_VERSION} REQUIRED COMPONENTS aruco)

set(DEPENDENCY_TARGETS "")
foreach (dependency IN LISTS DEPENDENCIES)
    list(APPEND DEPENDENCY_TARGETS ${${dependency}_TARGETS})
endforeach ()

list(APPEND DEPENDENCY_TARGETS ${YAML_CPP_LIBRARIES})
list(APPEND DEPENDENCY_TARGETS opencv_aruco)
list(APPEND DEPENDENCY_TARGETS magic_enum::magic_enum)

add_library(${PROJECT_NAME} SHARED
        src/aruco_tracker.cpp
        src/detector.cpp
        src/board_loader.cpp
        src/parameters.cpp
        src/utils.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

target_include_directories(${PROJECT_NAME} PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCY_TARGETS})

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "aruco_opencv::ArucoTracker"
        EXECUTABLE "aruco_tracker"
)
rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "aruco_opencv::ArucoTrackerAutostart"
        EXECUTABLE "aruco_tracker_autostart"
)

install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY
        config
        launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        PROGRAMS
        scripts/create_board
        scripts/create_marker
        DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
