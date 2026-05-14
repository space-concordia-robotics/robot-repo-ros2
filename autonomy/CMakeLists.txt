cmake_minimum_required(VERSION 3.22)
project(autonomy)

if (POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-fcoroutines)
endif ()

set(DEPENDENCIES
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
        rover_msgs

        GeographicLib

        ros2_fmt_logger
)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location on Ubuntu
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")

include(../cmake/CPM.cmake)

# find dependencies
CPMAddPackage("gh:Neargye/magic_enum#v0.9.7")

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(magic_enum REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

set(DEPENDENCY_TARGETS "")
foreach (dependency IN LISTS DEPENDENCIES)
    list(APPEND DEPENDENCY_TARGETS ${${dependency}_targets})
endforeach ()

add_library(autonomy
        src/autonomy.cpp
        src/targets/ar_target.cpp
        src/targets/gps_target.cpp
        src/targets/object_target.cpp
)
add_library(autonomy::autonomy ALIAS autonomy)

target_compile_features(autonomy PUBLIC cxx_std_23)

target_include_directories(
        autonomy PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
        autonomy PUBLIC
        ${DEPENDENCIES}
)

target_link_libraries(
        autonomy PRIVATE
        magic_enum::magic_enum
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
        TARGETS autonomy
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORTS
#ament_export_include_directories("include/${PROJECT_NAME}")
#ament_export_libraries(autonomy)
#ament_export_targets(export_${PROJECT_NAME})
ament_package()
