cmake_minimum_required(VERSION 3.22)
project(wheels_interface)

set(CMAKE_CXX_STANDARD 23)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(DEPENDENCIES
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        hardware_interface
        diagnostic_updater
        diagnostic_msgs
        pluginlib
        ros2_fmt_logger
        scrb_common_util
        can_util
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

## COMPILE

add_library(
        wheels_interface
        SHARED
        src/wheels_interface.cpp
        src/spark/spark_base.cpp
        src/spark/spark_max.cpp
)

target_compile_features(wheels_interface PUBLIC cxx_std_23)

target_include_directories(
        wheels_interface PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
        wheels_interface PUBLIC
        ${DEPENDENCIES}
)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
try_compile(HAS_JAZZY_HARDWARE_INTERFACE ${CMAKE_BINARY_DIR}/is_jazzy
        SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/has_jazzy_hardware_interface.cpp
        CXX_STANDARD 20 LINK_LIBRARIES hardware_interface::hardware_interface
        OUTPUT_VARIABLE COMPILE_LOG
)

if (HAS_JAZZY_HARDWARE_INTERFACE)
    target_compile_definitions(wheels_interface PUBLIC HARDWARE_INTERFACE_IS_JAZZY=1)
else ()
    target_compile_definitions(wheels_interface PUBLIC HARDWARE_INTERFACE_IS_JAZZY=0)
endif ()


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface wheels_interface.xml)

## INSTALL

install(
        DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS wheels_interface
        EXPORT export_wheels_interface
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_wheels_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
