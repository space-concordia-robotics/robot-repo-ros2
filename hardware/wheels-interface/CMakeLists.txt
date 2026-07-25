cmake_minimum_required(VERSION 3.22)
project(wheels_interface)

if (POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif ()

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)

find_dependencies(
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

## COMPILE

scrb_add_library(
        wheels_interface
        SHARED
        src/wheels_interface.cpp
        src/spark/spark_base.cpp
        src/spark/spark_max.cpp
)

target_include_directories(
        wheels_interface PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(wheels_interface PUBLIC ${DEPENDENCY_TARGETS})

scrb_target_precompile_headers(${PROJECT_NAME} PRIVATE
        <rclcpp/rclcpp.hpp>
        <hardware_interface/hardware_component_interface.hpp>
        <realtime_tools/realtime_buffer.hpp>
        <ros2_fmt_logger/logger.hpp>
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface wheels_interface.xml)

## INSTALL

install(
        DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS wheels_interface
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
