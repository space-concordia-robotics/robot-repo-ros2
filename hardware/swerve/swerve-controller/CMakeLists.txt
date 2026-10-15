cmake_minimum_required(VERSION 3.22)
project(swerve_controller)

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
        control_toolbox
        controller_interface
        realtime_tools
        pluginlib
        tf2
        angles
        ros2_fmt_logger
        scrb_common_util

        # messages
        control_msgs
        geometry_msgs
        nav_msgs
        tf2_msgs
        tf2_geometry_msgs
)

find_package(generate_parameter_library REQUIRED)

## COMPILE

get_target_property(TB_INCLUDE_DIRS control_toolbox::rate_limiter_parameters INTERFACE_INCLUDE_DIRECTORIES)
generate_parameter_library(${PROJECT_NAME}_parameters
        src/${PROJECT_NAME}_parameters.yaml
        ${TB_INCLUDE_DIRS}/control_toolbox/custom_validators.hpp
)

scrb_add_library(${PROJECT_NAME} SHARED
        src/swerve_drive.cpp
        src/swerve_kinematics.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCY_TARGETS} ${PROJECT_NAME}_parameters)

scrb_target_precompile_headers(${PROJECT_NAME} PRIVATE
        <rclcpp/rclcpp.hpp>
        <controller_interface/controller_interface.hpp>
        <realtime_tools/realtime_buffer.hpp>
        <ros2_fmt_logger/logger.hpp>
)

# Export hardware plugins
pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}.xml)

## INSTALL

install(
        DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_parameters
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORTS

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
