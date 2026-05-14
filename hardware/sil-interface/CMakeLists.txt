cmake_minimum_required(VERSION 3.22)
project(sil_interface)

set(CMAKE_CXX_STANDARD 23)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
set(DEPENDENCIES
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        hardware_interface
        controller_interface
        realtime_tools
        diagnostic_updater
        diagnostic_msgs
        pluginlib
        ros2_fmt_logger
        can_util
        rover_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

## COMPILE

add_library(${PROJECT_NAME} SHARED
        src/sil_interface.cpp
        src/sil_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface sil_interface.xml)
pluginlib_export_plugin_description_file(controller_interface sil_interface.xml)

## INSTALL

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
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
