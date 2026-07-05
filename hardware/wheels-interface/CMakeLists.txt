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
