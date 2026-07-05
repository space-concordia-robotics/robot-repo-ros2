cmake_minimum_required(VERSION 3.22)
project(arm_hardware)

if (POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif ()

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        sensor_msgs
        pluginlib
)

## COMPILE

scrb_add_library(arm_interface SHARED
        src/arm_interface.cpp
        src/absenc.cpp
)

target_include_directories(arm_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(arm_interface PUBLIC ${DEPENDENCY_TARGETS})

pluginlib_export_plugin_description_file(hardware_interface rover_hardware_interface.xml)

## INSTALL

install(
        TARGETS arm_interface
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

## EXPORT

ament_export_include_directories(include)
ament_export_libraries(arm_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
