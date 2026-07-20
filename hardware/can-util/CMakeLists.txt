cmake_minimum_required(VERSION 3.22)
project(can_util)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_dependencies(
        rclcpp
        ros2_fmt_logger
)

## COMPILE

scrb_add_library(
        can_util
        SHARED
        src/can_util.cpp
        src/can_controller.cpp
)
add_library(can_util::can_util ALIAS can_util)

target_include_directories(
        can_util PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCY_TARGETS})

scrb_target_precompile_headers(${PROJECT_NAME} PRIVATE
        <rclcpp/logging.hpp>
        <rclcpp/macros.hpp>
        <ros2_fmt_logger/logger.hpp>
)

## INSTALL

install(
        DIRECTORY include/
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

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(can_util)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
