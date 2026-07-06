cmake_minimum_required(VERSION 3.22)
project(absenc_interface)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp
        std_msgs
        sensor_msgs
        ros2_fmt_logger
)

## COMPILE

scrb_add_executable(absenc_node src/absenc_node.cpp src/absenc.cpp)

target_link_libraries(absenc_node PUBLIC ${DEPENDENCY_TARGETS})

target_precompile_headers(absenc_node PRIVATE
        <rclcpp/rclcpp.hpp>
        <ros2_fmt_logger/logger.hpp>
)

## INSTALL

install(
        TARGETS absenc_node
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORTS

ament_package()
