cmake_minimum_required(VERSION 3.22)
project(arm_controller)

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

include_directories(include)

scrb_add_executable(${PROJECT_NAME}_node src/arm_controller_node.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node PUBLIC ${DEPENDENCY_TARGETS})

target_precompile_headers(${PROJECT_NAME}_node PRIVATE
        <rclcpp/rclcpp.hpp>
        <ros2_fmt_logger/logger.hpp>
)

## INSTALL

install(
        DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}_node
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORTS

ament_package()
