cmake_minimum_required(VERSION 3.22)
project(sil_board)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_dependencies(
        rclcpp

        sensor_msgs

        can_util
        ros2_fmt_logger
)

## COMPILE

# TODO 2026-05-13 (Will Free): get rid of LedCommand message
rosidl_generate_interfaces(${PROJECT_NAME} "msg/LedCommand.msg")

scrb_add_executable(${PROJECT_NAME}_node
        src/sil_board_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node PUBLIC ${DEPENDENCY_TARGETS})

target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# TODO 2026-05-13 (Will Free): remove this entirely, and get rid of the LedCommand message
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${cpp_typesupport_target})

target_precompile_headers(${PROJECT_NAME}_node PRIVATE
        <rclcpp/rclcpp.hpp>
        <ros2_fmt_logger/logger.hpp>
)

scrb_add_executable(compat_board_node
        src/compat_board_node.cpp
)

target_link_libraries(compat_board_node PUBLIC ${DEPENDENCY_TARGETS})

target_include_directories(compat_board_node PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_precompile_headers(compat_board_node REUSE_FROM ${PROJECT_NAME}_node)

## INSTALL

install(
        TARGETS ${PROJECT_NAME}_node compat_board_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(PROGRAMS
        scripts/sil_board_teleop.py
        scripts/sil_board_joy_teleop.py
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/sil_board_presets.py
        DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS

ament_package()
