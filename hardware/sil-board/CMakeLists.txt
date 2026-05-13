cmake_minimum_required(VERSION 3.22)
project(sil_board)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 26)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

## DEPENDENCIES

set(DEPENDENCIES
        rclcpp

        sensor_msgs

        can_util
        ros2_fmt_logger
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

# TODO 2026-05-13 (Will Free): get rid of LedCommand message
rosidl_generate_interfaces(${PROJECT_NAME} "msg/LedCommand.msg")

add_executable(${PROJECT_NAME}_node
        src/sil_board_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC ${DEPENDENCIES})

target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# TODO 2026-05-13 (Will Free): remove this entirely, and get rid of the LedCommand message
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${cpp_typesupport_target})

add_executable(compat_board_node
        src/compat_board_node.cpp
)

ament_target_dependencies(compat_board_node PUBLIC ${DEPENDENCIES})

target_include_directories(compat_board_node PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

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

ament_package()
