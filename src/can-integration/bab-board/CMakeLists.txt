cmake_minimum_required(VERSION 3.22)
project(bab-board)
set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
    rclcpp
    can_utils
    ros2_fmt_logger
    diagnostic_updater
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(backward_ros REQUIRED)

foreach(dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach()

add_library(bab_board SHARED
    src/battery_board.cpp
    src/produce_diagnostics.cpp
)

target_include_directories(bab_board PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bab_board PUBLIC ${DEPENDENCIES})

install(DIRECTORY include/
    DESTINATION include
)

install(TARGETS bab_board
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Standalone telemetry node: prints BAB battery + rail info on the ROS log.
add_executable(bab_telemetry_node src/bab_telemetry_node.cpp)
target_link_libraries(bab_telemetry_node PUBLIC bab_board)
ament_target_dependencies(bab_telemetry_node PUBLIC ${DEPENDENCIES})

install(TARGETS bab_telemetry_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories("include")
ament_export_libraries(bab_board)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_package()