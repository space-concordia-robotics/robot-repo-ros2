cmake_minimum_required(VERSION 3.22)
project(can-utils)

set(CMAKE_CXX_STANDARD 23)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
set(DEPENDENCIES
        rclcpp
        ros2_fmt_logger
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(backward_ros REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

## COMPILE

add_library(
        can_util
        SHARED
        src/can_interface.cpp
        src/system_controller.cpp
)
target_include_directories(can_util
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(can_util PUBLIC ${DEPENDENCIES})

## INSTALL

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS can_util
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(can_util)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()