cmake_minimum_required(VERSION 3.16)
project(wheel_can_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  can-utils
  ros2_fmt_logger
)

set(ODOM_DEPENDENCIES
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

find_package(ament_cmake REQUIRED)
foreach(dep IN LISTS DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()
foreach(dep IN LISTS ODOM_DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()

add_library(wheel_can_interface SHARED
  src/wheel_can_interface.cpp
)

target_include_directories(wheel_can_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(wheel_can_interface ${DEPENDENCIES})

pluginlib_export_plugin_description_file(hardware_interface wheel_can_hardware.xml)

# wheel_odometry_node executable: integrates differential-drive odometry from
# /joint_states and publishes nav_msgs/Odometry + odom->base_link TF.
add_executable(wheel_odometry_node
  src/wheel_odometry_node.cpp
)
target_include_directories(wheel_odometry_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(wheel_odometry_node ${ODOM_DEPENDENCIES})

install(
  TARGETS wheel_can_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS wheel_odometry_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES wheel_can_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(wheel_can_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
