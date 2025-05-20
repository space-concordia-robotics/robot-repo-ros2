cmake_minimum_required(VERSION 3.22)
project(ceres_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Add your executable/library here (example shown)
add_executable(servo_node src/ceres_servo_control.cpp)
ament_target_dependencies(servo_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  control_msgs
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(TARGETS servo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
