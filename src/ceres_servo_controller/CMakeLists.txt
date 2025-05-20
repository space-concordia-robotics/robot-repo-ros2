cmake_minimum_required(VERSION 3.8)
project(ceres_servo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# Install the target
install(TARGETS
  servo_node
  DESTINATION lib/${PROJECT_NAME}
)

install (
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/    

)

# (Optional) environment hooks go here
# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/some_hook.sh.in")

# This MUST be the last macro
ament_package()
