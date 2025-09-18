cmake_minimum_required(VERSION 3.8)
project(joy_mux_controller_cpp)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executable
add_executable(joy_mux src/joy_mux_controller.cpp)
ament_target_dependencies(joy_mux
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install target
install(TARGETS
  joy_mux
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()