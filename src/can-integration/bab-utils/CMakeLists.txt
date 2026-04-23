cmake_minimum_required(VERSION 3.8)
project(bab-utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_fmt_logger REQUIRED)
find_package(can-utils REQUIRED)

include_directories(include)

add_executable(can_node
  src/can_controller_node.cpp
  
)

#rosidl_target_interfaces(wheels_controller_node
#        ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(
    can_node
    rclcpp 
    geometry_msgs  
    sensor_msgs
  std_msgs
  can-utils
)

install(TARGETS
        can_node
        DESTINATION lib/${PROJECT_NAME})
ament_package()

