cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)


add_executable(arm_interface src/arm_interface.cpp)
ament_target_dependencies(arm_interface
  rclcpp 
  hardware_interface
  sensor_msgs
)


install(TARGETS

  arm_interface
  DESTINATION lib/${PROJECT_NAME}
)




ament_package()
