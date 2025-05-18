cmake_minimum_required(VERSION 3.8)
project(ceres_servo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
<<<<<<< HEAD
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(TARGETS 


  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
=======
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ceres_servo src/ceres_servo_control.cpp)
ament_target_dependencies(ceres_servo rclcpp control_msgs sensor_msgs)

install(TARGETS 
  ceres_servo
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
>>>>>>> b32cd26b6f2a40cacf205ca9ad2fe8e26dbac12f
