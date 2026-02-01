cmake_minimum_required(VERSION 3.8)
project(ik_mux_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(ik_mux src/ik_mux.cpp)
ament_target_dependencies(ik_mux 
  rclcpp 
  control_msgs 
  moveit_msgs 
  geometry_msgs  
  sensor_msgs 
  std_srvs
)

add_executable(ik_nxt src/ik_mux_nxt.cpp)
ament_target_dependencies(ik_nxt 
  rclcpp 
  control_msgs 
  moveit_msgs 
  geometry_msgs  
  sensor_msgs 
  std_srvs
)

install(TARGETS

  ik_mux
  ik_nxt
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
