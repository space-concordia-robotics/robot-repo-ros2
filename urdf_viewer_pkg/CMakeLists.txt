cmake_minimum_required(VERSION 3.8)
project(urdf_viewer_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#find_package(rclcpp REQUIRED)
#find_package(robot_state_publisher REQUIRED)
#find_package(joint_state_publisher_gui REQUIRED)
#find_package(rviz2 REQUIRED)
#find_package(xacro REQUIRED)

install(
  DIRECTORY rviz_configuration launch URDF_Imports
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/urdf_preprocessor.py
  DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_package()
