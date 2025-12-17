cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Make sure headers are found
include_directories(include)
include_directories(../absenc_interface/include)

# Add the absenc source file for the AbsencDriver implementation
add_library(arm_interface SHARED 
  src/arm_interface.cpp
  ../absenc_interface/src/absenc.cpp
)
ament_target_dependencies(arm_interface
  rclcpp 
  rclcpp_lifecycle
  hardware_interface
  sensor_msgs
  pluginlib
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface rover_hardware_interface.xml)

# --- INSTALL RULES ---

# Install the library
install(TARGETS
  arm_interface
  DESTINATION lib
)

#install files
install(FILES
  rover_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
