cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

add_library(arm_interface SHARED
  src/arm_interface.cpp
  src/absenc.cpp
)

target_include_directories(arm_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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

install(
  TARGETS arm_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

<<<<<<< HEAD
install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(arm_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

=======
>>>>>>> origin/main
ament_package()
