cmake_minimum_required(VERSION 3.8)
project(arm_plugin)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  hardware_interface
  rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(
  arm_hardware
  SHARED
  hardware/rover_hardware.cpp
)

target_compile_features(arm_hardware PUBLIC cxx_std_17)
target_include_directories(arm_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/arm_hardware>
)

ament_target_dependencies(arm_hardware
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface arm_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/arm_hardware
)

install(TARGETS arm_hardware
  EXPORT export_arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_arm_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()