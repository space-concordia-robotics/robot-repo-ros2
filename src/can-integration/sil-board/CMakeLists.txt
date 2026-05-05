cmake_minimum_required(VERSION 3.22)
project(sil_board)

set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can-utils REQUIRED)
find_package(ros2_fmt_logger REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LedCommand.msg"
)

add_executable(sil_board_node
  src/sil_board_node.cpp
)
target_include_directories(sil_board_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(sil_board_node
  rclcpp
  can-utils
  ros2_fmt_logger
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(sil_board_node ${cpp_typesupport_target})

install(TARGETS sil_board_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sil_board_teleop.py
  scripts/sil_board_joy_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/sil_board_presets.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
