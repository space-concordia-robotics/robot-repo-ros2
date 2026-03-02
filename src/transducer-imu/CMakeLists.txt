cmake_minimum_required(VERSION 3.8)
project(tm_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include external/serialib external/EasyProfile)

add_executable(transducer_m_imu 
    src/tm_imu_node.cpp
    external/serialib/serialib.cpp
    external/EasyProfile/EasyObjectDictionary.cpp
    external/EasyProfile/EasyProfile.cpp
    external/EasyProfile/EasyProtocol.cpp
    )

ament_target_dependencies(transducer_m_imu
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
  )

install(TARGETS transducer_m_imu
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# ament_export_include_directories(include)

ament_package()
