cmake_minimum_required(VERSION 3.22)
project(tm_imu)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 26)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(transducer_m_imu
        src/tm_imu_node.cpp

        # serialib
        src/serialib/serialib.cpp

        # EasyProfile
        src/EasyProfile/EasyObjectDictionary.cpp
        src/EasyProfile/EasyProfile.cpp
        src/EasyProfile/EasyProtocol.cpp
)

target_compile_features(transducer_m_imu PUBLIC cxx_std_23)

target_include_directories(transducer_m_imu PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(transducer_m_imu
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        tf2
        tf2_ros
)

install(
        TARGETS transducer_m_imu
        DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/
)

# ament_export_include_directories(include)

ament_package()
