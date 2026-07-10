cmake_minimum_required(VERSION 3.22)
project(tm_imu)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        tf2
        tf2_ros
        ros2_fmt_logger
)

## COMPILE

scrb_add_executable(transducer_m_imu
        src/tm_imu_node.cpp

        # serialib
        src/serialib/serialib.cpp

        # EasyProfile
        src/EasyProfile/EasyObjectDictionary.cpp
        src/EasyProfile/EasyProfile.cpp
        src/EasyProfile/EasyProtocol.cpp
)

target_include_directories(transducer_m_imu PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(transducer_m_imu PUBLIC ${DEPENDENCY_TARGETS})

target_precompile_headers(transducer_m_imu PRIVATE
        <rclcpp/rclcpp.hpp>
        <ros2_fmt_logger/logger.hpp>
)

## INSTALL

install(
        TARGETS transducer_m_imu
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install launch files.
install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/
)

## EXPORT

ament_package()
