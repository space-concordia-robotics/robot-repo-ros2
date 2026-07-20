cmake_minimum_required(VERSION 3.22)
project(rtsp_camera)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

CPM()

CPMAddFmt()
CPMAddMagicEnum()

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        sensor_msgs
        image_transport
        cv_bridge
        camera_info_manager
        ros2_fmt_logger
        magic_enum
)

list(APPEND DEPENDENCY_TARGETS magic_enum::magic_enum fmt::fmt)

#find_package(generate_parameter_library REQUIRED)
#generate_parameter_library(rtsp_camera_parameters src/rtsp_camera_parameters.yaml)

find_package(PkgConfig REQUIRED)

pkg_check_modules(Gst REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GstApp REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(GstRtspServer REQUIRED IMPORTED_TARGET
        gstreamer-video-1.0
        gstreamer-sdp-1.0
        gstreamer-net-1.0
        gstreamer-rtp-1.0
        gstreamer-rtsp-1.0
        gstreamer-rtsp-server-1.0
)

list(APPEND DEPENDENCY_TARGETS PkgConfig::Gst PkgConfig::GstApp PkgConfig::GstRtspServer)

## COMPILE

scrb_add_executable(${PROJECT_NAME}_node
        src/rtsp_camera_node.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE ${DEPENDENCY_TARGETS})

scrb_target_precompile_headers(${PROJECT_NAME}_node PRIVATE
        <rclcpp/rclcpp.hpp>
        <ros2_fmt_logger/logger.hpp>
        <opencv2/opencv.hpp>
)

## INSTALL

install(
        TARGETS rtsp_camera_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

#install(
#        TARGETS rtsp_camera_node rtsp_camera_parameters
#        DESTINATION ${PROJECT_NAME}Targets
#)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

## PACKAGE

ament_package()
