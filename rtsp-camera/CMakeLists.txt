cmake_minimum_required(VERSION 3.22)
project(rtsp_camera)

set(CMAKE_CXX_STANDARD 23)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

include(../cmake/CPM.cmake)

# find dependencies
CPMAddPackage("gh:fmtlib/fmt#11.2.0")
CPMAddPackage("gh:Neargye/magic_enum#v0.9.7")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
#find_package(fmt REQUIRED)
find_package(ros2_fmt_logger REQUIRED)
find_package(magic_enum REQUIRED)

#find_package(generate_parameter_library REQUIRED)
#generate_parameter_library(rtsp_camera_parameters src/rtsp_camera_parameters.yaml)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED
        gstreamer-1.0
)
pkg_check_modules(GSTREAMER_RTSP_SERVER REQUIRED
        gstreamer-video-1.0
        gstreamer-sdp-1.0
        gstreamer-net-1.0
        gstreamer-rtp-1.0
        gstreamer-rtsp-1.0
        gstreamer-rtsp-server-1.0
)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

include_directories(
        include
        ${GSTREAMER_INCLUDE_DIRS}
        ${GSTREAMER_APP_INCLUDE_DIRS}
        ${GSTREAMER_RTSP_SERVER_INCLUDE_DIRS}
)

add_executable(
        rtsp_camera_node
        src/rtsp_camera_node.cpp
)

target_compile_features(rtsp_camera_node PUBLIC cxx_std_20)

target_include_directories(
        rtsp_camera_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(
        rtsp_camera_node
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        sensor_msgs
        image_transport
        cv_bridge
        camera_info_manager
        ros2_fmt_logger
)

target_link_libraries(
        rtsp_camera_node
        #rtsp_camera_parameters
        ${GSTREAMER_LIBRARIES}
        ${GSTREAMER_APP_LIBRARIES}
        ${GSTREAMER_RTSP_SERVER_LIBRARIES}
        fmt::fmt
        magic_enum::magic_enum
)

install(
        TARGETS rtsp_camera_node
        DESTINATION lib/${PROJECT_NAME}
)

#install(
#        TARGETS rtsp_camera_node rtsp_camera_parameters
#        DESTINATION ${PROJECT_NAME}Targets
#)

install(
        DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
