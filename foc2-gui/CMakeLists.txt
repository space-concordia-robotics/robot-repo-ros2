cmake_minimum_required(VERSION 3.22)
project(foc2_gui)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 26)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

## DEPENDENCIES

# statically link SDL
set(SDL_STATIC ON)
set(SDL_SHARED OFF)

# I'm compiling SDL3 from source so I don't need this
# find_package(SDL3 REQUIRED)

add_subdirectory(external)

set(DEPENDENCIES
        rclcpp
        rclcpp_components
        rclcpp_action

        tf2_ros
        tf2_eigen
        image_transport
        ros2_fmt_logger

        fmt
        X11
        OpenCV
        OpenGL
        Freetype
        GeographicLib
        magic_enum

        geometry_msgs
        tf2_geometry_msgs
        sensor_msgs
        nav2_msgs
        action_msgs
        lifecycle_msgs
        ros_aruco_opencv_msgs
        rover_msgs
)

include(../cmake/CPM.cmake)

# find dependencies
CPMAddPackage("gh:fmtlib/fmt#11.2.0")
CPMAddPackage("gh:Neargye/magic_enum#v0.9.7")

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_RTSP REQUIRED
        gstreamer-sdp-1.0
        gstreamer-net-1.0
        gstreamer-rtp-1.0
        gstreamer-rtsp-1.0
)

list(APPEND DEPENDENCIES GSTREAMER GSTREAMER_APP GSTREAMER_RTSP)

find_package(Boost REQUIRED CONFIG COMPONENTS)

list(APPEND DEPENDENCIES Boost)

## TARGETS

# TODO 2026-05-05 (Will Free): split these out into multiple libraries to improve compile times.

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/im_application.cpp

        # overlays
        src/overlays/aruco_video_overlay.cpp
        src/overlays/minimap.cpp
        src/overlays/video_stats_overlay.cpp

        # widgets
        src/widgets/logs_widget.cpp
        src/widgets/video_widget.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

target_link_libraries(${PROJECT_NAME} PUBLIC
        magic_enum::magic_enum
        SDL3::SDL3
        imgui
        implot
        implot3d
        imfonts
        stb
)

# fixes issue with mold
# TODO 2026-05-02 (Will Free): remove
target_link_options(${PROJECT_NAME} PUBLIC -static-libgcc -static-libstdc++)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

## INSTALL

install(
        DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_${PROJECT_NAME})
ament_package()
