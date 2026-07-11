cmake_minimum_required(VERSION 3.22)
project(foc2_gui)

if (POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif ()

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

include(external/dependencies.cmake)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location on Ubuntu
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")

CPM()

CPMAddFmt()
CPMAddMagicEnum()

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp
        rclcpp_components
        rclcpp_action
        ament_index_cpp

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
        CURL
        "Boost REQUIRED CONFIG COMPONENTS"

        geometry_msgs
        tf2_geometry_msgs
        sensor_msgs
        nav2_msgs
        action_msgs
        lifecycle_msgs
        ros_aruco_opencv_msgs
        rover_msgs

        scrb_common_util
)

list(APPEND DEPENDENCY_TARGETS magic_enum::magic_enum)

list(APPEND DEPENDENCY_TARGETS
        SDL3::SDL3

        imgui
        implot
        implot3d
        imfonts

        iconfonts

        lunasvg

        stb
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(Gst REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GstApp REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(GstRtsp REQUIRED IMPORTED_TARGET
        gstreamer-sdp-1.0
        gstreamer-net-1.0
        gstreamer-rtp-1.0
        gstreamer-rtsp-1.0
)

list(APPEND DEPENDENCY_TARGETS PkgConfig::Gst PkgConfig::GstApp PkgConfig::GstRtsp)

set(API_TWEAKS_FILE "${CMAKE_CURRENT_BINARY_DIR}/api-tweaks.txt")
set(GSTREAMER_FIRST_FIXED_VERSION "1.26.11")
if (Gst_VERSION VERSION_LESS GSTREAMER_FIRST_FIXED_VERSION)
    file(WRITE "${API_TWEAKS_FILE}" "
# these api tweaks can be removed when updating to gstreamer >=1.26.11. (aka updating to ubuntu 26.04 or later)
# they're only necessary, because I don't want to force people to compile gstreamer from source.
type	gst_calculate_linear_regression			xy		gpointer
type	gst_calculate_linear_regression			temp		gpointer
type	gst_util_simplify_fraction			numerator		gpointer
type	gst_util_simplify_fraction			denominator		gpointer
type	gst_caps_features_set_parent_refcount			refcount		gpointer
type	gst_meta_deserialize			data		gpointer
type	gst_iterator_new			master_cookie		gpointer
type	gst_structure_set_parent_refcount			refcount		gpointer

# these last two are either due to a bug in peel or in gstreamer's rtsp server. unsure.
skip	gst_rtsp_stream_transport_set_callbacks
skip	gst_rtsp_stream_transport_set_list_callbacks

# gstreamer moment. they forgot to add ref and unref to the gir
ref		GstMessage		gst_message_ref
unref	GstMessage		gst_message_unref

ref		GstSample		gst_sample_ref
unref	GstSample		gst_sample_unref
"
    )
else ()
    file(WRITE "${API_TWEAKS_FILE}" "")
endif ()

peel_generate(Gst 1.0 RECURSIVE TWEAKS "${API_TWEAKS_FILE}")
peel_generate(GstNet 1.0 RECURSIVE TWEAKS "${API_TWEAKS_FILE}")
peel_generate(GstApp 1.0 RECURSIVE TWEAKS "${API_TWEAKS_FILE}")
peel_generate(GstRtsp 1.0 RECURSIVE TWEAKS "${API_TWEAKS_FILE}")
peel_generate(GstRtspServer 1.0 RECURSIVE TWEAKS "${API_TWEAKS_FILE}")

list(APPEND DEPENDENCY_TARGETS
        peel::peel
        peel::Gst
        peel::GstNet
        peel::GstApp
        peel::GstRtsp
        peel::GstRtspServer
)

## COMPILE

# TODO 2026-05-05 (Will Free): split these out into multiple libraries to improve compile times.

scrb_add_executable(${PROJECT_NAME}
        src/main.cpp
        src/im_application.cpp

        # overlays
        src/overlays/aruco_video_overlay.cpp
        src/overlays/minimap.cpp
        src/overlays/nav_path_video_overlay.cpp
        src/overlays/video_stats_overlay.cpp

        # widgets
        src/widgets/logs_widget.cpp
        src/widgets/map_widget.cpp
        src/widgets/video_widget.cpp

        # OSM map stuff
        src/osm/osm_map_plot.cpp
        src/osm/osm_rich_destination_calc_widget.cpp
        src/osm/osm_rich_distance_calc_widget.cpp
        src/osm/osm_rich_map_plot.cpp
        src/osm/osm_rich_mark_editor_widget.cpp
        src/osm/osm_rich_mark_item.cpp
        src/osm/osm_rich_mark_item_widget.cpp
        src/osm/osm_rich_mark_storage.cpp
        src/osm/osm_tile.cpp
        src/osm/osm_tile_grabber.cpp
        src/osm/osm_tile_grabber_url.cpp
        src/osm/osm_tile_grabber_widget.cpp
        src/osm/osm_tile_loader.cpp
        src/osm/osm_tile_loader_url.cpp
        src/osm/osm_tile_saver.cpp
        src/osm/osm_tile_source_async.cpp
        src/osm/osm_tile_source_caching.cpp
        src/osm/osm_tile_source_fs.cpp
        src/osm/osm_tile_source_url.cpp
        src/osm/osm_tile_source_url_impl.cpp
        src/osm/osm_tile_source_widget.cpp
        src/osm/osm_tile_texture.cpp

        # OSM markers
        src/osm/markers/rich_robot_marker.cpp
        src/osm/markers/rich_gps_marker.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCY_TARGETS})

# fixes issue with mold
# TODO 2026-05-02 (Will Free): remove
target_link_options(${PROJECT_NAME} PUBLIC -static-libgcc -static-libstdc++)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)

configure_file(resources/resources.hpp.in ${GENERATED_INCLUDE_DIR}/foc2-gui/resources.hpp @ONLY)

target_include_directories(${PROJECT_NAME} PRIVATE ${GENERATED_INCLUDE_DIR})

set(FONT_AWESOME_PATH "share/resources/fonts/fa-solid-900.ttf")

target_compile_definitions(${PROJECT_NAME} INTERFACE FONT_AWESOME_PATH="${FONT_AWESOME_PATH}")

target_precompile_headers(${PROJECT_NAME} PRIVATE
        <filesystem>
        <rclcpp/rclcpp.hpp>
        <tf2_ros/buffer.hpp>
        <fmt/format.h>
        <ros2_fmt_logger/logger.hpp>
        <Eigen/Core>
        <Eigen/Dense>
        <opencv2/opencv.hpp>
        <opencv2/core/eigen.hpp>
        <imgui.h>
        <imgui_internal.h>
        <implot.h>
        <implot3d.h>
)

## INSTALL

install(
        DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(
        DIRECTORY resources
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*.in" EXCLUDE
)

install(
        FILES ${FONT_AWESOME_SOURCE_PATH}
        DESTINATION share/${PROJECT_NAME}/resources/fonts/
        RENAME fa-solid-900.woff2
)

install(
        TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

## EXPORTS

ament_export_targets(export_${PROJECT_NAME})
ament_package()
