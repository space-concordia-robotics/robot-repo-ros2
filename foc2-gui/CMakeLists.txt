cmake_minimum_required(VERSION 3.22)
project(foc2_gui)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 26)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

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

        geometry_msgs
        sensor_msgs
        nav2_msgs
        action_msgs
        lifecycle_msgs
        ros_aruco_opencv_msgs
        rover_msgs

        GeographicLib

        ros2_fmt_logger

        OpenCV
)

include(../cmake/CPM.cmake)

# find dependencies
CPMAddPackage("gh:Neargye/magic_enum#v0.9.7")

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(magic_enum REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Freetype REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

set(DEPENDENCY_TARGETS "")
foreach (dependency IN LISTS DEPENDENCIES)
    list(APPEND DEPENDENCY_TARGETS ${${dependency}_targets})
endforeach ()

list(APPEND DEPENDENCY_TARGETS OpenGL::GL)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/im_application.cpp
        src/widgets/video_widget.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

target_link_libraries(${PROJECT_NAME} PUBLIC
        OpenGL::GL
        SDL3::SDL3
        Freetype::Freetype
        imgui
        implot
        implot3d
        imfonts
        stb
        magic_enum
)

# fixes issue with mold
# TODO 2026-05-02 (Will Free): remove
target_link_options(${PROJECT_NAME} PUBLIC -static-libgcc -static-libstdc++)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${IMGUI_HEADERS}
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
