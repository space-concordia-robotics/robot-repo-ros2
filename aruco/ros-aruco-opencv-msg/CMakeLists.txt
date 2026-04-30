cmake_minimum_required(VERSION 3.5)
project(ros_aruco_opencv_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(DEPENDENCIES
        std_msgs
        geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/ArucoDetections.msg"
        "msg/ArucoMarker.msg"
        "msg/BoardPose.msg"
        "msg/RejectedMarker.msg"
        DEPENDENCIES ${DEPENDENCIES}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
