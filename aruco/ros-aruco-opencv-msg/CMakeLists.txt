cmake_minimum_required(VERSION 3.22)
project(ros_aruco_opencv_msgs)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_dependencies(
        std_msgs
        geometry_msgs
)

## COMPILE

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/ArucoDetections.msg"
        "msg/ArucoMarker.msg"
        "msg/BoardPose.msg"
        "msg/RejectedMarker.msg"
        DEPENDENCIES ${DEPENDENCIES}
)

## TESTING

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

## EXPORTS

ament_package()
