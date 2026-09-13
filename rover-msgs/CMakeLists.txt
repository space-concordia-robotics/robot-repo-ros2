cmake_minimum_required(VERSION 3.22)
project(rover_msgs)

find_package(scrb_cmake_utils)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_dependencies(
        std_msgs
        geometry_msgs
)

## COMPILE

set(MESSAGE_DEFINITIONS
        "msg/ImageDetection.msg"
        "msg/ImageDetectionArray.msg"
)

set(SERVICE_DEFINITIONS
        "srv/SetSILStatus.srv"
)

set(ACTION_DEFINITIONS
        "action/ScienceSpectrometerScan.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MESSAGE_DEFINITIONS}
        ${SERVICE_DEFINITIONS}
        ${ACTION_DEFINITIONS}
        DEPENDENCIES ${DEPENDENCIES}
)

## EXPORT

ament_export_dependencies(rosidl_default_runtime ${DEPENDENCIES})
ament_package()
