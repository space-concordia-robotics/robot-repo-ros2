cmake_minimum_required(VERSION 3.22)
project(rover_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(DEPENDENCIES
        std_msgs
        geometry_msgs
        action_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach (dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach ()

set(MESSAGE_DEFINITIONS
        "msg/ImageDetection.msg"
        "msg/ImageDetectionArray.msg"
)

set(SERVICE_DEFINITIONS
        # empty
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

ament_package()
