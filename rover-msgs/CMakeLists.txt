cmake_minimum_required(VERSION 3.8)
project(rover_msgs)

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
        "msg/ImageDetection.msg"
        "msg/ImageDetectionArray.msg"
        DEPENDENCIES ${DEPENDENCIES}
)

ament_package()
