cmake_minimum_required(VERSION 3.22)

project(scrb_cmake_utils NONE)

find_package(ament_cmake REQUIRED)

ament_package(
        CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)

install(
        DIRECTORY cmake
        DESTINATION share/${PROJECT_NAME}
)
