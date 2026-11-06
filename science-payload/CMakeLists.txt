cmake_minimum_required(VERSION 3.22)
project(science_payload)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
        ${PROJECT_NAME}
        SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_package()
