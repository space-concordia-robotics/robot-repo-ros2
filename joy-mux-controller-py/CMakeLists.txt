cmake_minimum_required(VERSION 3.22)
project(joy_mux_controller_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
        ${PROJECT_NAME}
        SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_gps_py test/)
endif ()

ament_package()
