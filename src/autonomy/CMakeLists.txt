cmake_minimum_required(VERSION 3.16)
project(autonomy)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
        DIRECTORY behaviour_trees config launch maps models worlds
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
