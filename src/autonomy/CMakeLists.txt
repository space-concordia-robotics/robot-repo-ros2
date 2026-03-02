cmake_minimum_required(VERSION 3.16)
project(autonomy)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
        DIRECTORY behaviour_trees config launch worlds
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
