cmake_minimum_required(VERSION 3.22)
project(ik_mux_controller)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)

find_dependencies(
        rclcpp
        control_msgs
        moveit_msgs
        geometry_msgs
        sensor_msgs
        std_srvs
)

## COMPILE

scrb_add_executable(ik_mux src/ik_mux.cpp)
target_link_libraries(ik_mux PUBLIC ${DEPENDENCY_TARGETS})

scrb_add_executable(ik_nxt src/ik_mux_nxt.cpp)
target_link_libraries(ik_nxt PUBLIC ${DEPENDENCY_TARGETS})

## INSTALL

install(
        TARGETS ik_mux ik_nxt
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORT
ament_package()
