cmake_minimum_required(VERSION 3.22)
project(scrb_common_util)

find_package(scrb_cmake_utils REQUIRED)

## DEPENDENCIES

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_dependencies(
        rclcpp
        ros2_fmt_logger

        # ideally not everything inherits all these transitive deps, but whatever.

        tf2_eigen
        tf2_geometry_msgs

        OpenCV
)

## COMPILE

scrb_add_library(
        ${PROJECT_NAME} SHARED
        src/common_util.cpp
        src/string_parsing.cpp
        src/fmt/formatters.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCY_TARGETS})

target_precompile_headers(${PROJECT_NAME} PRIVATE
        <rclcpp/rclcpp.hpp>
        <fmt/format.h>
        <Eigen/Core>
        <Eigen/Geometry>
        <Eigen/Sparse>
        <opencv2/core/eigen.hpp>
        <tf2/convert.hpp>
        <tf2_eigen/tf2_eigen.hpp>
)

## INSTALL

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## TESTING

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_string_parsing_test test/string_parsing_test.cpp)
    target_include_directories(${PROJECT_NAME}_string_parsing_test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${PROJECT_NAME}_string_parsing_test ${PROJECT_NAME})
    ament_target_dependencies(${PROJECT_NAME}_string_parsing_test ${DEPENDENCIES})

    set_target_properties(gtest PROPERTIES EXPORT_COMPILE_COMMANDS OFF)
    set_target_properties(gtest_main PROPERTIES EXPORT_COMPILE_COMMANDS OFF)
endif ()

## EXPORTS

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()
