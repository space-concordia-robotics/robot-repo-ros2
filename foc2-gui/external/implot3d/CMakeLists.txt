project(implot3d)

include(FetchContent)

FetchContent_Declare(
        implot3d
        GIT_REPOSITORY https://github.com/brenocq/implot3d.git
        GIT_TAG v0.4
)

FetchContent_MakeAvailable(implot3d)

add_library(${PROJECT_NAME}
        ${implot3d_SOURCE_DIR}/implot3d.cpp
        ${implot3d_SOURCE_DIR}/implot3d_items.cpp
        ${implot3d_SOURCE_DIR}/implot3d_meshes.cpp
        ${implot3d_SOURCE_DIR}/implot3d_demo.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${implot3d_SOURCE_DIR})

target_compile_options(${PROJECT_NAME} PRIVATE -w -Wno-everything)

target_link_libraries(${PROJECT_NAME} PUBLIC imgui)
