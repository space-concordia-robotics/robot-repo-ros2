include(FetchContent)

FetchContent_Declare(
        implot3d
        GIT_REPOSITORY https://github.com/brenocq/implot3d.git
        GIT_TAG v0.4
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(implot3d)

add_library(implot3d
        ${implot3d_SOURCE_DIR}/implot3d.cpp
        ${implot3d_SOURCE_DIR}/implot3d_items.cpp
        ${implot3d_SOURCE_DIR}/implot3d_meshes.cpp
        ${implot3d_SOURCE_DIR}/implot3d_demo.cpp
)

target_include_directories(implot3d PUBLIC ${implot3d_SOURCE_DIR})

target_compile_options(implot3d PRIVATE -w -Wno-everything)

target_link_libraries(implot3d PUBLIC imgui)
