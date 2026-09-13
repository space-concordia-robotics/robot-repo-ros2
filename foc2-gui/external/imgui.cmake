include(FetchContent)

FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.92.7-docking
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(imgui)

find_package(OpenGL REQUIRED)
find_package(Freetype REQUIRED)
#find_package(SDL3 REQUIRED)

# TODO 2026-04-30 (Will Free): switch from OpenGL to Vulkan

add_library(imgui
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/misc/freetype/imgui_freetype.cpp
        ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_compile_options(imgui PRIVATE -w -Wno-everything)

target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})

target_compile_definitions(imgui PUBLIC IMGUI_ENABLE_FREETYPE IMGUI_DEFINE_MATH_OPERATORS)

target_link_libraries(imgui PUBLIC SDL3::SDL3 OpenGL::GL Freetype::Freetype)

