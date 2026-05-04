project(implot)

include(FetchContent)

FetchContent_Declare(
        implot
        GIT_REPOSITORY https://github.com/epezent/implot.git
        GIT_TAG v1.0
)

FetchContent_MakeAvailable(implot)

add_library(${PROJECT_NAME}
        ${implot_SOURCE_DIR}/implot.cpp
        ${implot_SOURCE_DIR}/implot_items.cpp
        ${implot_SOURCE_DIR}/implot_demo.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${implot_SOURCE_DIR})

target_compile_options(${PROJECT_NAME} PRIVATE -w -Wno-everything)

target_link_libraries(${PROJECT_NAME} PUBLIC imgui)
