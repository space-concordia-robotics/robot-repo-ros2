include(FetchContent)

FetchContent_Declare(
        implot
        GIT_REPOSITORY https://github.com/epezent/implot.git
        GIT_TAG v1.0
)

FetchContent_MakeAvailable(implot)

add_library(implot
        ${implot_SOURCE_DIR}/implot.cpp
        ${implot_SOURCE_DIR}/implot_items.cpp
        ${implot_SOURCE_DIR}/implot_demo.cpp
)

target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})

target_compile_options(implot PRIVATE -w -Wno-everything)

target_link_libraries(implot PUBLIC imgui)
