include(FetchContent)

FetchContent_Declare(
        maplibre
        GIT_REPOSITORY https://github.com/maplibre/maplibre-native.git
        GIT_TAG core-a33d3f00fb9dbf9117ce18edd799aa0ca5bcce5b # 2026-09-20

        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE

        GIT_SUBMODULES_RECURSE TRUE
        GIT_CONFIG
        "fetch.parallel=8"
        "submodule.recurse=true"
        "submodule.fetchJobs=8"
        "submodule.*.shallow=true"
)

set(MLN_WITH_OPENGL ON)
set(MLN_WITH_GLFW ON)
set(MLN_USE_UNORDERED_DENSE ON)
set(MLN_TEXT_SHAPING_HARFBUZZ ON)
set(MLN_WITH_RTTI ON)

FetchContent_MakeAvailable(maplibre)

set(MAPLIBRE_TARGETS
        mbgl-core-deps
        generate_dummy
)

foreach (TARGET ${MAPLIBRE_TARGETS})
    set_target_properties(${TARGET} PROPERTIES EXPORT_COMPILE_COMMANDS OFF)
endforeach ()

# TODO 2026-08-02 (Will Free): is this necessary?
target_link_libraries(
        mbgl-core
        PUBLIC
        $<BUILD_INTERFACE:mbgl-vendor-parsedate>
        $<BUILD_INTERFACE:mbgl-vendor-nunicode>
        $<BUILD_INTERFACE:mbgl-vendor-csscolorparser>
        $<$<PLATFORM_ID:iOS>:$<BUILD_INTERFACE:mbgl-vendor-filesystem>>
        z
        $<BUILD_INTERFACE:mbgl-vendor-sqlite>
)
