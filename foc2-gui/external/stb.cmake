include(FetchContent)

FetchContent_Declare(
        stb
        GIT_REPOSITORY https://github.com/nothings/stb.git
        GIT_TAG 31c1ad37456438565541f4919958214b6e762fb4 # 2026-04-15
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(stb)

add_library(stb INTERFACE ${stb_SOURCE_DIR})

target_include_directories(stb INTERFACE ${stb_SOURCE_DIR})
