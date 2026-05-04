project(stb)

include(FetchContent)

FetchContent_Declare(
        stb
        GIT_REPOSITORY https://github.com/nothings/stb.git
        GIT_TAG 31c1ad37456438565541f4919958214b6e762fb4 # 2026-04-15
)

FetchContent_MakeAvailable(stb)

add_library(${PROJECT_NAME} INTERFACE ${stb_SOURCE_DIR})

target_include_directories(${PROJECT_NAME} INTERFACE ${stb_SOURCE_DIR})
