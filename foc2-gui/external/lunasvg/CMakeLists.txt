project(lunasvg)

include(FetchContent)

FetchContent_Declare(
        lunasvg
        GIT_REPOSITORY https://github.com/sammycage/lunasvg.git
        GIT_TAG v3.5.0
)

FetchContent_MakeAvailable(lunasvg)
