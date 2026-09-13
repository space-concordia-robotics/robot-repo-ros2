include(FetchContent)

FetchContent_Declare(
        lunasvg
        GIT_REPOSITORY https://github.com/sammycage/lunasvg.git
        GIT_TAG v3.5.0
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(lunasvg)
