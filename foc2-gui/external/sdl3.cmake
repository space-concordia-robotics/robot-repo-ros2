include(FetchContent)

set(SDL_STATIC ON)
set(SDL_SHARED OFF)

set(SDL_TEST_LIBRARY OFF)

FetchContent_Declare(
        SDL3
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-3.4.4
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(SDL3)

get_target_property(SDL3_ALIASED_TARGET SDL3::SDL3 ALIASED_TARGET)

target_compile_options(${SDL3_ALIASED_TARGET} PRIVATE -w -Wno-everything)
set_target_properties(${SDL3_ALIASED_TARGET} PROPERTIES EXPORT_COMPILE_COMMANDS OFF)

