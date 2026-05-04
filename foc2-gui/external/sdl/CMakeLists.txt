project(sdl)

include(FetchContent)

set(SDL_STATIC ON)
set(SDL_SHARED OFF)

set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)

set(SDL_TEST_LIBRARY OFF)

FetchContent_Declare(
        SDL3
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-3.4.4
)

FetchContent_MakeAvailable(SDL3)

get_target_property(aliased_sdl3 SDL3::SDL3 ALIASED_TARGET)

target_compile_options(${aliased_sdl3} PRIVATE -w -Wno-everything)

