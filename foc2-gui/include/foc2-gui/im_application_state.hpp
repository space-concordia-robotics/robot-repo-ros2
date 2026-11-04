#pragma once

#include <chrono>
#include <SDL3/SDL_video.h>

struct ImApplicationState {
    SDL_GLContext gl_context = nullptr;
    SDL_Window* window = nullptr;
    bool done = false;
    std::chrono::time_point<std::chrono::steady_clock> last_frame;
};
