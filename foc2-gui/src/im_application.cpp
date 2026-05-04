#include "foc2-gui/im_application.hpp"

#include <chrono>
#include <cmath>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>
#include <iostream>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl3.h>
#include <fmt/core.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <imfonts.h>
#include <rclcpp/rate.hpp>

// Based on Dear ImGui example "Dear ImGui: standalone example application for
// GLFW + OpenGL 3, using programmable pipeline"
// https://github.com/ocornut/imgui/blob/master/examples/example_glfw_opengl3/main.cpp

// See "Extremely Important Note" in README https://github.com/epezent/implot
// io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset' and handle
// ImDrawCmd::VtxOffset #define ImDrawIdx unsigned int

ImApplication::ImApplication(const std::string& node_name, const std::string& title) : Node(node_name), title(title), logger(this->get_logger()) {
    init();
}

ImApplication::~ImApplication() {}

int ImApplication::run() {
    onInit();

    using namespace std::chrono_literals;

    constexpr auto target_dt = 1.0s / 60.0;

    while (rclcpp::ok() && !done) {
        std::this_thread::sleep_until(last_frame + target_dt);

        last_frame = std::chrono::steady_clock::now();

        frame();
        onFrame();
        render();
    }
    onShutdown();
    quit();

    return 0;
}

void ImApplication::onFrame() {}

void ImApplication::init() {
    // Setup window
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        logger.error("Error: SDL_Init(): {}", SDL_GetError());
        std::cout << "sdl init error" << std::endl;
        return;
    }

    // Create window with graphics context
    window = SDL_CreateWindow("Dear ImGui + SDL3 Example", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) {
        logger.error("Error: SDL_CreateWindow(): {}", SDL_GetError());
        return;
    }

    gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    SDL_ShowWindow(window);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImPlot3D::CreateContext();

    auto& io = ImGui::GetIO();
    // Backend
    io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
    // Docking
    // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // Viewport
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    // Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // Gamepad
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(nullptr);

    // Setup Font
    io.Fonts->Clear();

    auto font_config = ImFontConfig();
    strcpy(font_config.Name, "Roboto");
    // font_cfg.PixelSnapH = true;
    font_config.OversampleH = 2;
    font_config.OversampleV = 2;
    font_config.FontDataOwnedByAtlas = false; // do not deallocate at the end of the program

    io.Fonts->AddFontFromMemoryTTF(
        Roboto_Regular_ttf,
        Roboto_Regular_ttf_len,
        std::round(16),
        &font_config,
        io.Fonts->GetGlyphRangesCyrillic()
    );
}

void ImApplication::frame() {
    // Poll and handle events (inputs, window resize, etc.)
    SDL_Event event;
    if (SDL_WaitEventTimeout(&event, 1)) {
        do {
            if (event.type == SDL_EVENT_QUIT)
                done = true;

            ImGui_ImplSDL3_ProcessEvent(&event);
        } while (SDL_PollEvent(&event));
    }

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
}

void ImApplication::render() const {
    ImGui::Render();
    glViewport(0, 0, 800, 600);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(window);
}

void ImApplication::quit() const {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();

    ImPlot3D::DestroyContext();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
