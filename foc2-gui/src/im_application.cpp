#include "foc2-gui/im_application.hpp"

#include <chrono>
#include <cmath>
#include <imfonts.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>
#include <utility>
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_opengl.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl3.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "foc2-gui/im_application_state.hpp"

// Based on Dear ImGui example "Dear ImGui: standalone example application for
// GLFW + OpenGL 3, using programmable pipeline"
// https://github.com/ocornut/imgui/blob/master/examples/example_glfw_opengl3/main.cpp

// See "Extremely Important Note" in README https://github.com/epezent/implot
// io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset' and handle
// ImDrawCmd::VtxOffset #define ImDrawIdx unsigned int

ImApplication::ImApplication(const std::string& node_name, std::string title)
    : Node(node_name),
      logger(this->get_logger()),
      title(std::move(title)) {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer, this);
    state = std::make_unique<ImApplicationState>();
}

ImApplication::~ImApplication() = default;

int ImApplication::run() {
    onInit();

    using namespace std::chrono_literals;
    using namespace std::chrono;

    constexpr auto target_frame_duration = 1000.0ms / 60.0;

    while (rclcpp::ok() && !state->done) {
        std::this_thread::sleep_until(state->last_frame + target_frame_duration);

        state->last_frame = steady_clock::now();

        frame();
        onFrame();
        render();
    }
    onShutdown();
    quit();

    return 0;
}

tf2_ros::Buffer& ImApplication::tfBuffer() const {
    return *tf_buffer;
}

void ImApplication::onInit() {}

void ImApplication::onFrame() {}

int ImApplication::init() {
    // Setup window
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        logger.error("Error: SDL_Init(): {}", SDL_GetError());
        return -1;
    }

    // Create window with graphics context
    state->window = SDL_CreateWindow(title.c_str(), 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_MAXIMIZED);
    if (state->window == nullptr) {
        logger.error("Error: SDL_CreateWindow(): {}", SDL_GetError());
        return -1;
    }

    state->gl_context = SDL_GL_CreateContext(state->window);
    SDL_GL_MakeCurrent(state->window, state->gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    SDL_ShowWindow(state->window);

    onWindow();

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImPlot3D::CreateContext();

    auto& io = ImGui::GetIO();
    // Backend
    io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
    // Docking
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // Viewport
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    // Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // Gamepad
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForOpenGL(state->window, state->gl_context);
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

    return 0;
}

void ImApplication::frame() const {
    // Poll and handle events (inputs, window resize, etc.)
    SDL_Event event;
    if (SDL_WaitEventTimeout(&event, 1)) {
        do {
            if (event.type == SDL_EVENT_QUIT)
                state->done = true;

            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(state->window))
                state->done = true;

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

    if ((ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) != 0) {
        // backup current window & context to restore them later
        const auto previous_window = SDL_GL_GetCurrentWindow();
        const auto previous_context = SDL_GL_GetCurrentContext();

        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();

        SDL_GL_MakeCurrent(previous_window, previous_context);
    }

    SDL_GL_SwapWindow(state->window);
}

void ImApplication::quit() const {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();

    ImPlot3D::DestroyContext();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_GL_DestroyContext(state->gl_context);
    SDL_DestroyWindow(state->window);
    SDL_Quit();
}
