#define IMGUI_USER_CONFIG "foc2-gui/imgui_user.hpp"

#include <filesystem>
#include <imgui.h>
#include <lunasvg.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <SDL3/SDL.h>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/resources.hpp"
#include "foc2-gui/widgets/logs_widget.hpp"
#include "foc2-gui/widgets/video_widget.hpp"

// include stb image implementation
#define STB_IMAGE_IMPLEMENTATION
// ReSharper disable once CppUnusedIncludeDirective
#include <stb_image.h>

#include "foc2-gui/widgets/map_widget.hpp"


SDL_Surface* loadSvgSurface(const std::filesystem::path& path, const int width = 0, const int height = 0) {
    auto doc = lunasvg::Document::loadFromFile(path);
    if (!doc)
        throw std::runtime_error(fmt::format("Failed to load SVG: {}", path.string()));
    const auto svg = std::move(doc);

    const auto bitmap = svg->renderToBitmap(width, height);

    // TODO 2026-05-09 (Will Free): this is load bearing. I do not know why, and it disturbs me.
    [[maybe_unused]] const auto ignored = bitmap.writeToPng(std::filesystem::temp_directory_path() / "foc2-gui-IGNORED.png");

    return SDL_CreateSurfaceFrom(
        bitmap.width(),
        bitmap.height(),
        SDL_PIXELFORMAT_BGRA32,
        bitmap.data(),
        bitmap.width() * 4
    );
}


class FOC2Application : public ImApplication {
public:
    FOC2Application() : ImApplication("foc2_gui", "SCRB C2 Station") {}

    FOC2Application(const FOC2Application& other) = delete;
    FOC2Application(FOC2Application&& other) noexcept = delete;
    FOC2Application& operator=(const FOC2Application& other) = delete;
    FOC2Application& operator=(FOC2Application&& other) noexcept = delete;

protected:
    void onWindow() override {
        const auto share_dir = ament_index_cpp::get_package_share_directory(FOC2_PACKAGE_NAME);

        const auto icon = loadSvgSurface(std::filesystem::path(share_dir) / "resources" / "icon.svg", 64, 64);

        if (!SDL_SetWindowIcon(window, icon)) {
            logger.warn("Error: SDL_CreateWindow(): {}", SDL_GetError());
        }
        SDL_DestroySurface(icon);
    }

    void onInit() override {
        ImApplication::onInit();

        video_top = std::make_shared<VideoWidget>(*this, "rtsp://127.0.0.1:8554/test", "/rover/ffc/front/image_raw", true, "none");
        video_bottom_left = std::make_shared<VideoWidget>(*this, "rtsp://10.240.0.10:8445/left", "/rover/ffc/left/image_raw", false, "none");
        video_bottom_right = std::make_shared<VideoWidget>(*this, "rtsp://10.240.0.10:8445/right", "/rover/ffc/right/image_raw", false, "none");

        map_widget = std::make_shared<MapWidget>(*this);

        logs = std::make_shared<RosLogWidget>(*this);

        video_top->onInit();
        video_bottom_left->onInit();
        video_bottom_right->onInit();

        map_widget->onInit();

        logs->onInit();

        auto& style = ImGui::GetStyle();
        style.WindowPadding = ImVec2(4.0, 4.0);
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.12, 0.12, 0.12, 1.0);
        style.Colors[ImGuiCol_ChildBg] = ImVec4(0.16, 0.16, 0.16, 1.0);

        style.AntiAliasedLines = true;
        style.AntiAliasedLinesUseTex = true;
    }

    void onFrame() override {
        const auto viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->Pos);
        ImGui::SetNextWindowSize(viewport->Size);

        constexpr auto flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoBringToFrontOnFocus;

        ImGui::Begin("MainUI", nullptr, flags);

        const ImVec2 available = ImGui::GetContentRegionAvail();

        // TODO 2026-05-05 (Will Free): for now these are just split 50/50. figure out something better for layout later.
        const float left_width = available.x * 0.5f;

        // TODO 2026-05-05 (Will Free): figure out what to put on the left side

        ImGui::BeginChild("Left", ImVec2(left_width, 0), false);
        drawLeft();
        ImGui::EndChild();

        ImGui::SameLine();

        {
            const auto draw_list = ImGui::GetWindowDrawList();
            const auto cursor_pos = ImGui::GetCursorScreenPos();

            const auto top = ImGui::GetWindowPos().y;
            const auto bottom = top + ImGui::GetWindowSize().y;

            draw_list->AddLine(
                ImVec2(cursor_pos.x, top + 8),
                ImVec2(cursor_pos.x, bottom - 8),
                ImGui::ImColor(200, 200, 200, 40),
                2.0
            );
        }

        ImGui::BeginChild("Video Streams", ImVec2(0, 0), false);
        drawRight();
        ImGui::EndChild();

        ImGui::End();
    }

    void onShutdown() override {
        video_top->onShutdown();
        video_bottom_left->onShutdown();
        video_bottom_right->onShutdown();
        map_widget->onShutdown();
        logs->onShutdown();
    }

private:
    std::shared_ptr<VideoWidget> video_top;
    std::shared_ptr<VideoWidget> video_bottom_left;
    std::shared_ptr<VideoWidget> video_bottom_right;
    std::shared_ptr<RosLogWidget> logs;

    std::shared_ptr<MapWidget> map_widget;

    void drawLeft() const {
        ImGui::Text("TODO");

        ImGui::Separator();

        const auto available = ImGui::GetContentRegionAvail();

        // TODO 2026-05-07 (Will Free): make this a percent
        ImGui::BeginChild("Map", ImVec2(0, available.y - 240), 0);
        map_widget->onFrame();
        ImGui::EndChild();

        ImGui::BeginChild("ROS Logs", ImGui::GetContentRegionAvail(), 0);
        logs->onFrame();
        ImGui::EndChild();
    }

    void drawRight() const {
        // TODO 2026-05-05 (Will Free): give the streams proper names

        // TODO 2026-05-06 (Will Free): vertically center windows.
        //  this can be done in two ways:
        //  1. compute the size of all the children manually and just that to set the cursor position
        //  2. render in two passes: a first invisible pass which is used to get the height, and a second pass to render.

        const auto available = ImGui::GetContentRegionAvail();

        ImGui::BeginChild("Top Stream", ImVec2(0, available.y * 0.6f), 0);
        drawStream(video_top);
        ImGui::EndChild();

        ImGui::BeginChild("Bottom Left Stream", ImVec2(available.x * 0.5f, 0), 0);
        drawStream(video_bottom_left);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("Bottom Right Stream", ImVec2(0, 0), 0);
        drawStream(video_bottom_right);
        ImGui::EndChild();
    }

    static void drawCenteredLabel(const std::string& label) {
        const auto available_x = ImGui::GetContentRegionAvail().x;

        const auto label_size = ImGui::CalcTextSize(label);

        const auto off = (available_x - label_size.x) / 2;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + off);

        ImGui::TextUnformatted(label);
    }

    static void drawStream(const std::shared_ptr<VideoWidget>& video_widget) {
        video_widget->onFrame();
    }
};

int main(int argc, char* * argv) {
    // initialize gstreamer
    gst_init(&argc, &argv);

    rclcpp::init(argc, argv);
    const auto node = std::make_shared<FOC2Application>();

    auto ros_thread = std::thread([&] {
        rclcpp::spin(node);
    });

    if (const auto result = node->init(); result != 0)
        return result;

    const auto result = node->run();

    rclcpp::shutdown();

    ros_thread.join();

    return result;
}
