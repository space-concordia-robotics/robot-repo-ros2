#define IMGUI_USER_CONFIG "foc2-gui/imgui_user.hpp"

#include <filesystem>
#include <IconsFontAwesome7.h>
#include <imgui.h>
#include <lunasvg.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <SDL3/SDL.h>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/resources.hpp"
#include "foc2-gui/widgets/logs_widget.hpp"
#include "foc2-gui/widgets/map_widget.hpp"
#include "foc2-gui/widgets/video_widget.hpp"

// include stb image implementation
#define STB_IMAGE_IMPLEMENTATION
// ReSharper disable once CppUnusedIncludeDirective
#include <stb_image.h>


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

static constexpr auto MAP_WIDGET_NAME = "Map";
static constexpr auto LOGS_WIDGET_NAME = "ROS Logs";
static constexpr auto TOP_STREAM_WIDGET_NAME = "Top Stream";
static constexpr auto BOTTOM_RIGHT_STREAM_WIDGET_NAME = "Bottom Right Stream";
static constexpr auto BOTTOM_LEFT_STREAM_WIDGET_NAME = "Bottom Left Stream";

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

        const auto share_dir = ament_index_cpp::get_package_share_directory(FOC2_PACKAGE_NAME);

        const auto& io = ImGui::GetIO();

        static constexpr auto BASE_FONT_SIZE = 13.0f; // 13.0f is the size of the default font. Change to the font size you use.
        static constexpr auto ICON_FONT_SIZE = BASE_FONT_SIZE * 2.0f / 3.0f;
        // FontAwesome fonts need to have their sizes reduced by 2.0f/3.0f in order to align correctly

        static constexpr ImWchar FONTAWESOME_ICON_RANGE[] = {ICON_MIN_FA, ICON_MAX_16_FA, 0};
        auto fontawesome_config = ImFontConfig();
        strcpy(fontawesome_config.Name, "FontAwesome Solid");
        fontawesome_config.MergeMode = true;
        fontawesome_config.PixelSnapH = true;
        fontawesome_config.GlyphMinAdvanceX = ICON_FONT_SIZE;

        const auto filename = std::filesystem::path(share_dir) / "resources" / "fonts" / FONT_ICON_FILE_NAME_FAS;
        io.Fonts->AddFontFromFileTTF(filename.c_str(), ICON_FONT_SIZE, &fontawesome_config, FONTAWESOME_ICON_RANGE);

        video_top = std::make_shared<VideoWidget>(*this, "rtsp://127.0.0.1:8554/test", "/rover/ffc/front/image_raw", true);
        video_bottom_left = std::make_shared<VideoWidget>(*this, "rtsp://10.240.0.10:8445/left", "/rover/ffc/left/image_raw", false);
        video_bottom_right = std::make_shared<VideoWidget>(*this, "rtsp://10.240.0.10:8445/right", "/rover/ffc/right/image_raw", false);

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

        // TODO 2026-05-24 (Will Free): for now we're just disabling the ini file so layout is never saved
        //  eventually, we should make sure to save the layout somewhere.
        ImGui::GetIO().IniFilename = nullptr;
    }

    void onFrame() override {
        const auto viewport = ImGui::GetMainViewport();
        const auto dockspace_id = ImGui::DockSpaceOverViewport(0, viewport, ImGuiDockNodeFlags_AutoHideTabBar);

        if (!layout_initialized) {
            setupInitialLayout(dockspace_id);
            layout_initialized = true;
        }

        drawUtilWindows();

        drawStreamWindows();
    }

    void onShutdown() override {
        video_top->onShutdown();
        video_bottom_left->onShutdown();
        video_bottom_right->onShutdown();
        map_widget->onShutdown();
        logs->onShutdown();
    }

private:
    bool layout_initialized = false;

    VideoWidget::SharedPtr video_top;
    VideoWidget::SharedPtr video_bottom_left;
    VideoWidget::SharedPtr video_bottom_right;
    RosLogWidget::SharedPtr logs;

    MapWidget::SharedPtr map_widget;

    static void setupInitialLayout(const ImGuiID& dock_main_id) {
        ImGuiID node_left;
        ImGuiID node_right;
        ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Left, 0.5, &node_left, &node_right);

        setupInitialLayoutLeft(node_left);

        setupInitialLayoutRight(node_right);

        ImGui::DockBuilderFinish(dock_main_id);
    }

    static void setupInitialLayoutLeft(const ImGuiID node_left) {
        ImGuiID node_left_bottom;
        ImGuiID node_left_top;
        ImGui::DockBuilderSplitNode(node_left, ImGuiDir_Up, 0.75, &node_left_top, &node_left_bottom);

        ImGui::DockBuilderGetNode(node_left_bottom)->SetLocalFlags(ImGuiDockNodeFlags_AutoHideTabBar);
        ImGui::DockBuilderGetNode(node_left_top)->SetLocalFlags(ImGuiDockNodeFlags_AutoHideTabBar);

        ImGui::DockBuilderDockWindow(MAP_WIDGET_NAME, node_left_top);
        ImGui::DockBuilderDockWindow(LOGS_WIDGET_NAME, node_left_bottom);
    }

    static void setupInitialLayoutRight(const ImGuiID node_right) {
        ImGuiID dock_right_top;
        ImGuiID dock_right_bottom;
        ImGui::DockBuilderSplitNode(node_right, ImGuiDir_Up, 0.6, &dock_right_top, &dock_right_bottom);

        ImGui::DockBuilderGetNode(dock_right_top)->SetLocalFlags(ImGuiDockNodeFlags_AutoHideTabBar);
        ImGui::DockBuilderDockWindow(TOP_STREAM_WIDGET_NAME, dock_right_top);

        ImGuiID dock_right_bottom_right;
        ImGuiID dock_right_bottom_left;
        ImGui::DockBuilderSplitNode(dock_right_bottom, ImGuiDir_Left, 0.5, &dock_right_bottom_left, &dock_right_bottom_right);

        ImGui::DockBuilderGetNode(dock_right_bottom_left)->SetLocalFlags(ImGuiDockNodeFlags_AutoHideTabBar);
        ImGui::DockBuilderGetNode(dock_right_bottom_right)->SetLocalFlags(ImGuiDockNodeFlags_AutoHideTabBar);

        ImGui::DockBuilderDockWindow(BOTTOM_LEFT_STREAM_WIDGET_NAME, dock_right_bottom_left);
        ImGui::DockBuilderDockWindow(BOTTOM_RIGHT_STREAM_WIDGET_NAME, dock_right_bottom_right);
    }

    void drawUtilWindows() const {
        ImGui::Begin(MAP_WIDGET_NAME, nullptr, ImGuiWindowFlags_None);
        map_widget->onFrame();
        ImGui::End();

        ImGui::Begin(LOGS_WIDGET_NAME, nullptr, ImGuiWindowFlags_None);
        logs->onFrame();
        ImGui::End();
    }

    void drawStreamWindows() const {
        constexpr auto drawStreamWindow = [](auto name, const VideoWidget::SharedPtr& video) {
            if (ImGui::Begin(name, nullptr, ImGuiWindowFlags_None)) {
                drawStream(video);
            }
            ImGui::End();
        };

        drawStreamWindow(TOP_STREAM_WIDGET_NAME, video_top);
        drawStreamWindow(BOTTOM_LEFT_STREAM_WIDGET_NAME, video_bottom_left);
        drawStreamWindow(BOTTOM_RIGHT_STREAM_WIDGET_NAME, video_bottom_right);
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
