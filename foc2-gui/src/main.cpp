#define IMGUI_USER_CONFIG "foc2-gui/imgui_user.hpp"

#include <imgui.h>
#include <implot.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/widgets/video_widget.hpp"

class FOC2Application : public ImApplication {
public:
    FOC2Application() : ImApplication("test_app", "Test App") {}

protected:
    void onInit() override {
        video_top = std::make_shared<VideoWidget>(*this);
        video_bottom_left = std::make_shared<VideoWidget>(*this);
        video_bottom_right = std::make_shared<VideoWidget>(*this);

        video_top->onInit();
        video_bottom_left->onInit();
        video_bottom_right->onInit();

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
    }

private:
    std::shared_ptr<VideoWidget> video_top;
    std::shared_ptr<VideoWidget> video_bottom_left;
    std::shared_ptr<VideoWidget> video_bottom_right;

    static void drawLeft() {
        ImGui::Text("TODO");
    }

    void drawRight() {
        // TODO 2026-05-05 (Will Free): give the streams proper names

        // TODO 2026-05-06 (Will Free): vertically center windows.
        //  this can be done in two ways:
        //  1. compute the size of all the children manually and just that to set the cursor position
        //  2. render in two passes: a first invisible pass which is used to get the height, and a second pass to render.

        const auto available = ImGui::GetContentRegionAvail();

        ImGui::BeginChild("Top Stream", ImVec2(0, available.y * 0.5f), 0);
        drawStream(video_top);
        drawCenteredLabel("Top");
        ImGui::EndChild();

        ImGui::BeginChild("Bottom Left Stream", ImVec2(available.x * 0.5f, 0), 0);
        drawCenteredLabel("Bottom Left");
        drawStream(video_bottom_left);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("Bottom Right Stream", ImVec2(0, 0), 0);
        drawCenteredLabel("Bottom Right");
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

    node->run();

    rclcpp::shutdown();

    ros_thread.join();

    return 0;
}
