#define IMGUI_USER_CONFIG "foc2-gui/imgui_user.hpp"

#include <imgui.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/widgets/video_widget.hpp"

class FOC2Application : public ImApplication {
public:
    FOC2Application() : ImApplication("test_app", "Test App"), widget(*this) {}

protected:
    void onInit() override {
        widget.onInit();
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
            ImGuiWindowFlags_NoBringToFrontOnFocus |
            ImGuiWindowFlags_NoBackground;

        ImGui::Begin("MainUI", nullptr, flags);

        // TODO 2026-05-04 (Will Free): do something better here for the layout

        widget.onFrame();

        ImGui::End();
    }

    void onShutdown() override {
        widget.onShutdown();
    }

private:
    VideoWidget widget;
};

int main(const int argc, char const* const* argv) {
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
