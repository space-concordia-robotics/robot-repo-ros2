#include "foc2-gui/overlays/minimap.hpp"

#include <tf2/utils.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "foc2-gui/util/tf2_util.hpp"

void MiniMapOverlay::onInit() {
    using namespace std::chrono_literals;

    tf_poll_timer = application.create_wall_timer(50ms, [this] {
        this->pollTransform();
    });
}

void MiniMapOverlay::onShutdown() {
    tf_poll_timer->reset();
    tf_poll_timer->cancel();
}

void MiniMapOverlay::onDraw(ImDrawList* draw_list, const ImRect& bounds) {
    const auto top_right = bounds.GetTR();

    // TODO 2026-05-04 (Will Free): scale compass with size of bounds

    const auto min = ImVec2(top_right.x - SIZE - MARGIN, top_right.y + MARGIN);
    const auto max = ImVec2(top_right.x - MARGIN, top_right.y + SIZE + MARGIN);

    const auto center = ImVec2((min.x + max.x) / 2, (min.y + max.y) / 2);

    draw_list->AddCircle(center, RADIUS, ImGui::ImColor(50, 60, 80, 200), 0, 4.0);
    draw_list->AddCircleFilled(center, RADIUS, ImGui::ImColor(50, 60, 80, 128));

    // TODO 2026-05-04 (Will Free): add the following sub-overlays:
    //   - overlay to draw any marked gps coordinates on the border of the minimap
    //     (in different colours depending on what the coordinates are for)
    //   - overlay to show any detected aruco tags
    //   - overlay to show any detected objects
    //   - overlay to show any detected obstacles in nav2's costmap

    drawRobotAtCenter(draw_list, center, has_pose);

    drawCompass(draw_list, center, robot_yaw);
}

void MiniMapOverlay::pollTransform() {
    // TODO 2026-05-04 (Will Free): This is broken right now. for some reason the buffer cannot find any frames.
    //  figure out why later.

    try {
        // TODO 2026-05-04 (Will Free): make these two frames parameterized somehow

        const auto transform = tf_buffer.lookupTransform("utm", "base_link", tf2::TimePointZero);

        const auto quat = tf2::fromMsg<tf2::Quaternion>(transform.transform.rotation);

        const auto yaw = tf2::getYaw(quat);

        std::lock_guard lock(pose_mutex);

        robot_yaw = yaw;
        has_pose = true;
    } catch (const std::exception& e) {
        using namespace std::chrono_literals;
        logger.warn_throttle(1s, "failed to look up tf: {}", e.what());
    }
}

void MiniMapOverlay::drawRobotAtCenter(ImDrawList* draw_list, const ImVec2& center, const bool active) {
    [[maybe_unused]] const auto halo = active ? ImGui::ImColor(0, 160, 220, 60) : ImGui::ImColor(80, 80, 80, 40);
    const auto body = active ? ImGui::ImColor(0, 200, 255, 200) : ImGui::ImColor(120, 120, 120, 180);
    constexpr auto accent = ImGui::ImColor(255, 255, 255, 255);

    // TODO 2026-05-04 (Will Free): unsure which of these I like the most tbh

    // draw_list->AddCircleFilled(center, 10.0, halo);
    // draw_list->AddCircleFilled(center, 6.0, body);
    // draw_list->AddCircle(center, 6.0, ImGui::ImColor(0, 0, 0, 120), 24, 1.0);

    constexpr double len = 16.0;
    const auto tip = ImVec2(center.x, center.y - len);
    constexpr auto side_ang = std::numbers::pi / 2 + std::numbers::pi / 4;
    const auto left = ImVec2(center.x + std::sin(side_ang) * (len * 1), center.y - std::cos(side_ang) * (len * 1));
    const auto right = ImVec2(center.x - std::sin(side_ang) * (len * 1), center.y - std::cos(side_ang) * (len * 1));
    const ImVec2 points[] = {
        tip, right, center, left, tip
    };
    draw_list->AddConvexPolyFilled(points, std::size(points), body);
    draw_list->AddPolyline(points, std::size(points), accent, 0, 1.5);
}

void MiniMapOverlay::drawCompass(ImDrawList* draw_list, const ImVec2& center, const double robot_yaw) {
    static constexpr int COMPASS_TICKS = 16;

    static constexpr auto MAJOR_TICK_LENGTH = RADIUS * 0.10;
    static constexpr auto MINOR_TICK_LENGTH = RADIUS * 0.05;

    static constexpr ImU32 tick_color = ImGui::ImColor(200, 200, 200, 120);
    static constexpr ImU32 label_color = ImGui::ImColor(240, 240, 240, 220);

    const auto angle_offset = -robot_yaw;

    for (int i = 0; i < COMPASS_TICKS; ++i) {
        const auto len = i % 2 == 0 ? MAJOR_TICK_LENGTH : MINOR_TICK_LENGTH;

        const auto angle = static_cast<double>(i) / COMPASS_TICKS * 2.0 * std::numbers::pi;
        const auto angle_cos = std::cos(angle + angle_offset);
        const auto angle_sin = std::sin(angle + angle_offset);

        const auto outer = ImVec2(center.x + angle_cos * (RADIUS - 4.0), center.y + angle_sin * (RADIUS - 4.0));
        const auto inner = ImVec2(center.x + angle_cos * (RADIUS - 4.0 - len), center.y + angle_sin * (RADIUS - 4.0 - len));

        draw_list->AddLine(outer, inner, tick_color, i % 2 == 0 ? 2.0 : 1.0);
    }

    auto placeLabel = [&](const std::string& text, const double angle) {
        constexpr auto labelOffset = RADIUS * 0.70;

        const auto angle_cos = std::cos(angle + angle_offset);
        const auto angle_sin = std::sin(angle + angle_offset);

        const auto text_size = ImGui::CalcTextSize(text);

        const auto pos = ImVec2(center.x + angle_cos * labelOffset - text_size.x * 0.5f, center.y + angle_sin * labelOffset - text_size.y * 0.5f);

        draw_list->AddText(pos, label_color, text.c_str());
    };

    placeLabel("N", 0.0);
    placeLabel("E", std::numbers::pi / 2);
    placeLabel("S", std::numbers::pi);
    placeLabel("W", -std::numbers::pi / 2);
}
