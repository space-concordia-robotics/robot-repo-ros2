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

    const auto size = std::min(bounds.GetWidth(), bounds.GetHeight()) * 0.3;
    const auto radius = size / 2;
    const auto margin = radius / 4;

    const auto min = ImVec2(top_right.x - size - margin, top_right.y + margin);
    const auto max = ImVec2(top_right.x - margin, top_right.y + size + margin);

    const auto center = ImVec2((min.x + max.x) / 2, (min.y + max.y) / 2);

    draw_list->AddCircle(center, radius, ImGui::ImColor(50, 60, 80, 200), 0, radius / 24);
    draw_list->AddCircleFilled(center, radius, ImGui::ImColor(50, 60, 80, 128));

    // TODO 2026-05-04 (Will Free): add the following sub-overlays:
    //   - overlay to draw any marked gps coordinates on the border of the minimap
    //     (in different colours depending on what the coordinates are for)
    //   - overlay to show any detected aruco tags
    //   - overlay to show any detected objects
    //   - overlay to show any detected obstacles in nav2's costmap

    drawRobotAtCenter(draw_list, radius, center, has_pose);

    drawCompass(draw_list, radius, center, robot_yaw);
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

void MiniMapOverlay::drawRobotAtCenter(ImDrawList* draw_list, const double radius, const ImVec2& center, const bool active) {
    [[maybe_unused]] const auto halo = active ? ImGui::ImColor(0, 160, 220, 60) : ImGui::ImColor(80, 80, 80, 40);
    const auto body = active ? ImGui::ImColor(0, 200, 255, 200) : ImGui::ImColor(120, 120, 120, 180);
    [[maybe_unused]] constexpr auto accent = ImGui::ImColor(255, 255, 255, 255);

    // TODO 2026-05-04 (Will Free): unsure which of these I like the most tbh

    // draw_list->AddCircleFilled(center, 10.0, halo);
    // draw_list->AddCircleFilled(center, 6.0, body);
    // draw_list->AddCircle(center, 6.0, ImGui::ImColor(0, 0, 0, 120), 24, 1.0);

    const double length = radius / 5;
    constexpr auto angle = std::numbers::pi / 2 + std::numbers::pi / 4;

    const auto tip = ImVec2(center.x, center.y - length);
    const auto left = ImVec2(center.x + std::sin(angle) * length, center.y - std::cos(angle) * length);
    const auto right = ImVec2(center.x - std::sin(angle) * length, center.y - std::cos(angle) * length);
    const ImVec2 points[] = {
        tip, right, center, left, tip
    };
    draw_list->AddConvexPolyFilled(points, std::size(points), body);

    draw_list->AddPolyline(points, std::size(points), accent, 0, radius / 48);
}

void MiniMapOverlay::drawCompass(ImDrawList* draw_list, const double radius, const ImVec2& center, const double robot_yaw) {
    static constexpr int COMPASS_TICKS_PER_QUADRANT = 4;

    const auto major_tick_length = radius * 0.12;
    const auto minor_tick_length = radius * 0.05;

    static constexpr ImU32 MAJOR_TICK_COLOR = ImGui::ImColor(255, 0, 0, 120);
    static constexpr ImU32 TICK_COLOR = ImGui::ImColor(200, 200, 200, 120);
    static constexpr ImU32 LABEL_COLOR = ImGui::ImColor(240, 240, 240, 220);

    const auto angle_offset = -robot_yaw;

    for (int i = 0; i < COMPASS_TICKS_PER_QUADRANT * 4; ++i) {
        const auto len = i % 2 == 0 ? major_tick_length : minor_tick_length;

        const auto angle = static_cast<double>(i) / (COMPASS_TICKS_PER_QUADRANT * 4) * 2.0 * std::numbers::pi;
        const auto angle_cos = std::cos(angle + angle_offset);
        const auto angle_sin = std::sin(angle + angle_offset);
        const auto direction = ImVec2(angle_cos, angle_sin);

        const auto outer_offset = radius - len / 2;
        const auto inner_offset = radius + len / 2;
        const auto outer = center + direction * outer_offset;
        const auto inner = center + direction * inner_offset;

        draw_list->AddLine(outer, inner, i % 4 == 0 ? MAJOR_TICK_COLOR : TICK_COLOR, i % 2 == 0 ? 2.0 : 1.0);
    }

    auto placeLabel = [&](const std::string& text, const double angle) {
        const auto labelOffset = radius * 0.70;

        const auto angle_cos = std::cos(angle + angle_offset);
        const auto angle_sin = std::sin(angle + angle_offset);
        const auto direction = ImVec2(angle_cos, angle_sin);

        const auto text_size = ImGui::CalcTextSize(text);

        const auto pos = center + direction * labelOffset - text_size * 0.5;

        draw_list->AddText(pos, LABEL_COLOR, text.c_str());
    };

    ImGui::PushFont(nullptr, radius / 4);

    placeLabel("N", 0.0);
    placeLabel("E", std::numbers::pi / 2);
    placeLabel("S", std::numbers::pi);
    placeLabel("W", -std::numbers::pi / 2);

    ImGui::PopFont();
}
