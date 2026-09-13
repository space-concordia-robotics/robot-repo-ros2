#pragma once

#include <sensor_msgs/msg/magnetic_field.hpp>

#include "foc2-gui/overlay.hpp"
#include "foc2-gui/util/imgui_util.hpp"


class MiniMapOverlay : public UiOverlay {
    using MagneticField = sensor_msgs::msg::MagneticField;

public:
    explicit MiniMapOverlay(ImApplication& application)
        : UiOverlay(application) {}

    void onInit() override;

    void onShutdown() override;

    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override;

private:
    const std::string map_frame;
    const std::string base_frame;

    rclcpp::Subscription<MagneticField>::SharedPtr magnetic_field_subscription;

    mutable std::mutex magnetic_field_mutex;
    MagneticField::SharedPtr magnetic_field;

    double robot_yaw = 0.0;
    bool has_pose = false;

    static void drawRobotAtCenter(ImDrawList* draw_list, double radius, const ImVec2& center, bool active);

    static void drawCompass(ImDrawList* draw_list, double radius, const ImVec2& center, double north_bearing);
};
