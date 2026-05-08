#pragma once

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "foc2-gui/overlay.hpp"
#include "foc2-gui/util/imgui_util.hpp"


class MiniMapOverlay : public UiOverlay {
public:
    explicit MiniMapOverlay(ImApplication& app)
        : UiOverlay(app),
          tf_buffer(app.get_clock()),
          tf_listener(
              tf_buffer,
              &app
          ) {}

    void onInit() override;

    void onShutdown() override;

    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override;

private:
    const std::string map_frame;
    const std::string base_frame;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    rclcpp::TimerBase::SharedPtr tf_poll_timer;

    mutable std::mutex pose_mutex;
    double robot_yaw = 0.0;
    bool has_pose = false;

    void pollTransform();

    static void drawRobotAtCenter(ImDrawList* draw_list, double radius, const ImVec2& center, bool active);

    static void drawCompass(ImDrawList* draw_list, double radius, const ImVec2& center, double robot_yaw);
};
