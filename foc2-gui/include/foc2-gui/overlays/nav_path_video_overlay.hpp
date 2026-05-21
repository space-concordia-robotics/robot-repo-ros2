#pragma once

#include <mutex>
#include <optional>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>

#include "foc2-gui/overlay.hpp"

class NavPathVideoOverlay : public UiOverlay {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(NavPathVideoOverlay)

    using Path = nav_msgs::msg::Path;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

    // tf_buffer is required to transform path poses into the camera frame.
    explicit NavPathVideoOverlay(ImApplication& application, std::string path_topic, const ImVec4& path_color);

    ~NavPathVideoOverlay() override = default;

    // Called by external code to deliver the current nav2 path.
    void onPath(const Path::UniquePtr& msg);

    // Public camera-info hook required by your pipeline.
    // This will be called by another class; we cache intrinsics for projection.
    void onCameraInfo(const CameraInfo::SharedPtr& msg);

    // UiOverlay lifecycle
    void onInit() override;
    void onShutdown() override;

    // Draw overlay onto the provided ImDrawList within bounds.
    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override;

private:
    std::string path_topic;
    rclcpp::Subscription<Path>::SharedPtr path_subscription;

    ImVec4 path_color;

    mutable std::mutex mutex;
    Path path;
    std::optional<CameraInfo> camera_info;

    tf2_ros::Buffer& tf_buffer;
};
