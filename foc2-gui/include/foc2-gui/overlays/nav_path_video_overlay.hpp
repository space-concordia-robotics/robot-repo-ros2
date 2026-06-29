#pragma once

#include <mutex>
#include <optional>
#include <opencv2/core/types.hpp>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "foc2-gui/overlay.hpp"

class NavPathVideoOverlay : public UiOverlay {
    using Path = nav_msgs::msg::Path;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

    struct PathPoint {
        ImVec2 pos;
        double alpha;
        double thickness;
    };

public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(NavPathVideoOverlay)

    explicit NavPathVideoOverlay(ImApplication& application, std::string path_topic, const ImVec4& path_color);

    ~NavPathVideoOverlay() override = default;

    void onInit() override;

    void onShutdown() override;

    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override;

    void onCameraInfo(const CameraInfo::SharedPtr& msg);

private:
    std::string path_topic;
    rclcpp::Subscription<Path>::SharedPtr path_subscription;

    ImVec4 path_color;

    mutable std::mutex mutex;
    Path path;
    std::optional<CameraInfo> camera_info;

    tf2_ros::Buffer& tf_buffer;

    static std::vector<std::vector<PathPoint>> segmentPathPoints(
        const ImRect& bounds,
        int camera_width,
        int camera_height,
        ImVec2 scale,
        const std::vector<cv::Point3d>& pts_cam,
        const std::vector<cv::Point2d>& image_pts
    );

    void drawSegments(ImDrawList* draw_list, const std::vector<std::vector<PathPoint>>& segments) const;

    void onPath(const Path::UniquePtr& msg);
};
