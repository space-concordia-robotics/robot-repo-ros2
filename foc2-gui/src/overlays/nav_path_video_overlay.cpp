#include "foc2-gui/overlays/nav_path_video_overlay.hpp"

#include <array>
#include <imgui.h>
#include <imgui_internal.h>
#include <ranges>
#include <tuple>
#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/matx.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "foc2-gui/util/imgui_util.hpp"
#include "foc2-gui/util/tf2_util.hpp"

NavPathVideoOverlay::NavPathVideoOverlay(ImApplication& application, std::string path_topic, const ImVec4& path_color)
    : UiOverlay(application),
      path_topic(std::move(path_topic)),
      path_color(path_color),
      tf_buffer(application.tfBuffer()) {}

void NavPathVideoOverlay::onInit() {
    path_subscription = application.create_subscription<Path>(
        path_topic,
        10,
        std::bind(&NavPathVideoOverlay::onPath, this, std::placeholders::_1)
    );
}

void NavPathVideoOverlay::onShutdown() {
    path_subscription.reset();
}

void NavPathVideoOverlay::onDraw(ImDrawList* draw_list, const ImRect& bounds) {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    cv::Matx33d camera_k;
    std::string cam_frame;
    std::string path_frame;
    int camera_width;
    int camera_height;

    {
        std::lock_guard lock(mutex);
        if (!camera_info.has_value() || path.poses.empty()) {
            using namespace std::chrono_literals;
            logger.warn_throttle(1s, "no poses (or camera info is empty: {})", camera_info.has_value());
            return;
        }
        poses.reserve(path.poses.size());
        for (const auto& p : path.poses)
            poses.push_back(p);

        camera_k = cv::Matx33d(camera_info->k.data());
        cam_frame = camera_info->header.frame_id;
        path_frame = path.header.frame_id;
        camera_width = camera_info->width;
        camera_height = camera_info->height;
    }

    using namespace std::chrono_literals;

    geometry_msgs::msg::TransformStamped tf2_transform;
    try {
        tf2_transform = tf_buffer.lookupTransform(
            cam_frame, path_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.05)
        );
    } catch (const tf2::TransformException& ex) {
        logger.error("TF lookup failed: {}", ex.what());
        return;
    }

    const auto transform = tf2::fromMsg<Eigen::Isometry3d>(tf2_transform);

    // TODO 2026-05-20 (Will Free): add _optical frames for all of the cameras.
    // convert points into the optical frame
    const auto orientation_matrix = Eigen::Matrix3d{
        {0, -1, 0},
        {0, 0, -1},
        {1, 0, 0}
    };

    std::vector<cv::Point3d> pts_cam;
    pts_cam.reserve(poses.size());
    for (const auto& pose : poses) {
        const auto position = tf2::fromMsg<Eigen::Vector3d>(pose.pose.position);

        const auto tmp = transform * position;
        const auto point = orientation_matrix * tmp;
        pts_cam.emplace_back(point.x(), point.y(), point.z());
    }

    std::vector<cv::Point2d> image_pts;
    if (!pts_cam.empty()) {
        const auto rvec = cv::Vec3d(0, 0, 0);
        const auto cvec = cv::Vec3d(0, 0, 0);
        cv::projectPoints(pts_cam, rvec, cvec, camera_k, cv::noArray(), image_pts);
    }

    const auto isInImage = [&](const cv::Point2d& point) {
        return point.x >= 0.0 && point.x < static_cast<double>(camera_width) && point.y >= 0.0 && point.y < static_cast<double>(camera_height);
    };

    const auto isInBounds = [&](const ImVec2& point) {
        return point.x >= bounds.Min.x && point.x <= bounds.Max.x && point.y >= bounds.Min.y && point.y <= bounds.Max.y;
    };

    const auto scale_x = bounds.GetWidth() / camera_width;
    const auto scale_y = bounds.GetHeight() / camera_height;
    const auto scale = ImVec2(scale_x, scale_y);

    auto cumulative_dist = 0.0;
    std::optional<Eigen::Vector3d> last_camera_point;

    struct PathPoint {
        ImVec2 pos;
        double alpha;
        double thickness;
    };

    std::vector<std::vector<PathPoint>> segments;
    segments.emplace_back();

    const auto nextSegment = [&] {
        if (!segments.back().empty())
            segments.emplace_back();
    };

    for (auto i = 0u; i < image_pts.size(); ++i) {
        const auto& point = image_pts[i];

        auto camera_point = Eigen::Vector3d(pts_cam[i].x, pts_cam[i].y, pts_cam[i].z);
        if (last_camera_point.has_value())
            cumulative_dist += (camera_point - last_camera_point.value()).norm();
        last_camera_point = camera_point;

        const auto screen_point = ImVec2(point.x, point.y) * scale + bounds.Min;

        if (camera_point.z() < 0.0 || !isInImage(point) || !isInBounds(screen_point)) {
            nextSegment();
            continue;
        }

        // TODO 2026-05-20 (Will Free): make fade distance a parameter/setting
        static constexpr auto FADE_DIST = 15.0;
        static constexpr auto BASE_THICKNESS = 3.0;
        const auto alpha = std::max(0.0, 1.0 - cumulative_dist / FADE_DIST);
        const auto thickness = std::clamp(
            BASE_THICKNESS * (5.0 / (5.0 + static_cast<float>(camera_point.norm()))),
            1.0,
            BASE_THICKNESS
        );

        if (static_cast<uint8_t>(alpha * 255) <= 0)
            break;

        segments.back().emplace_back(
            PathPoint{
                .pos       = screen_point,
                .alpha     = alpha,
                .thickness = thickness,
            }
        );
    }

    constexpr auto discontinuity_marker_size = 6.0;

    const auto drawDiscontinuityMarker = [&](const std::optional<PathPoint>& path_point) {
        if (!path_point.has_value())
            return;

        const auto point = path_point.value().pos;
        const auto alpha = path_point.value().alpha;

        const auto marker_color = ImGui::ImColor(255, 50, 50, 255 * alpha);

        constexpr auto d = ImVec2(discontinuity_marker_size, discontinuity_marker_size);
        constexpr auto d2 = ImVec2(discontinuity_marker_size, -discontinuity_marker_size);

        draw_list->AddLine(point - d, point + d, marker_color, 2.0);
        draw_list->AddLine(point - d2, point + d2, marker_color, 2.0);
    };

    const auto segmentColor = [&](const double alpha) {
        auto color = path_color;
        color.w *= alpha;
        return ImGui::GetColorU32(color);
    };

    std::optional<PathPoint> last_segment_end;

    for (const auto& segment : segments) {
        if (segment.size() == 0) {
            continue;
        } else if (segment.size() == 1) {
            const auto& [point, alpha, thickness] = segment[0];
            draw_list->AddCircle(point, thickness, segmentColor(alpha));

            drawDiscontinuityMarker(last_segment_end);

            last_segment_end = segment[0];
            continue;
        }

        drawDiscontinuityMarker(last_segment_end);

        for (auto&& [first, second] : std::views::adjacent<2>(segment)) {
            const auto& [start, start_alpha, start_thickness] = first;
            const auto& [end, end_alpha, end_thickness] = second;

            static constexpr auto PIXELS_PER_STEP = 4.0;

            if (const auto subdivisions = std::max(static_cast<int>(ImLengthSqr(end - start) / (PIXELS_PER_STEP * PIXELS_PER_STEP)), 1); subdivisions == 1) {
                draw_list->AddLine(start, end, segmentColor((start_alpha + end_alpha) / 2), (start_thickness + end_thickness) / 2);
            } else {
                for (int subdivision = 0; subdivision < subdivisions; ++subdivision) {
                    const auto ratio_start = static_cast<double>(subdivision) / subdivisions;
                    const auto ratio_end = static_cast<double>(subdivision + 1) / subdivisions;

                    const auto pos_start = ImLerp(start, end, ratio_start);
                    const auto pos_end = ImLerp(start, end, ratio_end);

                    const auto alpha = std::lerp(start_alpha, end_alpha, (ratio_start + ratio_end) * 0.5);
                    const auto thickness = std::lerp(start_thickness, end_thickness, (ratio_start + ratio_end) * 0.5);

                    draw_list->AddLine(pos_start, pos_end, segmentColor(alpha), thickness);
                }
            }
        }

        drawDiscontinuityMarker(last_segment_end);
        drawDiscontinuityMarker(segment.front());

        last_segment_end = segment.back();
    }
}

void NavPathVideoOverlay::onCameraInfo(const CameraInfo::SharedPtr& msg) {
    if (!msg)
        return;
    std::lock_guard lock(mutex);
    camera_info = *msg;
}

void NavPathVideoOverlay::onPath(const nav_msgs::msg::Path::UniquePtr& msg) {
    if (!msg)
        return;
    std::lock_guard lock(mutex);
    path = *msg;
}
