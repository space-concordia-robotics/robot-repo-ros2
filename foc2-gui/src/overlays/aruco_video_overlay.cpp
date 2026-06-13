#include "foc2-gui/overlays/aruco_video_overlay.hpp"

#include <Eigen/Geometry>
#include <image_transport/camera_common.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "foc2-gui/util/tf2_util.hpp"

using namespace std::chrono_literals;

void ArucoVideoOverlay::onInit() {
    aruco_subscription = application.create_subscription<ArucoDetections>(
        "/aruco_detections",
        10,
        [this](const ArucoDetections::UniquePtr& msg) {
            onDetections(msg);
        }
    );
}

constexpr ImVec2 pointToBounds(const cv::Point2f point, const ImRect& bounds, const double scale_x, const double scale_y) {
    return ImVec2(
        point.x * scale_x + bounds.Min.x,
        point.y * scale_y + bounds.Min.y
    );
}

void drawArucoMarker(
    ImDrawList* draw_list,
    const ImRect& bounds,
    const double scale_x,
    const double scale_y,
    const std::array<cv::Point2f, 4>& corners,
    const std::optional<std::string>& marker_label,
    const ImU32 marker_color,
    const ImU32 id_color
) {
    std::array<ImVec2, corners.size() + 1> points;
    // ImVec2 points[corners.size() + 1];

    for (auto i = 0u; i < corners.size(); i++) {
        points.at(i) = pointToBounds(corners.at(i), bounds, scale_x, scale_y);
    }

    // add the 1st point at the end to close the polyline
    points.at(corners.size()) = points.at(0);

    // TODO 2026-05-05 (Will Free): scale thickness with distance
    draw_list->AddPolyline(points.data(), corners.size() + 1, marker_color, 0, 1.0);

    // marker for the corner
    draw_list->AddRect(
        {points.at(0).x - 3, points.at(0).y - 3},
        {points.at(0).x + 3, points.at(0).y + 3},
        corners.size() + 1,
        ImGui::ImColor(0, 0, 255, 255),
        0,
        1.0
    );

    if (marker_label.has_value()) {
        // TODO 2026-05-05 (Will Free): scale marker text size with distance
        ImGui::PushFont(nullptr, 24.0);

        const auto& marker_text = marker_label.value();
        // 0 & 2 should be the two diagonal corners
        const auto center = ImRect(points.at(0), points.at(2)).GetCenter();

        const auto text_size = ImGui::CalcTextSize(marker_text);

        const auto text_position = ImVec2(center.x + 4, center.y - text_size.y);

        // TODO 2026-05-05 (Will Free): make it easier to read by maybe adding a text outline?
        //  the ways to add text outlines in imgui are all extremely hacky...
        //  I don't like imgui's font renderer.
        draw_list->AddText(text_position, id_color, marker_label.value().c_str());

        ImGui::PopFont();
    }
}

void drawFrameAxes(
    ImDrawList* draw_list,
    const ImRect& bounds,
    const double scale_x,
    const double scale_y,
    const ArucoVideoOverlay::ProjectedAxes& axes
) {
    const auto origin = pointToBounds(axes.origin, bounds, scale_x, scale_y);
    const auto x_end = pointToBounds(axes.x_end, bounds, scale_x, scale_y);
    const auto y_end = pointToBounds(axes.y_end, bounds, scale_x, scale_y);
    const auto z_end = pointToBounds(axes.z_end, bounds, scale_x, scale_y);

    // TODO 2026-05-05 (Will Free): scale thickness with distance?
    draw_list->AddLine(origin, x_end, ImGui::ImColor(0, 0, 255, 255), 2.0);
    draw_list->AddLine(origin, y_end, ImGui::ImColor(0, 255, 0, 255), 2.0);
    draw_list->AddLine(origin, z_end, ImGui::ImColor(255, 0, 0, 255), 2.0);
}

inline void ArucoVideoOverlay::onDraw(ImDrawList* draw_list, const ImRect& bounds) {
    // reject if we have not yet received the camera info topic
    if (!has_camera_info || camera_frame.empty()) {
        logger.warn_throttle(1s, "Have not yet received camera info/camera frame, so cannot drawing aruco tags");
        return;
    }

    std::vector<ProjectedMarker> markers;
    std::vector<RejectedMarker> rejected_markers;

    int camera_width = -1;
    int camera_height = -1;

    {
        // TODO 2026-05-05 (Will Free): there should probably be one mutex for the markers & one for the width/height, but I'm lazy
        std::scoped_lock lock(markers_mutex);
        markers = this->markers;
        rejected_markers = this->rejected_markers;

        camera_width = this->camera_width;
        camera_height = this->camera_height;
    }

    if (markers.empty() && rejected_markers.empty())
        return;

    // TODO 2026-05-05 (Will Free): this currently assumes that all cameras have the same dimensions
    //  (i.e. the dimensions of the camera a marker was detected in is the same as this camera's dimensions)

    const float scale_x = bounds.GetWidth() / camera_width;
    const float scale_y = bounds.GetHeight() / camera_height;

    constexpr auto marker_color = ImGui::ImColor(0, 255, 0, 255);
    constexpr auto rejected_marker_color = ImGui::ImColor(255, 0, 0, 255);
    constexpr auto id_color = ImGui::ImColor(255, 255, 255, 255);

    for (const auto& marker : markers) {
        drawArucoMarker(draw_list, bounds, scale_x, scale_y, marker.corners, marker.marker_label, marker_color, id_color);

        drawFrameAxes(draw_list, bounds, scale_x, scale_y, marker.axes);
    }

    // rejected markers can only be drawn if the camera frame matches the rejected frame.
    // this is because you cannot reproject the rejected markers from 2d into 3d,
    // then translate to the correct frame.
    for (const auto& rejected_marker : rejected_markers) {
        if (rejected_marker.header.frame_id != camera_frame)
            continue;

        // convert goemetry_msgs' Point32 to cv::Point2f
        std::array<cv::Point2f, 4> corners;
        for (int i = 0; i < 4; ++i) {
            corners.at(i).x = rejected_marker.corners.at(i).x;
            corners.at(i).y = rejected_marker.corners.at(i).y;
        }

        drawArucoMarker(draw_list, bounds, scale_x, scale_y, corners, std::nullopt, rejected_marker_color, id_color);
    }
}

void ArucoVideoOverlay::onDetections(const ArucoDetections::UniquePtr& msg) {
    cv::Matx33d camera_k;

    {
        std::scoped_lock lock(camera_mutex);
        camera_k = this->camera_k;
    }

    const auto msg_markers = msg->markers;
    auto markers_tmp = std::vector<ProjectedMarker>();
    markers_tmp.reserve(msg_markers.size());

    // TODO 2026-05-05 (Will Free): make this a parameter
    static constexpr auto MARKER_TAG_SIZE = 0.15;

    // we're just reprojecting the corners of a perfect tag (not necessarily the corners of the detected tag!) into the correct frame.
    constexpr auto size_half = MARKER_TAG_SIZE * 0.5;
    std::vector<cv::Point3f> tag_corners = {
        {-size_half, size_half, 0},
        {size_half, size_half, 0},
        {size_half, -size_half, 0},
        {-size_half, -size_half, 0}
    };

    const std::vector<cv::Point3d> tag_axes = {
        {0, 0, 0},
        {MARKER_TAG_SIZE * 0.75, 0, 0},
        {0, MARKER_TAG_SIZE * 0.75, 0},
        {0, 0, MARKER_TAG_SIZE * 0.75}
    };

    // reproject marker corners from the marker's frame into the current camera's frame
    for (auto marker : msg_markers) {
        auto translated_marker = ProjectedMarker();
        translated_marker.marker_label = fmt::format("id: {}", marker.id);

        geometry_msgs::msg::Pose transformed_pose;
        const auto frame_id = marker.header.frame_id.empty() ? msg->header.frame_id : marker.header.frame_id;
        if (frame_id.empty()) {
            logger.warn_throttle(1s, "frame id was empty for both the aruco marker and the detections message, skipping");
            continue;
        }

        if (frame_id == camera_frame) {
            transformed_pose = marker.pose;
        } else {
            try {
                // TODO 2026-05-05 (Will Free): the rtsp stream is buffered for ~200ms,
                //  we should consider adjusting the transform based on that.
                //  there may also be additional latency elsewhere, maybe make it configurable?

                auto tf = tf_buffer.lookupTransform(
                    camera_frame,
                    frame_id,
                    tf2::TimePointZero
                );
                tf2::doTransform(marker.pose, transformed_pose, tf);
                translated_marker.pose = transformed_pose;
            } catch (const std::exception& e) {
                logger.warn("could not transform marker: {}", e.what());
                continue;
            }
        }

        Eigen::Quaterniond quat = tf2::fromMsg<Eigen::Quaterniond>(transformed_pose.orientation).normalized();

        const auto aa = Eigen::AngleAxisd(quat);
        const auto rotation_vector_eigen = aa.axis() * aa.angle();

        const auto rotation_vector = cv::Vec3f(rotation_vector_eigen.x(), rotation_vector_eigen.y(), rotation_vector_eigen.z());

        const auto translation_vector = cv::Vec3f(
            transformed_pose.position.x,
            transformed_pose.position.y,
            transformed_pose.position.z
        );

        // reproject corners from original frame into the camera's frame
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(tag_corners, rotation_vector, translation_vector, camera_k, cv::noArray(), reprojected_points);

        // always exactly 4 points
        for (int i = 0; i < 4; i++) {
            translated_marker.corners.at(i) = reprojected_points.at(i);
        }

        // reproject axes from original frame into the camera's frame
        std::vector<cv::Point2d> projected_axes;
        cv::projectPoints(tag_axes, rotation_vector, translation_vector, camera_k, cv::noArray(), projected_axes);

        translated_marker.axes.origin = projected_axes.at(0);
        translated_marker.axes.x_end = projected_axes.at(1);
        translated_marker.axes.y_end = projected_axes.at(2);
        translated_marker.axes.z_end = projected_axes.at(3);

        markers_tmp.push_back(translated_marker);
    }

    std::scoped_lock lock(markers_mutex);

    markers = markers_tmp;

    rejected_markers = msg->rejected_markers;
}

void ArucoVideoOverlay::onCameraInfo(const CameraInfo::SharedPtr& msg) {
    std::scoped_lock lock(camera_mutex);
    camera_width = msg->width;
    camera_height = msg->height;

    if (!msg->header.frame_id.empty()) {
        camera_frame = msg->header.frame_id;
    } else {
        logger.warn_throttle(1s, "Camera info topic received, but frame id was empty");
    }

    camera_k = cv::Matx33d(msg->k.data());

    // TODO 2026-05-05 (Will Free): handle camera distortion

    has_camera_info = true;
}
