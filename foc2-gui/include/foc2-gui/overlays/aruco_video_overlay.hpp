#pragma once

#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <ros_aruco_opencv_msgs/msg/aruco_detections.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "foc2-gui/overlay.hpp"
#include "foc2-gui/util/imgui-util.h"


class ArucoVideoOverlay : public UiOverlay {
    using ArucoDetections = ros_aruco_opencv_msgs::msg::ArucoDetections;
    using ArucoMarker = ros_aruco_opencv_msgs::msg::ArucoMarker;
    using RejectedMarker = ros_aruco_opencv_msgs::msg::RejectedMarker;
    using BoardPose = ros_aruco_opencv_msgs::msg::BoardPose;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

public:
    struct ProjectedAxes {
        cv::Point2f origin;
        cv::Point2f x_end;
        cv::Point2f y_end;
        cv::Point2f z_end;
    };

    struct ProjectedMarker {
        std::string marker_label;
        std::array<cv::Point2f, 4> corners;
        geometry_msgs::msg::Pose pose;
        ProjectedAxes axes;
    };

    explicit ArucoVideoOverlay(ImApplication& application, const std::string& camera_topic, const std::string& target_frame)
        : UiOverlay(application),
          camera_topic(camera_topic),
          target_frame(target_frame),
          tf_buffer(application.get_clock()),
          tf_listener(tf_buffer, &application) {}

    void onInit() override;

    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override;

private:
    std::string camera_topic;
    std::string target_frame;

    rclcpp::Subscription<ArucoDetections>::SharedPtr aruco_subscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    std::mutex markers_mutex;
    std::vector<ProjectedMarker> markers;
    std::vector<RejectedMarker> rejected_markers;

    std::mutex camera_mutex;
    bool has_camera_info = false;
    int camera_width = 0;
    int camera_height = 0;
    std::string camera_frame;
    cv::Matx33d camera_k;

    // static void drawArucoMarker(
    //     ImDrawList* draw_list,
    //     const ImRect& bounds,
    //     double scale_x,
    //     double scale_y,
    //     const std::array<geometry_msgs::msg::Point32, 4>& corners,
    //     int marker_id = -1,
    //     ImU32 marker_color = ImGui::ImColor(0, 255, 0, 255),
    //     ImU32 id_color = ImGui::ImColor(255, 255, 255, 255)
    // );
    //
    // static void drawFrameAxis(
    //     ImDrawList* draw_list,
    //     const ImRect& bounds,
    //     double scale_x,
    //     double scale_y,
    //     const std::array<geometry_msgs::msg::Point32, 4>& corners,
    //     int marker_id = -1,
    //     ImU32 marker_color = ImGui::ImColor(0, 255, 0, 255),
    //     ImU32 id_color = ImGui::ImColor(255, 255, 255, 255)
    // );

    void onDetections(const ArucoDetections::UniquePtr& msg);

    void onCameraInfo(const CameraInfo::UniquePtr& msg);
};
