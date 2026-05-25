#pragma once

#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <ros_aruco_opencv_msgs/msg/aruco_detections.hpp>
#include <tf2_ros/buffer.hpp>

#include "foc2-gui/overlay.hpp"
#include "foc2-gui/util/imgui_util.hpp"


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

    explicit ArucoVideoOverlay(ImApplication& application)
        : UiOverlay(application),
          tf_buffer(application.tfBuffer()) {}

    void onInit() override;

    void onDraw(ImDrawList* draw_list, const ImRect& bounds) override;

    void onCameraInfo(const CameraInfo::SharedPtr& msg);

private:
    rclcpp::Subscription<ArucoDetections>::SharedPtr aruco_subscription;

    tf2_ros::Buffer& tf_buffer;

    std::mutex markers_mutex;
    std::vector<ProjectedMarker> markers;
    std::vector<RejectedMarker> rejected_markers;

    std::mutex camera_mutex;
    bool has_camera_info = false;
    int camera_width = 0;
    int camera_height = 0;
    std::string camera_frame;
    cv::Matx33d camera_k;

    void onDetections(const ArucoDetections::UniquePtr& msg);
};
