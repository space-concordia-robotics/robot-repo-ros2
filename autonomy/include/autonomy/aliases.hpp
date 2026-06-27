#pragma once

#include <geographic_msgs/msg/geo_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_gps_waypoints.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros_aruco_opencv_msgs/msg/aruco_detections.hpp>
#include <rover_msgs/msg/image_detection_array.hpp>
#include <rover_msgs/srv/set_sil_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace autonomy {
    using Node = rclcpp::Node;

    using GeoPoint = geographic_msgs::msg::GeoPoint;
    using ArucoDetections = ros_aruco_opencv_msgs::msg::ArucoDetections;
    using ArucoMarker = ros_aruco_opencv_msgs::msg::ArucoMarker;
    using Pose = geometry_msgs::msg::Pose;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    using GeoPose = geographic_msgs::msg::GeoPose;
    using NavSatFix = sensor_msgs::msg::NavSatFix;
    using ImageDetectionArray = rover_msgs::msg::ImageDetectionArray;

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using FollowGPSWaypoints = nav2_msgs::action::FollowGPSWaypoints;
    using FollowPath = nav2_msgs::action::FollowPath;
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
    using SetSILStatus = rover_msgs::srv::SetSILStatus;

    using Path = nav_msgs::msg::Path;
    template <typename T>
    using Subscription = rclcpp::Subscription<T>;
    template <typename T>
    using Publisher = rclcpp::Publisher<T>;
    template <typename T>
    using Client = rclcpp::Client<T>;
    template <typename T>
    using ActionClient = rclcpp_action::Client<T>;
    template <typename T>
    using ClientGoalHandle = rclcpp_action::ClientGoalHandle<T>;
    template <typename T>
    using WrappedResult = rclcpp_action::ClientGoalHandle<T>::WrappedResult;
}
