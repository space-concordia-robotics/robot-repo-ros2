#pragma once

#include "autonomy/aliases.hpp"
#include "autonomy/target.hpp"

namespace autonomy {
    class ARTarget : public Target {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(ARTarget)

        ARTarget(
            AutonomyMissionManagerInterface& node,
            const std::string& id,
            const TargetConfig::ARConfig& ar_config,
            const GeoPoint& center,
            const double radius,
            const int tag_id
        ) : Target(node, id), ar_config(ar_config), center(center), radius(radius), tag_id(tag_id) {}

        ~ARTarget() override = default;

        void setup() override;
        rclcpp_async::Task<> start() override;

        // TODO 2026-04-25 (Will Free): return the nearest point on the outer part of the spiral
        [[nodiscard]] geographic_msgs::msg::GeoPoint findNearestPoint(const geographic_msgs::msg::GeoPoint& /*current_pose*/) const override {
            return center;
        }

    private:
        const TargetConfig::ARConfig& ar_config;
        GeoPoint rover_position;
        const GeoPoint center;
        const double radius;
        const int tag_id;
        std::optional<ArucoDetection> aruco_detection;
        /**
         * whether or not the aruco tag has been detected
         */
        bool ar_detected = false;

        Subscription<NavSatFix>::SharedPtr navsat_topic;
        Subscription<ArucoDetection>::SharedPtr aruco_subscription;
        ActionClient<NavigateThroughPoses>::SharedPtr nav_client;
        ActionClient<FollowGPSWaypoints>::SharedPtr gps_client;

        rclcpp_async::Task<std::optional<WrappedResult<FollowGPSWaypoints>>> navigateToCenter();
        rclcpp_async::Task<std::optional<WrappedResult<NavigateThroughPoses>>> tryFindArucoPost();
        rclcpp_async::Task<std::optional<WrappedResult<NavigateThroughPoses>>> navigateToTag();
    };
}
