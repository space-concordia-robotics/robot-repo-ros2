#pragma once

#include "autonomy/aliases.hpp"
#include "autonomy/target.hpp"

namespace autonomy {
    class ObjectTarget : public Target {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(ObjectTarget)

        ObjectTarget(
            AutonomyMissionManagerInterface& node,
            const std::string& id,
            const TargetConfig::ObjectConfig& object_config,
            const GeoPoint& center,
            const double radius,
            const std::string& label
        ) : Target(node, id), object_config(object_config), center(center), radius(radius), label(label) {}

        ~ObjectTarget() override = default;

        void setup() override;
        rclcpp_async::Task<> start() override;

        // TODO 2026-04-25 (Will Free): return the nearest point on the outer part of the spiral
        [[nodiscard]] geographic_msgs::msg::GeoPoint findNearestPoint(const geographic_msgs::msg::GeoPoint& /*current_pose*/) const override {
            return center;
        }

    private:
        const TargetConfig::ObjectConfig& object_config;
        GeoPoint rover_position;
        const GeoPoint center;
        const double radius;
        const std::string label;
        std::optional<ImageDetectionArray> aruco_detection;
        /**
         * whether or not the aruco tag has been detected
         */
        bool ar_detected = false;

        Subscription<NavSatFix>::SharedPtr navsat_topic;
        Subscription<ImageDetectionArray>::SharedPtr detection_subscription;
        ActionClient<NavigateThroughPoses>::SharedPtr nav_client;
        ActionClient<FollowGPSWaypoints>::SharedPtr gps_client;

        rclcpp_async::Task<std::optional<WrappedResult<FollowGPSWaypoints>>> navigateToCenter() const;
        rclcpp_async::Task<std::optional<WrappedResult<NavigateThroughPoses>>> tryFindObject() const;
        rclcpp_async::Task<std::optional<WrappedResult<NavigateThroughPoses>>> navigateToObject();
    };
}
