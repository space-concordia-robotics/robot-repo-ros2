#pragma once

#include "autonomy/aliases.hpp"
#include "autonomy/target.hpp"

namespace autonomy {
    using GeoPoint = geographic_msgs::msg::GeoPoint;
    using FollowGPSWaypoints = nav2_msgs::action::FollowGPSWaypoints;
    template <typename T>
    using WrappedResult = rclcpp_action::ClientGoalHandle<T>::WrappedResult;

    class GPSTarget : public Target {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(GPSTarget);

        GPSTarget(
            AutonomyMissionManagerInterface& node,
            const std::string& id,
            const TargetConfig::GPSConfig& gps_config,
            const GeoPoint& location
        ) : Target(node, id), gps_config(gps_config), location(location) {}

        ~GPSTarget() override = default;

        void setup() override;
        rclcpp_async::Task<> start() override;

        // TODO 2026-04-25 (Will Free): find nearest point using gps acceptance radius
        [[nodiscard]] geographic_msgs::msg::GeoPoint findNearestPoint(const geographic_msgs::msg::GeoPoint& /*current_pose*/) const override {
            return location;
        }

        // todo

    private:
        const TargetConfig::GPSConfig& gps_config;
        const GeoPoint location;
        GeoPoint rover_position;
        Subscription<NavSatFix>::SharedPtr navsat_topic;
        ActionClient<FollowGPSWaypoints>::SharedPtr gps_client;

        rclcpp_async::Task<std::optional<WrappedResult<FollowGPSWaypoints>>> navigateToTarget();
    };
}
