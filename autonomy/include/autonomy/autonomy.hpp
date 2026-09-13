#pragma once

#include <ros2_fmt_logger/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_async/rclcpp_async.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "autonomy/autonomy_interface.hpp"
#include "autonomy/target.hpp"

namespace autonomy {
    class AutonomyMissionManager : public AutonomyMissionManagerInterface {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(AutonomyMissionManager)

        AutonomyMissionManager();
        ~AutonomyMissionManager() override = default;

        AutonomyMissionManager(const AutonomyMissionManager& other) = delete;
        AutonomyMissionManager(AutonomyMissionManager&& other) noexcept = delete;
        AutonomyMissionManager& operator=(const AutonomyMissionManager& other) = delete;
        AutonomyMissionManager& operator=(AutonomyMissionManager&& other) noexcept = delete;

        //! parses the config file
        void parseConfig();

        //! parses the targets from the config file
        std::vector<Target::SharedPtr> parseTargets();

        //! parses a specific target from the config file
        Target::SharedPtr parseTarget(const std::string& prefix, const std::string& target_id);

        //! parses a point from the config file
        GeoPoint parseGeoPoint(const std::string& prefix);

        rclcpp_async::Task<> start();

        Target::SharedPtr nearestTarget() const;

        bool allTargetsDone() const {
            return std::ranges::all_of(targets, [](auto&& target) {
                return target->getState() != TargetState::COMPLETED && target->getState() != TargetState::FAILED;
            });
        }

        TargetConfig getTargetConfig() const override {
            return target_config;
        }

        std::shared_ptr<rclcpp_async::CoContext> ctx() const override {
            return this->co_ctx;
        }

        rclcpp_async::Task<> setSILColour(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) override;

    private:
        std::shared_ptr<rclcpp_async::CoContext> co_ctx;
        ros2_fmt_logger::Logger logger;
        std::vector<Target::SharedPtr> targets = std::vector<Target::SharedPtr>();
        rclcpp::Duration mission_duration = rclcpp::Duration(0, 0);
        TargetConfig target_config = {};

        rclcpp::Subscription<NavSatFix>::SharedPtr navsat_topic;
        Client<SetSILStatus>::SharedPtr sil_client;

        GeoPoint rover_position;
    };
}
