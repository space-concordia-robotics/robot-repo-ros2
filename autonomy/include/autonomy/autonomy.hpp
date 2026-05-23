#pragma once

#include <geographic_msgs/msg/geo_point.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_async/rclcpp_async.hpp>
#include <ros2_fmt_logger/logger.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "autonomy/autonomy_interface.hpp"
#include "autonomy/target.hpp"

namespace autonomy {
    class AutonomyMissionManager : public AutonomyMissionManagerInterface {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(AutonomyMissionManager)

        AutonomyMissionManager();

        //! parses the config file
        void parseConfig();

        //! parses the targest from the config file
        std::vector<Target::SharedPtr> parseTargets();

        //! parses a specific target from the config file
        Target::SharedPtr parseTarget(const std::string& prefix, const std::string& target_id);

        //! parses a point from the config file
        GeoPoint parseGeoPoint(const std::string& prefix);

        void start();

        Target::SharedPtr nearestTarget() const;

        bool allTargetsDone() const {
            for (const auto& t : targets) {
                if (t->getState() != TargetState::COMPLETED && t->getState() != TargetState::FAILED)
                    return false;
            }
            return true;
        }

        TargetConfig getTargetConfig() const override {
            return target_config;
        }

        std::shared_ptr<rclcpp_async::CoContext> ctx() const override {
            return this->co_ctx;
        }

    private:
        std::shared_ptr<rclcpp_async::CoContext> co_ctx;
        ros2_fmt_logger::Logger logger;
        std::vector<Target::SharedPtr> targets = std::vector<Target::SharedPtr>();
        rclcpp::Duration mission_duration = rclcpp::Duration(0, 0);
        TargetConfig target_config = {};

        rclcpp::Subscription<NavSatFix>::SharedPtr navsat_topic;

        GeoPoint rover_position;
    };
}
