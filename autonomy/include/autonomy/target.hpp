#pragma once

#include <memory>
#include <utility>
#include <ros2_fmt_logger/logger.hpp>
#include <rclcpp/node.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include "autonomy/autonomy_interface.hpp"

namespace autonomy {
    enum class TargetState {
        UNCONFIGURED,
        INACTIVE,
        ACTIVE,
        COMPLETED,
        FAILED
    };

    class Target {
    public:
        RCLCPP_SMART_PTR_ALIASES_ONLY(Target)

        explicit Target(
            AutonomyMissionManagerInterface& node,
            const std::string& id
        )
            : logger(node.get_logger().get_child(id)), node(node), id(id) {}

        virtual ~Target() = default;

        Target(const Target& other) = delete;
        Target(Target&& other) noexcept = delete;
        Target& operator=(const Target& other) = delete;
        Target& operator=(Target&& other) noexcept = delete;

        virtual void setup() {
            if (this->state != TargetState::INACTIVE)
                throw std::runtime_error("Target is not in unconfigured state, cannot set up");

            this->state = TargetState::INACTIVE;
        }

        virtual rclcpp_async::Task<> start() {
            if (this->state != TargetState::INACTIVE)
                throw std::runtime_error("Target is not in inactive state, cannot start");

            this->state = TargetState::ACTIVE;

            co_return;
        }

        [[nodiscard]] virtual geographic_msgs::msg::GeoPoint findNearestPoint(const geographic_msgs::msg::GeoPoint& current_pose) const = 0;

        [[nodiscard]] TargetState getState() const {
            return this->state;
        }

    protected:
        const ros2_fmt_logger::Logger logger;
        AutonomyMissionManagerInterface& node;
        // TODO 2026-03-26 (Will Free): tbh, this state system is like *kinda* bad. I'll maybe redo it with something better later
        TargetState state = TargetState::UNCONFIGURED;
        const std::string id;
    };
}
