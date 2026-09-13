#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp_async/rclcpp_async.hpp>

#include "autonomy/aliases.hpp"
#include "autonomy/target_config.hpp"

namespace autonomy {
    class AutonomyMissionManagerInterface : public Node, public std::enable_shared_from_this<AutonomyMissionManagerInterface> {
    public:
        RCLCPP_SMART_PTR_ALIASES_ONLY(AutonomyMissionManagerInterface)

        explicit AutonomyMissionManagerInterface(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : Node(node_name, options) {}

        explicit AutonomyMissionManagerInterface(const std::string& node_name, const std::string& namespace_,
                                                 const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : Node(node_name, namespace_, options) {}

        explicit AutonomyMissionManagerInterface(const Node& other, const std::string& sub_namespace)
            : Node(other, sub_namespace) {}

        virtual TargetConfig getTargetConfig() const = 0;

        virtual std::shared_ptr<rclcpp_async::CoContext> ctx() const = 0;

        virtual rclcpp_async::Task<> setSILColour(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) = 0;
    };
}
