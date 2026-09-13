#include "autonomy/targets/gps_target.hpp"

#include <rclcpp_action/create_client.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "autonomy/util.hpp"

namespace autonomy {
    void GPSTarget::setup() {
        Target::setup();

        navsat_topic = node.create_subscription<NavSatFix>("/gps/fix", 10, [this](const NavSatFix::UniquePtr& msg) {
            this->rover_position.latitude = msg->latitude;
            this->rover_position.longitude = msg->longitude;
            this->rover_position.altitude = msg->altitude;
        });

        gps_client = rclcpp_action::create_client<FollowGPSWaypoints>(
            node.get_node_base_interface(),
            node.get_node_graph_interface(),
            node.get_node_logging_interface(),
            node.get_node_waitables_interface(),
            "follow_gps_waypoints"
        );
    }

    rclcpp_async::Task<> GPSTarget::start() {
        Target::start();

        auto tries = 0u;

        while (true) {
            tries += 1;

            if (tries > this->gps_config.max_retries) {
                this->state = TargetState::FAILED;
                co_return;
            }

            const auto optional_result = co_await navigateToTarget();

            if (!optional_result.has_value())
                continue;

            if (const auto& result = *optional_result; result.code != rclcpp_action::ResultCode::SUCCEEDED)
                continue;

            if (const auto distance = util::distance(this->location, this->rover_position); distance <= this->gps_config.acceptance_radius) {
                break;
            }
        }

        logger.info("reached GPS target {}, triggering SIL", this->id);

        const auto ctx = node.ctx();

        using namespace std::chrono_literals;

        for (int i = 0; i < 8; ++i) {
            // TODO 2026-05-30 (Will Free): make these not hardcoded

            // flash green
            node.setSILColour(0, 255, 0, i % 2 == 0 ? 255 : 0);

            ctx->sleep(250ms);
        }

        co_return;
    }

    rclcpp_async::Task<std::optional<WrappedResult<FollowGPSWaypoints>>> GPSTarget::navigateToTarget() const {
        const auto ctx = node.ctx();

        auto goal_msg = FollowGPSWaypoints::Goal();
        geographic_msgs::msg::GeoPose pose;
        pose.position = this->location;

        goal_msg.gps_poses = {pose};


        const auto goal_result = co_await ctx->send_goal<FollowGPSWaypoints>(gps_client, goal_msg);
        if (!goal_result.ok()) {
            co_return std::nullopt;
        }

        const auto stream = *goal_result.value;

        while (true) {
            // we're just discarding the feedback
            if (auto feedback = co_await stream->next(); !feedback.has_value())
                break;
        }

        auto result = stream->result();

        co_return std::optional(result);
    }
}
