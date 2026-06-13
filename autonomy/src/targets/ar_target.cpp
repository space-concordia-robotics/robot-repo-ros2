#include "autonomy/targets/ar_target.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

#include "autonomy/util.hpp"

namespace autonomy {
    void ARTarget::setup() {
        Target::setup();

        navsat_topic = node.create_subscription<NavSatFix>(
            "/gps/fix",
            10,
            [this](const NavSatFix::UniquePtr& msg) {
                rover_position.latitude = msg->latitude;
                rover_position.longitude = msg->longitude;
                rover_position.altitude = msg->altitude;
            }
        );

        aruco_subscription = node.create_subscription<ArucoDetections>(
            "/aruco",
            10,
            [this](const ArucoDetections::UniquePtr& msg) {
                const auto matched_tag = std::ranges::any_of(
                    msg->markers,
                    [&](const ArucoMarker& marker) {
                        return std::cmp_equal(marker.id, this->tag_id);
                    }
                );

                if (matched_tag) {
                    aruco_detection = *msg;
                    ar_detected = true;
                }
            }
        );

        gps_client = rclcpp_action::create_client<FollowGPSWaypoints>(
            node.get_node_base_interface(),
            node.get_node_graph_interface(),
            node.get_node_logging_interface(),
            node.get_node_waitables_interface(),
            "follow_gps_waypoints"
        );

        nav_client = rclcpp_action::create_client<NavigateThroughPoses>(
            node.get_node_base_interface(),
            node.get_node_graph_interface(),
            node.get_node_logging_interface(),
            node.get_node_waitables_interface(),
            "navigate_through_poses"
        );
    }

    rclcpp_async::Task<> ARTarget::start() {
        co_await Target::start();

        aruco_detection = std::nullopt;
        ar_detected = false;

        auto tries = 0u;
        constexpr static auto MAX_NAVIGATION_TRIES = 8u;

        while (true) {
            tries += 1;

            if (tries > MAX_NAVIGATION_TRIES) {
                this->state = TargetState::FAILED;
                co_return; // abandon goal if we cannot navigate to it
            }

            // TODO 2026-04-27 (Will Free): exit early if we detect the aruco tag before we reach the gps coordinates
            const auto optional_result = co_await navigateToCenter();

            if (!optional_result.has_value())
                continue;

            if (const auto& result = *optional_result; result.code != rclcpp_action::ResultCode::SUCCEEDED)
                continue;

            if (const auto distance = util::distance(this->center, this->rover_position); distance <= this->ar_config.acceptance_radius) {
                break;
            }
        }

        // reset the aruco detection in case it got detected while going to the gps coordinates
        aruco_detection = std::nullopt;
        ar_detected = false;

        tries = 0u;

        constexpr static auto MAX_SPIRAL_TRIES = 2u;

        while (true) {
            tries += 1;

            if (tries > MAX_SPIRAL_TRIES) {
                this->state = TargetState::FAILED;
                co_return; // abandon goal if we cannot find it after 2 spiral attempts{
            }

            const auto optional_result = co_await tryFindArucoPost();

            if (!optional_result.has_value())
                continue;

            if (const auto& result = *optional_result; result.code != rclcpp_action::ResultCode::SUCCEEDED)
                continue;

            // did not find aruco tag
            if (!aruco_detection)
                continue;

            if (const auto distance = util::distance(this->center, this->rover_position); distance <= this->ar_config.acceptance_radius) {
                break;
            }
        }

        // TODO 2026-04-27 (Will Free): retry logic for this

        if (const auto result = co_await navigateToTag(); !result) {
            logger.error("Failed to navigate to tag, bailing out.");
            co_return;
        }

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

    rclcpp_async::Task<std::optional<WrappedResult<FollowGPSWaypoints>>> ARTarget::navigateToCenter() const {
        const auto ctx = node.ctx();

        auto goal_msg = FollowGPSWaypoints::Goal();
        geographic_msgs::msg::GeoPose pose;
        pose.position = this->center;

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

    rclcpp_async::Task<std::optional<WrappedResult<NavigateThroughPoses>>> ARTarget::tryFindArucoPost() const {
        const auto ctx = node.ctx();

        auto goal_msg = NavigateThroughPoses::Goal();
        goal_msg.poses = util::generateSpiral(
            "base_link",
            this->node.get_clock()->now(),
            this->radius,
            this->ar_config.spiral_step,
            this->ar_config.spiral_radius_factor
        );

        const auto goal_result = co_await ctx->send_goal<NavigateThroughPoses>(nav_client, goal_msg);

        if (!goal_result.ok()) {
            co_return std::nullopt;
        }

        const auto stream = *goal_result.value;

        while (true) {
            if (aruco_detection) {
                co_await stream->cancel_goal();
                break;
            }

            // we're just discarding the feedback
            if (auto feedback = co_await stream->next(); !feedback.has_value())
                break;
        }

        auto result = stream->result();

        co_return std::optional(result);
    }

    rclcpp_async::Task<std::optional<WrappedResult<NavigateThroughPoses>>> ARTarget::navigateToTag() {
        const auto tmp = *aruco_detection;

        const auto ctx = node.ctx();

        rclcpp_async::TfBuffer tf(*ctx);

        const auto tag_iterator = std::ranges::find_if(
            aruco_detection->markers,
            [&](const ArucoMarker& marker) {
                return marker.id == static_cast<uint32_t>(this->tag_id);
            }
        );

        // we just assume it always exists, otherwise the property would never have been set.
        const auto index = tag_iterator - aruco_detection->markers.begin();
        const auto tag = *tag_iterator;
        const auto board = aruco_detection->boards[index];

        auto transform = co_await tf.lookup_transform("map", "base_link", rclcpp::Time(0));

        const geometry_msgs::msg::Pose tag_pose = tag.pose;
        geometry_msgs::msg::Pose tag_base;
        tf2::doTransform(tag_pose, tag_base, transform);

        const auto tx = tag_base.position.x;
        const auto ty = tag_base.position.y;
        const auto dist = std::hypot(tx, ty);

        // TODO 2026-04-27 (Will Free): do something to signal that we succeeded
        if (dist < this->ar_config.acceptance_radius)
            co_return std::nullopt;

        const auto ratio = (dist - this->ar_config.acceptance_radius) / dist;

        NavigateThroughPoses::Goal goal_msg;
        geometry_msgs::msg::PoseStamped target;

        target.header.frame_id = "base_link";
        target.header.stamp = node.get_clock()->now();

        target.pose.position.x = tx * ratio;
        target.pose.position.y = ty * ratio;
        target.pose.position.z = 0.0;

        // face tag in target
        tf2::Quaternion q;
        q.setRPY(0, 0, std::atan2(ty, tx));
        target.pose.orientation = tf2::toMsg<tf2::Quaternion, geometry_msgs::msg::Quaternion>(q);

        goal_msg.poses = std::vector{target};

        const auto goal_result = co_await ctx->send_goal<NavigateThroughPoses>(nav_client, goal_msg);

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
