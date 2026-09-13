#pragma once

#include <GeographicLib/Geodesic.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace autonomy::util {
    /**
     * Computes the distance between two points using the haversine formula
     * @param first The first point
     * @param second The second point
     * @return The distance in meters
     */
    constexpr double distance(const GeoPoint& first, const GeoPoint& second) {
        GeographicLib::Math::real distance;

        GeographicLib::Geodesic::WGS84().Inverse(first.latitude, first.longitude, second.latitude, second.longitude, distance);

        return distance;
    }

    /**
     * Generates a list of points that form a spiral.
     *
     * The code in this function should closely mirror the contents of scripts/generate_spiral.py,
     * as that file is used to prototype & visualize the spiral.
     *
     * @param frame_id The id of the frame to use as the origin
     * @param stamp The timestamp to use for the points
     * @param radius The outer radius of the spiral
     * @param step The distance between points in the spiral
     * @param radius_factor The growth factor for the spiral
     * @return The points in the generated spiral
     */
    inline std::vector<geometry_msgs::msg::PoseStamped> generateSpiral(
        const std::string& frame_id,
        const rclcpp::Time& stamp,
        const double radius,
        const double step,
        const double radius_factor
    ) {
        std_msgs::msg::Header header;
        header.frame_id = frame_id;
        header.stamp = stamp;
        std::vector<geometry_msgs::msg::PoseStamped> poses;

        auto push_pose = [&](const double x, const double y, const double yaw) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            tf2::Quaternion quat;
            quat.setRPY(0, 0, yaw);
            pose.pose.orientation = tf2::toMsg<tf2::Quaternion, geometry_msgs::msg::Quaternion>(quat);

            poses.push_back(pose);
        };

        double theta = 0.0;
        double r = 0.75; // idk why, but this seems to be a good starting point
        while (r < radius) {
            const double dtheta = step / std::sqrt(r * r);
            theta += dtheta;
            r = radius_factor * theta;
            const auto pose_r = std::min(r, radius);
            const auto x = pose_r * std::cos(theta);
            const auto y = pose_r * std::sin(theta);
            push_pose(x, y, theta + std::numbers::pi / 2.0);
        }

        // do another half turn at the max radius, to ensure everything is properly covered.
        const double limit = theta + 1.0 * std::numbers::pi;
        while (theta < limit) {
            const double dtheta = step / std::sqrt(radius * radius);
            theta += dtheta;
            const auto pose_r = std::min(r, radius);
            const auto x = pose_r * std::cos(theta);
            const auto y = pose_r * std::sin(theta);
            push_pose(x, y, theta + std::numbers::pi / 2.0);
        }

        return poses;
    }
}
