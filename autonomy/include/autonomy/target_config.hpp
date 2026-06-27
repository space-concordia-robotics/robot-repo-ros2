#pragma once
#include <rclcpp/duration.hpp>

struct TargetConfig {
    struct GPSConfig {
        double acceptance_radius;
        rclcpp::Duration timeout = rclcpp::Duration(0, 0);
        long max_retries;
    };

    struct ARConfig {
        std::string topic_name;
        double acceptance_radius;
        rclcpp::Duration timeout = rclcpp::Duration(0, 0);
        long max_retries;
        double tag_size;
        double confidence;
        double spiral_step;
        double spiral_radius_factor;
    };

    struct ObjectConfig {
        std::string topic_name;
        double acceptance_radius;
        rclcpp::Duration timeout = rclcpp::Duration(0, 0);
        long max_retries;
        double confidence;
        double spiral_step;
        double spiral_radius_factor;
    };

    GPSConfig gps;
    ARConfig ar;
    ObjectConfig object;
};
