#pragma once

#include "spark_base.hpp"

namespace wheels_interface {
    // using namespace std;
    // using namespace rclcpp;

    class SparkMax : public SparkBase {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(SparkMax);

        explicit SparkMax(rclcpp::Logger& logger, CANController& can_controller, uint8_t deviceId);

        ~SparkMax() override = default;
    };
}
