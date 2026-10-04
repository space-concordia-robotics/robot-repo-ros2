#pragma once

#include "spark_base.hpp"

namespace wheels_interface {
    class SparkMax : public SparkBase {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(SparkMax);

        explicit SparkMax(rclcpp::Logger& logger, can_util::CANController& can_controller, uint8_t deviceId);

        ~SparkMax() override = default;

        SparkMax(const SparkMax& other) = delete;
        SparkMax(SparkMax&& other) noexcept = delete;
        SparkMax& operator=(const SparkMax& other) = delete;
        SparkMax& operator=(SparkMax&& other) noexcept = delete;
    };
}
