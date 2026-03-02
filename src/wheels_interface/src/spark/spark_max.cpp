#include "wheels_interface/spark/spark_max.hpp"

namespace wheels_interface {
    SparkMax::SparkMax(rclcpp::Logger& logger, CANController& can_controller, const uint8_t deviceId) : SparkBase(logger, can_controller, deviceId) {}
}
