#pragma once

#include "can-utils/can_interface.hpp"

#include <memory>
#include <string>

#include <rclcpp/logger.hpp>

namespace can_util {

/// Create a CANController and call configureCan(). Returns nullptr on failure.
/// Catches exceptions so callers get a clean error path instead of an
/// uncaught std::runtime_error from node constructors.
std::shared_ptr<CANController> createConfiguredCanController(
  const std::string & interface_name, const rclcpp::Logger & logger);

/// Log common fixes when CAN setup fails (interface down, wrong name, etc.).
void logCanSetupRecoveryHints(const rclcpp::Logger & logger, const std::string & interface_name);

}  // namespace can_util
