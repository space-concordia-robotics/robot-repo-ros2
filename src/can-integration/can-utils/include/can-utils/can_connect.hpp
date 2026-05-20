#pragma once

#include "can-utils/can_interface.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/logger.hpp>

namespace can_util {

/// Create a CANController and call configureCan(). Returns nullptr on failure.
/// Catches exceptions so callers get a clean error path instead of an
/// uncaught std::runtime_error from node constructors.
std::shared_ptr<CANController> createConfiguredCanController(
  const std::string & interface_name, const rclcpp::Logger & logger);

/// Return (or create) a process-wide shared CANController for `interface_name`.
///
/// Multiple hardware plugins that share the same physical CAN bus (e.g.
/// arm_can_hardware and wheel_can_hardware both using "can0") must call this
/// instead of createConfiguredCanController so they bind a single SocketCAN
/// socket rather than opening two competing raw sockets on the same iface.
///
/// Thread-safe. First caller triggers configureCan(); subsequent callers
/// on the same interface_name get the cached instance. Returns nullptr on
/// failure (same semantics as createConfiguredCanController).
std::shared_ptr<CANController> getSharedCanController(
  const std::string & interface_name, const rclcpp::Logger & logger);

/// Log common fixes when CAN setup fails (interface down, wrong name, etc.).
void logCanSetupRecoveryHints(const rclcpp::Logger & logger, const std::string & interface_name);

}  // namespace can_util
