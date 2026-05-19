#include "can-utils/can_connect.hpp"

#include <exception>
#include <utility>

#include <rclcpp/logging.hpp>

namespace can_util {

void logCanSetupRecoveryHints(const rclcpp::Logger & logger, const std::string & interface_name)
{
  RCLCPP_ERROR(
    logger,
    "CAN recovery checklist for interface '%s':\n"
    "  1. List interfaces:  ip link show\n"
    "  2. Bring interface up (example):  sudo ip link set %s up type can bitrate 500000\n"
    "  3. Confirm modules loaded:  lsmod | grep can\n"
    "  4. Ensure no other process owns the bus (only one raw socket binder per interface)",
    interface_name.c_str(), interface_name.c_str());
}

std::shared_ptr<CANController> createConfiguredCanController(
  const std::string & interface_name, const rclcpp::Logger & logger)
{
  try {
    auto can = std::make_shared<CANController>(interface_name, logger);
    if (!can->configureCan()) {
      RCLCPP_FATAL(
        logger, "Failed to configure CAN on interface '%s' (see can_controller logs above)",
        interface_name.c_str());
      logCanSetupRecoveryHints(logger, interface_name);
      can.reset();
      return nullptr;
    }
    return can;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      logger, "Exception while configuring CAN on '%s': %s", interface_name.c_str(), e.what());
    logCanSetupRecoveryHints(logger, interface_name);
    return nullptr;
  } catch (...) {
    RCLCPP_FATAL(
      logger, "Unknown exception while configuring CAN on '%s'", interface_name.c_str());
    logCanSetupRecoveryHints(logger, interface_name);
    return nullptr;
  }
}

}  // namespace can_util
