// can_safety_node.hpp
//
// Standalone joystick-driven safety node. Replaces the embedded force-stop /
// resume logic that used to live inside can_controller_node so the safety
// path stays alive even if the higher-level hardware interfaces fault.
//
// Behaviour:
//   * Listens on /joy and watches two configurable button indices for
//     wheel-side force stop / resume (rising-edge triggered).
//   * Emits sendForceStop / sendResume frames addressed to the wheel hub.
//   * Publishes a latched std_msgs/Bool on /can_safety/wheel_stopped so
//     other nodes / dashboards can observe the latched safety state.
//
// The arm-side stop logic in the legacy node was already commented out and
// has been left out here on purpose; add an analogous block if/when the arm
// safety story stabilises.

#pragma once

#include "can-utils/can_interface.hpp"
#include "can-utils/system_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

#include <memory>
#include <string>

namespace can_safety_node
{

class CanSafetyNode : public rclcpp::Node
{
public:
  explicit CanSafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr & msg);

  // Helper: returns true on the rising edge of a button press given the
  // previous state; updates `prev` in place.
  static bool risingEdge(bool current, bool & prev);

  std::string can_interface_name_{"can0"};
  // Joy button indices (configurable via params; defaults match the legacy
  // can_controller_node mapping).
  int wheel_force_stop_button_{6};
  int wheel_resume_button_{7};

  std::shared_ptr<can_util::CANController> can_;
  std::unique_ptr<SystemFrameBuilder>      frame_builder_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      wheel_stopped_pub_;

  bool prev_force_stop_pressed_{false};
  bool prev_resume_pressed_{false};
  bool wheel_stopped_{false};
};

}  // namespace can_safety_node
