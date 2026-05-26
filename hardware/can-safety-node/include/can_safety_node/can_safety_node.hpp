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

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include "can_util/can_util.hpp"

namespace can_safety_node {
    class CanSafetyNode : public rclcpp::Node {
    public:
        explicit CanSafetyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        ros2_fmt_logger::Logger logger;

        std::string can_interface_name = "can0";
        // Joy button indices (configurable via params).
        // Defaults are VKBButtonLayout::F1 (26) and F2 (27) — unassigned by joy_mux_controller_py,
        // so they do not conflict with arm joints 2/5 (which used the old defaults 6 and 7).
        // Verify actual indices on your hardware with: ros2 run joy_mux_controller_py joy_button_probe
        int wheel_force_stop_button = 26;
        int wheel_resume_button = 27;

        can_util::CANController::SharedPtr can_controller;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wheel_stopped_publisher;

        bool prev_force_stop_pressed = false;
        bool prev_resume_pressed = false;
        bool wheel_stopped = false;

        void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg);
        bool sendShutDownRequest(can_util::constants::DeviceType device_type, uint32_t device_id) const;
        bool sendRestartCommand(can_util::constants::DeviceType device_type, uint32_t device_id) const;

        // Helper: returns true on the rising edge of a button press given the
        // previous state; updates `prev` in place.
        static bool risingEdge(bool current, bool& prev);
    };
}
