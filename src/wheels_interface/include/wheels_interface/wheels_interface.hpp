#pragma once

#include <memory>
#include <vector>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <wheels_interface/can_controller.hpp>

#include "spark/spark_max.hpp"

// #define HARDWARE_INTERFACE_IS_JAZZY=0

namespace wheels_interface {
    using CallbackReturn = hardware_interface::CallbackReturn;
    using HardwareInfo = hardware_interface::HardwareInfo;
    using StateInterface = hardware_interface::StateInterface;
    using CommandInterface = hardware_interface::CommandInterface;
    using return_type = hardware_interface::return_type;

    class RoverSystemWheelsHardware : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(RoverSystemWheelsHardware);

        // TODO 2026-02-26 (Will Free): Finish flushing this out
        struct WheelDescription {
            RCLCPP_SMART_PTR_DEFINITIONS(WheelDescription);

            SparkMax::SharedPtr motor;
            std::string name;
            double radius;

#if HARDWARE_INTERFACE_IS_JAZZY
            // TODO 2026-03-01 (Will Free): pretty sure this works
            std::string position_interface_name = fmt::format("{}/{}", name, hardware_interface::HW_IF_POSITION);
            std::string velocity_interface_name = fmt::format("{}/{}", name, hardware_interface::HW_IF_VELOCITY);
#else
            double velocity_command;
            double position_state;
            double velocity_state;
#endif

            explicit WheelDescription(
                const SparkMax::SharedPtr& motor,
                const std::string& name,
                const double radius
            ) : motor(motor), name(name), radius(radius) {}

            double getCircumference() const {
                return std::numbers::pi * 2 * radius;
            }
        };


        RoverSystemWheelsHardware() = default;

#if HARDWARE_INTERFACE_IS_JAZZY
        CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
#else
        // we are currently using on_init(const HardwareInfo& info),
        // as on_init(const HardwareComponentInterfaceParams & params) was only introduced in Jazzy
        [[deprecated]] CallbackReturn on_init(const HardwareInfo& info) override;

        // we are currently using export_state_interfaces & export_command_interfaces() as they were the only ones present in Humble
        // when upgrading to Jazzy, replace then with on_*

        [[deprecated]] std::vector<StateInterface> export_state_interfaces() override;

        [[deprecated]] std::vector<CommandInterface> export_command_interfaces() override;
#endif

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        // TODO 2026-02-14 (Will Free): when we upgrade to Jazzy, this is no longer necessary.

#if !HARDWARE_INTERFACE_IS_JAZZY
        /**
         * \return clock of the SystemInterface.
         */
        // ReSharper disable CppHidingFunction
        rclcpp::Clock::SharedPtr get_clock() const {
            // ReSharper restore CppHidingFunction
            return clock;
        }
#endif

    private:
        CANController::SharedPtr can_controller;
        double multiplier;

#if !HARDWARE_INTERFACE_IS_JAZZY
        std::shared_ptr<rclcpp::Clock> clock;
#endif
        std::shared_ptr<ros2_fmt_logger::Logger> logger;

        std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater;

        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, const WheelDescription::ConstSharedPtr& wheel);


#if HARDWARE_INTERFACE_IS_JAZZY
        rclcpp::TimerBase::SharedPtr heartbeat_timer;
#else
        bool heartbeat_running;
        std::thread heartbeat_thread;
#endif

        void heartbeat() const;

        std::vector<WheelDescription::SharedPtr> wheels;
    };
}

// namespace diagnostic_updater {
//     template <>
//     inline void DiagnosticStatusWrapper::add<float>(const std::string& key, const float& b) {
//         diagnostic_msgs::msg::KeyValue ds;
//         ds.key = key;
//         ds.value = b ? "True" : "False";
//
//         values.push_back(ds);
//     }
// }
