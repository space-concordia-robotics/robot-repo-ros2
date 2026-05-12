#pragma once

#include <memory>
#include <vector>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <ros2_fmt_logger/logger.hpp>
#include <can_util/can_controller.hpp>

#include "spark/spark_max.hpp"

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

            // TODO 2026-03-01 (Will Free): pretty sure this works
            std::string position_interface_name = fmt::format("{}/{}", name, hardware_interface::HW_IF_POSITION);
            std::string velocity_interface_name = fmt::format("{}/{}", name, hardware_interface::HW_IF_VELOCITY);

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

        CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::shared_ptr<ros2_fmt_logger::Logger> logger;
        can_util::CANController::SharedPtr can_controller;
        std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater;
        rclcpp::TimerBase::SharedPtr heartbeat_timer;
        double multiplier;
        std::vector<WheelDescription::SharedPtr> wheels;

        void heartbeat() const;

        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, const WheelDescription::ConstSharedPtr& wheel) const;
    };
}

namespace diagnostic_updater {
    template <>
    inline void DiagnosticStatusWrapper::add<float>(const std::string& key, const float& f) {
        diagnostic_msgs::msg::KeyValue ds;
        ds.key = key;
        ds.value = fmt::format("{:f}", f);

        values.push_back(ds);
    }

    template <>
    inline void DiagnosticStatusWrapper::add<double>(const std::string& key, const double& d) {
        diagnostic_msgs::msg::KeyValue ds;
        ds.key = key;
        ds.value = fmt::format("{:f}", d);

        values.push_back(ds);
    }

    template <>
    inline void DiagnosticStatusWrapper::add<uint16_t>(const std::string& key, const uint16_t& d) {
        diagnostic_msgs::msg::KeyValue ds;
        ds.key = key;
        ds.value = fmt::format("{:d}", d);

        values.push_back(ds);
    }
}
