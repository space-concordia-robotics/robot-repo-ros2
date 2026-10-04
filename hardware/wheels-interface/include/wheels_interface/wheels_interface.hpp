#pragma once

#include <memory>
#include <vector>
#include <can_util/can_controller.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <ros2_fmt_logger/logger.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "spark/spark_max.hpp"

namespace wheels_interface {
    using hardware_interface::CallbackReturn;
    using hardware_interface::HardwareInfo;
    using hardware_interface::InterfaceInfo;
    using hardware_interface::StateInterface;
    using hardware_interface::CommandInterface;
    using hardware_interface::return_type;

    class RoverSystemWheelsHardware : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(RoverSystemWheelsHardware);

        struct StatusPeriods {
            std::chrono::milliseconds period0;
            std::chrono::milliseconds period1;
            std::chrono::milliseconds period2;
            std::chrono::milliseconds period3;
            std::chrono::milliseconds period4;
        };

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
                SparkMax::SharedPtr motor,
                std::string name,
                const double radius
            )
                : motor(std::move(motor)), name(std::move(name)), radius(radius) {}

            [[nodiscard]] double getCircumference() const {
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
        double multiplier = 0.0;
        std::vector<WheelDescription::SharedPtr> wheels;
        StatusPeriods status_periods = {};
        std::chrono::milliseconds heartbeat_period = {};

        void heartbeat() const;

        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, const WheelDescription::ConstSharedPtr& wheel) const;
    };
}

namespace diagnostic_updater {
    template <>
    inline void DiagnosticStatusWrapper::add<float>(const std::string& key, const float& val) {
        diagnostic_msgs::msg::KeyValue ds;
        ds.key = key;
        ds.value = fmt::format("{:f}", val);

        values.push_back(ds);
    }

    template <>
    inline void DiagnosticStatusWrapper::add<double>(const std::string& key, const double& val) {
        diagnostic_msgs::msg::KeyValue ds;
        ds.key = key;
        ds.value = fmt::format("{:f}", val);

        values.push_back(ds);
    }

    template <>
    inline void DiagnosticStatusWrapper::add<uint16_t>(const std::string& key, const uint16_t& val) {
        diagnostic_msgs::msg::KeyValue ds;
        ds.key = key;
        ds.value = fmt::format("{:d}", val);

        values.push_back(ds);
    }
}
