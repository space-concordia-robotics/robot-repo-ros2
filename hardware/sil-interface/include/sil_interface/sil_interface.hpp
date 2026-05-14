#pragma once

#include <memory>
#include <can_util/can_controller.hpp>
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

namespace sil_interface {
    using CallbackReturn = hardware_interface::CallbackReturn;
    using HardwareInfo = hardware_interface::HardwareInfo;
    using InterfaceInfo = hardware_interface::InterfaceInfo;
    using StateInterface = hardware_interface::StateInterface;
    using CommandInterface = hardware_interface::CommandInterface;
    using return_type = hardware_interface::return_type;

    class SILSystemHardware : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(SILSystemHardware)

        SILSystemHardware();

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

        uint32_t device_id = 0;
    };
}
