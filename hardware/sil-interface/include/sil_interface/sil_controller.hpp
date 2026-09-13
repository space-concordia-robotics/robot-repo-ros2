#pragma once

#include <memory>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <ros2_fmt_logger/logger.hpp>

#include "rover_msgs/srv/set_sil_status.hpp"

namespace sil_interface {
    struct SILStatus {
        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 0;
        uint8_t brightness = 0;
    };

    class SILController : public controller_interface::ControllerInterface {
    public:
        SILController();

        controller_interface::CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::shared_ptr<ros2_fmt_logger::Logger> logger;

        rclcpp::Service<rover_msgs::srv::SetSILStatus>::SharedPtr sil_status_service;

        realtime_tools::RealtimeBuffer<std::shared_ptr<SILStatus>> status_ref;

        void handleSetSILStatus(
            const rover_msgs::srv::SetSILStatus::Request::SharedPtr& request,
            const rover_msgs::srv::SetSILStatus::Response::SharedPtr& response
        );
    };
}
