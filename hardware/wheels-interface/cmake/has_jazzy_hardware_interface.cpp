#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

class RoverSystemWheelsHardware : public hardware_interface::SystemInterface {
public:
    RoverSystemWheelsHardware() = default;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& info) override {
        const auto logger = this->get_logger();
        const auto clock = this->get_clock();
        const auto node = this->get_node();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override = 0;

    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override =0;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override = 0;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override = 0;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override = 0;

    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override = 0;
};
