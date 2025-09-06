// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef arm_hardware__R6BOT_HARDWARE_HPP_
#define arm_hardware__R6BOT_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include <termios.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace arm_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_velocities_;
    int SerialPort = -1;
    struct termios tty;
    int serial_fd_; // file descriptor for /dev/ttyUSB0

    std::string port_;

    // State machine variables for robust serial reading
    enum ReadState {
        WAITING_FOR_HEADER,
        READING_LENGTH,
        READING_PAYLOAD
    };
    ReadState read_state_;
    uint8_t read_payload_length_ = 0;
    uint8_t read_buffer_[256]; // Max buffer size
    size_t read_buffer_pos_ = 0;
};

}  // namespace arm_hardware

#endif  // arm_hardware__R6BOT_HARDWARE_HPP_