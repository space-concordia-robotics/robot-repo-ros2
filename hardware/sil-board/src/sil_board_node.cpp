#include <cstring>
#include <linux/can.h>
#include <rclcpp/rclcpp.hpp>
#include "can-utils/can_interface.hpp"
#include "sil_board/msg/led_command.hpp"

// TODO: migrate to buildAddress::buildCANID once SIL device type / instruction
// are defined in prefixes.hpp.  Until then, use the raw ID from the working
// cansend command: cansend can0 0009001E#FFFFFFFF0000
static constexpr uint32_t SIL_DEFAULT_CAN_ID = 0x0009001Eu;
static constexpr uint8_t  SIL_DLC = 6;

class SilBoardNode : public rclcpp::Node {
public:
    explicit SilBoardNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("sil_board_node", options)
    {
        const auto can_path = this->declare_parameter<std::string>("can_interface", "can0");
        raw_can_id_ = static_cast<uint32_t>(
            this->declare_parameter<int>("sil_can_id", static_cast<int>(SIL_DEFAULT_CAN_ID)));

        can_controller_ = std::make_shared<can_util::CANController>(can_path, this->get_logger());
        if (!can_controller_->configureCan()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to configure CAN on %s", can_path.c_str());
            throw std::runtime_error("CAN configure failed");
        }

        sub_ = this->create_subscription<sil_board::msg::LedCommand>(
            "/sil_board/rgb", rclcpp::SystemDefaultsQoS(),
            [this](const sil_board::msg::LedCommand::SharedPtr msg) { onLedCommand(msg); });

        RCLCPP_INFO(this->get_logger(),
                    "sil_board_node ready — CAN %s, ID 0x%08X", can_path.c_str(), raw_can_id_);
    }

private:
    void onLedCommand(const sil_board::msg::LedCommand::SharedPtr& msg)
    {
        struct can_frame frame {};
        frame.can_id  = raw_can_id_ | CAN_EFF_FLAG;
        frame.can_dlc = SIL_DLC;

        // Byte order: R, G, B, brightness, pad, pad  (matches cansend payload)
        frame.data[0] = msg->r;
        frame.data[1] = msg->g;
        frame.data[2] = msg->b;
        frame.data[3] = msg->brightness;
        frame.data[4] = 0;
        frame.data[5] = 0;

        if (!can_controller_->sendBlockingFrame(frame)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Failed to send LED CAN frame");
        }
    }

    uint32_t raw_can_id_{SIL_DEFAULT_CAN_ID};
    std::shared_ptr<can_util::CANController> can_controller_;
    rclcpp::Subscription<sil_board::msg::LedCommand>::SharedPtr sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SilBoardNode>());
    rclcpp::shutdown();
    return 0;
}
