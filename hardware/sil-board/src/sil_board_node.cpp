#include <can_util/can_util.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sil_board/msg/led_command.hpp"

// TODO: migrate to buildAddress::buildCANID once SIL device type / instruction
// are defined in prefixes.hpp.  Until then, use the raw ID from the working
// cansend command: cansend can0 0009001E#FFFFFFFF0000
static constexpr uint32_t SIL_DEFAULT_CAN_ID = 0x0009001Eu;
static constexpr uint8_t SIL_DLC = 6;

class SilBoardNode : public rclcpp::Node {
public:
    explicit SilBoardNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("sil_board_node", options), logger(this->get_logger()) {
        const auto can_path = this->declare_parameter<std::string>("can_interface", "can0");
        raw_can_id = static_cast<uint32_t>(this->declare_parameter<int>("sil_can_id", SIL_DEFAULT_CAN_ID));

        can_controller = std::make_shared<can_util::CANController>(can_path, this->get_logger());
        if (!can_controller->initialize()) {
            logger.fatal("Failed to initialize canbus");
            throw std::runtime_error("CAN failed to initialize");
        }

        sil_subscription = this->create_subscription<sil_board::msg::LedCommand>(
            "/sil_board/rgb",
            rclcpp::SystemDefaultsQoS(),
            [this](const sil_board::msg::LedCommand::UniquePtr& msg) {
                onLedCommand(msg);
            }
        );

        logger.info("sil_board_node ready — CAN {}, ID {:#08X}", can_path, raw_can_id);
    }

private:
    void onLedCommand(const sil_board::msg::LedCommand::UniquePtr& msg) const {
        if (const auto data = createSILCanData(msg->r, msg->g, msg->b, msg->brightness); !can_controller->sendBlockingFrame(raw_can_id, data)) {
            using namespace std::chrono_literals;
            logger.warn_throttle(1s, "Failed to send LED CAN frame");
        }
    }

    /**
     * Create the CAN data for SIL
     * @param r red channel
     * @param g green channel
     * @param b blue channel
     * @param brightness brightness
     * @param blinking if the LED is blinking
     * @param blink_period blink period (in hundredths of a second)
     * @return the CAN data
     */
    // TODO 2026-05-12 (Will Free): convert blinking to a chrono
    static constexpr std::array<uint8_t, 6> createSILCanData(
        const uint8_t r,
        const uint8_t g,
        const uint8_t b,
        const uint8_t brightness,
        const bool blinking = false,
        const uint8_t blink_period = 0
    ) {
        return {
            r,
            g,
            b,
            brightness,
            static_cast<uint8_t>(blinking),
            blink_period
        };
    }

    ros2_fmt_logger::Logger logger;
    uint32_t raw_can_id = SIL_DEFAULT_CAN_ID;
    std::shared_ptr<can_util::CANController> can_controller;
    rclcpp::Subscription<sil_board::msg::LedCommand>::SharedPtr sil_subscription;
};

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SilBoardNode>());
    rclcpp::shutdown();
    return 0;
}
